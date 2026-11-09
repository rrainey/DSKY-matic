/*
 * Saturn-IB ↔ CMC (yaAGC) bridge implementation.
 *
 * Holds the agc_t*, uplink queue, downlink buffer, IMU/PIPA last-sent
 * bookkeeping, and the extern "C" yaAGC callbacks.  All physics lives
 * on the SaturnIBStack base class; this translation unit is the only
 * place that includes <agc_engine.h> and references yaAGC symbols.
 */
#include "SaturnIBAgcBridge.h"
#include "SaturnIBStack.h"
#include "ChannelQueue.h"
#include "UplinkMessage.h"
#include "msvc_str.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <limits>
#include <iostream>

#include <agc_engine.h>

using namespace std;
using namespace sim;

/** One AGC CPU cycle is 11.7 microseconds. */
#define AGC_CPU_CYCLE_SEC   (11.71875 / 1000000.0)

/* Pack OUT22 enum used by the pad-frame ECI snapshot below. */
#define OUT22_lat_deg           0
#define OUT22_lon_deg           1
#define OUT22_h_m               2
#define OUT22_Xe_x_m            3
#define OUT22_Xe_y_m            4
#define OUT22_Xe_z_m            5
#define OUT22_Ve_x_mps          6
#define OUT22_Ve_y_mps          7
#define OUT22_Ve_z_mps          8

#define OUT11_lat_deg           0
#define OUT11_lon_deg           1
#define OUT11_h_m               2
#define OUT11_Xe_x_m            3
#define OUT11_Xe_y_m            4
#define OUT11_Xe_z_m            5
#define OUT11_Ve_x_mps          6
#define OUT11_Ve_y_mps          7
#define OUT11_Ve_z_mps          8
#define OUT11_IMU_theta_x_deg   9
#define OUT11_IMU_theta_y_deg   10
#define OUT11_IMU_theta_z_deg   11
#define OUT11_alpha_deg         12
#define OUT11_Vb_mps            13

ChannelIOList g_AGCChannelOutputQueue;
ChannelIOList g_AGCChannelInputQueue;

SaturnIBAgcBridge * g_pSaturnIBAgcBridge = nullptr;

static bool bSentFirstUplink = false;
static bool bSentStateVector = false;

static void SwriteDownlinkWriter();

// =====================================================================
// Ctor / dtor
// =====================================================================

SaturnIBAgcBridge::SaturnIBAgcBridge(const BatchConfig & config)
    : SaturnIBStack(config),
      m_pAGC(nullptr),
      m_pad_load_file(config.pad_load_file.empty()
                        ? std::string("TM-X-62831_PAD_LOAD.txt")
                        : config.pad_load_file),
      m_pad_load_start_sec(config.pad_load_start_sec < 0
                            ? 15.0
                            : config.pad_load_start_sec),
      m_pCurrentUplinkMessage(nullptr),
      m_dNextUplinkTime_sec(-1.0),
      m_vLastCM_IMU_PIPA_Pulses(0,0,0),
      m_vPipaPulsesInjectedCum(0,0,0),
      m_bLastSeenLiftoff(false),
      m_bNoAttLampOn(true),
      m_bIssTurnOnDelayComplete(false),
      m_dNoAttOffElapsed_sec(-1.0),
      m_bFirstResetReceived(false),
      m_dAGCResidualDeltaT_sec(0.0),
      m_bImuDebug(false),
      m_pImuDebugFile(nullptr),
      m_nImuDebugLastSec(-1)
{
    g_pSaturnIBAgcBridge = this;
    SetAgcBridge(this);

    SwritePtr = SwriteDownlinkWriter;

    m_pAGC = (agc_t *) malloc( sizeof( agc_t ) );
    memset( m_pAGC, 0, sizeof(agc_t) );
    int nResult = agc_engine_init( m_pAGC, NULL, NULL, 0 );
    (void)nResult;

    {
        const char * rom = config.rom_path.empty()
                             ? "Colossus249.bin"
                             : config.rom_path.c_str();
        agc_load_binfile( m_pAGC, rom );

        // Upstream yaAGC (2025+) loads downlink list specs from CWD TSV files
        // via dddConfigure(); without this, DecodeDigitalDownlink is a no-op
        // and the simulated downlink screen stays blank.  Expect
        // ddd-version-aliases.tsv and ddd-*-<software>.tsv next to the ROM.
        char soft[64];
        const char *base = strrchr(rom, '/');
        base = base ? base + 1 : rom;
        strncpy(soft, base, sizeof(soft) - 1);
        soft[sizeof(soft) - 1] = '\0';
        size_t n = strlen(soft);
        if (n > 4 && strcmp(soft + n - 4, ".bin") == 0)
            soft[n - 4] = '\0';
        if (soft[0] == '\0')
            strcpy(soft, "Colossus249");
        const int ddd = dddConfigure(soft, "");
        if (ddd < 0)
            std::fprintf(stderr,
                "AGC downlink: dddConfigure(%s) failed — "
                "place ddd-*-%s.tsv and ddd-version-aliases.tsv in CWD\n",
                soft, soft);
        else
            std::fprintf(stderr,
                "AGC downlink: configured for %s (CmOrLm=%d)\n", soft, ddd);
    }

    // Restart phase tables (Colossus -PHASE1..PHASE6 at 00752..00765).
    // GOPROG3 requires (-PHASEn) XOR PHASEn == -0.  Virgin zeros give
    // 0 XOR 0 == +0 → alarm 01107 (phase table failure) then DOFSTART.
    // Match MR.KLEAN idle: -PHASEn = -0 (077777), PHASEn = +0.
    for (int g = 0; g < 6; ++g) {
        const int addr = 00752 + 2 * g;
        const int bank = (addr >> 8) & 07;
        const int off  = addr & 0377;
        m_pAGC->Erasable[bank][off]     = 077777;
        m_pAGC->Erasable[bank][off + 1] = 000000;
    }
    {
        const int addr = 01011;  // MODREG
        m_pAGC->Erasable[(addr >> 8) & 07][addr & 0377] = 077777;
    }

    // IMU_DEBUG CSV logger — same contract as before.
    const char *dbg = std::getenv("IMU_DEBUG");
    m_bImuDebug = (dbg != nullptr && dbg[0] != '\0' && std::strcmp(dbg, "0") != 0);
    if (m_bImuDebug)
    {
        m_pImuDebugFile = std::fopen("imu_pipa_cdu.csv", "w");
        if (m_pImuDebugFile)
        {
            std::fprintf(m_pImuDebugFile,
                "t_sec,liftoff,imu_mode,no_att,pipa_inhibit,"
                "pipa_plant_x,pipa_plant_y,pipa_plant_z,"
                "pipa_injected_cum_x,pipa_injected_cum_y,pipa_injected_cum_z,"
                "pipa_residual_x,pipa_residual_y,pipa_residual_z,"
                "oga_deg,iga_deg,mga_deg,"
                "cdu_x_cnt,cdu_y_cnt,cdu_z_cnt,"
                "cdu_x_deg,cdu_y_deg,cdu_z_deg,"
                "q_eci_sm_0,q_eci_sm_1,q_eci_sm_2,q_eci_sm_3,"
                "out11_ve_x,out11_ve_y,out11_ve_z,out11_ve_mag,"
                "out11_h_m,out11_vb_mps,"
                "out22_ve_x,out22_ve_y,out22_ve_z,"
                "out22_xe_x,out22_xe_y,out22_xe_z,"
                "A_z_deg,"
                "sm_z_dot_down,sm_x_dot_az,sm_x_elev_deg,"
                "agc_rn_x,agc_rn_y,agc_rn_z,"
                "agc_vn_x,agc_vn_y,agc_vn_z,"
                "agc_piptime_sec,"
                "refsm_xx,refsm_xy,refsm_xz,"
                "refsm_yx,refsm_yy,refsm_yz,"
                "refsm_zx,refsm_zy,refsm_zz,"
                "rn_err_m,vn_err_mps,"
                "refsm_x_dot_smx,refsm_z_dot_smz\n");
            std::fflush(m_pImuDebugFile);
            std::fprintf(stderr,
                "  IMU_DEBUG: logging 1 Hz to imu_pipa_cdu.csv (CWD)\n");
        }
        else
        {
            std::fprintf(stderr,
                "  IMU_DEBUG: failed to open imu_pipa_cdu.csv for write\n");
            m_bImuDebug = false;
        }
    }

    strncpy_s( m_szDownlinkText, sizeof(m_szDownlinkText), "", 1 );

    // Mark IMU as "on" and "IMU Turn On request" on channel 030.
    unsigned short usValue;
    usValue = 0;
    usValue |= ReadIO( m_pAGC, 030 ) & ~(BIT14 | BIT9);
    WriteIO( m_pAGC, 030, usValue );

    usValue = ReadIO( m_pAGC, 031 ) & ~BIT15;
    WriteIO( m_pAGC, 031, usValue );
}

SaturnIBAgcBridge::~SaturnIBAgcBridge()
{
    if (m_pImuDebugFile) {
        std::fclose(m_pImuDebugFile);
        m_pImuDebugFile = nullptr;
    }
    if (m_pAGC) {
        free(m_pAGC);
        m_pAGC = nullptr;
    }
    if (g_pSaturnIBAgcBridge == this)
        g_pSaturnIBAgcBridge = nullptr;
    SetAgcBridge(nullptr);
}

// =====================================================================
// IAgcBridge overrides
// =====================================================================

void
SaturnIBAgcBridge::OnTimeStepBegin(double elapsed_sec,
                                   const sim::Vector3d &pad_pos_eci_m,
                                   const sim::Vector3d &pad_vel_eci_mps)
{
    // Pad-load uplink start.
    if ( bSentFirstUplink == false
         && m_pad_load_file != "-"
         && elapsed_sec >= m_pad_load_start_sec )
    {
        if ( StartUplink( m_pad_load_file ) )
        {
            std::fprintf(stderr,
                "  PAD LOAD STARTED at elapsed=%.3f sec (file=%s)\n",
                elapsed_sec, m_pad_load_file.c_str());
            LogInfo("PAD LOAD STARTED\n");
        }
        else
        {
            LogInfo("PAD LOAD file missing\n");
        }
        bSentFirstUplink = true;
    }

    // State-vector auto-uplink gate: NO ATT off + 45 s (so pilot can get
    // to P02 where P27 is legal).
    const bool noAttGateOk =
        !m_bNoAttLampOn
        && m_dNoAttOffElapsed_sec >= 0.0
        && elapsed_sec >= m_dNoAttOffElapsed_sec + SV_AFTER_NOATT_OFF_SEC;

    if ( bSentStateVector == false
         && bSentFirstUplink == true
         && m_pCurrentUplinkMessage == nullptr
         && noAttGateOk )
    {
        bSentStateVector = true;

        m_pCurrentUplinkMessage = new UplinkMessage();

        const Vector3d vECIPos_meters = pad_pos_eci_m;
        const Vector3d vECIVel_mps    = pad_vel_eci_mps;
        const double dTime_sec = elapsed_sec;

        const double r_mag = std::sqrt(
            vECIPos_meters.x * vECIPos_meters.x +
            vECIPos_meters.y * vECIPos_meters.y +
            vECIPos_meters.z * vECIPos_meters.z );
        const double v_mag = std::sqrt(
            vECIVel_mps.x * vECIVel_mps.x +
            vECIVel_mps.y * vECIVel_mps.y +
            vECIVel_mps.z * vECIVel_mps.z );

        if ( r_mag < 1.0e6 || r_mag > 1.0e7 )
        {
            std::fprintf(stderr,
                "  STATE VECTOR UPLINK SKIPPED: |R|=%.0f m out of Earth-pad range "
                "(expected ~6.4e6). Check OUT22.\n", r_mag);
            LogInfo("STATE VECTOR UPLINK SKIPPED (bad |R|)\n");
            delete m_pCurrentUplinkMessage;
            m_pCurrentUplinkMessage = nullptr;
        }
        else
        {
            std::fprintf(stderr,
                "  STATE VECTOR UPLINK STARTED at elapsed=%.3f sec\n"
                "    R_ECI=[%.1f, %.1f, %.1f] m   |R|=%.1f m\n"
                "    V_ECI=[%.4f, %.4f, %.4f] m/s |V|=%.4f m/s\n"
                "    T_clock=%.3f s (from CMC zero)  id=CSM/Earth  ECADR=01501\n"
                "    preamble: V37E 00E then V71E (need P00 or P02)\n"
                "    gate: NO ATT off at %.3f + %.0f s\n",
                dTime_sec,
                vECIPos_meters.x, vECIPos_meters.y, vECIPos_meters.z, r_mag,
                vECIVel_mps.x, vECIVel_mps.y, vECIVel_mps.z, v_mag,
                dTime_sec, m_dNoAttOffElapsed_sec, SV_AFTER_NOATT_OFF_SEC);
            LogInfo("STATE VECTOR UPLINK STARTED\n");

            m_pCurrentUplinkMessage->GenerateStateVectorUpdate(
                vECIPos_meters, vECIVel_mps, dTime_sec );
            int nKeycode = m_pCurrentUplinkMessage->GetFirstKeycode();
            WriteUplinkKeycode( nKeycode );
        }
    }
}

void
SaturnIBAgcBridge::UplinkTimeStep()
{
    if ( m_pCurrentUplinkMessage )
    {
        if ( GetSimClock()->GetElapsedTime() > m_dNextUplinkTime_sec )
        {
            int nKeycode = m_pCurrentUplinkMessage->GetNextKeycode();
            WriteUplinkKeycode( nKeycode );
        }
    }
}

void
SaturnIBAgcBridge::InjectGimbalCDU()
{
    AGCUpdateGimbalCDUCounters();
}

void
SaturnIBAgcBridge::StepAGC(double dt_sec)
{
    double dACGStep_sec = m_dAGCResidualDeltaT_sec + dt_sec;
    while ( dACGStep_sec >= AGC_CPU_CYCLE_SEC )
    {
        agc_engine( m_pAGC );
        dACGStep_sec -= AGC_CPU_CYCLE_SEC;
    }
    m_dAGCResidualDeltaT_sec = dACGStep_sec;
}

void
SaturnIBAgcBridge::InjectPIPA()
{
    // Edge-detect liftoff → push ch030 BIT5 (LIFTOFF discrete, 0 = TRUE).
    if ( m_bLiftoff != m_bLastSeenLiftoff )
    {
        unsigned short usValue = ReadIO( m_pAGC, 030 ) & ~(BIT5);
        if ( ! m_bLiftoff )
            usValue |= BIT5;
        WriteIO( m_pAGC, 030, usValue );
        m_bLastSeenLiftoff = m_bLiftoff;
    }

    AGCUpdatePIPACounters();
}

void
SaturnIBAgcBridge::OnTimeStepEnd()
{
    MaybeLogImuPipaCdu();

    // One-shot: first non-zero FAILREG → identify DSKY PROG alarm cause.
    if (m_pAGC) {
        static bool s_loggedFailreg = false;
        const unsigned fr0 = (unsigned)(m_pAGC->Erasable[0][0375] & 077777);
        const unsigned fr1 = (unsigned)(m_pAGC->Erasable[0][0376] & 077777);
        const unsigned fr2 = (unsigned)(m_pAGC->Erasable[0][0377] & 077777);
        if (!s_loggedFailreg && fr0 != 0) {
            s_loggedFailreg = true;
            std::fprintf(stderr,
                "  AGC PROGRAM ALARM at elapsed=%.3f sec: "
                "FAILREG=%05o %05o %05o  (V05N09; RESET clears PROG light)\n",
                GetSimClock()->GetElapsedTime(), fr0, fr1, fr2);
        }
    }
}

// =====================================================================
// AGC-side public methods (apolloa / channel handlers call these)
// =====================================================================

bool
SaturnIBAgcBridge::StartUplink( const std::string & strFile )
{
    if ( m_pCurrentUplinkMessage )
    {
        delete m_pCurrentUplinkMessage;
        m_pCurrentUplinkMessage = nullptr;
    }

    m_pCurrentUplinkMessage = new UplinkMessage();

    bool bResult = m_pCurrentUplinkMessage->LoadFromFile( strFile );
    if ( bResult )
    {
        int nKeycode = m_pCurrentUplinkMessage->GetFirstKeycode();
        WriteUplinkKeycode( nKeycode );
    }
    else
    {
        delete m_pCurrentUplinkMessage;
        m_pCurrentUplinkMessage = nullptr;
    }
    return bResult;
}

void
SaturnIBAgcBridge::WriteUplinkKeycode( int nKeycode )
{
    double dUplinkInterval_sec = 0.01;

    if ( nKeycode != -1 && nKeycode < 32 )
    {
        LogInfo1( "UplinkKeycode: %d\n", nKeycode );
        nKeycode &= 037;
        nKeycode |= ((nKeycode << 10) | ((nKeycode ^ 037) << 5));
        m_pAGC->Erasable[0][RegINLINK] = (nKeycode & 077777);
        m_pAGC->InterruptRequests[7] = 1;
        m_dNextUplinkTime_sec = GetSimClock()->GetElapsedTime() + dUplinkInterval_sec;
    }
    else if ( nKeycode == 254 )
    {
        m_dNextUplinkTime_sec = GetSimClock()->GetElapsedTime() + 2.0;
    }
    else if ( nKeycode == 255 )
    {
        m_dNextUplinkTime_sec = GetSimClock()->GetElapsedTime() + 1.0;
    }
    else if ( nKeycode == -1 )
    {
        std::fprintf(stderr, "  UPLINK COMPLETE at elapsed=%.3f sec\n",
                     GetSimClock()->GetElapsedTime());
        delete m_pCurrentUplinkMessage;
        m_pCurrentUplinkMessage = nullptr;
        m_dNextUplinkTime_sec = -1;
    }
}

void
SaturnIBAgcBridge::ISSTurnOnDelayComplete()
{
    // Respond to AGC's 90 second ISS turn "on delay complete" signal.
    unsigned short usValue = 0;
    usValue |= ReadIO( m_pAGC, 030 ) & ~(BIT9);
    WriteIO( m_pAGC, 030, usValue | BIT14 );

    m_eIMUMode = ISS_IMU_COARSE_ALIGN;
    UpdateIMUMode();

    m_bIssTurnOnDelayComplete = true;
    std::fprintf(stderr,
        "  ISS turn-on delay complete at elapsed=%.3f sec\n",
        GetSimClock()->GetElapsedTime());

    if (!m_bNoAttLampOn)
        ArmStateVectorAfterNoAttOff(GetSimClock()->GetElapsedTime());
}

void
SaturnIBAgcBridge::ArmStateVectorAfterNoAttOff(double elapsed_sec)
{
    if (m_dNoAttOffElapsed_sec >= 0.0) return;
    if (!m_bIssTurnOnDelayComplete)    return;

    m_dNoAttOffElapsed_sec = elapsed_sec;
    std::fprintf(stderr,
        "  NO ATT OFF (SV arm) at elapsed=%.3f sec — state-vector uplink in +%.0f s "
        "(enter P00 before then; then P01→P02 after SV)\n",
        m_dNoAttOffElapsed_sec, SV_AFTER_NOATT_OFF_SEC);
    LogInfo("NO ATT OFF (SV arm)\n");
}

void
SaturnIBAgcBridge::NotifyNoAttLamp(bool lampOn)
{
    if (m_bNoAttLampOn && !lampOn)
        ArmStateVectorAfterNoAttOff(GetSimClock()->GetElapsedTime());
    m_bNoAttLampOn = lampOn;
}

void
SaturnIBAgcBridge::ResetCDUReadCounters()
{
    m_dLastSentIMU_OGAResolver_deg = 0.0;
    m_dLastSentIMU_IGAResolver_deg = 0.0;
    m_dLastSentIMU_MGAResolver_deg = 0.0;

    ResetCDUInternalState();       // stack-side reseed (qECItoSM, coarse cmd, needles)
    m_bFirstResetReceived = true;

    m_pAGC->Erasable[0][RegCDUX] = 0;
    m_pAGC->Erasable[0][RegCDUY] = 0;
    m_pAGC->Erasable[0][RegCDUZ] = 0;
}

void
SaturnIBAgcBridge::SetCDURegister( int nRegister, double dAngle_deg )
{
    if ( dAngle_deg < 0.0 )
        dAngle_deg += 360.0;
    int nPulses = (int)( dAngle_deg * 32768.0 / 360.0 );
    m_pAGC->Erasable[0][nRegister] = nPulses & 077777;
}

// =====================================================================
// Private AGC helpers
// =====================================================================

static int CduRegToSignedCount(unsigned short raw)
{
    int v = (int)(raw & 077777);
    if (v & 040000) v -= 0100000;
    return v;
}

// Colossus249 absolute erasable (from ERASABLE_ASSIGNMENTS.s).
static const int AGC_ADDR_RN       = 01170;
static const int AGC_ADDR_VN       = 01176;
static const int AGC_ADDR_PIPTIME  = 01204;
static const int AGC_ADDR_REFSMMAT = 01735;

static int16_t AgcErasableWord(const agc_t *agc, int addr12)
{
    const int a = addr12 & 07777;
    return agc->Erasable[(a >> 8) & 07][a & 0377];
}

static double AgcDecodeDP(int16_t hi, int16_t lo, double scale_factor)
{
    int h = hi & 077777;
    int l = lo & 077777;
    const bool neg = (h & 040000) != 0;
    if (neg)
    {
        h = (~h) & 077777;
        l = (~l) & 077777;
    }
    h &= 037777;
    l &= 037777;
    const long nScaled = ((long)h << 14) | (long)l;
    const double mag = (double)nScaled / 268435456.0 * scale_factor;
    return neg ? -mag : mag;
}

static void AgcDecodeDPVector(const agc_t *agc, int addr0,
                              double scale, double out[3])
{
    for (int i = 0; i < 3; ++i)
    {
        const int16_t hi = AgcErasableWord(agc, addr0 + 2 * i);
        const int16_t lo = AgcErasableWord(agc, addr0 + 2 * i + 1);
        out[i] = AgcDecodeDP(hi, lo, scale);
    }
}

static const char *ImuModeName(ISS_IMU_Mode mode)
{
    switch (mode) {
    case ISS_IMU_TURN_ON: return "TURN_ON";
    case ISS_IMU_COARSE_ALIGN: return "COARSE_ALIGN";
    case ISS_IMU_FINE_ALIGN: return "FINE_ALIGN";
    case ISS_IMU_CAGE: return "CAGE";
    case ISS_IMU_INERTIAL_REFERENCE: return "INERTIAL_REFERENCE";
    case ISS_IMU_ATTITUDE_ERROR_DISPLAY: return "ATT_ERR_DISP";
    default: return "UNKNOWN";
    }
}

void
SaturnIBAgcBridge::AGCUpdatePIPACounters()
{
    // Host guard: plant CM_IMU_PIPA_Pulses currently accumulates large
    // non-physical counts on the pad.  Until the emit path is fixed,
    // do not inject into yaAGC before liftoff — and keep last=plant so
    // the inhibit→enable edge cannot dump a backlog in one step.
    if ( ! m_bLiftoff )
    {
        m_vLastCM_IMU_PIPA_Pulses = m_vCM_IMU_PIPA_Pulses;
        return;
    }

    Vector3d vPIPAPulseCount = m_vCM_IMU_PIPA_Pulses - m_vLastCM_IMU_PIPA_Pulses;
    bool bPIPAInhibit;

    if ( m_eIMUMode == ISS_IMU_INERTIAL_REFERENCE ||
         m_eIMUMode == ISS_IMU_ATTITUDE_ERROR_DISPLAY ||
         m_eIMUMode == ISS_IMU_FINE_ALIGN )
    {
        bPIPAInhibit = false;
    }
    else
    {
        bPIPAInhibit = true;
    }

    auto inject_axis = [&]( double *pLast, double *pInjectedCum,
                            int reg, double delta_axis ) {
        int n = (int) delta_axis;
        *pLast += n;
        while ( n != 0 )
        {
            if ( n > 0 )
            {
                if ( ! bPIPAInhibit )
                {
                    UnprogrammedIncrement ( m_pAGC, reg, 0 );
                    *pInjectedCum += 1.0;
                }
                n--;
            }
            else
            {
                if ( ! bPIPAInhibit )
                {
                    UnprogrammedIncrement ( m_pAGC, reg, 2 );
                    *pInjectedCum -= 1.0;
                }
                n++;
            }
        }
    };

    inject_axis( &m_vLastCM_IMU_PIPA_Pulses.x, &m_vPipaPulsesInjectedCum.x,
                 RegPIPAX, vPIPAPulseCount.x );
    inject_axis( &m_vLastCM_IMU_PIPA_Pulses.y, &m_vPipaPulsesInjectedCum.y,
                 RegPIPAY, vPIPAPulseCount.y );
    inject_axis( &m_vLastCM_IMU_PIPA_Pulses.z, &m_vPipaPulsesInjectedCum.z,
                 RegPIPAZ, vPIPAPulseCount.z );
}

void
SaturnIBAgcBridge::AGCUpdateGimbalCDUCounters()
{
    const double dOnePulse_deg = READ_COUNTER_PULSES_TO_DEG(1);
    double dGimbalDelta_deg;
    int nSign;

    if ( m_eIMUMode == ISS_IMU_TURN_ON )
    {
        UpdateDebugText();
        return;
    }

    if ( m_bFirstPass )
    {
        m_dLastSentIMU_OGAResolver_deg = m_dIMU_OGAResolver_deg;
        m_dLastSentIMU_IGAResolver_deg = m_dIMU_IGAResolver_deg;
        m_dLastSentIMU_MGAResolver_deg = m_dIMU_MGAResolver_deg;
    }
    else
    {
        UpdateIMUMode();

        // Outer (X)
        dGimbalDelta_deg = m_dIMU_OGAResolver_deg - m_dLastSentIMU_OGAResolver_deg;
        if ( dGimbalDelta_deg >  180.0 ) dGimbalDelta_deg -= 360.0;
        else if ( dGimbalDelta_deg < -180.0 ) dGimbalDelta_deg += 360.0;
        nSign = (dGimbalDelta_deg > 0.0) ? 1 : -1;
        dGimbalDelta_deg = fabs( dGimbalDelta_deg );
        while ( dGimbalDelta_deg > dOnePulse_deg )
        {
            if ( nSign == 1 ) m_dLastSentIMU_OGAResolver_deg += dOnePulse_deg;
            else              m_dLastSentIMU_OGAResolver_deg -= dOnePulse_deg;
            SetCDURegister( RegCDUX, m_dLastSentIMU_OGAResolver_deg );
            m_dTotalCDUPulseIncrements.x += 1.0;
            dGimbalDelta_deg -= dOnePulse_deg;
        }
        if ( m_dLastSentIMU_OGAResolver_deg >=  180.0 ) m_dLastSentIMU_OGAResolver_deg -= 360.0;
        else if ( m_dLastSentIMU_OGAResolver_deg < -180.0 ) m_dLastSentIMU_OGAResolver_deg += 360.0;

        // Inner (Y)
        dGimbalDelta_deg = m_dIMU_IGAResolver_deg - m_dLastSentIMU_IGAResolver_deg;
        if ( dGimbalDelta_deg >  180.0 ) dGimbalDelta_deg -= 360.0;
        else if ( dGimbalDelta_deg < -180.0 ) dGimbalDelta_deg += 360.0;
        nSign = (dGimbalDelta_deg > 0.0) ? 1 : -1;
        dGimbalDelta_deg = fabs( dGimbalDelta_deg );
        while ( dGimbalDelta_deg > dOnePulse_deg )
        {
            if ( nSign == 1 ) m_dLastSentIMU_IGAResolver_deg += dOnePulse_deg;
            else              m_dLastSentIMU_IGAResolver_deg -= dOnePulse_deg;
            SetCDURegister( RegCDUY, m_dLastSentIMU_IGAResolver_deg );
            m_dTotalCDUPulseIncrements.y += 1.0;
            dGimbalDelta_deg -= dOnePulse_deg;
        }
        if ( m_dLastSentIMU_IGAResolver_deg >=  180.0 ) m_dLastSentIMU_IGAResolver_deg -= 360.0;
        else if ( m_dLastSentIMU_IGAResolver_deg < -180.0 ) m_dLastSentIMU_IGAResolver_deg += 360.0;

        // Middle (Z)
        dGimbalDelta_deg = m_dIMU_MGAResolver_deg - m_dLastSentIMU_MGAResolver_deg;
        if ( dGimbalDelta_deg >  180.0 ) dGimbalDelta_deg -= 360.0;
        else if ( dGimbalDelta_deg < -180.0 ) dGimbalDelta_deg += 360.0;
        nSign = (dGimbalDelta_deg > 0.0) ? 1 : -1;
        dGimbalDelta_deg = fabs( dGimbalDelta_deg );
        while ( dGimbalDelta_deg > dOnePulse_deg )
        {
            if ( nSign == 1 ) m_dLastSentIMU_MGAResolver_deg += dOnePulse_deg;
            else              m_dLastSentIMU_MGAResolver_deg -= dOnePulse_deg;
            SetCDURegister( RegCDUZ, m_dLastSentIMU_MGAResolver_deg );
            m_dTotalCDUPulseIncrements.z += 1.0;
            dGimbalDelta_deg -= dOnePulse_deg;
        }
        if ( m_dLastSentIMU_MGAResolver_deg >=  180.0 ) m_dLastSentIMU_MGAResolver_deg -= 360.0;
        else if ( m_dLastSentIMU_MGAResolver_deg < -180.0 ) m_dLastSentIMU_MGAResolver_deg += 360.0;
    }

    UpdateDebugText();
}

void
SaturnIBAgcBridge::MaybeLogImuPipaCdu()
{
    if (!m_bImuDebug || !m_pImuDebugFile || !m_pAGC) return;

    const double t = GetSimClock()->GetElapsedTime();
    const int sec = (int)t;
    if (sec == m_nImuDebugLastSec) return;
    m_nImuDebugLastSec = sec;

    const bool pipaInhibit =
        !m_bLiftoff ||
        !(m_eIMUMode == ISS_IMU_INERTIAL_REFERENCE ||
          m_eIMUMode == ISS_IMU_ATTITUDE_ERROR_DISPLAY ||
          m_eIMUMode == ISS_IMU_FINE_ALIGN);

    const int cdu_x = CduRegToSignedCount(m_pAGC->Erasable[0][RegCDUX]);
    const int cdu_y = CduRegToSignedCount(m_pAGC->Erasable[0][RegCDUY]);
    const int cdu_z = CduRegToSignedCount(m_pAGC->Erasable[0][RegCDUZ]);
    const double cdu_x_deg = cdu_x * 360.0 / 32768.0;
    const double cdu_y_deg = cdu_y * 360.0 / 32768.0;
    const double cdu_z_deg = cdu_z * 360.0 / 32768.0;

    const ExternalOutputs_saturn_ib_stack *out = &m_outputs;
    const double vex = out->OUT11[OUT11_Ve_x_mps];
    const double vey = out->OUT11[OUT11_Ve_y_mps];
    const double vez = out->OUT11[OUT11_Ve_z_mps];
    const double vemag = std::sqrt(vex * vex + vey * vey + vez * vez);

    const double res_x = m_vCM_IMU_PIPA_Pulses.x - m_vLastCM_IMU_PIPA_Pulses.x;
    const double res_y = m_vCM_IMU_PIPA_Pulses.y - m_vLastCM_IMU_PIPA_Pulses.y;
    const double res_z = m_vCM_IMU_PIPA_Pulses.z - m_vLastCM_IMU_PIPA_Pulses.z;

    const double xe = out->OUT22[OUT22_Xe_x_m];
    const double ye = out->OUT22[OUT22_Xe_y_m];
    const double ze = out->OUT22[OUT22_Xe_z_m];
    const double rmag = std::sqrt(xe * xe + ye * ye + ze * ze);
    double sm_z_dot_down = 0.0, sm_x_dot_az = 0.0, sm_x_elev_deg = 0.0;
    double xsm_x = 0.0, xsm_y = 0.0, xsm_z = 0.0;
    double zsm_x = 0.0, zsm_y = 0.0, zsm_z = 0.0;
    const double az_deg = m_inputs.A_z_deg;
    if (rmag > 1.0) {
        const double up_x = xe / rmag;
        const double up_y = ye / rmag;
        const double up_z = ze / rmag;
        double east_x = -up_y, east_y = up_x, east_z = 0.0;
        const double east_mag = std::sqrt(east_x * east_x + east_y * east_y);
        if (east_mag > 1e-12) {
            east_x /= east_mag;
            east_y /= east_mag;
            const double north_x = up_y * east_z - up_z * east_y;
            const double north_y = up_z * east_x - up_x * east_z;
            const double north_z = up_x * east_y - up_y * east_x;
            const double az_rad = az_deg * (M_PI / 180.0);
            const double caz = std::cos(az_rad);
            const double saz = std::sin(az_rad);
            const double az_x = caz * north_x + saz * east_x;
            const double az_y = caz * north_y + saz * east_y;
            const double az_z = caz * north_z + saz * east_z;

            Matrix4d mECItoSM = m_qECItoSM.QuaternionToMatrix4d();
            xsm_x = mECItoSM.m[0][0]; xsm_y = mECItoSM.m[1][0]; xsm_z = mECItoSM.m[2][0];
            zsm_x = mECItoSM.m[0][2]; zsm_y = mECItoSM.m[1][2]; zsm_z = mECItoSM.m[2][2];
            sm_z_dot_down = -(zsm_x * up_x + zsm_y * up_y + zsm_z * up_z);
            sm_x_dot_az = xsm_x * az_x + xsm_y * az_y + xsm_z * az_z;
            const double x_up = xsm_x * up_x + xsm_y * up_y + xsm_z * up_z;
            sm_x_elev_deg = std::asin(std::max(-1.0, std::min(1.0, x_up))) * (180.0 / M_PI);
        }
    }

    const double POS_SCALE = 536870912.0;
    const double VEL_SCALE = 128.0 * 100.0;
    const double TIME_SCALE = 268435456.0;
    const double REFSM_SCALE = 2.0;
    double agc_rn[3] = {0, 0, 0};
    double agc_vn[3] = {0, 0, 0};
    double refsm[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    AgcDecodeDPVector(m_pAGC, AGC_ADDR_RN, POS_SCALE, agc_rn);
    AgcDecodeDPVector(m_pAGC, AGC_ADDR_VN, VEL_SCALE, agc_vn);
    for (int i = 0; i < 9; ++i)
    {
        const int16_t hi = AgcErasableWord(m_pAGC, AGC_ADDR_REFSMMAT + 2 * i);
        const int16_t lo = AgcErasableWord(m_pAGC, AGC_ADDR_REFSMMAT + 2 * i + 1);
        refsm[i] = AgcDecodeDP(hi, lo, REFSM_SCALE);
    }
    const double piptime_csec = AgcDecodeDP(
        AgcErasableWord(m_pAGC, AGC_ADDR_PIPTIME),
        AgcErasableWord(m_pAGC, AGC_ADDR_PIPTIME + 1),
        TIME_SCALE);
    const double piptime_sec = piptime_csec * 0.01;

    const double plant_ve_x = out->OUT22[OUT22_Ve_x_mps];
    const double plant_ve_y = out->OUT22[OUT22_Ve_y_mps];
    const double plant_ve_z = out->OUT22[OUT22_Ve_z_mps];
    const double drn_x = agc_rn[0] - xe;
    const double drn_y = agc_rn[1] - ye;
    const double drn_z = agc_rn[2] - ze;
    const double rn_err_m = std::sqrt(drn_x*drn_x + drn_y*drn_y + drn_z*drn_z);
    const double dvn_x = agc_vn[0] - plant_ve_x;
    const double dvn_y = agc_vn[1] - plant_ve_y;
    const double dvn_z = agc_vn[2] - plant_ve_z;
    const double vn_err_mps = std::sqrt(dvn_x*dvn_x + dvn_y*dvn_y + dvn_z*dvn_z);
    const double refsm_x_dot_smx =
        refsm[0]*xsm_x + refsm[1]*xsm_y + refsm[2]*xsm_z;
    const double refsm_z_dot_smz =
        refsm[6]*zsm_x + refsm[7]*zsm_y + refsm[8]*zsm_z;

    std::fprintf(m_pImuDebugFile,
        "%.3f,%d,%s,%d,%d,"
        "%.6f,%.6f,%.6f,"
        "%.1f,%.1f,%.1f,"
        "%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,"
        "%d,%d,%d,"
        "%.4f,%.4f,%.4f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.4f,"
        "%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,"
        "%.6f,%.6f,%.4f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%.4f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.3f,%.4f,"
        "%.6f,%.6f\n",
        t, m_bLiftoff ? 1 : 0, ImuModeName(m_eIMUMode),
        m_bNoAttLampOn ? 1 : 0, pipaInhibit ? 1 : 0,
        m_vCM_IMU_PIPA_Pulses.x, m_vCM_IMU_PIPA_Pulses.y, m_vCM_IMU_PIPA_Pulses.z,
        m_vPipaPulsesInjectedCum.x, m_vPipaPulsesInjectedCum.y, m_vPipaPulsesInjectedCum.z,
        res_x, res_y, res_z,
        m_dIMU_OGAResolver_deg, m_dIMU_IGAResolver_deg, m_dIMU_MGAResolver_deg,
        cdu_x, cdu_y, cdu_z, cdu_x_deg, cdu_y_deg, cdu_z_deg,
        m_qECItoSM.GetE(0), m_qECItoSM.GetE(1), m_qECItoSM.GetE(2), m_qECItoSM.GetE(3),
        vex, vey, vez, vemag,
        out->OUT11[OUT11_h_m], out->OUT11[OUT11_Vb_mps],
        plant_ve_x, plant_ve_y, plant_ve_z,
        xe, ye, ze, az_deg,
        sm_z_dot_down, sm_x_dot_az, sm_x_elev_deg,
        agc_rn[0], agc_rn[1], agc_rn[2],
        agc_vn[0], agc_vn[1], agc_vn[2],
        piptime_sec,
        refsm[0], refsm[1], refsm[2],
        refsm[3], refsm[4], refsm[5],
        refsm[6], refsm[7], refsm[8],
        rn_err_m, vn_err_mps,
        refsm_x_dot_smx, refsm_z_dot_smz);
    std::fflush(m_pImuDebugFile);
}

// =====================================================================
// SaveState override
// =====================================================================

void
SaturnIBAgcBridge::SaveState( const std::string & strName )
{
    char szCoreName[1024];

    strcpy_s( szCoreName, sizeof(szCoreName), strName.c_str() );
    strcat_s( szCoreName, sizeof(szCoreName), ".agc_core" );
    MakeCoreDump( m_pAGC, szCoreName );

    SaturnIBStack::SaveState(strName);
}

// =====================================================================
// yaAGC callbacks (extern "C"; all AGC-side logic).
// =====================================================================

extern "C" {

    int ChannelInput ( agc_t *State )
    {
        int nResult = 0;
        if ( g_AGCChannelInputQueue.size() > 0 )
        {
            int nValue;
            ChannelIOList::iterator p = g_AGCChannelInputQueue.begin();
            ChannelIOOperation op = *p;
            g_AGCChannelInputQueue.pop_front();

            nValue = op.m_usValue & op.m_usMask;
            nValue |= ReadIO( State, op.m_usChannel ) & ~op.m_usMask;
            WriteIO( State, op.m_usChannel, nValue );

            if ( op.m_usChannel == 015 )
                State->InterruptRequests[5] = 1;

            nResult = 1;
        }
        return nResult;
    }

    void ChannelRoutine (agc_t *State)
    {
        (void)State;
    }

    void RequestRadarData (agc_t *State)
    {
        (void)State;
    }

    void ChannelOutput (agc_t * State, int Channel, int Value)
    {
        extern int DebugMode;
        if (Channel == 7)
        {
            State->InputChannel[7] = State->OutputChannel7 = (Value & 0160);
            return;
        }
        if (Channel == 013 && 0600 == (0600 & Value) && !CmOrLm)
        {
            State->Erasable[0][042] = 0;  // LastRhcPitch — unused in CSM
            State->Erasable[0][043] = 0;
            State->Erasable[0][044] = 0;
        }

        if (Channel == 012)
        {
            if ( g_pSaturnIBAgcBridge )
            {
                g_pSaturnIBAgcBridge->SetCoarseAlignEnable( (Value & BIT4) ? true : false );
                g_pSaturnIBAgcBridge->SetErrorCounterEnable( (Value & BIT6) ? true : false );
                if ( Value & BIT5 )
                    g_pSaturnIBAgcBridge->ResetCDUReadCounters();
                if ( Value & BIT15 )
                    g_pSaturnIBAgcBridge->ISSTurnOnDelayComplete();
            }
        }
        else if ( Channel == 0174 )
        {
            short nSign = (Value & BIT15) ? -1 : 1;
            unsigned short nPulseCount = Value & 037777;
            if ( g_pSaturnIBAgcBridge )
                g_pSaturnIBAgcBridge->UpdateIMUXGimbal( nSign, nPulseCount );
        }
        else if ( Channel == 0175 )
        {
            short nSign = (Value & BIT15) ? -1 : 1;
            unsigned short nPulseCount = Value & 037777;
            if ( g_pSaturnIBAgcBridge )
                g_pSaturnIBAgcBridge->UpdateIMUYGimbal( nSign, nPulseCount );
        }
        else if ( Channel == 0176 )
        {
            short nSign = (Value & BIT15) ? -1 : 1;
            unsigned short nPulseCount = Value & 037777;
            if ( g_pSaturnIBAgcBridge )
                g_pSaturnIBAgcBridge->UpdateIMUZGimbal( nSign, nPulseCount );
        }
        else if ( Channel == 0177 )
        {
            int nSign = (Value & BIT15) ? -1 : 1;
            unsigned short usPulseCount = Value & 03777;
            ISS_Fine_Align_Axis eAxis = FA_NO_OPERATION;

            if ( Value & BIT12 )
            {
                if ( Value & BIT14 )
                {
                    if ( Value & BIT13 ) eAxis = FA_Z_AXIS;
                    else                 eAxis = FA_Y_AXIS;
                }
                else
                {
                    if ( Value & BIT13 ) eAxis = FA_X_AXIS;
                    else                 eAxis = FA_NO_OPERATION;
                }

                if ( g_pSaturnIBAgcBridge )
                {
                    if ( eAxis != FA_NO_OPERATION )
                        g_pSaturnIBAgcBridge->IncrementFineAlignPulseCount( eAxis, nSign, usPulseCount );
                    else
                        g_pSaturnIBAgcBridge->IncrementFineAlignPulseCount( FA_NO_OPERATION, 0, 0 );
                }
            }
            else
            {
                if ( g_pSaturnIBAgcBridge )
                    g_pSaturnIBAgcBridge->IncrementFineAlignPulseCount( FA_NO_OPERATION, 0, 0 );
            }
        }
        else if ( Channel == 013 || Channel == 034 || Channel == 035 )
        {
            int CmOrLm = 1;
            DecodeDigitalDownlink( Channel, Value, CmOrLm );
        }
        else if ( Channel == 010 )
        {
            const int cmd = (Value >> 11) & 037;
            if ( cmd == 12 && g_pSaturnIBAgcBridge )
                g_pSaturnIBAgcBridge->NotifyNoAttLamp( (Value & BIT4) != 0 );
            ChannelIOOperation op( (unsigned short) Channel, (unsigned short) Value );
            g_AGCChannelOutputQueue.push_back( op );
        }
        else
        {
            ChannelIOOperation op( (unsigned short) Channel, (unsigned short) Value );
            g_AGCChannelOutputQueue.push_back( op );
        }
    }

    void ShiftToDeda (agc_t *State, int Data)
    {
        (void)State; (void)Data;
    }
}

// =====================================================================
// AGC downlink → bridge downlink buffer
// =====================================================================

static void
SwriteDownlinkWriter()
{
    if (!g_pSaturnIBAgcBridge) return;

    char * pText = g_pSaturnIBAgcBridge->GetDownlinkText();
    *pText = '\0';

    for (size_t i = 0; i < (size_t) Sheight; i++)
    {
        size_t n = strlen( Sbuffer[i] );
        strcpy( pText, Sbuffer[i] );
        strcat( pText, "\n" );
        pText += n + 1;
    }

    static bool s_loggedFirstDownlink = false;
    if (!s_loggedFirstDownlink && Sheight > 0 && Sbuffer[0][0] != '\0')
    {
        s_loggedFirstDownlink = true;
        std::fprintf(stderr, "AGC downlink: first decoded list \"%s\"\n",
                     Sbuffer[0]);
    }
}
