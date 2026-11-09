#pragma once

#include "SaturnIBStack.h"
#include "IAgcBridge.h"

extern "C" {
// Forward declaration from yaAGC (full definition in agc_engine.h).
typedef struct agc_t agc_t;
};

class UplinkMessage;

/**
 * Saturn-IB ↔ CMC (yaAGC) bridge.
 *
 * Inherits from SaturnIBStack so that:
 *  - it IS-A SaturnIBStack from the SimClock's perspective (apolloa
 *    registers a bridge, batch_sim registers a bare stack),
 *  - bridge method bodies can access stack-protected IMU/physics state
 *    directly via inheritance (no accessor-chain overhead).
 *
 * All yaAGC includes, state, callbacks, and the Colossus ROM load live in
 * saturnib_cmc (this translation unit).  SaturnIBStack has no yaAGC
 * dependency.
 */
class SaturnIBAgcBridge : public SaturnIBStack, public IAgcBridge
{
public:
    SaturnIBAgcBridge(const BatchConfig & config = BatchConfig());
    virtual ~SaturnIBAgcBridge();

    /** yaAGC decoded downlink text for the host panel. */
    char * GetDownlinkText() { return m_szDownlinkText; }

    /** Diagnostic accessor; apolloa uses this to inspect channel discretes. */
    agc_t * DebugGetAGC() { return m_pAGC; }

    /** Open an uplink file and stream its contents to the CMC. */
    bool StartUplink( const std::string & strFile );

    /** Called when the AGC signals ISS turn-on-delay complete (ch012 BIT15). */
    void ISSTurnOnDelayComplete();

    /** DSKY NO ATT lamp edge from ch010 DSPTAB — gates auto SV uplink. */
    void NotifyNoAttLamp(bool lampOn);

    /** AGC-side CDU register zero + stack internal reset. */
    void ResetCDUReadCounters();

    /** Simulate P/MINC increment of a CDU register (writes AGC erasable). */
    void SetCDURegister( int nRegister, double dAngle_deg );

    // ---- IAgcBridge hook overrides, called from SaturnIBStack::TimeStep.
    void OnTimeStepBegin(double elapsed_sec,
                         const sim::Vector3d &pad_pos_eci_m,
                         const sim::Vector3d &pad_vel_eci_mps) override;
    void UplinkTimeStep() override;
    void InjectGimbalCDU() override;
    void StepAGC(double dt_sec) override;
    void InjectPIPA() override;
    void OnTimeStepEnd() override;

    /** Append the AGC core dump to the stack's state snapshot. */
    void SaveState( const std::string & strName ) override;

protected:
    // ---- AGC-owned state ---------------------------------------------------
    agc_t * m_pAGC;

    /* BatchConfig-derived state consulted at runtime. */
    std::string m_pad_load_file;
    double      m_pad_load_start_sec;

    /* Uplink queue state. */
    UplinkMessage * m_pCurrentUplinkMessage;
    double  m_dNextUplinkTime_sec;

    /* Downlink text buffer. */
    char   m_szDownlinkText[16*1024];

    /* Cumulative signed PIPA pulses actually injected into yaAGC. */
    sim::Vector3d m_vLastCM_IMU_PIPA_Pulses;
    sim::Vector3d m_vPipaPulsesInjectedCum;

    /* Tracks last observed stack liftoff flag so we can write the ch030 BIT5
     * discrete only on the edge. */
    bool   m_bLastSeenLiftoff;

    /* ISS turn-on / NO ATT synchronization. */
    bool   m_bNoAttLampOn;
    bool   m_bIssTurnOnDelayComplete;
    double m_dNoAttOffElapsed_sec;
    bool   m_bFirstResetReceived;

    /* AGC timestep remainder. */
    double m_dAGCResidualDeltaT_sec;

    /* IMU_DEBUG CSV logger state. */
    bool   m_bImuDebug;
    FILE * m_pImuDebugFile;
    int    m_nImuDebugLastSec;

    static constexpr double SV_AFTER_NOATT_OFF_SEC = 45.0;

    // ---- Bridge-internal helpers -------------------------------------------
    void WriteUplinkKeycode( int nKeycode );
    void AGCUpdatePIPACounters();
    void AGCUpdateGimbalCDUCounters();
    void MaybeLogImuPipaCdu();
    void ArmStateVectorAfterNoAttOff(double elapsed_sec);
};

/** Bridge-side singleton used by yaAGC callbacks (ChannelOutput etc.) and
 *  by the downlink writer. */
extern SaturnIBAgcBridge * g_pSaturnIBAgcBridge;
