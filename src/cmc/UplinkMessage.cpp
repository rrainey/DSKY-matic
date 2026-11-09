#include "UplinkMessage.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include "SimSupport.h"

using namespace std;
using namespace sim;

UplinkMessage::UplinkMessage(void)
{
    m_nCursor = 0;
}

UplinkMessage::~UplinkMessage(void)
{
}

int 
UplinkMessage::GetNextKeycode()
{ 
    int nResult = -1;

    if( m_nCursor < m_keycodeList.size() )
    {
        nResult = m_keycodeList[m_nCursor++];
    }
    return nResult;
}

bool UplinkMessage::LoadFromFile( const std::string & strPath )
{
    std::ifstream in( strPath.c_str(), ios::in );
    char    c[1];
    int nKeycode;

    while ( ! in.eof() )
    {
        in.read( c, 1 );
        switch (toupper (c[0]))
        {
        //case 'Z':  nKeycode = 254; break;

        case '#':  
            {
                // Skip remainder of the line
                while (c[0] != '\n' && ! in.eof() ) 
                {
                    in.read( c, 1 );
                }
            }
            break;

        case 'V':  nKeycode = 17; break;
        case 'N':  nKeycode = 31; break;
        case '+':  nKeycode = 26; break;
        case '-':  nKeycode = 27; break;
        case '0':  nKeycode = 16; break;
        case '7':  nKeycode = 7; break;
        case '4':  nKeycode = 4; break;
        case '1':  nKeycode = 1; break;
        case '8':  nKeycode = 8; break;
        case '5':  nKeycode = 5; break;
        case '3':  nKeycode = 3; break;
        case '2':  nKeycode = 2; break;
        case '9':  nKeycode = 9; break;
        case '6':  nKeycode = 6; break;
        case 'C':  nKeycode = 30; break;
        case 'K':  nKeycode = 25; break;
        case 'E':  nKeycode = 28; break;
        case 'R':  nKeycode = 18; break;
        case '!':  nKeycode = 255; break; // one second delay
        case 'P':  // The "PRO" key has no keycode, and falls through.
        default:   nKeycode = -1; break;
        }

        if (nKeycode >= 0)
        {
            m_keycodeList.push_back( nKeycode );
        }
    }
    in.close();

    return m_keycodeList.size() > 0;
}

void UplinkMessage::GenerateStateVectorUpdate ( const sim::Vector3d & vECIPos_meters,
                                                const sim::Vector3d & vECIVel_mps,
                                                double dTime_sec )
{
    // Colossus249 / UPDATE_PROGRAM.s CSM state-vector V71 contiguous update:
    //   V71E  21E  <ECADR UPSVFLAG>E  <id>E  Rx..Rz  Vx..Vz  T  V33E  K
    // II=21 (octal) => COMPNUMB=17d: AAAA + 15 data words (id + 3*DP pos +
    // 3*DP vel + DP time).  Each load flashes V21N01; after all data, P27
    // flashes V21N02 for verify — V33E stores.  KEY REL clears UPLK ACTY.
    //
    // Scales (Earth SOI): position m at B-29, velocity m/csec at B-7,
    // time centiseconds at B-28.  dTime_sec must be AGC clock time since
    // CMC clock zero (matches TEPHEM epoch), not an arbitrary GET.
    //
    // NOTE: Numeric correctness of pos/vel vs CMC frame is a separate
    // validation concern; this routine only emits a well-formed P27 dialog.

    const double TIME_SCALE_FACTOR = 268435456.;   // 2^28
    const double POS_SCALE_FACTOR = 536870912.;    // 2^29
    const double VEL_SCALE_FACTOR = 128. * 100.0;  // B-7 m/csec, input m/s
    const double F_SCALE = 268435456.0;            // 2^28
    // Colossus249: UPSVFLAG at E3,1501 → absolute erasable 01501.
    const int UPSVFLAG_ECADR = 01501;

    auto push_delay_1s = [this]() { m_keycodeList.push_back( 255 ); };
    auto push_delay_2s = [this]() { m_keycodeList.push_back( 254 ); };
    auto push_enter = [this, &push_delay_1s]() {
        m_keycodeList.push_back( 28 );  // ENTER
        // Allow P27 to accept the component and re-flash V21N01/N02.
        push_delay_1s();
    };
    auto push_dp_component = [this, &push_enter, F_SCALE]( double value,
                                                           double scale_factor ) {
        const bool negative = value < 0.0;
        long nScaled = (long) ( fabs( value ) / scale_factor * F_SCALE );
        int nValues[2];
        nValues[0] = (nScaled >> 14) & 037777;
        nValues[1] = (nScaled) & 037777;
        if ( negative )
        {
            // AGC SP ones'-complement (matches prior EncodeWord path).
            nValues[0] = (~nValues[0]) & 077777;
            nValues[1] = (~nValues[1]) & 077777;
        }
        EncodeWord( nValues[0] );
        push_enter();
        EncodeWord( nValues[1] );
        push_enter();
    };

    // Settle after prior pad-load KEY REL before touching the DSKY again.
    push_delay_2s();
    push_delay_2s();

    // Colossus249: P27 accepts CSM updates only in P00, P02, or fresh start
    // (UPDATE_PROGRAM.s).  Force P00 — same precondition MCC used
    // ("P00 and Accept for a new state vector").
    // V37 ENTER 00 ENTER — do NOT press NOUN (see ApolloA pad-load notes).
    m_keycodeList.push_back( 17 );  // VERB
    m_keycodeList.push_back( 3 );
    m_keycodeList.push_back( 7 );
    m_keycodeList.push_back( 28 );  // ENTER
    push_delay_1s();
    m_keycodeList.push_back( 16 );  // 0
    m_keycodeList.push_back( 16 );  // 0
    m_keycodeList.push_back( 28 );  // ENTER
    push_delay_2s();
    push_delay_1s();

    // V71E — contiguous block update (enters P27)
    m_keycodeList.push_back( 17 );  // VERB
    m_keycodeList.push_back( 7 );
    m_keycodeList.push_back( 1 );
    push_enter();
    push_delay_2s();  // wait for first V21N01 (II)

    // II = 21 (octal) — component count for CSM/LM state vector
    m_keycodeList.push_back( 2 );
    m_keycodeList.push_back( 1 );
    push_enter();

    // ECADR of UPSVFLAG
    EncodeWord( UPSVFLAG_ECADR );
    push_enter();

    // Identifier: 00001 = CSM, Earth SOI scaling
    EncodeWord( 000001 );
    push_enter();

    // Position (m, B-29)
    push_dp_component( vECIPos_meters.x, POS_SCALE_FACTOR );
    push_dp_component( vECIPos_meters.y, POS_SCALE_FACTOR );
    push_dp_component( vECIPos_meters.z, POS_SCALE_FACTOR );

    // Velocity (m/s → m/csec, B-7)
    push_dp_component( vECIVel_mps.x, VEL_SCALE_FACTOR );
    push_dp_component( vECIVel_mps.y, VEL_SCALE_FACTOR );
    push_dp_component( vECIVel_mps.z, VEL_SCALE_FACTOR );

    // Time from AGC clock zero (centiseconds, B-28)
    push_dp_component( dTime_sec * 100.0, TIME_SCALE_FACTOR );

    // Verify flash is V21N02 — V33E accepts and stores
    push_delay_2s();
    m_keycodeList.push_back( 17 );  // VERB
    m_keycodeList.push_back( 3 );
    m_keycodeList.push_back( 3 );
    push_enter();

    // Release uplink / display lock (same as pad-load trailer)
    push_delay_1s();
    m_keycodeList.push_back( 25 );  // KEY REL
}

static const int nMask[] = { 070000, 007000, 000700, 000070, 000007 };
static const int nShift[] = { 12, 9, 6, 3, 0 };

void UplinkMessage::EncodeWord( int nWord )
{
    for ( int i=0; i<5; ++i )
    {
        int nCode = ((nWord & nMask[i]) >> nShift[i]);
        if ( nCode == 0 ) 
        {
            nCode = 16;
        }
        m_keycodeList.push_back( nCode );
        //LogInfo1 ("Coded key: %d\n", ((nWord & nMask[i]) >> nShift[i]) );
    }
}