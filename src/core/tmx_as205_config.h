// Explicit ExternalInputs_saturn_ib_stack_tag presetting for AS-205 per
// NASA TM-X-62831.  Both dskymatic-studio and dskymatic-headless call
// tmx_as205_config() and feed the result to SaturnStartupHelper, so every
// member of ExternalInputs_saturn_ib_stack_tag is pinned explicitly here
// rather than relying on the upstream BatchConfig NaN-sentinel defaults.
//
// LaunchDate[6] is not a BatchConfig member — it flows in via SimClock,
// which SaturnStartupHelper::Initialize sets to 1968-10-11 14:57:45 UT.

#pragma once

#include <SaturnIBStack.h>

namespace dskymatic {

inline SaturnIBStack::BatchConfig tmx_as205_config() {
    SaturnIBStack::BatchConfig cfg;
    cfg.A_z_deg         = 72.0;                 // TM-X-62831 flight azimuth
    cfg.CG_LLA_deg_m[0] = 28.521963;            // KSC LC-34 (Table 3B phi_L)
    cfg.CG_LLA_deg_m[1] = -80.561141;
    cfg.CG_LLA_deg_m[2] = 34.7;
    cfg.T_L_prime_sec   = 300.0;                // sim-start → liftoff
    cfg.GRR_offset_sec  = -5.0;                 // GRR 5 s before liftoff (TMX)
    cfg.q_ECItoSM[0]    = 1.0;                  // identity; the plant
    cfg.q_ECItoSM[1]    = 0.0;                  // overwrites this on every
    cfg.q_ECItoSM[2]    = 0.0;                  // TimeStep from m_qECItoSM.
    cfg.q_ECItoSM[3]    = 0.0;
    return cfg;
}

}  // namespace dskymatic
