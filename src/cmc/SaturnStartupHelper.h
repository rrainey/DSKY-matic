// Brings up the clock, log and launch vehicle (with CMC bridge) for the
// interactive application.

#pragma once

#include "SimSupport.h"

class SaturnIBAgcBridge;

class SaturnStartupHelper
{
public:
    SaturnStartupHelper();
    ~SaturnStartupHelper();

    SaturnIBAgcBridge *GetLaunchVehicle();

private:
    void Initialize();

    sim::SimClock *m_pClock = nullptr;
    sim::LogManager *m_pLogger = nullptr;
    SaturnIBAgcBridge *m_pVehicle = nullptr;
};
