#pragma once

#include <vector>
#include "Vector.h"


class UplinkMessage
{
public:
    UplinkMessage(void);
    virtual ~UplinkMessage(void);

    int GetKeycodeCount() const { return m_keycodeList.size(); };

    int GetKeycode(int nIndex) const { return m_keycodeList[nIndex]; };

    int GetNextKeycode();

    int GetFirstKeycode() { m_nCursor = 0; return GetNextKeycode(); };

    bool LoadFromFile( const std::string & strPath );

    /**
     * Compose a Colossus249 P27/V71 CSM state-vector contiguous update.
     * Sequence: V71E, 21E, UPSVFLAG ECADR, id, Rxyz, Vxyz, T, V33E, KEY REL
     * (UPDATE_PROGRAM.s / GSOP R-577 §2.1).  dTime_sec is time from CMC
     * clock zero in seconds.  Caller supplies ECI meters / m/s; scaling to
     * AGC B-29 / B-7 / B-28 is performed here.  Numeric frame fidelity of
     * the host state vs CMC expectations is the caller's responsibility.
     */
    void GenerateStateVectorUpdate ( const sim::Vector3d & vECIPos_meters,
                                     const sim::Vector3d & vECIVel_mps,
                                     double dTime_sec );

    /**
     * Encode the low-order 15-bits of the given word for transmission
     */
    void EncodeWord( int nWord );

protected:
    std::vector<int> m_keycodeList;

    /**
     * Index of next keycode position to be returned by GetNextKeycode();
     */
    unsigned int m_nCursor;
};
