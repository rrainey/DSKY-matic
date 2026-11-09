#include <list>

class ChannelIOOperation {
public:
    ChannelIOOperation( unsigned short usChannel, 
                        unsigned short usValue,
                        unsigned short usMask = 077777 )
    {
        m_usChannel = usChannel;
        m_usValue = usValue;
        m_usMask = usMask;
    }

    unsigned short m_usChannel;
    unsigned short m_usValue;
    unsigned short m_usMask;
};

typedef std::list<ChannelIOOperation>   ChannelIOList;

extern ChannelIOList g_AGCChannelOutputQueue;
extern ChannelIOList g_AGCChannelInputQueue;

// Channel definitions from Delco Electronics, Apollo 15 CM SOFTWARE Manual
// Colossus 3 (ARTEMIS 72) Software Description

/**
 * CHANNEL WORD BIT NUMBERS
 */
#define BIT16  100000
#define BIT15  040000
#define BIT14  020000
#define BIT13  010000
#define BIT12  004000
#define BIT11  002000
#define BIT10  001000
#define BIT9   000400
#define BIT8   000200
#define BIT7   000100
#define BIT6   000040
#define BIT5   000020
#define BIT4   000010
#define BIT3   000004
#define BIT2   000002
#define BIT1   000001

/**
 * S-IVB Separate, Abort; 0 = TRUE
 */
#define CH030_SIVB_SEPARATE_ABORT       BIT4

/**
 * LIFTOFF DISCRETE, 0 = TRUE
 */
#define CH030_LIFTOFF       BIT5

/**
 * GUIDANCE REFERENCE RELEASE, 0 = TRUE
 */
#define CH030_GRR       BIT6

/**
 * IMU Operate; 0 = TRUE
 */
#define CH030_IMU_OPERATE   BIT9

/**
 * Spacecraft control of Saturn L/V; 1 = IU Control, 0 = S/C Control
 */
#define CH030_SC_CONTROL    BIT10

/**
 * ISS Turn on Request; 0 = TRUE
 */
#define CH030_ISS_TURN_ON_REQUEST    BIT14

/**
 * ISS Temp in limits; 0 = TRUE
 */
#define CH030_ISS_TEMP_IN_LIMITS    BIT15