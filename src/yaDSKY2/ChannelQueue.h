/*
 * DSKY-matic driver interface for Raspberry Pi OS
 * 
 * Copyright (c) 2020, Riley Rainey
 * 
 * This file is part of the DSKY-matic project. All software in that project
 * is release via the GPL v2 license. See src/SOFTWARE-LICENSE.txt. Hardware and 
 * artistic elements are covered by a separate CC-BY-SA license. 
 *
 * DSKY-matic is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * DSKY-matic is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSKY-matic; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

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
#define BIT16  0100000
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

// see https://www.ibiblio.org/apollo/developer.html

#define CH010_OPERATION_MASK        (15<<11)
#define CH010_OPERATION_M1M2        (11<<11)
#define CH010_OPERATION_V1V2        (10<<11)
#define CH010_OPERATION_N1N2        (9<<11)
#define CH010_OPERATION_DIGIT11     (8<<11)
#define CH010_OPERATION_DIGIT1213   (7<<11) //1+ in BIT11
#define CH010_OPERATION_DIGIT1415   (6<<11) //1- in BIT11
#define CH010_OPERATION_DIGIT2122   (5<<11) //2+ in BIT11
#define CH010_OPERATION_DIGIT2324   (4<<11) //2- in BIT11
#define CH010_OPERATION_DIGIT2531   (3<<11) 
#define CH010_OPERATION_DIGIT3233   (2<<11) //3+ in BIT11
#define CH010_OPERATION_DIGIT3435   (1<<11) //3- in BIT11
#define CH010_OPERATION_LAMPS       (12<<11)
#define CH010_PRIO_DISP_LAMP        BIT1
#define CH010_NO_DAP_LAMP           BIT2
#define CH010_VEL_LAMP              BIT3
#define CH010_NO_ATT_LAMP           BIT4
#define CH010_ALT_LAMP              BIT5
#define CH010_GIMBAL_LOCK_LAMP      BIT6
#define CH010_TRACKER_LAMP          BIT8
#define CH010_PROG_LAMP             BIT9
#define CH010_CCCCC_VALUE(x)        (((x) & (0x1F << 5)>> 5)
#define CH010_DDDDD_VALUE(x)        ((x) & 0x1F)

/*
 * From https://www.ibiblio.org/apollo/Documents/agc_programming_info.pdf
 */

#define CH011_ISS_WARNING            0x001
#define CH011_COMP_ACTY_LAMP         0x002
#define CH011_UPLINK_ACTY_LAMP       0x004
#define CH011_TEMP_CAUTION           0x008
#define CH011_KEYBOARD_RELEASE_LAMP  0x010   // flashes at 1.5 CPS
#define CH011_FLASH_VERB_NOUN_LAMPS  0x020   // flashes at 1.5 CPS see https://www.ibiblio.org/apollo/Documents/acelectroniclmma00acel_0.pdf
#define CH011_OPER_ERROR_LAMP        0x040   // flashes at 1.5 CPS

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