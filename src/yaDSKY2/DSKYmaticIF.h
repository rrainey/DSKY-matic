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

#ifndef DSKYMATICIF_H
#define DSKYMATICIF_H

/**
 * seven bit device address (binary): 011 1000
 */
#define I2C_ALARM_PANEL_ADDRESS 0x38

/**
 * seven bit device address (binary): 011 1001
 */
#define I2C_KEYBOARD_ADDRESS 0x39

#define LAMP(x) (0x8000>>x)
#define LAMP_UPLINK_ACTY	LAMP(0)
#define LAMP_NO_ATT		    LAMP(1)
#define LAMP_STBY		      LAMP(2)
#define LAMP_KEY_REL		  LAMP(3)
#define LAMP_OPR_ERR		  LAMP(4)
#define LAMP_TEMP		      LAMP(7)
#define LAMP_GIMBAL_LOCK	LAMP(8)
#define LAMP_PROG	        LAMP(9)
#define LAMP_RESTART	    LAMP(10)
#define LAMP_TRACKER      LAMP(11)
#define LAMP_ALT          LAMP(12)
#define LAMP_VEL          LAMP(13)

/*
 * DSKY EL Display serial protocol
 * as implemented in Ben Krasnow's EL Display Project
 * 
 * Ben's DSKY_driver_V4seg driver firmware implements a subset of the full DSKY display driver functionality.
 * Notably, non-digit display values are supported by the DSKY, but not by this protocol
 * (see https://www.ibiblio.org/apollo/developer.html).
 * Sometime I may update the driver to implement the full protocol.
 * 
 * The EL Display is controlled by sending a 26-character packet to the EL Display via the USB
 * serial interface:
 * 
 *          [PPNNVV+11111+22222+33333D
 * 
 * 0       ASCII Left Brace "[" packet "start character"
 * 1-2     Prog digits (PP)
 * 3-4     Noun digits (NN)
 * 5-6     Verb digits (VV)
 * 7-12    top line +/-DDDDD
 * 13-18   middle line +/-DDDDD
 * 19-24   bottom line +/-DDDDD
 * 25      various discrete segments (values below)
 * 
 * If the upper bit of any digit is set, the remaining bits are set/reset based on the
 * SEGMENT_ values below. If the upper bit is zero, then the character is interpreted
 * as an ASCII digit and the segments are set accordingly
 */
#define DIGIT_RAW_SEGMENT_CONTROL          0x80

/*
 * Raw segment bit defnitions recognized by DSKY EL Driver when the high-order bit 
 * corresponding to the digit is set to "1" (DIGIT_RAW_SEGMENT_CONTROL)
 * a-g are traditional industry-standard seven-segment names.  MIT used E, A, M, N, K, F, U as
 * names for the same segments.
 * 
 * see https://github.com/rrainey/DSKY_EL_replica/blob/master/firmware%20-%20Arduino/DSKY_driver_V4seg/DSKY_driver_V4seg.ino
 */
#define SEGMENT_a 0x01
#define SEGMENT_b 0x02
#define SEGMENT_c 0x04
#define SEGMENT_d 0x08
#define SEGMENT_e 0x10
#define SEGMENT_f 0x20
#define SEGMENT_g 0x40

// Discrete visual sgements (trailing byte, (#25) in the communications protocol)
// a=COMP ACTY, b=PROG, c=VERB, d=NOUN, e=upper line, f=middle line, g=lower line

#define EL_SEGMENT_COMP_ACTY   0x01
#define EL_SEGMENT_PROG        0x02
#define EL_SEGMENT_VERB        0x04
#define EL_SEGMENT_NOUN        0x08
#define EL_SEGMENT_UPPER       0x10
#define EL_SEGMENT_MIDDLE      0x20
#define EL_SEGMENT_LOWER       0x40

// Sign digit definitions
#define EL_SEGMENT_SIGN_MINUS       SEGMENT_a
#define EL_SEGMENT_SIGN_PLUS_TOP    SEGMENT_b
#define EL_SEGMENT_SIGN_PLUS_BOTTOM SEGMENT_g

/*
 * End of DSKY EL Display serial protocol definitions
 */

class DSKYmaticIF {

private:
    int fdAlarm;
    int fdELDisplay;
    int fdKeyboard;

    // Values received from Channel 010 (octal) output operations
    // Represents the EL Display state and some of the alarm/warning lamps
    unsigned short ch010Values[13];

    // Channel commands are translated into equivalent EL Driver segment
    // directives that will generate the appropriate outputs.  Those values are
    // saved here
    unsigned char prog[2];
    unsigned char noun[2];
    unsigned char verb[2];
    unsigned char upper[5];
    unsigned char middle[5];
    unsigned char lower[5];

    // EL Driver-compatible sign directived for each digit row;
    // Ben's EL Driver defines uses these segments when under raw control: 
    // a = minus segment, b,g = "plus" segments. " ", "-", & "+" will
    // also work, but we'll use the direct segment control in this interface
    // for consistency.
    unsigned char s1placeholder, s2placeholder,  s3placeholder;

    // cached alarm panel lamp states
    unsigned short lampState;

    // "Dirty" flags are set when a change is detected that will require updating the
    // appropriate display hardware.  A "dirty" flag is reset after an update is transmitted.
    bool lampStateDirty;
    bool elDisplayStateDirty;

    /*
     * Experimental: track number of relays tripped each cycle to be able to render
     * appropriate sound effects. This is tracked today, but remains to be
     * fully implemented.
     */
    int relayTripCount;

    bool compActyLamp;

    /**
     * Flash NOUN/VERB digits when commanded by the AGC
     * @see updateFlashingState
     */
    bool flashNounVerb;

    /**
     * Some display elements flash at 1.5 CPS.
     * When "false", these elements should be inhibited.
     * @see updateFlashingState
     */
    bool flashState;

    /**
     * Some display elements flash at 1.5 CPS.
     * Track wall clock time remaining to next on/off transition
     * @see updateFlashingState
     */
    int flashCountdown_ms;

    bool reported2;

public:
    DSKYmaticIF();

    /**
     * Call exactly once to open connections to DSKY-matic Alarm, Keyboard, and EL Display
     */
    int initialize();

    /**
     * Transmits current display state to the EL Display board
     */
    void updateELDisplay();

    /**
     * Transmits current alarm panel state to the Alarm Panel
     */
    void updateAlarmStatusPanel();

    /**
     * Poll Keyboard and generate appropriate AGC Input Channel events.
     */
    void checkKeyboard();

    /**
     * Some displays/lamps on the DSKY flash at 1.5 CPS; to synchronize this flashing across
     * modules, we must track the passing of time manually.  updateFlashingState must be called
     * periodically to keep allow this function to operate properly.
     * @param elapsed_ms elapsed wall clock time since last call
     */
    void updateFlashingState(int elapsed_ms);

    /**
     * Called when AGC Channel events are passed to the application.
     * yaAGC sends these as UDP packets to this application.
     */
    void processIncomingChannel(int channel, int value, int uBit);

private:
    /*
     * Update Alarm panel driver via I2C interface
     */
    bool sendLampState(unsigned int state);

    /*
     * Poll keyboard via I2C interface to determine if a key has been pressed or released
     * 
     * @parm millis relative time of the event measured by the keyboards internal clock
     * @param s_id key code
     * @param state 0 == key release event, 1 == key press event
     */
    bool checkKeyEvent( 
                unsigned long *millis, 
                unsigned char *s_id, 
                unsigned char *state);

    /**
     * Handle channel output potentially altering a lamp relay. Mark the lamp driver for update if
     * the new value is different than the current state.
     */
    void setLampState(int value, unsigned short channelBit, unsigned short driverBit);

    /**
     * Housekeeping to track changes in a specific EL Display digit.
     * Detects when a change actually occurs so that we can limit sending to the harware to
     * only when the display changed.
     */
    void updateIfDigitChanged(unsigned char &oldValue, unsigned short channelValue);

    /**
     * Housekeeping to track changes in a specific EL Display sign segment
     */
    void updateIfPlusSignChanged(unsigned char &oldValue, unsigned short channelValue);

    /**
     * Housekeeping to track changes in a specific EL Display sign segment
     */
    void updateIfMinusSignChanged(unsigned char &oldValue, unsigned short channelValue);

    /**
     * Count number of bits changing in a value.  Used to count the number of relays that are
     * changing state when a digit or lamp changes -- track this in order to be able to generate 
     * relay clicking sound effects.
     * @param oldVal original value
     * @param newVal new value
     * @return tally of the number of changed bits
     */
    unsigned short countChangedBits(unsigned short oldVal, unsigned short newVal);

    void outputKeycode (int keycode);
    void outputPro (int OffOn);

    /*
     * Translate DSKY segment code bits to EL Driver equivalent value;
     */
    unsigned short mapBitsToELDriver(int bits);
};

#endif