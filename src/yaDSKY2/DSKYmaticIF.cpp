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

#include "ChannelQueue.h"
#include "DSKYmaticIF.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <linux/i2c-dev.h>

#include <string>
#include <cstring>
#include <memory>

#include "../yaAGC/yaAGC.h"
#include "../yaAGC/agc_engine.h"

#include "yaDSKY2.h"
MainFrame * MainFrame;

struct _keyInfo {
    const char * name;
    int outputCode;
};

/**
 * Key event codes passed to us the keyboard I2C interface
 */
static struct _keyInfo keyMap[] = {
    { NULL, 0}, // entry 0 does not correspond to an S-key.
    {"STBY/PRO", 99},
    {"KEY REL", 25},
    {"ENTR", 28},
    {"VERB", 17},
    {"NOUN", 31},
    {"CLR", 30},
    {"(minus)", 27},
    {"0", 16},
    {"(plus)", 26},
    {"1", 1},
    {"2", 2},
    {"3", 3},
    {"4", 4},
    {"5", 5},
    {"6", 6},
    {"7", 7},
    {"8", 8},
    {"9", 9},
    {"RESET", 18}
};

struct _ch010CodeToDigitMap {
  unsigned char code;
  char value;
};

static struct _ch010CodeToDigitMap digitMap[] = {
  {0,  ' '},
  {21, '0'},
  {3,  '1'},
  {25, '2'},
  {27, '3'},
  {15, '4'},
  {30, '5'},
  {28, '6'},
  {19, '7'},
  {29, '8'},
  {31, '9'}
};

/*
 * See https://www.ibiblio.org/apollo/developer.html
 * for documentation of Channel 010 codes
 */
struct _ch010CodeToELDriver {
  unsigned char code;     // Code sent from Output Channel 010 directive
  unsigned char value;    // Value to be sent to DSKY Driver
};

/**
 * Map AGC-native channel key codes to EL Driver seven segment commands
 * The original DSKY interface actually defines 32 distinct codes -- not just ten decimal digits --
 * all are reproduced here.
 */
struct _ch010CodeToELDriver ch010CodeToSegmentMap[32] = {
    {0, 0},
    {1, SEGMENT_b },
    {2, SEGMENT_c },
    {3, SEGMENT_b | SEGMENT_c },
    {4, SEGMENT_c | SEGMENT_f },
    {5, SEGMENT_b | SEGMENT_c | SEGMENT_f },
    {6, SEGMENT_c | SEGMENT_f },
    {7, SEGMENT_b | SEGMENT_c | SEGMENT_f },
    {8, SEGMENT_g },
    {9, SEGMENT_b | SEGMENT_g },
    {10, SEGMENT_c | SEGMENT_g },
    {11, SEGMENT_b | SEGMENT_c | SEGMENT_g },
    {12, SEGMENT_c | SEGMENT_f | SEGMENT_g },
    {13, SEGMENT_b | SEGMENT_c | SEGMENT_f | SEGMENT_g },
    {14, SEGMENT_c | SEGMENT_f | SEGMENT_g },
    {15, SEGMENT_b | SEGMENT_c | SEGMENT_f | SEGMENT_g },
    {16, SEGMENT_a | SEGMENT_e },
    {17, SEGMENT_a | SEGMENT_b | SEGMENT_e },
    {18, SEGMENT_a | SEGMENT_c },
    {19, SEGMENT_a | SEGMENT_b | SEGMENT_c },
    {20, SEGMENT_a | SEGMENT_c | SEGMENT_d | SEGMENT_e | SEGMENT_f  },
    {21, SEGMENT_a | SEGMENT_b | SEGMENT_c | SEGMENT_d | SEGMENT_e | SEGMENT_f  },
    {22, SEGMENT_a | SEGMENT_c | SEGMENT_d | SEGMENT_f },
    {23, SEGMENT_a | SEGMENT_b | SEGMENT_c | SEGMENT_d | SEGMENT_f },
    {24, SEGMENT_a | SEGMENT_d | SEGMENT_e | SEGMENT_g },
    {25, SEGMENT_a | SEGMENT_b | SEGMENT_d | SEGMENT_e | SEGMENT_g },
    {26, SEGMENT_a | SEGMENT_c | SEGMENT_d | SEGMENT_g },
    {27, SEGMENT_a | SEGMENT_b | SEGMENT_c | SEGMENT_d | SEGMENT_g },
    {28, SEGMENT_a | SEGMENT_c | SEGMENT_d | SEGMENT_e | SEGMENT_f | SEGMENT_g },
    {29, SEGMENT_a | SEGMENT_b | SEGMENT_c | SEGMENT_d | SEGMENT_e | SEGMENT_f | SEGMENT_g },
    {30, SEGMENT_a | SEGMENT_c | SEGMENT_d | SEGMENT_f | SEGMENT_g },
    {31, SEGMENT_a | SEGMENT_b | SEGMENT_c | SEGMENT_d | SEGMENT_f | SEGMENT_g }

};

static int StartupDelay ;
static int DebugCounterMode;
static int DebugCounterReg, DebugCounterInc, DebugCounterWhich;
// TestUplink is set when we want to test the digital uplink by emitting
// keycodes on the digital uplink rather than on the DSKY channels.
static int TestUplink;
static int ServerSocket;
static bool ProceedPressed;
static int IoErrorCount, Last11;

DSKYmaticIF::DSKYmaticIF() {

  fdAlarm = -1;
  fdELDisplay = -1;
  fdKeyboard = -1;

  // initialize EL Display Segment map to enable the
  // direct control of each segment in the seven segment display (with all segments initially off)
  unsigned char init = DIGIT_RAW_SEGMENT_CONTROL;

  memset(&ch010Values, 0, sizeof(ch010Values));

  memset(prog, init, 2);
  memset(noun, init, 2);
  memset(verb, init, 2);
  memset(upper, init, 5);
  memset(middle, init, 5);
  memset(lower, init, 5);

  s1placeholder = init;
  s2placeholder = init;
  s3placeholder = init;

  lampState = 0;
  lampStateDirty = true;
  elDisplayStateDirty = true;
  relayTripCount = 0;

  compActyLamp = false;

  flashNounVerb = false;
  flashState = true;
  flashCountdown_ms = 750;
  reported2 = false;
}

void DSKYmaticIF::updateFlashingState(int elapsed_ms) {

  flashCountdown_ms -= elapsed_ms;

  if (flashCountdown_ms <=0) {
    flashCountdown_ms = 750 + flashCountdown_ms;
    flashState = !flashState;

    // TODO: track number of relays affected by each transition.
    // "punt" for now, assuming the NOUN and VERB digits would
    // be illuminated:
    relayTripCount += 14;
  }
}

void DSKYmaticIF::updateAlarmStatusPanel() {
  if (lampStateDirty && fdAlarm >= 0) {
    sendLampState(lampState);
    lampStateDirty = false;
  }
}

bool DSKYmaticIF::sendLampState(unsigned int state) {
  char buf[2];
  bool status = true;

  /*
   * Inhibit any flashing lamps marked for illumination
   */
  if (!flashState) {
    state &= ~(LAMP_KEY_REL | LAMP_OPR_ERR);
  }

  buf[0] = (state >> 8) & 0xff;
  buf[1] = (state & 0xff);
  if (write(fdAlarm, buf, 2) != 2) {
    status = false;
  }
  return status;
}

bool DSKYmaticIF::checkKeyEvent(
    unsigned long *millis,
    unsigned char *s_id,
    unsigned char *state) {
  unsigned char buf[6];
  *millis = 0;
  *s_id = 0;
  *state = 0;
  bool status = false;

  if (read(fdKeyboard, buf, 6) == 6) {
    *millis = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    //fprintf(stderr, "%ul\n", *millis);
    *s_id = buf[4];
    *state = buf[5];
    status = true;
  }
  return status;
}

int DSKYmaticIF::initialize() {
  bool reported = false;
  bool reported2 = false;

  termios old_tios;
  speed_t speed;
  int flags;

  struct timespec _500ms;
  _500ms.tv_sec = 0;
  //_500ms.tv_nsec = 5000000L;
  _500ms.tv_nsec = 500 * 1000000L;

  // '/dev/ttyAMA0'

  flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif

  fdELDisplay = open("/dev/ttyUSB0", flags);
  if (fdELDisplay < 0) {
    fprintf(stderr, "Error opening EL Display USB connection\n");
    exit(EXIT_FAILURE);
  }

  tcgetattr(fdELDisplay, &old_tios);
  memset(&old_tios, 0, sizeof(struct termios));
  speed = B9600;

  /* Set the baud rate on the usbserial interface */
  if ((cfsetispeed(&old_tios, speed) < 0) ||
      (cfsetospeed(&old_tios, speed) < 0)) {
    close(fdELDisplay);
    fdELDisplay = -1;
    return -1;
  }

  fdAlarm = open("/dev/i2c-1", O_RDWR);
  if (fdAlarm < 0) {
    fprintf(stderr, "Error opening device\n");
    exit(EXIT_FAILURE);
  }
  if (ioctl(fdAlarm, I2C_SLAVE, I2C_ALARM_PANEL_ADDRESS) < 0) {
    fprintf(stderr, "Error setting slave address\n");
    close(fdAlarm);
    exit(EXIT_FAILURE);
  }

  fdKeyboard = open("/dev/i2c-1", O_RDWR);
  if (fdKeyboard < 0) {
    fprintf(stderr, "Error opening device\n");
    exit(EXIT_FAILURE);
  }
  if (ioctl(fdKeyboard, I2C_SLAVE, I2C_KEYBOARD_ADDRESS) < 0) {
    fprintf(stderr, "Error setting slave address\n");
    close(fdKeyboard);
    exit(EXIT_FAILURE);
  }

  return 0;
}

/*
 * borrowed from yaDSKY2.cpp
 */
void DSKYmaticIF::outputKeycode(int Keycode)
{
  unsigned char Packet[4];
  int j;
  if (MainFrame->scriptFileOpen)
    return;
  MainFrame->record(015, Keycode);
  if (ServerSocket != -1) {
    if (TestUplink) {
      // In this case, we communicate keycodes to the AGC via the digital
      // uplink rather than through the normal DSKY input channel.
      Keycode &= 037;
      Keycode |= ((Keycode << 10) | ((Keycode ^ 037) << 5));
      FormIoPacket(0173, Keycode, Packet);
    }
    else
      FormIoPacket(015, Keycode, Packet);
    j = send(ServerSocket, (const char *)Packet, 4, MSG_NOSIGNAL);
    if (j == -1 && (errno == EPIPE)) {
      close(ServerSocket);
      ServerSocket = -1;
	  }
  }
}

void
DSKYmaticIF::outputPro (int OffOn)
{
  unsigned char Packet[8];
  int j;
  if (MainFrame->scriptFileOpen)
    return;
  MainFrame->record (032, OffOn ? 020000 : 0);
  if (ServerSocket != -1)
    {
      // First, create the mask which will tell the CPU to only pay attention to
      // bit 14 of the channel (032).
      FormIoPacket (0432, 020000, Packet);
      // Next, generate the data itself.
      if (OffOn)
	OffOn = 020000;
      FormIoPacket (032, OffOn, &Packet[4]);
      // And, send it all.
      j = send (ServerSocket, (const char *) Packet, 8, MSG_NOSIGNAL);
      if (j == SOCKET_ERROR && SOCKET_BROKEN)
	{
	  close (ServerSocket);
	  ServerSocket = -1;
	}
    }
}

void DSKYmaticIF::checkKeyboard() {
  bool looping = true;
  unsigned long millis;
  unsigned char state;
  unsigned char s_id;

  while ( looping && fdKeyboard >= 0 ) {

    if (checkKeyEvent(&millis, &s_id, &state)) {
      // Key down event?  prpoagate it to the AGC
      if ((s_id > 0) && (state != 0)) {
        // printf("%s %s\n", key_name[s_id], state ? "down" : "up");
        switch (keyMap[s_id].outputCode) {
          case 18:  // RESET
          case 25:  // KEY REL
          case 26:  // PLUS
          case 27:  // MINUS
          case 28:  // ENTR
          case 30:  // CLR
            outputKeycode (keyMap[s_id].outputCode);
            break;

          case 16:  // "0"
            if (DebugCounterMode) {
              if (DebugCounterWhich) {
                DebugCounterInc = 0;
              } else {
                DebugCounterReg = (DebugCounterReg * 8) + 0;
              }
            } else {
              outputKeycode(16);
            }
            break;

          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 31:
            if (DebugCounterMode)
            {
              DebugCounterWhich = 1;
              DebugCounterInc = 0;
            }
            else {
              outputKeycode (keyMap[s_id].outputCode);
            }
          break;

          case 99:
            if (DebugCounterMode) {
              int j;
              unsigned char Packet[4];
              if (DebugCounterReg < 032 || DebugCounterReg > 060)
                return;
              if (DebugCounterInc < 0 || DebugCounterInc > 6)
                return;
              Packet[0] = 0x10 | ((DebugCounterReg >> 3) & 0x0f);
              Packet[1] = 0x40 | ((DebugCounterReg & 7) << 3);
              Packet[2] = 0x80;
              Packet[3] = 0xC0 | (DebugCounterInc & 7);
              //printf ("Reg=%02o Inc=%o Packet=%02x %02x %02x %02x\n",
              //	      DebugCounterReg, DebugCounterInc,
              //	      Packet[0], Packet[1], Packet[2], Packet[3]);
              if (ServerSocket != -1) {
                j = send(ServerSocket, (const char *)Packet, 4, MSG_NOSIGNAL);

                if (j == -1 && (errno == EPIPE)) {
                  if (!DebugMode)
                    printf("Removing socket %d\n", ServerSocket);
                  close(ServerSocket);
                  ServerSocket = -1;
                }
              }
            } else {
              // Press.
              outputPro(0);
              ProceedPressed = true;
            }
            break;
        }
      } else {
        looping = false;
      }

    } else {
      if (!reported2) {
        reported2 = true;
        printf("error on keyboard poll\n");
      }
    }
  }
}

void DSKYmaticIF::updateELDisplay() {

  if (elDisplayStateDirty) {
    char buffer[32];

    int x = Last11 & 2;

    unsigned char blank[2];
    blank[0] = DIGIT_RAW_SEGMENT_CONTROL;
    blank[1] = DIGIT_RAW_SEGMENT_CONTROL;

    char extra = DIGIT_RAW_SEGMENT_CONTROL |
                EL_SEGMENT_UPPER | EL_SEGMENT_MIDDLE | EL_SEGMENT_LOWER |
                EL_SEGMENT_PROG | EL_SEGMENT_VERB | EL_SEGMENT_NOUN |
                (x ? EL_SEGMENT_COMP_ACTY : 0);
    
    unsigned char start = '[';

    /*
     * NOUN and VERB digits are normally illuminated. 
     * Under some conditions, these digits will flash at 1.5 CPS (750ms on, 750ms off)
     * See this Apollo 11 TV transmission https://www.youtube.com/watch?v=22adjMrYl0E 
     * for an example of this in action.
     */
    if (flashNounVerb && !flashState) {
      snprintf(buffer, sizeof(buffer), "%c%2.2s%2.2s%2.2s%c%5.5s%c%5.5s%c%5.5s%c",
        start,
        prog,
        blank,
        blank,
        s1placeholder,
        upper,
        s2placeholder,
        middle,
        s3placeholder,
        lower,
        extra);
      }
      else {
        snprintf(buffer, sizeof(buffer), "%c%2.2s%2.2s%2.2s%c%5.5s%c%5.5s%c%5.5s%c",
          start,
          prog,
          verb,
          noun,
          s1placeholder,
          upper,
          s2placeholder,
          middle,
          s3placeholder,
          lower,
          extra);
      }

      if (fdELDisplay >= 0) {
        write(fdELDisplay, buffer, strlen(buffer));
      }

      elDisplayStateDirty = false;
  }
}

void DSKYmaticIF::processIncomingChannel(int channel, int value, int uBit) {
 switch(channel) {
   case 010:
    {
      unsigned short op = value & CH010_OPERATION_MASK;
      unsigned short digits = value & 0x3ff; // low-order ten-bits, CCCCCDDDDD

      switch (op) {

        case CH010_OPERATION_M1M2:
          updateIfDigitChanged(prog[0], digits >> 5);
          updateIfDigitChanged(prog[1], digits & 0x1F);
          break;
        case CH010_OPERATION_V1V2:
          updateIfDigitChanged(verb[0], digits >> 5);
          updateIfDigitChanged(verb[1], digits & 0x1F);
          break;
        case CH010_OPERATION_N1N2:
          updateIfDigitChanged(noun[0], digits >> 5);
          updateIfDigitChanged(noun[1], digits & 0x1F);
          break;
        case CH010_OPERATION_DIGIT11:
          updateIfDigitChanged(upper[0], digits & 0x1F);
          break;
        case CH010_OPERATION_DIGIT1213: //1+ in BIT11
          updateIfDigitChanged(upper[1], digits >> 5);
          updateIfDigitChanged(upper[2], digits & 0x1F);
          updateIfPlusSignChanged(s1placeholder, value);
          break;
        case CH010_OPERATION_DIGIT1415: //1- in BIT11
          updateIfDigitChanged(upper[3], digits >> 5);
          updateIfDigitChanged(upper[4], digits & 0x1F);
          updateIfMinusSignChanged(s1placeholder, value);
          break;
        case CH010_OPERATION_DIGIT2122: //2+ in BIT11
          updateIfDigitChanged(middle[0], digits >> 5);
          updateIfDigitChanged(middle[1], digits & 0x1F);
          updateIfPlusSignChanged(s2placeholder, value);
          break;
        case CH010_OPERATION_DIGIT2324: //2- in BIT11
          updateIfDigitChanged(middle[2], digits >> 5);
          updateIfDigitChanged(middle[3], digits & 0x1F);
          updateIfMinusSignChanged(s2placeholder, value);
          break;
          break;
        case CH010_OPERATION_DIGIT2531: 
          updateIfDigitChanged(middle[4], digits >> 5);
          updateIfDigitChanged(lower[0], digits & 0x1F);
          break;
        case CH010_OPERATION_DIGIT3233: //3+ in BIT11
          updateIfDigitChanged(lower[1], digits >> 5);
          updateIfDigitChanged(lower[2], digits & 0x1F);
          updateIfPlusSignChanged(s3placeholder, value);
          break;
        case CH010_OPERATION_DIGIT3435: //3- in BIT11
          updateIfDigitChanged(lower[3], digits >> 5);
          updateIfDigitChanged(lower[4], digits & 0x1F);
          updateIfMinusSignChanged(s3placeholder, value);
          break;
        case CH010_OPERATION_LAMPS:
          // CH010_PRIO_DISP_LAMP not present on DSKY-matic
          // CH010_NO_DAP_LAMP not present on DSKY-matic
          setLampState(value, CH010_VEL_LAMP, LAMP_VEL);
          setLampState(value, CH010_NO_ATT_LAMP, LAMP_NO_ATT);
          setLampState(value, CH010_ALT_LAMP, LAMP_ALT);
          setLampState(value, CH010_GIMBAL_LOCK_LAMP, LAMP_GIMBAL_LOCK);
          setLampState(value, CH010_TRACKER_LAMP, LAMP_TRACKER);
          setLampState(value, CH010_PROG_LAMP, LAMP_PROG);
          break;
      }
      // save this channel command for comparison in future operations
      ch010Values[op >> 11] = digits;
    }
    break;

  case 011:
    {
      compActyLamp = (value & CH011_COMP_ACTY_LAMP);
      setLampState(value, CH011_UPLINK_ACTY_LAMP, LAMP_UPLINK_ACTY);
      setLampState(value, CH011_TEMP_CAUTION, LAMP_TEMP);

      // These flash a 1.5 CPS; flashing controlled elsewhere
      setLampState(value, CH011_KEYBOARD_RELEASE_LAMP, LAMP_KEY_REL);
      flashNounVerb = (value & CH011_FLASH_VERB_NOUN_LAMPS);
      setLampState(value, CH011_OPER_ERROR_LAMP, LAMP_OPR_ERR);
    }
    break;
 }
}

/*
 * Translate an AGC Channel 010 representation of a seven segment digit to the
 * EL Driver-compabile equivalent.
 */
unsigned short DSKYmaticIF::mapBitsToELDriver(int bits) {
  return ch010CodeToSegmentMap[bits].value | DIGIT_RAW_SEGMENT_CONTROL;
}

/*
 * Detect a changing EL display digit and mark it for output to the EL Driver if so.
 */
void DSKYmaticIF::updateIfDigitChanged(unsigned char &oldValue, unsigned short channelValue) {
  unsigned char newValue = mapBitsToELDriver(channelValue);
  if (oldValue != newValue) {
    // Experimental: track number of relays tripped in this cycle
    // TODO: cound actual number of segments changed
    relayTripCount += countChangedBits(oldValue, newValue);
    oldValue = newValue;
    elDisplayStateDirty = true;
  }
}

void DSKYmaticIF::updateIfMinusSignChanged(unsigned char &oldValue, unsigned short channelValue) {
  unsigned char newValue = 0;
  if (channelValue & BIT11) {
    newValue = oldValue | EL_SEGMENT_SIGN_MINUS;
  }
  else {
    newValue = oldValue & ~EL_SEGMENT_SIGN_MINUS;
  }
  if (oldValue != newValue) {
    // Experimental: track number of relays tripped in this cycle
    // TODO: cound actual number of segments changed
    relayTripCount += countChangedBits(oldValue, newValue);
    oldValue = newValue;
    elDisplayStateDirty = true;
  }
}

void DSKYmaticIF::updateIfPlusSignChanged(unsigned char &oldValue, unsigned short channelValue) {
  unsigned char 
  newValue = oldValue;
  if (channelValue & BIT11) {
    newValue = oldValue | (EL_SEGMENT_SIGN_PLUS_TOP | EL_SEGMENT_SIGN_PLUS_BOTTOM);
  }
  else {
    newValue = oldValue & ~(EL_SEGMENT_SIGN_PLUS_TOP | EL_SEGMENT_SIGN_PLUS_BOTTOM);
  }

  if (oldValue != newValue) {
    // Experimental: track number of relays tripped in this cycle
    // TODO: cound actual number of segments changed
    relayTripCount += countChangedBits(oldValue, newValue);
    oldValue = newValue;
    elDisplayStateDirty = true;
  }
}

void DSKYmaticIF::setLampState(int value, unsigned short channelBit, unsigned short driverBit) {
  // new value different than previous?
  if ((value & channelBit) != (ch010Values[12] & channelBit)) {
    lampState = lampState & (~ driverBit);
    if (value & channelBit) {
      lampState |= driverBit;
    }
    lampStateDirty = true;
    // Experimental: track number of relays tripped in this cycle
    ++relayTripCount;
  }
}

unsigned short DSKYmaticIF::countChangedBits(unsigned short oldVal, unsigned short newVal) 
{ 
  unsigned short n = oldVal ^ newVal;
  unsigned short count = 0; 
  while (n != 0) { 
    count += n & 1; 
    n >>= 1; 
  } 
  return count; 
} 
