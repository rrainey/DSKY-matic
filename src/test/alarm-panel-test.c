#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include "linux/i2c-dev.h"

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
#define LAMP_STBY		    LAMP(2)
#define LAMP_KEY_REL		LAMP(3)
#define LAMP_OPR_ERR		LAMP(4)
#define LAMP_TEMP		    LAMP(7)
#define LAMP_GIMBAL_LOCK	LAMP(8)
#define LAMP_PROG	        LAMP(9)
#define LAMP_RESTART	    LAMP(10)
#define LAMP_TRACKER        LAMP(11)
#define LAMP_ALT            LAMP(12)
#define LAMP_VEL            LAMP(13)

const char *key_name[] = {
    NULL, // entry 0 does not correspond to an S-key.
    "STBY",
    "KEY REL",
    "ENTR",
    "VERB",
    "NOUN",
    "CLR",
    "(minus)",
    "0",
    "(plus)",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "RESET"
};

bool setLamps(int fd, unsigned int state)
{
    char buf[2];
    bool status = true;
    buf[0] = (state >> 8) & 0xff;
    buf[1] = (state & 0xff);
    if(write(fd, buf, 2) != 2) {
        status = false;
    }
    return status;
}

bool checkKeyEvent( int fd_key, 
                    unsigned long *millis, 
                    unsigned char *s_id, 
                    unsigned char *state)
{
    unsigned char buf[6];
    *millis = 0;
    *s_id = 0;
    *state = 0;
    bool status = false;

    if( read(fd_key, buf, 6) == 6 ) {
        *millis = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	//fprintf(stderr, "%ul\n", *millis);
        *s_id = buf[4];
        *state = buf[5];
        status = true;
    }
    return status;
}

int main(int argc, char *argv[])
{
    int fd;
    int fd_key;
    bool reported = false;
    bool reported2 = false;

    struct timespec _500ms;
    _500ms.tv_sec = 0;
    //_500ms.tv_nsec = 5000000L;
    _500ms.tv_nsec = 500 * 1000000L;

    fd = open("/dev/i2c-1", O_RDWR);
    if(fd < 0) {
        fprintf(stderr, "Error opening device\n");
        exit(EXIT_FAILURE);
    }
    if(ioctl(fd, I2C_SLAVE, I2C_ALARM_PANEL_ADDRESS) < 0) {
        fprintf(stderr, "Error setting slave address\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    fd_key = open("/dev/i2c-1", O_RDWR);
    if(fd_key < 0) {
        fprintf(stderr, "Error opening device\n");
        exit(EXIT_FAILURE);
    }
    if(ioctl(fd_key, I2C_SLAVE, I2C_KEYBOARD_ADDRESS) < 0) {
        fprintf(stderr, "Error setting slave address\n");
        close(fd_key);
        exit(EXIT_FAILURE);
    }

    unsigned short lamp = 0x8000;

    while (1) {

        if (!setLamps(fd, lamp)) {
            if (!reported) {
	        reported = true;
            fprintf(stderr, "Error writing (1)\n");
	    }
            //close(fd);
            // exit(EXIT_FAILURE);
        }

        unsigned long millis;
        unsigned char s_id = 1;
        unsigned char state;

        while ( s_id > 0) {
            if (checkKeyEvent(fd_key, &millis, &s_id, &state) ) {
                if (s_id > 0) {
                    printf("%s %s\n", key_name[s_id], state ? "down" : "up");
                }
            }
            else {
		if (!reported2) {
			reported2 = true;
			printf ("error on keyboard poll\n");
		}
	    }
        }

        nanosleep(&_500ms, NULL);

        lamp = lamp >> 1;
        if (lamp == 0x0002) {
            lamp = 0x8000;
        }
    }

    close(fd);
    close(fd_key);
    exit(EXIT_SUCCESS);
}
