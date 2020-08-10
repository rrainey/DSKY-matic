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

#define LAMP(x) (1>>x)
#define LAMP_UPLINK_ACTY	LAMP(0)
#define LAMP_NO_ATT		LAMP(1)
#define LAMP_STBY		LAMP(2)
#define LAMP_KEY_REL		LAMP(3)
#define LAMP_OPR_ERR		LAMP(4)
#define LAMP_TEMP		LAMP(7)
#define LAMP_GIMBAL_LOCK	LAMP(8)
#define LAMP_PROG	LAMP(9)
#define LAMP_RESTART	LAMP(10)
#define LAMP_TRACKER LAMP(11)
#define LAMP_ALT LAMP(12)
#define LAMP_VEL LAMP(13)

bool setLamps(int fd, unsigned int state)
{
    char buf[10];
    bool status = true;
    buf[0] = (state >> 8) & 0xff;
    buf[1] = (state & 0xff);
    if(write(fd, buf, 2) != 2) {
        status = false;
    }
    return status;

}


int main(int argc, char *argv[])
{
    int fd;
    struct timespec _500ms;
    _500ms.tv_sec = 0;
    _500ms.tv_nsec = 5000000L;
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

    if (!setLamps(fd, 0xffff)) {
        fprintf(stderr, "Error writing (1)\n");
        close(fd);
        exit(EXIT_FAILURE);
    }
    printf("Write success!\n");

    nanosleep(&_500ms, NULL);

    if (!setLamps(fd, 0x0000)) {
        fprintf(stderr, "Error writing (1)\n");
        close(fd);
        exit(EXIT_FAILURE);
    }
    printf("Write success!\n");

    close(fd);
    printf("Done.\n");
    exit(EXIT_SUCCESS);
}
