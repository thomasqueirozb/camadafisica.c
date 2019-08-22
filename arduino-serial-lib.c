//
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//

#include "arduino-serial-lib.h"

#include <errno.h>   // Error number definitions
#include <fcntl.h>   // File control definitions
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>   // UNIX standard function definitions

// uncomment this to debug reads
//#define SERIALPORTDEBUG

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud) {
    struct termios toptions;
    int fd;

    // fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK);

    if (fd == -1) {
        perror("serialport_init: Unable to open port ");
        return -1;
    }

    // int iflags = TIOCM_DTR;
    // ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    // ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud;  // let you override switch below if needed
    switch (baud) {
        case 4800:
            brate = B4800;
            break;
        case 9600:
            brate = B9600;
            break;
#ifdef B14400
        case 14400:
            brate = B14400;
            break;
#endif
        case 19200:
            brate = B19200;
            break;
#ifdef B28800
        case 28800:
            brate = B28800;
            break;
#endif
        case 38400:
            brate = B38400;
            break;
        case 57600:
            brate = B57600;
            break;
        case 115200:
            brate = B115200;
            break;
    }
    // brate = B230400;
    brate = B4000000;
    // brate = (speed_t) 5000000;
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // DEFAULT
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_cflag |= CRTSCTS;         // enable hardware flow control
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);  // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // make raw
    toptions.c_oflag &= ~OPOST;                           // make raw

    // No delay for BSs, VTs, FFs, TABs, CR0, NL0
    toptions.c_oflag |= TAB0;
    toptions.c_oflag |= BS0;
    toptions.c_oflag |= VT0;
    toptions.c_oflag |= FF0;
    toptions.c_oflag |= CR0;
    toptions.c_oflag |= NL0;

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 0;

    // \r interpreted as \n
    // https://stackoverflow.com/questions/57608755/character-r-transforms-to-n-when-transmitting-through-serial-port-in-c
    toptions.c_iflag &= ~ICRNL;

    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

int serialport_close(int fd) { return close(fd); }

int serialport_write_bytes(int fd, const uint8_t* bytes, size_t n_bytes) {
    ssize_t n;
    size_t bytes_written = 0;

    while (bytes_written < (size_t)n_bytes) {
        n = write(fd, &bytes[bytes_written], n_bytes - bytes_written);
        // if (n == -1) return -1;  // couldn't write
        if (n == -1) continue;  // couldn't write
        if (n == 0) {
            continue;
        }
        bytes_written += n;
#ifdef SERIALPORTDEBUG
        printf("wrote total of: %ld bytes n=%ld\n", bytes_written, n);
#endif
        tcflush(fd, TCIOFLUSH);  // Flush port
    }
    printf("Total bytes written: %ld\n", bytes_written);
    return 0;
}

int serialport_write(int fd, const char* str) {
    int len = strlen(str);
    int n = write(fd, str, len);
    if (n != len) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

int serialport_read_bytes(int fd, uint8_t* buf, int n_bytes) {
    ssize_t n;
    size_t bytes_read = 0;

    while (bytes_read < (size_t)n_bytes) {
        n = read(fd, &buf[bytes_read], n_bytes - bytes_read);
        if (n == -1) return -1;  // couldn't read
        if (n == 0) {
            continue;
        }
#ifdef SERIALPORTDEBUG
        printf("read total: %ld bytes\n", bytes_read);
#endif
        bytes_read += n;
    }
#ifdef SERIALPORTDEBUG
    printf("Total bytes read: %ld\n", bytes_read);
    printf("serialport_read_bytes n_bytes=%d, read(n)=%d\n", n_bytes,
           bytes_read);
#endif
    return 0;
}

int serialport_flush(int fd) {
    sleep(2);  // required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}
