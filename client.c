/*
 * arduino-serial
 * --------------
 *
 * A simple command-line example program showing how a computer can
 * communicate with an Arduino board. Works on any POSIX system (Mac/Unix/PC)
 *
 *
 * Compile with something like:
 *   gcc -o arduino-serial arduino-serial-lib.c arduino-serial.c
 * or use the included Makefile
 *
 * Mac: make sure you have Xcode installed
 * Windows: try MinGW to get GCC
 *
 *
 * Originally created 5 December 2006
 * 2006-2013, Tod E. Kurt, http://todbot.com/blog/
 *
 *
 * Updated 8 December 2006:
 *  Justin McBride discovered B14400 & B28800 aren't in Linux's termios.h.
 *  I've included his patch, but commented out for now.  One really needs a
 *  real make system when doing cross-platform C and I wanted to avoid that
 *  for this little program. Those baudrates aren't used much anyway. :)
 *
 * Updated 26 December 2007:
 *  Added ability to specify a delay (so you can wait for Arduino Diecimila)
 *  Added ability to send a binary byte number
 *
 * Update 31 August 2008:
 *  Added patch to clean up odd baudrates from Andy at hexapodia.org
 *
 * Update 6 April 2012:
 *  Split into a library and app parts, put on github
 *
 * Update 20 Apr 2013:
 *  Small updates to deal with flushing and read backs
 *  Fixed re-opens
 *  Added --flush option
 *  Added --sendline option
 *  Added --eolchar option
 *  Added --timeout option
 *  Added -q/-quiet option
 *
 */

#include <getopt.h>
#include <stdio.h>  // Standard input/output definitions
#include <stdlib.h>
#include <string.h>  // String function definitions
#include <unistd.h>  // for usleep()

#include "arduino-serial-lib.h"

void usage(void) {
    printf(
        "Usage: arduino-serial -b <bps> -p <serialport> [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  -h, --help                 Print this help message\n"
        "  -b, --baud=baudrate        Baudrate (bps) of Arduino (default "
        "9600)\n"
        "  -p, --port=serialport      Serial port Arduino is connected to\n"
        "  -s, --send=string          Send string to Arduino\n"
        "  -S, --sendline=string      Send string with newline to Arduino\n"
        "  -i  --stdinput             Use standard input\n"
        "  -r, --receive              Receive string from Arduino & print it "
        "out\n"
        "  -n  --num=num              Send a number as a single byte\n"
        "  -F  --flush                Flush serial port buffers for fresh "
        "reading\n"
        "  -d  --delay=millis         Delay for specified milliseconds\n"
        "  -e  --eolchar=char         Specify EOL char for reads (default "
        "'\\n')\n"
        "  -t  --timeout=millis       Timeout for reads in millisecs (default "
        "5000)\n"
        "  -q  --quiet                Don't print out as much info\n"
        "\n"
        "Note: Order is important. Set '-b' baudrate before opening port'-p'. "
        "\n"
        "      Used to make series of actions: '-d 2000 -s hello -d 100 -r' \n"
        "      means 'wait 2secs, send 'hello', wait 100msec, get reply'\n"
        "\n");
    exit(EXIT_SUCCESS);
}

void error(char* msg) {
    fprintf(stderr, "%s\n", msg);
    exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        usage();
    }

    /*************************************************\
    |         DEFAULT CONFIGURATION VARIABLES         |
    \*************************************************/

    const int buf_max = 256;
    /* char serialport[buf_max]; */
    int fd = -1;
    char quiet = 0;
    char eolchar = '\n';
    int timeout = 5000;
    char buf[buf_max];
    int rc, n;

    int baudrate = 115200;
    char serialport[] = "/dev/ttyACM0";

    /* parse options */
    int option_index = 0, opt;
    static struct option loptions[] = {{"help", no_argument, 0, 'h'},
                                       {"port", required_argument, 0, 'p'},
                                       {"baud", required_argument, 0, 'b'},
                                       {"send", required_argument, 0, 's'},
                                       {"sendline", required_argument, 0, 'S'},
                                       {"stdinput", no_argument, 0, 'i'},
                                       {"receive", no_argument, 0, 'r'},
                                       {"flush", no_argument, 0, 'F'},
                                       {"num", required_argument, 0, 'n'},
                                       {"delay", required_argument, 0, 'd'},
                                       {"eolchar", required_argument, 0, 'e'},
                                       {"timeout", required_argument, 0, 't'},
                                       {"quiet", no_argument, 0, 'q'},
                                       {NULL, 0, 0, 0}};

    while (1) {
        opt = getopt_long(argc, argv, "hp:b:s:S:i:rFn:d:qe:t:", loptions,
                          &option_index);
        if (opt == -1) break;
        switch (opt) {
            case '0':
                break;
            case 'q':
                quiet = 1;
                break;
            case 'e':
                eolchar = optarg[0];
                if (!quiet) printf("eolchar set to '%c'\n", eolchar);
                break;
            case 't':
                /* OPTARG usage
                 * ------------
                 *
                 * First 10 characters of string converted to int
                 * timeout = int(optarg[:10])
                 * NULL = optarg[10:]
                 */
                timeout = strtol(optarg, NULL, 10);
                if (!quiet) printf("timeout set to %d millisecs\n", timeout);
                break;
            case 'd':
                n = strtol(optarg, NULL, 10);
                if (!quiet) printf("sleep %d millisecs\n", n);
                usleep(n * 1000);  // sleep milliseconds
                break;
            case 'h':
                usage();
                break;
            case 'b':
                baudrate = strtol(optarg, NULL, 10);
                break;

            case 'p':
                if (fd != -1) {
                    serialport_close(fd);
                    if (!quiet) printf("closed port %s\n", serialport);
                    fd = -1;
                }
                strcpy(serialport, optarg);
                // fd = serialport_init(optarg, baudrate);
                // if (fd == -1) error("couldn't open port");
                // if (!quiet) printf("opened port %s\n", serialport);
                // serialport_flush(fd);
                break;

            case 'n':
                if (fd == -1) error("serial port not opened");
                n = strtol(optarg, NULL, 10);  // convert string to number
                printf("n: %d\n", (uint8_t)n);
                rc = serialport_write_byte(fd, (uint8_t)n);
                if (rc == -1) error("error writing");
                break;
            case 'S':
            case 's':
                if (fd == -1) error("serial port not opened");
                sprintf(buf, (opt == 'S' ? "%s\n" : "%s"), optarg);

                if (!quiet) printf("send string:%s\n", buf);
                rc = serialport_write(fd, buf);
                if (rc == -1) error("error writing");
                break;
            case 'i':
                rc = -1;
                if (fd == -1) error("serial port not opened");
                while (fgets(buf, buf_max, stdin)) {
                    if (!quiet) printf("send string:%s\n", buf);
                    rc = serialport_write(fd, buf);
                }
                if (rc == -1) error("error writing");
                break;
            case 'r':
                if (fd == -1) error("serial port not opened");
                memset(buf, 0, buf_max);  // Copy (buf_max)x 0's into buf

                serialport_read_until(fd,        // device
                                      buf,       // buffer
                                      eolchar,   // end of line char
                                      buf_max,   // maximum buffer length
                                      timeout);  // timeout
                if (!quiet) printf("read string:");
                printf("%s\n", buf);
                break;
            case 'F':
                if (fd == -1) error("serial port not opened");
                if (!quiet) printf("flushing receive buffer\n");
                serialport_flush(fd);
                break;
        }
    }

    /*************************************************\
    |               OPEN AND FLUSH PORT               |
    \*************************************************/

    fd = serialport_init(serialport, baudrate);
    if (fd == -1) {
        error("Couldn't open port");
        return 5;
    }

    if (!quiet) printf("Opened port %s\n", serialport);
    serialport_flush(fd);  // Flush port (needed to clean buffer); Needs to
                           // sleep for 2 seconds (TODO: look into it)
    if (!quiet) printf("Port %s flushed\n", serialport);

    /*************************************************\
    |                  RECEIVE HEADER                 |
    \*************************************************/
    uint8_t header[5] = {0};
    if (!quiet) printf("Reading header\n");
    int header_received = serialport_read_bytes(fd, header, 5);
    if (!quiet) printf("Header receive status %d\n", header_received);

    /*************************************************\
    |                 RECEIVE PAYLOAD                 |
    \*************************************************/
    size_t size_payload =
        header[3] | header[2] << 8 | header[1] << 16 | header[0] << 24;
    if (!quiet) printf("header[0] = 0x%hhx\n", header[0]);
    if (!quiet) printf("header[1] = 0x%hhx\n", header[1]);
    if (!quiet) printf("header[2] = 0x%hhx\n", header[2]);
    if (!quiet) printf("header[3] = 0x%hhx\n", header[3]);
    if (!quiet) printf("header[4] = 0x%hhx\n", header[4]);

    if (!quiet) printf("size_payload: %lu\n", size_payload);
    uint8_t payload[size_payload];
    memset(payload, 0, size_payload);

    if (!quiet) printf("Reading payload\n");
    int payload_received = serialport_read_bytes(fd, payload, size_payload);
    if (!quiet) printf("Payload receive status %d\n", payload_received);

    if (!quiet) printf("Writing response\n");
    serialport_write_bytes(fd, header, 4);

    FILE* fw_ptr;  // File write pointer
    fw_ptr = fopen("received.bin", "wb+");
    if (!quiet) printf("Writing binary file\n");
    fwrite(payload, 1, size_payload, fw_ptr);

    // // DEBUG
    // for (size_t ch = 0; ch < size_payload; ch++) {
    //     if (!quiet) printf("byte %ld = 0x%02x\n", ch, (uint8_t)payload[ch]);
    //     if (ch == 500) return 200;
    //     // printf("%c\n", payload[ch]);
    //     // // printf("%c", ch);
    //     // fprintf(fw_ptr, "%c", payload[ch]);
    // }

    exit(EXIT_SUCCESS);
}
