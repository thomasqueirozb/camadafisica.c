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
#include <sys/time.h>
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

    int baudrate = 4000000;
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
                }
                strcpy(serialport, optarg);
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
    |           OPEN/READ PHOTO AND GET SIZE          |
    \*************************************************/

    printf("Nome do arquivo: ");
    fflush(stdout);
    FILE* fr_ptr;  // File read pointer
    {
        char filepath[4096];
        scanf("%s", filepath);
        fr_ptr = fopen(filepath, "rb");
    }

    if (fr_ptr == NULL) {
        error("File could not be opened");
        exit(5);
    }

    fseek(fr_ptr, 0L, SEEK_END);          // Go to end of file
    size_t size_payload = ftell(fr_ptr);  // Get size in bytes
    rewind(fr_ptr);                       // Go back to start of file

    if (!quiet) printf("size_payload: %lu\n", size_payload);
    /*************************************************\
    |         HEADER AND PAYLOAD (COUNT EOPs)         |
    \*************************************************/

    uint8_t header[5];

    header[0] = (size_payload >> 24) & 0xFF;
    header[1] = (size_payload >> 16) & 0xFF;
    header[2] = (size_payload >> 8) & 0xFF;
    header[3] = size_payload & 0xFF;

    int ch;
    // Needs to be int unless EOP is the first 0xff not actual EOP
    // on binary files only. Text files work fine if ch is char

    uint8_t EOP_bytes[] = {0xab, 0xcd, 0xef};  // Will break if size<3 TODO:fix
    uint8_t read_bytes[] = {0x00, 0x00, 0x00};

    uint8_t payload[size_payload];
    uint8_t EOPs = 0;

    size_t counter = 0;
    while ((ch = fgetc(fr_ptr)) != EOF ||
           counter < size_payload) {  // size: char==1 byte==uint8_t
        if (EOP_bytes[0] == read_bytes[0] && EOP_bytes[1] == read_bytes[1] &&
            EOP_bytes[2] == read_bytes[2]) {
            EOPs++;
        }
        read_bytes[0] = read_bytes[1];
        read_bytes[1] = read_bytes[2];
        read_bytes[2] = (uint8_t)ch;
        payload[counter] = ch;
        counter++;
    }
    header[4] = EOPs;

    if (!quiet) printf("Start time\n");
    struct timeval stop, start;
    gettimeofday(&start, NULL);

    if (!quiet) printf("Writing header\n");
    serialport_write_bytes(fd, header, 5);
    if (!quiet) printf("Writing payload\n");
    serialport_write_bytes(fd, payload, size_payload);

    uint8_t response[4] = {0};
    if (!quiet) printf("Reading response\n");
    int response_received = serialport_read_bytes(fd, response, 4);

    gettimeofday(&stop, NULL);

    double ms = (double)(stop.tv_sec - start.tv_sec) * 1000 +
                (double)(stop.tv_usec - start.tv_usec) / 1000;
    if (!quiet) printf("Duration MS %'.3f\n", ms);
    if (!quiet) printf("bytes/s %'.3f\n", 1000*(double)size_payload/ms);
    if (!quiet) printf("Response receive status %d\n", response_received);
    if (!quiet) printf("response[0] = 0x%hhx\n", response[0]);
    if (!quiet) printf("response[1] = 0x%hhx\n", response[1]);
    if (!quiet) printf("response[2] = 0x%hhx\n", response[2]);
    if (!quiet) printf("response[3] = 0x%hhx\n", response[3]);

    size_t size_response =
        response[3] | response[2] << 8 | response[1] << 16 | response[0] << 24;
    if (!quiet) printf("size_response = %ld\n", size_response);
    exit(EXIT_SUCCESS);
}
