# try to do some autodetecting
UNAME := $(shell uname -s)

ifeq "$(UNAME)" "Linux"
	OS=linux
endif


#################  Common  ##################################################
ERRORS = -Wall -Werror -Wextra
CFLAGS += $(INCLUDES) -O $(ERRORS) -std=gnu99


all: server client
# all: arduino-serial

client: client.o arduino-serial-lib.o
	$(CC) $(CFLAGS) -o client client.o arduino-serial-lib.o $(LIBS)

server: server.o arduino-serial-lib.o
	$(CC) $(CFLAGS) -o server server.o arduino-serial-lib.o $(LIBS)

# arduino-serial: arduino-serial.o arduino-serial-lib.o
# 	$(CC) $(CFLAGS) -o arduino-serial arduino-serial.o arduino-serial-lib.o $(LIBS)

arduino-serial-server: arduino-serial-lib.o
	$(CC) $(CFLAGS) -o arduino-serial-server arduino-serial-server.c arduino-serial-lib.o $(LIBS)

.c.o:
	$(CC) $(CFLAGS) -c $*.c -o $*.o


clean:
	rm server server.o
	rm client client.o
	rm -f $(OBJ) arduino-serial *.o *.a
	rm -f $(OBJ) arduino-serial-server *.o *.a

