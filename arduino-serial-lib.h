//
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//


#ifndef __ARDUINO_SERIAL_LIB_H__
#define __ARDUINO_SERIAL_LIB_H__

#include <stdint.h>   // Standard types
#include <stddef.h>   // Standard types

int serialport_init(const char* serialport, int baud);
int serialport_close(int fd);
int serialport_write_bytes(int fd, const uint8_t* bytes, size_t n_bytes);
int serialport_write(int fd, const char* str);
int serialport_read_bytes(int fd, uint8_t* buf, int n_bytes);
int serialport_flush(int fd);

#endif

