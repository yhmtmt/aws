#ifndef _AWS_SERIAL_H_
#define _AWS_SERIAL_H_

#ifndef _WIN32 // for Linux
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#else
#include <Windows.h>
#endif


int enc_cbr(int cbr);

#ifdef _WIN32
#define NULL_SERIAL INVALID_HANDLE_VALUE
#define AWS_SERIAL HANDLE
AWS_SERIAL open_serial(unsigned short port, int cbr);
#else
#define NULL_SERIAL -1
#define AWS_SERIAL int
AWS_SERIAL open_serial(const char * dname, int cbr, bool nonblk = false);
#endif

bool close_serial(AWS_SERIAL h);
int write_serial(AWS_SERIAL, char * buf, int len);
int read_serial(AWS_SERIAL, char * buf, int len);
#endif