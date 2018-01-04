#include "stdafx.h"
#include <iostream>
using namespace std;

#include "aws_serial.h"

int enc_cbr(int cbr)
{
#ifdef _WIN32
	switch(cbr){
	case 110:
		return CBR_110;     //  baud rate
	case 300:
		return CBR_300;     //  baud rate
	case 600:
		return CBR_600;     //  baud rate
	case 1200:
		return CBR_1200;     //  baud rate
	case 2400:
		return CBR_2400;     //  baud rate
	case 4800:
		return CBR_4800;     //  baud rate
	case 9600:
		return CBR_9600;     //  baud rate
	case 14400:
		return CBR_14400;     //  baud rate
	case 19200:
		return CBR_19200;     //  baud rate
	case 38400:
		return CBR_38400;     //  baud rate
	case 56000:
		return CBR_56000;     //  baud rate
	case 57600:
		return CBR_57600;     //  baud rate
	case 115200:
		return CBR_115200;     //  baud rate
	case 128000:
		return CBR_128000;     //  baud rate
	case 256000:
		return CBR_256000;     //  baud rate
	}
#else
	switch(cbr){
	case 110:
		return B110;     //  baud rate
	case 300:
		return B300;     //  baud rate
	case 600:
		return B600;     //  baud rate
	case 1200:
		return B1200;     //  baud rate
	case 2400:
		return B2400;     //  baud rate
	case 4800:
		return B4800;     //  baud rate
	case 9600:
		return B9600;     //  baud rate
	case 19200:
		return B19200;     //  baud rate
	case 38400:
		return B38400;     //  baud rate
	case 57600:
		return B57600;     //  baud rate
	case 115200:
		return B115200;     //  baud rate
	}

#endif
	return -1;
}

#ifdef _WIN32

AWS_SERIAL open_serial(unsigned short port, int cbr)
{
	AWS_SERIAL h = NULL_SERIAL;
	DCB dcb;
	COMMTIMEOUTS timeout;
	if(port <= 0 || port > 256)
		return NULL_SERIAL;
	if(enc_cbr(cbr) < 0)
		return NULL_SERIAL;

	wchar_t com_path[32];
	swprintf(com_path, 32, L"\\\\.\\COM%d", port);
	BOOL fSuccess;

	h = CreateFile( com_path,
		GENERIC_READ | GENERIC_WRITE,
		0,      //  must be opened with exclusive-access
		NULL,   //  default security attributes
		OPEN_EXISTING, //  must use OPEN_EXISTING
		0,      //  not overlapped I/O
		NULL ); //  hTemplate must be NULL for comm devices

	if(h == NULL_SERIAL){
		return NULL_SERIAL;
	}

	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);
	dcb.ByteSize = 8;             //  data size, xmit and rcv
	dcb.BaudRate = enc_cbr(cbr);
	dcb.Parity   = NOPARITY;      //  parity bit
	dcb.StopBits = ONESTOPBIT;    //  stop bit

	fSuccess = SetCommState(h, &dcb);
	if(!fSuccess){
		CloseHandle(h);
		return NULL_SERIAL;
	}

	timeout.ReadIntervalTimeout = 10;
	timeout.ReadTotalTimeoutMultiplier = 0;
	timeout.ReadTotalTimeoutConstant = 10;
	timeout.WriteTotalTimeoutMultiplier = 0;
	timeout.WriteTotalTimeoutConstant = 10;
	fSuccess = SetCommTimeouts(h, &timeout);
	if(!fSuccess){
		CloseHandle(h);
		return NULL_SERIAL;
	}

	return h;
}

#else
AWS_SERIAL open_serial(const char * dname, int cbr, bool nonblk)
{
	AWS_SERIAL h = NULL_SERIAL;
	termios copt;
	if(enc_cbr(cbr) < 0)
		return h;

	h = ::open(dname, O_RDWR | O_NOCTTY /*| (nonblk ? O_NDELAY : 0)| O_NDELAY*/);
	if(h == NULL_SERIAL){
		return h;
	}

	tcgetattr(h, &copt);

	copt.c_cflag = enc_cbr(cbr) | CS8 | CLOCAL | CREAD;	
	copt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	copt.c_iflag &= ~(IGNCR | ICRNL | INLCR);
	copt.c_oflag &= ~(OPOST);
	/*
	cfsetispeed(&copt, enc_cbr(cbr));
	cfsetospeed(&copt, enc_cbr(cbr));
	copt.c_cflag &= ~PARENB;
	copt.c_cflag &= ~CSTOPB;
	copt.c_cflag &= ~CSIZE;
	copt.c_cflag |= CS8;
	copt.c_cc[VMIN] = 0;
	copt.c_cc[VTIME] = 1;
	copt.c_cflag |= (CLOCAL | CREAD);
	*/
	cfmakeraw(&copt);

	tcflush(h, TCIFLUSH);

	if(tcsetattr(h, TCSANOW, &copt) != 0){
		::close(h);
		return NULL_SERIAL;
	}

	return h;
}

#endif


bool close_serial(AWS_SERIAL h)
{
#ifdef _WIN32
	if(CloseHandle(h))
		return true;
#else
	if(::close(h) == 0)
		return true;
#endif
	return false;
}

int write_serial(AWS_SERIAL h, char * buf, int len)
{
  if(h == NULL_SERIAL)
    return 0;
  int len_sent;
#ifdef _WIN32
  if(!WriteFile(h, (LPVOID) buf, (DWORD) len,
		(DWORD*) &len_sent, NULL))
    return -1;
#else
  len_sent = write(h, buf, len);
  /*
    cout << "Write serial: " << len << " bytes." << endl;
    cout.write(buf, len);
    cout << "(";
    for(int i = 0; i < len; i++)
    printf("%02x ", (int) buf[i]);
    cout << ")";
  */
#endif
  
  return len_sent;
}

int read_serial(AWS_SERIAL h, char * buf, int len)
{
  int len_rcvd;
  
#ifdef _WIN32
  COMSTAT stat;
  DWORD err;
  int ninq = 0;
  ClearCommError(h, &err, &stat);
  ninq = stat.cbInQue;
  if(ninq == 0)
    return 0;
  
  if(!ReadFile(h, buf, 
	       (DWORD) len, 
	       (DWORD*) &len_rcvd, NULL)){
    return -1;
  }
#else
  fd_set rd, er;
  timeval tout;
  tout.tv_sec = 0;
  tout.tv_usec = 0;
  FD_ZERO(&rd);
  FD_ZERO(&er);
  FD_SET(h, &rd);
  FD_SET(h, &er);
  
  int res = select(h+1, &rd, NULL, &er, &tout);
  len_rcvd = 0;
  if(res > 0){
    if(FD_ISSET(h, &rd)){
      len_rcvd = read(h, buf, len);
    }else if(FD_ISSET(h, &er)){
      cerr << "Error in read_serial." << endl;
      return -1;
    }
  }else if(res < 0){
    cerr << "Error in read_serial." << endl;
    return -1;
  }else{
    return 0;
  }  
#endif
  return len_rcvd;
}
