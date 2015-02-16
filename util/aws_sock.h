#ifndef _AWS_SOCK_H
#define _AWS_SOCK_H
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_sock.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_sock.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_sock.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifdef _WIN32
#include <Windows.h>
//#include <winsock2.h>
#define MSG_MORE MSG_PARTIAL
#else /* for unix */
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#define SOCKET int

#ifndef SOCKET_ERROR 
#define SOCKET_ERROR (-1)
#endif

inline int closesocket(SOCKET s)
{
	return ::close(s);
}


#define SD_RECEIVE 0
#define SD_SEND 1
#define SD_BOTH 2

#endif

inline int set_sock_nb(SOCKET s){
#ifdef _WIN32
	u_long val = 1;
	// if succeeded, NO_ERROR(0L) is returned
	return ioctlsocket(s, FIONBIO, &val);
#else
	int val = 1;
	// if succeeded, zero is returned
	return ioctl(s, FIONBIO, &val);
#endif
}

inline int get_socket_error()
{
#ifdef _WIN32
	return WSAGetLastError();
#else
	return errno;
#endif
}

inline bool ewouldblock(int er){
#ifdef _WIN32
	return er == WSAEWOULDBLOCK;
#else
	return er == EWOULDBLOCK;
#endif
}

inline bool econnreset(int er){
#ifdef _WIN32
	return er == WSAECONNRESET;
#else
	return er == ECONNRESET;
#endif
}

int dump_socket_error();

inline void set_sockaddr_addr(sockaddr_in & addr, const char * str_addr = NULL)
{
	if(str_addr == NULL){
#ifdef _WIN32
		addr.sin_addr.S_un.S_addr = INADDR_ANY;
#else
		addr.sin_addr.s_addr = INADDR_ANY;
#endif
	}else{
#ifdef _WIN32
		addr.sin_addr.S_un.S_addr = inet_addr(str_addr);
#else
		addr.sin_addr.s_addr = inet_addr(str_addr);
#endif
	}
};
#endif
