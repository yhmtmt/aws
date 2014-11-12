#ifndef _AWS_SOCK_H
#define _AWS_SOCK_H
#ifdef _WIN32
#include <Windows.h>
//#include <winsock2.h>
#define MSG_MORE MSG_PARTIAL
#define SD_RECEIVE 0
#define SD_SEND 1
#define SD_BOTH 2
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

#endif


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
