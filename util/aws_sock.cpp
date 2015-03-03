#include "stdafx.h"
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_sock.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_sock.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_sock.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
using namespace std;
#include "aws_sock.h"

int dump_socket_error()
{
#ifdef _WIN32
	int er = WSAGetLastError();
	wchar_t * s = NULL;
	FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 
		NULL, er,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPWSTR)&s, 0, NULL);
	printf("E%d %S\n", er, s);	
	LocalFree(s);
	return er;
#else
	cout << strerror(errno) << endl;
#endif
}