#ifdef _WIN32
#include <Windows.h>
#else 
#include <sys/statvfs.h>
#include <sys/types.h>
#endif
#include <errno.h>
#include <string.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;

#include "aws_stdlib.h"

float getDrvUse(const char * path)
{
#ifdef _WIN32
	//ULARGE_INTEGER FBA, TNB, TNFB;
	unsigned __int64 FBA, TNB, TNFB;
#ifdef UNICODE
	wchar_t wpath[2048];
	mbstowcs(wpath, path, 2048);
	int n;
	n = GetDiskFreeSpaceEx(wpath, (PULARGE_INTEGER)&FBA, (PULARGE_INTEGER)&TNB, (PULARGE_INTEGER)&TNFB);
	if(n == 0){
		wcerr << L"Error GetDiskFreeSpaceEx() " << path << endl;
	}

#else
	n = GetDiskFreeSpaceEx(path, (PULARGE_INTEGER)&FBA, (PULARGE_INTEGER)&TNB, (PULARGE_INTEGER)&TNFB);
	if(n == 0){
		cerr << "Error GetDiskFreeSpaceEx() " << path << endl;
	}
#endif
	if(n == 0){
		cerr << "Error GetDiskFreeSpaceEx() " << path << endl;
	}
	return (float)(1.0 - (double) FBA  / (double) TNB);
#else
	struct statvfs buf;
	int rc =	statvfs(path, &buf);
	if(rc < 0){
	  cerr << "Error statvfs() " <<  strerror(errno)  << endl;
		return 1.;
	}
	return (float)(1.0 - (double)buf.f_bfree / (double)buf.f_blocks);
#endif
}
