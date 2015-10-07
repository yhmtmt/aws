#ifndef AWS_STDLIB_H
#ifdef _MSC_VER
#define snprintf sprintf_s
#define snwprintf swprintf_s
#define atoll _atoi64 
#define strtoull _strtoui64
#define chdir _chdir
#endif
#define AWS_STDLIB_H

#include "aws_const.h"

// comparison function used in the map 
struct cmp { 
	bool operator () (const char *a,const char *b) const 
	{
		return strcmp(a,b) < 0;
	} 
};

// h2i converts HEX char to integer
inline unsigned char h2i(char h){
	unsigned char i;
	i = h - '0';
	if(i < 10)
		return i;
	i = h - 'A' + 10;
	return i;
}

// box-muller random normal variable
inline double nrand(double u, double s)
{
	double u0 = (double) rand() / (double) RAND_MAX;
	double u1 = (double) rand() / (double) RAND_MAX;
	return s * sqrt(-2.0 * log(u0))*cos(2*PI*u1) + u;
}


// Returns rate of the drive used
 float getDrvUse(const char * path);

#endif