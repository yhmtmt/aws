#ifndef AWS_STDLIB_H
#ifdef _MSC_VER
#define snprintf sprintf_s
#define snwprintf swprintf_s
#define atoll _atoi64 
#define strtoull _strtoui64
#define chdir _chdir
#endif
#define AWS_STDLIB_H
#endif