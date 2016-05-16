#include <time.h>
#include <stdlib.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <iostream>

using namespace std;

#include "c_clock.h"
#include "aws_stdlib.h"

int main(int argc, char ** argv)
{
  if(argc != 2){
    cout << "Usage: t2str <aws time>" << endl;
    return 0;
  }

  long long t = atoll(argv[1]);
  t /= MSEC;
  tmex tm;
  gmtimeex(t, tm);
  printf("[%s %s %02d %02d:%02d:%02d.%03d %d] ", 
	   getWeekStr(tm.tm_wday), 
	   getMonthStr(tm.tm_mon),
	   tm.tm_mday,
	   tm.tm_hour,
	   tm.tm_min,
	   tm.tm_sec,
	   tm.tm_msec,
	   tm.tm_year + 1900);

  return true;
};
