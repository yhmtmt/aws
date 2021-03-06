
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_clock.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_clock.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_clock.h.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef _C_CLOCK_H_
#define _C_CLOCK_H_

#pragma once
#ifdef _WIN32
#include <dmusicc.h>
#else
#include <sys/time.h>
#endif
#define USEC 10L
#define MSEC 10000L
#define SEC 10000000L
#define MNU 600000000L



inline const long long cnvTimeNanoSec(const long long taws)
{
  return taws * 100;
}

inline const long long cnvTimeMicroSec(const long long taws)
{
  return taws / USEC;
}

inline const long long cnvTimeMilliSec(const long long taws)
{
  return taws / MSEC;
}

inline const long long cnvTimeSec(const long long taws)
{
  return taws / SEC;
}

struct tmex: public tm{
  int tm_msec;
};

#ifdef _WIN32
#define mkgmtime _mkgmtime
#else
#define mkgmtime timegm
#endif

void gmtimeex(long long msec, tmex & tm);
long long mkgmtimeex(tmex & tm);
long long mkgmtimeex_tz(tmex & tm, int tzm);

bool decTmStr(char * tmStr, tmex & tm);
const char * getMonthStr(int month);
const char * getWeekStr(int wday);

// class c_clock provides clock object for c_aws.
// Internally, the class holds the start time objtained through APIs in the system,
// and returns time from the start time via get_time() method.
// The time zone is not treated in the class. The time epoch passed to start()
// should be UTC.
class c_clock
{
 private:
  long long m_tcyc;  // m_rate scaled cycle time
  long long m_offset;// offset time (only used in offline mode)
  long long m_tcur;  // current time.
                     // In offline mode, elapsed time from "go" command 
  long long m_delta; // time delta to be corrected.
  int m_delta_adjust;// minimum precision in time delta correction.
  bool m_bonline;    // online mode flag.
  
  enum e_state{
    STOP, RUN, PAUSE
  } m_state;
  
#ifdef _WIN32
  IReferenceClock * m_pclk;
  DWORD m_token;
  HANDLE m_sem;
#else
  timespec m_ts_start;  
#endif
  int m_rate;
 public:
  c_clock(void);
  ~c_clock(void);
  
  bool start(unsigned period = 166667, unsigned delay = 0,
	     long long offset = 0, bool online = true, int rate = 1); // pause or stop to run transition
  void stop(); // run or pause to stop transition
  bool pause();// run to pause transition
  bool restart(); // pause to run transition
  bool step(int cycle);
  bool step(long long tabs);
  
  const long long get_period()
  {
    return m_tcyc;
  }

  const long long get_resolution()
  {
    return m_delta_adjust;
  }
  
  bool is_run(){
    return m_state == RUN;
  }
  
  bool is_stop(){
    return m_state == STOP;
  }
  
  bool is_pause(){
    return m_state == PAUSE;
  }
  
  long long get_time(); // get UTC time
  const long long get_time_from_start()
  {
#ifdef _WIN32
    return m_tcur - m_offset;
#else
    return m_tcur;
#endif
  }
  
  void set_time(tmex & tm); // set new UTC time
  void set_time(long long & t);	// set new UTC time
  void set_time_delta(long long & delta); // adjust time by gradually adding delta
  
  void wait();
};

#endif
