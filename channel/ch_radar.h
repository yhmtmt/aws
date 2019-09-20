#ifndef CH_RADAR_H
#define CH_RADAR_H
// Copyright(c) 2019 Yohei Matsumoto, All right reserved. 

// ch_radar.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_radar.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_radar.h.  If not, see <http://www.gnu.org/licenses/>.


#include "ch_base.h"
#include "../filter/f_radar_srcs/socketutil.h"
#include "../filter/f_radar_srcs/garminxhd.h"

enum RadarControlState {
  RCS_OFF = -1,
  RCS_MANUAL = 0,
  RCS_AUTO_1,
  RCS_AUTO_2,
  RCS_AUTO_3,
  RCS_AUTO_4,
  RCS_AUTO_5,
  RCS_AUTO_6,
  RCS_AUTO_7,
  RCS_AUTO_8,
  RCS_AUTO_9
};

enum radar_command_id{
  RC_TXOFF, RC_TXON, RC_RANGE,
  RC_BEARING_ALIGNMENT,
  RC_NO_TRANSMIT_START,
  RC_NO_TRANSMIT_END,  
  RC_GAIN,
  RC_SEA,
  RC_RAIN,
  RC_INTERFERENCE_REJECTION,
  RC_SCAN_SPEED,
  RC_TIMED_IDLE,
  RC_TIMED_RUN,
  RC_IMG,
  RC_NONE
};

struct radar_command{
  radar_command_id id;
  int val;
  RadarControlState state;

radar_command(const radar_command_id _id, const int _val, const RadarControlState _state):
  id(_id), val(_val), state(_state)
  {
  }    
};


class ch_radar_state: public ch_base
{
 protected:
  int m_state, m_range, m_gain, m_gain_state, m_scan_speed,
    m_bearing_alignment, m_interference_rejection,
    m_rain_clutter, m_rain_mode, m_sea_clutter,
    m_sea_mode, m_no_transmit_start,
    m_no_transmit_start_state, m_no_transmit_end,
    m_no_transmit_end_state, m_timed_idle,
    m_timed_idle_mode, m_timed_run, m_next_state_change;
  
 public:
 ch_radar_state(const char * name):ch_base(name)
  {
  }
  virtual ~ch_radar_state()
    {
    }
  void set_state(const int state)
  {
    lock();
    m_state = state;
    unlock();
  }

  const int get_state()
  {
    int state;
    lock();
    state = m_state;
    unlock();
    return state;
  }
  
  void set_scan_speed(const int scan_speed)
  {
    lock();
    m_scan_speed = scan_speed;
    unlock();
  }

  const int get_scan_speed()
  {
    int scan_speed;
    lock();
    scan_speed = m_scan_speed;
    unlock();
    return scan_speed;
  }
  
  void set_range(const int range)
  {
    lock();
    m_range = range;
    unlock();
  }

  const int get_range()
  {
    int range;
    lock();
    range = m_range;
    unlock();
    return range;
  }
  
  void set_gain(const int gain, const int state)
  {
    lock();
    m_gain = gain;
    m_gain_state = state;
    unlock();
  }

  int get_gain()
  {
    int gain;
    lock();
    gain = m_gain;
    unlock();
    return gain;
  }

  int get_gain_state()
  {
    int gain_state;
    lock();
    gain_state = m_gain_state;
    unlock();
    return gain_state;
  }

  void set_bearing_alignment(const int bearing_alignment)
  {
    lock();
    m_bearing_alignment = bearing_alignment;
    unlock();
  }

  int get_bearing_alignment()
  {
    int bearing_alignment;
    lock();
    bearing_alignment = m_bearing_alignment;
    unlock();
    return bearing_alignment;
  }

  void set_interference_rejection(const int interference_rejection)
  {
    lock();
    m_interference_rejection = interference_rejection;
    unlock();
  }

  int get_interference_rejection()
  {
    int interference_rejection;
    lock();
    interference_rejection = m_interference_rejection;
    unlock();
    return interference_rejection;
  }

  void set_rain(const int rain, const int mode)
  {
    lock();
    m_rain_clutter = rain;
    m_rain_mode = m_rain_mode;
    unlock();
  }

  int get_rain()
  {
    int rain;
    lock();
    rain = m_rain_clutter;
    unlock();
    return rain;
  }

  int get_rain_mode()
  {
    int rain_mode;
    lock();
    rain_mode = m_rain_mode;
    unlock();
    return rain_mode;
  }
  
  void set_sea(const int sea, const int mode)
  {
    lock();
    m_sea_clutter = sea;
    m_sea_mode = mode;
    unlock();
  }

  int get_sea()
  {
    int sea;
    lock();
    sea = m_sea_clutter;
    unlock();
    return sea;
  }

  int get_sea_mode()
  {
    int sea_mode;
    lock();
    sea_mode = m_sea_mode;
    unlock();
    return sea_mode;
  }

  void set_no_transmit_start(const int no_transmit_start, const int state)
  {
    lock();
    m_no_transmit_start = no_transmit_start;
    m_no_transmit_start_state = state;
    unlock();
  }

  int get_no_transmit_start()
  {
    int no_transmit_start;
    lock();
    no_transmit_start = m_no_transmit_start;
    unlock();
    return no_transmit_start;
  }
  
  int get_no_transmit_start_state()
  {
    int no_transmit_start_state;
    lock();
    no_transmit_start_state = m_no_transmit_start_state;
    unlock();
    return no_transmit_start_state;
  }

  void set_no_transmit_end(const int no_transmit_end, const int state)
  {
    lock();
    m_no_transmit_end = no_transmit_end;
    m_no_transmit_end_state = state;
    unlock();
  }

  int get_no_transmit_end()
  {
    int no_transmit_end;
    lock();
    no_transmit_end = m_no_transmit_end;
    unlock();
    return no_transmit_end;
  }
  
  int get_no_transmit_end_state()
  {
    int no_transmit_end_state;
    lock();
    no_transmit_end_state = m_no_transmit_end_state;
    unlock();
    return no_transmit_end_state;
  }

  
  void set_timed_idle(const int timed_idle, const int mode)
  {
    lock();
    m_timed_idle = timed_idle;
    m_timed_idle_mode = mode;
    unlock();
  }

  int get_timed_idle()
  {
    int timed_idle;
    lock();
    timed_idle = m_timed_idle;
    unlock();
    return timed_idle;
  }

  int get_timed_idle_mode()
  {
    int timed_idle_mode;
    lock();
    timed_idle_mode = m_timed_idle_mode;
    unlock();
    return timed_idle_mode;
  }

  void set_timed_run(const int timed_run)
  {
    lock();
    m_timed_run = timed_run;
    unlock();
  }

  int get_timed_run()
  {
    int timed_run;
    lock();
    timed_run = m_timed_run;
    unlock();
    return timed_run;
  }

  void set_next_state_change(const int next_state_change)
  {
    lock();
    m_next_state_change = next_state_change;
    unlock();
  }

  int get_next_state_change()
  {
    int next_state_change;
    lock();
    next_state_change = m_next_state_change;
    unlock();
    return next_state_change;
  }
};

class ch_radar_ctrl: public ch_base
{
 protected:
  queue<radar_command> command_queue;
 public:
 ch_radar_ctrl(const char * name):ch_base(name)
  {
  }

  virtual ~ch_radar_ctrl()
    {      
    }

  void push(const radar_command_id id, const int val, const RadarControlState state)
  {
    
    lock();
    command_queue.push(radar_command(id, val, state));    
    unlock();
  }

  bool pop(radar_command_id & id, int & val, RadarControlState & state)
  {
    lock();
    
    if(command_queue.empty()){
      unlock();
      return false;
    }
    
    radar_command & rc = command_queue.front();
    id = rc.id;
    val = rc.val;
    state = rc.state;
    command_queue.pop();
    unlock();
    return true;
  }
};

class ch_radar_image: public ch_base
{
 protected:
  struct line_history {
    unsigned char *line;
    size_t len;
    long long time;
    GeoPosition pos;
  } * m_history;

  int m_range_meters;
 public:
 ch_radar_image(const char * name):ch_base(name)
  {
    m_history = (line_history *)calloc(sizeof(line_history), GARMIN_XHD_SPOKES);
    for (size_t i = 0; i < GARMIN_XHD_SPOKES; i++) {
      m_history[i].line = (unsigned char *)calloc(sizeof(unsigned char), GARMIN_XHD_MAX_SPOKE_LEN);
    }    
  }

  virtual ~ch_radar_image()
    {
      if (m_history) {
	for (size_t i = 0; i < GARMIN_XHD_SPOKES; i++) {
	  if (m_history[i].line) {
	    free(m_history[i].line);
	  }
	}
	free(m_history);
      }      
    }

  void reset_image()
  {
    lock();
    for (size_t i = 0; i < GARMIN_XHD_SPOKES; i++) {
      memset(m_history[i].line, 0, GARMIN_XHD_MAX_SPOKE_LEN);
      m_history[i].len = 0;
      m_history[i].time = 0;
      m_history[i].pos.lat = 0.;
      m_history[i].pos.lon = 0.;
    }
    unlock();
  }

  void set_spoke(long long t, double lat, double lon,
		 int bearing, unsigned char * data,
		 size_t len, int range_meters)
  {
    if(m_range_meters != range_meters){      
      reset_image();
    }
    lock();
    m_range_meters = range_meters;
    unsigned char * hist_data = m_history[bearing].line;
    m_history[bearing].len = len;
    m_history[bearing].pos.lat = lat;
    m_history[bearing].pos.lon = lon;
    m_history[bearing].time = t;
    memcpy(hist_data, data, len);
    unlock();
  }

  void get_spoke(long long & t, double & lat, double & lon,
		 int & bearing, unsigned char * data,
		 size_t & len, int & range_meters)
  {
    lock();
    unsigned char * hist_data = m_history[bearing].line;
    len = m_history[bearing].len;
    lat = m_history[bearing].pos.lat;
    lon = m_history[bearing].pos.lon;
    t = m_history[bearing].time;
    memcpy(data, hist_data, len);
    unlock();
  }

  void get_spoke_data(unsigned char * data)
  {
    lock();
    for (size_t i = 0; i < GARMIN_XHD_SPOKES; i++) {
      memcpy(data + GARMIN_XHD_MAX_SPOKE_LEN * i, m_history[i].line, GARMIN_XHD_MAX_SPOKE_LEN * sizeof(unsigned char));
    }    
    unlock();
  }
};

#endif
