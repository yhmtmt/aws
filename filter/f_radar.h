// Copyright(c) 2019 Yohei Matsumoto, All right reserved.
// f_radar.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_radar.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_radar.h.  If not, see <http://www.gnu.org/licenses/>.


// This program is based on radar_pi developed under GPLv2.
// See https://github.com/opencpn-radar-pi/radar_i



#ifndef _F_RADAR_H_
#define _F_RADAR_H_

#define DEGREES_PER_ROTATION (360)
#define WATCHDOG_TIMEOUT (10)  // After 10s assume GPS and heading data is invalid
#define DATA_TIMEOUT (5)
#define TIMED_OUT(t, timeout) (t >= timeout)

enum RadarState {
  RADAR_OFF,
  RADAR_STANDBY,
  RADAR_WARMING_UP,
  RADAR_TIMED_IDLE,
  RADAR_STOPPING,
  RADAR_SPINNING_DOWN,
  RADAR_STARTING,
  RADAR_SPINNING_UP,
  RADAR_TRANSMIT
};

enum RangeUnits { RANGE_MIXED, RANGE_METRIC, RANGE_NAUTIC };
static const int RangeUnitsToMeters[3] = {1852, 1000, 1852};

#include "../channel/ch_base.h"
#include "f_base.h"

static const NetworkAddress gx_data(239, 254, 2, 0, 50102);
static const NetworkAddress gx_report(239, 254, 2, 0, 50100);
static const NetworkAddress gx_send(172, 16, 2, 0, 50101);
#define RANGE_METRIC_RT_GARMIN_XHD \
  { 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000, 36000, 48000, 64000 }
// Garmin mixed range is the same as nautical miles, it does not support really short ranges
#define RANGE_MIXED_RT_GARMIN_XHD                                                                                         \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }
#define RANGE_NAUTIC_RT_GARMIN_XHD                                                                                        \
  {                                                                                                                       \
    1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 1, 1852 * 3 / 2, 1852 * 2, 1852 * 3, 1852 * 4, 1852 * 6, 1852 * 8, \
        1852 * 12, 1852 * 16, 1852 * 24, 1852 * 36, 1852 * 48                                                             \
  }

// Garmin xHD has 1440 spokes of varying 519 - 705 bytes each
#define GARMIN_XHD_SPOKES 1440
#define GARMIN_XHD_MAX_SPOKE_LEN 705



#include "f_radar_srcs/garminxhd/GarminxHDControl.h"
#include "f_radar_srcs/garminxhd/GarminxHDReceive.h"

struct receive_statistics {
  int packets;
  int broken_packets;
  int spokes;
  int broken_spokes;
  int missing_spokes;
};


class f_radar:public f_base
{
 protected:
  GarminxHDControl control;
  GarminxHDReceive receive;
  NetworkAddress interface_address;

 public:
  // RadarInfo related data members
  struct line_history {
    uint8_t *line;
    wxLongLong time;
    GeoPosition pos;
  };

  line_history *m_history;  
  long long m_boot_time;
  long long m_radar_timeout, m_data_timeout, m_stayalive_timeout;
  int m_state, m_gain, m_gain_state, m_scan_speed, m_bearing_alignment, m_interference_rejection,  m_rain_clutter, m_rain_mode, m_sea_clutter, m_sea_mode, m_no_transmit_start, m_no_transmit_start_state, m_no_transmit_end, m_no_transmit_end_state, m_timed_idle, m_timed_idle_mode, m_timed_run, m_next_state_change;
  receive_statistics m_statistics;
  int m_range;
  long long GetBootTime(){ return m_boot_time;};
  double GetHeadingTrue(){ return 0.0;};
  void DetectedRadar(NetworkAddress & radarAddress){
    if(!control.Init("GarminxHDControl", interface_address, radarAddress)){
      return ;
    }
    m_stayalive_timeout = 0;    
  }
  void ProcessRadarSpoke(int angle, int bearing, uint8_t *data, size_t len, int range_meters, long long time);

  void ResetRadarImage()
  {
    for (size_t i = 0; i < GARMIN_XHD_SPOKES; i++) {
      memset(m_history[i].line, 0, GARMIN_XHD_MAX_SPOKE_LEN);
      m_history[i].time = 0;
      m_history[i].pos.lat = 0.;
      m_history[i].pos.lon = 0.;
    }    
  }

  void reesetTimeout(long long now)
  {
    m_radar_timeout = now + WATCHDOG_TIMEOUT;
  }
  
 f_radar(const char * name): f_base(name), interface_address(172,16,1,1,0),
    receive(this, interface_address, gx_report, gx_data), control(this, gx_send)
  {
  }

  virtual ~f_radar()
    {
    }

  virtual bool init_run()
  {
    if(!control.Init("GarminxHDControl", interface_address, gx_send)){
      return false;
    }

    if(!receive.Init(interface_address)){
      return false;
    }
    
    m_history = (line_history *)calloc(sizeof(line_history), m_spokes);
    for (size_t i = 0; i < m_spokes; i++) {
      m_history[i].line = (uint8_t *)calloc(sizeof(uint8_t), m_spoke_len_max);
    }

    return true;
  }

  virtual void destroy_run()
  {
    if (m_history) {
      for (size_t i = 0; i < m_spokes; i++) {
	if (m_history[i].line) {
	  free(m_history[i].line);
	}
      }
      free(m_history);
    }

    receive.Destroy();
  }

  virtual bool proc()
  {
    
    return receive.Loop();
  }
};

class f_radar_ctrl: public f_base
{
 protected:
  RadarType radar_type;
  RadarControl * pcontrol;
 public:
 f_radar_ctrl(const char * name):f_base(name), pcontrol(NULL)
  {
    register_fpar("type", (int*)&radar_type, (int)RT_MAX, RadarTypeName, "Radar type name.");
  }

  virtual ~f_radar_ctrl()
    {      
    }

  virtual bool init_run()
  {
  }

  virtual void destroy_run()
  {
    delete pcontrol;
    pcontrol = NULL;
  }

  virtual bool proc()
  {    
  }
};

class f_radar_rcv: public f_base
{
 protected:
  RadarReceive * preceive;
  RadarType radar_type;
 public:
 f_radar_rcv(const char *name):
  f_base(name), preceive(NULL)
  {
    register_fpar("type", (int*)&radar_type, (int)RT_MAX, RadarTypeName, "Radar type name.");
  }
  
  virtual ~f_radar_rcv()
    {
    }

  virtual bool init_run()
  {
  }

  virtual void destroy_run()
  {
    delete preceive;
    preceive = NULL;
  }

  virtual bool proc()
  {
  }
};

#endif
