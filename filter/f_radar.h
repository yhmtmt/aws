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

#include "../channel/ch_radar.h"
#include "f_base.h"

#include "f_radar_srcs/socketutil.h"
#include "f_radar_srcs/garminxhd/garminxhd.h"
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

  ch_state * state;
  ch_radar_image * radar;
  ch_radar_state * radar_state;
  ch_radar_ctrl * radar_ctrl;
  
 public:
  // RadarInfo related data members

  line_history *m_history;
  
  long long m_boot_time;
  long long m_radar_timeout, m_data_timeout, m_stayalive_timeout;
  int m_state, m_gain, m_gain_state, m_scan_speed,
    m_bearing_alignment, m_interference_rejection,
    m_rain_clutter, m_rain_mode, m_sea_clutter,
    m_sea_mode, m_no_transmit_start,
    m_no_transmit_start_state, m_no_transmit_end,
    m_no_transmit_end_state, m_timed_idle,
    m_timed_idle_mode, m_timed_run, m_next_state_change;
  
  receive_statistics m_statistics;
  int m_range;
  long long GetBootTime(){ return m_boot_time;};
  double GetHeadingTrue(){ return 0.0;};
  void DetectedRadar(const NetworkAddress & radarAddress){
    if(!control.Init("GarminxHDControl", interface_address, radarAddress)){
      return ;
    }
    m_stayalive_timeout = 0;    
  }
  void ProcessRadarSpoke(int angle, int bearing, uint8_t *data, size_t len, int range_meters, long long t)
  {
    long long tpos;
    float lat, lon;
    state->get_pos(tpos, lat, lon);
    radar->set_spoke(t, lat, lon, angle, bearing, data, len, range_meters);
  }

  void resetTimeout(long long now)
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
    if(!radar)
      return false;

    return true;
  }

  virtual void destroy_run()
  {
    receive.Destroy();
  }

  virtual bool proc()
  {
    radar_command_id id;
    int val;
    int state;
    while(radar_ctrl.pop(id, val, state))
      {
	switch(id){
	case RC_TXOFF:
	  control.RadarTxOff();
	  break;
	case RC_TXON:
	  control.RadarTxOn();
	  break;
	case RC_RANGE:
	  control.SetRange(val);
	  break;
	default:
	  control.SetControlValue(id, val, state);
	  break;	  	  
      }
    return receive.Loop();
  }
};
#endif
