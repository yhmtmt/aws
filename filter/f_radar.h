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

#include "../channel/ch_state.h"
#include "../channel/ch_radar.h"
#include "f_base.h"

#include "f_radar_srcs/socketutil.h"
#include "f_radar_srcs/garminxhd.h"
#include "f_radar_srcs/GarminxHDControl.h"
#include "f_radar_srcs/GarminxHDReceive.h"

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
  ch_radar_image * radar_image;
  ch_radar_state * radar_state;
  ch_radar_ctrl * radar_ctrl;
  static const int range_vals[16];
  const int find_nearest_range(const int range_meter){
    int irange = 0;
    while(range_vals[irange] > range_meter){
      irange++;
    }

    return range_vals[irange];
  }

  void write_radar_image(int val);
 public:
  // RadarInfo related data members
 
  long long m_boot_time;
  long long m_radar_timeout, m_data_timeout, m_stayalive_timeout;
  
  receive_statistics m_statistics;
  long long GetBootTime(){ return m_boot_time;};
  
  void DetectedRadar(const NetworkAddress & radarAddress){
    if(!control.Init("GarminxHDControl", interface_address, radarAddress)){
      return ;
    }
    m_stayalive_timeout = 0;    
  }
  
  void resetTimeout(long long now)
  {
    m_radar_timeout = now + WATCHDOG_TIMEOUT;
  }

  static const char * str_radar_command_id[RC_NONE];
  radar_command cmd;
  int cmd_state;
  
  f_radar(const char * name);

  virtual ~f_radar()
    {
    }

  virtual bool init_run()
  {
    if(!control.Init("GarminxHDControl", interface_address, gx_send)){
      return false;
    }

    if(!receive.Init(state, radar_state, radar_image, interface_address)){
      return false;
    }
    if(!radar_image || !radar_state || !radar_ctrl)
      return false;
    

    return true;
  }

  virtual void destroy_run()
  {
    receive.Destroy();
  }

  virtual bool proc();
};
#endif
