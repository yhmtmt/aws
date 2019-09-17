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

enum RangeUnits { RANGE_MIXED, RANGE_METRIC, RANGE_NAUTIC };
static const int RangeUnitsToMeters[3] = {1852, 1000, 1852};

#include "../channel/ch_base.h"
#include "f_base.h"

#include "f_radar_srcs/RadarDefs.h"
#include "f_radar_srcs/RadarReceive.h"
#include "f_radar_srcs/RadarFactory.h"


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
    pcontrol = RadarFactory::MakeRadarControl(radar_type);
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
    preceive = RadarFactory::MakeRadarReceive(radar_type);
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
