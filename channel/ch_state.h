#ifndef _CH_STATE_H_
#define _CH_STATE_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_state.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_state.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_state.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"

class ch_state: public ch_base
{
 protected:
  float r, p, y; // roll pitch yaw
  float lon, lat, alt; // longitude, latitude, altitude
 public:
 ch_state(const char * name): ch_base(name), r(0), p(0), y(0),
    lon(0), lat(0), alt(0)
    {
    }

  void set_attitude(const float _r, const float _p, const float _y)
  {
    lock();
    r = _r; 
    p = _p;
    y = _y;
    unlock();
  }

  void set_position(const float _lat, const float _lon, const float _alt)
  {
    lock();
    lat = _lat;
    lon = _lon;
    alt = _alt;
    unlock();
  }

  void get_attitude(float & _r, float & _p, float & _y)
  {
    lock();
    _r = r;
    _p = p;
    _y = y;
    unlock();
  }

  void get_position(float & _lat, float & _lon, float & _alt)
  {
    lock();
    _lat = lat;
    _lon = lon;
    _alt = alt;
    unlock();
  }

  virtual size_t get_dsize()
  {
    return sizeof(float) * 6;
  }
  
  virtual size_t write_buf(const char *buf)
  {
    lock();
    const float * ptr = (const float*) buf;
    r = ptr[0];
    p = ptr[1];
    y = ptr[2];
    lat = ptr[3];
    lon = ptr[4];
    alt = ptr[5];
    unlock();
    return sizeof(float) * 6;
  }

  virtual size_t read_buf(char * buf)
  {
    lock();
    float * ptr =  (float*) buf;
    ptr[0] = r;
    ptr[1] = p;
    ptr[2] = y;
    ptr[3] = lat;
    ptr[4] = lon;
    ptr[5] = alt;
    unlock();
    return sizeof(float) * 6;
  }
  
  virtual void print(ostream & out)
  {
    out << "channel " << m_name << " RPY= " << r << "," << p << "," << y 
	<< " pos= " << lat << "," << lon << "," << alt << endl;
  }
};

#endif
