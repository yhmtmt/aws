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

#include "../util/aws_coord.h"

#include "ch_base.h"

class ch_state: public ch_base
{
 protected:
  float roll, pitch, yaw; // roll(deg), pitch(deg), yaw(deg)
  float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
  float x, y, z; // ecef coordinate
  Mat R; // Rotation matrix for ENU transformation
  float cog, sog; // Course over ground(deg), Speed over ground (kts)
  float depth; // water depth
 public:
 ch_state(const char * name): ch_base(name), roll(0), pitch(0), yaw(0),
    lon(0), lat(0), alt(0), galt(0), x(0), y(0), z(0), cog(0), sog(0), depth(0)
    {
    }

  void set_attitude(const float _r, const float _p, const float _y)
  {
    lock();
    roll = _r; 
    pitch = _p;
    yaw = _y;
    unlock();
  }

  void set_position(const float _lat, const float _lon, const float _alt, const float _galt)
  {
    lock();
    lat = _lat;
    lon = _lon;
    alt = _alt;
	galt = _galt;
	getwrldrot(lat, lon, R);
	bihtoecef(lat, lon, alt, x, y, z);
    unlock();
  }

  void set_velocity(const float _cog, const float _sog)
  {
	  lock();
	  cog = _cog;
	  sog = _sog;
	  unlock();
  }

  void set_depth(const float _depth){
	  lock();
	  depth = _depth;
	  unlock();
  }

  void get_attitude(float & _r, float & _p, float & _y)
  {
    lock();
    _r = roll;
    _p = pitch;
    _y = yaw;
    unlock();
  }

  void get_position(float & _lat, float & _lon, float & _alt, float & _galt)
  {
    lock();
    _lat = lat;
    _lon = lon;
    _alt = alt;
	_galt = galt;
    unlock();
  }

  void get_position_ecef(float & _x, float & _y, float & _z)
  {
	  lock();
	  _x = x;
	  _y = y;
	  _z = z;
	  unlock();
  }

  const Mat & get_enu_rotation()
  {
	  return R;
  }

  void get_velocity(float & _cog, float & _sog)
  {
	  lock();
	  _cog = cog;
	  _sog = sog;
	  unlock();
  }

  void get_depth(float & _depth)
  {
	  lock();
	  _depth = depth;
	  unlock();
  }

  virtual size_t get_dsize()
  {
    return sizeof(float) * 13 + sizeof(double) * 9;
  }
  
  virtual size_t write_buf(const char *buf)
  {
    lock();
    const float * ptr = (const float*) buf;
    roll = ptr[0];
    pitch = ptr[1];
    yaw = ptr[2];
    lat = ptr[3];
    lon = ptr[4];
    alt = ptr[5];
    galt = ptr[6];
	x = ptr[7];
	y = ptr[8];
	z = ptr[9];
    cog = ptr[10];
    sog = ptr[11];
    depth = ptr[12];
	const double * dptr = (const double*)(ptr + 13);
	memcpy((void*)R.data, (void*) dptr, sizeof(double) * 9);
    unlock();
    return get_dsize();
  }

  virtual size_t read_buf(char * buf)
  {
    lock();
    float * ptr =  (float*) buf;
    ptr[0] = roll;
    ptr[1] = pitch;
    ptr[2] = yaw;
    ptr[3] = lat;
    ptr[4] = lon;
    ptr[5] = alt;
    ptr[6] = galt;
	ptr[7] = x;
	ptr[8] = y;
	ptr[9] = z;
    ptr[10] = cog;
    ptr[11] = sog;
    ptr[12] = depth;
	double * dptr = (double*) (ptr + 13);
	memcpy((void*)dptr, (void*) R.data, sizeof(double) * 9);
    unlock();
    return get_dsize();
  }
  
  virtual void print(ostream & out)
  {
    out << "channel " << m_name << " RPY= " << roll << "," << pitch << "," << yaw 
	<< " pos= " << lat << "," << lon << "," << alt << endl;
  }
};

#endif
