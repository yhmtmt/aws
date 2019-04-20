// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_wp_manager.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_wp_manager.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_wp_manager.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_wp_manager.h"

const char * f_wp_manager::str_cmd[cmd_null] = {
	"ins", "ers", "save", "load", "next", "prev"
};

f_wp_manager::f_wp_manager(const char * name) :f_base(name),
					       m_aws1_waypoint_file_version("#aws1_waypoint_v_0.00"), id(0),

					       m_state(NULL), m_wp(NULL), cmd(cmd_null),
					       lat(0.f), lon(0.f), rarv(10.f), vel(10.f)
{
  path[0] = '\0';
  register_fpar("path", path, 1024, "Path to the route file.");
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
  
  register_fpar("cmd", (int*)&cmd, (int)cmd_null, str_cmd, "Command.");
  register_fpar("id", &id, "Route id");
  register_fpar("lat", &lat, "Lattitude");
  register_fpar("lon", &lon, "Logitude");
  register_fpar("rarv", &rarv, "Radius for arrival detection.");
  register_fpar("vel", &vel, "Velocity for navigation.");
}

f_wp_manager::~f_wp_manager()
{
}

bool f_wp_manager::init_run()
{
	return true;
}

void f_wp_manager::destroy_run()
{
}

bool f_wp_manager::proc()
{
  float cog, sog;
  long long t = 0;
  m_state->get_velocity(t, cog, sog);
  Mat Rorg;
  Point3f Porg;
  Rorg = m_state->get_enu_rotation(t);
  m_state->get_position_ecef(t, Porg.x, Porg.y, Porg.z);
  
  // process command
  m_wp->lock();
  switch (cmd) {
  case cmd_save:
    m_wp->set_route_id(id);
    m_wp->set_cmd(ch_wp::e_cmd::cmd_save);
    break;
  case cmd_load:
    m_wp->set_route_id(id);
    m_wp->set_cmd(ch_wp::e_cmd::cmd_load);
    break;
  case cmd_ers:
    m_wp->ers();
    break;
  case cmd_ins:
    m_wp->ins((float)(lat * (PI/180.f)), (float)(lon * (PI/180.f)), (float)rarv, (float)vel);
    break;
  case cmd_next:
    m_wp->next();
    break;
  case cmd_prev:
    m_wp->prev();
    break;
    
  }
  cmd = cmd_null;
  
  switch (m_wp->get_cmd())
    {
    case ch_wp::cmd_load:
      load(m_wp->get_route_id());
      break;
    case ch_wp::cmd_save:
      save(m_wp->get_route_id());
      break;
    }
  m_wp->set_cmd(ch_wp::cmd_none);
  m_wp->unlock();
  
  // updating relative position for all waypoints
  m_wp->lock();
  m_wp->begin();
  for(;!m_wp->is_end(); m_wp->next()){
    s_wp & wp = m_wp->cur();
    wp.update_pos_rel(Rorg, Porg.x, Porg.y, Porg.z);	 
    wp = m_wp->cur();
  }
  
  // if there is next waypoint, calculate the difference between waypoint and ship state, and checking waypoint arrival
  if(!m_wp->is_finished()){
    s_wp & wp = m_wp->get_next_wp();
    float rx_tgt = wp.rx, ry_tgt = wp.ry;
    float d2 = rx_tgt * rx_tgt + ry_tgt * ry_tgt;      
    float d = (float)sqrt(d2);
    if(d < wp.rarv){// arrived
      wp.set_arrival_time(get_time());
      m_wp->set_next_wp();
      m_wp->set_diff(d, 0, 0);
      m_wp->unlock();
      return true;
    }
    s_wp & wp_prev = m_wp->get_prev_wp();
    wp.update_pos_rel(Rorg, Porg.x, Porg.y, Porg.z);
    wp_prev.update_pos_rel(Rorg, Porg.x, Porg.y, Porg.z);
    
    float xdiff = 0.0;
    if(wp_prev.rx != wp.rx && wp_prev.ry != wp.ry){
      // note that my own ship is at (0, 0)
      //(wpx, wpy) : vector from previous wp to next wp
      //(-wp_prev.rx, -wp_prev.ry) : vector from previous wp to own ship
      // awp : length of (wpx, wpy)
      // iawp : inverse of awp
      // bwp : length of  (-wp_prev.rx, -wp_prev.ry)
      // swp : prjection of (-wp_prev.rx, -wp_prev.ry) on (wpx, wpy)
      // (dx, dy) : perpendicular vector on (wpx, wpy) toward own ship
      // (wpy, -wpx) : right perpendicular vector of (wpx, wpy)
      float wpx = (float)(wp.rx - wp_prev.rx);
      float wpy = (float)(wp.ry - wp_prev.ry);
      float awp = (float)sqrt(wpx * wpx + wpy * wpy);
      float bwp = (float)sqrt(wp_prev.rx * wp_prev.rx 
			      + wp_prev.ry * wp_prev.ry);
      float iawp = (float)(1.0 / awp);
      float swp = (float)(iawp * (-wpx * wp_prev.rx 
				  - wpy * wp_prev.ry));
      float siawp = (float)(swp * iawp);
      float swpx = (float)(siawp * wpx);
      float swpy = (float)(siawp * wpy);
      float dx = (float)(-wp_prev.rx - swpx);
      float dy = (float)(-wp_prev.ry - swpy);
      
      // xdiff is given as (dx, dy).(wpy,-wpx)
      // (it means that xdiff is positive in right direction)
      xdiff = (float)(iawp * (wpy * dx - wpx * dy));
      float rlim = (float)(wp.rarv * 3.0f);
      if(xdiff >= rlim){
	rx_tgt = (float)(swpx + wp_prev.rx);
	ry_tgt = (float)(swpy + wp_prev.ry);
      }else if(xdiff > 0 || xdiff < 0){
	float sahd = sqrt(rlim * rlim - xdiff * xdiff);
	siawp = (float)((swp + sahd) * iawp);
	siawp = min(siawp, 1.0f);
	siawp = max(siawp, 0.0f);
	rx_tgt = (float)(siawp * wpx + wp_prev.rx);
	ry_tgt = (float) (siawp * wpy + wp_prev.ry);
      }
      }
      
      float ctgt = (float)(atan2(rx_tgt, ry_tgt) * 180. / PI);
      float cdiff = (float)(ctgt - cog);
      if(abs(cdiff) > 180.){
	if(cdiff < 0)
	  cdiff += 360.;
	else
	  cdiff -= 360.;
      }
      
      m_wp->set_diff(d, cdiff, xdiff);     
    }
    
    m_wp->unlock();
    return true;
}

void f_wp_manager::load(const int id)
{
  char fname[1024];
  snprintf(fname, 1024, "%s/%03d.rt", path, id);
  FILE * pf = fopen(fname, "r");
  
  if (pf){
    bool valid = true;
    const char * p = m_aws1_waypoint_file_version;
    char c;
    for (c = fgetc(pf); c != EOF && c != '\n'; c = fgetc(pf), p++){
      if (*p != c){
	valid = false;
      }
    }
    if (c == '\n' && valid){
      int num_wps;
      if (EOF == fscanf(pf, "%d\n", &num_wps)){
	cerr << "Unexpected end of file was detected in " << fname << "." << endl;
	goto fail_and_close;
      }
      float lat, lon, rarv;
      m_wp->clear();
      
      for (int i = 0; i < num_wps; i++){
	int _i;
	if (EOF == fscanf(pf, "%d %f %f %f\n", &_i, &lat, &lon, &rarv)){
	  cerr << "Unexpected end of file was detected in " << fname << "." << endl;
	  goto fail_and_close;
	}
	m_wp->ins((float)(lat * (PI / 180.)), (float)(lon * (PI / 180.)), rarv);
      }
    }
    else{
      cerr << "The file " << fname << " does not match the current waypoint file version." << endl;
    }
  }
  else{
    cerr << "Failed to load route file " << fname << "." << endl;
    return;
  }
 fail_and_close:
  fclose(pf);
}
 
 void f_wp_manager::save(const int id)
 {
   char fname[1024];
   snprintf(fname, 1024, "%s/%03d.rt", path, id);
   FILE * pf = fopen(fname, "w");
   
   if (pf){
     fprintf(pf, "%s\n", m_aws1_waypoint_file_version);
     
     fprintf(pf, "%d\n", m_wp->get_num_wps());
     
     int i = 0;
     for (m_wp->begin(); !m_wp->is_end(); m_wp->next()){
       s_wp & wp = m_wp->cur();
       fprintf(pf, "%d %013.8f %013.8f %03.1f\n", i,
	       (float)(wp.lat * (180 / PI)),
	       (float)(wp.lon * (180 / PI)), wp.rarv);
       i++;
     }
     fclose(pf);
   }
   else{
     cerr << "Failed to save route file " << fname << "." << endl;
   }
 }
