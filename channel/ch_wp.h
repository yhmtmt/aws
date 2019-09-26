#ifndef _CH_WP_H_
#define _CH_WP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_wp.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_wp.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_wp.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"
#include "../util/aws_coord.h"

struct s_wp
{
  float lat, lon; // longitude lattitude
  float x, y, z;	// corresponding ECEF to lat, lon
  float rx, ry, rz; // Relative position in my own ship coordinate
  float rarv;		// Arrival threashold radius
  float v;		// velocity to go
  long long t;	// arrival time
  bool update_rpos;
  
s_wp() :lat(0.), lon(0.), x(0.), y(0.), z(0.), rarv(0.), v(0.), t(-1),
    update_rpos(false)
  {
  }
  
s_wp(float _lat, float _lon, float _rarv, float _v) :
  lat(_lat), lon(_lon), x(0.), y(0.), z(0.), rarv(_rarv), v(_v), t(-1),
    update_rpos(false)
  {
    bihtoecef(lat, lon, 0., x, y, z);
  }
  
s_wp(bool bnull): lat(FLT_MAX), lon(FLT_MAX), x(FLT_MAX), y(FLT_MAX), z(FLT_MAX), rarv(FLT_MAX), 
    v(FLT_MAX), t(LLONG_MAX), update_rpos(bnull){
}
  
  ~s_wp()
  {
  }
  
  bool is_null(){
    return lat == FLT_MAX;
  }
  
  void update_pos_rel(const Mat & Rorg, float xorg, float yorg, float zorg)
  {
    eceftowrld(Rorg, xorg, yorg, zorg, x, y, z, rx, ry, rz);
    update_rpos = true;
  }
  
  void set_arrival_time(const long long _t)
  {
    t = _t;
  }
  
  const long long get_arrival_time()
  {
    return t;
  }
};

// contains waypoints
// has insert, delete, access method
class ch_wp: public ch_base
{
public:
	enum e_cmd{
		cmd_save, cmd_load, cmd_none
	};
protected:
	e_cmd cmd;

	int id;

	static s_wp wp_null;

	list<s_wp*> wps;
	list<s_wp*>::iterator itr;
	int focus, inext;
	list<s_wp*>::iterator itr_focus;
	list<s_wp*>::iterator itr_next;
	float dist_next;
	float cdiff_next;
	float xdiff_next;

	void find_next(){
		for(itr = wps.begin(), inext = 0; itr != wps.end() && (*itr)->get_arrival_time() > 0; inext++, itr++);
		itr_next = itr;
	}

public:
 ch_wp(const char * name) :ch_base(name), focus(0), dist_next(0.), cdiff_next(0), xdiff_next(0), cmd(cmd_none), id(0)
	{
		itr_next = itr_focus = itr = wps.begin();
	}

	void clear()
	{
		for(itr = wps.begin(); itr != wps.end(); itr++)
			delete *itr;
		wps.clear();
		focus = 0;
		itr_next = itr_focus = itr = wps.begin();
	}

	void set_cmd(const e_cmd _cmd)
	{
		cmd = _cmd;
	}

	const e_cmd get_cmd()
	{
		return cmd;
	}

	void set_route_id(const int _id)
	{
		id = _id;
	}

	const int get_route_id()
	{
		return id;
	}

	void set_diff(const float dist, const float cdiff, const float xdiff)
	{	  
	  dist_next = dist;
	  cdiff_next = cdiff;
	  xdiff_next = xdiff;
	}

	void get_diff(float & dist, float & cdiff, float & xdiff)
	{
		dist = dist_next;
		cdiff = cdiff_next;
		xdiff = xdiff_next;
	}


	bool is_finished(){
		return itr_next == wps.end();
	}

	s_wp & get_next_wp(){		
		return **itr_next;
	}

	s_wp & get_prev_wp(){
	  if(itr_next == wps.begin()){
	    return **itr_next;
	  }
	  return **std::prev(itr_next);
	}

	void set_next_wp(){
		if (itr_next != wps.end()){
			inext++;
			itr_next++;
		}
	}

	void ins(float lat, float lon, float rarv, float v = 0.){
		s_wp * pwp = new s_wp(lat, lon, rarv, v);
		itr_focus = wps.insert(itr_focus, pwp);
		focus++;
		itr_focus++;

		find_next();
	}

	void ers(){
		if(wps.end() != itr_focus){
			delete *itr_focus;
			itr_focus = wps.erase(itr_focus);
		}
		if(itr_focus == wps.end())
			focus = (int) wps.size();

	  find_next();
	}

	void set_focus(int i)
	{
		int j;
		for(j = 0, itr_focus = wps.begin(); itr_focus != wps.end() && j < i; j++, itr_focus++);
		focus = j;
	}

	int get_focus()
	{
		return focus;
	}

	int get_next()
	{
		return inext;
	}

	void next_focus()
	{
		if(itr_focus != wps.end())
			itr_focus++;

		if(itr_focus == wps.end()){
			focus = (int) wps.size();
		}else{
			focus++;
		}
	}

	void prev_focus()
	{
		if(itr_focus != wps.begin())
			itr_focus--;
		if(itr_focus == wps.begin())
			focus = 0;
		else
			focus--;
	}

	s_wp & get_focused_wp()
	{
		if (itr_focus == wps.end())
			return wp_null;
		return **itr_focus;
	}

	bool is_focused()
	{
		bool r = itr == itr_focus;
		return r;
	}
	
	s_wp & cur(){
		return **itr;
	}

	bool is_end(){
		return itr == wps.end();
	}

	bool is_begin(){
		bool r = itr == wps.begin();
		return r;
	}

	void  begin(){
		itr = wps.begin();
	}

	void end(){
		itr = wps.end();
	}

	s_wp & seek(int i){
		for(itr = wps.begin(); i!= 0 && itr != wps.end(); itr++, i--);
		s_wp & r = **itr;
		return r;
	}

	void next(){
		if(wps.end() != itr)
			itr++;
	}

	void prev(){
		if(wps.begin() != itr)
			itr--;
	}

	int get_num_wps(){
		int r = (int) wps.size();
		return r;
	}
};


class ch_route :public ch_base
{
private:
public:
	ch_route(const char * name) : ch_base(name)
	{
	}

	virtual ~ch_route()
	{

	}
};

#endif
