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
 	char label[32]; // waypoint label (0 terminated string)
	float lat, lon; // longitude lattitude
	float x, y, z;	// corresponding ECEF to lat, lon
	float rx, ry, rz; // Relative position in my own ship coordinate
	float rarv;		// Arrival threashold radius
	float v;		// velocity to go
	long long t;	// arrival time

	s_wp():lat(0.), lon(0.), x(0.), y(0.), z(0.), rarv(0.), v(0.), t(-1)
	{
	}

	s_wp(const char _label[32], float _lat, float _lon, float _rarv, float _v):
		lat(_lat), lon(_lon), x(0.), y(0.), z(0.), rarv(_rarv), v(_v), t(-1)
	{
		memcpy(label, label, 32);
		bihtoecef(lat, lon, 0., x, y, z);
	}

	~s_wp()
	{
	}

	void update_pos_rel(const Mat & Rorg, float xorg, float yorg, float zorg)
	{
		eceftowrld(Rorg, xorg, yorg, zorg, x, y, z, rx, ry, rz);
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
protected:
	list<s_wp*> wps;
	list<s_wp*>::iterator itr;
	int focus;
	list<s_wp*>::iterator itr_focus;
	list<s_wp*>::iterator itr_next;

	void find_next(){
		for(itr = wps.begin(); itr != wps.end() && (*itr)->get_arrival_time() > 0; itr++);

		itr_next = itr;
	}

public:
	ch_wp(const char * name):ch_base(name), focus(0)
	{
		itr_next = itr_focus = itr = wps.begin();
	}


	void ins(float lat, float lon, float rarv){
		if(itr_focus == wps.end()){
			s_wp * pwp = new s_wp;
			pwp->lat = lat;
			pwp->lon = lon;
			pwp->rarv = rarv;
			itr_focus = wps.insert(itr_focus, pwp);
		}else{
			itr_focus++;
		}

	  find_next();
	}

	bool is_finished(){
		return itr_next == wps.end();
	}

	s_wp & get_next_wp(){		
		return **itr_next;
	}

	void set_next_wp(){
		if(itr_next != wps.end())
			itr_next++;
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
		for(j = 0, itr_focus = wps.begin(); itr_focus != wps.end() && j < i; j++);
		focus = j;
	}

	int get_focus()
	{
		return focus;
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

#endif
