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
	list<s_wp> wps;
	list<s_wp>::iterator itr;
	int focus;
	list<s_wp>::iterator itr_focus;
public:
	ch_wp(const char * name):ch_base(name), focus(0)
	{
		lock();
		itr_focus = itr = wps.begin();
		unlock();
	}

	void ins(const s_wp & wp){
		lock();
		if(itr_focus == wps.end())
			itr_focus = wps.insert(itr_focus, wp);
		else{
			itr_focus++;
			itr_focus = wps.insert(itr_focus, wp);
		}
		unlock();
	}

	void ers(){
		lock();
		if(wps.end() != itr_focus)
			itr_focus = wps.erase(itr_focus);

		if(itr_focus == wps.end())
			focus = (int) wps.size();
		unlock();
	}

	void set_focus(int i)
	{
		lock();
		int j;
		for(j = 0, itr_focus = wps.begin(); itr_focus != wps.end() && j < i; j++);
		focus = j;
		unlock();
	}

	int get_focus()
	{
		return focus;
	}

	void next_focus()
	{
		lock();
		if(itr_focus != wps.end())
			itr_focus++;

		if(itr_focus == wps.end()){
			focus = (int) wps.size();
		}else{
			focus++;
		}
		unlock();
	}

	void prev_focus()
	{
		lock();
		if(itr_focus != wps.begin())
			itr_focus--;
		if(itr_focus == wps.begin())
			focus = 0;
		else
			focus--;
		unlock();
	}
	bool is_focused()
	{
		lock();
		bool r = itr == itr_focus;
		unlock();
		return r;
	}
	
	s_wp & cur(){
		return *itr;
	}

	bool is_end(){
		return itr == wps.end();
	}

	bool is_begin(){
		lock();
		bool r = itr == wps.begin();
		unlock();
		return r;
	}

	void  begin(){
		lock();
		itr = wps.begin();
		unlock();
	}

	void end(){
		lock();
		itr = wps.end();
		unlock();
	}

	s_wp & seek(int i){
		lock();
		for(itr = wps.begin(); i!= 0 && itr != wps.end(); itr++, i--);
		s_wp & r = *itr;
		unlock();
		return r;
	}

	void next(){
		lock();
		if(wps.end() != itr)
			itr++;
		unlock();
	}

	void prev(){
		lock();
		if(wps.begin() != itr)
			itr--;
		unlock();
	}

	int get_num_wps(){
		lock();
		int r = (int) wps.size();
		unlock();
		return r;
	}
};

#endif