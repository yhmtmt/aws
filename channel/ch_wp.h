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

struct s_wp
{
	char label[32]; // waypoint label
	float lat, lon; // longitude lattitude
	float x, y, z;	// corresponding ECEF to lat, lon
	float rarv;		// Arrival threashold radius
	float v;		// velocity to go
	long long t;	// arrival time
};

// contains waypoints
// has insert, delete, access method
class ch_wp: public ch_base
{
protected:
	list<s_wp> wps;
	list<s_wp>::iterator itr;
public:
	ch_wp(const char * name):ch_base(name)
	{
		itr = wps.begin();
	}

	void ins(const s_wp & wp){
		itr = wps.insert(itr, wp);
	}

	void ers(){
		itr = wps.erase(itr);
	}
	
	const s_wp & cur(){
		return *itr;
	}

	bool is_end(){
		return itr == wps.end();
	}

	bool is_begin(){
		return itr == wps.begin();
	}

	const s_wp & begin(){
		return *(itr = wps.begin());
	}

	void end(){
		itr = wps.end();
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
		wps.size();
	}
};

#endif