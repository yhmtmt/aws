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

f_wp_manager::f_wp_manager(const char * name):f_base(name), m_state(NULL), m_wp(NULL)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
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
		wp.update_pos_rel(Rorg, Porg.x, Porg.y, Porg.z);			
		float d2 = wp.rx * wp.rx + wp.ry * wp.ry;

		float d = (float)sqrt(d2);
		float ctgt = (float)(atan2(wp.rx, wp.ry) * 180. / PI);
		float cdiff = (float)(ctgt - cog);
		if(abs(cdiff) > 180.){
			if(cdiff < 0)
				cdiff += 360.;
			else
				cdiff -= 360.;
		}

		m_wp->set_diff(d, cdiff);
		if(d < wp.rarv){// arrived
			wp.set_arrival_time(get_time());
			m_wp->set_next_wp();
		}
	}

	m_wp->unlock();
	return true;
}
