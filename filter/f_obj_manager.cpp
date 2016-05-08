// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_obj_manager.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_obj_manager.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_obj_manager.h.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include <opencv2/opencv.hpp>

using namespace cv;


#include "f_obj_manager.h"

f_obj_manager::f_obj_manager(const char * name): f_base(name), m_state(NULL), m_ais_obj(NULL),
	m_range(10000), m_dtold(180 * SEC)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
	register_fpar("ch_obj", (ch_base**)&m_obj, typeid(ch_obj).name(), "Generic object channel.");
	register_fpar("dtold", &m_dtold, "Time the objects alive from their update.");
	register_fpar("range", &m_range, "The object range of interest.");
}

f_obj_manager::~f_obj_manager()
{
}

bool f_obj_manager::init_run()
{
	return true;
}

void f_obj_manager::destroy_run()
{
}

bool f_obj_manager::proc()
{
	Mat Renu;
	float x, y, z;
	if(m_state){
		long long t = 0;
		Renu = m_state->get_enu_rotation(t);
		m_state->get_position_ecef(t, x, y, z);
	}

	if(m_ais_obj){
		// update enu coordinate
		if(!Renu.empty()){
			m_ais_obj->update_rel_pos_and_vel(Renu, x, y, z);
			m_ais_obj->remove_old(m_cur_time - m_dtold);
			m_ais_obj->remove_out(m_range);
		}
		m_ais_obj->reset_updates();
	}

	if(m_obj){
	}

	return true;
}
