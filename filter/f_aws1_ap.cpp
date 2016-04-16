// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ui.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ui.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ui.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
using namespace std;
#include <cmath>
#include <cstring>

#include "f_aws1_ap.h"

f_aws1_ap::f_aws1_ap(const char * name): f_base(name), m_state(NULL), m_ctrl_out(NULL), m_ctrl_in(NULL),
	m_wp(NULL)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_ctrl_out", (ch_base**)&m_ctrl_out, typeid(ch_aws1_ctrl).name(), "Ctrl output channel");
	register_fpar("ch_ctrl_in", (ch_base**)&m_ctrl_in, typeid(ch_aws1_ctrl).name(), "Ctrl input channel");
	register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
	register_fpar("rud", &m_acp.rud_aws, "Rudder value");
	register_fpar("meng", &m_acp.meng_aws, "Main engine value");
	register_fpar("seng", &m_acp.seng_aws, "Sub engine value");
}

f_aws1_ap::~f_aws1_ap()
{
}

bool f_aws1_ap::init_run()
{
	return true;
}

void f_aws1_ap::destroy_run()
{
}

bool f_aws1_ap::proc()
{
	if(m_ctrl_out){
		m_ctrl_out->set_pars(m_acp);
	}
	return true;
}
