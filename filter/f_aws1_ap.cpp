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

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_aws1_ap.h"

f_aws1_ap::f_aws1_ap(const char * name): f_base(name), m_state(NULL), m_ctrl_out(NULL), m_ctrl_in(NULL),
	m_wp(NULL), m_meng(127.), m_seng(127.), m_rud(127.), m_smax(10), m_meng_max(200), m_meng_min(80), m_seng_max(200), m_seng_min(80),
	m_pc(0.1), m_ic(0.1), m_dc(0.1), m_ps(0.1), m_is(0.1), m_ds(0.1),
	m_cdiff(0.), m_sdiff(0.), m_dcdiff(0.), m_icdiff(0.), m_dsdiff(0.), m_isdiff(0.)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_ctrl_out", (ch_base**)&m_ctrl_out, typeid(ch_aws1_ctrl).name(), "Ctrl output channel");
	register_fpar("ch_ctrl_in", (ch_base**)&m_ctrl_in, typeid(ch_aws1_ctrl).name(), "Ctrl input channel");
	register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
	register_fpar("rud", &m_acp.rud_aws, "Rudder value");
	register_fpar("meng", &m_acp.meng_aws, "Main engine value");
	register_fpar("seng", &m_acp.seng_aws, "Sub engine value");

	register_fpar("smax", &m_smax, "Maximum speed in knot");

	register_fpar("meng_max", &m_meng_max, "The maximum value for engine control.");
	register_fpar("meng_min", &m_meng_min, "The minimum value for engine control.");
	register_fpar("seng_max", &m_seng_max, "The maximum value for engine control.");
	register_fpar("seng_min", &m_seng_min, "The minimum value for engine control.");

	register_fpar("pc", &m_pc, "Coefficient P in the course control with PID.");
	register_fpar("ic", &m_ic, "Coefficient I in the course control with PID.");
	register_fpar("dc", &m_dc, "Coefficient D in the course control with PID.");

	register_fpar("ps", &m_ps, "Coefficient P in the speed control with PID.");
	register_fpar("is", &m_is, "Coefficient I in the speed control with PID.");
	register_fpar("ds", &m_ds, "Coefficient D in the speed control with PID.");
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
	float cog, sog;
	if(!m_state){
		return false;
	}
	if(!m_ctrl_in){
		return false;
	}

	s_aws1_ctrl_pars acpkt;
	m_state->get_velocity(cog, sog);
	m_ctrl_in->get_pars(acpkt);
	if(acpkt.ctrl_src == ACS_AP1)
	{		
		m_wp->lock();
		if(m_wp->is_finished()){
			m_rud = 127.;
			m_meng = 127.;
			m_seng = 127.;
			m_icdiff = m_isdiff = 0.;
		}else{
			s_wp & wp = m_wp->get_next_wp();			
			float ctgt = (float) (atan2f(wp.rx, wp.ry) * 180. / PI);
			float cdiff = (float)(ctgt - cog);		
			float cdiff_n;
			if(abs(cdiff) > 180.){
				if(cdiff < 0){
					cdiff += 360.;
				}else{
					cdiff -= 360.;
				}
			}
			float sdiff = (float)(m_smax - sog);
			m_dcdiff = (float)(cdiff - m_cdiff);
			m_dsdiff = (float)(sdiff - m_sdiff);
			m_icdiff += cdiff;
			m_isdiff += sdiff;
			m_cdiff = cdiff;
			m_sdiff = sdiff;
			
			m_rud += m_pc * m_cdiff + m_ic * m_icdiff + m_dc * m_dcdiff;
			m_meng += m_ps * m_sdiff + m_is * m_isdiff + m_ds * m_dsdiff;

			m_meng = min(m_meng, m_meng_max);
			m_meng = max(m_meng, m_meng_min);
		}

		m_wp->unlock();
	}else{
		m_rud = 127.;
		m_meng = 127.;
		m_seng = 127.;
		m_icdiff = m_isdiff = 0.;
	}

	if(m_ctrl_out){
		m_acp.meng = saturate_cast<unsigned char>(m_meng);
		m_acp.seng = saturate_cast<unsigned char>(m_seng);
		m_acp.rud = saturate_cast<unsigned char>(m_rud);
		m_ctrl_out->set_pars(m_acp);
	}

	return true;
}
