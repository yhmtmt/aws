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

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_aws1_ap.h"

f_aws1_ap::f_aws1_ap(const char * name) : f_base(name), m_state(NULL), m_ctrl_inst(NULL), m_ctrl_stat(NULL), m_obst(NULL),
m_ap_inst(NULL), m_verb(false),
m_wp(NULL), m_meng(127.), m_seng(127.), m_rud(127.), m_smax(10), m_smin(3), m_meng_max(200), m_meng_min(80), m_seng_max(200), m_seng_min(80),
	m_pc(0.1f), m_ic(0.1f), m_dc(0.1f), m_ps(0.1f), m_is(0.1f), m_ds(0.1f),
	m_cdiff(0.f), m_sdiff(0.f), m_dcdiff(0.f), m_icdiff(0.f), m_dsdiff(0.f), m_isdiff(0.f),
					  m_ssmax(3), m_dssmax(30), m_meng_max_stay(167), m_meng_min_stay(87), m_rud_max_stay(191), m_rud_min_stay(63),
m_pc_s(0.1f), m_ic_s(0.1f), m_dc_s(0.1f), m_ps_s(0.1f), m_is_s(0.1f), m_ds_s(0.1f)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_ctrl_inst", (ch_base**)&m_ctrl_inst, typeid(ch_aws1_ctrl_inst).name(), "Ctrl instruction channel");
	register_fpar("ch_ctrl_stat", (ch_base**)&m_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Ctrl status channel");
	register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
	register_fpar("ch_obst", (ch_base**)&m_obst, typeid(ch_obst).name(), "Obstacle channel.");
	register_fpar("ch_ap_inst", (ch_base**)&m_ap_inst, typeid(ch_aws1_ap_inst).name(), "Autopilot instruction channel");
	register_fpar("verb", &m_verb, "Verbose for debug.");
	register_fpar("rud", &m_inst.rud_aws, "Rudder value");
	register_fpar("meng", &m_inst.meng_aws, "Main engine value");
	register_fpar("seng", &m_inst.seng_aws, "Sub engine value");

	register_fpar("smax", &m_smax, "Maximum speed in knot");
	register_fpar("smin", &m_smin, "Minimum speed in knot");
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

	register_fpar("ssmax", &m_ssmax, "Maximum speed for stay mode.");
	register_fpar("dssmax", &m_dssmax, "Distance allows maximum speed in stay mode.");
	register_fpar("meng_max_stay", &m_meng_max_stay, "Main engine maximum output in stay mode.");
	register_fpar("meng_min_stay", &m_meng_min_stay, "Main engine minimum output in stay mode.");
	register_fpar("rud_max_stay", &m_rud_max_stay, "Rudder maximum output in stay mode.");
	register_fpar("rud_min_stay", &m_rud_min_stay, "Rudder minimum output in stay mode.");

	register_fpar("pc_s", &m_pc, "Coefficient P in the course control with PID in stay mode.");
	register_fpar("ic_s", &m_ic, "Coefficient I in the course control with PIDD in stay mode..");
	register_fpar("dc_s", &m_dc, "Coefficient D in the course control with PID D in stay mode..");

	register_fpar("ps_s", &m_ps, "Coefficient P in the speed control with PIDD in stay mode..");
	register_fpar("is_s", &m_is, "Coefficient I in the speed control with PIDD in stay mode..");
	register_fpar("ds_s", &m_ds, "Coefficient D in the speed control with PIDD in stay mode..");

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
	float cog, sog, yaw;
	if(!m_state){
		return false;
	}
	if(!m_ctrl_stat){
		return false;
	}

	s_aws1_ctrl_stat stat;
	long long t = 0;
	m_state->get_velocity(t, cog, sog);
	Mat Rorg;
	Point3f Porg;
	Rorg = m_state->get_enu_rotation(t);
	m_state->get_position_ecef(t, Porg.x, Porg.y, Porg.z);
	{
		float roll, pitch;
		m_state->get_attitude(t, roll, pitch, yaw);
	}

	m_ctrl_stat->get(stat);
	if(stat.ctrl_src == ACS_AP1)
	{	
		if (!m_ap_inst){
			wp(sog, cog, yaw);
		}
		else{
			e_ap_mode mode = m_ap_inst->get_mode();
			switch (mode){
			case EAP_CURSOR:
				cursor();
				break;
			case EAP_STAY:
			  stay(sog, cog, yaw);
				break;
			case EAP_WP:
				wp(sog, cog, yaw);
				break;
			}
		}
	}else{
		m_rud = 127.;
		m_meng = 127.;
		m_seng = 127.;
		m_icdiff = m_isdiff = 0.;
		m_iydiff = 0;
	}

	if(m_ctrl_inst){
	  m_inst.tcur = get_time();
	  m_inst.meng_aws = saturate_cast<unsigned char>(m_meng);
	  m_inst.seng_aws = saturate_cast<unsigned char>(m_seng);
	  m_inst.rud_aws = saturate_cast<unsigned char>(m_rud);
	  m_ctrl_inst->set(m_inst);
	}

	return true;
}


void f_aws1_ap::wp(const float sog, const float cog, const float yaw)
{
	m_wp->lock();
	if (m_wp->is_finished()){
		m_rud = 127.;
		m_meng = 127.;
		m_seng = 127.;
		m_icdiff = m_isdiff = 0.;
	}
	else{
		float cy = (float)((cog - yaw) * PI / 180);

		s_wp & wp = m_wp->get_next_wp();
		float d = 0.;
		float cdiff = 0;

		m_wp->get_diff(d, cdiff);
		cdiff *= (float)(1. / 180.); // normalize
		m_dcdiff = (float)(cdiff - m_cdiff);
		m_icdiff += cdiff;
		m_cdiff = cdiff;

		m_rud = (float)((m_pc * m_cdiff + m_ic * m_icdiff + m_dc * m_dcdiff) * 255. + 127.);
		m_rud = (float)min(m_rud, 255.f);
		m_rud = (float)max(m_rud, 0.f);

		float stgt = (float)((m_smax - m_smin) *(1.0 - abs(m_rud - 127.) * (1 / 127.)) + m_smin);
		float sdiff = (float)(stgt - sog);
		sdiff *= (float)(1. / stgt);

		m_dsdiff = (float)(sdiff - m_sdiff);
		m_isdiff += sdiff;
		m_sdiff = sdiff;

		m_meng = (float)((m_ps * m_sdiff + m_is * m_isdiff + m_ds * m_dsdiff) * 255. + 127.);
		m_meng = (float)min(m_meng, m_meng_max);
		m_meng = (float)max(m_meng, m_meng_min);
		if (m_verb){
			printf("ap rud=%3.1f c=%2.2f dc=%2.2f ic=%2.2f", m_rud, m_cdiff, m_dcdiff, m_icdiff);
			printf(" meg=%3.1f s=%2.2f ds=%2.2f is=%2.2f \n", m_meng, m_sdiff, m_dsdiff, m_isdiff);
		}
	}

	m_wp->unlock();
}

void f_aws1_ap::cursor()
{
}

void f_aws1_ap::stay(const float sog, const float cog, const float yaw)
{
	{ // updating relative position of the stay point.
		long long t;
		Mat Rorg;
		float xorg, yorg, zorg;
		Rorg = m_state->get_enu_rotation(t);
		m_state->get_position_ecef(t, xorg, yorg, zorg);
		m_ap_inst->update_pos_rel(Rorg, xorg, yorg, zorg);
	}

	float cy = (float)((cog - yaw) * PI / 180);
	float ysog = cos(cy) * sog; // the speed in yaw direction

	float rx, ry, d, dir;
	m_ap_inst->get_stay_pos_rel(rx, ry, d, dir);

	float cdiff = (float)(dir - cog);
	if (abs(cdiff) > 180.){
		if (cdiff < 0)
			cdiff += 360.;
		else
			cdiff -= 360.;
	}
	cdiff *= (float)(1. / 180.);

	float ydiff = (float)(dir - yaw);
	if (abs(ydiff) > 180.){
		if (ydiff < 0)
			ydiff += 360.;
		else
			ydiff -= 360.;
	}
	ydiff *= (float)(1. / 180.);
	m_dydiff = (float)(ydiff - m_ydiff);
	m_iydiff += ydiff;
	m_ydiff = ydiff;

	m_dcdiff = (float)(cdiff - m_cdiff);
	m_icdiff += cdiff;
	m_cdiff = cdiff;

	// ssmax is allowed in d is calculated by linear scaling.
	float ssmax = m_ssmax * (min(d, m_dssmax)) / m_dssmax;

	if (ydiff < 0.5 && ydiff > -0.5){ // forward			
		float sdiff = (float)(ssmax - ysog);
		sdiff *= (float)(1. / ssmax);
		m_dsdiff = (float)(sdiff - m_sdiff);
		m_isdiff += sdiff;
		m_sdiff = sdiff;

		m_rud = (float)((m_pc_s * m_cdiff + m_ic_s * m_icdiff + m_dc_s * m_dcdiff) * 255. + 127.);
		m_meng = (float)((m_ps_s * m_sdiff + m_is_s * m_isdiff + m_ds_s * m_dsdiff) * 255. + 127.);
		m_rud = (float)min(m_rud, m_rud_max_stay);
		m_rud = (float)max(m_rud, m_rud_min_stay);
	}
	else{ // backward
		float sdiff = (float)(ssmax + ysog);
		sdiff *= (float)(-1. / ssmax);
		m_dsdiff = (float)(sdiff - m_sdiff);
		m_isdiff += sdiff;
		m_sdiff = sdiff;

		m_rud = (float)((m_pc_s * m_cdiff + m_ic_s * m_icdiff + m_dc_s * m_dcdiff) * 255. + 127.);
		m_meng = (float)((m_ps_s * m_sdiff + m_is_s * m_isdiff + m_ds_s * m_dsdiff) * 255. + 127.);
		m_rud = (float)min(m_rud, m_rud_max_stay);
		m_rud = (float)max(m_rud, m_rud_min_stay);
	}
	m_meng = (float)min(m_meng, m_meng_max_stay);
	m_meng = (float)max(m_meng, m_meng_min_stay);

	if(m_verb){
	  float lat, lon;
	  m_ap_inst->get_stay_pos(lat, lon);
	  cout << "s d " << m_dsdiff << " i " << m_isdiff << " p " << m_sdiff <<endl;
	  cout << "r d " << m_dcdiff << " i " << m_icdiff << " p " << m_cdiff << endl;
	  
	  cout << "ydiff " << ydiff << endl;
	}
}
