// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_aws1_sim.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_sim.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_sim.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <list>
#include <map>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#ifdef _WIN32
#include <Windows.h>
#endif

#include "f_aws1_sim.h"

/////////////////////////////////////////////////////////////////////////// f_aws1_sim members

f_aws1_sim::f_aws1_sim(const char * name) :f_base(name),
m_state(NULL), m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL), m_ch_ctrl_stat(NULL),
m_ahrs(false), m_gps(false), r(0), p(0), y(0), lon(0), lat(0), alt(0), galt(0), cog(0), sog(0), depth(0),
m_rud_sta_sim(0.f)
{
	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_ctrl_ui", (ch_base**)&m_ch_ctrl_ui, typeid(ch_aws1_ctrl_inst).name(), "Control input channel.");
	register_fpar("ch_ctrl_ap1", (ch_base**)&m_ch_ctrl_ap1, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 1 control input channel.");
	register_fpar("ch_ctrl_ap2", (ch_base**)&m_ch_ctrl_ap2, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 2 control input channel.");
	register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");

	// for m_state
	register_fpar("ahrs", &m_ahrs, "Yes if AHRS is on the test.");
	register_fpar("gps", &m_gps, "Yes if GPS is on the test.");
	register_fpar("roll", &r, "Roll(deg)");
	register_fpar("pitch", &p, "Pitch(deg)");
	register_fpar("yaw", &y, "Yaw(deg)");
	register_fpar("lon", &lon, "Longitude(deg)");
	register_fpar("lat", &lat, "Latitude(deg)");
	register_fpar("alt", &alt, "Altitude(m)");
	register_fpar("galt", &galt, "Geoid height(m)");
	register_fpar("cog", &cog, "Course over ground(deg)");
	register_fpar("sog", &sog, "Speed over ground(kts)");
	register_fpar("depth", &depth, "Depth of the water(m).");

	// for ch_ctrl
	// aws's control parameters
	register_fpar("awsrud", &m_stat.rud_aws, "Control value of AWS1's rudder.");
	register_fpar("awsmeng", &m_stat.meng_aws, "Control value of AWS1's main engine.");
	register_fpar("awsseng", &m_stat.seng_aws, "Control value of AWS1's sub engine.");

	// remote controller's control parameters (Read Only)
	register_fpar("rmcrud", &m_stat.rud_rmc, "Control value of AWS1's rudder controller.");
	register_fpar("rmcmeng", &m_stat.meng_rmc, "Control value of AWS1's main engine controller.");
	register_fpar("rmcseng", &m_stat.seng_rmc, "Control value of AWS1's sub engine controller.");
	register_fpar("rud_sta", &m_stat.rud_sta, "Rudder Status of AWS1's.");

	// Remote controllers control points of the main engine. 
	register_fpar("meng_max_rmc", &m_stat.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
	register_fpar("meng_nuf_rmc", &m_stat.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
	register_fpar("meng_nut_rmc", &m_stat.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
	register_fpar("meng_nub_rmc", &m_stat.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
	register_fpar("meng_min_rmc", &m_stat.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

	// Each control points of the main engine output.
	register_fpar("meng_max", &m_stat.meng_max, "Maximum control value for AWS1's main engine.");
	register_fpar("meng_nuf", &m_stat.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
	register_fpar("meng_nut", &m_stat.meng_nut, "Nutral control value for AWS1's main engine.");
	register_fpar("meng_nub", &m_stat.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
	register_fpar("meng_min", &m_stat.meng_min, "Minimum control value for AWS1's main engine.");

	// Remote controllers control points of the sub engine.
	register_fpar("seng_max_rmc", &m_stat.seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
	register_fpar("seng_nuf_rmc", &m_stat.seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
	register_fpar("seng_nut_rmc", &m_stat.seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
	register_fpar("seng_nub_rmc", &m_stat.seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
	register_fpar("seng_min_rmc", &m_stat.seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

	// Each control points of the sub engine output
	register_fpar("seng_max", &m_stat.seng_max, "Maximum control value for AWS1's sub engine.");
	register_fpar("seng_nuf", &m_stat.seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
	register_fpar("seng_nut", &m_stat.seng_nut, "Nutral control value for AWS1's sub engine.");
	register_fpar("seng_nub", &m_stat.seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
	register_fpar("seng_min", &m_stat.seng_min, "Minimum control value for AWS1's sub engine.");

	// Remote controller's control points of the rudder.
	register_fpar("rud_max_rmc", &m_stat.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
	register_fpar("rud_nut_rmc", &m_stat.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
	register_fpar("rud_min_rmc", &m_stat.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

	// Each controll points of the rudder output.
	register_fpar("rud_max", &m_stat.rud_max, "Maximum control value for AWS1's rudder.");
	register_fpar("rud_nut", &m_stat.rud_nut, "Nutral control value for AWS1's rudder.");
	register_fpar("rud_min", &m_stat.rud_min, "Minimum control value for AWS1's rudder.");

	// Rudder indicator's controll points.
	register_fpar("rud_sta_max", &m_stat.rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
	register_fpar("rud_sta_nut", &m_stat.rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
	register_fpar("rud_sta_min", &m_stat.rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

	// Control points as the rudder indicator output.
	register_fpar("rud_sta_out_max", &m_stat.rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
	register_fpar("rud_sta_out_nut", &m_stat.rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
	register_fpar("rud_sta_out_min", &m_stat.rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

	register_fpar("meng", &m_stat.meng, "Output value for main engine.");
	register_fpar("seng", &m_stat.meng, "Output value for sub engine.");
	register_fpar("rud", &m_stat.rud, "Output value for rudder.");
	register_fpar("rud_sta_out", &m_stat.rud_sta_out, "Output value for rudder status.");
}

bool f_aws1_sim::init_run()
{
	return true;
}

void f_aws1_sim::destroy_run()
{

}

void f_aws1_sim::select_control_input()
{
	// Control input selection
	s_aws1_ctrl_inst acp;
	if (m_ch_ctrl_ui)
		m_ch_ctrl_ui->get(acp);
	m_stat.tcur = acp.tcur;
	m_stat.ctrl_src = acp.ctrl_src;
	switch (m_stat.ctrl_src){
	case ACS_UI:
		m_stat.rud_aws = acp.rud_aws;
		m_stat.meng_aws = acp.meng_aws;
		m_stat.seng_aws = acp.seng_aws;
		break;
	case ACS_AP1:
		if (m_ch_ctrl_ap1){
			m_ch_ctrl_ap1->get(acp);
			m_stat.rud_aws = acp.rud_aws;
			m_stat.meng_aws = acp.meng_aws;
			m_stat.seng_aws = acp.seng_aws;
		}
		break;
	case ACS_AP2:
		if (m_ch_ctrl_ap2){
			m_ch_ctrl_ap2->get(acp);
			m_stat.rud_aws = acp.rud_aws;
			m_stat.meng_aws = acp.meng_aws;
			m_stat.seng_aws = acp.seng_aws;
		}
		break;
	default:
		break;
	}
}


void f_aws1_sim::set_control_output()
{
	switch (m_stat.ctrl_src){
	case ACS_UI:
	case ACS_AP1:
	case ACS_AP2:
	case ACS_FSET:
	case ACS_NONE:
		m_stat.rud = map_oval(m_stat.rud_aws,
			0xff, 0x7f, 0x00,
			m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
		m_stat.meng = map_oval(m_stat.meng_aws,
			0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
			m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut,
			m_stat.meng_nub, m_stat.meng_min);
		m_stat.seng = map_oval(m_stat.seng_aws,
			0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
			m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut,
			m_stat.seng_nub, m_stat.seng_min);
		break;
	case ACS_RMT:
		m_stat.rud = map_oval(m_stat.rud_rmc,
			m_stat.rud_max_rmc, m_stat.rud_nut_rmc, m_stat.rud_min_rmc,
			m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
		m_stat.meng = map_oval(m_stat.meng_rmc,
			m_stat.meng_max_rmc, m_stat.meng_nuf_rmc, m_stat.meng_nut_rmc,
			m_stat.meng_nub_rmc, m_stat.meng_min_rmc,
			m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, m_stat.meng_nub,
			m_stat.meng_min);
		m_stat.seng = map_oval(m_stat.seng_rmc,
			m_stat.seng_max_rmc, m_stat.seng_nuf_rmc, m_stat.seng_nut_rmc,
			m_stat.seng_nub_rmc, m_stat.seng_min_rmc,
			m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, m_stat.seng_nub,
			m_stat.seng_min);
		break;
	}
	if (m_ch_ctrl_stat){
		m_ch_ctrl_stat->set(m_stat);
	}
}

void f_aws1_sim::simulate_rudder(){
	// Rudder response simulation
	unsigned rud_inst = map_oval(m_stat.rud,
		m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min,
		m_stat.rud_sta_max, m_stat.rud_sta_nut, m_stat.rud_sta_min);
	if (rud_inst > m_stat.rud_sta){
		m_rud_sta_sim += RUD_PER_CYCLE;
	}
	else{
		m_rud_sta_sim -= RUD_PER_CYCLE;
	}

	m_stat.rud_sta = (unsigned char)m_rud_sta_sim;
}


void f_aws1_sim::set_state(){
	if (m_state){
		long long t = 0;
		if (!m_ahrs)
			m_state->set_attitude(t, r, p, y);

		if (!m_gps){
			m_state->set_position(t, lat, lon, alt, galt);
			m_state->set_velocity(t, cog, sog);
		}

		m_state->set_depth(t, depth);
	}
}

void f_aws1_sim::simulate_dynamics()
{
	simulate_rudder();

}

bool f_aws1_sim::proc()
{
	select_control_input();

	simulate_dynamics();

	set_control_output();

	set_state();

	simulate_dynamics();

	return true;
}


