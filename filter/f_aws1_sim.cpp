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

#include "../util/aws_coord.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include "f_aws1_sim.h"

/////////////////////////////////////////////////////////////////////////// f_aws1_sim members

f_aws1_sim::f_aws1_sim(const char * name) :
	f_base(name),
	m_state(NULL), m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL),
	m_ch_ctrl_stat(NULL),
	m_state_sim(NULL), m_engstate_sim(NULL), m_ch_ctrl_stat_sim(NULL),
	m_rud_sta_sim(0.f), m_trud_swing((float)(5.0)), 
	m_tgear_swing((float)(2.0)), m_tthro_swing((float)(2.0)), m_tau_sog(5.0),
	m_tprev(0), m_bcsv_out(false), m_int_smpl_sec(0.1), m_wismpl(100), m_wosmpl(100)
{
	// input channels for simulation results
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_engstate", (ch_base**)&m_engstate, typeid(ch_eng_state).name(), "Engine Status channel");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");

  register_fpar("ch_ctrl_ui", (ch_base**)&m_ch_ctrl_ui, typeid(ch_aws1_ctrl_inst).name(), "Control input channel.");
  register_fpar("ch_ctrl_ap1", (ch_base**)&m_ch_ctrl_ap1, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 1 control input channel.");
  register_fpar("ch_ctrl_ap2", (ch_base**)&m_ch_ctrl_ap2, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 2 control input channel.");

  // output channels for simulation results
  register_fpar("ch_state_sim", (ch_base**)&m_state_sim, typeid(ch_state).name(), "State channel");
  register_fpar("ch_engstate_sim", (ch_base**)&m_engstate_sim, typeid(ch_eng_state).name(), "Engine Status channel");
  register_fpar("ch_ctrl_stat_sim", (ch_base**)&m_ch_ctrl_stat_sim, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");

  register_fpar("int_smpl", &m_int_smpl_sec, "Sampling interval in second");
  register_fpar("wismpl", &m_wismpl, "Width of input sampling window");
  register_fpar("wosmpl", &m_wosmpl, "Width of output sampling window");


  // initial values of input vector
  register_fpar("lat0", &m_sv_init.lat, "Initial Latitude(deg)");
  register_fpar("lon0", &m_sv_init.lon, "Initial Longitude(deg)");
  register_fpar("roll0", &m_sv_init.roll, "Initial Roll(deg)");
  register_fpar("pitch0", &m_sv_init.pitch, "Initial Pitch(deg)");
  register_fpar("yaw0", &m_sv_init.yaw, "Initial Yaw(deg)");
  register_fpar("cog0", &m_sv_init.cog, "Initial Course over ground(deg)");
  register_fpar("sog0", &m_sv_init.sog, "Initial Speed over ground(kts)");
  register_fpar("meng0", &m_sv_init.eng, "Initial Engine control value");
  register_fpar("rud0", &m_sv_init.rud, "Initial Rudder control value");
  register_fpar("rev0", &m_sv_init.rev, "Initial Engine rev (RPM).");
  register_fpar("fuel0", &m_sv_init.fuel, "Initial fuel flow rate (L/h)");

  // for ch_ctrl
  // aws's control parameters
  register_fpar("awsrud", &m_ctrl_stat.rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_ctrl_stat.meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_ctrl_stat.seng_aws, "Control value of AWS1's sub engine.");
  
  // remote controller's control parameters (Read Only)
  register_fpar("rmcrud", &m_ctrl_stat.rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_ctrl_stat.meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_ctrl_stat.seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_ctrl_stat.rud_sta, "Rudder Status of AWS1's.");
  
  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_ctrl_stat.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_ctrl_stat.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_ctrl_stat.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_ctrl_stat.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_ctrl_stat.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");
  
  // Each control points of the main engine output.
  register_fpar("meng_max", &m_ctrl_stat.meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_ctrl_stat.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_ctrl_stat.meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_ctrl_stat.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_ctrl_stat.meng_min, "Minimum control value for AWS1's main engine.");
  
  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_ctrl_stat.seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_ctrl_stat.seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_ctrl_stat.seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_ctrl_stat.seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_ctrl_stat.seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");
  
  // Each control points of the sub engine output
  register_fpar("seng_max", &m_ctrl_stat.seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_ctrl_stat.seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_ctrl_stat.seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_ctrl_stat.seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_ctrl_stat.seng_min, "Minimum control value for AWS1's sub engine.");
  
  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_ctrl_stat.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_ctrl_stat.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_ctrl_stat.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");
  
	// Each controll points of the rudder output.
  register_fpar("rud_max", &m_ctrl_stat.rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_ctrl_stat.rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_ctrl_stat.rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_ctrl_stat.rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_ctrl_stat.rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_ctrl_stat.rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");
  
  // Control points as the rudder indicator output.
  register_fpar("rud_sta_out_max", &m_ctrl_stat.rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_ctrl_stat.rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_ctrl_stat.rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

  register_fpar("meng", &m_ctrl_stat.meng, "Output value for main engine.");
  register_fpar("seng", &m_ctrl_stat.meng, "Output value for sub engine.");
  register_fpar("rud", &m_ctrl_stat.rud, "Output value for rudder.");
  register_fpar("rud_sta_out", &m_ctrl_stat.rud_sta_out, "Output value for rudder status.");
  
  register_fpar("trud_swing", &m_trud_swing, "Full swing duration of aws1 rudder");
  register_fpar("tgear_swing", &m_tgear_swing, "Half swing duration of aws1 gear");
  register_fpar("tthro_swing", &m_tthro_swing, "Full swing duration of aws1 throttle");
  register_fpar("tau_sog", &m_tau_sog, "Time constant for sog to its final value.");

  m_fcsv_out[0] = '\0';
  register_fpar("fcsv", m_fcsv_out, 1024, "CSV output file.");

}

bool f_aws1_sim::init_run()
{
 // int rud_val_swing = abs((int)m_ctrl_stat.rud_sta_max - (int)m_ctrl_stat.rud_sta_min);
  m_spd_rud_swing = (float)((double)(2.0 / m_trud_swing) * (double)(m_int_smpl_sec));
  m_spd_gear_swing = (float)((double)(1.0 / m_tgear_swing) * (double)(m_int_smpl_sec));
  m_spd_thro_swing = (float)((double)(1.0 / m_tthro_swing) * (double)(m_int_smpl_sec));

  m_int_smpl = (unsigned int)(m_int_smpl_sec * SEC);

  init_input_sample();
  init_output_sample();
    
  if (m_fcsv_out[0]) {
	  m_fcsv.open(m_fcsv_out, ios::binary);
	  if (!m_fcsv.is_open()) {
		  cerr << "Failed to open file " << m_fcsv_out << endl;
		  return false;
	  }

	  // first row of the csv file
	  m_fcsv <<
		  "t,lat_o,lon_o,xe_o,ye_o,ze_o,roll_o,pitch_o,yaw_o,sog_o,cog_o,eng_o,rud_o,rev_o,fuel_o,"
		  << "thro,gear,rud,"
		  <<"lat_i,lon_i,xe_i,ye_i,ze_i,roll_i,pitch_i,yaw_i,sog_i,cog_i,eng_i,rud_i,rev_i,fuel_i," 
		  << endl;
	  m_fcsv.precision(3);
  }

  return true;
}

void f_aws1_sim::destroy_run()
{
	if (m_fcsv.is_open()) {
		m_fcsv.close();
	}
}

void f_aws1_sim::set_control_input()
{
  // Control input selection
	// control input source is selected by ctrl_src parameter in ctrl_ui channel.
	// meng_aws,  seng_aws, rud_aws are normalized control value to [0 255], their neutral value is 127.
  s_aws1_ctrl_inst acp;
  if (m_ch_ctrl_ui)
	  m_ch_ctrl_ui->get(acp);
  else
	  return;

  switch (acp.ctrl_src){
  case ACS_UI:
	  m_sv_cur.rud = (float)acp.rud_aws;
	  m_sv_cur.eng = (float)acp.meng_aws;
    break;
  case ACS_AP1:
    if (m_ch_ctrl_ap1){
      m_ch_ctrl_ap1->get(acp);
	  m_sv_cur.rud = (float)acp.rud_aws;
	  m_sv_cur.eng = (float)acp.meng_aws;
	}
    break;
  case ACS_AP2:
    if (m_ch_ctrl_ap2){
      m_ch_ctrl_ap2->get(acp);
	  m_sv_cur.rud = (float)acp.rud_aws;
	  m_sv_cur.eng = (float)acp.meng_aws;
	}
    break;
  default:
    break;
  }
  m_sv_cur.thro_pos = m_output_vectors[0].thro_pos;
  m_sv_cur.gear_pos = m_output_vectors[0].gear_pos;
  m_sv_cur.rud_pos = m_output_vectors[0].rud_pos;
}


void f_aws1_sim::set_control_output()
{
	s_aws1_ctrl_inst acp;
	if (m_ch_ctrl_ui)
		m_ch_ctrl_ui->get(acp);
	else
		return;

	// control values are directry from previous update.
	m_ctrl_stat.ctrl_src = acp.ctrl_src;
	m_ctrl_stat.meng_aws = saturate_cast<unsigned char>(m_sv_cur.eng);
	m_ctrl_stat.rud_aws = saturate_cast<unsigned char>(m_sv_cur.rud);

  switch (m_ctrl_stat.ctrl_src){
  case ACS_UI:
  case ACS_AP1:
  case ACS_AP2:
  case ACS_FSET:
  case ACS_NONE:
    m_ctrl_stat.rud = map_oval(m_ctrl_stat.rud_aws,
			  0xff, 0x7f, 0x00,
			  m_ctrl_stat.rud_max, m_ctrl_stat.rud_nut, m_ctrl_stat.rud_min);
    m_ctrl_stat.meng = map_oval(m_ctrl_stat.meng_aws,
			   0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
			   m_ctrl_stat.meng_max, m_ctrl_stat.meng_nuf, m_ctrl_stat.meng_nut,
			   m_ctrl_stat.meng_nub, m_ctrl_stat.meng_min);
    m_ctrl_stat.seng = map_oval(m_ctrl_stat.seng_aws,
			   0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
			   m_ctrl_stat.seng_max, m_ctrl_stat.seng_nuf, m_ctrl_stat.seng_nut,
			   m_ctrl_stat.seng_nub, m_ctrl_stat.seng_min);
    break;
  case ACS_RMT:
    m_ctrl_stat.rud = map_oval(m_ctrl_stat.rud_rmc,
			  m_ctrl_stat.rud_max_rmc, m_ctrl_stat.rud_nut_rmc, m_ctrl_stat.rud_min_rmc,
			  m_ctrl_stat.rud_max, m_ctrl_stat.rud_nut, m_ctrl_stat.rud_min);
    m_ctrl_stat.meng = map_oval(m_ctrl_stat.meng_rmc,
			   m_ctrl_stat.meng_max_rmc, m_ctrl_stat.meng_nuf_rmc, m_ctrl_stat.meng_nut_rmc,
			   m_ctrl_stat.meng_nub_rmc, m_ctrl_stat.meng_min_rmc,
			   m_ctrl_stat.meng_max, m_ctrl_stat.meng_nuf, m_ctrl_stat.meng_nut, m_ctrl_stat.meng_nub,
			   m_ctrl_stat.meng_min);
    m_ctrl_stat.seng = map_oval(m_ctrl_stat.seng_rmc,
			   m_ctrl_stat.seng_max_rmc, m_ctrl_stat.seng_nuf_rmc, m_ctrl_stat.seng_nut_rmc,
			   m_ctrl_stat.seng_nub_rmc, m_ctrl_stat.seng_min_rmc,
			   m_ctrl_stat.seng_max, m_ctrl_stat.seng_nuf, m_ctrl_stat.seng_nut, m_ctrl_stat.seng_nub,
			   m_ctrl_stat.seng_min);
    break;
  }
  if (m_ch_ctrl_stat_sim){
    m_ch_ctrl_stat_sim->set(m_ctrl_stat);
  }
}

void f_aws1_sim::simulate_rudder(const float rud, const float rud_pos, float & rud_pos_next)
{
	// Rudder response simulation
  double tgt_rud = (double) (rud - 0x7f)*(2.0f / 255.f);

  if(fabs(tgt_rud-rud_pos) > m_spd_rud_swing){
	  if (tgt_rud > rud_pos)
		  rud_pos_next = rud_pos + m_spd_rud_swing;
	  else
		  rud_pos_next = rud_pos - m_spd_rud_swing;
  }
  else {
	  rud_pos_next = tgt_rud;
  }
}


void f_aws1_sim::simulate_engine(const float eng, const float thro_pos, const float gear_pos, float & thro_pos_next, float & gear_pos_next)
{
	const int fpos = 0x7f + 0x19;
	const int fthrange = 255 - fpos;
	const int bpos = 0x7f - 0x19;
	const int bthrange = bpos;

  if (bpos >= eng) {
	  if (gear_pos > -1.0) {
		  gear_pos_next = gear_pos - m_spd_gear_swing;
		  gear_pos_next = max(gear_pos_next, -1.0f);
	  }
	  else {
		  float tgt_thro_pos = ((float)(((float)bpos - m_sv_cur.eng)/(float)bthrange));
		  if (fabs(thro_pos-tgt_thro_pos) >= m_spd_thro_swing) {
			  if (thro_pos < tgt_thro_pos)
				  thro_pos_next = thro_pos + m_spd_thro_swing;
			  else
				  thro_pos_next = thro_pos - m_spd_thro_swing;
		  }
		  else
			  thro_pos_next = thro_pos;
	  }
  }
  else if (fpos <= eng) {
	  if (gear_pos < 1.0) {
		  gear_pos_next = gear_pos + m_spd_gear_swing;
		  gear_pos_next = min(gear_pos_next, 1.0f);
	  }
	  else {
		  float tgt_thro_pos = (float)((m_sv_cur.eng - (float)fpos)/(float)fthrange);
		  if (fabs(thro_pos-tgt_thro_pos) >= m_spd_thro_swing) {
			  if (thro_pos < tgt_thro_pos)
				  thro_pos_next = thro_pos + m_spd_thro_swing;
			  else
				  thro_pos_next = thro_pos - m_spd_thro_swing;
		  }
	  }
  }
  else {
	  if (fabs(gear_pos) < m_spd_gear_swing)
		  gear_pos_next = 0.f;
	  else if (gear_pos < 0)
		  gear_pos_next = gear_pos + m_spd_gear_swing;
	  else
		  gear_pos_next = gear_pos + m_spd_gear_swing;
  }

  // to calculate engine rpm, we need to know
  // 1. log speed or engine load  (not available) 
  // 2. engine control value
  // But the first one is not available because we do not have sensors for it.
  // So we assume that the log speed is the same as speed over ground,
  // in other words, the sea current is zero.

  // Np: Propeller rotations per sec
  // G: Gear ratio
  // Ne: Engine rotations per sec
  // S: Slip rate (0 < S < 1)
  // P: Propeller Pitch (m) 
  // r: Propeller radius (m)
  // v: Ship velocity (m/s)
  // T: Thrust force (N)
  // Qp: Propeller Torque (Nm)
  // Qe: Engine Torque (Nm)
  // c: Engine control value
  // Cl: Lift Coefficient
  // Cd: Drag coefficient
  // Rs: Speed Resistance 
  // M: Mass of aws1
  // Ma: Addes Mass
  //
  // Np = Ne / G ... (1)
  // v = S Np P ... (2)
  // S(Qp, Qe) ... (3)
  // Qe(c, t) ... (4)
  // Rs(v)  ... (5)
  // 
  // Qp = I [v^2 + (2 Pi r Np)^2] [Cl sin(phi) + Cd cos(phi)] r dr ... (6)
  // T = I [v^2 + (2 Pi r Np)^2] [Cl cos(phi) - Cd sin(phi)] dr ... (7)
  //
  // Note  [v^2 + (2 Pi r Np)^2] = (S^2P^2 + 4 Pi^2 r^2) Np^2
  // Then we can replace the (6) and (7) with
  // 
  // Qp = K(S) Np^2 ... (6)'
  // T = J(S) Np^2 ... (7)'
  // 
  // where K and J are the function of the slip ratio obtained through
  // experimental results
  // 
  // v = I [(T(S, Np) - R(v))/(M + Ma)] dt ... (8)
  //
  //
}


void f_aws1_sim::set_input_state_vector(const long long & tcur)
{
	m_sv_cur.t = tcur;
  if (m_state){
    long long t = 0;
	float roll, pitch, yaw, lat, lon, alt, galt, cog, sog;
	m_state->get_attitude(t, roll, pitch, yaw);
	m_state->get_position(t, lat, lon, alt, galt);
	m_state->get_velocity(t, cog, sog);
	m_sv_cur.roll = roll;
	m_sv_cur.pitch = pitch;
	m_sv_cur.yaw = yaw;
	m_sv_cur.cog = cog;
	m_sv_cur.sog = sog;
	m_sv_cur.lat = lat;
	m_sv_cur.lon = lon;
	m_sv_cur.update_coordinates();
  }

  if (m_engstate)
  {
	  long long t = 0;
	  unsigned char trim = 0;
	  int poil = 0;
	  float toil = 0.0f;
	  float temp = 0.0f;
	  float valt = 0.0f;
	  float frate = 0.0f;
	  unsigned int teng = 0;
	  int pclnt = 0;
	  int pfl = 0;
	  unsigned char ld = 0;
	  unsigned char tq = 0;
	  StatEng1 steng1 = (StatEng1)(EmergencyStop + 1);
	  StatEng2 steng2 = (StatEng2)(EngineShuttingDown + 1);

	  m_engstate->get_rapid(t, m_sv_cur.rev, trim);
	  m_engstate->get_dynamic(t, poil, toil, temp, valt, frate, teng, pclnt, pfl, steng1, steng2, ld, tq);
	  m_sv_cur.fuel = frate;
  }

  set_control_input(); // select and load correct control values of correct source to msv_cur.rud and m_sv_cur.eng
}


void f_aws1_sim::set_output_state_vector()
{
	s_state_vector sv = m_output_vectors[0];
	if (m_engstate_sim)
	{
		// output simulated engine state
		long long t = 0;
		unsigned char trim = 0;
		int poil = 0;
		float toil = 0.0f;
		float temp = 0.0f;
		float valt = 0.0f;
		float frate = 0.0f;
		float rev = 0.0f;
		unsigned int teng = 0;
		int pclnt = 0;
		int pfl = 0;
		unsigned char ld = 0;
		unsigned char tq = 0;
		StatEng1 steng1 = (StatEng1)(EmergencyStop + 1);
		StatEng2 steng2 = (StatEng2)(EngineShuttingDown + 1);

		// overwrite only rev 
		m_engstate->get_rapid(t, rev, trim); 
		m_engstate_sim->set_rapid(sv.t, sv.rev, trim);

		// overwrite only frate
		m_engstate->get_dynamic(t, poil, toil, temp, valt, frate, teng, pclnt, pfl, steng1, steng2, ld, tq);
		m_engstate_sim->set_dynamic(sv.t, poil, toil, temp, valt, sv.fuel, teng, pclnt, pfl, steng1, steng2, ld, tq);
	}

	if (m_state_sim)
	{
		float alt = 0.f, galt = 0.f;
		// output simulated lat, lon, roll, pitch, yaw, cog, sog
		m_state_sim->set_attitude(sv.t, sv.roll, sv.pitch, sv.yaw);
		m_state_sim->set_position(sv.t, sv.lat, sv.lon, alt, galt);
		m_state_sim->set_velocity(sv.t, sv.cog, sv.sog);
	}

	if (m_ch_ctrl_stat_sim)
	{
		// output control stat meng, rud is from (directry from ctrl_ui, ctrl_ap1, ctrl_ap2), otherwise, from m_ctrl_stat
		set_control_output();
	}
}

void f_aws1_sim::init_input_sample()
{
	m_sv_init.update_coordinates();
	m_input_vectors.resize(m_wismpl);
	m_iv_head = m_wismpl - 1;
	long long t = get_time();

	for (int iv = m_iv_head, nv = 0; nv < m_wismpl; nv++) {
		m_input_vectors[iv] = m_sv_init;
		m_input_vectors[iv].t = t - nv * m_int_smpl;
		if (iv == 0)
			iv = m_wismpl - 1;
		else
			iv = iv - 1;
	}
}

void f_aws1_sim::update_input_sample()
{
	m_iv_head++;
	if (m_iv_head == m_wismpl)
		m_iv_head = 0;

	m_input_vectors[m_iv_head] = m_sv_cur;
}

void f_aws1_sim::init_output_sample()
{
	m_sv_init.update_coordinates();
	m_output_vectors.resize(m_wosmpl);
	long long t = get_time();
	for (int ov = 0; ov < m_wosmpl; ov++) {
		m_output_vectors[ov] = m_sv_init;
		m_output_vectors[ov].t = t + ov * m_int_smpl;
	}
}

void f_aws1_sim::update_output_sample(const long long & tcur)
{
	// here the simulation data
	for (int iosv = 0; iosv < m_wosmpl; iosv++) {
		// simulation at tcur + ios * m_int_smpl using m_wismpl past samples.
		simulate(tcur, iosv);
	}
}

void f_aws1_sim::simulate(const long long tcur, const int iosv)
{
	long long tsim = tcur + iosv * m_int_smpl;
	// in the current implementation, output is same as the input. 
	// here we should insert simulation code
	if (iosv == 0) {
		s_state_vector & stprev = m_input_vectors[m_iv_head];
		s_state_vector & stcur = m_output_vectors[iosv];
		stcur = stprev;
		simulate_engine(stprev.eng, stprev.thro_pos, stprev.gear_pos, stcur.thro_pos, stcur.gear_pos);
		simulate_rudder(stprev.rud, stprev.rud_pos, stcur.rud_pos);
		stcur.rev = final_rev(stcur.thro_pos);
		stcur.sog = simulate_sog(stcur.gear_pos, stcur.rev, stprev.sog);
	}else{
		s_state_vector & stprev = m_output_vectors[iosv-1];
		s_state_vector & stcur = m_output_vectors[iosv];
		stcur = stprev;
		simulate_engine(stprev.eng, stprev.thro_pos, stprev.gear_pos, stcur.thro_pos, stcur.gear_pos);
		simulate_rudder(stprev.rud, stprev.rud_pos, stcur.rud_pos);
		stcur.rev = final_rev(stcur.thro_pos);
		stcur.sog = simulate_sog(stcur.gear_pos, stcur.rev, stprev.sog);
	}
}

void f_aws1_sim::save_csv(const long long tcur)
{
	s_state_vector & svo = m_output_vectors[0];
	s_state_vector & svi = m_input_vectors[m_iv_head];
	m_fcsv << tcur << ",";

	m_fcsv.precision(8);
	m_fcsv <<
		svo.lat << "," <<
		svo.lon << "," <<
		svo.xe << "," <<
		svo.ye << "," <<
		svo.ze << ",";

	m_fcsv.precision(3);
	m_fcsv <<
		svo.roll << "," <<
		svo.pitch << "," <<
		svo.yaw << "," <<
		svo.sog << "," <<
		svo.cog << "," <<
		svo.eng << "," <<
		svo.rud << "," <<
		svo.rev << "," <<
		svo.fuel << ",";
	m_fcsv <<
		svo.thro_pos << "," <<
		svo.gear_pos << "," <<
		svo.rud_pos << ",";

	m_fcsv.precision(8);
	m_fcsv <<
		svi.lat << "," <<
		svi.lon << "," <<
		svi.xe << "," <<
		svi.ye << "," <<
		svi.ze << ",";

	m_fcsv.precision(3);
	m_fcsv <<
		svi.roll << "," <<
		svi.pitch << "," <<
		svi.yaw << "," <<
		svi.sog << "," <<
		svi.cog << "," <<
		svi.eng << "," <<
		svi.rud << "," <<
		svi.rev << "," <<
		svi.fuel << ",";

	m_fcsv << endl;
}

bool f_aws1_sim::proc()
{
	long long tcur = get_time();
	if (tcur < m_tprev + m_int_smpl)
		return true;

	// simulate state vector Vt(eng, rud, lat, lon, pitch, yaw, cog, sog, rev, fuel) from {Vt-1, Vt-2, ..., Vt-n)
	update_output_sample(tcur);
	set_output_state_vector();

	set_input_state_vector(tcur);
	update_input_sample();

	if (m_fcsv.is_open()) {
		save_csv(tcur);
	}

	m_tprev = tcur;
	return true;
}


