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

bool c_model_base::register_param(f_aws1_sim * psim, int _index)
{
  index = _index;
  int n = get_num_params();
  str_param = new char*[n];
  for(int ipar = 0; ipar < n; ipar++){
    if(index == -1){
      psim->register_model_params(get_str_param(ipar),
			  get_param(ipar),
			  get_str_param_exp(ipar));
      continue;
    }
    str_param[ipar] = gen_str_indexed_param(ipar);
    psim->register_model_params(str_param[ipar], get_param(ipar),
				get_str_param_exp(ipar));
  }
}


const char * c_model_3dof::_str_par[num_params] = {
  "xg", "yg",
  "m",
  "ma_xu", "ma_yv", "ma_yr", "ma_nv", "ma_nr",
  "dl_xu", "dl_yv", "dl_yr", "dl_nv", "dl_nr",
  "dq_xu", "dq_yv", "dq_yr", "dq_nv", "dq_nr" 
};

const char * c_model_3dof::_str_par_exp[num_params] = {
  "Center of gravity in x", "Center of gravity in y",
  "Mass",
  "Added mass in x for x speed", "Added mass in y for y speed", "Added mass in y for yaw rate", "Added mass in yaw for y speed", "Added mass in yaw for yaw rate",
  "Linear drag coefficient in x for x speed", "Linear drag in y for y speed", "Linear drag in y for yaw rate", "Linear drag in yaw for y speed", "Linear drag in yaw for yaw rate",
  "Quadratic drag coefficient in x for x speed", "Quadratic drag in y for y speed", "Quadratic drag in y for yaw rate", "Quadratic drag in yaw for y speed", "Quadratic drag in yaw for yaw rate"

};

void c_model_3dof::update(double * _v,
			  double * f /* x-y force and z moment applied */,
			  const double dt /* time step */, double * _vnew)
{
  if (Dq.empty() || M.empty() || Dl.empty()) {
    cerr << "Matrix should be initialized!" << endl;
    _vnew[0] = _vnew[1] = _vnew[2] = 0.;
    return;
  }
  
  // initializing quadratic drag
  // q00|v0| q01|v1| q02|v2|
  // q10|v0| q11|v1| q12|v2|
  // q20|v0| q21|v1| q22|v2|
  double * data = Dq.ptr<double>();
  for (int i = 0; i < 9; i++) {
    data[i] = abs(_v[i % 3]) * dq[i];
  }
  
  // initializing coriolis and centrifugal force matrix
  data = C.ptr<double>();
  double * mdata = M.ptr<double>();
  double mxxu = mxx * _v[0];
  double myyv = myy * _v[1];
  double mxgr = mxg * _v[2];
  double mygr = myg * _v[2];
  double nyr = ny * _v[2];
  data[2] = -mxgr - myyv + nyr; // c02
  data[5] = -mygr + mxxu;       // c12
  data[6] = -data[2];           // c20
  data[7] = -data[5];           // c21
  
  // initializing force vector T
  Mat T(3, 1, CV_64FC1, f);
  
  // Calculating next velocity
  // Mv'+(C+Dl+Dq)v=T
  // v'=M^-1 * (T-(C+Dl+Dq)v)
  // v=v+v'dt
  Mat V(3, 1, CV_64FC1, _v);
  Mat Vnext;
  Vnext = V + M.inv() * (T - (C + Dl + Dq) * V) * dt;
  
  // Updating velocity
  data = Vnext.ptr<double>();
  _vnew[0] = data[0];
  _vnew[1] = data[1];
  _vnew[2] = data[2];
}

const char * c_model_engine_ctrl::_str_par[num_params] =
  {
    "fth", "bth", "umax", "umin",
    "rgamma", "rfdelta", "rbdelta", "fslack", "bslack"    
  };

const char * c_model_engine_ctrl::_str_par_exp[num_params] =
  {
    "Threshold, neutral to forward", "Threshold neutral to backward",
    "Maximum value of control input", "Minimum value of control input",
    "Speed of gear switching (rate per second)", "Speed of throttle control in forward gear (rate per second)", "Speed of throttle control in backward gear (rate per second)", "Throttle slack in forward gear", "Throttle slack in backward gear"    
  };

void c_model_engine_ctrl::update(const int u, const float gamma,
				 const float delta,
				 const float slack, const float dt,
				 float & gamma_new, float & delta_new,
				 float & slack_new)
{
  e_gear_state gs, gs_inf;
  if(gamma == -1.0){
    gs = gs_b;
  }else if(gamma == 1.0){
    gs = gs_f;
  }else{
    gs = gs_n;
  }

  // determining action mode, and final (gamma, delta) for the input u.
  double unorm, rdelta;
  double gamma_inf, delta_inf, slack_inf;
  if(u <= bth){
    gs_inf = gs_b;
    unorm = (bth - u) /  (bth - umin);
    gamma_inf = -1.0;
  }if (u >= fth){
    gs_inf = gs_f;
    unorm = (u - fth) / (umax - fth);
    gamma_inf = 1.0;
  }else{
    gs_inf = gs_n;
    unorm = 0;
    gamma_inf = 0.0;      
  }

  e_action_mode am;
  switch(gs){
  case gs_f:
    switch(gs_inf){
    case gs_n:
    case gs_b:
      delta_inf = 0.f;
      if(delta == 0.f){
	am = am_fn;
      }else{
	am = am_fd;
	rdelta = rfdelta;
	delta_inf = 0.f;
	slack_inf = 0.f;
      }
      break;
    case gs_f:
      delta_inf = unorm;
      rdelta = rfdelta;
      if(delta_inf > delta){
	am = am_fu;
	slack_inf = fslack;	    
      }else if(delta_inf < delta){
	am = am_fd;
	slack_inf = 0.f;
      }else{
	am = am_none;
      }      
      break;
    }
    break;
  case gs_n:
    switch(gs_inf){
    case gs_f:
      am = am_nf;
      break;
    case gs_n:
      am = am_none;
      break;
    case gs_b:
      am = am_nb;
      break;
    }
    break;
  case gs_b:
    switch(gs_inf){
    case gs_f:
    case gs_n:
      if(delta == 0.f){
	am = am_bn;
      }else{
	am = am_bd;
	rdelta = rbdelta;
	delta_inf = 0.f;
	slack_inf = 0.f;
      }
      
      break;
    case gs_b:
      delta_inf = unorm;
      rdelta = rbdelta;
      if(delta_inf > delta){
	am = am_bu;
	slack_inf = bslack;
      }else if(delta_inf < delta){
	am = am_bd;
	slack_inf = 0.0;
      }else{
	am = am_none;
      }
      break;
    }
    break;
  }

  // calculating next (gamma, delta, slack)
  double dgamma, egamma, eslack, ddelta, edelta;
  switch(am){
  case am_fn:
  case am_nf:
  case am_bn:
  case am_nb:
    // gamma -> gamma_inf
    dgamma = rgamma * dt;
    egamma = gamma_inf - gamma;
    if(abs(egamma) < dgamma){
      gamma_new = (float)(gamma_inf);	
    }else if(egamma < 0){
      gamma_new = (float)(gamma - rgamma * dt);
    }else{
      gamma_new = (float)(gamma + rgamma * dt);
    }
    break;
  case am_fu:
  case am_fd:
  case am_bu:
  case am_bd:
    // delta, slack -> delta_inf, slack_inf
    ddelta = rdelta * dt;
    eslack = slack_inf - slack;
    edelta = delta_inf - delta;
    
    if(abs(eslack) < ddelta){
      slack_new = (float)(slack_inf);
    }else if(eslack < 0){
      slack_new = (float)(slack + ddelta);
    }else{
      slack_new = (float)(slack - ddelta);
    }
    
    if(abs(edelta) < ddelta){
      delta_new = (float)delta_inf;
    }else if(edelta < 0){
      delta_new = (float)(delta + ddelta);
    }else{
      delta_new = (float)(delta - ddelta);
    }     
    break;
  }    
}

const char * c_model_rudder_ctrl::_str_par[num_params] =
  {
    "rslack", "rrud", "ruds", "rudp"
  };

const char * c_model_rudder_ctrl::_str_par_exp[num_params] =
  {
    "Rudder slack",
    "Speed of rudder rotation (rate per second)",
    "Full starboard rudder angle (negative value).",
    "Full port rudder angle (positive value)"
  };

void c_model_rudder_ctrl::update(const int u, const float ra,
				 const float slack, const float dt,
				 float & ra_new, float & slack_new)
{
  // Note that rudder control input u [0,255] increases in starboard
  double alpha = (double) u * (1.0f / 255.f);
  double ra_inf = ruds * alpha + rudp * (1.0 - alpha);
  double dra = rrud * dt;
  double era = ra_inf - ra;
  if(abs(era) < dra){
    ra_new = (float)ra_inf;
    slack_new = (float)(slack + era);
  }else if(era < 0){    
    ra_new = (float)(ra - dra);
    slack_new = (float)(slack - dra);
  }else if(era > 0){
    ra_new = (float)(ra + dra);
    slack_new = (float)(slack + dra);
  }

  if(rslack < 0){
    slack_new = min((float)rslack, slack_new);
    slack_new = max(0.f, slack_new);
  }else{
    slack_new = max((float)rslack, slack_new);
    slack_new = min(0.f, slack_new);
  }
}

const char * c_model_outboard_force::_str_par[num_params] = {
  "xr",
  "yr",
  "CTL",
  "CTQ",
  "CD",
  "CL"
};

const char * c_model_outboard_force::_str_par_exp[num_params] = {
  "Rudder center in x",
  "Rudder center in y",
  "Linear coefficient for thrust force model",
  "Quadratic coefficient for thrust force model",
  "Rudder drag force coefficient",
  "Rudder lift force coefficient"
};

void c_model_outboard_force::update(const double _rud, const double _gear,
				    const double _thro, const double _rev,
				    const double * v, double * f)
{
  // update rev
  // calculate rudder angle (port positive) and direction vector (nrx,nry)
  double nrx = cos(_rud), nry = sin(_rud);
  
  // calculate velocity of rudder center
  double vx = v[0] + v[2] * yr, vy = v[1] + v[2] * xr;
  double va2 = vx * vx + vy * vy, va = sqrt(va2);
  double iva = 1.0 / va;
  double nvx = vx * iva, nvy = vy * iva; // normal velocity vector at rudder 
  
  double vr = vx * nrx + vy * nry;         // rudder speed along rudder
  double vrx = vr * nrx, vry = vr * nry;   // rudder velocity along rudder
  double vrox = vx - vrx, vroy = vy - vry; // rudder velocity perpendicular to rudder
  double vro = -vx * nry + vy * nrx;
  
  // calculate x, y thrust force T(v, rev), and
  // decompose Tx = T cos phi, Ty=T sin phi
  double T = (CTL * vr * _rev + CTQ * _rev * _rev) * _gear;
  double Tx = nrx * T, Ty = nry * T;
  
  // flow to rudder angle psi
  // double psi = acos(cpsi);
  double cpsi = vr * iva;
  double spsi = vro * iva;
  
  // calculate disturbance D and lift L
  double D = -CD * va2 * spsi;
  double Dx = D * nvx, Dy = D * nvy;
  
  double L = 
    - (nvx * nrx + nvy * nry > 0  ? 1.0 /*forward*/: -1.0/*backward*/) * 
    (-nvx * nry + nvy * nrx > 0 ? 1.0 /* port */: -1.0/*starboard*/) * 
    CL * va2 * spsi;
  double Lx = - L * nvy, Ly = L * nvx;
  
  // Forces in x, y axes
  f[0] = Tx + Dx + Lx;
  f[1] = Ty + Dy + Ly;
  
  // calculate moment xr * Ty + yr * Ty
  f[2] = xr * f[1] + yr * f[0];
}


/////////////////////////////////////////////////////////////////////////// f_aws1_sim members

f_aws1_sim::f_aws1_sim(const char * name) :
	f_base(name),
	m_state(NULL), m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL),
	m_ch_ctrl_stat(NULL),
	m_state_sim(NULL), m_engstate_sim(NULL), m_ch_ctrl_stat_sim(NULL),
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
  
  m_fcsv_out[0] = '\0';
  register_fpar("fcsv", m_fcsv_out, 1024, "CSV output file.");

  mrctrl.register_param(this);
  mectrl.register_param(this);
  mobf.register_param(this);
  m3dof.register_param(this);
}

bool f_aws1_sim::init_run()
{
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
  

  m3dof.init();
  
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
			       m_ctrl_stat.rud_max, m_ctrl_stat.rud_nut,
			       m_ctrl_stat.rud_min);
    m_ctrl_stat.meng = map_oval(m_ctrl_stat.meng_aws,
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_ctrl_stat.meng_max, m_ctrl_stat.meng_nuf,
				m_ctrl_stat.meng_nut,
				m_ctrl_stat.meng_nub, m_ctrl_stat.meng_min);
    m_ctrl_stat.seng = map_oval(m_ctrl_stat.seng_aws,
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_ctrl_stat.seng_max, m_ctrl_stat.seng_nuf,
				m_ctrl_stat.seng_nut,
				m_ctrl_stat.seng_nub, m_ctrl_stat.seng_min);
    break;
  case ACS_RMT:
    m_ctrl_stat.rud = map_oval(m_ctrl_stat.rud_rmc,
			       m_ctrl_stat.rud_max_rmc, m_ctrl_stat.rud_nut_rmc,
			       m_ctrl_stat.rud_min_rmc,
			       m_ctrl_stat.rud_max, m_ctrl_stat.rud_nut, m_ctrl_stat.rud_min);
    m_ctrl_stat.meng = map_oval(m_ctrl_stat.meng_rmc,
				m_ctrl_stat.meng_max_rmc,
				m_ctrl_stat.meng_nuf_rmc,
				m_ctrl_stat.meng_nut_rmc,
				m_ctrl_stat.meng_nub_rmc,
				m_ctrl_stat.meng_min_rmc,
				m_ctrl_stat.meng_max, m_ctrl_stat.meng_nuf,
				m_ctrl_stat.meng_nut, m_ctrl_stat.meng_nub,
				m_ctrl_stat.meng_min);
    m_ctrl_stat.seng = map_oval(m_ctrl_stat.seng_rmc,
				m_ctrl_stat.seng_max_rmc,
				m_ctrl_stat.seng_nuf_rmc,
				m_ctrl_stat.seng_nut_rmc,
				m_ctrl_stat.seng_nub_rmc,
				m_ctrl_stat.seng_min_rmc,
				m_ctrl_stat.seng_max,
				m_ctrl_stat.seng_nuf,
				m_ctrl_stat.seng_nut,
				m_ctrl_stat.seng_nub,
				m_ctrl_stat.seng_min);
    break;
  }
  if (m_ch_ctrl_stat_sim){
    m_ch_ctrl_stat_sim->set(m_ctrl_stat);
  }
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
	m_sv_cur.roll = roll * (PI / 180.f);
	m_sv_cur.pitch = pitch * (PI / 180.f);
	m_sv_cur.yaw = yaw * (PI / 180.f);
	m_sv_cur.cog = cog * (PI / 180.f);
	m_sv_cur.sog = sog;
	m_sv_cur.lat = lat * (PI / 180.f);
	m_sv_cur.lon = lon * (PI / 180.f);
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
	  m_engstate->get_dynamic(t, poil, toil, temp, valt, frate,
				  teng, pclnt, pfl, steng1, steng2, ld, tq);
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
      m_engstate->get_dynamic(t, poil, toil, temp, valt, frate,
			      teng, pclnt, pfl, steng1, steng2, ld, tq);
      m_engstate_sim->set_dynamic(sv.t, poil, toil, temp, valt, sv.fuel,
				  teng, pclnt, pfl, steng1, steng2, ld, tq);
    }
  
  if (m_state_sim)
    {
      float alt = 0.f, galt = 0.f;
      // output simulated lat, lon, roll, pitch, yaw, cog, sog
      m_state_sim->set_attitude(sv.t, sv.roll * (180.f / PI), sv.pitch * (180.f / PI), sv.yaw * (180.f / PI));
      m_state_sim->set_position(sv.t, sv.lat * (180.f / PI), sv.lon * (180.f / PI), alt, galt);
      m_state_sim->set_velocity(sv.t, sv.cog * (180.f / PI), sv.sog);
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
  double dt = (double) m_int_smpl / (double) SEC;
  for (int iosv = 0; iosv < m_wosmpl; iosv++) {		
    s_state_vector & stprev =
      (iosv == 0 ? m_input_vectors[m_iv_head] : m_output_vectors[iosv-1]);
    s_state_vector & stcur = m_output_vectors[iosv];
    
    // simulate actuator and pump  
    double v[3];
    double f[3];
    double phi = (stprev.cog - stprev.yaw);
    double th = stprev.cog;
    
    double sog_ms = stprev.sog * (1852. / 3600.);
    double dx = sog_ms * dt * sin(th), dy = sog_ms * dt * cos(th); //next position in enu coordinate
    double alt = 0.;
    wrldtoecef(stprev.Rwrld, stprev.xe, stprev.ye, stprev.ze, dx, dy, 0.,
	       stcur.xe, stcur.ye, stcur.ze);
    eceftobih(stcur.xe, stcur.ye, stcur.ze, stcur.lat, stcur.lon, alt);
    
    v[0] = sog_ms * cos(phi);
    v[1] = sog_ms * sin(phi);
    v[2] = stprev.ryaw;

    mrctrl.update(stprev.rud, stprev.rud_pos, stprev.rud_slack, dt,
		  stcur.rud_pos, stcur.rud_slack);
    mectrl.update(stprev.eng, stprev.gear_pos, stprev.thro_pos,
		  stprev.thro_slack, dt,
		  stcur.gear_pos, stcur.thro_pos, stcur.thro_slack);
    mobf.update(stprev.rud_pos - stprev.rud_slack,
		stprev.gear_pos, stprev.thro_pos - stprev.thro_slack,
		stprev.rev, v, f);    
    m3dof.update(v, f, dt, v);
    
    phi = atan2(v[0], v[1]);
    stcur.yaw += v[2] * dt * (180. / PI);
    stcur.cog = stcur.yaw + phi * (180. / PI);
    stcur.sog = sqrt(v[0] * v[0] + v[1] * v[1]) * (3600. / 1852.);
    stcur.rev = final_rev(stcur.thro_pos);
  }
}

void f_aws1_sim::save_csv(const long long tcur)
{
  s_state_vector & svo = m_output_vectors[0];
  s_state_vector & svi = m_input_vectors[m_iv_head];
  m_fcsv << tcur << ",";
  
  m_fcsv.precision(8);
  m_fcsv <<
    svo.lat * (180.f/PI) << "," <<
    svo.lon * (180.f/PI) << "," <<
    svo.xe << "," <<
    svo.ye << "," <<
    svo.ze << ",";
  
  m_fcsv.precision(3);
  m_fcsv <<
    svo.roll * (180.f/PI)<< "," <<
    svo.pitch * (180.f/PI)<< "," <<
    svo.yaw  * (180.f/PI)<< "," <<
    svo.sog << "," <<
    svo.cog  * (180.f/PI)<< "," <<
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
    svi.lat * (180.f/PI) << "," <<
    svi.lon * (180.f/PI) << "," <<
    svi.xe << "," <<
    svi.ye << "," <<
    svi.ze << ",";
  
  m_fcsv.precision(3);
  m_fcsv <<
    svi.roll * (180.f/PI) << "," <<
    svi.pitch * (180.f/PI) << "," <<
    svi.yaw * (180.f/PI) << "," <<
    svi.sog << "," <<
    svi.cog * (180.f/PI) << "," <<
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

