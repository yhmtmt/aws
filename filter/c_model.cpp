#include "stdafx.h"
#include <iostream>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_coord.h"

#include "c_model.hpp"

// Copyright(c) 2018 Yohei Matsumoto, All right reserved. 

// c_model.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_sim.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_model.hpp.  If not, see <http://www.gnu.org/licenses/>. 


///////////////////////////////// base of  simulation model for boat parts
bool c_model_base::alloc_param(int _index)
{
  index = _index;
  if (index < 0)
    return true;

  int n = get_num_params();
  str_param = new char*[n];
  for(int ipar = 0; ipar < n; ipar++){
    str_param[ipar] = gen_str_indexed_param(ipar);
  }  
}

char * c_model_base::gen_str_indexed_param(int iparam)
{
  if(index >= 10 || index < 0)
    return NULL;
  
  const char * base_str_param = _get_str_param(iparam);
  if(!base_str_param)
    return NULL;
  
  int len = strlen(base_str_param) + 2;
  char * indexed_str_param = new char[len];
  snprintf(indexed_str_param, len, "%s%d", base_str_param, index);
  return indexed_str_param;
}

void c_model_base::release_param()
{
  if(index == -1)
    return;
  
  if(str_param){
    int npars = get_num_params();
    for(int ipar = 0; ipar < npars; ipar++){
      delete[] str_param[ipar];
    }
    delete[] str_param;
    str_param = NULL;
  }
}  

//////////////////////////////////// 3dof kinetic model
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

void c_model_3dof::init()
{
  M = Mat::zeros(3, 3, CV_64FC1);
  double * data = M.ptr<double>();
  
  for (int i = 0; i < 9; i++)
    data[i] = ma[i];
  data[0] += m;
  data[4] += m;
  iz = m * (xg * xg + yg * yg);
  data[8] += iz;
  
  mxx = data[0];
  myy = data[4];
  mxg = m * xg;
  myg = m * yg;
  ny = (ma[5] + ma[7]) * 0.5;
  
  Dl = Mat::zeros(3, 3, CV_64FC1);
  data = Dl.ptr<double>();
  for (int i = 0; i < 9; i++) {
    data[i] = dl[i];		
  }
  
  Dq = Mat::zeros(3, 3, CV_64FC1);
  C = Mat::zeros(3, 3, CV_64FC1);
}

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
  Mat R1 = (C + Dl + Dq) * V;
  Mat R2 = T - R1;
  Mat R3 = M.inv() * R2;
  Mat R4 = R3 * dt;
  Vnext = V + R3 * dt;
  /*
  cout << "V:" << V << endl;
  cout << "Vnext:" << Vnext << endl;
  cout << "T:" << T << endl;
  cout << "C:" << C << endl;
  cout << "Dl:" << Dl << endl;
  cout << "Dq:" << Dq << endl;
  cout << "M:" << M << endl;
  cout << "Mi:" << M.inv() << endl;
  cout << "R1:" << R1 << endl;
  cout << "R2:" << R2 << endl;
  cout << "R3:" << R3 << endl;
  cout << "R4:" << R4 << endl;
  cout << "dt:" << dt << endl;
  */
  // Updating velocity
  data = Vnext.ptr<double>();
  _vnew[0] = data[0];
  _vnew[1] = data[1];
  _vnew[2] = data[2];
}

////////////////////////////////////////////// rudder control model
const char * c_model_rudder_ctrl::_str_par[num_params] =
  {
    "rslack", "rrud", "ruds", "rudp"
  };

const char * c_model_rudder_ctrl::_str_par_exp[num_params] =
  {
    "Rudder slack (degree)",
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
    slack_new = (float)(slack - era);
  }else if(era < 0){    
    ra_new = (float)(ra - dra);
    slack_new = (float)(slack + dra);
  }else if(era > 0){
    ra_new = (float)(ra + dra);
    slack_new = (float)(slack - dra);
  }

  if(rslack < 0){
    slack_new = max((float)rslack, slack_new);
    slack_new = min(0.f, slack_new);
  }else{
    slack_new = min((float)rslack, slack_new);
    slack_new = max(0.f, slack_new);
  }
}

///////////////////////////////////// engine control model
const char * c_model_engine_ctrl::_str_par[num_params] =
  {
    "fth", "bth", "umax", "umin",
    "rgamma", "rfdelta", "rbdelta",
    "e0f", "e0df", "epf", "epdf", "eff", "r0f", "rpf", "rff", "qddf", "qudf", "qdpf", "qupf",
    "e0b", "e0db", "efb", "r0b", "rfb", "qdb", "qub"    
  };

const char * c_model_engine_ctrl::_str_par_exp[num_params] =
  {
    "Threshold, neutral to forward", "Threshold neutral to backward",
    "Maximum value of control input", "Minimum value of control input",
    "Speed of gear switching (rate per second)", "Speed of throttle control in forward gear (rate per second)", "Speed of throttle control in backward gear (rate per second)",
    "Forward engine control, idle point (up control)",
    "Forward engine control, idle point (down control)",
    "Forward engine control, planing point (up control)",
    "Forward engine control, planing point (down control)",
    "Forward engine control, full point",
    "Forward engine rev, idle point",
    "Forward engine rev, planing point",
    "Forward engine rev, full point",
    "Quadratic coefficient, forward-down control curve, displacement mode",
    "Quadratic coefficient, forward-up control curve, displacement mode",
    "Quadratic coefficient, forward-down control curve, planing mode",
    "Quadratic coefficient, forward-up control curve, planing mode",
    "Backward engine control, idle point (up control)",
    "Backward engine control, idle point (down control)",
    "Backward engine control, full point",
    "Quadratic coefficient, backward-down control curve",
    "Quadratic coefficeint, backward-up control curve"
  };

void c_model_engine_ctrl::init()
{
  double sf = umax-fth;
  double sb = umin-bth;
  
  _e0f=(e0f-fth)/sf;
  _e0df=(e0df-fth)/sf;
  _epf=(epf-fth)/sf;
  _epdf=(epdf-fth)/sf;
  _eff=(eff-fth)/sf;
  _qddf=sf*sf*qddf;
  _qudf=sf*sf*qudf;
  _qdpf=sf*sf*qdpf;
  _qupf=sf*sf*qupf;
  calc_qeq_coeff(_e0df, r0f, _epdf, rpf, _qddf, lddf, cddf);
  calc_qeq_coeff(_e0f, r0f, _epf, rpf, _qudf, ludf, cudf);
  calc_qeq_coeff(_epdf, rpf, _eff, rff, _qdpf, ldpf, cdpf);
  calc_qeq_coeff(_epf, rpf,  _eff, rff, _qupf, lupf, cupf);
  
  _e0b=(e0b-bth)/sb;
  _e0db=(e0db-bth)/sb;
  _efb=(efb-bth)/sb;
  _qdb=sb*sb*qdb;
  _qub=sb*sb*qub;
  calc_qeq_coeff(_e0db,r0b,_efb,rfb,_qdb, ldb, cdb);
  calc_qeq_coeff(_e0b,r0b,_efb,rfb, _qub, lub, cub);
}

void c_model_engine_ctrl::calc_qeq_coeff(const double x0, const double y0,
					 const double x1,const double y1,
					 const double cq,
					 double & cl, double & cc)
{
  double base = 1 / (x1-x0);
  double x0x0 = x0 * x0;
  double x1x1 = x1 * x1;
  cl = (cq * (x0x0 - x1x1) - y0 + y1) * base;
  cc = -(-cq * x0 * x1x1 + x1 * (cq * x0x0 - y0) + x0 * y1) * base;
}

void c_model_engine_ctrl::calc_final_rev(const double gamma, const double delta, bool is_ctrl_up, const float rev, float & rev_new)
{
  if(gamma >= 1.0){ // forward
    if(is_ctrl_up){ // up ctrl
      if(delta < _e0f) // idle
	rev_new = (float)r0f;
      else if(delta < _epf) // displacement
	rev_new = (float)(_qudf * delta * delta + ludf * delta + cudf);
      else if(delta < _eff) // planing
	rev_new = (float)(_qupf * delta * delta + lupf * delta + cupf);
      else
	rev_new = (float)rff;
      
      if(rev_new < rev) // down to up transient state
	rev_new = rev;
    }else{ // down ctrl
      if(delta < _e0df)
	rev_new = r0f;
      else if (delta < _epdf)
	rev_new = (float)(_qddf * delta * delta + lddf * delta + cddf);
      else if (delta < _eff)
	rev_new = (float)(_qdpf * delta * delta + ldpf * delta + cdpf);
      else
	rev_new = (float)rff;
      
      if(rev_new > rev) // up to down transient state
	rev_new = (float)rev;
    }
  }else if(gamma <= -1.0){ // backward
    if(is_ctrl_up){ // up ctrl
      if(delta < _e0b) // idle
	rev_new = (float)r0b;
      else if(delta < _efb) 
	rev_new = (float)(_qub * delta * delta + lub * delta + cub);
      else
	rev_new = (float)rfb;
      
      if(rev_new > rev)
	rev_new = (float)rev;      
    }else{ // down ctrl
      if(delta < _e0db) //idle
	rev_new = (float)r0b;
      else if(delta < _efb)
	rev_new = (float)(_qdb * delta * delta + ldb * delta + cdb);
      else
	rev_new = (float)rfb;
      
      if(rev_new < rev)
	rev_new = (float)rev;
    }
  }else{ //neutral
    rev_new = 0.5 * (r0b + r0f);
  }
}

void c_model_engine_ctrl::update(const int u,
				 const float gamma, const float delta, const float rev,
				 const float dt,
				 float & gamma_new, float & delta_new, float & rev_new)
{
  gamma_new = gamma;// gear position
  delta_new = delta;// throttle position
  rev_new = rev; // rev value
  
  // determining action mode, and final (gamma, delta) for the input u.
  double unorm, rdelta;
  double gamma_inf, delta_inf;
  if(u <= bth){
    delta_inf = (gamma == -1.0 ? (bth - u) /  (bth - umin) : 0.0);
    gamma_inf = -1.0;
    rdelta = rbdelta;
  }else if (u >= fth){
    delta_inf = (gamma == 1.0 ? (u - fth) / (umax - fth) : 0.0);
    gamma_inf = 1.0;
    rdelta = rfdelta;
  }else{
    delta_inf = 0.0;
    gamma_inf = 0.0;
    rdelta = gamma < 0 ? rbdelta : rfdelta;
  }

  if(gamma != gamma_inf && delta == 0.0) // gear is moving
    {
      // gamma -> gamma_inf
      double dgamma = rgamma * dt;
      double egamma = gamma_inf - gamma;
      if(abs(egamma) < dgamma){
	gamma_new = (float)(gamma_inf);	
      }else if(egamma < 0){
	gamma_new = (float)(gamma - dgamma);
      }else{
	gamma_new = (float)(gamma + dgamma);
      }
      calc_final_rev(gamma_new, 0.0, true, rev, rev_new);
      return; 
    }

  if(delta_inf == delta) // no throttle change
    return;
  
  double ddelta = rdelta * dt;
  double edelta = delta_inf - delta;

  bool is_ctrl_up = false;
  if(gamma_inf < 0){ // backward
    is_ctrl_up = edelta < 0;
  }else if (gamma_inf > 0){ // forward
    is_ctrl_up = edelta > 0;
  }

  if(abs(edelta) < ddelta){
    delta_new = (float)delta_inf;
  }else if(edelta > 0){
    delta_new = (float)(delta + ddelta);
  }else{
    delta_new = (float)(delta - ddelta);
  }

  calc_final_rev(gamma_new, delta_new, is_ctrl_up, rev, rev_new);
}

//////////////////////////////////////// force model for outboard mortor
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

#ifdef PY_EXPORT
boost::python::tuple c_model_outboard_force::update_py(const double _rud, const double _gear,
				       const double _thro, const double _rev,
				       const double _u, const double _v, const double _r)
{
  double v[3]={_u, _v, _r};
  double f[3];
  update(_rud, _gear, _thro, _rev, v, f);
  return boost::python::make_tuple(f[0],f[1],f[2]);
}
#endif

void c_model_outboard_force::update(const double _rud, const double _gear,
				    const double _thro, const double _rev,
				    const double * v, double * f)
{
  double g = 0.0;
  if(_gear != 1.0 &&  _gear != -1.0){
    g = 0.0;
  }else{
    g = _gear;
  }
  // update rev
  // calculate rudder angle (port positive) and direction vector (nrx,nry)
  double nrx = cos(_rud), nry = sin(_rud);
  double nrox = -nry, nroy = nrx;
  
  // calculate velocity of rudder center
  double vrx = v[0] - v[2] * yr, vry = v[1] + v[2] * xr;
  double vrox = -vry, vroy = vrx;
  
  double vr2 = vrx * vrx + vry * vry, vr = sqrt(vr2);
  
  // calculate x, y thrust force T(v, rev), and
  // decompose Tx = T cos phi, Ty=T sin phi
  double T = (CTL * vr * _rev + CTQ * _rev * _rev) * g;
  double Tx = nrx * T, Ty = nry * T;
  
  // flow to rudder angle psi
  // double psi = acos(cpsi);
  double D, Dx, Dy, L, Lx, Ly;
  D = Dx = Dy = L = Lx = Ly = 0.0;

  f[0] = Tx;
  f[1] = Ty;
  
  if(vr != 0){
    // nr.vr
    double nrvr = nrx * vrx + nry * vry;
    double nrovr = nrox * vrx + nroy * vry;
   
    // calculate disturbance D and lift L
    D = - 0.5 * CD * abs(nrovr);
    Dx = D * vrx;
    Dy = D * vry;
    
    L = 
      - (nrvr > 0  ? 0.5 : -0.5) * CL * nrovr;
    Lx = L * vrox;
    Ly = L * vroy;
    
    f[0] += Dx + Lx;
    f[1] += Dy + Ly;
  }
  
  // calculate moment xr * Ty + yr * Ty
  f[2] = xr * f[1] + yr * f[0];
}
