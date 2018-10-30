#ifndef F_AWS1_SIM_H
#define F_AWS1_SIM_H
// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_aws1_sim.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_sim.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_sim.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "f_base.h"

#include "../channel/ch_state.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_aws1_sys.h"

#define RUD_PER_CYCLE 0.45f

class f_aws1_sim;

///////////////////////////////// base of  simulation model for boat parts
class c_model_base
{
 private:
  char ** str_param;
  int index;
  
  virtual const char * get_str_param(int iparam) = 0;
  virtual const char * get_str_param_exp(int iparam) = 0;
  virtual double * get_param(int iparam) = 0;
  
  virtual int get_num_params() = 0;
  
  char * gen_str_indexed_param(int iparam);
  
 public:
 c_model_base():str_param(NULL), index(-1)
    {
    }
  
  virtual ~c_model_base()
    {
      release_param();
    }
  
  bool register_param(f_aws1_sim * psim, int _index = -1);

  void release_param();
};


//////////////////////////////////// 3dof kinetic model
class c_model_3dof: public c_model_base
{
 private:
  double xg, yg;
  double m;     // mass
  double ma[9]; // added mass matrix
  double dl[9]; // linear drag matrix
  double dq[9]; // quadratic drag matrix
  
  Mat M;
  Mat Minv;
  Mat Dl;
  Mat Dq;
  Mat C;

  double iz;
  double mxx;
  double myy;
  double mxg, myg;
  double ny;
  
  enum {
    par_xg, par_yg,
    par_m,
    par_ma_xu, par_ma_yv, par_ma_yr, par_ma_nv, par_ma_nr,
    par_dl_xu, par_dl_yv, par_dl_yr, par_dl_nv, par_dl_nr,
    par_dq_xu, par_dq_yv, par_dq_yr, par_dq_nv, par_dq_nr,
    num_params
  };
  static const char * _str_par[num_params];
  static const char * _str_par_exp[num_params];
  virtual const char * get_str_param(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par[iparam];
  }

  virtual const char * get_str_param_exp(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par_exp[iparam];
  }
  
  virtual double * get_param(int iparam)
  {
    switch(iparam){
    case par_xg:
      return &xg;
    case par_yg:
      return &yg;
    case par_m:
      return &m;
    case par_ma_xu:
      return ma;
    case par_ma_yv:
      return ma+4;
    case par_ma_yr:
      return ma+5;
    case par_ma_nv:
      return ma+7;
    case par_ma_nr:
      return ma+8;
    case par_dl_xu:
      return dl;
    case par_dl_yv:
      return dl+4;
    case par_dl_yr:
      return dl+5;
    case par_dl_nv:
      return dl+7;
    case par_dl_nr:
      return dl+8;
    case par_dq_xu:
      return dq;
    case par_dq_yv:
      return dq+4;
    case par_dq_yr:
      return dq+5;
    case par_dq_nv:
      return dq+7;
    case par_dq_nr:
      return dq+8;
    }
    return NULL;
  }
  
  virtual int get_num_params()
  {
    return num_params;
  }

 public:
 c_model_3dof():xg(0), yg(0)
    {
      for (int i = 0; i < 9; i++)
	ma[i] = dl[i] = dq[i] = 0;
    }
  
  virtual ~c_model_3dof()
    {
    }  
  
  void init();  
  void update(double * _v,
	      double * f /* x-y force and z moment applied */,
	      const double dt /* time step */, double * _vnew);
};

////////////////////////////////////////////// rudder control model
class c_model_rudder_ctrl: public c_model_base
{
 private:
  double rslack, rrud, ruds, rudp;
  enum {
    par_rslack, par_rrud, par_ruds, par_rudp,
    num_params
  };
  static const char * _str_par[num_params];
  static const char * _str_par_exp[num_params];
  virtual const char * get_str_param(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par[iparam];
  }

  virtual const char * get_str_param_exp(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par_exp[iparam];
  }
  
  virtual double * get_param(int iparam)
  {
    switch(iparam){
    case par_rslack:
      return &rslack;
    case par_rrud:
      return &rrud;
    case par_ruds:
      return &ruds;
    case par_rudp:
      return &rudp;
    }
    return NULL;
  }
  
  virtual int get_num_params()
  {
    return num_params;
  }

 public:
 c_model_rudder_ctrl():rslack(0.f), rrud(12*PI/180.f),
    rudp(30*PI/180.f), ruds(-30*PI/180.f)
    {
    }

  virtual ~c_model_rudder_ctrl()
    {
    }
  
  void update(const int u,
	      const float ra, const float slack,
	      const float dt,
	      float & ra_new, float & slack_new);
};


///////////////////////////////////// engine control model
class c_model_engine_ctrl: public c_model_base
{
 private:
  double fth, bth, umax, umin;
  double rgamma, rfdelta, rbdelta, fslack, bslack;
  enum {
    par_fth, par_bth, par_umax, par_umin,
    par_rgamma, par_rfdelta, par_rbdelta, par_fslack, par_bslack,
    num_params
  };
  static const char * _str_par[num_params];
  static const char * _str_par_exp[num_params];
  
  virtual const char * get_str_param(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par[iparam];
  }

  virtual const char * get_str_param_exp(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par_exp[iparam];
  }
  
  virtual double * get_param(int iparam)
  {
    switch(iparam){
    case par_fth:
      return &fth;
    case par_bth:
      return &bth;
    case par_umax:
      return &umax;
    case par_umin:
      return &umin;
    case par_rgamma:
      return &rgamma;
    case par_rfdelta:
      return &rfdelta;
    case par_rbdelta:
      return &rbdelta;
    case par_fslack:
      return &fslack;
    case par_bslack:
      return &bslack;      
    }
    return NULL;
  }

  virtual int get_num_params()
  {
    return num_params;
  }
    
 public:
 c_model_engine_ctrl():fth(0x7f+0x19), bth(0x7f - 0x19),
    umax(0x00ff), umin(0x0000),
    rgamma(0.5), rfdelta(0.5), rbdelta(0.5), fslack(0.05), bslack(0.05)
    {      
    }

  virtual ~c_model_engine_ctrl()
    {
    }
  
  // u: control input [0,255]
  // gamma: gear position [-1,1]; Forward: 1, Backward: -1, Neutral: 0
  // delta: throttle position [0,1]
  // slack: slack in throttle position
  //        Note that actual throttle position is delta - slack
  // dt: time step in second
  // (ganmma_new, delta_new, slack_new): updated (gamma, delta, slack)
  void update(const int u, const float gamma, const float delta,
	      const float slack, const float dt,
	      float & gamma_new, float & delta_new, float & slack_new);
};

//////////////////////////////////////// force model for outboard mortor
class c_model_outboard_force: public c_model_base
{
 private:
  double xr, yr; // rudder position from rotation center
  double CTL, CTQ; // linear and quadratic coefficient for thrust force model
  double CD, CL; // rudder drag and lift coefficient
  enum {
    par_xr, par_yr, par_CTL, par_CTQ, par_CD, par_CL, num_params
  };
  static const char * _str_par[num_params];
  static const char * _str_par_exp[num_params];
  virtual const char * get_str_param(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par[iparam];
  }

  virtual const char * get_str_param_exp(int iparam)
  {
    if(iparam >= num_params || iparam < 0)
      return NULL;
    return _str_par_exp[iparam];
  }
  
  virtual double * get_param(int iparam)
  {
    switch(iparam){
    case par_xr:
      return &xr;
    case par_yr:
      return &yr;
    case par_CTL:
      return &CTL;
    case par_CTQ:
      return &CTQ;
    case par_CD:
      return &CD;
    case par_CL:
      return &CL;
    }
  }
  
  virtual int get_num_params()
  {
    return num_params;
  }

 public:
 c_model_outboard_force() :xr(0.f), yr(-3.f), CTL(0.), CTQ(0.), CD(0.), CL(0.)
    {
    }
  
  virtual ~c_model_outboard_force()
    {
    }

  void update(const double _rud, const double _gear,
	      const double _thro, const double _rev, const double * v,
	      double * f);
};


//////////////////////////////////////////////////////// f_aws1_sim
class f_aws1_sim : public f_base
{
protected:
  // simulation models
  c_model_rudder_ctrl mrctrl; // rudder control model
  c_model_engine_ctrl mectrl; // engine control model
  c_model_outboard_force mobf, mobfp; // engine force model, later is for the planing mode
  c_model_3dof m3dof, m3dofp;         // kinetic model, later is for the planing mode
  double vplane; // planing velocity
  
  // input channels
  ch_state * m_state;
  ch_eng_state * m_engstate;
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;
  ch_aws1_ctrl_inst * m_ch_ctrl_ui, *m_ch_ctrl_ap1, *m_ch_ctrl_ap2;
  
  s_aws1_ctrl_stat m_ctrl_stat;
  
  // output channels
  ch_state * m_state_sim;
  ch_eng_state * m_engstate_sim;
  ch_aws1_ctrl_stat * m_ch_ctrl_stat_sim;
  
  struct s_state_vector
  {
    long long t;
    double lat, lon, xe, ye, ze, roll, pitch, yaw, cog, sog, ryaw;		
    Mat Rwrld;
    float eng, rud, rev, fuel; 
    float thro_pos, thro_slack, gear_pos, rud_pos, rud_slack;
    
  s_state_vector(const long long & _t,
		 const double & _lat, const double & _lon,
		 const double & _roll, const double & _pitch,
		 const double & _yaw,
		 const double & _cog, const double & _sog,
		 const float & _eng, const float & _rud,
		 const float & _rev, const float & _fuel) :
    t(_t), lat(_lat), lon(_lon),
      roll(_roll), pitch(_pitch), yaw(_yaw), cog(_cog), sog(_sog),
      eng(_eng), rud(_rud), rev(_rev), fuel(_fuel), ryaw(0)
    {
      update_coordinates();
    }
    
  s_state_vector() :lat(135.f), lon(35.f),
      roll(0.f), pitch(0.f), yaw(0.f), cog(0.f), sog(0.f),
      eng(127.0f), rud(127.0f), rev(700.f), fuel(0.1f),
      thro_pos(0.f), thro_slack(0.f), gear_pos(0.f),
      rud_pos(0.f), rud_slack(0.f)
    {
      update_coordinates();
    }
    
    void update_coordinates() {
      bihtoecef(lat, lon, 0, xe, ye, ze);
      getwrldrot(lat, lon, Rwrld);			
    }

    void print()
    {
      cout << " rud:" << rud << " rud_pos:" << rud_pos
	   << " rud_slack:" << rud_slack;
      cout << " eng: " << eng << " gear_pos:" << gear_pos
	   << " thro_pos: " << thro_pos << " thro_slack:" << thro_slack;
      cout << " rev:" << rev;
      cout << " yaw:" << yaw;
      cout << " cog:" << cog;
      cout << " sog:" << sog;
    }
  };

  float m_int_smpl_sec;
  unsigned int m_int_smpl; // sampling interval (m_int_smpl_sec * 10e7)
  unsigned int m_iv_head;
  unsigned int m_wismpl; // inputs past m_wismpl * m_int_smpl secs are hold 
  vector<s_state_vector> m_input_vectors; // time sequence of  input vectors
  unsigned int m_wosmpl; // outputs next m_wismpl * m_int_smpl secs are calculated
  vector<s_state_vector> m_output_vectors; // time sequence of output vectors
  
  s_state_vector m_sv_init, m_sv_cur;
  void init_input_sample();
  void update_input_sample();
  void init_output_sample();
  void update_output_sample(const long long & tcur);
  
  // time of previous sampling 
  long long m_tprev;
  
  void set_control_input();
  void set_control_output();
  void set_input_state_vector(const long long & tcur);
  void set_output_state_vector();
  
  void simulate(const long long tcur, const int iosv);
  void simulate_rudder(const float rud, const float rud_pos, float & rud_pos_next);
  void simulate_engine(const float eng, const float eng_pos, const float gear_pos, float & eng_pos_next, float & gear_pos_next);
  const float final_rev(const float thr_pos)
	{
	  if (thr_pos < 0.417)
	    return 700;
	  if (thr_pos < 0.709)
	    return (5000 - 700) * (thr_pos - 0.417) / (0.709 - 0.417) + 700;
	  if (thr_pos < 0.854)
	    return (5600 - 5000) * (thr_pos - 0.709) / (0.854 - 0.709) + 5000;
		return 5600;
	}
   
  bool m_bcsv_out;
  char m_fcsv_out[1024];
  ofstream m_fcsv;
  void save_csv(const long long tcur);
 public:
  f_aws1_sim(const char * name);

  void register_model_params(const char * par_str, double * par,
			     const char * par_exp_str)
  {
    register_fpar(par_str, par, par_exp_str);
  }
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif
