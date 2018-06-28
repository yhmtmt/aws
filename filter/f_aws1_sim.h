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

//////////////////////////////////////////////////////// f_aws1_sim
#define RUD_PER_CYCLE 0.45f

class f_aws1_sim;

class c_model_outboard_force
{
 private:
  double xr, yr; // rudder position from rotation center
  double CTL, CTQ; // linear and quadratic coefficient for thrust force model
  double CD, CL; // rudder drag and lift coefficient
  enum {
    num_params = 6
  };
  static const char * _str_par[num_params];
  char * str_par[num_params];
 public:
 c_model_outboard_force() :xr(0.f), yr(-3.f), CTL(0.), CTQ(0.), CD(0.), CL(0.)
    {
      for(int i = 0; i < num_params; i++){
	str_par[i] = NULL;
      }
    }
  
  ~c_model_outboard_force()
    {
      for(int i = 0; i < num_params; i++){
	if(str_par[i])
	  delete[] str_par[i];
	str_par[i] = NULL;
      }
    }

  void register_params(f_aws1_sim * psim, int index = -1);
  
  void update(const double _rud, const double _gear,
	      const double _thro, const double _rev, const double * v,
	      double * f);
};

class c_model_3dof
{
 private:
  double xg, yg, iz;
  double m;     // mass
  double ma[9]; // added mass matrix
  double dl[9]; // linear drag matrix
  double dq[9]; // quadratic drag matrix
  
  Mat M;
  Mat Minv;
  Mat Dl;
  Mat Dq;
  Mat C;
  
  double mxx;
  double myy;
  double mxg;
  double ny;
  
 public:
 c_model_3dof():xg(0), yg(0)
    {
      for (int i = 0; i < 9; i++)
	ma[i] = dl[i] = dq[i] = 0;
    }
  
  ~c_model_3dof()
    {
    }  
  
  void init()
  {
    M = Mat::zeros(3, 3, CV_64FC1);
    double * data = M.ptr<double>();
    
    for (int i = 0; i < 9; i++)
      data[i] = ma[i];
    data[0] += m;
    data[4] += m; 
    data[8] += iz;
    
    mxx = data[0];
    myy = data[4];
    mxg = m * xg;
    ny = (ma[5] + ma[7]) * 0.5;
    
    Dl = Mat::zeros(3, 3, CV_64FC1);
    data = Dl.ptr<double>();
    for (int i = 0; i < 9; i++) {
      data[i] = dl[i];		
    }
    
    Dq = Mat::zeros(3, 3, CV_64FC1);
    C = Mat(3, 3, CV_64FC1);    
  }
  
  void update(double * _v,
	      double * f /* x-y force and z moment applied */,
	      const double dt /* time step */, double * _vnew);
};

class c_model_rudder_ctrl
{
 private:
  double rslack, rra, ras, rap;
 public:
 c_model_rudder_ctrl():rslack(0.f), rra(12*PI/180.f),
    rap(30*PI/180.f), ras(-30*PI/180.f)
    {
    }
  
  void update(const int u,
	      const float ra, const float slack,
	      const float dt,
	      float & ra_new, float & slack_new);
};

class c_model_engine_ctrl
{
 private:
  int fth, bth, umax, umin;
  double rgamma, rfdelta, rbdelta, fslack, bslack;
  
  enum e_gear_state{
    gs_n, gs_f, gs_b, gs_none
  };

  enum e_action_mode{
    am_nf, am_nb, am_fn, am_bn, am_fu, am_fd, am_bu, am_bd, am_none
  };
  
 public:
 c_model_engine_ctrl():fth(0x7f+0x19), bth(0x7f - 0x19),
    umax(0x00ff), umin(0x0000),
    rgamma(0.5), rfdelta(0.5), rbdelta(0.5), fslack(0.05), bslack(0.05)
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

class f_aws1_sim : public f_base
{
protected:

  friend class c_model_outboard_force;
  
  // simulation models
  c_model_rudder_ctrl mrctrl; // rudder control model
  c_model_engine_ctrl mectrl; // engine control model
  c_model_outboard_force mobf;// engine force model
  c_model_3dof m3dof;         // kinetic model

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
  
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif
