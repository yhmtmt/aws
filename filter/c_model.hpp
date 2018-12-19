#ifndef C_MODEL_HPP
#define C_MODEL_HPP
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

#ifdef PY_EXPORT
#include <boost/python.hpp>

template<class T> void dict2map(boost::python::dict & d, map<string, T> & m){
  int len = boost::python::len(d);
  boost::python::list keys = d.keys();
  for (int i = 0; i < len; i++){
    string key = boost::python::extract<string>(keys[i]);
    m[key] = boost::python::extract<double>(d[key]);
  }
}

#endif

///////////////////////////////// base of  simulation model for boat parts
class c_model_base
{
 private:
  char ** str_param;
  int index;
 public:
 c_model_base():str_param(NULL), index(-1)
    {
    }
  
  virtual ~c_model_base()
    {
      release_param();
    }

  virtual void init()
  {
    return;
  }
  
  virtual const char * get_str_param(int iparam)
  {
    if (index < 0)
      return _get_str_param(iparam);
    if (iparam >= 0 && iparam < get_num_params())
      return str_param[iparam];
    return NULL;
  }
  
  virtual const char * _get_str_param(int iparam){return NULL;};
  virtual const char * get_str_param_exp(int iparam){return NULL;};
  virtual double * get_param(int iparam){return NULL;};  
  virtual int get_num_params(){return 0;};

  void set_params(map<string,double> & vals)
  {
    int npars = get_num_params();
    for (int iparam = 0; iparam < npars; iparam++){
      if(vals.find(get_str_param(iparam)) == vals.end())
	continue;
      *get_param(iparam) = vals[get_str_param(iparam)];      
    }
    init();
  }

  map<string, double> get_params()
  {
    map<string, double> mvals;
    int npars = get_num_params();
    for (int iparam = 0; iparam < npars; iparam++){
      mvals[get_str_param(iparam)] = *get_param(iparam);
    }
    return mvals;
  }

  #ifdef PY_EXPORT
  void set_params_py(boost::python::dict & dvals)
  {
    map<string,double> mvals;
    dict2map(dvals, mvals);
    set_params(mvals);
  }
  
  boost::python::dict get_params_py()
  {
    boost::python::dict dvals;
    int npars = get_num_params();
    for (int iparam = 0; iparam < npars; iparam++){
      dvals[get_str_param(iparam)] = *get_param(iparam);
    }
    return dvals;    
  }

  boost::python::dict get_str_params_py()
  {
    boost::python::dict dstrs;
    int npars = get_num_params();
    for (int iparam = 0; iparam < npars; iparam++){
      dstrs[get_str_param(iparam)] = string(get_str_param_exp(iparam));
    }
    return dstrs;
  }
  
  #endif
  
  char * gen_str_indexed_param(int iparam);
  
  bool alloc_param(int _index = -1);
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

 public:
 c_model_3dof():xg(0), yg(0)
    {
      for (int i = 0; i < 9; i++)
	ma[i] = dl[i] = dq[i] = 0;
    }
  
  virtual ~c_model_3dof()
    {
    }  
    
  virtual const char * _get_str_param(int iparam)
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

  virtual void init();

  #ifdef PY_EXPORT
  boost::python::tuple update_py(double & _u, double & _v, double & _r, double & _taux, double & _tauy, double & _taun, double dt)
  {
    double v[3]={_u,_v,_r};
    double vnew[3];
    double f[3]={_taux,_tauy,_taun};
    update(v, f, dt, vnew);
    return boost::python::make_tuple(vnew[0],vnew[1],vnew[2]);
  }
  #endif
  
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
 public:
  c_model_rudder_ctrl():rslack(0.f), rrud(12*PI/180.f),
    rudp(30*PI/180.f), ruds(-30*PI/180.f)
    {
    }

  virtual ~c_model_rudder_ctrl()
    {
    }
  
  virtual const char * _get_str_param(int iparam)
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

  void update(const int u,
	      const float ra, const float slack,
	      const float dt,
	      float & ra_new, float & slack_new);
  #ifdef PY_EXPORT
  boost::python::tuple update_py(const int u,
	      const float ra, const float slack,
				 const float dt)
  {
    float ra_new=0., slack_new=0.;
    update(u,ra,slack,dt, ra_new, slack_new);
    return boost::python::make_tuple(ra_new, slack_new);
  }
  #endif
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
public:
  c_model_engine_ctrl():fth(0x7f+0x19), bth(0x7f - 0x19),
    umax(0x00ff), umin(0x0000),
    rgamma(0.5), rfdelta(0.5), rbdelta(0.5), fslack(0.05), bslack(0.05)
    {      
    }

  virtual ~c_model_engine_ctrl()
    {
    }
  
  virtual const char * _get_str_param(int iparam)
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
  #ifdef PY_EXPORT
  boost::python::tuple update_py(const int u, const float gamma, const float delta,
			      const float slack, const float dt)
  {
    float gamma_new=0., delta_new=0., slack_new=0.;
    update(u, gamma, delta, slack, dt, gamma_new, delta_new, slack_new);
    return boost::python::make_tuple(gamma_new, delta_new, slack_new);
  }
  #endif
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
  
public:
 c_model_outboard_force() :xr(0.f), yr(-3.f), CTL(0.), CTQ(0.), CD(0.), CL(0.)
    {
    }
  
  virtual ~c_model_outboard_force()
    {
    }
  
  virtual const char * _get_str_param(int iparam)
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

  void update(const double _rud, const double _gear,
	      const double _thro, const double _rev, const double * v,
	      double * f);
  #ifdef PY_EXPORT
  boost::python::tuple update_py(const double _rud, const double _gear,
	      const double _thro, const double _rev,
	       const double _u, const double _v, const double _r);
  #endif
};

#ifdef PY_EXPORT
BOOST_PYTHON_MODULE( pyawssim ){
  namespace python = boost::python;
  python::class_<c_model_base>("c_model_base")
    .def("set_params", &c_model_base::set_params_py)
    .def("get_params", &c_model_base::get_params_py)
    .def("get_str_params", &c_model_base::get_str_params_py)
    .def("init", &c_model_base::init)
    .def("alloc_param", &c_model_base::alloc_param)
    ;
  
  python::class_<c_model_3dof, python::bases<c_model_base> >("c_model_3dof")
    .def("update", &c_model_3dof::update_py)
    ;
  
  python::class_<c_model_rudder_ctrl, python::bases<c_model_base> >("c_model_rudder_ctrl")
    .def("update", &c_model_rudder_ctrl::update_py)
    ;

  python::class_<c_model_engine_ctrl, python::bases<c_model_base> >("c_model_engine_ctrl")
    .def("update", &c_model_engine_ctrl::update_py)
    ;

  python::class_<c_model_outboard_force, python::bases<c_model_base> >("c_model_outboard_force")
    .def("update", &c_model_outboard_force::update_py)
    ;
}

#endif

#endif
