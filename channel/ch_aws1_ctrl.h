#ifndef _CH_AWS1_CTRL_H_
#define _CH_AWS1_CTRL_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_aws1_ctrl.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_aws1_ctrl.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_aws1_ctrl.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"

enum e_aws1_ctrl_src{
  ACS_FSET, ACS_UDP, ACS_CHAN, ACS_NONE
};

// Both for transmission and reception the same packet structure is used.
struct s_aws1_ctrl_pars{
  // current time
  long long tcur;

  // control modes
  bool ctrl;
  e_aws1_ctrl_src ctrl_src;

  // control output
  unsigned char rud;
  unsigned char meng;
  unsigned char seng;

  // aws control input (from aws)
  unsigned char rud_aws;
  unsigned char meng_aws;
  unsigned char seng_aws;

  // aws remote controller's input (from adc)
  unsigned char rud_rmc;
  unsigned char meng_rmc;
  unsigned char seng_rmc;

  // rudder angle 
  unsigned char rud_sta;     // from adc
  unsigned char rud_sta_out; // to digi-pot

  // remote controller's values corresponding positions 
  unsigned char meng_max_rmc; 
  unsigned char meng_nuf_rmc;
  unsigned char meng_nut_rmc;
  unsigned char meng_nub_rmc;
  unsigned char meng_min_rmc;

  unsigned char seng_max_rmc;
  unsigned char seng_nuf_rmc;
  unsigned char seng_nut_rmc;
  unsigned char seng_nub_rmc;
  unsigned char seng_min_rmc;

  unsigned char rud_max_rmc;
  unsigned char rud_nut_rmc;
  unsigned char rud_min_rmc;

  unsigned char rud_sta_max;
  unsigned char rud_sta_nut;
  unsigned char rud_sta_min;

  // Threashold values of digital potentiometer's
  unsigned char meng_max;
  unsigned char meng_nuf;
  unsigned char meng_nut;
  unsigned char meng_nub;
  unsigned char meng_min;

  unsigned char seng_max;
  unsigned char seng_nuf;
  unsigned char seng_nut;
  unsigned char seng_nub;
  unsigned char seng_min;

  unsigned char rud_max;
  unsigned char rud_nut;
  unsigned char rud_min;

  unsigned char rud_sta_out_max;
  unsigned char rud_sta_out_nut;
  unsigned char rud_sta_out_min;

  // success flag
  bool suc;

s_aws1_ctrl_pars():
  ctrl(false), ctrl_src(ACS_FSET),
    meng_max_rmc(0x81), meng_nuf_rmc(0x80),  meng_nut_rmc(0x7f),  
    meng_nub_rmc(0x7e), meng_min_rmc(0x7d),  
    seng_max_rmc(0x81),  seng_nuf_rmc(0x80), seng_nut_rmc(0x7f),
    seng_nub_rmc(0x7e),  seng_min_rmc(0x7d),
    rud_max_rmc(0x80),  rud_nut_rmc(0x7f),  rud_min_rmc(0x7e),
    rud_sta_max(0xff), rud_sta_nut(0x7f), rud_sta_min(0x00),
    meng(0x7f),  seng(0x7f),  rud(0x7f),  
    meng_max(0x81),meng_nuf(0x80),  meng_nut(0x7f),  
    meng_nub(0x7e), meng_min(0x7d),  
    seng_max(0x81),  seng_nuf(0x80), seng_nut(0x7f),
    seng_nub(0x7e),  seng_min(0x7d),
    rud_max(0x80),  rud_nut(0x7f),  rud_min(0x7e),
    rud_sta_out_max(0xff), rud_sta_out_nut(0x7f), rud_sta_out_min(0x00)
  {
  }
};

class ch_aws1_ctrl: public ch_base
{
 protected:
  s_aws1_ctrl_pars pars;
 public:
 ch_aws1_ctrl(const char * name):ch_base(name)
  {
  }

  void set_pars(const s_aws1_ctrl_pars & _pars){
    lock();
    pars = _pars;
    unlock();
  }

  void get_pars(s_aws1_ctrl_pars & _pars){
    lock();
    _pars = pars;
    unlock();
  }

  virtual size_t get_dsize()
  {
    return sizeof(s_aws1_ctrl_pars);
  }

  virtual size_t write_buf(const char * buf)
  {
    lock();
    memcpy((void*)&pars, (void*)buf, sizeof(s_aws1_ctrl_pars));
    unlock();
    return sizeof(s_aws1_ctrl_pars);
  }

  virtual size_t read_buf(char * buf)
  {
    lock();
    memcpy((void*)buf, (void*)&pars, sizeof(s_aws1_ctrl_pars));
    unlock();
    return sizeof(s_aws1_ctrl_pars);
  }

  virtual void print(ostream & out)
  {
    cout << "channel " << m_name << " rud " << pars.rud  << " " 
	 << pars.rud_aws << " meng " << pars.meng << " " 
	 << pars.meng_aws << " seng " << pars.seng << " " << pars.seng_aws << " rud_sta " 
	 << pars.rud_sta << " rud_sta_out " << pars.rud_sta_out << endl;
  }
};

#endif
