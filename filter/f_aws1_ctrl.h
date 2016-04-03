// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ctrl.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ctrl.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ctrl.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_CTRL_H_
#define _F_AWS1_CTRL_H_

#include "../channel/ch_vector.h"
#include "../channel/ch_aws1_ctrl.h"

#include "../util/aws_stdlib.h"

#include "f_base.h"


extern  const char * str_aws1_ctrl_src[ACS_NONE];



  // map a value with 3 threasholds (for rudder contrl and states)
inline  int map_oval(int val, 
		     int vmax, int vnut, int vmin, 
		     int omax, int onut, int omin
		     )
{
  int dvmax = val - vmax;
  int dvnut = val - vnut;
  int dvmin = val - vmin;
  int dvmax_vnut = vmax - vnut;
  int dvnut_vmin = vnut - vmin;
  
  if(abs(dvmax) <= abs(dvmax_vnut) && abs(dvnut) < abs(dvmax_vnut))
    return (int) ((double)((omax - onut) * (dvnut)) / (double) (dvmax_vnut)) + onut;
  else if(abs(dvnut) <= abs(dvnut_vmin) && abs(dvmin) < abs(dvnut_vmin))
    return (int) ((double)((onut - omin) * (dvnut)) / (double) (dvnut_vmin)) + onut;
  else if(abs(dvmax) < abs(dvmin))
    return omax;
  else
    return omin;
}

// map a value with 5 threasholds (for engines)
inline  int map_oval(int val, 
		     int vmax, int vfnut, int vnut, int vbnut, int vmin,
		     int omax, int ofnut, int onut, int obnut, int omin
		     )
{
  int dvmax = val - vmax;
  int dvfnut = val - vfnut;
  int dvnut = val - vnut;
  int dvbnut = val - vbnut;
  int dvmin = val - vmin;
  int dvmax_vfnut = vmax - vfnut;
  int dvfnut_vnut = vfnut - vnut;
  int dvnut_vbnut = vnut - vbnut;
  int dvbnut_vmin = vbnut - vmin;
  
  if(abs(dvmax) <= abs(dvmax_vfnut) && abs(dvfnut) < abs(dvmax_vfnut))
    return (int) ((double)((omax - ofnut) * (dvfnut)) / (double) (dvmax_vfnut)) + ofnut;
  else if(abs(dvfnut) <= abs(dvfnut_vnut) && abs(dvnut) < abs(dvfnut_vnut))
    return (int) ((double)((ofnut - onut) * (dvnut)) / (double) (dvfnut_vnut)) + onut;
  else if(abs(dvnut) <= abs(dvnut_vbnut) && abs(dvbnut) < abs(dvnut_vbnut))
    return (int) ((double)((onut - obnut) * (dvbnut)) / (double) (dvnut_vbnut)) + obnut;
  else if(abs(dvbnut) <= abs(dvbnut_vmin) && abs(dvmin) < abs(dvbnut_vmin))
    return (int) ((double)((obnut - omin) * dvmin) / (double) (dvbnut_vmin)) + omin;
  else if(abs(dvmax) < abs(dvmin))
    return omax;
  else
    return omin;
}


class f_aws1_ctrl: public f_base
{
 protected:
  ch_aws1_ctrl * m_ch_ctrl_ui, * m_ch_ctrl_ap1, * m_ch_ctrl_ap2, * m_ch_ctrl_out;   // channel for control parameters
  bool m_udp_ui;
  char m_dev[1024];         // device path, e.g. "/dev/zgpio0"
  char m_flog_name[1024];
  ofstream m_flog;
  int m_fd;                 // file descriptor for zgpio
  
  // for simulation
  bool m_sim;
  float m_rud_sta_sim;

  bool m_verb;
 
  /// LPF related parameters
  bool m_adclpf;           // Enabling ADC's low pass filter.
  int m_sz_adclpf;         // Window size of the low pass filter.
  int m_cur_adcsmpl;       // Current position in the adc sample buffer
  float m_sigma_adclpf;    // Standard deviation of the gaussian kernel.

  enum e_adclpf_type{
    ADCLPF_AVG, ADCLPF_GAUSS, ADCLPF_NONE
  } m_type_adclpf;
  static const char * m_str_adclpf_type[ADCLPF_NONE];
  
  vector<float> m_kern_adclpf;
  vector<int> m_rud_smpl;
  vector<int> m_meng_smpl;
  vector<int> m_seng_smpl;
  vector<int> m_rud_sta_smpl;

  void lpf();

  s_aws1_ctrl_pars m_acp;

  void get_gpio();
  void set_gpio();

  // Control server (for ACS_UDP)
  unsigned short m_acs_port;
  SOCKET m_acs_sock;
  sockaddr_in m_acs_sock_addr, m_acs_ret_addr;
  socklen_t m_sz_acs_ret_addr;

  void rcv_acs_udp(s_aws1_ctrl_pars & acpkt);
  void snd_acs_udp(s_aws1_ctrl_pars & acpkt);

  // Control channel (for ACS_CHAN)
  void rcv_acs_chan(s_aws1_ctrl_pars & acpkt);
  void snd_acs_chan(s_aws1_ctrl_pars & acpkt);

  void set_acpkt(s_aws1_ctrl_pars & acpkt);
  void set_ctrl(s_aws1_ctrl_pars & acpkt);

public:
  f_aws1_ctrl(const char * name);
  
  virtual ~f_aws1_ctrl();
  
  virtual bool init_run();
  
  virtual void destroy_run();
  
  virtual bool proc();
};

#endif
