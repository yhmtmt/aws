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
#include "../util/aws_stdlib.h"

#include "f_base.h"


enum e_aws1_ctrl_src{
  ACS_FSET, ACS_UDP, ACS_CHAN, ACS_NONE
};

extern  const char * str_aws1_ctrl_src[ACS_NONE];

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
};


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
  char m_dev[1024];         // device path, e.g. "/dev/zgpio0"
  char m_flog_name[1024];
  ofstream m_flog;
  int m_fd;                 // file descriptor for zgpio
  
  bool m_verb;
  bool m_aws_ctrl;          // remote control flag. 
                            //    true: control values from aws are sat. 
                            //    false: control values from remote controller are sat.
  bool m_udp_ctrl;
  bool m_ch_ctrl;
  
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

  // adc input values
  unsigned char m_meng_rmc; // main engine control value from remote control
  unsigned char m_seng_rmc; // sub engine control value from remote control
  unsigned char m_rud_rmc;  // rudder control value from remote control
  unsigned char m_rud_sta;  // rudder status value.

  // aws control values
  unsigned char m_rud_aws;
  unsigned char m_meng_aws;
  unsigned char m_seng_aws;

  e_aws1_ctrl_src m_aws1_ctrl_src;
  s_aws1_ctrl_pars m_acs_pkt;

  // remote controller's values corresponding positions 
  unsigned char m_meng_max_rmc; 
  unsigned char m_meng_nuf_rmc;
  unsigned char m_meng_nut_rmc;
  unsigned char m_meng_nub_rmc;
  unsigned char m_meng_min_rmc;

  unsigned char m_seng_max_rmc;
  unsigned char m_seng_nuf_rmc;
  unsigned char m_seng_nut_rmc;
  unsigned char m_seng_nub_rmc;
  unsigned char m_seng_min_rmc;

  unsigned char m_rud_max_rmc;
  unsigned char m_rud_nut_rmc;
  unsigned char m_rud_min_rmc;

  unsigned char m_rud_sta_max;
  unsigned char m_rud_sta_nut;
  unsigned char m_rud_sta_min;
  
  // final control values
  unsigned char m_meng;     // main engine control value
  unsigned char m_seng;     // sub engine control value
  unsigned char m_rud;      // rudder control value
  unsigned char m_rud_sta_out; // rudder status output

  // Threashold values of digital potentiometer's
  unsigned char m_meng_max;
  unsigned char m_meng_nuf;
  unsigned char m_meng_nut;
  unsigned char m_meng_nub;
  unsigned char m_meng_min;

  unsigned char m_seng_max;
  unsigned char m_seng_nuf;
  unsigned char m_seng_nut;
  unsigned char m_seng_nub;
  unsigned char m_seng_min;

  unsigned char m_rud_max;
  unsigned char m_rud_nut;
  unsigned char m_rud_min;

  unsigned char m_rud_sta_out_max;
  unsigned char m_rud_sta_out_nut;
  unsigned char m_rud_sta_out_min;

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
  ch_ring<char> * m_pacs_in, * m_pacs_out;
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
