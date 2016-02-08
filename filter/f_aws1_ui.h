// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ui.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ui.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ui.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_UI_H_
#define _F_AWS1_UI_H_

#include "../util/aws_sock.h"
#include "f_aws1_ctrl.h"
#include "f_glfw_window.h"

class f_aws1_ui: public f_glfw_window
{
 private:
  char m_acd_host[1024];
  unsigned short m_acd_port;
  SOCKET m_acd_sock;
  sockaddr_in m_acd_addr;

  bool m_aws_ctrl;
  e_aws1_ctrl_src m_ctrl_src;
  unsigned char m_rud_aws;
  unsigned char m_meng_aws;
  unsigned char m_seng_aws;

  // control output
  unsigned char m_rud;
  unsigned char m_meng;
  unsigned char m_seng;

  unsigned char m_rud_rmc;
  unsigned char m_meng_rmc;
  unsigned char m_seng_rmc;

  unsigned char m_rud_sta;
  unsigned char m_rud_sta_out;

  void snd_ctrl(s_aws1_ctrl_pkt & acspkt);
  void rcv_state(s_aws1_ctrl_pkt & acspkt);

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
  
 public:
  f_aws1_ui(const char * name);
  virtual ~f_aws1_ui();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();

};

#endif
