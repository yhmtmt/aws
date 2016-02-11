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
  bool m_verb;
  char m_acd_host[1024];
  unsigned short m_acd_port;
  SOCKET m_acd_sock;
  sockaddr_in m_acd_sock_addr;

  s_aws1_ctrl_pars m_acp;

  void snd_ctrl(s_aws1_ctrl_pars & acpkt);
  void rcv_state(s_aws1_ctrl_pars & acpkt);
  
 public:
  f_aws1_ui(const char * name);
  virtual ~f_aws1_ui();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();
};

void drawGlEngineIndicator(const char * title, 
			   float xorg, float yorg, float w, float h, 
			   float wfont, float hfont,
			   float lw, float val_inst, float val_cur);

void drawGlRudderIndicator(const char * title, 
			   float xorg, float yorg, float w, float h,
			   float wfont, float hfont, 
			   float lw, float rud_inst, float rud_cur, 
			   float rud_sta);

#endif
