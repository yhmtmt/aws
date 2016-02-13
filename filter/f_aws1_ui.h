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
  int m_js; 
  const char * m_js_name;
  char m_acd_host[1024];
  unsigned short m_acd_port;
  SOCKET m_acd_sock;
  sockaddr_in m_acd_sock_addr;

  s_aws1_ctrl_pars m_acp;

  void snd_ctrl(s_aws1_ctrl_pars & acpkt);
  void rcv_state(s_aws1_ctrl_pars & acpkt);

  double m_mx, m_my;
  virtual void _cursor_position_callback(double xpos, double ypos){
    m_mx = xpos;
    m_my = ypos;
  }
  virtual void _mouse_button_callback(int button, int action, int mods);
  virtual void _key_callback(int key, int scancode, int action, int mods);

  // for keyboard control
  int m_num_ctrl_steps;
  enum e_eng_ctrl{
    EC_MAIN, EC_SUB
  } m_ec;

  unsigned char * m_rud_pos;
  unsigned char * m_meng_pos;
  unsigned char * m_seng_pos;

  unsigned char step_down(unsigned char val, unsigned char * vpos){
    int i = m_num_ctrl_steps;
    int iup = 2 * m_num_ctrl_steps, idown = 0;
    do{
      if(vpos[i] < val){
	idown = i;
      }else if(vpos[i] > val){
	iup = i;
      }else{
	return vpos[max(i - 1, 0)];
      }

      i = (idown + iup) >> 1;
      if(i == idown){
	return vpos[idown];
      }
    }while(1);

    return vpos[m_num_ctrl_steps];
  }

  unsigned char step_up(unsigned char val, unsigned char * vpos){
    int i = m_num_ctrl_steps;
    int iup = 2 * m_num_ctrl_steps, idown = 0;
    do{
      if(vpos[i] < val){
	idown = i;
      }else if(vpos[i] > val){
	iup = i;
      }else{
	return vpos[min(i + 1, 2 * m_num_ctrl_steps)];
      }

      i = (idown + iup) >> 1;
      if(i == idown){
	return vpos[iup];
      }
    }while(1);

    return vpos[m_num_ctrl_steps];
  }

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
