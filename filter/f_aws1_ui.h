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
#include "../util/aws_stdlib.h"
#include "f_aws1_ctrl.h"
#include "f_glfw_window.h"

#include "../channel/ch_image.h"
#include "../channel/ch_state.h"
#include "../channel/ch_aws1_ctrl.h"


// Joystick handling structure
struct s_jc_u3613m
{
	int id;
	const char * name;
	int naxs, nbtn;
	enum e_btn{
		EB_EVUP=0x1, EB_EVDOWN=0x2, EB_STUP=0x4, EB_STDOWN = 0x8
	};

	float thstk; // stick response threashold
	float lr1, ud1, lr2, ud2;
	unsigned char elrx, eudx;
	int tx, ty, ta, tb, tlb, trb, tlt, tlst, trst, trt, tback, tstart, tguide; // cycle time of the button pressed
	unsigned char ex, ey, ea, eb, elb, erb, elt, elst, erst, ert, eback, estart, eguide; // cycle time of the button pressed

	s_jc_u3613m():
		id(-1),	thstk(0.2f),
		tx(0), ty(0), ta(0), tb(0), tlb(0), trb(0), tlt(0), tlst(0),
		trst(0), trt(0), tback(0), tstart(0), tguide(0),
		ex(0), ey(0), ea(0), eb(0), elb(0), erb(0), elt(0), elst(0),
		erst(0), ert(0), eback(0), estart(0), eguide(0)
	{
	}

	bool init(int _id){
		if(glfwJoystickPresent(id) == GL_TRUE){
			id = _id;
			name = glfwGetJoystickName(id);
			return true;
		}
		id = -1;
		return false;
	}

	void set_stk(){
		const float * axs = glfwGetJoystickAxes(id, &naxs);
		lr1 = set_stk(axs[0]);
		ud1 = set_stk(axs[1]);
		lr2 = set_stk(axs[2]);
		ud2 = set_stk(axs[3]);
		elrx = set_btn(axs[4] > 0.5 ? 1 : 0, elrx);
		eudx = set_btn(axs[5] > 0.5 ? 1 : 0, eudx);
	}

	const float set_stk(const float v){
		if(v < thstk){
			return v + thstk;
		}else if(v > thstk){
			return v - thstk;
		}
		return 0;
	}

	void set_btn(){
		const unsigned char * btn = glfwGetJoystickButtons(id, &nbtn);
		ex = set_btn(btn[0], ex);
		ey = set_btn(btn[1], ey);
		ea = set_btn(btn[2], ea);
		eb = set_btn(btn[3], eb);
		elb = set_btn(btn[4], elb);
		erb = set_btn(btn[5], erb);
		elst = set_btn(btn[6], elst);
		erst = set_btn(btn[7], erst);
		ert = set_btn(btn[8], ert);
		eback = set_btn(btn[9], eback);
		estart = set_btn(btn[10], estart);
		eguide = set_btn(btn[11], eguide);

		tx = set_tbtn(ex, tx);
		ty = set_tbtn(ey, ty);
		ta = set_tbtn(ea, ta);
		tb = set_tbtn(eb, tb);
		tlb = set_tbtn(elb, tlb);
		trb = set_tbtn(erb, trb);
		tlst = set_tbtn(elst, tlst);
		trst = set_tbtn(erst, trst);
		trt = set_tbtn(ert, trt);
		tback = set_tbtn(eback, tback);
		tstart = set_tbtn(estart, tstart);
		tguide = set_tbtn(eguide, tguide);
	}

	unsigned char set_btn(unsigned char vnew, unsigned char eold)
	{
		return 
			/* 1st bit event up */(((EB_STDOWN & eold) >> 3) & ~vnew)  |
			/* 2nd bit event down */ ((((EB_STUP & eold) >> 2) & vnew) << 1) | 
			/* 3rd bit state up */  ((0x1 & ~vnew) << 2)|
			/* 3rd bit state down */ (vnew  << 3);
	}

	int set_tbtn(unsigned char enew, int told){
		if(EB_EVUP & enew)
			return 0;
		return (EB_STDOWN & enew) ? told + 1 : told;
	}
	void print(ostream & out){
	}
};

class f_aws1_ui: public f_glfw_window
{
 private:
  ch_state * m_state;
  ch_aws1_ctrl * m_ch_ctrl_in, * m_ch_ctrl_out;
  ch_image_ref * m_ch_img;
  bool m_udp_ctrl;
  bool m_verb;
  s_jc_u3613m m_js;
  int m_js_id;
  const char * m_js_name;
  char m_acd_host[1024];
  unsigned short m_acd_port;
  SOCKET m_acd_sock;
  sockaddr_in m_acd_sock_addr;

  s_aws1_ctrl_pars m_acp;

  // send control packet to m_acd_socket or m_ch_ctrl_out
  void snd_ctrl(s_aws1_ctrl_pars & acpkt);
  // Recive control state (rudder angle) from m_acd_socket or m_ch_ctrl_in.
  void rcv_ctrl(s_aws1_ctrl_pars & acpkt);

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

  float m_rud_aws_f;
  float m_meng_aws_f;
  float m_seng_aws_f;
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

void drawGlStateInfTxt(float xorg  /* left bottom x */, float yorg, /* left bottom y */
					float wfont, float hfont,
				  float lat, float lon, float alt, float galt, 
				  float cog, float sog, 
				  float roll, float pitch, float yaw,
				  float depth, float sz);

void drawGlSysStateInfTxt(float xorg/* right top x */, float yorg /* right top y */,
						  float wfont, float hfont,
						  e_aws1_ctrl_src ctrl_src, float sz);
#endif
