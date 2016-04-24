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
#include "../channel/ch_wp.h"
#include "../channel/ch_obj.h"


//s_jc_u3613m  Joystick handling structure
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
	unsigned char eux, erx, edx, elx;
	int tux, trx, tdx, tlx;

	int tx, ty, ta, tb, tlb, trb, tlt, tlst, trst, trt, tback, tstart, tguide; // cycle time of the button pressed
	unsigned char ex, ey, ea, eb, elb, erb, elt, elst, erst, ert, eback, estart, eguide; // button event flags

	s_jc_u3613m():
		id(-1),	thstk(0.2f), eux(0), erx(0), edx(0), elx(0), tux(0), trx(0), tdx(0), tlx(0),
		tx(0), ty(0), ta(0), tb(0), tlb(0), trb(0), tlt(0), tlst(0),
		trst(0), trt(0), tback(0), tstart(0), tguide(0),
		ex(0), ey(0), ea(0), eb(0), elb(0), erb(0), elt(0), elst(0),
		erst(0), ert(0), eback(0), estart(0), eguide(0)
	{
	}

	bool init(int _id){
		if(glfwJoystickPresent(_id) == GL_TRUE){
			id = _id;
			name = glfwGetJoystickName(_id);
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
		
		if(naxs > 4){ // may be in linux
			erx = set_btn(axs[4] > 0.5 ? 1 : 0, erx);
			elx = set_btn(axs[4] < -0.5 ? 1 : 0, elx);
			edx = set_btn(axs[5] > 0.5 ? 1 : 0, edx);
			eux = set_btn(axs[5] < -0.5 ? 1 : 0, eux);
			trx = set_tbtn(erx, trx);
			tlx = set_tbtn(elx, tlx);
			tux = set_tbtn(eux, tux);
			tdx = set_tbtn(edx, tdx);
		}
	}

	const float set_stk(const float v){
		if(v < -thstk){
			return v + thstk;
		}else if(v > thstk){
			return v - thstk;
		}
		return 0;
	}

	void set_btn()
	{
		const unsigned char * btn = glfwGetJoystickButtons(id, &nbtn);
		ex = set_btn(btn[0], ex);
		ey = set_btn(btn[1], ey);
		ea = set_btn(btn[2], ea);
		eb = set_btn(btn[3], eb);
		elb = set_btn(btn[4], elb);
		erb = set_btn(btn[5], erb);
		elt = set_btn(btn[6], elt);
		ert = set_btn(btn[7], ert);
		elst = set_btn(btn[8], elst);
		erst = set_btn(btn[9], erst);
		eback = set_btn(btn[10], eback);
		estart = set_btn(btn[11], estart);
		eguide = set_btn(btn[12], eguide);
		tx = set_tbtn(ex, tx);
		ty = set_tbtn(ey, ty);
		ta = set_tbtn(ea, ta);
		tb = set_tbtn(eb, tb);
		tlb = set_tbtn(elb, tlb);
		trb = set_tbtn(erb, trb);
		tlt = set_tbtn(elt, tlt);
		tlst = set_tbtn(elst, tlst);
		trst = set_tbtn(erst, trst);
		trt = set_tbtn(ert, trt);
		tback = set_tbtn(eback, tback);
		tstart = set_tbtn(estart, tstart);
		tguide = set_tbtn(eguide, tguide);
		if(nbtn > 13){ // maybe windows case
			// U13, R14, D15, L16
			erx = set_btn(btn[14], erx);
			elx = set_btn(btn[16], elx);
			eux = set_btn(btn[13], eux);
			edx = set_btn(btn[15], edx);

			trx = set_tbtn(erx, trx);
			tlx = set_tbtn(elx, tlx);
			tux = set_tbtn(eux, tux);
			tdx = set_tbtn(edx, tdx);
		}
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

	void print(ostream & out)
	{
		out << "STK>>";
		out << "LR1:" << lr1 << " UD1:" << ud1 << " ";
		out << "LR2:" << lr2 << " UD2:" << ud2 << " ";
		print(out, "LX", elx, tlx); out << " ";
		print(out, "RX", erx, trx); out << " ";
		print(out, "UX", eux, tux); out << " ";
		print(out, "DX", edx, tdx); 
		out << endl;

		out << "BTN>>";
		print(out, "x", ex, tx); out << " ";
		print(out, "y", ey, ty); out << " ";	
		print(out, "a", ea, ta); out << " ";
		print(out, "b", eb, tb); out << " ";
		print(out, "lb", elb, tlb);	out << " ";
		print(out, "rb", erb, trb);	out << " ";
		print(out, "lt", elt, tlt); out << " ";
		print(out, "rt", ert, trt);	out << " ";
		print(out, "lst", elst, tlst); out << " ";
		print(out, "rst", erst, trst); out << " ";
		print(out, "back", eback, tback); out << " ";
		print(out, "start", estart, tstart); out << " ";
		print(out, "guide", eguide, tguide);
		out << endl;
	}

	void print(ostream & out, const char * btn, unsigned char e, int t){
		out << btn << ":" 
			<< (EB_STDOWN & e ? "D" : "X") << (EB_EVDOWN & e ? "^" : "_")
			<< (EB_STUP & e ? "U" : "X") << (EB_EVUP & e ? "^" : "_") << " " << t;
	}

	// Event detectors.
	bool is_event_down(unsigned char e)
	{
		return EB_EVDOWN & e ? true : false;
	}

	bool is_state_down(unsigned char e)
	{
		return EB_STDOWN & e ? true : false;
	}

	bool is_event_up(unsigned char e)
	{
		return EB_EVUP & e ? true : false;
	}

	bool is_state_up(unsigned char e)
	{
		return EB_STUP & e ? true : false;
	}
};

#include "c_aws1_ui_core.h"

class f_aws1_ui: public f_glfw_window
{
 private:
  ch_state * m_state;
  ch_aws1_ctrl * m_ch_ctrl_in, * m_ch_ctrl_out;
  ch_wp * m_ch_wp;
  ch_obj * m_ch_obj;
  ch_ais_obj * m_ch_ais_obj;
  ch_image_ref * m_ch_img;

  friend class c_aws1_ui_core;

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

  float m_xscale, m_yscale, m_ixscale, m_iyscale;

  virtual void _mouse_button_callback(int button, int action, int mods);
  virtual void _key_callback(int key, int scancode, int action, int mods);

  // User interface mode sets
  enum e_aws1_ui_mode {
	  AUM_NORMAL, AUM_MAP, AUM_DEV, AUM_UNDEF
  } m_mode;

  static const  char * m_str_aws1_ui_mode[AUM_UNDEF];
  c_aws1_ui_core * m_ui[AUM_UNDEF];

 public:
  f_aws1_ui(const char * name);
  virtual ~f_aws1_ui();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();

  // If LT+LB+RT+RB is detected, the system forces the controls to be nutral state. Called by default.
  void ui_force_ctrl_stop();

  void ui_show_img();
  void ui_show_rudder();
  void ui_show_meng();
  void ui_show_seng();
  void ui_show_state();
  void ui_show_sys_state();

  bool m_ui_menu;
  
  int m_menu_focus;
  e_aws1_ctrl_src m_menu_acs;
  e_aws1_ui_mode m_menu_mode;
  bool m_quit;
  void ui_show_menu();
  void ui_handle_menu();
};

// helps f_aws1_ui::ui_show_meng and ui_show_seng 
void drawGlEngineIndicator(const char * title, 
			   float xorg, float yorg, float w, float h, 
			   float wfont, float hfont,
			   float lw, float val_inst, float val_cur);

//////////////////////////////////////////////////////// f_aws1_ui_test
// This class is a test class interfacing only with f_aws1_ui via the compatible channel set.
// This class is used to manipulate the channel values by fset parameters. 
class f_aws1_ui_test: public f_base
{
protected:
	ch_state * m_state;
	ch_aws1_ctrl * m_ch_ctrl_ui, * m_ch_ctrl_ap1, * m_ch_ctrl_ap2, * m_ch_ctrl_out;
	ch_image_ref * m_ch_img;

	float r, p, y; // roll(deg), pitch(deg), yaw(deg)
	float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	float cog, sog; // Course over ground(deg), Speed over ground (kts)
	float depth; // water depth

	s_aws1_ctrl_pars m_acp;


	float m_rud_sta_sim;
public:
	f_aws1_ui_test(const char * name);

	void select_control_input()
	{
		// Control input selection
		s_aws1_ctrl_pars acp;
		if(m_ch_ctrl_ui)
			m_ch_ctrl_ui->get_pars(acp);
		m_acp.tcur = acp.tcur;
		m_acp.ctrl_src = acp.ctrl_src;
		switch(m_acp.ctrl_src){
		case ACS_UI:
			m_acp.rud_aws = acp.rud_aws;
			m_acp.meng_aws = acp.meng_aws;
			m_acp.seng_aws = acp.seng_aws;
			break;
		case ACS_AP1:
			if(m_ch_ctrl_ap1){
				m_ch_ctrl_ap1->get_pars(acp);
				m_acp.rud_aws = acp.rud_aws;
				m_acp.meng_aws = acp.meng_aws;
				m_acp.seng_aws = acp.seng_aws;
			}
			break;
		case ACS_AP2:
			if(m_ch_ctrl_ap2){
				m_ch_ctrl_ap2->get_pars(acp);
				m_acp.rud_aws = acp.rud_aws;
				m_acp.meng_aws = acp.meng_aws;
				m_acp.seng_aws = acp.seng_aws;
			}
			break;
		default:
			break;
		}
	}

	void simulate_rudder(){
		// Rudder response simulation
		unsigned rud_inst = map_oval(m_acp.rud,
			m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min,
			m_acp.rud_sta_max, m_acp.rud_sta_nut, m_acp.rud_sta_min);
#define RUD_PER_CYCLE 0.45f
		if(rud_inst > m_acp.rud_sta){
			m_rud_sta_sim += RUD_PER_CYCLE;
		}else{
			m_rud_sta_sim -= RUD_PER_CYCLE;
		}

		m_acp.rud_sta = (unsigned char) m_rud_sta_sim;
	}

	void set_control_output()
	{
		switch(m_acp.ctrl_src){
		case ACS_UI:
		case ACS_AP1:
		case ACS_AP2:
		case ACS_FSET:
		case ACS_NONE:
			m_acp.rud = map_oval(m_acp.rud_aws, 
				0xff, 0x7f, 0x00, 
				m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min);
			m_acp.meng = map_oval(m_acp.meng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, 
				m_acp.meng_nub, m_acp.meng_min);  
			m_acp.seng = map_oval(m_acp.seng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, 
				m_acp.seng_nub, m_acp.seng_min);
			break;
		case ACS_RMT:
			m_acp.rud = map_oval(m_acp.rud_rmc, 
				m_acp.rud_max_rmc, m_acp.rud_nut_rmc, m_acp.rud_min_rmc,
				m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min);
			m_acp.meng = map_oval(m_acp.meng_rmc, 
				m_acp.meng_max_rmc, m_acp.meng_nuf_rmc, m_acp.meng_nut_rmc, 
				m_acp.meng_nub_rmc, m_acp.meng_min_rmc,
				m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, m_acp.meng_nub, 
				m_acp.meng_min);  
			m_acp.seng = map_oval(m_acp.seng_rmc, 
				m_acp.seng_max_rmc, m_acp.seng_nuf_rmc, m_acp.seng_nut_rmc, 
				m_acp.seng_nub_rmc, m_acp.seng_min_rmc,
				m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, m_acp.seng_nub, 
				m_acp.seng_min);
			break;
		}
		if(m_ch_ctrl_out){
			m_ch_ctrl_out->set_pars(m_acp);
		}
	}

	void set_state(){
		if(m_state){
			m_state->set_attitude(r, p, y);
			m_state->set_position(lat, lon, alt, galt);
			m_state->set_velocity(cog, sog);
			m_state->set_depth(depth);
		}
	}

	void simulate_dynamics()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};
#endif
