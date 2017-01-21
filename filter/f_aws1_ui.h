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
#include "../channel/ch_aws1_sys.h"
#include "../channel/ch_map.h"
#include "../channel/ch_wp.h"
#include "../channel/ch_obj.h"

#include "../util/aws_jpad.h"

#include "c_aws1_ui_core.h"


class f_aws1_ui: public f_glfw_window
{
 private:
  ch_state * m_state;
  ch_aws1_sys * m_ch_sys;
  ch_aws1_ctrl_inst * m_ch_ctrl_inst;
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;
  ch_wp * m_ch_wp;
  ch_map * m_ch_map;
  ch_obj * m_ch_obj;
  ch_ais_obj * m_ch_ais_obj;
  ch_image_ref * m_ch_img, * m_ch_img2, * m_ch_disp;
  ch_obst * m_ch_obst;
  ch_aws1_ap_inst * m_ch_ap_inst;

  bool m_img_x_flip, m_img_y_flip;
  bool m_img2_x_flip, m_img2_y_flip;
 void cnv_img_to_view(Mat & img, float av, Size & sz, bool flipx, bool flipy);

  enum e_imv{
	  IMV_IMG1, IMV_IMG2, IMV_DISP, IMV_IMG12, IMV_IMG12D, IMV_UNDEF
  } m_imv;
  static const char * m_str_imv[IMV_UNDEF];

  char m_path_storage[1024];
  // Main image view related parameters
  // Main image view shows an image from m_ch_img, the focal length and the principal point are defined as follows.
  float m_fx, m_fy; // focal length of the main image view
  float m_cx, m_cy; // principal point of the main image view
  // Note that image in m_ch_img should be the undistorted image.

  friend class c_aws1_ui_core;

  bool m_verb;
  s_jc_u3613m m_js;
  int m_js_id;
  const char * m_js_name;

  s_aws1_ctrl_inst m_inst;
  s_aws1_ctrl_stat m_stat;

  // send control packet to m_acd_socket or m_ch_ctrl_out
  void snd_ctrl_inst();
  // Recive control state (rudder angle) from m_acd_socket or m_ch_ctrl_in.
  void rcv_ctrl_stat();

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

  VideoWriter m_vw;
  bool m_bsvw; // screen video write
  bool m_bss; // screen shot
  Mat m_simg;	// screen img

  void write_screen();

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
  void ui_show_attitude();

  bool m_ui_menu;
  
  int m_menu_focus;
  e_aws1_ctrl_src m_menu_acs;
  e_aws1_ui_mode m_menu_mode;
  e_ap_mode m_menu_ap_mode;

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
	ch_ais_obj * m_ch_ais_obj;
	ch_aws1_ctrl_stat * m_ch_ctrl_stat;

	ch_aws1_ctrl_inst * m_ch_ctrl_ui, * m_ch_ctrl_ap1, * m_ch_ctrl_ap2;
	ch_image_ref * m_ch_img;

	bool m_ahrs, m_gps; 
	float r, p, y; // roll(deg), pitch(deg), yaw(deg)
	float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	float cog, sog; // Course over ground(deg), Speed over ground (kts)
	float depth; // water depth
	s_aws1_ctrl_stat m_stat;

	bool m_add_ais_ship;
	unsigned int ais_mmsi;
	float ais_lat, ais_lon, ais_cog, ais_sog, ais_yaw;

	float m_rud_sta_sim;
public:
	f_aws1_ui_test(const char * name);

	void select_control_input()
	{
		// Control input selection
		s_aws1_ctrl_inst acp;
		if(m_ch_ctrl_ui)
			m_ch_ctrl_ui->get(acp);
		m_stat.tcur = acp.tcur;
		m_stat.ctrl_src = acp.ctrl_src;
		switch(m_stat.ctrl_src){
		case ACS_UI:
			m_stat.rud_aws = acp.rud_aws;
			m_stat.meng_aws = acp.meng_aws;
			m_stat.seng_aws = acp.seng_aws;
			break;
		case ACS_AP1:
			if(m_ch_ctrl_ap1){
				m_ch_ctrl_ap1->get(acp);
				m_stat.rud_aws = acp.rud_aws;
				m_stat.meng_aws = acp.meng_aws;
				m_stat.seng_aws = acp.seng_aws;
			}
			break;
		case ACS_AP2:
			if(m_ch_ctrl_ap2){
				m_ch_ctrl_ap2->get(acp);
				m_stat.rud_aws = acp.rud_aws;
				m_stat.meng_aws = acp.meng_aws;
				m_stat.seng_aws = acp.seng_aws;
			}
			break;
		default:
			break;
		}
	}

	void simulate_rudder(){
		// Rudder response simulation
		unsigned rud_inst = map_oval(m_stat.rud,
			m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min,
			m_stat.rud_sta_max, m_stat.rud_sta_nut, m_stat.rud_sta_min);
#define RUD_PER_CYCLE 0.45f
		if(rud_inst > m_stat.rud_sta){
			m_rud_sta_sim += RUD_PER_CYCLE;
		}else{
			m_rud_sta_sim -= RUD_PER_CYCLE;
		}

		m_stat.rud_sta = (unsigned char) m_rud_sta_sim;
	}

	void set_control_output()
	{
		switch(m_stat.ctrl_src){
		case ACS_UI:
		case ACS_AP1:
		case ACS_AP2:
		case ACS_FSET:
		case ACS_NONE:
			m_stat.rud = map_oval(m_stat.rud_aws, 
				0xff, 0x7f, 0x00, 
				m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
			m_stat.meng = map_oval(m_stat.meng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, 
				m_stat.meng_nub, m_stat.meng_min);  
			m_stat.seng = map_oval(m_stat.seng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, 
				m_stat.seng_nub, m_stat.seng_min);
			break;
		case ACS_RMT:
			m_stat.rud = map_oval(m_stat.rud_rmc, 
				m_stat.rud_max_rmc, m_stat.rud_nut_rmc, m_stat.rud_min_rmc,
				m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
			m_stat.meng = map_oval(m_stat.meng_rmc, 
				m_stat.meng_max_rmc, m_stat.meng_nuf_rmc, m_stat.meng_nut_rmc, 
				m_stat.meng_nub_rmc, m_stat.meng_min_rmc,
				m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, m_stat.meng_nub, 
				m_stat.meng_min);  
			m_stat.seng = map_oval(m_stat.seng_rmc, 
				m_stat.seng_max_rmc, m_stat.seng_nuf_rmc, m_stat.seng_nut_rmc, 
				m_stat.seng_nub_rmc, m_stat.seng_min_rmc,
				m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, m_stat.seng_nub, 
				m_stat.seng_min);
			break;
		}
		if(m_ch_ctrl_stat){
			m_ch_ctrl_stat->set(m_stat);
		}
	}

	void set_state(){
		if(m_state){
			long long t = 0;
			if(!m_ahrs)
				m_state->set_attitude(t, r, p, y);

			if(!m_gps){
				m_state->set_position(t, lat, lon, alt, galt);		
				m_state->set_velocity(t, cog, sog);
			}

			m_state->set_depth(t, depth);
		}
	}

	void simulate_dynamics()
	{
	}

	void add_ais_ship()
	{
		if(m_ch_ais_obj && m_add_ais_ship){
			m_add_ais_ship = false;
			m_ch_ais_obj->push(m_cur_time, ais_mmsi, ais_lat, ais_lon, ais_cog, ais_sog, ais_yaw);
			if(m_state){
				long long t = 0;
				Mat R = m_state->get_enu_rotation(t);
				float x, y, z;
				m_state->get_position_ecef(t, x, y, z);
				m_ch_ais_obj->update_rel_pos_and_vel(R, x, y, z);
			}
		}
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};
#endif
