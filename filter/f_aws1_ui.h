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

#define MAX_RT_FILES 10

enum e_ui_mode {
  ui_mode_fpv, ui_mode_map, ui_mode_sys, ui_mode_undef
};

enum e_ui_obj{
  ui_obj_wp = 0, ui_obj_vsl, ui_obj_mrk, ui_obj_cl, ui_obj_undef 
};

#include "f_aws1_ui_util/c_ui_box.h"
#include "f_aws1_ui_util/c_map_obj.h"

class f_aws1_ui: public f_glfw_window
{
 public:
  enum e_obj_type{
    ot_wp, ot_cl, ot_ais, ot_mark, ot_nul
  };
  
  struct s_obj{
    e_obj_type type;
    int handle;
  s_obj():type(ot_nul), handle(-1){}
  };
  
  enum e_mouse_state{
    ms_add_wp, ms_drag, ms_normal
  } mouse_state;
  
 private:
  void cnv_img_to_view(Mat & img, float av, Size & sz, bool flipx, bool flipy);
  
  bool m_verb;
  
  ////////////////////////////////////////// Channel Declaration  
  ch_state * m_state;			// required
  ch_eng_state * m_engstate;            // optional
  ch_aws1_sys * m_ch_sys;		// is not used
  ch_aws1_ctrl_inst * m_ch_ctrl_inst;	// optional
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;   // optional
  ch_wp * m_ch_wp;	 	        // optional,
                                        //ref. update_route_cfg_box(),
                                        //     update_route(), add_waypoint()
  ch_map * m_ch_map;			// optional, ref. update_map()
  ch_obj * m_ch_obj;			// is not used
  ch_ais_obj * m_ch_ais_obj;		// optional, ref. update_ais_obj()
  ch_obst * m_ch_obst;			// is not used
  ch_aws1_ap_inst * m_ch_ap_inst;	// optional,
                                        // ref. update_ctrl_mode_box(),
                                        //      handle_ctrl_csr()
  ch_image_ref * m_ch_cam;

  char m_path_storage[1024]; // path string to the storage

  /////////////////////////////////////////// shader related members
  char fvs[1024], ffs[1024], ftex[1024], ftexinf[1024];
  GLuint p;// shader program
  GLuint loc_mode /* rendering mode flag. See glsl.*/ , 
    loc_gcolor /* object color */ , loc_gcolorb /* object background color */,
    loc_pos2d /* 2d point */, loc_inv_sz_half_scrn /* inverse half screen size */,
    loc_Mmvp /* transform matrix */ , loc_Mm /* object rotation/translation matrix */, 
    loc_Lpar /* Light direction */, loc_sampler /* Texture sampler */, 
    loc_depth2d, loc_position, loc_normal, loc_texcoord;
  float inv_sz_half_scrn[2], fov_cam_x, fov_cam_y, fcam, ifcam, height_cam, dir_cam_hdg, dir_cam_hdg_drag;
  float iRE, height_cam_ec, dhorizon_cam, dhorizon_arc, zhorizon, th_horizon;
  bool setup_shader();
  
  /////////////////////////////////////////// graphics elements. 
  // The elements can be used by changing position, scale, and rotation for each graphics artifacts.
  c_gl_2d_obj orect      /* 2d rectangle */,
    otri       /* 2d triangle */,
    ocirc      /* 2d circle */,
    omap_mask /* map mode mask */;
  
  c_gl_text_obj otxt     /* Text */;
  c_gl_2d_line_obj oline /* 2d line */;
  c_gl_line_obj oline3d  /* 3d line */;
  c_gl_line_obj oline3d_map; /* 3d lin for map*/
  Mat m_cam;                   // main camera image
  long long m_tcam, m_frm_cam; // time and frame number m_cam grabbed
  e_imfmt m_fmt_cam;           // image format of main camera

  int hmap_mask[2];
  bool init_map_mask();
  
  void render_gl_objs(c_view_mode_box * pvm_box); // renders all elements above declared.

  // ui boxes
  c_aws_ui_box_manager uim;
  void handle_updated_ui_box(c_view_mode_box * pvm_box,
			     c_ctrl_mode_box * pcm_box,
			     c_map_cfg_box * pmc_box,
			     c_route_cfg_box * prc_box);
 
  void update_view_mode_box(c_view_mode_box * pvm_box);
  void update_ctrl_mode_box(c_ctrl_mode_box * pcm_box);
  void update_map_cfg_box(c_map_cfg_box * pmc_box);
  void update_route_cfg_box(c_route_cfg_box * prc_box,
			    e_mouse_state mouse_state_new);
  void update_ui_params(c_view_mode_box * pvm_box,
			const float xown, const float yown, const float zown,
			const float vx, const float vy, const float yaw);
 
  // for Android wear 
  bool bwear;
  unsigned short wmeng, wrud, wrev, wsog, wcog, wyaw, wdpt, whbt, whbt0;
  long long twhbt, twhbt_out;

  // Indicator
  c_indicator ind;
  void update_indicator(const float cog, const float sog, 
	  const float roll, const float pitch, const float yaw)
  {
    long long t = 0;
    float rpm = 0.0f;
    unsigned char trim = 0;
    int poil = 0;
    float toil = 0.0f;
    float temp = 0.0f;
    float valt = 0.0f;
    float frate = 0.0f;
    unsigned int teng = 0;
    int pclnt = 0;
    int pfl = 0;
    unsigned char ld = 0;
    unsigned char tq = 0;
    StatEng1 steng1 = (StatEng1)(EmergencyStop + 1);
    StatEng2 steng2 = (StatEng2)(EngineShuttingDown + 1);
    float depth = 0.f;
    
    if(m_engstate){ // currentlly only for main engine
      m_engstate->get_rapid(t, rpm, trim);
      m_engstate->get_dynamic(t, poil, toil, temp, valt, frate, teng, pclnt, pfl, steng1, steng2, ld, tq);
    }

    if(m_state){
      m_state->get_depth(t, depth);
    }

    float cogf = (float)(cog * (PI / 180.f));
    float yawf = (float)(yaw * (PI / 180.f));
    ind.set_param(
		  m_time_str,
		  m_stat.meng_aws,
		  rpm, rev_tgt, trim, poil, toil, temp,
		  valt, frate, teng, pclnt, pfl,
		  ld, tq,
		  ((steng1 >= 0 && steng1 <= EmergencyStop) ? strStatEng1[steng1] : NULL),
		  ((steng2 >= 0 && steng2 <= EngineShuttingDown) ? strStatEng2[steng2] : NULL),
		  m_stat.seng_aws, m_stat.rud_aws,
		  cogf, (float)(cog_tgt * (PI/180.f)),
		  sog, sog_tgt,
		  yawf,
		  (float)(pitch* (PI / 180.f)),
		  (float)(-roll* (PI / 180.f)),
		  depth);
    ind.set_dir_cam(dir_cam_hdg + dir_cam_hdg_drag);

    // for android wear
    wmeng = (unsigned short) m_stat.meng_aws;
    wrud = (unsigned short) m_stat.rud_aws;
    wrev = (unsigned short) rpm;
    wsog = (unsigned short)(sog * 10);
    wcog = (unsigned short) cog;
    wyaw = (unsigned short) (yaw < 0 ? yaw + 360: yaw);
    wdpt = (unsigned short) depth * 10;
  }
  
  //////////////////////////////////////////// map objects
  float sz_mark;		// Basic size of each object.
  vector<bool> visible_obj; // Visible objects (copied from map_cfg_box), the index is e_obj_type

  c_map_waypoint_obj owp;	// waypoint object
  int num_max_wps;		// maximum number of waypoints user can create.

  c_map_coast_line_obj coast_line; // coast line object
  bool bupdate_map;		// flag to notify reload the map 
  glm::vec3 pt_prev_map_update;	// position previous map update invoked. 
								    //(if the distance from the point has been larger than map range, bupdate_map is asserted.) 

  c_map_ais_obj oais;		// ais object
  int num_max_ais;	        // maximum number of ais objects ui can draw.

  void update_ais_objs();
  void update_route(c_route_cfg_box * prc_box);
  void update_map();

  c_own_ship own_ship;// own ship object
  c_cursor ocsr;		  // cursor 
 
  enum e_button{
    ebtn_lock_map_own_ship, ebtn_lock_cam_dir_hdg, ebtn_wear_dev_ctrl, ebtn_js_ctrl, ebtn_nul
  };
  e_button btn_pushed, btn_released;
  c_aws_ui_button btn_lock_map_own_ship, btn_lock_cam_dir_hdg, btn_wear_dev_ctrl, btn_js_ctrl;
  e_button get_col_button();
  bool handle_btn_pushed();
  bool handle_btn_released();
  void update_button(c_view_mode_box * pvm_box);

  ////////////////////////////////////////// Control instruction and status
  // Control source is categorized into two: one is manual, another is autopilot.
  // Manual control is done via joystic by instructing the control value to the m_ch_ctrl_inst channel,
  // and autopilot control is done by instructing the autopilot command to the m_ch_ap_inst channel.

  s_aws1_ctrl_inst m_inst; // the control instruction value
  void snd_ctrl_inst(); // send m_inst to m_ch_ctrl_inst

  s_aws1_ctrl_stat m_stat; // the control state value 
  void rcv_ctrl_stat(); // recieve m_stat from m_ch_ctrl_stat.

  // AWS1's manual control mode, crz: Cruise mode (for usual crusing), ctl: Control mode (for precise control), csr: Cursor mode (AWS1 follows mouse cursor)
  enum e_ctrl_mode{
    cm_crz, cm_ctl, cm_csr, cm_ap, cm_stb, cm_undef
  } ctrl_mode;

  static const char * str_ctrl_mode[cm_undef];
  
  enum e_crz_cmd{
    crz_stp,
    crz_ds_ah, crz_sl_ah, crz_hf_ah, crz_fl_ah, crz_nf,
    crz_ds_as, crz_sl_as, crz_hf_as, crz_fl_as, 
    crz_mds,
    crz_p10,crz_p20,crz_hap,
    crz_s10,crz_s20, crz_has,
    crz_undef
  } crz_cm;

  static const char * str_crz_cmd[crz_undef];
  static const char * str_crz_cmd_exp[crz_undef];
  unsigned char crz_cmd_val[crz_undef];
  float sog_max, rev_max;
  float cog_tgt, sog_tgt, rev_tgt;
  
  float m_rud_f, m_meng_f, m_seng_f;
  void handle_ctrl_crz(); // cruise mode: sticks are used to increase/decrease engine throttle and rudder angle 
  void handle_ctrl_ctl(); // control mode: positions of sticks are the throttle values. 
  void handle_ctrl_csr(); // cursor mode: follows cursor position 

  void handle_ctrl_stb(); // stabilized mode: instruct cog and rpm
  
  ///////////////////////////////////////////////// joypad handlers (used for manual control)
  s_jc_u3613m m_js;			// joystick wrapper. (Now i only support jc_u3613) 
  bool bjs;					// joystick control enable flag (switched from fset, touch panel, and js's start button)
  int m_js_id;				// joystick id (glfw's ordering)
  const char * m_js_name;	// joystick name (glfw's naming)

  // mouse related members
  glm::vec2 pt_mouse, pt_mouse_drag_begin, pt_mouse_bih;
  glm::vec3 pt_mouse_ecef, pt_mouse_enu;
  int mouse_button, mouse_action, mouse_mods;
  s_obj obj_mouse_on;

  // calc_mouse_enu_and_ecef_pos calculates global position the mouse
  // pointer is pointing on. 
  void calc_mouse_enu_and_ecef_pos(e_ui_mode vm, Mat & Rown,
				   const float lat, const float lon, 
				   const float xown, const float yown,
				   const float zown, const float yaw);

  // these event handlers are called when the mouse event isnot handled in
  // ui boxes. 
  void handle_base_mouse_event(c_view_mode_box * pvm_box,
			       c_ctrl_mode_box * pcm_box,
			       c_map_cfg_box * pmc_box,
			       c_route_cfg_box * prc_box);
  
  void handle_mouse_lbtn_push(c_view_mode_box * pvm_box,
			      c_ctrl_mode_box * pcm_box,
			      c_map_cfg_box * pmc_box,
			      c_route_cfg_box * prc_box);
  void handle_mouse_lbtn_release(c_view_mode_box * pvm_box,
				 c_ctrl_mode_box * pcm_box,
				 c_map_cfg_box * pmc_box,
				 c_route_cfg_box * prc_box);
  void handle_mouse_mv(c_view_mode_box * pvm_box,
		       c_ctrl_mode_box * pcm_box,
		       c_map_cfg_box * pmc_box,
		       c_route_cfg_box * prc_box);
  void handle_mouse_drag(c_view_mode_box * pvm_box, s_obj & obj_tmp);
  void clear_mouse_event()
  {
    mouse_button = -1;
    mouse_action = -1;
    mouse_mods = -1;
  }

  void clear_mouse_state(c_route_cfg_box * prc_box)
  {
	  if (mouse_state == ms_add_wp) {
		  prc_box->command_processed(c_route_cfg_box::wp_add);
	  }
	  mouse_state = ms_normal;
  }
  
  void add_waypoint(c_route_cfg_box * prc_box);
  void drag_waypoint();
  void drag_cam_dir();
  void drag_map();
  void det_obj_collision();

  // map related members
  bool bmap_center_free;
  Mat Rmap;
  glm::vec2 pt_map_center_bih;
  glm::vec3 pt_map_center_ecef;

  unsigned int map_range_base;
  unsigned int map_range /* radius in meter */;
  float meter_per_pix, pix_per_meter;
  
  unsigned int get_max_circle_radius()
  {
    if(m_sz_win.height < m_sz_win.width){
      return m_sz_win.height >> 1;
    }
    return m_sz_win.width >> 1;
  }
  
  void recalc_range()
  {
    bupdate_map = true;
    meter_per_pix = (float)((float)map_range
			    / (float)(get_max_circle_radius()));
    pix_per_meter = (float)(1.0 / meter_per_pix);
  }

  // FPV related member
  glm::mat4 pm, vm, pvm, mm;
  glm::vec3 light;

  // video/screen capture related members
  VideoWriter m_vw;
  bool m_bsvw;         // screen video write
  bool m_bss;          // screen shot
  Mat m_simg;	       // screen img
  void print_screen();

  // glfw event handler 
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
  virtual void _key_callback(int key, int scancode, int action, int mods);

 public:
  f_aws1_ui(const char * name);
  virtual ~f_aws1_ui();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();

  // If LT+LB+RT+RB is detected, the system forces the controls to be nutral state. Called by default.
  void ui_force_ctrl_stop(c_ctrl_mode_box * pcm_box);
  void js_force_ctrl_stop(c_ctrl_mode_box * pcm_box);
  bool m_quit;
};
#endif
