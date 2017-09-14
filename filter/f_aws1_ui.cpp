// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ui.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ui.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ui.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <list>
#include <map>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>


#include "../util/aws_glib.h"
#include "f_aws1_ui.h"


f_aws1_ui::f_aws1_ui(const char * name): f_glfw_window(name), 
m_state(NULL), m_ch_sys(NULL), m_ch_ctrl_inst(NULL), m_ch_ctrl_stat(NULL), m_ch_wp(NULL), m_ch_map(NULL), 
m_ch_obj(NULL), m_ch_ais_obj(NULL), m_ch_obst(NULL), 
m_ch_ap_inst(NULL),
m_js_id(0), m_bsvw(false), m_bss(false),
fov_cam_x(55.0f), fcam(0), height_cam(2.0f), dir_cam_hdg(0.f), dir_cam_hdg_drag(0.f),
num_max_wps(100), num_max_ais(100),
bupdate_map(true), pt_prev_map_update(0,0,0),
map_range(100), sz_mark(10.0f), mouse_state(ms_normal),
bmap_center_free(false)
{
	m_path_storage[0] = '.';m_path_storage[1] = '\0';

  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_sys", (ch_base**)&m_ch_sys, typeid(ch_aws1_sys).name(), "System property channel");
  register_fpar("ch_ctrl_inst", (ch_base**)&m_ch_ctrl_inst, typeid(ch_aws1_ctrl_inst).name(), "Control input channel.");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");
  register_fpar("ch_wp", (ch_base**)&m_ch_wp, typeid(ch_wp).name(), "Waypoint channel");
  register_fpar("ch_map", (ch_base**)&m_ch_map, typeid(ch_map).name(), "Map channel");
  register_fpar("ch_obj", (ch_base**)&m_ch_obj, typeid(ch_obj).name(), "Object channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ch_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel");
  register_fpar("ch_obst", (ch_base**)&m_ch_obst, typeid(ch_obst).name(), "Obstacle channel.");
  register_fpar("ch_ap_inst", (ch_base**)&m_ch_ap_inst, typeid(ch_aws1_ap_inst).name(), "Autopilot instruction channel");

  fvs[0] = ffs[0] = '\0';
  register_fpar("fvs", fvs, 1024, "File path to the vertex shader program.");
  register_fpar("ffs", ffs, 1024, "File path to the fragment shader program.");
  register_fpar("ffnttex", ftex, 1024, "File path to the text texture.");
  register_fpar("ffnttexinf", ftexinf, 1024, "File path to the text texture information corresponding to ffnttex.");

  register_fpar("storage", m_path_storage, 1024, "Path to the storage device");

  register_fpar("acs", (int*) &m_stat.ctrl_src, (int) ACS_NONE, str_aws1_ctrl_src, "Control source.");
  register_fpar("verb", &m_verb, "Debug mode.");
  
  register_fpar("js", &m_js_id, "Joystick id");

  register_fpar("nwps", &num_max_wps, "Maximum number of waypoints.");
  register_fpar("nais", &num_max_ais, "Maximum number of ais objects.");
  register_fpar("sz_mark", &sz_mark, "Radius of the waypoint marker.");

  register_fpar("fov", &fov_cam_x, "Field of view in FPV mode.");
  register_fpar("fcam", &fcam, "Focal length of the camera assumed in FPV mode in pixel(calculated from fov if not given).");

  register_fpar("height_cam", &height_cam, "Camera height in FPV mode");
  register_fpar("dir_cam_hdg", &dir_cam_hdg, "Camera direction relative to the ship heading.");

  register_fpar("free_map_center", &bmap_center_free, "Free map center.");
  register_fpar("map_range", &map_range, "Range of the map (Radius in meter).");

  register_fpar("ss", &m_bss, "Screen shot now.");
  register_fpar("svw", &m_bsvw, "Screen video write.");
}


f_aws1_ui::~f_aws1_ui()
{
}

bool f_aws1_ui::init_run()
{
	if (!m_state)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "State channel is not connected." << endl;
		return false;
	}

	if (!m_ch_ctrl_inst)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "Control instruction channel is not connected. " << endl;
	}

	if (!m_ch_ctrl_stat)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "Control state channel is not connected." << endl;
	}

	if (!m_ch_map)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "Map channel is not connected." << endl;
	}

	if (!m_ch_ais_obj)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "AIS object channel is not connected." << endl;
	}

	if (!m_ch_obst)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "Obstacle channel is not connected." << endl;
	}

	if (!m_ch_ap_inst)
	{
		cerr << "In filter " << m_name << ", ";
		cerr << "Autopilot instruction channel is not connected." << endl;
	}

  m_inst.ctrl_src = ACS_UI;
  m_inst.tcur = get_time();
  m_inst.rud_aws = 127;
  m_inst.meng_aws = 127;
  m_inst.seng_aws = 127;

  if(!f_glfw_window::init_run())
    return false;

  if(m_js.init(m_js_id)){
	  cout << "Joystick " << m_js.name << " found." << endl;
  }

    ///////////////////////////////// Preparing graphics resources///////////////////////////
  if (!setup_shader()){
	  return false;
  }

  inv_sz_half_scrn[0] = (float)(2. / (float)m_sz_win.width);
  inv_sz_half_scrn[1] = (float)(2. / (float)m_sz_win.height);
  if (fcam == 0.0f){
	  fcam = (float)((float)(m_sz_win.width >> 1) / tan(fov_cam_x * (0.5 *  PI / 180.0f)));
	  ifcam = (float)(1.0 / fcam);
  }
  else{ // fov is overriden
	  ifcam = (float)(1.0 / fcam);
	  fov_cam_x = (float)(2.0 * atan((m_sz_win.width >> 1) * ifcam) * 180.0f / PI);
  }
  fov_cam_y = (float)(2.0 * atan((m_sz_win.height >> 1) * ifcam) * 180.0f / PI);

  // calculating horizon related parameters 
  iRE = (float)(1.0 / RE);
  height_cam_ec = (float)(RE + height_cam);
  dhorizon_cam = (float)sqrt(-RE * RE + height_cam_ec * height_cam_ec);
  th_horizon = (float) acos(RE / height_cam_ec); // angle of the arc to the horizon
  dhorizon_arc = (float)(RE * th_horizon);
  zhorizon = dhorizon_cam * height_cam_ec * iRE;

  recalc_range();

  {
	  glm::vec2 lb(0, 0);
	  glm::vec2 sz(1, 1);
	  if (!orect.init_rectangle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, lb, sz, 256))
		  return false;
	  if (!otri.init_circle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 3, 1, 1, 256))
		  return false;
	  if (!ocirc.init_circle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 10, 1, 1, 256))
		  return false;
	  if (!otxt.init(ftex, ftexinf, loc_mode, loc_pos2d, loc_texcoord, loc_sampler, loc_gcolor, loc_gcolorb, loc_depth2d, 65535))
		  return false;
	  if (!oline.init(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 8192))
		  return false;
  }

  glm::vec4 clr(0, 1, 0, 1);
  glm::vec4 clrb(0, 0, 0, 0);
  glm::vec2 sz_fnt(20, 20), sz_fnt_small(10, 10);
  glm::vec2 sz_scrn(m_sz_win.width, m_sz_win.height);
  glm::vec2 sz_mark_xy((float)(2.0 * sz_mark), (float)(2.0 * sz_mark));
  glm::vec2 sz_ship2d(sz_fnt.x, (float)(sz_fnt.y * 0.5));

  // initializing ui components
  uim.init(&orect, &otri, &otxt, &oline, 
	  clr, clrb, sz_fnt, fov_cam_x, sz_scrn);

  if (!ind.init(&oline, &otxt, &orect, &otri, sz_fnt, clr, fov_cam_x, sz_scrn))
	  return false;
 
  if (!owp.init(&ocirc, &otxt, &oline, clr, sz_fnt_small, sz_mark, num_max_wps))
	  return false;

  if (!oais.init(&orect, &otri, &otxt, &oline, clr, sz_fnt_small, sz_mark_xy, num_max_ais))
	  return false;

  if (!own_ship.init(&otri, &oline, clr, sz_ship2d))
	  return false;

  if (!ocsr.init(&oline, &otxt, clr, sz_fnt, sz_fnt))
	  return false;
  
  if (!coast_line.init(&oline, clr, 4096))
	  return false;

  // Visible object
  visible_obj.resize(ot_nul, true);
  
  // Initializing OpenGL flags
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  mouse_button = -1;
  mouse_action = -1;
  mouse_mods = -1;

  return true;
}

bool f_aws1_ui::setup_shader()
{
	load_glsl_program(ffs, fvs, p);

	loc_inv_sz_half_scrn = glGetUniformLocation(p, "inv_sz_half_scrn");
	loc_Mmvp = glGetUniformLocation(p, "Mmvp");
	loc_Mm = glGetUniformLocation(p, "Mm");
	loc_Lpar = glGetUniformLocation(p, "Lpar");
	loc_sampler = glGetUniformLocation(p, "sampler");
	loc_depth2d = glGetUniformLocation(p, "depth2d");
	loc_position = glGetAttribLocation(p, "position");
	loc_normal = glGetAttribLocation(p, "normal");
	loc_texcoord = glGetAttribLocation(p, "texcoord");

	loc_mode = glGetUniformLocation(p, "mode");
	loc_gcolor = glGetUniformLocation(p, "gcolor");
	loc_gcolorb = glGetUniformLocation(p, "gcolorb");
	loc_pos2d = glGetAttribLocation(p, "pos2d");

	return true;
}


void f_aws1_ui::destroy_run()
{
}

void f_aws1_ui::ui_force_ctrl_stop(c_ctrl_mode_box * pcm_box)
{
	if(m_js.id != -1){
		if(m_js.elb & s_jc_u3613m::EB_STDOWN &&
			m_js.elt & s_jc_u3613m::EB_STDOWN &&
			m_js.erb & s_jc_u3613m::EB_STDOWN &&
			m_js.ert & s_jc_u3613m::EB_STDOWN){
				m_inst.ctrl_src = ACS_UI;
				ctrl_mode = cm_crz;
				pcm_box->set_mode(c_ctrl_mode_box::crz);
		}
	}
}

void f_aws1_ui::cnv_img_to_view(Mat & img, float av, Size & sz, bool flipx, bool flipy)
{
	if(!img.empty()){
		if(sz.width != img.cols || sz.height != img.rows){
			float ai = (float)((float) img.cols / (float) img.rows);
			if(av > ai){
				sz.width = (int)((float)img.cols * ((float)sz.height / (float)img.rows));
			}else{
				sz.height = (int)((float)img.rows * ((float)sz.width / (float)img.cols));				
			}
			sz.width &= 0xFFFFFFFE;
			sz.height &= 0xFFFFFFFE;

			Mat tmp;
			resize(img, tmp, sz);
			img = tmp;
		}else{
			Mat tmp;
			tmp = img.clone();
			img = tmp;
		}

		awsFlip(img, flipx, flipy, false);
		GLenum fmt = GL_LUMINANCE;
		GLenum clr = GL_UNSIGNED_BYTE;
		switch(img.type()){
		case CV_8U:
			break;
		case CV_16U:
			clr = GL_UNSIGNED_SHORT;
			break;
		case CV_8UC3:
			fmt = GL_BGR;
			clr = GL_UNSIGNED_BYTE;
			break;
		case CV_16UC3:
			fmt = GL_BGR;
			clr = GL_UNSIGNED_SHORT;
			break;
		}
		glDrawPixels(img.cols, img.rows, fmt, clr, img.data);
	}
}

void f_aws1_ui::print_screen()
{
	if(m_bsvw || m_bss){
		if(m_simg.cols != m_sz_win.width || m_simg.rows != m_sz_win.height)
			m_simg.create(m_sz_win.height, m_sz_win.width, CV_8UC3);

		glReadPixels(0, 0, m_sz_win.width, m_sz_win.height, GL_BGR, GL_UNSIGNED_BYTE, m_simg.data);
		awsFlip(m_simg, false, true, false);

		if(m_bsvw){
			if(!m_vw.isOpened()){
				char fname[1024];
				double fps = (double) SEC / (double) get_period();
				int fourcc = CV_FOURCC('D', 'I', 'V', 'X');
				snprintf(fname, 1024, "%s/%s_%lld.avi", m_path_storage, m_name, get_time());

				m_vw.open(fname, fourcc, fps, m_sz_win, true);
				if(!m_vw.isOpened()){
					cerr << "Failed to create " << fname << endl;
					m_bsvw = false;
				}
			}else{
				m_vw.write(m_simg);
			}
		}else{
			if(m_vw.isOpened()){
				m_vw.release();
			}
		}

		if(m_bss){
			char fname[1024];
			snprintf(fname, 1024, "%s/%s_%lld.png", m_path_storage, m_name, get_time());
			imwrite(fname, m_simg);
		}
	}
}

void f_aws1_ui::update_route()
{
	if (!m_ch_wp)
		return;

	owp.disable();

	if (visible_obj[ui_obj_wp])
		return;
	m_ch_wp->lock();
	int iwp = 0;
	for (m_ch_wp->begin(); !m_ch_wp->is_end(); m_ch_wp->next())
	{
		s_wp wp = m_ch_wp->cur();
		eceftowrld(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
			wp.x, wp.y, wp.z, wp.rx, wp.ry, wp.rz);
		owp.update_wps(iwp, wp);
		owp.enable(iwp);

		iwp++;
	}

	float ddiff, cdiff;
	m_ch_wp->get_diff(ddiff, cdiff);
	owp.set_next(m_ch_wp->get_next(), ddiff, cdiff);

	if (obj_mouse_on.type == ot_wp){
		owp.set_focus(obj_mouse_on.handle);
		if (obj_mouse_on.handle != m_ch_wp->get_focus())
			m_ch_wp->set_focus(obj_mouse_on.handle);
		obj_mouse_on.type = ot_nul;
		obj_mouse_on.handle = -1;
	}
	owp.set_focus(m_ch_wp->get_focus());

	m_ch_wp->unlock();
	owp.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
	owp.set_map_param(pix_per_meter, Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);

	owp.update_drawings();
}

void f_aws1_ui::update_ais_objs()
{
	if (!m_ch_ais_obj)
		return;

	oais.disable();

	if (!visible_obj[ui_obj_vsl])
		return;

	m_ch_ais_obj->lock();
	int iobj = 0;
	for (m_ch_ais_obj->begin(); !m_ch_ais_obj->is_end(); m_ch_ais_obj->next()){
		{
			float bear = 0.f, dist = 0.f;
			if (!m_ch_ais_obj->get_pos_bd(bear, dist) || dist > map_range)
				continue;
		}

		oais.enable(iobj);
		oais.update_ais_obj(iobj, m_ch_ais_obj->cur());
		iobj++;
	}
	m_ch_ais_obj->unlock();

	oais.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
	oais.set_map_param(pix_per_meter, Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);
	
	oais.update_drawings();
}

void f_aws1_ui::update_map()
{
  cout << "update_map 0" << endl;
  
  if (!m_ch_map)
    return;
  coast_line.set_fpv_param(pvm, glm::vec2(m_sz_win.width, m_sz_win.height));
  coast_line.set_map_param(pix_per_meter, Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);
  // bupdate_map is asserted when map_scale is changed or drawing object types are re-selected

  cout << "update_map 1" << endl;
  if (glm::distance(pt_prev_map_update, pt_map_center_ecef) > 0.5 * map_range || bupdate_map){
    // update map

    m_ch_map->lock();
    m_ch_map->set_center(pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);
    m_ch_map->set_range((float)(2 * map_range));
    m_ch_map->set_resolution(meter_per_pix);

    m_ch_map->unlock();

    bupdate_map = false;
  }
  cout << "update_map 2" << endl;
  if (visible_obj[ui_obj_cl])
    {
      list<const AWSMap2::LayerData *> layerData;
      m_ch_map->lock();
      layerData = m_ch_map->get_layer_data(AWSMap2::lt_coast_line);
      coast_line.update_points(layerData);
      m_ch_map->unlock();
    }
  cout << "update_map 3" << endl;
}

void f_aws1_ui::render_gl_objs()
{	
	// the image is completely fitted to the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	glUseProgram(p);
	glUniform2fv(loc_inv_sz_half_scrn, 1, inv_sz_half_scrn);


	orect.render();
	otri.render();
	ocirc.render();
	otxt.render(0);
	oline.render();

	// show rendering surface.
	glfwSwapBuffers(pwin());	
}

bool f_aws1_ui::proc()
{
   c_view_mode_box * pvm_box =
    dynamic_cast<c_view_mode_box*>(uim.get_ui_box(c_aws_ui_box_manager::view_mode));
  c_ctrl_mode_box * pcm_box = 
    dynamic_cast<c_ctrl_mode_box*>(uim.get_ui_box(c_aws_ui_box_manager::ctrl_mode));
  c_map_cfg_box * pmc_box = 
    dynamic_cast<c_map_cfg_box*>(uim.get_ui_box(c_aws_ui_box_manager::map_cfg));
  c_route_cfg_box * prc_box =
    dynamic_cast<c_route_cfg_box*>(uim.get_ui_box(c_aws_ui_box_manager::route_cfg));

  // process joypad inputs
  if(m_js.id != -1){
    m_js.set_btn();
    m_js.set_stk();
  }

  ui_force_ctrl_stop(pcm_box);
  
  switch (ctrl_mode){
  case cm_crz:
    handle_ctrl_crz();
    break;
  case cm_ctl:
    handle_ctrl_ctl();
    break;
  case cm_csr:
    handle_ctrl_csr();
    break;
  }

  //m_inst to m_ch_ctrl_inst
	snd_ctrl_inst();

	// m_ch_ctrl_stat to m_stat
	rcv_ctrl_stat();

	// Window forcus is now at this window
	glfwMakeContextCurrent(pwin());

	// UI polling.
	glfwPollEvents();

	// loading states
	long long t;
	float roll, pitch, yaw, cog, sog, vx, vy;
	float xown, yown, zown;
	m_state->get_attitude(t, roll, pitch, yaw);
	m_state->get_velocity(t, cog, sog);
	m_state->get_velocity_vector(t, vx, vy);
	m_state->get_position_ecef(t, xown, yown, zown);

	float lat, lon, alt, galt;
	Mat Rown = m_state->get_enu_rotation(t);
	m_state->get_position(t, lat, lon, alt, galt);

	float depth;
	m_state->get_depth(t, depth);

	// handling ui events
	bool event_handled = uim.set_mouse_event(pt_mouse, mouse_button, mouse_action, mouse_mods);

	// calculating mouse point coordinate
	calc_mouse_enu_and_ecef_pos(pvm_box->get_mode(), Rown, lat, lon, xown, yown, zown, yaw);
	if (uim.set_mouse_event(pt_mouse, mouse_button, mouse_action, mouse_mods)){
		handle_updated_ui_box(pvm_box, pcm_box, pmc_box, prc_box);
	}
	else{
		handle_base_mouse_event(pvm_box, pcm_box, pmc_box, prc_box);
	}
	
 	// update ui params 
	update_ui_params(pvm_box, xown, yown, zown, vx, vy, yaw);

	// updating indicator
	update_indicator(cog, sog, roll, pitch, yaw);

	// update route object
	update_route();

	// update ais object
	update_ais_objs();

	// update coast line (map)
	update_map();

	// rendering graphics
	render_gl_objs();

	// screen shot or video capturue
	print_screen();
 
	return true;
}

void f_aws1_ui::snd_ctrl_inst()
{
  if(m_ch_ctrl_inst){
    m_ch_ctrl_inst->set(m_inst);
  }
}

void f_aws1_ui::rcv_ctrl_stat()
{
	s_aws1_ctrl_stat stat;
  if(m_ch_ctrl_stat){
    m_ch_ctrl_stat->get(stat);
  }else 
	  return;
  m_stat.ctrl_src = stat.ctrl_src;
  m_stat.rud_rmc = stat.rud_rmc;
  m_stat.meng_rmc = stat.meng_rmc;
  m_stat.seng_rmc = stat.seng_rmc;
  m_stat.rud = stat.rud;
  m_stat.meng = stat.meng;
  m_stat.seng = stat.seng;
  m_stat.rud_sta = stat.rud_sta;
  m_stat.rud_sta_out = stat.rud_sta_out;

  m_stat.rud_max_rmc = stat.rud_max_rmc;
  m_stat.rud_nut_rmc = stat.rud_nut_rmc;
  m_stat.rud_min_rmc = stat.rud_min_rmc;

  m_stat.meng_max_rmc = stat.meng_max_rmc;
  m_stat.meng_nuf_rmc = stat.meng_nuf_rmc;
  m_stat.meng_nut_rmc = stat.meng_nut_rmc;
  m_stat.meng_nub_rmc = stat.meng_nub_rmc;
  m_stat.meng_min_rmc = stat.meng_min_rmc;

  m_stat.seng_max_rmc = stat.seng_max_rmc;
  m_stat.seng_nuf_rmc = stat.seng_nuf_rmc;
  m_stat.seng_nut_rmc = stat.seng_nut_rmc;
  m_stat.seng_nub_rmc = stat.seng_nub_rmc;
  m_stat.seng_min_rmc = stat.seng_min_rmc;

  m_stat.rud_sta_max = stat.rud_sta_max;
  m_stat.rud_sta_nut = stat.rud_sta_nut;
  m_stat.rud_sta_min = stat.rud_sta_min;

  m_stat.meng_max = stat.meng_max;
  m_stat.meng_nuf = stat.meng_nuf;
  m_stat.meng_nut = stat.meng_nut;
  m_stat.meng_nub = stat.meng_nub;
  m_stat.meng_min = stat.meng_min;

  m_stat.seng_max = stat.seng_max;
  m_stat.seng_nuf = stat.seng_nuf;
  m_stat.seng_nut = stat.seng_nut;
  m_stat.seng_nub = stat.seng_nub;
  m_stat.seng_min = stat.seng_min;
  m_stat.rud_max = stat.rud_max;
  m_stat.rud_nut = stat.rud_nut;
  m_stat.rud_min = stat.rud_min;

  m_stat.rud_sta_out_max = stat.rud_sta_out_max;
  m_stat.rud_sta_out_nut = stat.rud_sta_out_nut;
  m_stat.rud_sta_out_min = stat.rud_sta_out_min;
}

void f_aws1_ui::handle_ctrl_crz()
{
	if (m_js.id != -1){
		m_rud_f += (float)(m_js.lr1 * (255. / 180.));
		m_rud_f += (float)(m_js.lr2 * (255. / 180.));
		m_rud_f = min((float)255.0, m_rud_f);
		m_rud_f = max((float)0.0, m_rud_f);

		m_meng_f -= (float)(m_js.ud1 * (255. / 180));
		m_meng_f = min((float)255.0, m_meng_f);
		m_meng_f = max((float)0.0, m_meng_f);

		m_seng_f -= (float)(m_js.ud2 * (255. / 180));
		m_seng_f = min((float) 255.0, m_seng_f);
		m_seng_f = max((float)0.0, m_seng_f);
	}
	m_inst.tcur = get_time();
	m_inst.rud_aws = (unsigned char)m_rud_f;
	m_inst.meng_aws = (unsigned char)m_meng_f;
	m_inst.seng_aws = (unsigned char)m_seng_f;
}

void f_aws1_ui::handle_ctrl_ctl()
{
	if (m_js.id != -1){
		m_rud_f = 127.5;
		m_rud_f += (float)(m_js.lr1 * (127.5));
		m_rud_f += (float)(m_js.lr2 * (127.5));
		m_rud_f = min((float)255.0, m_rud_f);
		m_rud_f = max((float)0.0, m_rud_f);

		m_meng_f = 127.5;
		m_meng_f -= (float)(m_js.ud1 * (127.5));
		m_meng_f = min((float)255.0, m_meng_f);
		m_meng_f = max((float)0.0, m_meng_f);

		m_seng_f = 127.5;
		m_seng_f -= (float)(m_js.ud2 * (127.5));
		m_seng_f = min((float) 255.0, m_seng_f);
		m_seng_f = max((float)0.0, m_seng_f);
	}
	m_inst.tcur = get_time();
	m_inst.rud_aws = (unsigned char)m_rud_f;
	m_inst.meng_aws = (unsigned char)m_meng_f;
	m_inst.seng_aws = (unsigned char)m_seng_f;
}

void f_aws1_ui::handle_ctrl_csr()
{
	float th_mouse = (float)(atan2(pt_mouse.y, pt_mouse.x) * 180.0 / PI);
	float d_mouse = (float)sqrt(pt_mouse_enu.x * pt_mouse_enu.x + pt_mouse_enu.y + pt_mouse_enu.y);

	m_ch_ap_inst->set_csr_pos(pt_mouse_bih.x, pt_mouse_bih.y, pt_mouse_enu.x, pt_mouse_enu.y, d_mouse, th_mouse);
}

void f_aws1_ui::calc_mouse_enu_and_ecef_pos(
	e_ui_mode vm, Mat & Rown,
	const float lat, const float lon,
	const float xown, const float yown, const float zown, const float yaw)
{
	if (vm == ui_mode_map)
	{
		if (!bmap_center_free){
			pt_map_center_ecef.x = xown;
			pt_map_center_ecef.y = yown;
			pt_map_center_ecef.z = zown;
			pt_map_center_bih.x = (float)(lat * PI / 180.0f);
			pt_map_center_bih.y = (float)(lon * PI / 180.0f);
			Rmap = Rown;
		}
		pt_mouse_enu.x = pt_mouse.x * meter_per_pix;
		pt_mouse_enu.y = pt_mouse.y * meter_per_pix;
		pt_mouse_enu.z = 0.f;
		float alt = 0;
		wrldtoecef(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
			pt_mouse_enu.x, pt_mouse_enu.y, pt_mouse_enu.z,
			pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z);
		eceftobih(pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z,
			pt_mouse_bih.x, pt_mouse_bih.y, alt);
	}
	else if (vm == ui_mode_fpv)
	{
		pt_map_center_ecef.x = xown;
		pt_map_center_ecef.y = yown;
		pt_map_center_ecef.z = zown;
		pt_map_center_bih.x = lat;
		pt_map_center_bih.y = lon;
		Rmap = Rown;

		glm::vec2 phi(atan2(pt_mouse.x, fcam), atan2(-pt_mouse.y, fcam));
		float th, RE_sin_th;
		if (phi.y > th_horizon) {
			phi.y = th_horizon;
			th = th_horizon;
		}
		else {
			th = (float)(phi.y - asin((height_cam_ec * iRE) * cos(phi.y)) - 0.5 * PI);
		}
		RE_sin_th = (float)(RE * sin(th));
		glm::vec3 pcam(
			(float)(RE_sin_th * sin(phi.x)),
			(float)(RE_sin_th * (-pt_mouse.y * ifcam)),
			(float)(RE_sin_th * cos(phi.x))
		);

		float c, s, thcam = (float)((PI / 180.0) * (yaw + dir_cam_hdg));
		c = (float)cos(thcam);
		s = (float)sin(thcam);

		pt_mouse_enu.x = (float)(c * pcam.x + s * pcam.y);
		pt_mouse_enu.y = (float)(-s * pcam.x + c * pcam.y);
		pt_mouse_enu.z = (float)(pcam.z - height_cam);

		float alt = 0;
		wrldtoecef(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
			pt_mouse_enu.x, pt_mouse_enu.y, pt_mouse_enu.z,
			pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z);
		eceftobih(pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z,
			pt_mouse_bih.x, pt_mouse_bih.y, alt);
	}
}

void f_aws1_ui::add_waypoint(c_route_cfg_box * prc_box)
{
	unsigned int rt, spd, wp;
	prc_box->get_params(wp, spd, rt);
	// set current cursor position as the new waypoint
	if (!m_ch_wp){
		cerr << "No waypoint channel is connected to filter " << m_name << endl;
		return;
	}

	m_ch_wp->lock();
	m_ch_wp->ins(pt_mouse_bih.x, pt_mouse_bih.y, 10.0, (float)spd);
	m_ch_wp->unlock();
}

void f_aws1_ui::drag_waypoint()
{
	m_ch_wp->lock();
	s_wp & wp = m_ch_wp->get_focused_wp();
	wp.lat = pt_mouse_bih.x;
	wp.lon = pt_mouse_bih.y;
	bihtoecef(wp.lat, wp.lon, 0., wp.x, wp.y, wp.z);

	m_ch_wp->unlock();
}

void f_aws1_ui::drag_map()
{
	if (!bmap_center_free){
		bmap_center_free = true;
	}
	// change map center
	glm::vec2 d_pt_mouse = pt_mouse_drag_begin - pt_mouse;
	d_pt_mouse.x *= meter_per_pix;
	d_pt_mouse.y *= meter_per_pix;
	glm::vec3 pt_map_center_ecef_new;
	float alt;
	wrldtoecef(Rmap, 
		pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
		d_pt_mouse.x, d_pt_mouse.y, 0.,
		pt_map_center_ecef_new.x, pt_map_center_ecef_new.y, pt_map_center_ecef_new.z);
	pt_map_center_ecef = pt_map_center_ecef_new;

	eceftobih(pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
		pt_map_center_bih.x, pt_map_center_bih.y, alt);
	getwrldrot(pt_map_center_bih.x, pt_map_center_bih.y, Rmap);
	pt_mouse_drag_begin = pt_mouse;
}

void f_aws1_ui::drag_cam_dir()
{
	float th0 = (float) atan2(pt_mouse.x, fcam);
	float th1 = (float) atan2(pt_mouse_drag_begin.x,fcam);
	dir_cam_hdg_drag = (float)(th1 - th0);
}

void f_aws1_ui::det_obj_collision()
{
	int handle;
	obj_mouse_on.handle = -1;
	obj_mouse_on.type = ot_nul;

	handle = owp.collision(pt_mouse);
	if (handle >= 0){
		obj_mouse_on.handle = handle;
		obj_mouse_on.type = ot_wp;
	}

	handle = oais.collision(pt_mouse);
	if (handle >= 0){
		obj_mouse_on.handle = handle;
		obj_mouse_on.type = ot_ais;
	}
}

void f_aws1_ui::handle_mouse_lbtn_push(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_map_cfg_box * pmc_box, c_route_cfg_box * prc_box)
{
	det_obj_collision();
	switch (mouse_state){
	case ms_normal:
		pt_mouse_drag_begin = pt_mouse;
		mouse_state = ms_drag;
		break;
	}
}

void f_aws1_ui::handle_mouse_lbtn_release(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_map_cfg_box * pmc_box, c_route_cfg_box * prc_box)
{
	s_obj obj_tmp = obj_mouse_on;
	det_obj_collision();
	switch (mouse_state){
	case ms_add_wp:
		if (obj_tmp.type == ot_nul && obj_mouse_on.type == ot_nul)
		{
			add_waypoint(prc_box);
		}
		prc_box->command_processed(c_route_cfg_box::wp_add);
		mouse_state = ms_normal;
		break;
	case ms_drag:
		handle_mouse_drag(pvm_box, obj_tmp);
		dir_cam_hdg += dir_cam_hdg_drag;
		dir_cam_hdg_drag = 0.f;
		mouse_state = ms_normal;
		break;
	case ms_normal:
		break;
	}
}

void f_aws1_ui::handle_mouse_mv(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_map_cfg_box * pmc_box, c_route_cfg_box * prc_box)
{
	switch (mouse_state){
	case ms_drag:
		handle_mouse_drag(pvm_box, obj_mouse_on);
	}
}

void f_aws1_ui::handle_mouse_drag(c_view_mode_box * pvm_box, s_obj & obj_tmp)
{
	if (obj_tmp.type == ot_wp){
		drag_waypoint();
	}
	else if (obj_tmp.type == ot_nul)
	{
		if (pvm_box->get_mode() == ui_mode_fpv){
			// change dir_cam_hdg
			drag_cam_dir();
		}
		else if (pvm_box->get_mode() == ui_mode_map){
			drag_map();
		}
	}
}

void f_aws1_ui::handle_updated_ui_box(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box, c_map_cfg_box * pmc_box, c_route_cfg_box * prc_box) 
{
	e_mouse_state mouse_state_new = ms_normal;
	switch (uim.get_box_updated()){
	case c_aws_ui_box_manager::view_mode:
		update_view_mode_box(pvm_box);
		break;
	case c_aws_ui_box_manager::ctrl_mode:
		update_ctrl_mode_box(pcm_box);
		break;
	case c_aws_ui_box_manager::map_cfg:
		update_map_cfg_box(pmc_box);
		break;
	case c_aws_ui_box_manager::route_cfg:
		update_route_cfg_box(prc_box, mouse_state_new);
		break;
	}

	uim.reset_box_updated();

	ocsr.enable_arrow();
	ocsr.set_cursor_position(pt_mouse, pt_mouse_bih);
	clear_mouse_state();
}


void f_aws1_ui::update_view_mode_box(c_view_mode_box * pvm_box)
{
	ind.set_mode(pvm_box->get_mode());
	owp.set_ui_mode(pvm_box->get_mode());
	oais.set_ui_mode(pvm_box->get_mode());
	coast_line.set_ui_mode(pvm_box->get_mode());
}

void f_aws1_ui::update_ctrl_mode_box(c_ctrl_mode_box * pcm_box)
{	
	switch (pcm_box->get_mode())
	{
	case c_ctrl_mode_box::crz:
		m_inst.ctrl_src = ACS_UI;
		ctrl_mode = cm_crz;
		break;
	case c_ctrl_mode_box::csr:
		m_inst.ctrl_src = ACS_AP1;
		ctrl_mode = cm_csr;
		m_ch_ap_inst->set_mode(EAP_CURSOR);
		break;
	case c_ctrl_mode_box::ctl:
		m_inst.ctrl_src = ACS_UI;
		ctrl_mode = cm_ctl;
		break;
	case c_ctrl_mode_box::sty:
		m_inst.ctrl_src = ACS_AP1;
		m_ch_ap_inst->set_mode(EAP_STAY);
		{
			long long t; 
			float lat, lon, alt, galt;
			m_state->get_position(t, lat, lon, alt, galt);
			m_ch_ap_inst->set_stay_pos(lat, lon);
		}	
		break;
	case c_ctrl_mode_box::fwp:
		m_inst.ctrl_src = ACS_AP1;
		m_ch_ap_inst->set_mode(EAP_WP);
		break;
	case c_ctrl_mode_box::ftg:
		m_inst.ctrl_src = ACS_AP1;
		m_ch_ap_inst->set_mode(EAP_FLW_TGT);
		break;
	}
}

void f_aws1_ui::update_map_cfg_box(c_map_cfg_box * pmc_box)
{
	pmc_box->get_params(map_range, visible_obj);
	switch (pmc_box->get_command())
	{
	case c_map_cfg_box::range_up:
		if(map_range < 10000000){
			map_range *= 10.f;
			recalc_range();
		}
		break;
	case c_map_cfg_box::range_down:
		if(map_range > 100){
			map_range *= 0.1f;
			recalc_range();
		}
		break;
	case c_map_cfg_box::wp:
		break;
	case c_map_cfg_box::vsl:
		break;
	case c_map_cfg_box::cl:
		bupdate_map = true;
		break;
	case c_map_cfg_box::mrk:
		break;
	}
	pmc_box->set_params(map_range, visible_obj);
}

void f_aws1_ui::update_route_cfg_box(c_route_cfg_box * prc_box, e_mouse_state mouse_state_new)
{
	if (!m_ch_wp){
		cerr << "No waypoint channel is connected to filter " << m_name << endl;
		return;
	}

	m_ch_wp->lock();
	bool bno_focused_wp = m_ch_wp->get_focus() == m_ch_wp->get_num_wps();
	s_wp fwp;

	unsigned int rt, spd, wp;
	prc_box->get_params(wp, spd, rt);
	if (!bno_focused_wp)
		fwp = m_ch_wp->get_focused_wp();
	else{
		fwp.v = (float)spd;	
	}

	switch (prc_box->get_command())
	{
	case c_route_cfg_box::wp_prev:
		m_ch_wp->prev_focus();
		wp = m_ch_wp->get_focus();
		owp.set_focus(wp);	
		bno_focused_wp = wp == m_ch_wp->get_num_wps();
		if (!bno_focused_wp)
			fwp = m_ch_wp->get_focused_wp();
		break;
	case c_route_cfg_box::wp_next:
		m_ch_wp->next_focus();
		wp = m_ch_wp->get_focus();
		owp.set_focus(wp);
		bno_focused_wp = wp == m_ch_wp->get_num_wps();
		if (!bno_focused_wp)
			fwp = m_ch_wp->get_focused_wp();
		break;
	case c_route_cfg_box::wp_spd_up:
		fwp.v += 1.0;
		fwp.v = min(fwp.v, 40.0f);
		break;
	case c_route_cfg_box::wp_spd_down:
		fwp.v -= 1.0;
		fwp.v = max(fwp.v, 0.0f);
		break;
	case c_route_cfg_box::wp_add:
		mouse_state_new = ms_add_wp;
		break;
	case c_route_cfg_box::wp_del:
		m_ch_wp->ers();
		break;
	case c_route_cfg_box::rt_prev:
		rt = m_ch_wp->get_route_id();
		if (rt == 0)
			rt = MAX_RT_FILES - 1;
		else
			rt--;
		m_ch_wp->set_route_id(rt);
		break;
	case c_route_cfg_box::rt_next:
		rt = m_ch_wp->get_route_id();
		if (rt == MAX_RT_FILES - 1)
			rt = 0;
		else
			rt++;
		m_ch_wp->set_route_id(rt);
		break;
	case c_route_cfg_box::rt_load:
		m_ch_wp->set_cmd(ch_wp::cmd_load);
		break;
	case c_route_cfg_box::rt_save:
		m_ch_wp->set_cmd(ch_wp::cmd_save);
		break;
	}

	if (mouse_state_new == mouse_state){
		mouse_state = ms_normal;
	}
	else{
		mouse_state = mouse_state_new;
	}

	wp = m_ch_wp->get_focus();

	if (!bno_focused_wp && wp < (unsigned int) m_ch_wp->get_num_wps() && wp >= 0)
		m_ch_wp->get_focused_wp() = fwp;
	spd = (unsigned int)fwp.v;
	prc_box->set_params(wp, spd, rt);
	prc_box->reset_command();
	m_ch_wp->unlock();

}

void f_aws1_ui::update_ui_params(c_view_mode_box * pvm_box,
	const float xown, const float yown, const float zown,
	const float vx, const float vy, const float yaw)
{
	if (pvm_box->get_mode() == ui_mode_fpv){ // calculating projection matrix
		float c, s, th = (float)(dir_cam_hdg + dir_cam_hdg_drag + yaw * PI / 180.);
		c = (float)cos(th);
		s = (float)sin(th);

		float ratio = (float)((float)m_sz_win.width / (float)m_sz_win.height);
		pm = glm::perspective((float)(fov_cam_y * PI / 180.0f), ratio, 1.f, 30e6f);
		vm = glm::lookAt(glm::vec3(0, 0, height_cam), glm::vec3(s, c, height_cam), glm::vec3(0, 0, 1));
		pvm = pm * vm;
		own_ship.disable();
	}
	else if (pvm_box->get_mode() == ui_mode_map){
		float rx, ry, rz;
		eceftowrld(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z, xown, yown, zown, rx, ry, rz);
		own_ship.enable();
		own_ship.set_param(rx, ry, rz, yaw, vx, vy, pix_per_meter);
	}
}

void f_aws1_ui::_cursor_position_callback(double xpos, double ypos){
	pt_mouse.x = (float)(xpos - (double)(m_sz_win.width >> 1));
	pt_mouse.y = (float)((double)(m_sz_win.height >> 1) - ypos);
}

void f_aws1_ui::_mouse_button_callback(int button, int action, int mods)
{
	mouse_button = button;
	mouse_action = action;
	mouse_mods = mods;
}

void f_aws1_ui::_key_callback(int key, int scancode, int action, int mods)
{
}
