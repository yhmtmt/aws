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

const char * f_aws1_ui::m_str_aws1_ui_mode[AUM_UNDEF] = {
	"normal", "map", "dev"
};

const char * f_aws1_ui::m_str_imv[IMV_UNDEF] = {
	"img1", "img2", "disp", "img12", "img12d"
};

f_aws1_ui::f_aws1_ui(const char * name): f_glfw_window(name), 
					m_state(NULL), m_ch_sys(NULL),
					 m_ch_ctrl_inst(NULL), m_ch_ctrl_stat(NULL), m_ch_wp(NULL),
					 m_ch_map(NULL),
					 m_ch_obj(NULL), m_ch_ais_obj(NULL), m_ch_img(NULL), m_ch_img2(NULL), m_ch_disp(NULL),
					 m_ch_obst(NULL), m_ch_ap_inst(NULL),
					 m_imv(IMV_IMG1), m_js_id(0),
					 m_img_x_flip(false), m_img_y_flip(false), m_img2_x_flip(false), m_img2_y_flip(false),
					 m_mode(AUM_NORMAL), m_ui_menu(false), m_menu_focus(0),
					 m_fx(0.), m_fy(0.), m_cx(0.), m_cy(0.), m_bsvw(false), m_bss(false),
					 fov_cam(55.0f), fcam(0), height_cam(2.0f), dir_cam_hdg(0.f), num_max_wps(100),
					 map_range(100), r_wp_mark(10.0f), mouse_state(ms_normal),
					 pt_map_center_enu(0, 0), bmap_center_free(false)
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
  register_fpar("ch_img", (ch_base**)&m_ch_img, typeid(ch_image_ref).name(), "Image channel");
  register_fpar("ch_img2", (ch_base**)&m_ch_img2, typeid(ch_image_ref).name(), "Second Image channel.");
  register_fpar("ch_disp", (ch_base**)&m_ch_disp, typeid(ch_image_ref).name(), "Disparity image between ch_img and ch_img2");
  register_fpar("ch_obst", (ch_base**)&m_ch_obst, typeid(ch_obst).name(), "Obstacle channel.");
  register_fpar("ch_ap_inst", (ch_base**)&m_ch_ap_inst, typeid(ch_aws1_ap_inst).name(), "Autopilot instruction channel");
  register_fpar("flip_img_x", &m_img_x_flip, "Image in ch_img is fliped in x direction.");
  register_fpar("flip_img_y", &m_img_y_flip, "Image in ch_img is fliped in y direction.");
  register_fpar("flip_img2_x", &m_img2_x_flip, "Image in ch_img is fliped in x direction.");
  register_fpar("flip_img2_y", &m_img2_y_flip, "Image in ch_img is fliped in y direction.");
  register_fpar("imv", (int*)&m_imv, IMV_UNDEF, m_str_imv, "Image view mode.");

  register_fpar("fvs", fvs, 1024, "File path to the vertex shader program.");
  register_fpar("ffs", ffs, 1024, "File path to the fragment shader program.");
  register_fpar("ffnttex", ftex, 1024, "File path to the text texture.");
  register_fpar("ffnttexinf", ftexinf, 1024, "File path to the text texture information corresponding to ffnttex.");

  register_fpar("storage", m_path_storage, 1024, "Path to the storage device");

  register_fpar("acs", (int*) &m_stat.ctrl_src, (int) ACS_NONE, str_aws1_ctrl_src, "Control source.");
  register_fpar("verb", &m_verb, "Debug mode.");
  
  register_fpar("js", &m_js_id, "Joystick id");

  register_fpar("mode", (int*)&m_mode, AUM_UNDEF, m_str_aws1_ui_mode, "UI mode.");
  m_ui[AUM_NORMAL]	= new c_aws1_ui_normal(this);
  m_ui[AUM_MAP]		= new c_aws1_ui_map(this);
  m_ui[AUM_DEV]		= new c_aws1_ui_dev(this);

  register_fpar("menu", &m_ui_menu, "Invoke menu");

  register_fpar("nwps", &num_max_wps, "Maximum number of waypoints.");
  register_fpar("rwp", &r_wp_mark, "Radius of the waypoint marker.");

  register_fpar("fov", &fov_cam, "Field of view in FPV mode.");
  register_fpar("fcam", &fcam, "Focal length of the camera assumed in FPV mode in pixel(calculated from fov if not given).");

  register_fpar("height_cam", &height_cam, "Camera height in FPV mode");
  register_fpar("dir_cam_hdg", &dir_cam_hdg, "Camera direction relative to the ship heading.");

  register_fpar("free_map_center", &bmap_center_free, "Free map center.");
  register_fpar("map_range", &map_range, "Range of the map (Radius in meter).");

  register_fpar("fx", &m_fx, "Focal length in x direction.");
  register_fpar("fy", &m_fy, "Focal length in y direction.");
  register_fpar("cx", &m_cx, "Principal point in x direction.");
  register_fpar("cy", &m_cy, "Principal point in y direction.");

  register_fpar("ss", &m_bss, "Screen shot now.");
  register_fpar("svw", &m_bsvw, "Screen video write.");
}


f_aws1_ui::~f_aws1_ui()
{
	delete m_ui[AUM_NORMAL];
	m_ui[AUM_NORMAL] = NULL;
	delete m_ui[AUM_MAP];
	m_ui[AUM_NORMAL] = NULL;
	delete m_ui[AUM_DEV];
	m_ui[AUM_DEV] = NULL;
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

bool f_aws1_ui::init_run()
{
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

  if (!setup_shader()){
	  return false;
  }

  inv_sz_half_scrn[0] = (float)(2. / (float)m_sz_win.width);
  inv_sz_half_scrn[1] = (float)(2. / (float)m_sz_win.height);
  if (fcam == 0.0f){
	  fcam = (float)((float)(m_sz_win.width >> 1) / tan(fov_cam * (0.5 *  PI / 180.0f)));
	  ifcam = (float)(1.0 / fcam);
  }
  else{ // fov is overriden
	  ifcam = (float)(1.0 / fcam);
	  fov_cam = (float)(2.0 * atan((m_sz_win.width >> 1) * ifcam) * 180.0f / PI);
  }

  // calculating horizon related parameters 
  iRE = (float)(1.0 / RE);
  height_cam_ec = (float)(RE + height_cam);
  dhorizon_cam = (float)sqrt(RE * RE + height_cam_ec * height_cam_ec);
  th_horizon_arc = acos(RE / height_cam_ec);
  dhorizon_arc = (float)(RE * th_horizon_arc);
  zhorizon = dhorizon_cam * height_cam_ec * iRE;

  recalc_range();

  {
	  glm::vec2 lb(0, 0);
	  glm::vec2 sz(1, 1);
	  if (!orect.init_rectangle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, lb, sz, 64))
		  return false;
	  if (!otri.init_circle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 3, 1, 1, 16))
		  return false;
	  if (!otri.init_circle(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 10, 1, 1, num_max_wps))
		  return false;
	  if (!otxt.init(ftex, ftexinf, loc_mode, loc_pos2d, loc_texcoord, loc_sampler, loc_gcolor, loc_gcolorb, loc_depth2d, 4096))
		  return false;

	  if (!oline.init(loc_mode, loc_pos2d, loc_gcolor, loc_depth2d, 4096))
		  return false;
  }

  glm::vec4 clr(0, 1, 0, 1);
  glm::vec4 clrb(0, 0, 0, 0);
  glm::vec2 sz_fnt(20, 20), sz_fnt_small(10, 10);
  glm::vec2 sz_scrn(m_sz_win.width, m_sz_win.height);
  uim.init(&orect, &otri, &otxt, &oline, 
	  clr, clrb, sz_fnt, fov_cam, sz_scrn);

  if (!ind.init(&oline, &otxt, &orect, &otri, sz_fnt, clr, fov_cam, sz_scrn))
	  return false;
  if (!owp.init(&ocirc, &otxt, &oline, clr, sz_fnt_small, r_wp_mark, num_max_wps))
	  return false;

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



void f_aws1_ui::destroy_run()
{
}

void f_aws1_ui::ui_force_ctrl_stop()
{
	if(m_js.id != -1){
		if(m_js.elb & s_jc_u3613m::EB_STDOWN &&
			m_js.elt & s_jc_u3613m::EB_STDOWN &&
			m_js.erb & s_jc_u3613m::EB_STDOWN &&
			m_js.ert & s_jc_u3613m::EB_STDOWN){
				((c_aws1_ui_normal *)m_ui[AUM_NORMAL])->set_ctrl(127, 127, 127);
				m_inst.ctrl_src = ACS_UI;
				m_mode = AUM_NORMAL;
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

void f_aws1_ui::ui_show_img()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	long long timg;
	Mat img, img2, disp;
	float av = m_xscale * m_iyscale;
	Size sz;
	switch(m_imv){
	case IMV_IMG1:
		if(m_ch_img){
			img = m_ch_img->get_img(timg);
			if(!img.empty()){
				glRasterPos2i(-1, -1);
				sz = m_sz_win;
				cnv_img_to_view(img, av, sz, m_img_x_flip, m_img_y_flip);
			}
		}
		break;
	case IMV_IMG2:
		if(m_ch_img2){
			img = m_ch_img2->get_img(timg);
			if(!img.empty()){
				glRasterPos2i(-1, -1);
				sz = m_sz_win;
				cnv_img_to_view(img, av, sz, m_img2_x_flip, m_img2_y_flip);
			}
		}
		break;
	case IMV_DISP:
		if(m_ch_disp){
			img = m_ch_disp->get_img(timg);
			if(!img.empty()){
				glRasterPos2i(-1, -1);
				sz = m_sz_win;
				cnv_img_to_view(img, av, sz, false, true);
			}
		}
		break;
	case IMV_IMG12:
		if(m_ch_img && m_ch_img2){			
			img = m_ch_img->get_img(timg);
			if(!img.empty()){				
				glRasterPos2i(-1, 0);
				sz.width = m_sz_win.width >> 1;
				sz.height = m_sz_win.height >> 1;
				cnv_img_to_view(img ,av, sz, m_img_x_flip, m_img_y_flip);
			}
			img2 = m_ch_img2->get_img(timg);
			if(!img2.empty()){
				glRasterPos2i(0, 0);
				sz.width = m_sz_win.width >> 1;
				sz.height = m_sz_win.height >> 1;
				cnv_img_to_view(img2, av, sz, m_img2_x_flip, m_img2_y_flip);
			}
		}
		break;
	case IMV_IMG12D:
		if(m_ch_img && m_ch_img2 && m_ch_disp){
			img = m_ch_img->get_img(timg);
			if(!img.empty()){				
				glRasterPos2i(-1, 0);
				sz.width = m_sz_win.width >> 1;
				sz.height = m_sz_win.height >> 1;
				cnv_img_to_view(img ,av, sz, m_img_x_flip, m_img_y_flip);
			}
			img2 = m_ch_img2->get_img(timg);
			if(!img2.empty()){
				glRasterPos2i(0, 0);
				sz.width = m_sz_win.width >> 1;
				sz.height = m_sz_win.height >> 1;
				cnv_img_to_view(img2, av, sz, m_img2_x_flip, m_img2_y_flip);
			}
			disp = m_ch_disp->get_img(timg);
			if(!disp.empty()){
				glRasterPos2f(-0.5, -1.);
				sz.width = m_sz_win.width >> 1;
				sz.height = m_sz_win.height >> 1;
				cnv_img_to_view(disp, av, sz, false, true);
			}
		}
	default:
		return;
	}
}

void f_aws1_ui::ui_show_rudder()
{
	glRasterPos2i(-1, -1);
	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float wm = (float)(255. * m_ixscale);
	float hm = (float)(1.5 * hfont);
	float lw = (float)(1.0 / m_sz_win.width);

	float rud_inst = (float)m_inst.rud_aws;
	float rud_inst_cur = 
		(float)map_oval(m_stat.rud, 
		m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min,
		0xff, 0x7f, 0x00);
	float rud_sta = 
		(float) map_oval(m_stat.rud_sta, 
		m_stat.rud_sta_max, m_stat.rud_sta_nut, m_stat.rud_sta_min,
		0xff, 0x7f, 0x00);

	float xorg =  (float)(0. - 255. * 0.5 * m_ixscale);
	float yorg = (float)(1.0 - 6 * hfont);
	float x1, x2, y1, y2, ytxt;

	// draw title
	drawGlText((float)(xorg + 0.5 * (wm - wfont * (double)strlen("RUDDER"))), 
		(float)(yorg + 0.5 * hfont), "RUDDER", 
		0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

	x1 = xorg;
	x2 = (float)(xorg + wm);
	y1 = yorg;
	y2 = (float)(yorg - hm);
	ytxt = (float) (yorg - 1.2 * hm - hfont);

	// indicator box
	drawGlSquare2Df(x1, y1, x2, y2, 0, 0, 0, 1);
	drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);

	x1 = (float)(xorg + 0.5 * wm);

	// rudder instruction value
	x2 = (float)(rud_inst * m_ixscale - 0.5 * wm);
	drawGlLine2Df(x2, y1, x2, y2, 0, 1, 0, 1, lw);

	// current rudder instruction value
	x2 = (float)(rud_inst_cur * m_ixscale - 0.5 * wm);
	y2 = (float)(yorg - 0.666 * hm);
	drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

	// current rudder angle
	x2 = (float)(rud_sta * m_ixscale - 0.5 * wm);
	y1 = y2;
	y2 = (float)(yorg - hm) ;
	drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

	// Midship
	float dwfont = (float)(- 0.5 * wfont);
	drawGlLine2Df(x1, y1, x1, y2, 0, 1, 0, 1, lw);
	drawGlText(x1 + dwfont, ytxt, "0", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	drawGlText(xorg + dwfont, ytxt, "P", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	drawGlText((float)(xorg + wm + dwfont), ytxt, "S", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

	if(m_verb){
		cout << "    Inst rud " << rud_inst;
		cout << "    Ctrl rud " << rud_inst_cur;
		cout << "    Rud stat " << rud_sta << endl;
	}
}

void f_aws1_ui::ui_show_meng()
{
	glRasterPos2i(-1, -1);
	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float wm = (float)(3.0 * wfont);
	float hm = (float)(255.0 * m_iyscale);
	float lw = (float)(1.0 / m_sz_win.width);

	float meng_inst = (float)m_inst.meng_aws;
	float meng_inst_cur = 
		(float)map_oval(m_stat.meng,
		m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, 
		m_stat.meng_nub, m_stat.meng_min,
		0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
	float x = (float)(wfont - 1.0);
	float y = (float)(1.0 - 6 * hfont);
	drawGlEngineIndicator("M/E", x, y, wm, hm, wfont, hfont, lw, 
		(float)(meng_inst * m_iyscale),
		(float)(meng_inst_cur * m_iyscale));
	if(m_verb){
		cout << "    Inst meng " << meng_inst ;
		cout << "    Ctrl meng " << meng_inst_cur << endl;
	}
}

void f_aws1_ui::ui_show_seng()
{
	glRasterPos2i(-1, -1);

	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float wm = (float)(3.0 * wfont);
	float hm = (float)(255.0 * m_iyscale);
	float lw = (float)(1.0 / m_sz_win.width);

	float seng_inst = (float)m_inst.seng_aws;
	float seng_inst_cur = 
		(float) map_oval(m_stat.seng,
		m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, 
		m_stat.seng_nub, m_stat.seng_min,
		0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
	float x = (float)(6 * wfont - 1.0);
	float y = (float)(1.0 - 6 * hfont);
	drawGlEngineIndicator("S/E", x, y, wm, hm, wfont, hfont, lw, 
		(float)(seng_inst * m_iyscale),
		(float)(seng_inst_cur * m_iyscale));
  if(m_verb){
	  cout << "    Inst seng " << seng_inst;
	  cout << "    Ctrl seng " << seng_inst_cur << endl;
  }
}

void f_aws1_ui::ui_show_state()
{
	glRasterPos2i(-1, -1);

	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float lw = (float)(1.0 / m_sz_win.width);
	float roll, pitch, yaw;
	float lat, lon, alt, galt;
	float cog, sog;
	float depth;
	roll = pitch = yaw = lat = lon = alt = galt = cog = sog = depth = 0.;
	if(m_state){
		long long t = 0;
		m_state->get_attitude(t, roll, pitch, yaw);
		m_state->get_position(t, lat, lon, alt, galt);
		m_state->get_velocity(t, cog, sog);
		m_state->get_depth(t, depth);
	}

	if(yaw < 0){
		yaw = (float)(yaw + 360.);
	}

	// Drawing ship state information
	float xorg = (float)(wfont - 1.0);
	float yorg = (float)(hfont - 1.0);
	// box and the informations
	char slat[32]; // "LAT     : XXX.XXXXXXXXdg"
	char slon[32]; // "LON     : XXX.XXXXXXXXdg"
	char salt[32]; // "ALT(GEO): XXX.Xm (XXX.Xm)"
	char scog[32]; // "COG     : XXX.Xdg"
	char ssog[32]; // "SOG     : XXX.Xkt"
	char syaw[32]; // "YAW     : XXX.Xdg"
	char spch[32]; // "PITCH   : XXX.Xdg"
	char srol[32]; // "ROLL    : XXX.Xdg"
	char sdpt[32]; // "DEPTH   : XXX.Xm"
	snprintf(slat, 32, "LAT     : %+013.8fdg", lat);
	snprintf(slon, 32, "LON     : %+013.8fdg", lon);
	snprintf(salt, 32, "ALT(GEO): %+06.1fm (%+06.1fm)", alt, galt);
	snprintf(syaw, 32, "YAW     : %+06.1fdg", yaw);
	snprintf(spch, 32, "PITCH   : %+06.1fdg", pitch);
	snprintf(srol, 32, "ROLL    : %+06.1fdg", roll);
	snprintf(scog, 32, "COG     : %+06.1fdg", cog);
	snprintf(ssog, 32, "SOG     : %+06.1fkt", sog);
	snprintf(sdpt, 32, "DEPTH   : %+06.1fm", depth);

	float w = (float)((strlen(salt) + 2) * wfont);
	float h = (float)(10 * hfont);
	drawGlSquare2Df(xorg, yorg, (float)(xorg + w), (float)(yorg + h), 0, 0, 0, 1);
	drawGlSquare2Df(xorg, yorg, (float)(xorg + w), (float)(yorg + h), 0, 1, 0, 1, lw);

	float x, y;
	x = (float)(xorg + wfont);
	y = (float)(yorg + 0.5 * hfont);

	drawGlText(x, y, slat, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, slon, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, salt, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, scog, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, ssog, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, syaw, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, spch, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, srol, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += hfont;
	drawGlText(x, y, sdpt, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

  if(m_verb){
	  cout << " RPY = " << roll << "," << pitch << "," << yaw << endl;
	  cout << " Pos = " << lat << "," << lon << "," << alt << endl;
	  cout << " Vel = " << cog << "," << sog << endl;
	  cout << " Depth = " << depth << endl;
  }
}

void f_aws1_ui::ui_show_attitude()
{
	glRasterPos2i(-1, -1);

	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float lw = (float)(1.0 / m_sz_win.width);
	float roll, pitch, yaw;
	float lat, lon, alt, galt;
	float cog, sog;
	float depth;

	roll = pitch = yaw = lat = lon = alt = galt = cog = sog = depth = 0.;
	if(m_state){
		long long t = 0;
		m_state->get_attitude(t, roll, pitch, yaw);
		m_state->get_position(t, lat, lon, alt, galt);
		m_state->get_velocity(t, cog, sog);
		m_state->get_depth(t, depth);
	}
}

void f_aws1_ui::ui_show_sys_state()
{
	glRasterPos2i(-1, -1);

	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float xorg = (float)(1.0 - wfont);
	float yorg = (float)(1.0 - hfont);
	float lw = (float)(1.0 / m_sz_win.width);

	char str[32]; // "XXXX: xxxxxxx"	
	float w = (float)((15 + 2) * wfont);
	float h = (float)(4 * hfont);
	float x = (float)(xorg - w);
	drawGlSquare2Df(xorg, yorg, x, (float)(yorg - h), 0, 1, 0, 1, lw);

	snprintf(str, 32, "CTRL: %8s", str_aws1_ctrl_src[m_stat.ctrl_src]);
	x += wfont;
	float y = (float)(yorg - 1.5 * hfont);
	drawGlText(x, y, str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

	if (m_ch_ap_inst){
		snprintf(str, 32, "APMD: %8s", str_aws1_ap_mode[m_ch_ap_inst->get_mode()]);
		y -= hfont;
		drawGlText(x, y, str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	}

	snprintf(str, 32, "MODE: %8s", m_str_aws1_ui_mode[m_mode]);
	y -= hfont;
	drawGlText(x, y, str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
}

void f_aws1_ui::ui_show_menu()
{
	glRasterPos2i(-1, -1);
	//-----------------------//
	//     System MENU       //
	//-----------------------//
	//  <Control> | <Value>  //
	//  <AP mode> | <value>  //
	//  <IM view> | <value>  //
	//  <UI     > | <Value>  //
	//  <Exit   > | <Value>  //
	//-----------------------//
	//<Apply(a)>  <Cancel(b)>//
	///////////////////////////
	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	const char * title = "System Menu"; /* 11 characters */
	const int num_items = 5;
	const char * items[num_items] = { /* 7 characters the longest.*/
		"Control", "AP mode", "Cam", "UI", "Quit"
	};

	float alpha_bg = 0.5;
	float alpha_txt = 1.0;
	
	float wcol_l = 12 * wfont;
	float wcol_r = 12 * wfont;
	float wm = wcol_l + wcol_r + 2 * wfont;
	float hm = (float)(((float)num_items + 3.5) * hfont); 
	float lw = (float)(1.0 / m_sz_win.width);

	float xorg = (float)(- 0.5 * wm);
	float yorg = (float)(- 0.5 * hm);

	// draw menu background
	drawGlSquare2Df(xorg, yorg, (float)(xorg + wm), (float)(yorg + hm), 0, 0, 0, alpha_bg);

	// draw frame 
	drawGlSquare2Df(xorg, yorg, (float)(xorg + wm), (float)(yorg + hm), 0, 1, 0, alpha_bg, lw);

	// draw bars
	float x = (float)(xorg + wm);
	float y = (float)(yorg + 1.5 * hfont);
	drawGlLine2Df(xorg, y, x, y, 0, 1, 0, alpha_bg, lw); 
	y = (float)(yorg + hm - 1.5 * hfont);
	drawGlLine2Df(xorg, y, x, y, 0, 1, 0, alpha_bg, lw);
	x = 0.;
	drawGlLine2Df(x, y, x, (float)(yorg + 1.5 * hfont), 0, 1, 0, alpha_bg, lw);

	// Draw Text
	x = (float) (xorg + 0.5 * wfont);
	float xv = wfont;
	y = (float)(yorg + hm - hfont);

	drawGlText(x, y, title, 0, 1, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	int ifocus = 0;
	// Control
	y -= (float)(1.5 * hfont);
	if(m_menu_focus == ifocus)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	float g = m_menu_focus == ifocus ? 0.f : 1.f;
	drawGlText(x, y, items[ifocus], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, str_aws1_ctrl_src[m_menu_acs], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// AP mode
	ifocus++;
	y -= (float)(hfont);
	if (m_menu_focus == ifocus)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
		0, 1, 0, alpha_bg);
	g = m_menu_focus == ifocus ? 0.f : 1.f;
	drawGlText(x, y, items[ifocus], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	if (m_ch_ap_inst)
		drawGlText(xv, y, str_aws1_ap_mode[m_menu_ap_mode], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	else
		drawGlText(xv, y, str_aws1_ap_mode[EAP_WP], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// Cam
	ifocus++;
	y -= (float)(hfont);
	if(m_menu_focus == ifocus)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	g = m_menu_focus == ifocus ? 0.f : 1.f;
	drawGlText(x, y, items[ifocus], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, m_str_imv[m_imv], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// UI
	ifocus++;
	y -= (float)(hfont);
	if(m_menu_focus == ifocus)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	g = m_menu_focus == ifocus ? 0.f : 1.f;
	drawGlText(x, y, items[ifocus], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, m_str_aws1_ui_mode[m_menu_mode], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// Exit
	ifocus++;
	y -= (float)(hfont);
	if(m_menu_focus == ifocus)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	g = m_menu_focus == ifocus ? 0.f : 1.f;
	drawGlText(x, y, items[ifocus], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, m_quit ? "yes":"no", 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// OK/Cancel
	y -= (float)(1.5 * hfont);
	drawGlText(x, y,  "(a) Apply", 0, 1, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, "(b) Cancel", 0, 1, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
}

void f_aws1_ui::ui_handle_menu()
{
	if((m_js.estart & s_jc_u3613m::EB_EVDOWN) && (m_ui_menu == false)){
		m_ui_menu = true;
		m_quit = false;
		m_menu_acs = m_stat.ctrl_src;
		m_menu_mode = m_mode;
		if (m_ch_ap_inst){
			m_menu_ap_mode = m_ch_ap_inst->get_mode();
		}
		else{
			m_menu_ap_mode = EAP_WP;
		}


		return;
	}

	if(!m_ui_menu)
		return;

	if((m_js.estart & s_jc_u3613m::EB_EVDOWN) || (m_js.eb & s_jc_u3613m::EB_EVDOWN))
	{// cancel 
		m_ui_menu = false;
	}

	if(m_js.ea & s_jc_u3613m::EB_EVDOWN)
	{
		if(m_quit){
			m_bactive = false;
		}
		m_inst.ctrl_src = m_menu_acs;
		m_mode = m_menu_mode;

		if (m_ch_ap_inst){
			e_ap_mode ap_mode = m_ch_ap_inst->get_mode();
			if (ap_mode != m_menu_ap_mode){
				m_ch_ap_inst->set_mode(m_menu_ap_mode);
				long long t;
				float lat, lon, alt, galt;
				m_state->get_position(t, lat, lon, alt, galt);
				m_ch_ap_inst->set_stay_pos((float)(lat * (PI / 180)), 
					(float)(lon * (PI / 180)));
			}
		}
		m_ui_menu = false;
	}

	if(m_js.edx & s_jc_u3613m::EB_EVDOWN || m_js.tdx > 60){
		m_js.tdx = 0;
		m_menu_focus = (m_menu_focus + 1 ) % 5;
	}

	if(m_js.eux & s_jc_u3613m::EB_EVDOWN || m_js.tux > 60){
		m_js.tux = 0;
		m_menu_focus = (m_menu_focus + 4 ) % 5;
	}

	if(m_js.elx & s_jc_u3613m::EB_EVDOWN || m_js.tlx > 60){
		m_js.tlx = 0;
		switch(m_menu_focus){
		case 0:
			m_menu_acs = (e_aws1_ctrl_src) ((m_menu_acs + ACS_NONE - 1) % ACS_NONE);
			break;
		case 1:
			m_menu_ap_mode = (e_ap_mode)((m_menu_ap_mode + EAP_NONE - 1) % EAP_NONE);
			break;
		case 2:
			m_imv = (e_imv) ((m_imv + IMV_UNDEF - 1) % IMV_UNDEF);
			break;
		case 3:
			m_menu_mode = (e_aws1_ui_mode) ((m_menu_mode + AUM_UNDEF - 1) % AUM_UNDEF);
			break;
		case 4:
			m_quit = !m_quit;
			break;
		}
	}

	if(m_js.erx & s_jc_u3613m::EB_EVDOWN || m_js.trx > 60){
		m_js.trx = 0;
		switch(m_menu_focus){
		case 0:
			m_menu_acs = (e_aws1_ctrl_src) ((m_menu_acs + 1) % ACS_NONE);
			break;
		case 1:
			m_menu_ap_mode = (e_ap_mode)((m_menu_ap_mode + 1) % EAP_NONE);
			break;
		case 2:
			m_imv = (e_imv)((m_imv + 1) % IMV_UNDEF);
			break;
		case 3:
			m_menu_mode = (e_aws1_ui_mode) ((m_menu_mode + 1) % AUM_UNDEF);
			break;
		case 4:
			m_quit = !m_quit;
			break;
		}
	}
}

void f_aws1_ui::write_screen()
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
	owp.disable();

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
	owp.set_focus(iwp);
	m_ch_wp->unlock();
}


bool f_aws1_ui::proc()
{
	// process joypad inputs
	if(m_js.id != -1){
		m_js.set_btn();
		m_js.set_stk();
	}

	ui_force_ctrl_stop();
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

	glUseProgram(p);
	glUniform2fv(loc_inv_sz_half_scrn, 1, inv_sz_half_scrn);

	mouse_button = -1;
	mouse_action = -1;
	mouse_mods = -1;

	// loading states
	long long t;
	float roll, pitch, yaw, cog, sog;
	m_state->get_attitude(t, roll, pitch, yaw);
	m_state->get_velocity(t, cog, sog);

	float xown, yown, zown;
	m_state->get_position_ecef(t, xown, yown, zown);

	float lat, lon, alt, galt;
	Mat Rown = m_state->get_enu_rotation(t);
	m_state->get_position(t, lat, lon, alt, galt);

	float depth;
	m_state->get_depth(t, depth);

	// handling ui events
	bool event_handled = uim.set_mouse_event(pt_mouse, mouse_button, mouse_action, mouse_mods);

	c_view_mode_box * pvm_box =
		dynamic_cast<c_view_mode_box*>(uim.get_ui_box(c_aws_ui_box_manager::view_mode));
	c_ctrl_mode_box * pcm_box = 
		dynamic_cast<c_ctrl_mode_box*>(uim.get_ui_box(c_aws_ui_box_manager::ctrl_mode));
	c_obj_cfg_box * poc_box = 
		dynamic_cast<c_obj_cfg_box*>(uim.get_ui_box(c_aws_ui_box_manager::obj_cfg));
	c_route_cfg_box * prc_box =
		dynamic_cast<c_route_cfg_box*>(uim.get_ui_box(c_aws_ui_box_manager::route_cfg));

	// calculating mouse point coordinate
	calc_mouse_enu_and_ecef_pos(pvm_box->get_mode(), Rown, lat, lon, xown, yown, zown, yaw);

	if (!event_handled){ // handling an event is not handled with ui
		if (mouse_button == GLFW_MOUSE_BUTTON_LEFT){
			if (mouse_action == GLFW_PRESS){
				handle_mouse_lbtn_push(pvm_box, pcm_box, poc_box, prc_box);
			}
			else if (mouse_action == GLFW_RELEASE){
				handle_mouse_lbtn_up(pvm_box, pcm_box, poc_box, prc_box);
			}
		}
		else{
			handle_mouse_mv(pvm_box, pcm_box, poc_box, prc_box);
		}
	}
	else{ //handling ui event
		e_mouse_state mouse_state_new = ms_normal;
		switch (uim.get_box_updated()){
		case c_aws_ui_box_manager::view_mode:
			update_view_mode_box(pvm_box);
			break;
		case c_aws_ui_box_manager::ctrl_mode:
			update_ctrl_mode_box(pcm_box);
			break;
		case c_aws_ui_box_manager::obj_cfg:
			update_obj_cfg_box(poc_box);
			break;
		case c_aws_ui_box_manager::route_cfg:
			update_route_cfg_box(prc_box, mouse_state_new);
			break;
		}	
	}

	// updating indicator
	ind.set_param(m_stat.meng, m_stat.seng, m_stat.rud, (float)(cog * (PI / 180.f)), sog,
		(float)(yaw * (PI / 180.f)), (float)(pitch* (PI / 180.f)), (float)(roll* (PI / 180.f)));
	ind.set_dir_cam(dir_cam_hdg);

	// update route object
	update_route();

	// rendering graphics
	render_gl_objs();

	// show rendering surface.
	glfwSwapBuffers(pwin());

	// screen shot or video capturue
	write_screen();

	// UI polling.
	glfwPollEvents();

	return true;
}

void drawGlEngineIndicator(const char * title,
			   float xorg, float yorg, float w, float h,
			   float wflont, float hfont, 
			   float lw, float val_inst, float val_cur)
{
  float x1, y1, x2, y2, xtxt;
  x1 = xorg;
  y1 = yorg;
  x2 = (float) (x1 + w);
  y2 = (float) (y1 - h);
  xtxt = (float)(x2 + 0.1 * w);  

  // draw title
  drawGlText(xorg, (float)(yorg + 0.5 * hfont), title, 
	     0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

  // indicator box
  drawGlSquare2Df(x1, y1, x2, y2, 0, 0, 0, 1);
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);

  // current value
  x2 = (float)(x1 + 0.666 * w);
  y1 = (float)(yorg + val_cur - h);
  y2 = (float)(yorg - 0.5 * h);
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // current instructed value
  y1 = (float) (yorg - h + val_inst);
  drawGlLine2Df(x1, y1, x2, y1, 0, 1, 0, 1, lw);

  // neutral
  float dhfont = (float)(-0.5 * hfont);
  x1 = xorg;
  x2 = (float)(xorg + w);
  y1 = y2 = (float) (yorg - 0.5 * h);
  drawGlLine2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y1 + dhfont), 
	     "N", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

  // neutral forward
  y2 = (float)(y1 + (25. / 255.) * h);
  drawGlLine2Df(x1, y2, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y2), 
	     "F", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

  // neutral backward
  y2 = (float)(y1 - (25. / 255.) * h);
  drawGlLine2Df(x1, y2, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y2 - hfont), 
	     "B", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
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


void f_aws1_ui::calc_mouse_enu_and_ecef_pos(
	c_view_mode_box::e_btn vm, Mat & Rown,
	const float lat, const float lon,
	const float xown, const float yown, const float zown, const float yaw)
{
	if (vm == c_view_mode_box::map)
	{
		if (!bmap_center_free){
			pt_map_center_enu.x = pt_map_center_enu.y = 0;
			pt_map_center_ecef.x = xown;
			pt_map_center_ecef.y = yown;
			pt_map_center_ecef.z = zown;
			pt_map_center_bih.x = lat;
			pt_map_center_bih.y = lon;
			Rmap = Rown;
		}

		pt_mouse_enu.x = pt_mouse.x * meter_per_pix;
		pt_mouse_enu.y = pt_mouse.y * meter_per_pix;
	}
	else if (vm == c_view_mode_box::fpv)
	{
		pt_map_center_enu.x = pt_map_center_enu.y = 0;
		pt_map_center_ecef.x = xown;
		pt_map_center_ecef.y = yown;
		pt_map_center_ecef.z = zown;
		pt_map_center_bih.x = lat;
		pt_map_center_bih.y = lon;
		Rmap = Rown;

		float x, y;
		if (pt_mouse.y > 0){ // pointing to sky
			x = zhorizon * pt_mouse.x * ifcam;
			y = dhorizon_arc;
		}
		else{
			float th_cam_surface = (float)atan(-pt_mouse.y * ifcam);
			float th_surface_arc = (float)(th_cam_surface - acos(cos(th_cam_surface) * height_cam_ec * iRE));
			float dsurface_arc = (float)(RE * th_surface_arc);
			float zsurface = (float)(tan(th_surface_arc) * height_cam_ec);
			x = zsurface * pt_mouse.x / fcam;
			y = dsurface_arc;
		}
		float c, s, th = (PI / 180.0) * (yaw + dir_cam_hdg);
		c = (float)cos(th);
		s = (float)sin(th);

		pt_mouse_enu.x = (float)(c * x + s * y);
		pt_mouse_enu.y = (float)(-s * x + c * y);
	}

	float alt = 0;
	wrldtoecef(Rmap, pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z,
		pt_mouse_enu.x, pt_mouse_enu.y, 0.f,
		pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z);
	eceftobih(pt_mouse_ecef.x, pt_mouse_ecef.y, pt_mouse_ecef.z,
		pt_mouse_bih.x, pt_mouse_bih.y, alt);
}


void f_aws1_ui::add_waypoint(c_route_cfg_box * prc_box)
{
	unsigned int rt, spd, wp;
	prc_box->get_params(wp, spd, rt);
	// set current cursor position as the new waypoint
	m_ch_wp->lock();
	m_ch_wp->ins(pt_mouse_bih.x, pt_mouse_bih.y, 10.0, spd);
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
	if (bmap_center_free){
		bmap_center_free = true;
	}
	// change map center
	pt_map_center_bih = pt_mouse_bih;
	bihtoecef(pt_mouse_bih.x, pt_mouse_bih.y, 0.0f,
		pt_map_center_ecef.x, pt_map_center_ecef.y, pt_map_center_ecef.z);
	pt_map_center_enu.x = pt_map_center_enu.y = 0;
	getwrldrotf(pt_map_center_ecef.x, pt_map_center_ecef.y, Rmap);
}

void f_aws1_ui::drag_cam_dir()
{
	float th0 = (float) atan2(pt_mouse_enu.y, pt_mouse_enu.x);
	float th1 = (float) atan2(pt_mouse_drag_begin_enu.y, pt_mouse_drag_begin_enu.x);
	dir_cam_hdg = (float)((th1 - th0) * 180.0f / PI);
}

void f_aws1_ui::handle_mouse_lbtn_push(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box)
{
	det_obj_collision();
	switch (mouse_state){
	case ms_normal:
		pt_mouse_drag_begin = pt_mouse;
		pt_mouse_drag_begin_enu = pt_mouse_enu;
		mouse_state = ms_drag;
		break;
	}
}

void f_aws1_ui::handle_mouse_lbtn_up(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box)
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
		mouse_state = ms_normal;
		break;
	case ms_normal:
		// find object selected
		break;
	}
}

void f_aws1_ui::handle_mouse_mv(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box)
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
		if (pvm_box->get_mode() == c_view_mode_box::fpv){
			// change dir_cam_hdg
			drag_cam_dir();
		}
		else if (pvm_box->get_mode() == c_view_mode_box::map){
			drag_map();
		}
	}
}

void f_aws1_ui::update_view_mode_box(c_view_mode_box * pvm_box)
{
	ind.set_mode((c_indicator::e_ind_mode)pvm_box->get_mode());
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

void f_aws1_ui::update_obj_cfg_box(c_obj_cfg_box * poc_box)
{
	poc_box->get_params(map_range, visible_obj);
	switch (poc_box->get_command())
	{
	case c_obj_cfg_box::range_up:
		map_range *= 10.f;
		recalc_range();
		break;
	case c_obj_cfg_box::range_down:
		map_range *= 0.1f;
		recalc_range();
		break;
	}
	poc_box->set_params(map_range, visible_obj);
}

void f_aws1_ui::update_route_cfg_box(c_route_cfg_box * prc_box, e_mouse_state mouse_state_new)
{
	m_ch_wp->lock();
	s_wp & fwp = m_ch_wp->get_focused_wp();
	unsigned int rt, spd, wp;
	prc_box->get_params(wp, spd, rt);
	switch (prc_box->get_command())
	{
	case c_route_cfg_box::wp_prev:
		m_ch_wp->prev_focus();
		owp.set_focus(m_ch_wp->get_focus());
		break;
	case c_route_cfg_box::wp_next:
		m_ch_wp->next_focus();
		owp.set_focus(m_ch_wp->get_focus());
		break;
	case c_route_cfg_box::wp_spd_down:
		fwp.v += 1.0;
		fwp.v = min(fwp.v, 40.0f);
		break;
	case c_route_cfg_box::wp_spd_up:
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
	fwp = m_ch_wp->get_focused_wp();
	spd = (unsigned int)fwp.v;
	prc_box->set_params(wp, spd, rt);
	prc_box->reset_command();
	m_ch_wp->unlock();

}

void f_aws1_ui::_cursor_position_callback(double xpos, double ypos){
	pt_mouse.x = (float)(xpos - (double)(m_sz_win.width >> 1));
	pt_mouse.y = (float)((double)(m_sz_win.height >> 1) - ypos);
	m_mx = xpos;
	m_my = ypos;
}

void f_aws1_ui::_mouse_button_callback(int button, int action, int mods)
{
	mouse_button = button;
	mouse_action = action;
	mouse_mods = mods;
}

void f_aws1_ui::_key_callback(int key, int scancode, int action, int mods)
{
	m_ui[m_mode]->key(key, scancode, action, mods);
}

/////////////////////////////////////////////////////////////////////////// f_aws1_ui_test members

f_aws1_ui_test::f_aws1_ui_test(const char * name):f_base(name),
	m_state(NULL), m_ch_ais_obj(NULL),
	m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL), m_ch_ctrl_stat(NULL),
	m_ahrs(false), m_gps(false), r(0), p(0), y(0), lon(0), lat(0), alt(0), galt(0), cog(0), sog(0), depth(0),
	m_rud_sta_sim(0.f),
	m_add_ais_ship(false), ais_mmsi(0), ais_lat(0), ais_lon(0), ais_cog(0), ais_sog(0), ais_yaw(0)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ch_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel");
  register_fpar("ch_ctrl_ui", (ch_base**)&m_ch_ctrl_ui, typeid(ch_aws1_ctrl_inst).name(), "Control input channel.");
  register_fpar("ch_ctrl_ap1", (ch_base**)&m_ch_ctrl_ap1, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 1 control input channel.");
  register_fpar("ch_ctrl_ap2", (ch_base**)&m_ch_ctrl_ap2, typeid(ch_aws1_ctrl_inst).name(), "Autopilot 2 control input channel.");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");
  register_fpar("ch_img", (ch_base**)&m_ch_img, typeid(ch_image_ref).name(), "Image channel");	

  // for m_state
  register_fpar("ahrs", &m_ahrs, "Yes if AHRS is on the test."); 
  register_fpar("gps", &m_gps, "Yes if GPS is on the test.");
  register_fpar("roll", &r, "Roll(deg)");
  register_fpar("pitch", &p, "Pitch(deg)");
  register_fpar("yaw", &y, "Yaw(deg)");
  register_fpar("lon", &lon, "Longitude(deg)");
  register_fpar("lat", &lat, "Latitude(deg)");
  register_fpar("alt", &alt, "Altitude(m)");
  register_fpar("galt", &galt, "Geoid height(m)");
  register_fpar("cog", &cog, "Course over ground(deg)");
  register_fpar("sog", &sog, "Speed over ground(kts)");
  register_fpar("depth", &depth, "Depth of the water(m).");

  // for ch_ctrl
  // aws's control parameters
  register_fpar("awsrud", &m_stat.rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_stat.meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_stat.seng_aws, "Control value of AWS1's sub engine.");

  // remote controller's control parameters (Read Only)
  register_fpar("rmcrud", &m_stat.rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_stat.meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_stat.seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_stat.rud_sta, "Rudder Status of AWS1's.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_stat.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_stat.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_stat.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_stat.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_stat.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_stat.meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_stat.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_stat.meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_stat.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_stat.meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_stat.seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_stat.seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_stat.seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_stat.seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_stat.seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  // Each control points of the sub engine output
  register_fpar("seng_max", &m_stat.seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_stat.seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_stat.seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_stat.seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_stat.seng_min, "Minimum control value for AWS1's sub engine.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_stat.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_stat.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_stat.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_stat.rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_stat.rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_stat.rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_stat.rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_stat.rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_stat.rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  // Control points as the rudder indicator output.
  register_fpar("rud_sta_out_max", &m_stat.rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_stat.rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_stat.rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

  register_fpar("meng", &m_stat.meng, "Output value for main engine.");
  register_fpar("seng", &m_stat.meng, "Output value for sub engine.");
  register_fpar("rud", &m_stat.rud, "Output value for rudder.");
  register_fpar("rud_sta_out", &m_stat.rud_sta_out, "Output value for rudder status.");

  // for AIS data injection
  register_fpar("add_ais", &m_add_ais_ship, "If yew, new ais ship is added according to the parameter.");
  register_fpar("ais_mmsi", &ais_mmsi, "MMSI of the AIS ship to be added.");
  register_fpar("ais_lat", &ais_lat, "lattitude of the AIS ship to be added.");
  register_fpar("ais_lon", &ais_lon, "longitude of the AIS ship to be added.");
  register_fpar("ais_cog", &ais_cog, "COG of the AIS ship to be added.");	
  register_fpar("ais_sog", &ais_sog, "SOG of the AIS ship to be added.");	
  register_fpar("ais_yaw", &ais_yaw, "YAW of the AIS ship to be added.");	
}

bool f_aws1_ui_test::init_run()
{
	return true;
}

void f_aws1_ui_test::destroy_run()
{

}

bool f_aws1_ui_test::proc()
{
	select_control_input();

	simulate_rudder();

	set_control_output();

	set_state();

	simulate_dynamics();

	add_ais_ship();

	return true;
}


/////////////////////////////////////////////////////////////////////////////// ui controls

///////////////////////////////////////////////////////////////////////// c_aws_ui_box
c_gl_2d_obj * c_aws_ui_box::porect = NULL;
c_gl_2d_obj * c_aws_ui_box::potri = NULL;
c_gl_2d_line_obj * c_aws_ui_box::poline = NULL;
c_gl_text_obj * c_aws_ui_box::potxt = NULL;

void c_aws_ui_box::set_gl_objs(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline)
{
	porect = _porect;
	potri = _potri;
	potxt = _potxt;
	poline = _poline;
}

void c_aws_ui_box::add_btn(int & hbtn, int & hstr, const char * str, 
	const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_fnt)
{
	hbtn = porect->add(clr, pos, 0, sz_btn);
	porect->config_border(hbtn, true, 1.0);
	porect->config_depth(hbtn, 1.0);

	hstr = potxt->reserv(strlen(str) + 1);
	potxt->set(hstr, str);
	glm::vec2 pos_txt((float)(pos.x + sz_btn.x * 0.5), (float)(pos.y + sz_btn.y * 0.5));
	potxt->config(hstr, clr, bkgclr, sz_fnt, sz_fnt, c_gl_text_obj::an_cc, pos_txt, 0.0);
	potxt->config_depth(hstr, 0);

	set_normal_color(hbtn, hstr);
}

void c_aws_ui_box::add_select_box(int & hlbtn, int & hlstr, const char * lstr, 
	int & hrbtn, int & hrstr, const char * rstr, int & hvalstr,
	const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_box, 
	const glm::vec2 & sz_fnt, const unsigned int len_str)
{
	glm::vec2 pos_btn;
	pos_btn.x = pos.x;
	pos_btn.y = pos.y;
	add_btn(hlbtn, hlstr, lstr, pos_btn, sz_btn, sz_fnt);

	pos_btn.x += (float)(sz_box.x * 0.5);
	pos_btn.y += (float)(sz_fnt.y * 0.5);
	hvalstr = potxt->reserv(len_str);
	potxt->config(hvalstr, clr, bkgclr, sz_fnt, sz_fnt, c_gl_text_obj::an_cc, pos, 0.0);
	
	pos_btn.x = (float)(pos.x + sz_box.x - sz_btn.x);
	pos_btn.y = pos.y;
	add_btn(hrbtn, hrstr, rstr, pos_btn, sz_btn, sz_fnt);
}


void c_aws_ui_box::setup_frame(const float y, const bool left, 
	const glm::vec2 & sz_scrn, const glm::vec2 &sz_box, 
	const glm::vec2  & sz_fnt, const glm::vec4 & clr)
{
	float xmax = (float)(0.5 * sz_scrn.x), xmin = -xmax,
		ymax = (float)(0.5 * sz_scrn.y), ymin = -ymax;

	sz_close.x = sz_fnt.x;
	sz_close.y = sz_box.y;
	sz_open.x = (float)(sz_box.x + sz_close.x);
	sz_open.y = sz_box.y;
	pos_open.x = (left ? xmin : xmax - sz_open.x);
	pos_open.y = y;
	pos_close.x = (left ? xmin : xmax - sz_close.x);
	pos_close.y = y;

	glm::vec2 pos;
	pos.y = (float)(y + sz_box.y * 0.5);
	pos.x = (left ? xmin + sz_box.x + 0.5 * sz_fnt.x : xmax - sz_box.x - 0.5 * sz_fnt.x);
	hopen = potri->add(clr, pos, (left ? PI : 0.0f), sz_fnt.x * 0.5);
	pos.x = (left ? xmin + 0.5 * sz_fnt.x : xmax - 0.5 * sz_fnt.x);
	hclose = potri->add(clr, pos, (left ? 0.0f : PI), sz_fnt.x * 0.5);

	// appearance setting
	porect->config_border(hbox, true, 1.0);
	porect->config_color(hbox, clr);
	porect->config_depth(hbox, 2);

	potri->config_border(hopen, false, 1.0);
	potri->config_color(hopen, clr);
	potri->config_depth(hopen, 1);

	potri->config_border(hclose, false, 1.0);
	potri->config_color(hclose, clr);
	potri->config_depth(hclose, 1);
}

bool c_aws_ui_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		if (potri->collision(pt, hopen)){
			btn_oc_pushed = true;
		}
	}
	else{
		if (potri->collision(pt, hclose)){
			btn_oc_pushed = true;
		}
	}
	return btn_oc_pushed;
}

bool c_aws_ui_box::handle_left_release(const glm::vec2 & pt)
{
	pt_mouse = pt;
	if (bopened){
		if (potri->collision(pt, hopen)){
			btn_oc_released = true;
		}
	}
	else{
		if (potri->collision(pt, hclose)){
			btn_oc_released = true;
		}
	}

	return btn_oc_released;
}

void c_aws_ui_box::open()
{
	potri->enable(hopen);
	potri->disable(hclose);
	porect->config_position(hbox, pos_open);
	porect->config_scale(hbox, sz_open);	
}

void c_aws_ui_box::close()
{
	potri->enable(hclose);
	potri->disable(hopen);
	porect->config_position(hbox, pos_close);
	porect->config_scale(hbox, sz_close);
}

bool c_aws_ui_box::proc(const bool bpushed, const bool breleased)
{
	if (bpushed){
		if (btn_oc_pushed){
			if (breleased){
				if (btn_oc_released){
					bopened = !bopened;
					if (bopened){
						open();
					}
					else{
						close();
					}
				}
				set_normal_color(hbox);
				btn_oc_pushed = btn_oc_released = false;
				return true;
			}
			else{
				set_selected_color(hbox);
				return true;
			}
		}
	}
	else{
		if (breleased && btn_oc_released){
			bopened = !bopened;
			if (bopened){
				open();
			}
			else{
				close();
			}

			set_normal_color(hbox);
			btn_oc_pushed = btn_oc_released = false;
		}
	}
	return false;
}

void c_aws_ui_box::set_selected_color(const int hrect)
{
	glm::vec4 box_clr(clr.x * 0.5, clr.y * 0.5, clr.z * 0.5, clr.w * 0.5);
	porect->config_color(hrect, box_clr);
	porect->config_border(hrect, false, 1.0);
}

void c_aws_ui_box::set_normal_color(const int hrect)
{
	glm::vec4 box_clr = clr;
	porect->config_color(hrect, box_clr);
	porect->config_border(hrect, true, 1.0);
}

void c_aws_ui_box::set_checked_color(const int hbtn, const int hstr)
{
	glm::vec4 box_clr = clr;
	glm::vec4 txt_clr(0, 0, 0, 1);

	porect->config_color(hbtn, box_clr);
	porect->config_border(hbtn, false, 1.0);
	potxt->config_color(hstr, txt_clr, bkgclr);
}

void c_aws_ui_box::set_normal_color(const int hbtn, const int hstr)
{
	glm::vec4 box_clr = clr;
	porect->config_color(hbtn, box_clr);
	porect->config_border(hbtn, true, 1.0);
	potxt->config_color(hstr, clr, bkgclr);
}

bool c_aws_ui_box::set_mouse_event(const glm::vec2 & pt, const int button, const int action, const int modifier)
{
	switch (button){
	case GLFW_MOUSE_BUTTON_LEFT:
		switch (action){
		case GLFW_PRESS:
			if (handle_left_push(pt))
				return true;
			break;
		case GLFW_RELEASE:
			if (handle_left_release(pt))
				return true;
			break;
		}
		break;
	}

	return false;
};

////////////////////////////////////////////////////////////////////////// c_obj_cfg_box
const char * c_view_mode_box::str_btn[nul] =
{
	"FPV", "MAP", "SYS"
};

bool c_view_mode_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = -xmax,
		ymax = (float)(0.5 * sz_scrn.y), ymin = -ymax;

	glm::vec2 pos, sz_btn, sz_box;
	pos.x = -sz_scrn.x * 0.5;
	pos.y = y;
	sz_btn.x = 4 * sz_fnt.x;
	sz_btn.y = 1.5 * sz_fnt.y;
	sz_box.x = sz_btn.x;
	sz_box.y = sz_btn.y * (float)nul;

	pos.x = xmin;
	pos.y = y;
	for (int ibtn = 0; ibtn < nul; ibtn++){
		add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
		pos.y += sz_btn.y;
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}

void c_view_mode_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (mode == ibtn){
			set_checked_color(hbtn[ibtn], hstr[ibtn]);
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
}

void c_view_mode_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
}

bool c_view_mode_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}
	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_view_mode_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_view_mode_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){

		if (breleased){
			if (btn_released == btn_pushed){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_pushed;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			else{
				if (btn_pushed == mode)
					set_checked_color(hbtn[mode], hstr[mode]);
				else
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
			}

			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_released;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			btn_released = nul;
			return true;
		}
	}


	return false;
}

/////////////////////////////////////////////////////////////////// c_ctrl_mode_box
const char * c_ctrl_mode_box::str_btn[nul] =
{
	"CRZ", "CTL", "CSR", "FWP", "STY", "FTG"
};

bool c_ctrl_mode_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = -xmax,
		ymax = (float)(0.5 * sz_scrn.y), ymin = -ymax;

	glm::vec2 pos, sz_btn, sz_box;
	pos.x = -sz_scrn.x * 0.5;
	pos.y = y;
	sz_btn.x = 4 * sz_fnt.x;
	sz_btn.y = 1.5 * sz_fnt.y;
	sz_box.x = sz_btn.x;
	sz_box.y = sz_btn.y * (float)nul;

	pos.x = xmin;
	pos.y = y;

	for (int ibtn = 0; ibtn < nul; ibtn++){
		add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
		pos.y += sz_btn.y;
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_ctrl_mode_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (mode == ibtn){
			set_checked_color(hbtn[ibtn], hstr[ibtn]);
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
}

void c_ctrl_mode_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
}

bool c_ctrl_mode_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_ctrl_mode_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_ctrl_mode_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			if (btn_released == btn_pushed){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_pushed;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			else{
				if (btn_pushed == mode)
					set_checked_color(hbtn[mode], hstr[mode]);
				else
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);

			}
			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_released;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			btn_released = nul;
			return true;
		}
	}


	return false;
}


//////////////////////////////////////////////////////////////////// c_obj_cfg_box
const char * c_obj_cfg_box::str_btn[nul] = 
{
	"WP", "VSL", "MRK", "CL", "-", "+"
};

bool c_obj_cfg_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = -xmax, 
		ymax = (float)(0.5 * sz_scrn.y), ymin = -ymax;

	glm::vec2 sz_btn, sz_box, sz_udbtn;
	int rows = nul / 3 + (nul % 3 ? 1 : 0);
	sz_btn.x = 4 * sz_fnt.x;
	sz_btn.y = 1.5 * sz_fnt.y;
	sz_udbtn.x = 2 * sz_fnt.x;
	sz_udbtn.y = 1.5 * sz_fnt.y;

	sz_box.x = (float)(sz_btn.x * 3);
	sz_box.y = (float)(sz_btn.y * (rows + 1));

	glm::vec2 pos;
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y;

	add_select_box(hbtn[range_down], hstr[range_down], str_btn[range_down],
		hbtn[range_up], hstr[range_up], str_btn[range_up], hstr_range, pos, sz_udbtn, sz_box, sz_fnt, 7);

	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = (float)(y + sz_udbtn.y);
	{
		int ibtn = 0;
		while (ibtn < range_down)
		{
			pos.x = (float)(left ? xmin : xmax - sz_box.x);
			for (int i = 0; i < 3 && ibtn < range_down; i++){
				add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
				ibtn++;
				pos.x += sz_btn.x;

			}
			pos.y += sz_btn.y;
		}
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_obj_cfg_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (ibtn < range_down){
			if (check[ibtn]){
				set_checked_color(hbtn[ibtn], hstr[ibtn]);
			}
			else{
				set_normal_color(hbtn[ibtn], hstr[ibtn]);
			}
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
	potxt->enable(hstr_range);
}

void c_obj_cfg_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
	potxt->disable(hstr_range);
}


bool c_obj_cfg_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_obj_cfg_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_obj_cfg_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			if (btn_released == btn_pushed){
				command = btn_pushed;
				if (btn_pushed < range_down){
					check[btn_pushed] = !check[btn_pushed];
				}
			}

			if (btn_pushed < range_down){
				if (check[btn_pushed]){
					set_checked_color(hbtn[btn_pushed], hstr[btn_pushed]);
				}
				else{
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
				}
			}
			else{
				set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
			}
			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				command = btn_released;
				if (btn_released < range_down){
					check[btn_released] = !check[btn_released];
					if (check[btn_released]){
						set_checked_color(hbtn[btn_released], hstr[btn_released]);
					}
					else{
						set_normal_color(hbtn[btn_released], hstr[btn_released]);
					}
				}
			}
			btn_released = nul;
			return true;
		}
	}

	return false;
}



/////////////////////////////////////////////////////////////////// c_route_cfg_box
const char * c_route_cfg_box::str_btn[nul] =
{
	"<", ">", "-", "+", "ADD", "DEL", "<", ">", "LOAD", "SAVE"
};

bool c_route_cfg_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = -xmax,
		ymax = (float)(0.5 * sz_scrn.y), ymin = -ymax;

	glm::vec2 sz_btn, sz_box, sz_udbtn;
	sz_btn.x = 5 * sz_fnt.x;
	sz_btn.y = sz_udbtn.y = 1.5 * sz_fnt.y;
	sz_udbtn.x = 2 * sz_fnt.x;

	sz_box.x = (float)(sz_btn.x * 2);
	sz_box.y = (float)(sz_btn.y * 5);

	glm::vec2 pos;

	// waypoint selection interface
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y + sz_box.y - sz_btn.y;
	add_select_box(hbtn[wp_prev], hstr[wp_prev], str_btn[wp_prev],
		hbtn[wp_next], hstr[wp_next], str_btn[wp_next], hstr_wp, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// waypoint speed selection interface
	pos.y = y + sz_box.y - 2 * sz_btn.y;
	add_select_box(hbtn[wp_spd_down], hstr[wp_spd_down], str_btn[wp_spd_down],
		hbtn[wp_spd_up], hstr[wp_spd_up], str_btn[wp_spd_up], hstr_spd, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// add/delete button
	pos.y = y + sz_box.y - 3 * sz_btn.y;
	add_btn(hbtn[wp_add], hstr[wp_add], str_btn[wp_add], pos, sz_btn, sz_fnt);
	pos.x += sz_btn.x;
	add_btn(hbtn[wp_del], hstr[wp_del], str_btn[wp_del], pos, sz_btn, sz_fnt);

	// route selection interface
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y + sz_box.y - 4 * sz_btn.y;
	add_select_box(hbtn[rt_prev], hstr[rt_prev], str_btn[rt_prev],
		hbtn[rt_next], hstr[rt_next], str_btn[rt_next], hstr_rt, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// save/load button
	pos.y = y + sz_box.y - 5 * sz_btn.y;
	add_btn(hbtn[rt_save], hstr[rt_save], str_btn[rt_save], pos, sz_btn, sz_fnt);
	pos.x += sz_btn.x;
	add_btn(hbtn[rt_load], hstr[rt_load], str_btn[rt_load], pos, sz_btn, sz_fnt);

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_route_cfg_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		porect->config_border(hbtn[ibtn], true, 1.0);
		porect->config_color(hbtn[ibtn], clr);
		potxt->enable(hstr[ibtn]);
	}
	potxt->enable(hstr_wp);
	potxt->enable(hstr_spd);
	potxt->enable(hstr_rt);
}

void c_route_cfg_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
	potxt->disable(hstr_wp);
	potxt->disable(hstr_spd);
	potxt->disable(hstr_rt);
}


bool c_route_cfg_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_route_cfg_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_route_cfg_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			set_normal_color(hbtn[btn_pushed]);
			if (btn_released == btn_pushed){
				command = btn_pushed;
				if (command == wp_add)
					set_checked_color(btn_pushed, hstr[btn_pushed]);
				else
					set_normal_color(btn_pushed, hstr[btn_pushed]);
				btn_released = btn_pushed = nul;
			}
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				command = btn_released;
				if (command == wp_add)
					set_checked_color(btn_pushed, hstr[btn_released]);
				else
					set_normal_color(btn_pushed, hstr[btn_released]);
			}
			btn_released = nul;
			return true;
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////// c_indicator
c_indicator::c_indicator() : porect(NULL), potri(NULL), poline(NULL), potxt(NULL),
meng(127), seng(127), rud(0), cog(0.1), sog(10), yaw(0.05), pitch(0.5), roll(0.5),
veng_n(0x7f), veng_nf(0x7f + 0x19), veng_nb(0x7f - 0x19), dir_cam(0.f), mode(im_fpv)
{

}

c_indicator::~c_indicator()
{

}

void c_indicator::create_engine_indicator(int & heng_in, int & heng_out,
	int & heng_n, int & heng_f, int & heng_b,
	glm::vec2 & pos, const glm::vec2 & sz_fnt, 
	const glm::vec4 & clr)
{
	glm::vec2
		pos_eng,
		pos_eng_out,
		pos_eng_in, 
		scl_eng_out,
		scl_eng_in, 
		pos_eng_g, pos_eng_n,  sz_g, sz_n;
	pos_eng = pos;
	sz_n.x = (float)(sz_fnt.x * 0.8);
	sz_n.y = (float)(sz_fnt.y * 0.8);
	sz_g.x = (float)(sz_fnt.x * 0.4);
	sz_g.y = (float)(sz_fnt.y * 0.4);
	pos_eng_in.x = (float)(pos_eng.x);
	pos_eng_in.y = (float)(pos_eng.y + sz_fnt.y);
	pos_eng_out = pos_eng_in;
	pos_eng_n.x = (float)(pos_eng.x + 0.1 * sz_fnt.x);
	pos_eng_n.y = (float)(pos_eng.y + 0.1 * sz_fnt.y);
	pos_eng_g.x = (float)(pos_eng.x + 0.5 * sz_fnt.x);
	pos_eng_g.y = (float)(pos_eng.y + 0.5 * sz_fnt.y);
	scl_eng_in.x = scl_eng.x;
	scl_eng_in.y = 0.f;
	scl_eng_out.x = scl_eng.x;
	scl_eng_out.y = scl_eng.y;
	heng_in = porect->add(clr, pos_eng_in, 0.f, 1.f);
	porect->config_border(heng_in, false, 1.f);
	porect->config_depth(heng_in, 0);
	porect->config_scale(heng_in, scl_eng_in);

	heng_out = porect->add(clr, pos_eng_out, 0.f, 1.f);
	porect->config_scale(heng_out, scl_eng_out);
	porect->config_border(heng_out, true, 1.f);
	porect->config_depth(heng_out, 0);

	heng_n = porect->add(clr, pos_eng_n, 0.f, sz_n);
	porect->config_border(heng_n, true, 1.f);
	porect->config_depth(heng_n, 0);
	heng_f = potri->add(clr, pos_eng_g, (float)(PI * 0.5), sz_g);
	potri->config_border(heng_b, false, 1.f);
	potri->config_depth(heng_f, 0);
	heng_b = potri->add(clr, pos_eng_g, (float)(-PI * 0.5), sz_g);
	potri->config_border(heng_b, false, 1.f);
	potri->config_depth(heng_b, 0);
}

void c_indicator::update_engine_indicator(int & heng_in, int & heng_n, int & heng_f, int & heng_b, const unsigned char val)
{
	glm::vec2 scl(scl_eng.x, scl_eng.y * abs((int)val - (int)127) * (1 / 127.));
	porect->config_scale(heng_in, scl);

	if (val == veng_n){
		porect->config_border(heng_n, false, 1.0);
		porect->enable(heng_n);
		potri->disable(heng_f);
		potri->disable(heng_b);
	}
	else if (val > veng_n){
		if (val > veng_nf){
			porect->disable(heng_n);
			potri->enable(heng_f);
			potri->disable(heng_b);
		}
		else{
			porect->config_border(heng_n, true, 1.0);
			porect->enable(heng_n);
			potri->enable(heng_f);
			potri->disable(heng_b);
		}
	}
	else{
		if (val < veng_nb){
			porect->disable(heng_n);
			potri->disable(heng_f);
			potri->enable(heng_b);
		}
		else{
			porect->config_border(heng_n, true, 1.0);
			porect->enable(heng_n);
			potri->disable(heng_f);
			potri->enable(heng_b);
		}
	}	
}

void c_indicator::create_rudder_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt,
	 const glm::vec4 & clr)
{
	glm::vec2 pos_rud_in, pos_rud_out, scl_rud_in, scl_rud_out;
	scl_rud_in.x = 0;
	scl_rud_in.y = sz_fnt.y;
	scl_rud_out = scl_rud;
	
	pos_rud_in.x = pos.x;
	pos_rud_in.y = pos.y;
	pos_rud_out.x = pos.x - scl_rud_out.x * 0.5;
	pos_rud_out.y = pos.y;

	hrud_in = porect->add(clr, pos_rud_in, 0.f, 1.f);
	porect->config_scale(hrud_in, scl_rud_in);
	porect->config_border(hrud_in, false, 1.f);
	porect->config_depth(hrud_in, 0.f);
	hrud_out = porect->add(clr, pos_rud_out, 0.f, 1.f);
	porect->config_scale(hrud_out, scl_rud_out);
	porect->config_border(hrud_out, true, 1.f);
	porect->config_depth(hrud_out, 0.f);
}

void c_indicator::update_rudder_indicator()
{
	int srud = (int) rud - (int) 127;
	
	glm::vec2 scl(scl_rud.x * abs(srud) * (1.0f / 255.0f), scl_rud.y);
	glm::vec2 pos;
	if (srud < 0)
		pos.x = (float)(pos_rud.x - scl.x);
	else
		pos.x = pos_rud.x;
	pos.y = pos_rud.y;
	porect->config_position(hrud_in, pos);
	porect->config_scale(hrud_in, scl);
}

void c_indicator::create_sog_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt, const glm::vec4 & clr)
{
#define RAD_SOG_ARC 6.5
#define NUM_SOG_SCALE (2*(SOG_STEP-1)+1)
#define NUM_SOG_ARC_PTS (2*(NUM_SOG_SCALE-1)+1)
	rad_sog = sz_fnt;
	rad_sog.x *= (RAD_SOG_ARC + 0.5);
	rad_sog.y *= (RAD_SOG_ARC + 0.5);

	pos_sog = pos;
	glm::vec2 mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y);
	struct s_vertex{
		float x, y;
	};
	// arc
	{
		s_vertex pts[NUM_SOG_ARC_PTS];
		float c, s, ths = (float)(PI / (float)(NUM_SOG_ARC_PTS -1));
		for (int i = 0; i < NUM_SOG_ARC_PTS; i++)
		{
			float th = (float)(i * ths);
			c = cos(th);
			s = sin(th);
			pts[i].x = c * sz_fnt.x * RAD_SOG_ARC;
			pts[i].y = s * sz_fnt.y * RAD_SOG_ARC;
		}

		hsog_arc = poline->add(NUM_SOG_ARC_PTS, (float*)pts);
		poline->config_position(hsog_arc, pos);
		poline->config_rotation(hsog_arc, 0);
		poline->config_width(hsog_arc, 1.0);
		poline->config_color(hsog_arc, clr);
		poline->config_depth(hsog_arc, 0.0);
	}

	// scale
	{
		float c, s, ths = (float)(PI / (float)(NUM_SOG_SCALE - 1));
		s_vertex pts[NUM_SOG_SCALE * 2];

		for (int i = 0, iscl = SOG_STEP - 1; i < NUM_SOG_SCALE; i++){
			float th = (float)(ths * i);
			c = (float)(cos(th) * sz_fnt.x);
			s = (float)(sin(th) * sz_fnt.y);
			s_vertex & vtx0 = pts[2 * i];
			s_vertex & vtx1 = pts[2 * i + 1];
			vtx0.x = (float)(c * RAD_SOG_ARC);
			vtx0.y = (float)(s * RAD_SOG_ARC);
			float rscl = (i % 2 == 0 ? RAD_SOG_ARC - 0.5 : RAD_SOG_ARC - 0.25);
			vtx1.x = (float)(c * rscl);
			vtx1.y = (float)(s * rscl);

			if (i % 2 == 0){ // creating scale string
				char str[4];
				float rstr = (float)(rscl - 0.25);
				glm::vec2 pos_str(c * rstr + pos.x, s * rstr + pos.y);
				snprintf(str, 4, "%d", iscl * 10);
				hstr_sog_scale[iscl] = potxt->reserv(2);
				potxt->set(hstr_sog_scale[iscl], str);
				potxt->config(hstr_sog_scale[iscl], clr, glm::vec4(0, 0, 0, 0),
					sz_fnt, mgn_fnt, c_gl_text_obj::an_ct, pos_str, (float)(th - 0.5 * PI));
				potxt->config_depth(hstr_sog_scale[iscl], 0);
				potxt->enable(hstr_sog_scale[iscl]);
				iscl--;
			}
		}

		hsog_scale = poline->add(NUM_SOG_SCALE * 2, (float*)pts, true);
		poline->config_depth(hsog_scale, 0);
		poline->config_position(hsog_scale, pos);
		poline->config_rotation(hsog_scale, 0.0);
		poline->config_color(hsog_scale, clr);
		poline->config_width(hsog_scale, 1.0f);
	}
	//indicator
	hsog_ptr = potri->add(clr, pos, 0.0f, 0.0f);
	potri->config_border(hsog_ptr, false, 1.0);
	glm::vec2 sz_ptr((float)(sz_fnt.x * 0.5), (float)(sz_fnt.y * 0.5));
	potri->config_scale(hsog_ptr, sz_ptr);
	potri->config_depth(hsog_ptr, 0);
	potri->enable(hsog_ptr);
}

void c_indicator::update_sog_indicator()
{
	float thtri =(float)(-sog * PI / 40.0);
	potri->config_rotation(hsog_ptr, thtri);
	float th = (float)(PI + thtri);
	glm::vec2 pos((float)(cos(th) * rad_sog.x + pos_sog.x), (float)(sin(th) * rad_sog.y + pos_sog.y));
	potri->config_position(hsog_ptr, pos);
}

void c_indicator::create_rp_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt, const glm::vec4 & clr)
{
	// pitch measure
	struct s_vertex{
		float x, y;
	};
	pos_rp = pos;
	lpmeas = 4 * sz_fnt.y;
	glm::vec2 sz_sfnt((float)(sz_fnt.x * 0.75), (float)(sz_fnt.y * 0.75));
	glm::vec2 mgn_sfnt((float)(sz_sfnt.x * 0.6), sz_fnt.y);

	{
		s_vertex pts[5] = {
			{ -sz_fnt.x, lpmeas },
			{ -sz_fnt.x, -lpmeas },
			{ 0, (float)(-(lpmeas + 2 * sz_fnt.y)) },
			{ sz_fnt.x, -lpmeas },
			{ sz_fnt.x, lpmeas }
		};
		hpmeas = poline->add(5, (float*)pts);
		poline->config_depth(hpmeas, 0);
		poline->config_position(hpmeas, pos);
		poline->config_rotation(hpmeas, 0.0);
		poline->config_color(hpmeas, clr);
		poline->config_width(hpmeas, 1.0f);
	}

	{
		s_vertex pts[5] = {
			{ -sz_fnt.x, 0 },
			{ -0.5 * sz_fnt.x , 0 },
			{ 0, (float)(- sz_fnt.y) },
			{ 0.5 * sz_fnt.x, 0 },
			{ sz_fnt.x, 0 }
		};
		hpptr = poline->add(5, (float*)pts);
		poline->config_depth(hpptr, 0);
		poline->config_position(hpptr, pos);
		poline->config_rotation(hpptr, 0.0);
		poline->config_color(hpptr, clr);
		poline->config_width(hpptr, 1.0f);
	}

	{
		s_vertex pts[((PITCH_STEP - 1) * 2 + 1) * 2];
		glm::vec2 ptstart(sz_fnt.x, lpmeas);
		float sstep = (float)(lpmeas * 2 / (float)((PITCH_STEP - 1) * 2));
		for (int i = 0, ip = PITCH_STEP - 1; i < PITCH_STEP * 2 - 1; i++){
			s_vertex & pt0 = pts[i * 2];
			s_vertex & pt1 = pts[i * 2 + 1];
			float l = (i % 2 == 0 ? 0.5 * sz_fnt.x : 0.25 * sz_fnt.y);
			pt0.x = ptstart.x;
			pt1.x = (float)(pt0.x + l);
			pt0.y = pt1.y = (float)(ptstart.y - sstep * i);

			if (i % 2 == 0){
				hstr_pscale[ip] = potxt->reserv(3);
				pos_pscale[ip].x = pt1.x;
				pos_pscale[ip].y = pt1.y;
				char str[4];
				snprintf(str, 4, "%d", abs((ip - PITCH_STEP / 2) * 10));
				potxt->set(hstr_pscale[ip], str);
				potxt->config(hstr_pscale[ip], clr, glm::vec4(0, 0, 0, 0),
					sz_sfnt, mgn_sfnt, c_gl_text_obj::an_lc, pos_pscale[ip] + pos_rp, 0.f);
				potxt->enable(hstr_pscale[ip]);
				ip--;
			}
		}

		hpscale = poline->add(PITCH_STEP * 4, (float*)pts, true);
		poline->config_depth(hpscale, 0);
		poline->config_position(hpscale, pos);
		poline->config_rotation(hpscale, 0.0);
		poline->config_color(hpscale, clr);
		poline->config_width(hpscale, 1.0f);
	}
#define NUM_RARC_PTS ((ROLL_STEP-1)*4+1)
#define RAD_RARC 6
	// roll arc
	{
		s_vertex pts[NUM_RARC_PTS];
		float c, s, ths = (float)(PI / (float)(NUM_RARC_PTS - 1));
		for (int i = 0; i < NUM_RARC_PTS; i++){
			float th = (float)(PI + i * ths);
			c = cos(th);
			s = sin(th);
			pts[i].x = (c * sz_fnt.x * RAD_RARC);
			pts[i].y = (s * sz_fnt.y * RAD_RARC);
		}

		hrarc = poline->add(NUM_RARC_PTS, (float*)pts);
		poline->config_position(hrarc, pos);
		poline->config_rotation(hrarc, 0);
		poline->config_width(hrarc, 1.0);
		poline->config_color(hrarc, clr);
		poline->config_depth(hrarc, 0.0);
	}
#define NUM_RSCL_PTS ((ROLL_STEP-1)*2+1)
	// roll scale
	{
		s_vertex pts[NUM_RSCL_PTS * 2];
		float rr;
		float c, s, ths = (float)(PI / (float)(NUM_RSCL_PTS - 1));
		for (int i = 0, ir = ROLL_STEP; i < NUM_RSCL_PTS; i++){
			float th = (float)(PI + i * ths);
			c = (float)(cos(th) * sz_fnt.x);
			s = (float)(sin(th) * sz_fnt.y);
			s_vertex & pt0 = pts[i * 2];
			s_vertex & pt1 = pts[i * 2 + 1];
			pt0.x = (float)(c * RAD_RARC);
			pt0.y = (float)(s * RAD_RARC);
			rr = (i % 2 == 0? RAD_RARC + 0.5 : RAD_RARC + 0.25);
			pt1.x = (float)(c * rr);
			pt1.y = (float)(s * rr);
			if (i % 2 == 0){
				char buf[4];
				glm::vec2 pos_str((float)(pt1.x + pos.x), (float)(pt1.y + pos.y));
				hstr_rscale[ir - 1] = potxt->reserv(3);
				snprintf(buf, 4, "%d", abs((ir - 10) * 10));
				potxt->set(hstr_rscale[ir-1], buf);
				potxt->config(hstr_rscale[ir-1], clr, glm::vec4(0, 0, 0, 0),
					sz_sfnt, mgn_sfnt, c_gl_text_obj::an_ct, pos_str, (float)(-0.5 * PI + ths * i));
				potxt->enable(hstr_rscale[ir - 1]);
				ir--;
			}
		}
		hrscale = poline->add(NUM_RSCL_PTS * 2, (float*)pts, true);
		poline->config_position(hrscale, pos);
		poline->config_rotation(hrscale, 0);
		poline->config_width(hrscale, 1.0);
		poline->config_color(hrscale, clr);
		poline->config_depth(hrscale, 0.0);
	}
}

void c_indicator::update_rp_indicator()
{
	float thr = -roll;
	float c = cos(thr), s = sin(thr);

	poline->config_rotation(hpmeas, roll, c, s);
	poline->config_rotation(hpscale, roll, c, s);
	poline->config_rotation(hpptr, roll, c, s);
	glm::vec2 pos_pptr(0, lpmeas * pitch * (1.0 / (PI * 40. / 180.)));
	pos_pptr.x = -s * pos_pptr.y + pos_rp.x;
	pos_pptr.y = c * pos_pptr.y + pos_rp.y;
	poline->config_position(hpptr, pos_pptr);

	for (int i = 0; i < PITCH_STEP; i++)
	{
		glm::vec2 pos;
		pos.x = c * pos_pscale[i].x - s * pos_pscale[i].y + pos_rp.x;
		pos.y = s * pos_pscale[i].x + c * pos_pscale[i].y + pos_rp.y;
		potxt->config_rotation(hstr_pscale[i], c, s);
		potxt->config_position(hstr_pscale[i], pos);
	}
}

void c_indicator::create_hc_indicator(const float fov, 
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const glm::vec4 & clr)
{
	fxcam = (float)(0.5 * cos(0.5 * fov * PI / 180.f) * sz_scrn.x);
	float xmin = (float)(-sz_scrn.x * 0.5), xmax = -xmin,
		ymin = (float)(-sz_scrn.y * 0.5), ymax = -ymin;

	struct s_vertex{
		float x, y;
	};

	glm::vec2 mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y), rad_ptr((float)(sz_fnt.x * 0.3), sz_fnt.y);
	glm::vec2 pos_str, pos_scale, pos_ptr;
	{
		s_vertex vtx[2] = {
			{ xmin, 0 }, { xmax, 0 }
		};
		pos_scale.x = pos_scale.y = 0;
		hhlzn = poline->add(2, (float*)vtx);
		poline->config_position(hhlzn, pos_scale);
		poline->config_rotation(hhlzn, 0);
		poline->config_width(hhlzn, 1.0);
		poline->config_color(hhlzn, clr);
		poline->config_depth(hhlzn, 0.0);

		vtx[0].x = vtx[1].x = pos_str.x = pos_ptr.x = 0;
		vtx[0].y = 0;
		vtx[1].y = pos_str.y = pos_ystr = (float)(sz_fnt.y * 0.5);
		pos_ptr.y = pos_yptr = (float)(- sz_fnt.y);
		float ths = (float)(2 * PI / (float) YAW_STEP);
		for (int i = 0; i < YAW_STEP; i++)
		{
			char str[3];
			
			hstr_yscale[i] = potxt->reserv(3);
			hyscale[i] = poline->add(2, (float*)vtx);
			snprintf(str, 3, "%02d", i);
			potxt->set(hstr_yscale[i], str);
			potxt->config(hstr_yscale[i], clr, glm::vec4(0, 0, 0, 0), 
				sz_fnt, mgn_fnt, c_gl_text_obj::an_cb, pos_str, 0);
			potxt->config_depth(hstr_yscale[i], 0);
			potxt->enable(hstr_yscale[i]);

			poline->config_position(hyscale[i], pos_scale);
			poline->config_rotation(hyscale[i], 0);
			poline->config_width(hyscale[i], 1.0);
			poline->config_color(hyscale[i], clr);
			poline->config_depth(hyscale[i], 0.0);
			pos_yscl[i].x = sin((float)(ths * i));
			pos_yscl[i].y = cos((float)(ths * i));
		}
	}

	hhptr = potri->add(clr, pos_ptr, (float)(0.5 * PI), rad_ptr);
	potri->config_border(hhptr, false, 1.0);
	potri->config_depth(hhptr, 0);
	potri->enable(hhptr);
	hcptr = potri->add(clr, pos_ptr, (float)(0.5 * PI), rad_ptr);
	potri->config_border(hcptr, true, 1.0);
	potri->config_depth(hcptr, 0);
	potri->enable(hcptr);
}

void c_indicator::update_hc_indicator()
{
	float dir_cam_abs = yaw + dir_cam;
	glm::vec2 pos_cam((float)sin(dir_cam_abs), (float)cos(dir_cam_abs));
	glm::vec2 pos_crs((float)sin(cog), (float)cos(cog));
	glm::vec2 pos_yaw((float)sin(yaw), (float)cos(yaw));

	if (mode == im_fpv){
		glm::vec2 pos_crs_tmp(
			(float)(pos_cam.y * pos_crs.x - pos_cam.x * pos_crs.y),
			(float)(pos_cam.x * pos_crs.x + pos_cam.y * pos_crs.y));
		if (pos_crs_tmp.y > 0){
			pos_crs_tmp.y /= pos_crs_tmp.x;
			pos_crs_tmp.x *= fxcam;
			pos_crs_tmp.y = pos_yptr;
			potri->config_position(hcptr, pos_crs_tmp);
			potri->enable(hcptr);
		}
		else{
			potri->disable(hcptr);
		}

		glm::vec2 pos_yaw_tmp(
			pos_cam.y * pos_yaw.x - pos_cam.x * pos_yaw.y,
			pos_cam.x * pos_yaw.x + pos_cam.y * pos_yaw.y);
		if (pos_yaw_tmp.y > 0){
			pos_yaw_tmp.x /= pos_yaw_tmp.y;
			pos_yaw_tmp.x *= fxcam;
			pos_yaw_tmp.y = pos_yptr;
			potri->config_position(hhptr, pos_yaw_tmp);
			potri->enable(hhptr);
		}
		else{
			potri->disable(hhptr);
		}

		for (int i = 0; i < YAW_STEP; i++){
			pos_yscl_tmp[i].x = pos_cam.y * pos_yscl[i].x - pos_cam.x * pos_yscl[i].y;
			pos_yscl_tmp[i].y = pos_cam.x * pos_yscl[i].x + pos_cam.y * pos_yscl[i].y;
			pos_yscl_tmp[i].x /= pos_yscl_tmp[i].y;
			pos_yscl_tmp[i].x *= fxcam;
			if (pos_yscl_tmp[i].y < 0){
				poline->disable(hyscale[i]);
				potxt->disable(hstr_yscale[i]);
			}
			else{
				poline->enable(hyscale[i]);
				pos_yscl_tmp[i].y = 0;
				poline->config_position(hyscale[i], pos_yscl_tmp[i]);
				potxt->enable(hstr_yscale[i]);
				pos_yscl_tmp[i].y = pos_ystr;
				potxt->config_position(hstr_yscale[i], pos_yscl_tmp[i]);
			}
		}
	}
	else{
		potri->disable(hhptr);
		potri->disable(hcptr);
		for (int i = 0; i < YAW_STEP; i++){
			poline->disable(hyscale[i]);
			potxt->disable(hstr_yscale[i]);
		}
	}
}

bool c_indicator::init(c_gl_2d_line_obj * _poline, c_gl_text_obj * _potxt,
	c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	const glm::vec2 & sz_fnt, const glm::vec4 & clr,
	const float fovx, const glm::vec2 & sz_scrn)
{
	poline = _poline;
	potxt = _potxt;
	porect = _porect;
	potri = _potri;

	float xmin = (float) (-sz_scrn.x * 0.5), xmax = (float)(-xmin), 
		ymin = (float)(-sz_scrn.y * 0.5), ymax = (float)(-ymin);
	// create meng/seng indicator (left bottom corner) 

	scl_eng.x = sz_fnt.x;
	scl_eng.y = (float)(0.25 * sz_scrn.y);
	scl_rud.x = (float)(sz_scrn.x * 0.4);
	scl_rud.y = sz_fnt.y;

	glm::vec2 pos_meng, pos_seng;
	pos_meng.x = xmin;
	pos_meng.y = ymin;
	pos_seng.x = xmin + sz_fnt.x;
	pos_seng.y = ymin;
	pos_rud.x = 0.f;
	pos_rud.y = ymin;
	create_engine_indicator(hmeng_in, hmeng_out, hmeng_n, hmeng_f, hmeng_b, pos_meng, sz_fnt, clr);
	create_engine_indicator(hseng_in, hseng_out, hseng_n, hseng_f, hseng_b, pos_seng, sz_fnt, clr);

	// create rudder indicator (bottom center)
	create_rudder_indicator(pos_rud, sz_fnt, clr);

	// create sog indicator (right bottom corner)
	glm::vec2 pos_sog_indicator(xmax- 8 * sz_fnt.x, ymin + 10 * sz_fnt.y);
	create_sog_indicator(pos_sog_indicator, sz_fnt, clr);

	// create roll/pitch indicator (right bottom corner)
	glm::vec2 pos_rp_indicator(xmax - 8 * sz_fnt.x, ymin + 8 * sz_fnt.y);
	create_rp_indicator(pos_rp_indicator, sz_fnt, clr);

	// create hdg/cog indicator (top center, only in fpv)
	create_hc_indicator(fovx, sz_fnt, sz_scrn, clr);

	update_engine_indicator(hmeng_in, hmeng_n, hmeng_f, hmeng_b, meng);
	update_engine_indicator(hseng_in, hseng_n, hseng_f, hseng_b, seng);
	update_sog_indicator();
	update_rp_indicator();
	update_hc_indicator();
	update_rudder_indicator();

	return true;
}

void c_indicator::set_param(
	const unsigned char _meng, const unsigned char _seng, const unsigned char _rud,
	const float _cog, const float _sog,
	const float _yaw, const float _pitch, const float _roll)
{
	meng = _meng;
	seng = _seng;
	rud = _rud;
	cog = _cog;
	sog = _sog;
	yaw = _yaw;
	pitch = _pitch;
	roll = _roll;

	update_engine_indicator(hmeng_in, hmeng_n, hmeng_f, hmeng_b, meng);
	update_engine_indicator(hseng_in, hseng_n, hseng_f, hseng_b, seng);
	update_sog_indicator();
	update_rp_indicator();
	update_hc_indicator();
	update_rudder_indicator();
}

/////////////////////////////////////////////////////////////////// c_ui_waypoint_obj
c_ui_waypoint_obj::c_ui_waypoint_obj() :pocirc(NULL), potxt(NULL)
{
}

bool c_ui_waypoint_obj::init(c_gl_2d_obj * _pocirc, c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline,
	const glm::vec4 & clr, const glm::vec2 & sz_fnt, const float _rmark, const unsigned int _nmaxwps)
{
	nmaxwps = _nmaxwps;
	rmark = _rmark;
	pocirc = _pocirc;
	potxt = _potxt;
	poline = _poline;

	glm::vec2 pos(0.f, 0.f), r(_rmark, _rmark), mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y);
	wps.resize(nmaxwps);
	hmarks.resize(nmaxwps);
	for (int i = 0; i < nmaxwps; i++){
		hmarks[i].hmark = pocirc->add(clr, pos, 0.0f, r);
		pocirc->config_border(hmarks[i].hmark, true, 1.0);
		pocirc->config_depth(hmarks[i].hmark, 0);
		pocirc->disable(hmarks[i].hmark);
		hmarks[i].hstr = potxt->reserv(15);
		potxt->config(hmarks[i].hstr, clr, glm::vec4(0, 0, 0, 0), sz_fnt, mgn_fnt, c_gl_text_obj::an_lb, pos, 0, 0);
	}
	return true;
}

void c_ui_waypoint_obj::update_wps(const int iwp, const s_wp & wp)
{
	if (iwp < nmaxwps){
		wps[iwp] = wp;
	}
}

void c_ui_waypoint_obj::enable(const int iwp)
{
	if (iwp < nmaxwps)
		pocirc->enable(hmarks[iwp].hmark);
}

void c_ui_waypoint_obj::disable(const int iwp)
{
	if (iwp < nmaxwps){
		pocirc->enable(hmarks[iwp].hmark);
	}
}

void c_ui_waypoint_obj::disable()
{
	for (int i = 0; i < nmaxwps; i++){
		pocirc->disable(hmarks[i].hmark);
	}
}

void c_ui_waypoint_obj::set_focus(const int iwp)
{
	focus = iwp;
}

int c_ui_waypoint_obj::collision(const glm::vec2 pos)
{
	for (int i = 0; i < nmaxwps; i++){
		if (pocirc->is_enabled(hmarks[i].hmark)){
			if (pocirc->collision(pos, hmarks[i].hmark))
				return i;
		}
	}
	return -1;
}

/////////////////////////////////////////////////////////////////// c_ais_obj
bool c_ui_ais_obj::init()
{
	return true;
}

void c_ui_ais_obj::render()
{
}

/////////////////////////////////////////////////////////////////// c_aws_ui_box_manager
c_aws_ui_box_manager::c_aws_ui_box_manager() : box_pushed(nul), box_released(nul),
blpushed(false), blreleased(false)
{	
}

c_aws_ui_box_manager::~c_aws_ui_box_manager()
{
	if (pboxes.size()){
		for (int ibox = 0; ibox < pboxes.size(); ibox++)
			delete pboxes[ibox];
		pboxes.clear();
		hboxes.clear();
	}
}

bool c_aws_ui_box_manager::init(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline,
	const glm::vec4 & _clr, const glm::vec4 & _bkgclr, const glm::vec2 & _sz_font, 
	const float fovx, const glm::vec2 & _sz_screen)
{
	clr = _clr;
	bkgclr = _bkgclr;
	sz_font = _sz_font;
	sz_screen = _sz_screen;
	porect = _porect;
	potri = _potri;
	potxt = _potxt;
	poline = _poline;
	c_aws_ui_box::set_gl_objs(porect, potri, potxt, poline);

	{
		glm::vec2 lb(0, 0);
		glm::vec2 scale(0, 0);
		porect->add(clr, lb, 0, scale);
		porect->add(clr, lb, 0, scale);
		porect->add(clr, lb, 0, scale);
		porect->add(clr, lb, 0, scale);
	}

	pboxes.resize(4, NULL);
	hboxes.resize(4);

	pboxes[view_mode] = ((c_aws_ui_box*) new c_view_mode_box());
	pboxes[ctrl_mode] = ((c_aws_ui_box*) new c_ctrl_mode_box());
	pboxes[obj_cfg] = ((c_aws_ui_box*) new c_obj_cfg_box());
	pboxes[route_cfg] = ((c_aws_ui_box*) new c_route_cfg_box());
	hboxes[view_mode] = 0;
	hboxes[ctrl_mode] = 1;
	hboxes[obj_cfg] = 2;
	hboxes[route_cfg] = 3;

	float xmax = (float)(0.5 * sz_screen.x), xmin = -xmax,
		ymax = (float)(0.5 * sz_screen.y), ymin = -ymax;
	float y;

	y = (float)(ymax - pboxes[view_mode]->get_box_size(sz_font).y);
	pboxes[view_mode]->init(view_mode, clr, bkgclr, sz_font, sz_screen, y, true);
	y -= pboxes[ctrl_mode]->get_box_size(sz_font).y;
	pboxes[ctrl_mode]->init(ctrl_mode, clr, bkgclr, sz_font, sz_screen, y, true);

	y = (float)(ymax - pboxes[obj_cfg]->get_box_size(sz_font).y);
	pboxes[obj_cfg]->init(obj_cfg, clr, bkgclr, sz_font, sz_screen, y, false);
	y -= pboxes[route_cfg]->get_box_size(sz_font).y;
	pboxes[route_cfg]->init(route_cfg, clr, bkgclr, sz_font, sz_screen, y, false);

	return true;
}


bool c_aws_ui_box_manager::set_mouse_event(const glm::vec2 & pt,
	const int button, const int action, const int modifier)
{
	bool handled = false;
	if (button == GLFW_MOUSE_BUTTON_LEFT){
		if (action == GLFW_PRESS){
			for (int ibox = 0; ibox < nul; ibox++){
				if (porect->collision(pt, hboxes[ibox])){
					box_pushed = (e_box)ibox;
					pboxes[box_pushed]->set_mouse_event(pt, button, action, modifier);
					handled = true;
					blpushed = true;
				}
			}
		}

		if (action == GLFW_RELEASE){
			blreleased = true;
			for (int ibox = 0; ibox < nul; ibox++){
				if (porect->collision(pt, hboxes[ibox])){
					box_released = (e_box)ibox;
					if (box_pushed != nul)
						pboxes[box_pushed]->set_mouse_event(pt, button, action, modifier);
					else
						pboxes[box_released]->set_mouse_event(pt, button, action, modifier);
					handled = true;
				}
			}
		}
	}

	// process the input 
	box_updated = nul;
	if (blpushed){
		if (box_pushed != nul){
			if (pboxes[box_pushed]->proc(blpushed, blreleased))
				box_updated = box_pushed;
		}
		if (blreleased){
			box_pushed = nul;
			box_released = nul;
			blreleased = false;
			blpushed = false;
		}
	}
	else{
		if (box_released != nul){
			if(pboxes[box_released]->proc(blpushed, blreleased))
				box_updated = box_released;
		}
		if (blreleased){
			box_pushed = nul;
			box_released = nul;
			blreleased = false;
			blpushed = false;
		}
	}

	return handled;
}
