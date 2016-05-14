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

#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include "f_aws1_ui.h"

const char * f_aws1_ui::m_str_aws1_ui_mode[AUM_UNDEF] = {
	"normal", "map", "dev"
};

f_aws1_ui::f_aws1_ui(const char * name): f_glfw_window(name), 
					m_state(NULL), m_ch_sys(NULL),
					 m_ch_ctrl_inst(NULL), m_ch_ctrl_stat(NULL), m_ch_wp(NULL),
					 m_ch_map(NULL),
					 m_ch_obj(NULL), m_ch_ais_obj(NULL), m_ch_img(NULL),
					 m_mode(AUM_NORMAL), m_ui_menu(false), m_menu_focus(0),
					 m_fx(0.), m_fy(0.), m_cx(0.), m_cy(0.)
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

  register_fpar("storage", m_path_storage, 1024, "Path to the storage device");

  register_fpar("acs", (int*) &m_stat.ctrl_src, (int) ACS_NONE, str_aws1_ctrl_src, "Control source.");
  register_fpar("verb", &m_verb, "Debug mode.");
  
  register_fpar("js", &m_js_id, "Joystick id");

  register_fpar("mode", (int*)&m_mode, AUM_UNDEF, m_str_aws1_ui_mode, "UI mode.");
  m_ui[AUM_NORMAL]	= new c_aws1_ui_normal(this);
  m_ui[AUM_MAP]		= new c_aws1_ui_map(this);
  m_ui[AUM_DEV]		= new c_aws1_ui_dev(this);

  register_fpar("menu", &m_ui_menu, "Invoke menu");

  register_fpar("fx", &m_fx, "Focal length in x direction.");
  register_fpar("fy", &m_fy, "Focal length in y direction.");
  register_fpar("cx", &m_cx, "Principal point in x direction.");
  register_fpar("cy", &m_cy, "Principal point in y direction.");
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


bool f_aws1_ui::init_run()
{
  m_inst.ctrl_src = ACS_UI;
  m_inst.tcur = m_cur_time;
  m_inst.rud_aws = 127;
  m_inst.meng_aws = 127;
  m_inst.seng_aws = 127;


  if(!f_glfw_window::init_run())
    return false;

  if(m_js.init(m_js_id)){
	  cout << "Joystick " << m_js.name << " found." << endl;
  }


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

void f_aws1_ui::ui_show_img()
{
	glRasterPos2i(-1, -1);
	if(m_ch_img){
		Mat img;
		long long timg;
		img = m_ch_img->get_img(timg);
		if(!img.empty()){
			if(m_sz_win.width != img.cols || m_sz_win.height != img.rows){
				Mat tmp;
				resize(img, tmp, m_sz_win);
				img = tmp;
			}

			if(img.type() == CV_8U){
				glDrawPixels(img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);
			}
			else{
				cnvCVBGR8toGLRGB8(img);
				glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);
			}
		}
	}
	else
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
	float y = (float)(yorg - 2 * hfont);
	drawGlText(x, y, str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

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
	//  <UI     > | <Value>  //
	//  <Exit   > | <Value>  //
	//-----------------------//
	//<Apply(a)>  <Cancel(b)>//
	///////////////////////////
	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	const char * title = "System Menu"; /* 11 characters */

	const char * items[3] = { /* 7characters the longest.*/
		"Control", "UI", "Quit"
	};

	float alpha_bg = 0.5;
	float alpha_txt = 1.0;
	
	float wcol_l = 12 * wfont;
	float wcol_r = 12 * wfont;
	float wm = wcol_l + wcol_r + 2 * wfont;
	float hm = (float)(6.5 * hfont); 
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

	// Control
	y -= (float)(1.5 * hfont);
	if(m_menu_focus == 0)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	float g = m_menu_focus == 0 ? 0.f : 1.f;
	drawGlText(x, y, items[0], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, str_aws1_ctrl_src[m_menu_acs], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// UI
	y -= (float)(hfont);
	if(m_menu_focus == 1)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	g = m_menu_focus == 1 ? 0.f : 1.f;
	drawGlText(x, y, items[1], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
	drawGlText(xv, y, m_str_aws1_ui_mode[m_menu_mode], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);

	// Exit
	y -= (float)(hfont);
	if(m_menu_focus == 2)
		drawGlSquare2Df(xorg, (float)(y + 0.85 * hfont), (float)(xorg + wm), (float)(y - 0.15 * hfont),
			0, 1, 0, alpha_bg);
	g = m_menu_focus == 2 ? 0.f : 1.f;
	drawGlText(x, y, items[2], 0, g, 0, alpha_txt, GLUT_BITMAP_8_BY_13);
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
		m_ui_menu = false;
	}

	if(m_js.edx & s_jc_u3613m::EB_EVDOWN || m_js.tdx > 60){
		m_js.tdx = 0;
		m_menu_focus = (m_menu_focus + 1 ) % 3;
	}

	if(m_js.eux & s_jc_u3613m::EB_EVDOWN || m_js.tux > 60){
		m_js.tux = 0;
		m_menu_focus = (m_menu_focus + 2 ) % 3;
	}

	if(m_js.elx & s_jc_u3613m::EB_EVDOWN || m_js.tlx > 60){
		m_js.tlx = 0;
		switch(m_menu_focus){
		case 0:
			m_menu_acs = (e_aws1_ctrl_src) ((m_menu_acs + ACS_NONE - 1) % ACS_NONE);
			break;
		case 1:
			m_menu_mode = (e_aws1_ui_mode) ((m_menu_mode + AUM_UNDEF - 1) % AUM_UNDEF);
			break;
		case 2:
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
			m_menu_mode = (e_aws1_ui_mode) ((m_menu_mode + 1) % AUM_UNDEF);
			break;
		case 2:
			m_quit = !m_quit;
			break;
		}
	}
}

bool f_aws1_ui::proc()
{
	if(m_ch_img){
		double fx, fy, cx, cy;
		bool bfx, bfy, bcx, bcy;

		if(bfx = m_ch_img->get_int_campar(ECP_FX, fx)){
			m_fx = (float)(fx * m_ixscale);

			bfy = m_ch_img->get_int_campar(ECP_FY, fy);
			if(!bfy) // if y focal length is not sat, fx is used.
				m_fy = (float)(fx * m_iyscale);
			else
				m_fy = (float)(fy * m_iyscale);

			// Note that the image coordinate is left to right, top to bottom.
			// In the OpenGL coordinate, the vertical axis should be upsidedown.
			bcx = m_ch_img->get_int_campar(ECP_CX, cx);
			if(bcx){
				m_cx = (float)(cx * m_ixscale - 1.0);
			}else{
				m_cx = 0.;
			}

			bcy = m_ch_img->get_int_campar(ECP_CY, cy);
			if(bcy){
				m_cy = (float)(1. - cy * m_iyscale);
			}else{
				m_cy = 0.;
			}
		}
	}

	c_aws1_ui_core & ui = *m_ui[m_mode];

	// process joypad inputs
	if(m_js.id != -1){
		m_js.set_btn();
		m_js.set_stk();
	}

	ui_force_ctrl_stop();

	if(!m_ui_menu)
		ui.js(m_js); // mode dependent joypad handler

	//--> system joypad handling
	ui_handle_menu();
	//<-- system joypad handling	

	// communictation with control channels or control udp sockets
	snd_ctrl_inst();
	rcv_ctrl_stat();

	// Window forcus is now at this window
	glfwMakeContextCurrent(pwin());

	// render graphics
	if(m_xscale != (float) m_sz_win.width || m_yscale != (float) m_sz_win.height){
		m_xscale = (float) (m_sz_win.width * 0.5);
		m_yscale = (float) (m_sz_win.height * 0.5);
		m_ixscale = (float)(2.0 / (double) m_sz_win.width);
		m_iyscale = (float)(2.0 / (double) m_sz_win.height);
	}

	// information rendering
	ui.draw(); // mode dependent rendering

	//--> system information rendering
	// show time

	float wfont = (float)(8. * m_ixscale);
	float hfont = (float)(13. * m_iyscale);
	float x = (float)(wfont - 1);
	float y = (float)(1 - 2 * hfont);
	drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

	ui_show_sys_state();

	if(m_ui_menu)
		ui_show_menu();
	
	//<-- system information rendering

	// show rendering surface.
	glfwSwapBuffers(pwin());

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

void f_aws1_ui::_mouse_button_callback(int button, int action, int mods)
{
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
