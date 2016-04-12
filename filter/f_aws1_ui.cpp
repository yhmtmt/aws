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

#include <GL/glew.h>

#include <GLFW/glfw3.h>
#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glut.h>
#include <GL/glu.h>

#include "f_aws1_ui.h"

f_aws1_ui::f_aws1_ui(const char * name): f_glfw_window(name), 
					m_state(NULL),
					 m_ch_ctrl_in(NULL), m_ch_ctrl_out(NULL), m_ch_img(NULL),
					 m_acd_sock(-1), m_acd_port(20100), m_num_ctrl_steps(4), m_ec(EC_MAIN), m_rud_pos(NULL), m_meng_pos(NULL), m_seng_pos(NULL)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_ctrl_in", (ch_base**)&m_ch_ctrl_in, typeid(ch_aws1_ctrl).name(), "Control input channel.");
  register_fpar("ch_ctrl_out", (ch_base**)&m_ch_ctrl_out, typeid(ch_aws1_ctrl).name(), "Control output channel.");
  register_fpar("ch_img", (ch_base**)&m_ch_img, typeid(ch_image_ref).name(), "Image channel");

  register_fpar("udpctrl", &m_udp_ctrl, "If asserted, Direct UDP is used for control channel. Otherwise, ch_ctrl_{in,out} are used.");
  register_fpar("acdhost", m_acd_host, 1023, "Host address controlling AWS1.");
  register_fpar("acdport", &m_acd_port, "Port number opened for controlling AWS1.");
  register_fpar("acs", (int*) &m_acp.ctrl_src, (int) ACS_NONE, str_aws1_ctrl_src, "Control source.");
  register_fpar("verb", &m_verb, "Debug mode.");
  register_fpar("rud", &m_rud_aws_f, "Rudder.");
  register_fpar("meng", &m_meng_aws_f, "Main Engine.");
  register_fpar("seng", &m_seng_aws_f, "Sub Engine.");
  
  register_fpar("js", &m_js_id, "Joystick id");
}


f_aws1_ui::~f_aws1_ui()
{
}


bool f_aws1_ui::init_run()
{
  m_acp.ctrl_src = ACS_UI;
  m_acp.rud_aws = 127;
  m_acp.meng_aws = 127;
  m_acp.seng_aws = 127;
  m_rud_aws_f = 127.;
  m_meng_aws_f = 127.;
  m_seng_aws_f = 127.;

  // initializing udp socket
  if(m_udp_ctrl){
    m_acd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_acd_sock == -1){
      cerr << "Failed to create control socket in " << m_name << "." << endl;
      return false;
    }
    
    m_acd_sock_addr.sin_family = AF_INET;
    m_acd_sock_addr.sin_port = htons(m_acd_port);
    set_sockaddr_addr(m_acd_sock_addr, m_acd_host);
  }

  if(!f_glfw_window::init_run())
    return false;

  if(m_js.init(m_js_id)){
	  cout << "Joystick " << m_js.name << " found." << endl;
  }

  // allocate control positions
  m_rud_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];
  m_meng_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];
  m_seng_pos = new unsigned char[m_num_ctrl_steps * 2 + 1];

  double stepf, stepb;
  double sumf, sumb;
  m_rud_pos[m_num_ctrl_steps] = 127;
  stepf = (double) (255 - 127) / (double) m_num_ctrl_steps;
  stepb = (double) (127 - 0) / (double) m_num_ctrl_steps;
  sumf = sumb = 127.;
  for(int i = 1; i < m_num_ctrl_steps; i++){
    sumf += stepf;
    sumb -= stepb;
    m_rud_pos[m_num_ctrl_steps - i] = (unsigned char) sumb;
    m_rud_pos[m_num_ctrl_steps + i] = (unsigned char) sumf;
  }
  m_rud_pos[m_num_ctrl_steps * 2] = 255;
  m_rud_pos[0] = 0;

  stepf = (double) (255 - 127 - 25) / (double) (m_num_ctrl_steps - 1);
  stepb = (double) (127 - 25 - 0) / (double) (m_num_ctrl_steps - 1);
  sumf = sumb = 127.;

  m_meng_pos[m_num_ctrl_steps] = 127;
  m_meng_pos[m_num_ctrl_steps+1] = 127 + 25;
  m_meng_pos[m_num_ctrl_steps-1] = 127 - 25;

  m_seng_pos[m_num_ctrl_steps] = 127;
  m_seng_pos[m_num_ctrl_steps+1] = 127 + 25;
  m_seng_pos[m_num_ctrl_steps-1] = 127 - 25;

  sumf = 127 + 25;
  sumb = 127 - 25;
  for (int i = 2; i < m_num_ctrl_steps; i++){
    sumf += stepf;
    sumb -= stepb;
    m_meng_pos[m_num_ctrl_steps + i] = m_seng_pos[m_num_ctrl_steps + i] = saturate_cast<unsigned char>(sumf);
    m_meng_pos[m_num_ctrl_steps - i] = m_seng_pos[m_num_ctrl_steps - i] = saturate_cast<unsigned char>(sumb);
  }
  
  m_meng_pos[m_num_ctrl_steps * 2] = m_seng_pos[m_num_ctrl_steps * 2] = 255;
  m_meng_pos[0] = m_seng_pos[0] = 0;

  return true;
}

void f_aws1_ui::destroy_run()
{
  delete[] m_rud_pos;
  delete[] m_meng_pos;
  delete[] m_seng_pos;

  m_rud_pos = NULL;
  m_meng_pos = NULL;
  m_seng_pos = NULL;

  if(m_udp_ctrl){
    closesocket(m_acd_sock);
    m_acd_sock = -1;
  }
}

bool f_aws1_ui::proc()
{
  if(m_js.id != -1){
	  m_js.set_btn();
	  m_js.set_stk();
  // joystic handling (assuming JC-U3613M)
    // Stick value
    // U: -1
    // D: 1
    // L: -1
    // R: 1
    // Button Value: 
    //off: 0  
    //on: 1
    // AXES 0: 9LR 1: 9UD 2: 10LR 3: 10UD 4: XLR 5: XUD 
    // BTNS 0: 1 1: 2 2: 3 3: 4 4: 5 5: 6 7: 8 8: 9 9: 10 10: 11 12: 13
    //      X, Y, A, B, LB, RB, LT, LSt, RSt, RT, BACK, START, Guide
    // axis 0, 2  is assigned to rudder
    // axis 1 is assigned to main engine control
    // axis 3 is assigned to sub engine control
    m_rud_aws_f += (float)(m_js.lr1 * (255. / 180.));
    m_rud_aws_f += (float)(m_js.lr2 * (255. / 180.));
    m_rud_aws_f = min((float)255.0, m_rud_aws_f);
    m_rud_aws_f = max((float)0.0, m_rud_aws_f);

    m_meng_aws_f -= (float)(m_js.ud1 * (255. / 180));
    m_meng_aws_f = min((float)255.0, m_meng_aws_f);
    m_meng_aws_f = max((float)0.0, m_meng_aws_f);

    m_seng_aws_f -= (float)(m_js.ud2 * (255. / 180));
    m_seng_aws_f = min((float) 255.0, m_seng_aws_f);
    m_seng_aws_f = max((float)0.0, m_seng_aws_f);
    
    if(m_verb){
		m_js.print(cout);
    }
  }

  m_acp.rud_aws = (unsigned char) m_rud_aws_f;
  m_acp.meng_aws = (unsigned char) m_meng_aws_f;
  m_acp.seng_aws = (unsigned char) m_seng_aws_f;

  s_aws1_ctrl_pars acpkt;
  snd_ctrl(acpkt);
  rcv_ctrl(acpkt);
 
  float roll, pitch, yaw;
  float lat, lon, alt, galt;
  float cog, sog;
  float depth;
  roll = pitch = yaw = lat = lon = alt = galt = cog = sog = depth = 0.;
  if(m_state){
	  m_state->get_attitude(roll, pitch, yaw);
	  m_state->get_position(lat, lon, alt, galt);
	  m_state->get_velocity(cog, sog);
	  m_state->get_depth(depth);
  }
  yaw = (float)(-yaw + 180.);

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
  
  // render graphics
  glfwMakeContextCurrent(pwin());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glRasterPos2i(-1, -1);
  double wscale = 1.0 / (double) m_sz_win.width;
  double hscale = 1.0 / (double) m_sz_win.height;  
  float wfont = (float)(13. * wscale);
  float hfont = (float)(13. * hscale);
  float x = (float)(wfont - 1);
  float y = (float)(1 - 3 * hfont);

  // show time
  drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);

  float rud_inst, rud_inst_cur;
  float meng_inst, meng_inst_cur;
  float seng_inst, seng_inst_cur;
  float rud_sta;

  rud_inst = (float)m_acp.rud_aws;
  rud_inst_cur = 
    (float)map_oval(m_acp.rud, 
		    m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min,
		    0xff, 0x7f, 0x00);

  meng_inst = (float)m_acp.meng_aws;
  meng_inst_cur = 
    (float)map_oval(m_acp.meng,
		    m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, 
		    m_acp.meng_nub, m_acp.meng_min,
		    0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
  
  seng_inst = (float)m_acp.seng_aws;
  seng_inst_cur = 
    (float) map_oval(m_acp.seng,
		     m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, 
		     m_acp.seng_nub, m_acp.seng_min,
		     0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
  
  rud_sta = 
    (float) map_oval(m_acp.rud_sta, 
		     m_acp.rud_sta_max, m_acp.rud_sta_nut, m_acp.rud_sta_min,
		     0xff, 0x7f, 0x00);
  
  if(m_verb){
    cout << "Control value in " << m_name << endl;
    cout << "    Inst rud " << rud_inst << " meng " << meng_inst << " seng " << seng_inst << endl;
    cout << "    Ctrl rud " << rud_inst_cur << " meng " << meng_inst_cur << " seng " << seng_inst_cur << endl;
    cout << "    Rud stat " << rud_sta << endl;
    cout << " RPY = " << roll << "," << pitch << "," << yaw << endl;
    cout << " Pos = " << lat << "," << lon << "," << alt << endl;
    cout << " Vel = " << cog << "," << sog << endl;
    cout << " Depth = " << depth << endl;
  }

  float wm, hm, lw;

  // Draw main view (1: camera image, 2: 3D rendered map, 3: 2D rendered map)

  // Drawing engine control indicator 
  wm = (float)(3 * wfont);
  hm = (float)(255.0 * hscale);
  lw = (float)(1.0 / m_sz_win.width);

  x = (float)(wfont - 1.0);
  y = (float)(1.0 - 6 * hfont);
  drawGlEngineIndicator("M/E", x, y, wm, hm, wfont, hfont, lw, 
			(float)(meng_inst * hscale),
			(float)(meng_inst_cur * hscale));
 
  x += (float)(5 * wfont);
  drawGlEngineIndicator("S/E", x, y, wm, hm, wfont, hfont, lw, 
			(float)(seng_inst * hscale),
			(float)(seng_inst_cur * hscale));
  

  // Drawing rudder control indicator
  wm = (float)(255. * wscale);
  hm = (float)(3 * hfont);
  x =  (float)(0. - 255. * 0.5 * wscale);
  drawGlRudderIndicator("RUDDER", 
			x, y, 
			wm, hm,  wfont, hfont, lw, 
			(float)(rud_inst * wscale),
			(float)(rud_inst_cur * wscale),
			(float)(rud_sta * wscale));

  // Drawing ship state information
  x = (float)(wfont - 1.0);
  y = (float)(hfont - 1.0);
  drawGlStateInfTxt(x, y, wfont, hfont, 
	  lat, lon, alt, galt, cog, sog, roll, pitch, yaw, depth, 1);

  // Indicate System State
  x = (float)(1.0 - wfont);
  y = (float)(1.0 - hfont);
  drawGlSysStateInfTxt(x, y, wfont, hfont, m_acp.ctrl_src, 1);

  // Drawing attitude indicator (w-mark, hdg scale, pitch scale) for main view type 1 only

  // Drawing map information. both for main view type 1 and 2. 

  
  glfwSwapBuffers(pwin());
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

void drawGlRudderIndicator(const char * title,
			   float xorg, float yorg, float w, float h,
			   float wfont, float hfont, 
			   float lw, float rud_inst, float rud_cur, float rud_sta)
{
  float x1, x2, y1, y2, ytxt;
  
  // draw title
  drawGlText((float)(xorg + 0.5 * (w - wfont * (double)strlen(title))), 
	     (float)(yorg + 0.5 * hfont), title, 
	     0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
  
  x1 = xorg;
  x2 = (float)(xorg + w);
  y1 = yorg;
  y2 = (float)(yorg - h);
  
  ytxt = (float) (yorg - 1.2 * h - hfont);

  // indicator box
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);

  x1 = (float)(xorg + 0.5 * w);

  // rudder instruction value
  x2 = (float)(rud_inst - 0.5 * w);
  drawGlLine2Df(x2, y1, x2, y2, 0, 1, 0, 1, lw);

  // current rudder instruction value
  x2 = (float)(rud_cur - 0.5 * w);
  y2 = (float)(yorg - 0.666 * h);
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // current rudder angle
  x2 = (float)(rud_sta - 0.5 * w);
  y1 = y2;
  y2 = (float)(yorg - h) ;
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // Midship
  float dwfont = (float)(- 0.5 * wfont);
  drawGlLine2Df(x1, y1, x1, y2, 0, 1, 0, 1, lw);
  drawGlText(x1 + dwfont, ytxt, "0", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
  drawGlText(xorg + dwfont, ytxt, "P", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
  drawGlText((float)(xorg + w + dwfont), ytxt, "S", 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
}

void drawGlStateInfTxt(float xorg, float yorg, 
		       float wfont, float hfont,
		       float lat, float lon, float alt, float galt, 
		       float cog, float sog, 
		       float roll, float pitch, float yaw,
		       float depth, float sz)
{
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
	float w = (float)((strlen(salt) + 2) * wfont * 1.2);
	float h = (float)(20 * hfont);
	drawGlSquare2Df(xorg, yorg, (float)(xorg + w), (float)(yorg + h), 0, 1, 0, 1, sz);

	float x, y;
	x = (float)(xorg + wfont);
	y = (float)(yorg + hfont);

	drawGlText(x, y, slat, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, slon, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, salt, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, scog, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, ssog, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, syaw, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, spch, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, srol, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
	y += 2 * hfont;
	drawGlText(x, y, sdpt, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
}

void drawGlSysStateInfTxt(float xorg, float yorg,
			  float wfont, float hfont,
			  e_aws1_ctrl_src ctrl_src, float sz)
{
	char str[32]; // "Ctrl: xxxx"
	snprintf(str, 32, "CTRL: %5s", str_aws1_ctrl_src[ctrl_src]);
	float w = (float)((strlen(str) + 2) * wfont * 1.2);
	float h = (float)(4 * hfont);

	float x = (float)(xorg - w);
	drawGlSquare2Df(xorg, yorg, x, (float)(yorg - h), 0, 1, 0, 1, sz);
	x += wfont;
	float y = (float)(yorg - 2 * hfont);

	drawGlText(x, y, str, 0, 1, 0, 1, GLUT_BITMAP_8_BY_13);
}


void f_aws1_ui::snd_ctrl(s_aws1_ctrl_pars & acpkt)
{
  acpkt.ctrl_src = m_acp.ctrl_src;
  acpkt.rud_aws = m_acp.rud_aws;
  acpkt.meng_aws = m_acp.meng_aws;
  acpkt.seng_aws = m_acp.seng_aws;
  acpkt.suc = true;
  int len;
  if(m_udp_ctrl){
    len = sendto(m_acd_sock, (char*) &acpkt, sizeof(acpkt), 
		 0, (sockaddr*)&m_acd_sock_addr, sizeof(m_acd_sock_addr));
  }else if(m_ch_ctrl_out){
    m_ch_ctrl_out->set_pars(acpkt);
  }
}

void f_aws1_ui::rcv_ctrl(s_aws1_ctrl_pars & acpkt)
{
  acpkt.suc = false;

  if(m_udp_ctrl){
    int res;
    fd_set fr, fe;
    timeval tv;
    
    FD_ZERO(&fr);
    FD_ZERO(&fe);
    FD_SET(m_acd_sock, &fr);
    FD_SET(m_acd_sock, &fe);
    
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    
    res = select((int) m_acd_sock + 1, &fr, NULL, &fe, &tv);
    
    if(res > 0){
      if(FD_ISSET(m_acd_sock, &fr)){
	int len = recv(m_acd_sock, (char*) &acpkt, sizeof(acpkt), 0);
	if(len == SOCKET_ERROR){
	  cerr << "Socket error during recieving packet in " << m_name << "." << endl;
	  acpkt.suc = false;
	}
      }else if(FD_ISSET(m_acd_sock, &fe)){
	cerr << "Socket error during recieving packet in " << m_name << "." << endl;
	acpkt.suc = false;
      }
      
    }else if(res == -1){
      int en = errno;
      cerr << "Error no " << en << " " << strerror(en) << endl;
    }else{
      cerr << "Unknown error in " << m_name << "." << endl;
    }
  }else if(m_ch_ctrl_in){
    m_ch_ctrl_in->get_pars(acpkt);
  }

  if(acpkt.suc){
    m_acp.rud_rmc = acpkt.rud_rmc;
    m_acp.meng_rmc = acpkt.meng_rmc;
    m_acp.seng_rmc = acpkt.seng_rmc;
    m_acp.rud = acpkt.rud;
    m_acp.meng = acpkt.meng;
    m_acp.seng = acpkt.seng;
    m_acp.rud_sta = acpkt.rud_sta;
    m_acp.rud_sta_out = acpkt.rud_sta_out;
    
    m_acp.rud_max_rmc = acpkt.rud_max_rmc;
    m_acp.rud_nut_rmc = acpkt.rud_nut_rmc;
    m_acp.rud_min_rmc = acpkt.rud_min_rmc;
    
    m_acp.meng_max_rmc = acpkt.meng_max_rmc;
    m_acp.meng_nuf_rmc = acpkt.meng_nuf_rmc;
    m_acp.meng_nut_rmc = acpkt.meng_nut_rmc;
    m_acp.meng_nub_rmc = acpkt.meng_nub_rmc;
    m_acp.meng_min_rmc = acpkt.meng_min_rmc;
    
    m_acp.seng_max_rmc = acpkt.seng_max_rmc;
    m_acp.seng_nuf_rmc = acpkt.seng_nuf_rmc;
    m_acp.seng_nut_rmc = acpkt.seng_nut_rmc;
    m_acp.seng_nub_rmc = acpkt.seng_nub_rmc;
    m_acp.seng_min_rmc = acpkt.seng_min_rmc;
    
    m_acp.rud_sta_max = acpkt.rud_sta_max;
    m_acp.rud_sta_nut = acpkt.rud_sta_nut;
    m_acp.rud_sta_min = acpkt.rud_sta_min;

    m_acp.meng_max = acpkt.meng_max;
    m_acp.meng_nuf = acpkt.meng_nuf;
    m_acp.meng_nut = acpkt.meng_nut;
    m_acp.meng_nub = acpkt.meng_nub;
    m_acp.meng_min = acpkt.meng_min;

    m_acp.seng_max = acpkt.seng_max;
    m_acp.seng_nuf = acpkt.seng_nuf;
    m_acp.seng_nut = acpkt.seng_nut;
    m_acp.seng_nub = acpkt.seng_nub;
    m_acp.seng_min = acpkt.seng_min;
    m_acp.rud_max = acpkt.rud_max;
    m_acp.rud_nut = acpkt.rud_nut;
    m_acp.rud_min = acpkt.rud_min;
    
    m_acp.rud_sta_out_max = acpkt.rud_sta_out_max;
    m_acp.rud_sta_out_nut = acpkt.rud_sta_out_nut;
    m_acp.rud_sta_out_min = acpkt.rud_sta_out_min;
  }
}


void f_aws1_ui::_mouse_button_callback(int button, int action, int mods)
{
}

void f_aws1_ui::_key_callback(int key, int scancode, int action, int mods)
{
  if(action == GLFW_PRESS){

    switch(key){
    case GLFW_KEY_RIGHT:
      m_rud_aws_f = step_up(m_acp.rud_aws, m_rud_pos);
      break;
    case GLFW_KEY_LEFT:
      m_rud_aws_f = step_down(m_acp.rud_aws, m_rud_pos);
      break;
    case GLFW_KEY_UP:
      if(m_ec == EC_MAIN){
	m_meng_aws_f = step_up(m_acp.meng_aws, m_meng_pos);
      }else{
	m_seng_aws_f = step_up(m_acp.seng_aws, m_seng_pos);
      }
      break;
    case GLFW_KEY_DOWN:
      if(m_ec == EC_MAIN){
	m_meng_aws_f = step_down(m_acp.meng_aws, m_meng_pos);
      }else{
	m_seng_aws_f = step_down(m_acp.seng_aws, m_seng_pos);
      }
      break;
    case GLFW_KEY_E:
      if(m_ec == EC_MAIN){
	m_ec = EC_SUB;
      }else{
	m_ec = EC_MAIN;
      }
    default:
      break;
    }
  }
}



/////////////////////////////////////////////////////////////////////////// f_aws1_ui_test members

f_aws1_ui_test::f_aws1_ui_test(const char * name):f_base(name),
	m_state(NULL),
	m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL), m_ch_ctrl_out(NULL),
	r(0), p(0), y(0), lon(0), lat(0), alt(0), galt(0), cog(0), sog(0), depth(0),
	m_rud_sta_sim(0.f)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_ctrl_ui", (ch_base**)&m_ch_ctrl_ui, typeid(ch_aws1_ctrl).name(), "Control input channel.");
  register_fpar("ch_ctrl_ap1", (ch_base**)&m_ch_ctrl_ap1, typeid(ch_aws1_ctrl).name(), "Autopilot 1 control input channel.");
  register_fpar("ch_ctrl_ap2", (ch_base**)&m_ch_ctrl_ap2, typeid(ch_aws1_ctrl).name(), "Autopilot 2 control input channel.");
  register_fpar("ch_ctrl_out", (ch_base**)&m_ch_ctrl_out, typeid(ch_aws1_ctrl).name(), "Control output channel.");
  register_fpar("ch_img", (ch_base**)&m_ch_img, typeid(ch_image_ref).name(), "Image channel");	

  // for m_state
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
  register_fpar("awsrud", &m_acp.rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_acp.meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_acp.seng_aws, "Control value of AWS1's sub engine.");

  // remote controller's control parameters (Read Only)
  register_fpar("rmcrud", &m_acp.rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_acp.meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_acp.seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_acp.rud_sta, "Rudder Status of AWS1's.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_acp.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_acp.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_acp.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_acp.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_acp.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_acp.meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_acp.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_acp.meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_acp.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_acp.meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_acp.seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_acp.seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_acp.seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_acp.seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_acp.seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  // Each control points of the sub engine output
  register_fpar("seng_max", &m_acp.seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_acp.seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_acp.seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_acp.seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_acp.seng_min, "Minimum control value for AWS1's sub engine.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_acp.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_acp.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_acp.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_acp.rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_acp.rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_acp.rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_acp.rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_acp.rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_acp.rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  // Control points as the rudder indicator output.
  register_fpar("rud_sta_out_max", &m_acp.rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_acp.rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_acp.rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

  register_fpar("meng", &m_acp.meng, "Output value for main engine.");
  register_fpar("seng", &m_acp.meng, "Output value for sub engine.");
  register_fpar("rud", &m_acp.rud, "Output value for rudder.");
  register_fpar("rud_sta_out", &m_acp.rud_sta_out, "Output value for rudder status.");


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
	return true;
}
