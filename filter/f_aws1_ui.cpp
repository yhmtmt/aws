
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

f_aws1_ui::f_aws1_ui(const char * name): f_glfw_window(name), m_acd_sock(-1), m_acd_port(20100)
{
  register_fpar("acdhost", m_acd_host, 1023, "Host address controlling AWS1.");
  register_fpar("acdport", &m_acd_port, "Port number opened for controlling AWS1.");

}


f_aws1_ui::~f_aws1_ui()
{
}


bool f_aws1_ui::init_run()
{
  m_acd_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_acd_sock == -1){
    cerr << "Failed to create control socket in " << m_name << "." << endl;
    return false;
  }

  m_acd_sock_addr.sin_family = AF_INET;
  m_acd_sock_addr.sin_port = htons(m_acd_port);
  set_sockaddr_addr(m_acd_sock_addr, m_acd_host);

  if(::bind(m_acd_sock, (sockaddr*)&m_acd_sock_addr, sizeof(m_acd_sock_addr)) == SOCKET_ERROR){
    cerr << "Failed to bind control socket to (" << m_acd_host << "," << m_acd_port << ")" << endl;
    return false;
  }
  
  return true;
}

void f_aws1_ui::destroy_run()
{
  closesocket(m_acd_sock);
}

bool f_aws1_ui::proc()
{
  s_aws1_ctrl_pars acpkt;
  snd_ctrl(acpkt);
  rcv_state(acpkt);

  // render graphics
  glfwMakeContextCurrent(pwin());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glRasterPos2i(-1, -1);
  double hscale = 1.0 / (double) m_sz_win.height;
  double vscale = 1.0 / (double) m_sz_win.width;  
  float hfont = (float)(24. * hscale);
  float wfont = (float)(24. * vscale);
  float x = (float)(wfont - 1);
  float y = (float)(1 - 2 * hfont);

  // show time
  drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

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
  
  float wm, hm, lw;

  // Drawing engine control indicator 
  wm = 3 * wfont;
  hm = (float)(255.0 / (double)m_sz_win.height);
  lw = 1.0 / m_sz_win.width;

  x = (float)(wfont - 1.0);
  y = (float)(1.0 - 4 * hfont);
  drawGlEngineIndicator("M/E", x, y, wm, hm, wscale, hscale, lw, 
			(float)(meng_inst * hscale),
			(float)(meng_inst_cur * hscale));
 
  x += (float)(5 * wfont);
  drawGlEngineIndicator("S/E", x, y, wm, hm, wscale, hscale, lw, 
			(float)(seng_inst * hscale),
			(float)(seng_inst_cur * hscale));
  

  // Drawing rudder control indicator
  wm = (float)(255. * hscale);
  hm = (float)(3 * hfont);
  x =  0. - 255. * 0.5 * scale;
  drawGlRudderIndicator("RUDDER", 
			x, y, 
			wm, vm, lw, 
			(float)(rud_inst * vscale),
			(float)(rud_inst_cur * vscale),
			(float)(rud_sta * vscale));

  glfwSwapBuffers(pwin());
  glfwPollEvents();
  
  return false;
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
  xtxt = x2 + 0.1 * w;  

  // draw title
  drawGlText(xorg, (float)(yorg + 1.5 * hfont), title, 
	     0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  // indicator box
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);

  // current value
  x2 = (float)(x1 + 0.666 * w);
  y1 = (float)(y2 + val_cur);
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // current instructed value
  y1 = (float) (y2 + val_inst);
  drawGlLine2Df(x1, y1, x2, y1, 0, 1, 0, 1, lw);

  // neutral
  float dhfont = -0.5 * h;
  x1 = x2;
  x2 = (float)(x1 + w);
  y1 = y2 = (float)(0.5 * h + yorg);
  drawGlLine2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y1 + dhfont), 
	     "N", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  // neutral forward
  y2 = (float)(y1 + (25. / 255.) * h);
  drawGlLine2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y2 - dhfont), 
	     "F", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);


  // neutral backward
  y2 = (float)(y1 - (25. / 255.) * h);
  drawGlLine2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);
  drawGlText(xtxt, (float)(y2 - dhfont), 
	     "B", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
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
	     0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
  
  x1 = xorg;
  x2 = (float)(xorg + w);
  y1 = yorg;
  y2 = (float)(yorg - h);
  
  ytxt = (float) (y2 - 0.1 * h);

  // indicator box
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, lw);

  float xc;
  xc = (float)(xorg + 0.5 * w);

  // rudder instruction value
  x2 = rud_inst;
  drawGlLine2Df(x2, y1, x2, y2, 0, 1, 0, 1, lw);

  // current rudder instruction value
  x1 = xc;
  x2 = rud_cur;
  y2 = (float)(yorg - 0.666 * h);
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // current rudder angle
  x2 = rud_sta;
  y1 = y2;
  y2 = (float)(yorg - h) ;
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  // Midship
  float dwfont = - 0.5 * wfont;
  drawGlLine2Df(x1, y1, x1, y2, 0, 1, 0, 1, lw);
  drawGlText(x1 + dwfont, ytxt, "0", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
  drawGlText(xorg + dwfont, ytxt, "P", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
  drawGlText((float)(xorg + w + dwfont), ytxt, "S", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
}


void f_aws1_ui::snd_ctrl(s_aws1_ctrl_pars & acpkt)
{
  acpkt.ctrl = m_acp.ctrl;
  acpkt.ctrl_src = m_acp.ctrl_src;
  acpkt.rud_aws = m_acp.rud_aws;
  acpkt.meng_aws = m_acp.meng_aws;
  acpkt.seng_aws = m_acp.seng_aws;

  int len = send(m_acd_sock, (char*) &acpkt, sizeof(acpkt), 0);
}

void f_aws1_ui::rcv_state(s_aws1_ctrl_pars & acpkt)
{
  acpkt.suc = false;

  int res;
  fd_set fr, fe;
  timeval tv;

  FD_ZERO(&fr);
  FD_ZERO(&fe);

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
  
  m_acp = acpkt;
}
