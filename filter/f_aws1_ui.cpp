
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
  set_sock_addr(m_acd_sock_addr, m_acd_host);

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
  s_aws1_ctrl_pkt acpkt;
  snd_ctrl(acpkt);
  rcv_state(acpkt);

  // render graphics
  glfwMakeContextCurrent(pwin());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glRasterPos2i(-1, -1);
  
  float hfont = (float)(24. / (float) m_sz_win.height);
  float wfont = (float)(24. / (float) m_sz_win.width);
  float x = (float)(wfont - 1);
  float y = (float)(1 - 2 * hfont);

  // show time
  drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  float rud_inst, rud_inst_cur;
  float meng_inst, meng_inst_cur;
  float seng_inst, seng_inst_cur;
  float rud_sta;

  rud_inst = (float)m_acp.m_rud_aws;
  rud_inst_cur = 
    (float)map_oval(m_acp.m_rud, 
		    m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min,
		    0xff, 0x7f, 0x00);
  meng_inst = (float)m_acp.m_meng_aws;
  meng_inst_cur = 
    (float)map_oval(m_acp.m_meng,
		    m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, 
		    m_acp.meng_nub, m_acp.meng_min,
		    0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
  
  seng_inst = (float)m_acp.m_seng_aws;
  seng_inst_cur = 
    (float) map_oval(m_acp.m_seng,
		     m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, 
		     m_acp.seng_nub, m_acp.seng_min,
		     0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
  
  rud_sta = 
    (float) map_oval(m_acp.m_rud_sta, 
		     m_acp.rud_sta_max, m_acp.rud_sta_nut, m_acp.rud_sta_min,
		     0xff, 0x7f, 0x00);
  
  x = (float)(wfont - 1.0);
  y = (float)(1.0 - 4 * hfont);  
  
  drawGlText(x, y, "M/E", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  float wm, hm, wl;

  // drawing indicator box (only boundary)
  wm = 3 * wfont;
  hm = (float)(255.0 / (double)m_sz_win.height);
  wl = 1.0 / m_sz_win.width;

  y += 1.5 * hfont;
  float x1, y1, x2, y2;
  x1 = x;
  y1 = y;
  x2 = (float) (x + wm);
  y2 = (float) (y - hm);
  
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1, wl);
  
  // drawing meng_inst_cur indicator 
  wm = 2 * wfont;
  y1 = y2 + meng_inst_cur / (double) m_sz_win.height;
  drawGlSquare2Df(x1, y1, x2, y2, 0, 1, 0, 1);

  x += 4 * wfont;
  drawGlText(x, y, "S/E", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  x = (float)(0. - 3 * wfont);  
  y = (float) (1.0 - 6 * hfont);
  drawGlText(x, y, "RUDDER", 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

  
  
  glfwSwapBuffers(pwin());
  glfwPollEvents();
  
  return false;
}

void f_aws1_ui::snd_ctrl(s_aws1_ctrl_pkt & acpkt)
{
  acpkt.aws_ctrl = m_aws_ctrl;
  acpkt.ctrl_src = m_ctrl_src;
  acpkt.rud_aws = m_rud_aws;
  acpkt.meng_aws = m_meng_aws;
  acpkt.seng_aws = m_segn_aws;

  int len = send(m_acd_sock, (char*) &acpkt, sizeof(acpkt), 0);
}

void f_aws1_ui::rcv_state(s_aws1_ctrl_pkt & acpkt)
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
