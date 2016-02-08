
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

  register_fpar("ctrl", &m_aws_ctrl, "Yes if aws controls AWS1 (default no)");
  register_fpar("acs", (int*) &m_ctrl_src, ACS_NONE, str_aws1_ctrl_src,  "AWS control source.");

  // aws's control parameters
  register_fpar("awsrud", &m_rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_seng_aws, "Control value of AWS1's sub engine.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  // Each control points of the sub engine output
  register_fpar("seng_max", &m_seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_seng_min, "Minimum control value for AWS1's sub engine.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  // Control points as the rudder indicator output.
  register_fpar("rud_sta_out_max", &m_rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");
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
  
  //  m_aws_ctrl = acpkt.ctrl;
  //  m_ctrl_src = acpkt.ctrl_src;
 
  m_rud = acpkt.rud;
  m_meng = acpkt.meng;
  m_seng = acpkt.seng;

  m_rud_rmc = acpkt.rud_rmc;
  m_meng_rmc = acpkt.meng_rmc;
  m_seng_rmc = acpkt.seng_rmc;

  m_rud_sta = acpkt.rud_sta;
  m_rud_sta_out = acpkt.rud_sta_out;
}
