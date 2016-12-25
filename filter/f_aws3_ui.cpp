// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws3_ui.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws3_ui.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws3_ui.cpp.  If not, see <http://www.gnu.org/licenses/>. 

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
#include "f_aws3_ui.h"

f_aws3_ui::f_aws3_ui(const char * name) :f_glfw_window(name), m_ch_param(NULL), m_ch_state(NULL), m_ch_cmd(NULL), m_verb(false),
					 m_js_id(0)
{
	register_fpar("js", &m_js_id, "Joystick id");
	register_fpar("ch_param", (ch_base**)&m_ch_param, typeid(ch_aws3_param).name(), "Channel of AWS3's parameters.");
	register_fpar("ch_state", (ch_base**)&m_ch_state, typeid(ch_aws3_state).name(), "Channel of AWS3 state.");
	register_fpar("ch_cmd", (ch_base**)&m_ch_cmd, typeid(ch_aws3_cmd).name(), "Channel of AWS3 command.");
	register_fpar("verb", &m_verb, "Debug mode.");

}

f_aws3_ui::~f_aws3_ui()
{

}

bool f_aws3_ui::init_run()
{
	if (!m_ch_param){
		cerr << "ch_param is not connected." << endl;
		return false;
	}

	if (!m_ch_state){
		cerr << "ch_state is not connected." << endl;
		return false;
	}

	if (!m_ch_cmd){
		cerr << "ch_cmd is not connected." << endl;
		return false;
	}

	if (!f_glfw_window::init_run())
		return false;

	if (m_js.init(m_js_id)){
	  cout << "Joystick " << m_js.name << " found." << endl;
	}else{
	  cout << "Joystik " << m_js_id << " cannot be found." << endl;
	  return false;
	}

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	return true;
}

void f_aws3_ui::destroy_run()
{
}


bool f_aws3_ui::proc()
{
  m_js.set_stk();
  m_js.set_btn();
	short x = (short)1000 * m_js.lr1;
	short y = (short)1000 * m_js.ud1;
	short z = -(short)1000 * m_js.ud2;
	short r = (short)1000 * m_js.lr2;
	m_ch_cmd->set(x, y, z, r);
	// flt mode 1 manual
	// flt mode 2 stabilized
	// flt mode 3 alt hold
	m_ch_cmd->set(0, m_js.is_state_down(m_js.ex)); // 1 shift 0
	m_ch_cmd->set(1, m_js.is_state_down(m_js.ey)); // 7 mode2 0
	m_ch_cmd->set(2, m_js.is_state_down(m_js.ea)); // 0 0
	m_ch_cmd->set(3, m_js.is_state_down(m_js.eb)); // 6 mode1 0
	m_ch_cmd->set(4, m_js.is_state_down(m_js.elb)); // 32 lights1_brighter 0
	m_ch_cmd->set(5, m_js.is_state_down(m_js.erb)); // 33 lights1_dimmer 0
	m_ch_cmd->set(6, m_js.is_state_down(m_js.elt)); // 0 45 trim_roll_dec 
	m_ch_cmd->set(7, m_js.is_state_down(m_js.ert)); // 0 44 trim_roll_inc
	m_ch_cmd->set(8, m_js.is_state_down(m_js.elst)); // 0 0
	m_ch_cmd->set(9, m_js.is_state_down(m_js.erst)); // 0 0
	m_ch_cmd->set(10, m_js.is_state_down(m_js.eback)); //4 disarm 0 
	m_ch_cmd->set(11, m_js.is_state_down(m_js.estart)); // 3 arm 0
	m_ch_cmd->set(12, m_js.is_state_down(m_js.erx)); // 0 0
	m_ch_cmd->set(13, m_js.is_state_down(m_js.elx)); // 32 lights1_brighter 0
	m_ch_cmd->set(14, m_js.is_state_down(m_js.eux)); // 33 lights1_dimmer 0 
	m_ch_cmd->set(15, m_js.is_state_down(m_js.edx)); // 0  0

	if(m_verb){
	  short x, y, z, r;
	  uint16_t b;
	  m_ch_cmd->get(x, y, z,r);
	  b = m_ch_cmd->get_btn();
	  cout << "Control x:" << x << " y:" << y 
	       << " z:" << z << " r:" << r << endl;
	  printf("Btn:%04x\n", b);
	}

	if(glfwWindowShouldClose(pwin()))
	  return false;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glfwSwapBuffers(pwin());
	glfwPollEvents();
	return true;
}

