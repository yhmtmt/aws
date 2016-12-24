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

f_aws3_ui::f_aws3_ui(const char * name) :f_glfw_window(name), m_ch_param(NULL), m_ch_state(NULL), m_ch_cmd(NULL),
m_js_id(0)
{
	register_fpar("js", &m_js_id, "Joystick id");
	register_fpar("ch_param", (ch_base**)&m_ch_param, typeid(ch_aws3_param).name(), "Channel of AWS3's parameters.");
	register_fpar("ch_state", (ch_base**)&m_ch_state, typeid(ch_aws3_state).name(), "Channel of AWS3 state.");
	register_fpar("ch_cmd", (ch_base**)&m_ch_cmd, typeid(ch_aws3_cmd).name(), "Channel of AWS3 command.");

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

	if (m_js.init(m_js_id)){
		cout << "Joystick " << m_js.name << " found." << endl;
	}

	return true;
}

void f_aws3_ui::destroy_run()
{
}


bool f_aws3_ui::proc()
{
	return true;
}

