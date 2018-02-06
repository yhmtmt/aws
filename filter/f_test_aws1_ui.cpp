// Copyright(c) 2018 Yohei Matsumoto, All right reserved. 

// f_test_aws1_ui.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_test_aws1_ui.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_test_aws1_ui.cpp.  If not, see <http://www.gnu.org/licenses/>. 

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

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>


#include "../util/aws_glib.h"
#include "f_test_aws1_ui.h"


f_test_aws1_ui::f_test_aws1_ui(const char * name):
  f_base(name),
  m_state(NULL), m_engstate(NULL), m_ch_sys(NULL), m_ch_ctrl_inst(NULL),
  m_ch_ctrl_stat(NULL), m_ch_wp(NULL), m_ch_map(NULL),
  m_ch_obj(NULL), m_ch_ais_obj(NULL), m_ch_obst(NULL),
  m_ch_ap_inst(NULL), m_ch_cam(NULL)
{
    register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_sys", (ch_base**)&m_ch_sys, typeid(ch_aws1_sys).name(), "System property channel");
  register_fpar("ch_engstate", (ch_base**)&m_engstate, typeid(ch_eng_state).name(), "Engine Status channel");
  register_fpar("ch_ctrl_inst", (ch_base**)&m_ch_ctrl_inst, typeid(ch_aws1_ctrl_inst).name(), "Control input channel.");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Control output channel.");
  register_fpar("ch_wp", (ch_base**)&m_ch_wp, typeid(ch_wp).name(), "Waypoint channel");
  register_fpar("ch_map", (ch_base**)&m_ch_map, typeid(ch_map).name(), "Map channel");
  register_fpar("ch_obj", (ch_base**)&m_ch_obj, typeid(ch_obj).name(), "Object channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ch_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel");
  register_fpar("ch_obst", (ch_base**)&m_ch_obst, typeid(ch_obst).name(), "Obstacle channel.");
  register_fpar("ch_ap_inst", (ch_base**)&m_ch_ap_inst, typeid(ch_aws1_ap_inst).name(), "Autopilot instruction channel");

  register_fpar("ch_cam", (ch_base**)&m_ch_cam, typeid(ch_image_ref).name(), "Maincamera Image channel.");
};

f_test_aws1_ui::~f_test_aws1_ui()
{
}

bool f_test_aws1_ui::init_run()
{
  return true;
}

void f_test_aws1_ui::destroy_run()
{
}


bool f_test_aws1_ui::proc()
{
  return true;
}
