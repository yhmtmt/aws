// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_normal.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_normal.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_normal.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include "stdafx.h"
#include <cstdio>
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

#ifdef _WIN32
#include <Windows.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>


#include "../util/aws_glib.h"
#include "f_aws1_ui.h"

/////////////////////////////////////////////////////////////////////////// c_aws1_ui_normal
void c_aws1_ui_normal::js(const s_jc_u3613m & js)
{
  if(js.id != -1){
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
    m_rud_aws_f += (float)(js.lr1 * (255. / 180.));
    m_rud_aws_f += (float)(js.lr2 * (255. / 180.));
    m_rud_aws_f = min((float)255.0, m_rud_aws_f);
    m_rud_aws_f = max((float)0.0, m_rud_aws_f);

    m_meng_aws_f -= (float)(js.ud1 * (255. / 180));
    m_meng_aws_f = min((float)255.0, m_meng_aws_f);
    m_meng_aws_f = max((float)0.0, m_meng_aws_f);

    m_seng_aws_f -= (float)(js.ud2 * (255. / 180));
    m_seng_aws_f = min((float) 255.0, m_seng_aws_f);
    m_seng_aws_f = max((float)0.0, m_seng_aws_f);    
  }

  s_aws1_ctrl_inst & inst = get_ctrl_inst();
  inst.tcur = get_cur_time();
  inst.rud_aws = (unsigned char) m_rud_aws_f;
  inst.meng_aws = (unsigned char) m_meng_aws_f;
  inst.seng_aws = (unsigned char) m_seng_aws_f;
}

void c_aws1_ui_normal::draw()
{
	pui->ui_show_img();

	pui->ui_show_meng();
	pui->ui_show_seng();
	pui->ui_show_rudder();
	pui->ui_show_state();

  // Drawing attitude indicator (w-mark, hdg scale, pitch scale) for main view type 1 only
  // Drawing map information. both for main view type 1 and 2. 
}

void c_aws1_ui_normal::key(int key, int scancode, int action, int mods)
{
	if(action == GLFW_PRESS){
		switch(key){
		case GLFW_KEY_RIGHT:
			{
				const s_aws1_ctrl_inst & acp = get_ctrl_inst();
				m_rud_aws_f = (step_up(acp.rud_aws, m_rud_pos));
			}
			break;
		case GLFW_KEY_LEFT:
			{
				const s_aws1_ctrl_inst & acp = get_ctrl_inst();
				m_rud_aws_f = (step_down(acp.rud_aws, m_rud_pos));
			}
			break;
		case GLFW_KEY_UP:
			{
				const s_aws1_ctrl_inst & acp = get_ctrl_inst();
				if(m_ec == EC_MAIN){
					m_meng_aws_f = (step_up(acp.meng_aws, m_meng_pos));
				}else{
					m_seng_aws_f = (step_up(acp.seng_aws, m_seng_pos));
				}
			}
			break;
		case GLFW_KEY_DOWN:
			{
				const s_aws1_ctrl_inst & acp = get_ctrl_inst();
				if(m_ec == EC_MAIN){
					m_meng_aws_f = (step_down(acp.meng_aws, m_meng_pos));
				}else{
					m_seng_aws_f = (step_down(acp.seng_aws, m_seng_pos));
				}
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


///////////////////////////////////////////////////// members of c_aws1_ui_core
const long long c_aws1_ui_core::get_cur_time()
{
	return pui->get_time();
}

ch_state * c_aws1_ui_core::get_ch_state()
{
	return pui->m_state;
}

ch_aws1_ctrl_inst * c_aws1_ui_core::get_ch_ctrl_inst()
{
	return pui->m_ch_ctrl_inst;
}

ch_aws1_ctrl_stat * c_aws1_ui_core::get_ch_ctrl_stat()
{
	return pui->m_ch_ctrl_stat;
}

ch_wp * c_aws1_ui_core::get_ch_wp(){
	return pui->m_ch_wp;
}

ch_obj * c_aws1_ui_core::get_ch_obj(){
	return pui->m_ch_obj;
}

ch_ais_obj * c_aws1_ui_core::get_ch_ais_obj()
{
	return pui->m_ch_ais_obj;
}

ch_image * c_aws1_ui_core::get_ch_img(){
	return pui->m_ch_img;
}

ch_map * c_aws1_ui_core::get_ch_map()
{
	return pui->m_ch_map;
}

ch_obst * c_aws1_ui_core::get_ch_obst()
{
	return pui->m_ch_obst;
}

ch_aws1_ap_inst * c_aws1_ui_core::get_ch_ap_inst()
{
	return pui->m_ch_ap_inst;
}

s_aws1_ctrl_inst & c_aws1_ui_core::get_ctrl_inst()
{
	return pui->m_inst;
}

const Size & c_aws1_ui_core::get_window_size()
{
	return pui->m_sz_win;
}

const char * c_aws1_ui_core::get_path_storage()
{
	return pui->m_path_storage;
}

// normal coordinate to pixel coordinate transformation
void c_aws1_ui_core::nml2pix(const float xnml, const float ynml, float & xpix, float & ypix){
	xpix = (float)(xnml * pui->m_xscale);
	ypix = (float)(ynml * pui->m_yscale);
}

// pixel coordinate to normal coordinate transformation
void c_aws1_ui_core::pix2nml(const float xpix, const float ypix, float & xnml, float & ynml){		
	xnml = (float)(xpix * pui->m_ixscale);
	ynml = (float)(ypix * pui->m_iyscale);
}
