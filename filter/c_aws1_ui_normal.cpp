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

/////////////////////////////////////////////////////////////////////////// c_aws1_ui_normal
void c_aws1_ui_normal::js(const s_jc_u3613m & js)
{
	pui->ui_set_js_ctrl();	
}

void c_aws1_ui_normal::draw(float xscale, float yscale)
{
	pui->ui_show_img();

	pui->ui_show_meng(xscale, yscale);
	pui->ui_show_seng(xscale, yscale);
	pui->ui_show_rudder(xscale, yscale);
	pui->ui_show_state(xscale, yscale);

  // Drawing attitude indicator (w-mark, hdg scale, pitch scale) for main view type 1 only
  // Drawing map information. both for main view type 1 and 2. 
}

void c_aws1_ui_normal::key(int key, int scancode, int action, int mods)
{
	if(action == GLFW_PRESS){
		switch(key){
		case GLFW_KEY_RIGHT:
			{
				const s_aws1_ctrl_pars & acp = pui->ui_get_ctrl_par();
				pui->ui_set_rud_f(step_up(acp.rud_aws, m_rud_pos));
			}
			break;
		case GLFW_KEY_LEFT:
			{
				const s_aws1_ctrl_pars & acp = pui->ui_get_ctrl_par();
				pui->ui_set_rud_f(step_down(acp.rud_aws, m_rud_pos));
			}
			break;
		case GLFW_KEY_UP:
			{
				const s_aws1_ctrl_pars & acp = pui->ui_get_ctrl_par();
				if(m_ec == EC_MAIN){
					pui->ui_set_meng_f(step_up(acp.meng_aws, m_meng_pos));
				}else{
					pui->ui_set_seng_f(step_up(acp.seng_aws, m_seng_pos));
				}
			}
			break;
		case GLFW_KEY_DOWN:
			{
				const s_aws1_ctrl_pars & acp = pui->ui_get_ctrl_par();
				if(m_ec == EC_MAIN){
					pui->ui_set_meng_f(step_down(acp.meng_aws, m_meng_pos));
				}else{
					pui->ui_set_seng_f(step_down(acp.seng_aws, m_seng_pos));
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
