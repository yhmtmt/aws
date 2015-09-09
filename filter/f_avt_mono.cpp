// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_mono.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_mono.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_mono.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#ifdef AVT_CAM
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <list>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"
#include "../util/util.h"
#include "f_avt_mono.h"

f_avt_mono::f_avt_mono(const char * name): f_avt_cam(name), m_cam(-1)
{
	register_params(m_cam);
}

f_avt_mono::~f_avt_mono()
{
}


bool f_avt_mono::init_run()
{
	if(m_chout.size() < 1){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CH);
		return false;
	}
	return m_cam.init(this, m_chout[0]);
}

void f_avt_mono::destroy_run()
{
	m_cam.destroy(this);
}


bool f_avt_mono::proc()
{
	// if any, reconfigure camera
	if(m_cam.m_update){
		m_cam.config_param_dynamic();
		m_cam.m_update = false;
	}

	if(m_cam.m_FrameStartTriggerMode == efstmSoftware && 
		m_ttrig_prev + m_ttrig_int > m_cur_time){
		PvCommandRun(m_cam.m_hcam, "FrameStartTriggerSoftware");
		m_ttrig_prev = m_cur_time;
	}

	return true;
}

#endif
