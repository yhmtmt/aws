// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_stereo.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_stereo.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_stereo.  If not, see <http://www.gnu.org/licenses/>. 

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
#include "f_base.h"
#include "f_avt_stereo.h"

f_avt_stereo::f_avt_stereo(const char * name): f_avt_cam(name), m_cam1(1), m_cam2(2)
{
	register_fpar(m_strParams[3], (int*)&m_FrameStartTriggerMode, efstmUndef, strFrameStartTriggerMode, "Frame Start Trigger mode (default Freerun)");
	register_params(m_cam1);
	register_params(m_cam2);
}

f_avt_stereo::~f_avt_stereo()
{
}


bool f_avt_stereo::init_run()
{
	int m_size_buf = 0;

	if(m_chout.size() < 1){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CH);
		return false;
	}

	m_cam1.m_FrameStartTriggerMode = m_FrameStartTriggerMode;

	if(!m_cam1.init(this, m_chout[0]))
		return false;

	if(m_chout.size() < 2){
		f_base::send_err(this, __FILE__, __LINE__, FERR_AVT_CAM_CH);
		return false;
	}

	m_cam2.m_FrameStartTriggerMode = m_FrameStartTriggerMode;

	if(!m_cam2.init(this, m_chout[1]))
		return false;
	return true;
}

void f_avt_stereo::destroy_run()
{
	m_cam1.destroy(this);
	m_cam2.destroy(this);
}


bool f_avt_stereo::proc()
{
	// if any, reconfigure camera
	if(m_cam1.m_update){
		m_cam1.config_param_dynamic();
		m_cam1.m_update = false;
	}

	if(m_cam2.m_update){
		m_cam2.config_param_dynamic();
		m_cam2.m_update = false;
	}

	if(m_cam1.m_FrameStartTriggerMode == efstmSoftware && 
		m_ttrig_prev + m_ttrig_int > m_cur_time){
		PvCommandRun(m_cam1.m_hcam, "FrameTriggerSoftware");
		PvCommandRun(m_cam2.m_hcam, "FrameTriggerSoftware");
		m_ttrig_prev = m_cur_time;
	}
	return true;
}

#endif
