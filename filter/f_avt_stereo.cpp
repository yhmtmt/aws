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

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;
#include "f_avt_stereo.h"

f_avt_stereo::f_avt_stereo(const char * name): f_avt_cam(name), m_cam1(1), m_cam2(2), szorg(-1,-1), szrct(-1,-1), rectify(false)
{
	register_fpar(m_strParams[4], (int*)&m_FrameStartTriggerMode, efstmUndef, strFrameStartTriggerMode, "Frame Start Trigger mode (default Freerun)");
	register_params(m_cam1);
	register_params(m_cam2);
	register_fpar("rc", &rectify, "Enable rectification.");
	frt[0] = '\0';
	register_fpar("frt", frt, 1024, "Path to the file Rotation and translation matrix represents 2nd camera attitude relative to the 1st one.");
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

	if(rectify){
		if(!at.read(frt))
			rectify = false;
		else{
			m_cam1.bundist = false;
			m_cam2.bundist = false;
		}
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

	if(rectify){
		if(!m_cam1.cp.read(m_cam1.fcp)){
			rectify = false;
		}

		if(!m_cam2.cp.read(m_cam2.fcp)){
			rectify = false;
		}
	}

	if(m_cam1.cp.isFishEye() != m_cam2.cp.isFishEye()){ // both camera should have the same camera model.
		rectify = false;
	}

	if(rectify){
		Mat P1, P2;
		if(m_cam1.cp.isFishEye() && m_cam2.cp.isFishEye()){
			if(szrct.width != -1){
				fisheye::stereoRectify(
					m_cam1.cp.getCvPrjMat(), m_cam1.cp.getCvDistFishEyeMat(),
					m_cam2.cp.getCvPrjMat(), m_cam2.cp.getCvDistFishEyeMat(), 
					szorg, at.getRmtx(), at.getT(), m_cam1.R, m_cam2.R, P1, P2, 
					Q, CV_CALIB_ZERO_DISPARITY, szrct); 
				fisheye::initUndistortRectifyMap(P1, m_cam1.cp.getCvDistFishEyeMat(),
					m_cam1.R, m_cam1.Pud, szorg, CV_16SC2, m_cam1.udmap1, m_cam1.udmap2);
				fisheye::initUndistortRectifyMap(P2, m_cam2.cp.getCvDistFishEyeMat(),
					m_cam2.R, m_cam2.Pud, szorg, CV_16SC2, m_cam2.udmap1, m_cam2.udmap2);
			}else{
				fisheye::stereoRectify(
					m_cam1.cp.getCvPrjMat(), m_cam1.cp.getCvDistFishEyeMat(),
					m_cam2.cp.getCvPrjMat(), m_cam2.cp.getCvDistFishEyeMat(), 
					szorg, at.getRmtx(), at.getT(), m_cam1.R, m_cam2.R, P1, P2, 
					Q, CV_CALIB_ZERO_DISPARITY); 
				fisheye::initUndistortRectifyMap(P1, m_cam1.cp.getCvDistFishEyeMat(),
					m_cam1.R, m_cam1.Pud, szrct, CV_16SC2, m_cam1.udmap1, m_cam1.udmap2);
				fisheye::initUndistortRectifyMap(P2, m_cam2.cp.getCvDistFishEyeMat(),
					m_cam2.R, m_cam2.Pud, szrct, CV_16SC2, m_cam2.udmap1, m_cam2.udmap2);
			}
		}else{
			if(szrct.width != -1){
				stereoRectify(
					m_cam1.cp.getCvPrjMat(), m_cam1.cp.getCvDistFishEyeMat(),
					m_cam2.cp.getCvPrjMat(), m_cam2.cp.getCvDistFishEyeMat(), 
					szorg, at.getRmtx(), at.getT(), m_cam1.R, m_cam2.R, P1, P2, 
					Q); 
				initUndistortRectifyMap(P1, m_cam1.cp.getCvDistMat(),
					m_cam1.R, m_cam1.Pud, szorg, CV_16SC2, m_cam1.udmap1, m_cam1.udmap2);
				initUndistortRectifyMap(P2, m_cam2.cp.getCvDistMat(),
					m_cam2.R, m_cam2.Pud, szorg, CV_16SC2, m_cam2.udmap1, m_cam2.udmap2);
			}else{
				stereoRectify(
					m_cam1.cp.getCvPrjMat(), m_cam1.cp.getCvDistFishEyeMat(),
					m_cam2.cp.getCvPrjMat(), m_cam2.cp.getCvDistFishEyeMat(), 
					szorg, at.getRmtx(), at.getT(), m_cam1.R, m_cam2.R, P1, P2, 
					Q, CV_CALIB_ZERO_DISPARITY, -1.0, szrct); 
				initUndistortRectifyMap(P1, m_cam1.cp.getCvDistMat(),
					m_cam1.R, m_cam1.Pud, szrct, CV_16SC2, m_cam1.udmap1, m_cam1.udmap2);
				initUndistortRectifyMap(P2, m_cam2.cp.getCvDistMat(),
					m_cam2.R, m_cam2.Pud, szrct, CV_16SC2, m_cam2.udmap1, m_cam2.udmap2);
			}
			m_cam1.bundist = true;
			m_cam2.bundist = true;
		}		
	}

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
	  cout << "The first camera parameters updated." << endl;
		m_cam1.config_param_dynamic();
		m_cam1.m_update = false;
	}

	if(m_cam2.m_update){
	  cout << "The second camera parameters updated." << endl;
		m_cam2.config_param_dynamic();
		m_cam2.m_update = false;
	}

	if(m_cam1.m_FrameStartTriggerMode == efstmSoftware && 
		m_ttrig_prev + m_ttrig_int < get_time()){
		PvCommandRun(m_cam1.m_hcam, "FrameStartTriggerSoftware");
		PvCommandRun(m_cam2.m_hcam, "FrameStartTriggerSoftware");
		m_ttrig_prev = get_time();
	}
	return true;
}

#endif
