#include "stdafx.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_cam.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_cam.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_cam.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_cam.h"

//////////////////////////////////////////////////// class f_cam members

f_cam::f_cam(const char * name):f_base(name), 
		m_intpar(false), m_pout(NULL), m_pcamparout(NULL), m_bstream(false)
{
	register_fpar("bstrm", &m_bstream, "Stream activity status.");
}

f_cam::~f_cam()
{
}

bool f_cam::write_intpar(const char * fname)
{
	if(!m_intpar){
		cout << "Intrinsic camera parameters have not been loaded." << endl;
		return false;

	}
	FileStorage fs(fname, FileStorage::WRITE);
	if(!fs.isOpened()){
		cout << "Failed to open " << fname << endl;
		return false;
	}

	fs << "CamMat" << m_cam_mat;
	fs << "DistCoeff" << m_dist_coeff;
	return true;
}

bool f_cam::read_intpar(const char * fname)
{
	FileStorage fs(fname, FileStorage::READ);
	if(!fs.isOpened()){
		cout << "Failed to open " << fname << endl;
		return false;
	}

	fs["CamMat"] >> m_cam_mat;
	cout << "Camera Matrix" << endl;
	cout << m_cam_mat << endl;

	fs["DistCoeff"] >> m_dist_coeff;
	cout << "Distortion Coefficient" << endl;
	cout << m_dist_coeff << endl;
	m_intpar = true;

	return true;
}

bool f_cam::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;
	int itok = 2;
	if(strcmp(args[itok], "w") == 0)
		return write_intpar(args[itok+1]);
	else if(strcmp(args[itok], "r") == 0)
		return read_intpar(args[itok+1]);

	ch_campar * pcamparout = dynamic_cast<ch_campar*>(m_chout[1]);

	if(strcmp(args[itok], "pos") == 0){

		if(!pcamparout)
			return false;

		if(num_args != 6)
			return false;
		Point3d pos(atof(args[itok+1]),atof(args[itok+2]),atof(args[itok+3]));

		pcamparout->set_pos(pos);
		return true;
	}

	if(strcmp(args[itok], "rot") == 0){
		if(!pcamparout)
			return false;

		if(num_args != 6)
			return false;

		s_rotpar rot;
		rot.roll = atof(args[itok+1]);
		rot.pitch = atof(args[itok+2]);
		rot.yaw = atof(args[itok+3]);

		pcamparout->set_rot(rot);
		return true;
	}

	if(strcmp(args[itok], "Int") == 0){
		if(!pcamparout)
			return false;

		if(num_args != 7){
			return false;
		}
		Mat Int = Mat::eye(3, 3, CV_64FC1);

		Int.at<double>(0, 0) = atof(args[itok+1]);
		Int.at<double>(1, 1) = atof(args[itok+2]);
		Int.at<double>(0, 2) = atof(args[itok+3]);
		Int.at<double>(1, 2) = atof(args[itok+4]);

		pcamparout->set_Int(Int);
		return true;
	}

	return f_base::cmd_proc(cmd);
}


bool f_cam::proc()
{
	ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
	if(pout == NULL)
		return false;

	Mat img;
	m_bstream = grab(img);

	if(!m_bstream)
		return true;

	m_time_shot = m_cur_time;
	pout->set_img(img, m_cur_time);
	return true;
}
