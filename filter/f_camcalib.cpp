#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_camcalib.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_camcalib.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_camcalib.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

#define XMD_H

#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "f_base.h"
#include "f_camcalib.h"

bool f_camcalib::write_pts(const char * fname)
{
	long long timg;
	Mat img = m_pin->get_img(timg);
	if(img.empty())
		return false;
	Size sz(img.cols, img.rows);

	if(m_2dchsbd.size() == 0)
		return false;

	ofstream file(fname);
	if(!file.is_open()){
		cerr << "Cannot open file " << fname << endl;
		return false;
	}

	m_3dchsbd.resize(m_sz_chsbd.height*m_sz_chsbd.width);
	for(int i= 0; i < m_sz_chsbd.height; i++){
		for(int j = 0; j < m_sz_chsbd.width; j++){
			int ipt = m_sz_chsbd.width * i + j;
			m_3dchsbd[ipt].x = m_pitch_chsbd * i;
			m_3dchsbd[ipt].y = m_pitch_chsbd * j;
			m_3dchsbd[ipt].z = 0;
		}
	}

	vector<vector<Point3f > > chsbd3d;
	for(int i = 0; i < m_2dchsbd.size(); i++)
		chsbd3d.push_back(m_3dchsbd);

	file << "size " << sz.width << " " << sz.height << endl;
	for(int i = 0; i < m_2dchsbd.size(); i++){
		file << "view " << m_2dchsbd[i].size() << endl;
		for(int j = 0; j < m_2dchsbd[i].size(); j++){
			file << m_2dchsbd[i][j].x;
			file << " ";
			file << m_2dchsbd[i][j].y;
			file << " ";
			file << m_3dchsbd[j].x;
			file << " ";
			file << m_3dchsbd[j].y;
			file << " ";
			file << m_3dchsbd[j].z;
			file << endl;
		}
	}

	return true;
}

bool f_camcalib::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;
	int itok = 2;

	if(strcmp(args[itok], "calib") == 0){
		if(!calibrate())
			return false;
		return true;
	}else if(strcmp(args[itok], "save") == 0){
		if(num_args != 5){
			cerr << "save <points file name> <camera parameters yml file name>" << endl;
			return false;
		}

		if(!write_pts(args[itok+1]) || !write_campars(args[itok+2]))
			return false;

		return true;
	}else if(strcmp(args[itok], "cba") == 0){
		if(num_args != 5){
			cerr << "cba <x size> <y size>" << endl;
			return false;
		}

		m_sz_chsbd.width = atoi(args[itok+1]);
		m_sz_chsbd.height = atoi(args[itok+2]);

		return true;
	}else if(strcmp(args[itok], "cbs") == 0){
		if(num_args != 4){
			cerr << "cbs <pitch>" << endl;
			return false;
		}
		m_pitch_chsbd = (float) atof(args[itok+1]);
		return true;
	}

	return f_base::cmd_proc(cmd);
}

bool f_camcalib::proc()
{
	// ch_image * pin = m_pin;
	// if(pin == NULL)
	// 	return false;
	// ch_image * pout = m_pout;
	if(m_pin == NULL)
		return false;
	long long timg;
	Mat img = m_pin->get_img(timg);

	if(img.empty())
		return true;

	vector<Point2f> corners;
	if(!findChessboardCorners(img, m_sz_chsbd, corners)){
		cout << "Cannot find chessboard." << endl;
		return true;
	}

	static int num_det = 0;
	num_det++;
	cout << num_det << "th " 
	     << "Chessboard was detected" << endl;

	cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	m_2dchsbd.push_back(corners);

	for(int i = 0; i < m_2dchsbd.size(); i++){
		drawChessboardCorners(img, m_sz_chsbd, m_2dchsbd[i], true);
	}

	m_pout->set_img(img, timg);
	return true;
}

bool f_camcalib::calibrate()
{
	long long timg;
	Mat img = m_pin->get_img(timg);
	if(m_2dchsbd.empty()){
		cout << "No chessboard found." << endl;
		return false;
	}

	m_3dchsbd.resize(m_sz_chsbd.height*m_sz_chsbd.width);
	for(int i= 0; i < m_sz_chsbd.height; i++){
		for(int j = 0; j < m_sz_chsbd.width; j++){
			int ipt = m_sz_chsbd.width * i + j;
			m_3dchsbd[ipt].x = m_pitch_chsbd * i;
			m_3dchsbd[ipt].y = m_pitch_chsbd * j;
			m_3dchsbd[ipt].z = 0;
		}
	}

	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<vector<Point3f > > chsbd3d;
	for(int i = 0; i < m_2dchsbd.size(); i++)
		chsbd3d.push_back(m_3dchsbd);

	m_err = calibrateCamera(chsbd3d, m_2dchsbd, 
		Size(img.cols, img.rows),
		m_Mcam, m_discoeff, rvecs, tvecs);

	cout << "reprojection error = " << m_err << endl;

	cout << "Camera Matrix" << endl;
	cout << m_Mcam << endl;
	cout << "Distortion Coefficient" << endl;
	cout << m_discoeff << endl;

	return true;
}

bool f_camcalib::write_campars(const char * fname)
{
	char buf[1024];
	snprintf(buf, 1024, "%s", fname);
	FileStorage fs(buf, FileStorage::WRITE);
	if(!fs.isOpened())
		return false;

	//save camera parameters
	fs << "CamInt" << m_Mcam;
	fs << "CamDist" << m_discoeff;
	fs << "RepErr" << m_err;
	fs.release();
}

bool f_camcalib::init_run()
{
	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_chin.size() == 0){
		cerr << m_name << "requires an input channel." << endl;
		return false;
	}
	if(m_pin == NULL){
		cerr << m_name << "'s first input channel should be image channel." << endl;
		return false;
	}
	m_pout = dynamic_cast<ch_image*>(m_chout[0]);
	return true;
}

void f_camcalib::destroy_run()
{
}
