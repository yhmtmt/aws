#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_imgshk.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_imgshk.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_imgshk.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_imgshk.h"

bool f_imgshk::grab(Mat & img)
{
	// translation
	m_W.at<double>(0,2) = nrand(0., m_S.at<double>(0, 2));
	m_W.at<double>(1,2) = nrand(0., m_S.at<double>(1, 2));
	if(m_wt == EWT_AFN || m_wt == EWT_HMG){
		// scale and rotation
		m_W.at<double>(0,0) = nrand(1., m_S.at<double>(0, 0));
		m_W.at<double>(1,1) = nrand(1., m_S.at<double>(1, 1));
		m_W.at<double>(0,1) = nrand(0, m_S.at<double>(0, 1));
		m_W.at<double>(1,0) = nrand(0, m_S.at<double>(1, 0));
	}

	if(m_wt == EWT_RGD){
		double theta = nrand(0, m_S.at<double>(0, 0));
		double c = cos(theta);
		double s = sin(theta);
		m_W.at<double>(0,0) = c;
		m_W.at<double>(1,1) = c;
		m_W.at<double>(0,1) = -s;
		m_W.at<double>(1,0) = s;			
	}

	if(m_wt == EWT_HMG){
		m_W.at<double>(2,0) = nrand(0, m_S.at<double>(2, 0));
		m_W.at<double>(2,1) = nrand(0, m_S.at<double>(2, 1));
	}

	//		cout << " org W = " << m_W << endl;
	warpPerspective(m_img, img, m_W, Size(m_img.cols, m_img.rows));
	return true;
}


bool f_imgshk::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "open") == 0){
		itok++;
		if(num_args != 4)
			return false;

		return open(args[itok]);
	}else if(strcmp(args[itok], "wt") == 0){
		itok ++;
		if(num_args != 4)
			return false;
		m_wt = get_warp_type(args[itok]);
		return true;
	}else if(strcmp(args[itok], "s") == 0){
		itok ++;
		if(num_args != 11)
			return false;
		double * ptr = m_S.ptr<double>(0);
		for(int i = 0; i < 8; i++, itok++){
			ptr[i] = atof(args[itok]);
		}
	}

	return f_cam::cmd_proc(cmd);
}
