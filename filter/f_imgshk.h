// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_imgshk.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_imgshk.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_imgshk.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_IMGSHK_H_
#define _F_IMGSHK_H_

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "../util/c_clock.h"

#include "../util/c_imgalign.h"

#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_cam.h"

class f_imgshk: public f_cam
{
protected:
	Mat m_img;
	Mat m_W, m_S;
	e_warp_type m_wt;
public:
	f_imgshk(const char * name):f_cam(name), m_wt(EWT_TRN)
	{
		srand(0);
		m_S = Mat::zeros(3, 3, CV_64FC1);
		m_W = Mat::eye(3, 3, CV_64FC1);
		m_S.at<double>(0, 0) = 0.01;
		m_S.at<double>(0, 1) = 0.01;
		m_S.at<double>(1, 0) = 0.01;
		m_S.at<double>(1, 1) = 0.01;

		m_S.at<double>(0, 2) = 10;
		m_S.at<double>(1, 2) = 10;
	}

	virtual ~f_imgshk()
	{
		close();
	};

	bool open(char * fname)
	{
		m_img = imread(fname);
		return true;
	}

	virtual void close()
	{
	}

	virtual bool grab(Mat & img);
	virtual bool cmd_proc(s_cmd & cmd);
};

#endif