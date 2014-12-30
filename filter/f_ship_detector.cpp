// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ship_detector is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ship_detector is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ship_detector.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

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
#include "../util/thread_util.h"
#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_vector.h"

#include "f_base.h"
#include "f_ship_detector.h"

bool f_ship_detector::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	if(num_args == 2){
		return false;
	}

	int itok = 2;

	if(strcmp(args[itok], "bgroi") == 0){
		if(num_args != 7){
			m_dtctroi.pop_back();
			return true;
		}

		m_smplroi.push_back(
			Rect(atoi(args[itok+1]), atoi(args[itok+2]), 
			atoi(args[itok+3]), atoi(args[itok+4])));

		return true;

	}else if(strcmp(args[itok], "dtroi") == 0){
		if(num_args != 7){
			m_dtctroi.pop_back();
			return true;
		}

		m_dtctroi.push_back(
			Rect(atoi(args[itok+1]), atoi(args[itok+2]), 
			atoi(args[itok+3]), atoi(args[itok+4])));

		return true;
	}else if(strcmp(args[itok], "cdepth") == 0){
		if(num_args != 4)
			return false;
		
		m_num_depth_per_chan = atoi(args[itok+1]);
		m_num_depth_per_chan = min(m_num_depth_per_chan, 8);
		m_num_depth_per_chan = max(m_num_depth_per_chan, 1);
		
		init();
		return true;
	}else if(strcmp(args[itok], "alpha") == 0){
		if(num_args != 4)
			return false;
		m_alpha = atof(args[itok+1]);
		return true;
	}else if(strcmp(args[itok], "th") == 0){
		if(num_args != 4)
			return false;

		m_th = atof(args[itok+1]);
		return true;
	}

	return f_base::cmd_proc(cmd);
}

bool f_ship_detector::proc()
{
	if(!m_smplroi.size())
		return false;
	if(!m_dtctroi.size())
		return false;

	ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pin == NULL)
		return false;

	long long timg;
	Mat img = pin->get_img(timg);

	ch_vector<Rect> * pout = dynamic_cast<ch_vector<Rect> *>(m_chout[0]);
	if(pout == NULL)
		return false;

	if(img.empty())
		return true;

	// zero clear temporal variables
	for(int i = 0; i < m_num_bins_per_chan; i++)
		for(int j = 0; j < m_num_bins_per_chan; j++)
			for(int k = 0; k < m_num_bins_per_chan; k++)
				m_hist_tmp[i][j][k] = 0;

	// sampling color
	m_num_smpls = 0;
	int shift = 8 - m_num_depth_per_chan;
	for(int i = 0; i < m_smplroi.size(); i++)
	{
		int endx  = m_smplroi[i].width + m_smplroi[i].x;
		int endy =  m_smplroi[i].height + m_smplroi[i].y;
		for(int y = m_smplroi[i].y; y < endy; y++){
			unsigned char * pix = img.ptr<unsigned char>(y) + 3 * m_smplroi[i].x;
			for(int x = m_smplroi[i].x; x < endx; x++){
				int ib = *pix >> shift;
				pix++;
				int ig = *pix >> shift;
				pix++;
				int ir = *pix >> shift;
				pix++;

				m_hist_tmp[ir][ig][ib]++;
				m_num_smpls++;
			}
		}
	}

	// model update
	double coeff = m_alpha / (double) m_num_smpls;
	double ialpha = 1.0 - m_alpha;
	
	for(int i = 0; i < m_num_bins_per_chan; i++)
		for(int j = 0; j < m_num_bins_per_chan; j++)
			for(int k = 0; k < m_num_bins_per_chan; k++)
				m_hist[i][j][k] = m_hist[i][j][k] * ialpha 
				+ coeff * (double) m_hist_tmp[i][j][k];

	// color model is to be made sufficient time after initialization
	if(m_wait_cnt < (int) (1.0 / m_alpha)){
		m_wait_cnt++;
		return true;
	}

	m_bin = Mat::zeros(img.rows, img.cols, CV_8U);

	// thresholding
	for(int i = 0; i < m_dtctroi.size(); i++){
		int endx  = m_dtctroi[i].width + m_dtctroi[i].x;
		int endy =  m_dtctroi[i].height + m_dtctroi[i].y;
		for(int y = m_dtctroi[i].y; y < endy; y++){
			unsigned char * pix = img.ptr<unsigned char>(y) + 3 * m_dtctroi[i].x;
			unsigned char * bpix = m_bin.ptr<unsigned char>(y) + m_dtctroi[i].x;
			for(int x = m_dtctroi[i].x; x < endx; x++){

				int ib = *pix >> shift;
				pix++;
				int ig = *pix >> shift;
				pix++;
				int ir = *pix >> shift;
				pix++;

				if(m_hist[ir][ig][ib] < m_th)
					*bpix = 255;
				else
					*bpix = 0;
				bpix++;
			}
		}
	}

	// labeling
	int ilabel = 0;
	int num_pix;
	s_wf wf;
	int xmin, xmax, ymin, ymax;
	m_label = Mat::zeros(img.rows, img.cols, CV_32S);
	for(int i = 0; i < m_dtctroi.size(); i++){
//		cout << "Roi[" << i << "]" << endl;
		int endx  = m_dtctroi[i].width + m_dtctroi[i].x;
		int endy =  m_dtctroi[i].height + m_dtctroi[i].y;
		for(int y = m_dtctroi[i].y; y < endy; y++){
			int * label = m_label.ptr<int>(y) + m_dtctroi[i].x;
			unsigned char * bpix = m_bin.ptr<unsigned char>(y) + m_dtctroi[i].x;
			for(int x = m_dtctroi[i].x; x < endx; x++){
				if(*label || !*bpix){
					label++;
					bpix++;
					continue;
				}

				// breadth first search
				m_sq.push_back(s_wf(x, y, m_sd));

				num_pix = 0;
				xmin = ymin = INT_MAX;
				xmax = ymax = 0;

				while(m_sq.size()){
					wf = m_sq.back();
					m_sq.pop_back();
					num_pix++;
					xmin = min(wf.x, xmin);
					xmax = max(wf.x, xmax);
					ymin = min(wf.y, ymin);
					ymax = max(wf.y, ymax);

					expand(wf.x, wf.y - 1, wf.d, ilabel);
					expand(wf.x, wf.y + 1, wf.d, ilabel);
					expand(wf.x + 1, wf.y, wf.d, ilabel);
					expand(wf.x - 1, wf.y, wf.d, ilabel);
				}

				if(num_pix > m_th_pix){
//					cout << "(" << xmin << "," << ymin << ")-(" 
//						<< xmax << "," << ymax << ")" << endl;
					pout->push(new Rect(xmin, ymin, xmax - xmin, ymax - ymin));
				}

				bpix++;
				label++;
				ilabel++;
			}
		}
	}

#ifdef DEBUG_SD
	for(int i = 0; i < m_smplroi.size(); i++){
		rectangle(img, m_smplroi[i], CV_RGB(255, 0, 0));
	}
	for(int i = 0; i < m_dtctroi.size(); i++){
		rectangle(img, m_dtctroi[i], CV_RGB(0, 0, 255));
	}
	Rect * prc;
	while(prc = m_pout->pop()){
		rectangle(img, *prc, CV_RGB(0, 255, 0));
		delete prc;
	}
#endif

	return true;
}

void f_ship_detector::expand(int x, int y, int d, int ilabel)
{
	if(x >= m_label.cols || x < 0 ||
		y >= m_label.rows || y < 0)
		return;

	if(m_label.at<int>(y, x) == ilabel)
		return;

	m_label.at<int>(y, x) = ilabel;
	if(m_bin.at<unsigned char>(y, x))
		m_sq.push_back(s_wf(x, y, m_sd));
	else if(d != 0)
		m_sq.push_back(s_wf(x, y, d-1));
}