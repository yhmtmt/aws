#include "stdafx.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_base is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_base is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_base.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;

#define XMD_H
#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_misc.h"

////////////////////////////////////////////////////////// f_bkg_mask members
f_bkg_mask::f_bkg_mask(const char * name): f_misc(name), 
	m_ch_img_in(NULL), m_ch_img_out(NULL), m_ch_mask_out(NULL), m_t(-1), m_ifrm(-1),
	m_alpha(0.0001), m_mth(10), m_mbkgth(0.5), m_bupdate(true)
{
	register_fpar("ch_in", (ch_base**)&m_ch_img_in, typeid(ch_image_ref).name(), "Input image channel");
	register_fpar("ch_out", (ch_base**)&m_ch_img_out, typeid(ch_image_ref).name(), "Output image channel");
	register_fpar("ch_mask_out", (ch_base**)&m_ch_mask_out, typeid(ch_image_ref).name(), "Output image channel");
	register_fpar("fmask", m_fmask, 1024, "File name for saving mask image.");
	register_fpar("alpha", &m_alpha, "Moving average constant.");
	register_fpar("mth", &m_mth, "Motion threashold.");
	register_fpar("bkgth", &m_mbkgth, "Background threashold.");
	register_fpar("update", &m_bupdate, "Update mask.");
}

f_bkg_mask::~f_bkg_mask()
{
}

bool f_bkg_mask::init_run()
{
	if(m_fmask[0]){
		char buf[1024];
		snprintf(buf, 1024, "%s.msk", m_fmask);
		if(!read_raw_img(m_mask, buf)){
			m_bupdate = true;
		}
		if(m_bupdate){
			snprintf(buf, 1024, "%s.ma", m_fmask);
			read_raw_img(m_mavg, buf);
		}
	}
	return true;
}

void f_bkg_mask::destroy_run()
{
	if(m_fmask[0]){
		char buf[1024];
		snprintf(buf, 1024, "%s.msk", m_fmask);
		write_raw_img(m_mask, buf);
		snprintf(buf, 1024, "%s.png", m_fmask);
		imwrite(buf, m_mask);
		if(m_bupdate){
			snprintf(buf, 1024, "%s.ma", m_fmask);
			write_raw_img(m_mavg, buf);
		}
		Mat img;
		cnv32FC1to8UC1(m_mavg, img);
		snprintf(buf, 1024, "%s_ma.png", m_fmask);
		imwrite(buf, img);
	}
}

bool f_bkg_mask::proc()
{
	if(m_ch_img_in){
		long long t = 0, ifrm = 0;
		Mat img = m_ch_img_in->get_img(t, ifrm);

		if(m_t == t || img.empty()){
			return true;
		}

		m_t = t;
		m_ifrm = ifrm;

		m_img = img.clone();

		if(m_bupdate){
			if(m_mask.empty()){
				m_mask = Mat::zeros(m_img.rows, m_img.cols, CV_8UC1);
				m_mask = 255;
			}
			if(m_mavg.empty()){
				m_mavg = Mat::zeros(m_img.rows, m_img.cols, CV_32FC1);
				m_mavg = 1.0;
			}

			if(!m_img_prev.empty()){
				switch(m_img.type()){
				case CV_16UC1:
					calc_mask_16uc1();
					break;
				case CV_16UC3:
					calc_mask_16uc3();
					break;
				case CV_8UC1:
					calc_mask_8uc1();
					break;
				case CV_8UC3:
					calc_mask_8uc3();
					break;
				}
			}

			if(!m_img.empty()){
				m_img_prev = m_img;
			}
		}
	}

	if(m_ch_mask_out){
		if(!m_mask.empty()){
			m_ch_img_out->set_img(m_mask, get_time());
		}
	}

	if(m_ch_img_out){
		if(!m_img.empty() && !m_mask.empty()){
			Mat img_m(m_img.rows, m_img.cols, m_img.type());
			switch(m_img.type()){
			case CV_16UC1:
				aply_mask_16uc1(img_m);
				break;
			case CV_16UC3:
				aply_mask_16uc3(img_m);
				break;
			case CV_8UC1:
				aply_mask_8uc1(img_m);
				break;
			case CV_8UC3:
				aply_mask_8uc3(img_m);
				break;
			}

			m_ch_img_out->set_img(img_m, m_t);
		}
	}

	return true;
}

void f_bkg_mask::calc_mask_16uc1()
{
	MatIterator_<ushort> itr = m_img.begin<ushort>();
	MatIterator_<ushort> itr_prev = m_img_prev.begin<ushort>();
	MatIterator_<float> itr_ma = m_mavg.begin<float>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	double ialpha = 1.0 - m_alpha;
	for(;itr != m_img.end<ushort>(); itr++, itr_prev++, itr_ma++, mitr++){
		int diff = abs((int)*itr - (int)*itr_prev);
		if(diff > m_mth){
			*itr_ma = (float)(ialpha * *itr_ma + m_alpha);
		}else{
			*itr_ma = (float)(ialpha * *itr_ma);
		}
		if(*itr_ma > m_mbkgth){
			*mitr = 255;
		}else{
			*mitr = 0;
		}
	}
}

void f_bkg_mask::calc_mask_16uc3()
{
	MatIterator_<Vec3w> itr = m_img.begin<Vec3w>();
	MatIterator_<Vec3w> itr_prev = m_img_prev.begin<Vec3w>();
	MatIterator_<float> itr_ma = m_mavg.begin<float>();
	MatIterator_<Vec3w> mitr = m_mask.begin<Vec3w>();
	double ialpha = 1.0 - m_alpha;
	for(;itr != m_img.end<Vec3w>(); itr++, itr_prev++, itr_ma++, mitr++){
		int diff = 
			abs((int)(*itr)[0] - (int)(*itr_prev)[0]) + 
			abs((int)(*itr)[1] - (int)(*itr_prev)[1]) +
			abs((int)(*itr)[2] - (int)(*itr_prev)[2]);
		if(diff > m_mth){
			*itr_ma = (float)(ialpha * *itr_ma + m_alpha);
		}else{
			*itr_ma = (float)(ialpha * *itr_ma);
		}
		if(*itr_ma > m_mbkgth){
			*mitr = 255;
		}else{
			*mitr = 0;
		}
	}
}

void f_bkg_mask::calc_mask_8uc1()
{
	MatIterator_<uchar> itr = m_img.begin<uchar>();
	MatIterator_<uchar> itr_prev = m_img_prev.begin<uchar>();
	MatIterator_<float> itr_ma = m_mavg.begin<float>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	double ialpha = 1.0 - m_alpha;
	for(;itr != m_img.end<uchar>(); itr++, itr_prev++, itr_ma++, mitr++){
		int diff = abs((int)*itr - (int)*itr_prev);
		float ma = *itr_ma;
		if(diff > m_mth){
			ma = (float)(ialpha * ma + m_alpha);
		}else{
			ma = (float)(ialpha * ma);
		}
		if(ma > m_mbkgth){
			*mitr = 255;
		}else{
			*mitr = 0;
		}
		*itr_ma = ma;
	}
}

void f_bkg_mask::calc_mask_8uc3()
{
	MatIterator_<Vec3b> itr = m_img.begin<Vec3b>();
	MatIterator_<Vec3b> itr_prev = m_img_prev.begin<Vec3b>();
	MatIterator_<float> itr_ma = m_mavg.begin<float>();
	MatIterator_<Vec3b> mitr = m_mask.begin<Vec3b>();
	double ialpha = 1.0 - m_alpha;
	for(;itr != m_img.end<Vec3b>(); itr++, itr_prev++, itr_ma++, mitr++){
		int diff = 
			abs((int)(*itr)[0] - (int)(*itr_prev)[0]) + 
			abs((int)(*itr)[1] - (int)(*itr_prev)[1]) +
			abs((int)(*itr)[2] - (int)(*itr_prev)[2]);
		if(diff > m_mth){
			*itr_ma = (float)(ialpha * *itr_ma + m_alpha);
		}else{
			*itr_ma = (float)(ialpha * *itr_ma);
		}
		if(*itr_ma > m_mbkgth){
			*mitr = 255;
		}else{
			*mitr = 0;
		}
	}
}

void f_bkg_mask::aply_mask_8uc1(Mat & img)
{
	MatIterator_<uchar> itr_org = m_img.begin<uchar>();
	MatIterator_<uchar> itr = img.begin<uchar>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	for(; itr != img.end<uchar>(); itr++, mitr++, itr_org++){
		*itr = *itr_org & *mitr;
	}
}

void f_bkg_mask::aply_mask_8uc3(Mat & img)
{
	MatIterator_<Vec3b> itr_org = m_img.begin<Vec3b>();
	MatIterator_<Vec3b> itr = img.begin<Vec3b>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	for(; itr != img.end<Vec3b>(); itr++, mitr++, itr_org++){
		(*itr)[0] = (*itr_org)[0] & *mitr;
		(*itr)[1] = (*itr_org)[1] & *mitr;
		(*itr)[2] = (*itr_org)[2] & *mitr;
	}
}

void f_bkg_mask::aply_mask_16uc1(Mat & img)
{
	MatIterator_<ushort> itr_org = m_img.begin<ushort>();
	MatIterator_<ushort> itr = img.begin<ushort>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	for(; itr != img.end<ushort>(); itr++, mitr++, itr_org++){
		ushort mask = *mitr;
		mask = (mask << 16) + *mitr;
		*itr = *itr_org & mask;
	}
}

void f_bkg_mask::aply_mask_16uc3(Mat & img)
{
	MatIterator_<Vec3w> itr_org = m_img.begin<Vec3w>();
	MatIterator_<Vec3w> itr = img.begin<Vec3w>();
	MatIterator_<uchar> mitr = m_mask.begin<uchar>();
	for(; itr != img.end<Vec3w>(); itr++, mitr++, itr_org++){
		ushort mask = *mitr;
		mask = (mask << 16) + *mitr;
		(*itr)[0] = (*itr_org)[0] & mask;
		(*itr)[1] = (*itr_org)[1] & mask;
		(*itr)[2] = (*itr_org)[2] & mask;
	}
}

////////////////////////////////////////////////////////// f_lcc members
const char * f_lcc::m_str_alg[UNDEF] =
{
	"rad", "full", "mm", "quad"
};

f_lcc::f_lcc(const char * name): f_misc(name), m_ch_img_in(NULL), m_ch_img_out(NULL), m_alg(FULL),
	m_alpha(0.01), m_range(3.0), m_bias(1.0), m_sigma(0),m_qs(0.00001), m_qb(0.00001), m_depth(14),
	m_update_map(true), m_bpass(false)
{
	m_fmap[0] = '\0';

	register_fpar("ch_in", (ch_base**)&m_ch_img_in, typeid(ch_image_ref).name(), "Input image channel");
	register_fpar("ch_out", (ch_base**)&m_ch_img_out, typeid(ch_image_ref).name(), "Output image channel");
	
	register_fpar("fcp", m_fcp, 1024, "File of camera parameter.");
	register_fpar("pass", &m_bpass, "Pass through.");
	register_fpar("flipx", &m_flipx, "Flip in x.");
	register_fpar("flipy", &m_flipy, "Flip in y.");
	register_fpar("alg", (int*)&m_alg, UNDEF, m_str_alg, "Algorithm.");
	register_fpar("depth", &m_depth, "Color depth. Only for 16bit image.");
	register_fpar("alpha", &m_alpha, "Averaging coeffecient.");
	register_fpar("range", &m_range, "Clipping range.");
	register_fpar("bias", &m_bias, "Bias of the color range.");
	register_fpar("sigma", &m_sigma, "Sigma for gaussian blur.");
	register_fpar("qs", &m_qs, "Scale coefficient for quadratic algorithm.");
	register_fpar("qb", &m_qb, "Bias coefficient for quadratic algorith.");
	register_fpar("fmap", m_fmap, 1024, "File path for map.");
	register_fpar("update", &m_update_map, "Update map.");
}

bool f_lcc::init_run()
{
	if(m_fmap[0]){
		switch(m_alg){
		case RAD:
			if(!load_rad_data())
				m_update_map = true;
		case QUAD:
			if(!m_fcp[0] || !m_campar.read(m_fcp)){
				cerr << "Failed to load camear parameter." << endl;
				return false;
			}else{
				double cx, cy;
				cx = m_campar.getCvPrj()[2];
				cy = m_campar.getCvPrj()[3];
				m_cx = (int)(cx + 0.5);
				m_cy = (int)(cy + 0.5);
				m_cx2 = m_cx << 1;
				m_cy2 = m_cy << 1;
			}
			break;
		case FULL:
			if(!load_full_data())
				m_update_map = true;
			break;
		case MM:
			if(!load_mm_data())
				m_update_map = true;
			break;
		}
	}else{
		m_update_map = true;
	}
	return true;
}

void f_lcc::destroy_run()
{
	if(m_update_map && m_fmap[0]){
		switch(m_alg){
		case RAD:
			save_rad_data();
			break;
		case FULL:
			save_full_data();		
			break;
		case MM:
			save_mm_data();
		}
	}
}

bool f_lcc::load_rad_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s.yml", m_fmap);
	int size;
	FileStorage fs(buf, FileStorage::READ);
	if(!fs.isOpened()){
		cerr << "Failed to open " << buf << endl;
		return false;
	}else{
		FileNode fn = fs["LCCR"];
		if(!fn.empty()){
			fn >> size; 
		}
		fn = fs["LCCAMap"];
		if(!fn.empty()){
			fn >> m_amap;
		}

		fn = fs["LCCVMap"];
		if(!fn.empty()){
			fn >> m_vmap;
		}
	}
	return m_amap.size() == size && m_vmap.size() == size;
}

void f_lcc::save_rad_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s.yml", m_fmap);

	FileStorage fs(buf, FileStorage::WRITE);
	fs << "LCCR" << (int) m_amap.size();
	fs << "LCCAMap" << m_amap;
	fs << "LCCVMap" << m_vmap;

	snprintf(buf, 1024, "%s.csv", m_fmap);
	ofstream ofile(buf);
	if(ofile.is_open()){
		for(int i = 0; i < m_amap.size(); i++){
			ofile << i << "," << m_amap[i] << "," << m_vmap[i] << endl;
		}
	}
}

bool f_lcc::load_full_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s_v.raw", m_fmap);
	read_raw_img(m_vimg, buf);
	snprintf(buf, 1024, "%s_a.raw", m_fmap);
	read_raw_img(m_aimg, buf);
	return !m_vimg.empty() && !m_aimg.empty();
}

void f_lcc::save_full_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s_v.raw", m_fmap);
	write_raw_img(m_vimg, buf);				
	snprintf(buf, 1024, "%s_a.raw", m_fmap);
	write_raw_img(m_aimg, buf);

	Mat img;
	snprintf(buf, 1024, "%s_v.png", m_fmap);
	MatIterator_<float> itr = m_vimg.begin<float>();
	for(;itr != m_vimg.end<float>(); itr++){
		*itr = (float) sqrt(*itr);
	}
	cnv32FC1to8UC1(m_vimg, img);
	imwrite(buf, img);
	snprintf(buf, 1024, "%s_a.png", m_fmap);
	cnv32FC1to8UC1(m_aimg, img);
	imwrite(buf, img);
}

bool f_lcc::load_mm_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s_min.raw", m_fmap);
	read_raw_img(m_min, buf);
	snprintf(buf, 1024, "%s_max.raw", m_fmap);
	read_raw_img(m_max, buf);
	return !m_max.empty() && !m_min.empty();
}

void f_lcc::save_mm_data()
{
	char buf[1024];
	snprintf(buf, 1024, "%s_min.raw", m_fmap);
	write_raw_img(m_min, buf);				
	snprintf(buf, 1024, "%s_max.raw", m_fmap);
	write_raw_img(m_max, buf);

	Mat img;
	snprintf(buf, 1024, "%s_min.png", m_fmap);
	switch(m_min.type()){
	case CV_16UC1:
		cnv16UC1to8UC1(m_min, img);
		break;
	}
	imwrite(buf, img);
	snprintf(buf, 1024, "%s_max.png", m_fmap);
	switch(m_max.type()){
	case CV_16UC1:
		cnv16UC1to8UC1(m_max, img);
		break;
	}
	imwrite(buf, img);
}

bool f_lcc::proc()
{
	// updatign image
	{
		long long t, i;
		Mat img = m_ch_img_in->get_img(t, i);

		if(img.empty())
			return true;

		if(m_t >= t){
			return true;
		}

		m_img = img.clone();
		m_t = t;
		m_ifrm = i;
	}
	
	// flip the image if required.
	if(m_flipx || m_flipy){
		awsFlip(m_img, m_flipx, m_flipy, false);
	}

	// Pass through mode
	if(m_bpass){
		Mat img_out;
		switch(m_img.type()){
		case CV_16UC1:
			m_img.convertTo(img_out, CV_8UC1, 255./(double)((1 << 14) - 1));
		//	cnv16UC1to8UC1(m_img, img_out);
			break;
		case CV_16UC3:
			m_img.convertTo(img_out, CV_8UC3, 255./(double)((1 << 14) - 1));
			//cnv16UC3to8UC3(m_img, img_out);
			break;
		case CV_8UC1:
		case CV_8UC3:
			img_out = m_img.clone();
			break;
		}
		m_ch_img_out->set_img(img_out, m_t, m_ifrm);
		return true;
	}

	// allocate and init data structure if needed.
	if(m_update_map){
		switch(m_alg){
		case RAD:
			if(!m_amap.size() || !m_vmap.size())
				init_rad_data();
			break;
		case FULL:
			if(m_aimg.size() != m_img.size() || m_vimg.size() != m_img.size())
				init_full_data();
			break;
		case MM:
			if(m_min.size() != m_img.size() || m_max.size() != m_img.size()
				|| m_min.depth() != m_img.depth() || m_max.depth() != m_img.depth())
				init_mm_data();
			break;
		}
	}
	
	// allocate and calculate map if needed
	if(m_map.empty() || m_map.size() != m_img.size()){
		m_map.create(m_img.rows, m_img.cols, CV_32FC2);
		switch(m_alg){
		case RAD:
			calc_rad_map();
			break;
		case FULL:
			calc_full_map();
			break;
		case MM:
			calc_mm_map();
			break;
		case QUAD:
			switch(m_img.type()){
			case CV_16UC1:
			case CV_16UC3:
				calc_qmap_16u();
				break;
			case CV_8UC1:
			case CV_8UC3:
				calc_qmap_8u();
				break;
			}
			break;
		}
	}

	// updating map
	Mat img_out;
	if(m_update_map){
		Mat img_calc;
		if(m_sigma != 0){
			int w = (int)(m_sigma * 2) | 0x1;
			GaussianBlur(m_img, img_calc, Size(w, w), m_sigma, m_sigma);
		}else{
			img_calc = m_img;
		}
		switch(m_alg){
		case RAD:
			switch(m_img.type()){
			case CV_16UC1:
				calc_avg_and_var_16uc1_rad(img_calc);
				break;
			case CV_8UC1:
				calc_avg_and_var_8uc1_rad(img_calc);
				break;
			case CV_8UC3:
				calc_avg_and_var_16uc3_rad(img_calc);
				break;
			case CV_16UC3:
				calc_avg_and_var_8uc3_rad(img_calc);
				break;
			}
			break;
		case FULL:
			switch(m_img.type()){
			case CV_16UC1:
				calc_avg_and_var_16uc1(img_calc);
				break;
			case CV_8UC1:
				calc_avg_and_var_8uc1(img_calc);
				break;
			case CV_8UC3:
				calc_avg_and_var_16uc3(img_calc);
				break;
			case CV_16UC3:
				calc_avg_and_var_8uc3(img_calc);
				break;
			}
			break;
		case MM:
			switch(m_img.type()){
			case CV_16UC1:
				calc_mm_16uc1(img_calc);
				break;
			case CV_8UC1:
				calc_mm_8uc1(img_calc);
				break;
			case CV_8UC3:
				calc_mm_16uc3(img_calc);
				break;
			case CV_16UC3:
				calc_mm_8uc3(img_calc);
				break;
			}
		case QUAD:
			switch(m_img.type()){
			case CV_16UC1:
			case CV_16UC3:
				calc_qmap_16u();
				break;
			case CV_8UC1:
			case CV_8UC3:
				calc_qmap_8u();
				break;
			}
		}
	}

	// applying filter
	switch(m_img.type()){
	case CV_16UC1:
		img_out.create(m_img.rows, m_img.cols, CV_8UC1);
		filter_16uc1(m_img, img_out);
		break;
	case CV_8UC1:
		img_out.create(m_img.rows, m_img.cols, CV_8UC1);
		filter_8uc1(m_img, img_out);
		break;
	case CV_8UC3:
		img_out.create(m_img.rows, m_img.cols, CV_8UC3);
		filter_8uc3(m_img, img_out);
		break;
	case CV_16UC3:
		img_out.create(m_img.rows, m_img.cols, CV_8UC3);
		filter_16uc3(m_img, img_out);
		break;
	}

	m_ch_img_out->set_img(img_out, m_t, m_ifrm);

	return true;
}

void f_lcc::init_rad_data()
{
	int rmax = 0;
	{
		double ry_max = max(m_img.rows - m_cy, m_cy);
		double rx_max = max(m_img.cols - m_cx, m_cx);
		rmax = (int)(sqrt(ry_max * ry_max + rx_max * rx_max) + 1.);
	}

	float d = (float)(127. / m_range);
	float v = (float)(d * d);
	m_amap.resize(rmax, 127.f);
	m_vmap.resize(rmax, v);
}

void f_lcc::init_full_data()
{
	m_aimg.create(m_img.rows, m_img.cols, CV_32FC1);
	m_vimg.create(m_img.rows, m_img.cols, CV_32FC1);
	float d = (float)(127. / m_range);
	float v = (float)(d * d);
	m_aimg = 127.;
	m_vimg = v;
}

void f_lcc::init_mm_data()
{
	switch(m_img.type()){
	case CV_8UC1:
	case CV_8UC3:	
		m_min.create(m_img.rows, m_img.cols, CV_8UC1);
		m_max.create(m_img.rows, m_img.cols, CV_8UC1);
		m_min = 255;
		m_max = 0;
		break;
	case CV_16UC1:
	case CV_16UC3:
		m_min.create(m_img.rows, m_img.cols, CV_16UC1);
		m_max.create(m_img.rows, m_img.cols, CV_16UC1);
		m_min = USHRT_MAX;
		m_max = 0;
		break;
	}
}

void f_lcc::calc_rad_map()
{
	float * pm = m_map.ptr<float>();
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	double ibias = 2.0 - m_bias;
	for(int y = 0; y < m_map.rows; y++){
		int x2 = cx * cx;
		for(int x = 0; x < m_map.cols; x++){
			int r = (int) (sqrt((double)(x2 + y2)) + 0.5);

			double a, b;
			double d = sqrt(m_vmap[r]);
			double rd = m_range * d;
			a = m_amap[r] - m_bias * rd;
			b = m_amap[r] + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);

			x2 += (x << 1) - cx2 + 1;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_full_map()
{
	float * pm = m_map.ptr<float>();
	float * paimg = m_aimg.ptr<float>();
	float * pvimg = m_vimg.ptr<float>();
	double ibias = 2.0 - m_bias;
	for(int y = 0; y < m_map.rows; y++){
		for(int x = 0; x < m_map.cols; x++){
			double a, b;
			double d = sqrt(*pvimg);
			double rd = m_range * d;
			a = *paimg - m_bias * rd;
			b = *paimg + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
			pvimg++;
			paimg++;
			pm += 2;
		}
	}
}

void f_lcc::calc_mm_map()
{
	switch(m_min.type()){
	case CV_16UC1:
		calc_mm_map_16u();
		break;
	case CV_8UC1:
		calc_mm_map_8u();
		break;
	}
}

void f_lcc::calc_mm_map_16u()
{
	float * pm = m_map.ptr<float>();
	ushort * pmin = m_min.ptr<ushort>();
	ushort * pmax = m_max.ptr<ushort>();
	for(int y = 0; y < m_map.rows; y++){
		for(int x = 0; x < m_map.cols; x++){
			pm[0] = (float)(255. / (float)((int)*pmax - (int)*pmin));
			pm[1] = (float)(-*pmin * pm[0]);
			pm += 2;
			pmin++;
			pmax++;
		}
	}
}

void f_lcc::calc_mm_map_8u()
{
	float * pm = m_map.ptr<float>();
	uchar * pmin = m_min.ptr<uchar>();
	uchar * pmax = m_max.ptr<uchar>();
	for(int y = 0; y < m_map.rows; y++){
		for(int x = 0; x < m_map.cols; x++){
			pm[0] = (float)(255. / (float)((short)*pmax - (short)*pmin));
			pm[1] = (float)(-*pmin * pm[0]);
			pm += 2;
			pmin++;
			pmax++;
		}
	}
}


void f_lcc::calc_avg_and_var_16uc1(Mat & img)
{
	float * paimg = m_aimg.ptr<float>();
	float * pvimg = m_vimg.ptr<float>();
	ushort * pimg = img.ptr<ushort>();
	float * pm = m_map.ptr<float>();
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++, pm+=2, pimg++, paimg++, pvimg++){
			*paimg = (float)(m_alpha * *pimg +  ialpha * *paimg);
			double d = *pimg - *paimg;
			*pvimg = (float)(m_alpha * d * d + ialpha * *pvimg);

			d = sqrt(*pvimg);

			double a, b;
			double rd = m_range * d;
			a = *paimg - m_bias * rd;
			b = *paimg + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
		}
	}
}

void f_lcc::calc_avg_and_var_8uc1(Mat & img)
{
	float * paimg = m_aimg.ptr<float>();
	float * pvimg = m_vimg.ptr<float>();
	uchar * pimg = img.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++, pm+=2, pimg++, paimg++, pvimg++){
			*paimg = (float)(m_alpha * *pimg +  ialpha * *paimg);
			double d = *pimg - *paimg;
			*pvimg = (float)(m_alpha * d * d + ialpha * *pvimg);

			d = sqrt(*pvimg);

			double a, b;
			double rd = m_range * d;
			a = *paimg - m_bias * rd;
			b = *paimg +ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
		}
	}
}

void f_lcc::calc_avg_and_var_16uc3(Mat & img)
{
	float * paimg = m_aimg.ptr<float>();
	float * pvimg = m_vimg.ptr<float>();
	ushort * pimg = img.ptr<ushort>();
	float * pm = m_map.ptr<float>();
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++, pm+=2, pimg+=3, paimg++, pvimg++){
			double p = (double) (pimg[0] + pimg[1] + pimg[2]) * 0.333;
			*paimg = (float)(m_alpha * p +  ialpha * *paimg);
			double d = *pimg - *paimg;
			*pvimg = (float)(m_alpha * d * d + ialpha * *pvimg);

			d = sqrt(*pvimg);

			double a, b;
			double rd = m_range * d;
			a = *paimg - m_bias * rd;
			b = *paimg + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
		}
	}
}

void f_lcc::calc_avg_and_var_8uc3(Mat & img)
{
	float * paimg = m_aimg.ptr<float>();
	float * pvimg = m_vimg.ptr<float>();
	uchar * pimg = img.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++, pm+=2, pimg+=3, paimg++, pvimg++){
			double p = (double) (pimg[0] + pimg[1] + pimg[2]) * 0.333;

			*paimg = (float)(m_alpha * p +  ialpha * *paimg);
			double d = *pimg - *paimg;
			*pvimg = (float)(m_alpha * d * d + ialpha * *pvimg);

			d = sqrt(*pvimg);

			double a, b;
			double rd = m_range * d;
			a = *paimg - m_bias * rd;
			b = *paimg + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
		}
	}
}

void f_lcc::calc_avg_and_var_16uc1_rad(Mat & img)
{
	unsigned short * pimg = img.ptr<unsigned short>();

	float * pm = m_map.ptr<float>();
	int w = img.cols, h = img.rows;
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	for(int y = 0; y < h; y++){			
		int x2 = cx * cx;
		for(int x = 0; x < w; x++){
			int r = (int) (sqrt((double)(x2 + y2)) + 0.5);
			
			m_amap[r] = (float)(m_alpha * *pimg +  ialpha * m_amap[r]);
			double d = *pimg - m_amap[r];
			m_vmap[r] = (float)(m_alpha * d * d + ialpha * m_vmap[r]);

			d = sqrt(m_vmap[r]);

			double a, b;
			double rd = m_range * d;
			a = m_amap[r] - m_bias * rd;
			b = m_amap[r] + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
			x2 += (x << 1) - m_cx2 + 1;
			pimg++;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_avg_and_var_8uc1_rad(Mat & img)
{
	unsigned char * pimg = img.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	int w = img.cols, h = img.rows;
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}

	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	for(int y = 0; y < h; y++){
		int x2 = cx * cx;
		for(int x = 0; x < w; x++){
			int r = (int) (sqrt((double)(x2 + y2)) + 0.5);

			m_amap[r] = (float)(m_alpha * *pimg +  ialpha * m_amap[r]);
			double d = *pimg - m_amap[r];
			m_vmap[r] = (float)(m_alpha * d * d + ialpha * m_vmap[r]);

			d = sqrt(m_vmap[r]);

			double a, b;
			double rd = m_range * d;
			a = m_amap[r] - m_bias * rd;
			b = m_amap[r] + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
			x2 += (x << 1) - cx2 + 1;
			pimg++;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_avg_and_var_16uc3_rad(Mat & img)
{
	unsigned short * pimg = img.ptr<unsigned short>();
	float * pm = m_map.ptr<float>();
	int w = img.cols, h = img.rows;
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	for(int y = 0; y < h; y++){
		int x2 = cx * cx;
		for(int x = 0; x < w; x++){
			double p = (double) (pimg[0] + pimg[1] + pimg[2]) * 0.333;
			int r = (int) (sqrt((double)(x2 + y2)) + 0.5);

			m_amap[r] = (float)(m_alpha * p +  ialpha * m_amap[r]);
			double d = *pimg - m_amap[r];
			m_vmap[r] = (float)(m_alpha * d * d + ialpha * m_vmap[r]);

			d = sqrt(m_vmap[r]);

			double a, b;
			double rd = m_range * d;
			a = m_amap[r] - m_bias * rd;
			b = m_amap[r] + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
			x2 += (x << 1) - cx2 + 1;
			pimg+=3;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_avg_and_var_8uc3_rad(Mat & img)
{
	unsigned char * pimg = img.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	int w = img.cols, h = img.rows;
	double ibias = 2.0 - m_bias;
	double ialpha = 1.0 - m_alpha;
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	for(int y = 0; y < h; y++){
		int x2 = cx * cx;
		for(int x = 0; x < w; x++){
			double p = (double) (pimg[0] + pimg[1] + pimg[2]) * 0.333;
			int r = (int) (sqrt((double)(x2 + y2)) + 0.5);

			m_amap[r] = (float)(m_alpha * p +  ialpha * m_amap[r]);
			double d = *pimg - m_amap[r];
			m_vmap[r] = (float)(m_alpha * d * d + ialpha * m_vmap[r]);

			d = sqrt(m_vmap[r]);

			double a, b;
			double rd = m_range * d;
			a = m_amap[r] - m_bias * rd;
			b = m_amap[r] + ibias * rd;
			pm[0] = (float)(255.0 / (2 * rd));
			pm[1] = (float)(- pm[0] * a);
			x2 += (x << 1) - cx2 + 1;
			pimg+=3;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_mm_16uc1(Mat & img)
{
	ushort * pimg = img.ptr<ushort>();
	ushort * pmax = m_max.ptr<ushort>();
	ushort * pmin = m_min.ptr<ushort>();
	float * pm = m_map.ptr<float>();
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++){
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pmax++;
			pmin++;
			pimg++;
		}
	}
	calc_mm_map_16u();
}

void f_lcc::calc_mm_8uc1(Mat & img)
{
	uchar * pimg = img.ptr<uchar>();
	uchar * pmax = m_max.ptr<uchar>();
	uchar * pmin = m_min.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++){
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pmax++;
			pmin++;
			pimg++;
		}
	}
	calc_mm_map_8u();
}

void f_lcc::calc_mm_16uc3(Mat & img)
{
	ushort * pimg = img.ptr<ushort>();
	ushort * pmax = m_max.ptr<ushort>();
	ushort * pmin = m_min.ptr<ushort>();
	float * pm = m_map.ptr<float>();
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++){
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pimg++;
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pimg++;
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pmax++;
			pmin++;
			pimg++;
		}
	}
	calc_mm_map_16u();
}

void f_lcc::calc_mm_8uc3(Mat & img)
{
	unsigned char * pimg = img.ptr<uchar>();
	uchar * pmax = m_max.ptr<uchar>();
	uchar * pmin = m_min.ptr<uchar>();
	float * pm = m_map.ptr<float>();
	for(int y = 0; y < img.rows; y++){
		for(int x = 0; x < img.cols; x++){
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pimg++;
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pimg++;
			*pmax = max(*pmax, *pimg);
			*pmin = min(*pmin, *pimg);
			pmax++;
			pmin++;
			pimg++;
		}
	}
	calc_mm_map_8u();
}

void f_lcc::calc_qmap_16u()
{
	float * pm = m_map.ptr<float>();
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;
	double sn = (double)UCHAR_MAX / (double) ((1 << m_depth) - 1);
	double qs = m_qs * sn;
	for(int y = 0; y < m_img.rows; y++){
		int x2 = cx * cx;
		for(int x = 0; x < m_img.cols; x++){
			int r2 = x2 + y2;
			pm[0] = (float) (r2 * qs + sn);
			pm[1] = (float) (r2 * m_qb);
			x2 += (x << 1) - cx2 + 1;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::calc_qmap_8u()
{
	int cx = 0, cy = 0;
	m_ch_img_in->get_offset(cx, cy);
	int sx = 0, sy = 0;
	m_ch_img_in->get_sz_sensor(sx, sy);
	if(m_flipx){
	  cx = sx - (m_img.cols + cx); 
	}
	if(m_flipy){
	  cy = sy - (m_img.rows + cy);
	}
	cx = m_cx - cx;
	cy = m_cy - cy;
	int cx2 = cx << 1;
	int cy2 = cy << 1;
	int y2 = cy * cy;

	float * pm = m_map.ptr<float>();
	for(int y = 0; y < m_img.rows; y++){
		int x2 = cx * cx;
		for(int x = 0; x < m_img.cols; x++){
			int r2 = x2 + y2;
			pm[0] = (float) (r2 * m_qs + 1.);
			pm[1] = (float) (r2 * m_qb);
			x2 += (x << 1) - cx2 + 1;
			pm += 2;
		}
		y2 += (y << 1) - cy2 + 1;
	}
}

void f_lcc::filter_16uc1(Mat & in, Mat & out)
{
	unsigned short * pin = in.ptr<unsigned short>();
	unsigned char * pout = out.ptr<unsigned char>();
	int num_pix = in.cols * in.rows;
	float * pm = m_map.ptr<float>();
	for(int i = 0; i < num_pix; i++){
		*pout = saturate_cast<uchar>(pm[0] * *pin + pm[1]);
		pin++;
		pout++;
		pm+=2;
	}
}

void f_lcc::filter_8uc1(Mat & in, Mat & out)
{
	unsigned char * pin = in.ptr<unsigned char>();
	unsigned char * pout = out.ptr<unsigned char>();
	int num_pix = in.cols * in.rows;
	float * pm = m_map.ptr<float>();
	for(int i = 0; i < num_pix; i++){
		*pout = saturate_cast<uchar>(pm[0] * *pin + pm[1]);
		pin++;
		pout++;
		pm+=2;
	}
}

void f_lcc::filter_16uc3(Mat & in, Mat & out)
{
	unsigned short * pin = in.ptr<unsigned short>();
	unsigned char * pout = out.ptr<unsigned char>();
	int num_pix = in.cols * in.rows;
	float * pm = m_map.ptr<float>();
	for(int i = 0; i < num_pix; i++){
		pout[0] = saturate_cast<uchar>(pm[0] * pin[0] + pm[1]);
		pout[1] = saturate_cast<uchar>(pm[0] * pin[1] + pm[1]);
		pout[2] = saturate_cast<uchar>(pm[0] * pin[2] + pm[1]);
		pin+=3;
		pout+=3;
		pm+=2;
	}
}

void f_lcc::filter_8uc3(Mat & in, Mat & out)
{
	unsigned char * pin = in.ptr<unsigned char>();
	unsigned char * pout = out.ptr<unsigned char>();
	int num_pix = in.cols * in.rows;
	float * pm = m_map.ptr<float>();
	for(int i = 0; i < num_pix; i++){
		pout[0] = saturate_cast<uchar>(pm[0] * pin[0] + pm[1]);
		pout[1] = saturate_cast<uchar>(pm[0] * pin[1] + pm[1]);
		pout[2] = saturate_cast<uchar>(pm[0] * pin[2] + pm[1]);
		pin+=3;
		pout+=3;
		pm+=2;
	}
}

////////////////////////////////////////////////////////// f_stereo_disp members
const char * f_stereo_disp::m_str_out[IMG2 + 1] = {
	"disp", "img1", "img2"
};

f_stereo_disp::f_stereo_disp(const char * name): f_misc(name), m_ch_img1(NULL), m_ch_img2(NULL),
	m_ch_disp(NULL), m_bflipx(false), m_bflipy(false), m_bnew(false), m_bsync(false),
	m_bpl(false), m_bpr(false), m_bstp(false), m_brct(false),
	m_timg1(-1), m_timg2(-1), m_ifrm1(-1), m_ifrm2(-1), m_out(DISP)
{
	// channels
	register_fpar("ch_caml", (ch_base**)&m_ch_img1, typeid(ch_image_ref).name(), "Left camera channel");
	register_fpar("ch_camr", (ch_base**)&m_ch_img2, typeid(ch_image_ref).name(), "Right camera channel");
	register_fpar("ch_disp", (ch_base**)&m_ch_disp, typeid(ch_image_ref).name(), "Disparity image channel."); 

	register_fpar("out", (int*)&m_out, IMG2 + 1, m_str_out, "Output image type (for debug)");

	// file path for camera parameters
	register_fpar("fcpl", m_fcpl, 1024, "Camera parameter file of left camera.");
	register_fpar("fcpr", m_fcpr, 1024, "Camera parameter file of right camera.");
	register_fpar("fstp", m_fstp, 1024, "Stereo parameter file.");

	// flip flag
	register_fpar("flipx", &m_bflipx, "Flip image in x");
	register_fpar("flipy", &m_bflipy, "Flip image in y");

	// parameter for stereoSGBM
	register_fpar("update_bm", &m_sgbm_par.m_update, "Update BM parameter.");
	register_fpar("sgbm", &m_sgbm_par.m_bsg, "SGBM enabling flag.");
	register_fpar("minDisparity", &m_sgbm_par.minDisparity, "minDisparity for cv::StereoSGBM");
	register_fpar("numDisparities", &m_sgbm_par.numDisparities, "numDisparities for cv::StereoSGBM");
	register_fpar("blockSize", &m_sgbm_par.blockSize, "blockSize for cv::StereoSGBM");
	register_fpar("P1", &m_sgbm_par.P1, "P1 for cv::StereoSGBM");
	register_fpar("P2", &m_sgbm_par.P2, "P2 for cv::StereoSGBM");
	register_fpar("disp12MaxDiff", &m_sgbm_par.disp12MaxDiff, "disp12maxDiff for cv::StereoSGBM");
	register_fpar("preFilterCap", &m_sgbm_par.preFilterCap, "preFilterCap for cv::StereoSGBM");
	register_fpar("uniquenessRatio", &m_sgbm_par.uniquenessRatio, "uniquenessRatio for cv::StereoSGBM");
	register_fpar("speckleWindowSize", &m_sgbm_par.speckleWindowSize, "speckleWindowSize for cv::StereoSGBM");
	register_fpar("speckleRange", &m_sgbm_par.speckleRange, "speckleWindowSize for cv::StereoSGBM");
	register_fpar("mode", &m_sgbm_par.mode, "mode for cv::StereoSGBM");
}

f_stereo_disp::~f_stereo_disp()
{
}


bool f_stereo_disp::init_run()
{
	if(!m_ch_img1)
	{
		cerr << "Please set img1 channel." << endl;
		return false;
	}

	if(!m_ch_img2)
	{
		cerr << "Please set img2 channel." << endl;
		return false;
	}

	if(!m_ch_disp)
	{
		cerr << "Please set diparity image channel." << endl;
		return false;
	}

	if(!(m_bpl = m_camparl.read(m_fcpl))){
		cerr << "Failed to load camera1's intrinsic parameter." << endl;
		return false;
	}

	if(!(m_bpr = m_camparr.read(m_fcpr))){
		cerr << "Failed to load camera2's instrinsic parameter." << endl;
		return false;
	}

	if(!(m_bstp = load_stereo_pars())){
		cerr << "Failed to load stereo parameters." << endl;
		return false;
	}

	m_sgbm = StereoSGBM::create(m_sgbm_par.minDisparity, m_sgbm_par.numDisparities,
		m_sgbm_par.blockSize, m_sgbm_par.P1, m_sgbm_par.P2, m_sgbm_par.disp12MaxDiff, 
		m_sgbm_par.preFilterCap, m_sgbm_par.uniquenessRatio, m_sgbm_par.speckleWindowSize,
		m_sgbm_par.speckleRange, m_sgbm_par.mode);

	m_bm = StereoBM::create(m_sgbm_par.numDisparities, m_sgbm_par.blockSize);

	return true;
}

void f_stereo_disp::destroy_run()
{

}

bool f_stereo_disp::proc()
{
	long long timg1, timg2, ifrm1, ifrm2;
	Mat img1 = m_ch_img1->get_img(timg1, ifrm1);
	Mat img2 = m_ch_img2->get_img(timg2, ifrm2);

	if(img1.empty() || img2.empty())
		return true;

	if(m_timg1 != timg1){
		m_img1 = img1.clone();
		m_timg1 = timg1;
		m_ifrm1 = ifrm1;
		m_bnew = true;
	}

	if(m_timg2 != timg2){
		m_img2 = img2.clone();
		m_timg2 = timg2;
		m_ifrm2 = ifrm2;
		m_bnew = true;
	}

	if(m_ifrm1 == m_ifrm2)
		m_bsync =  true;

	if(!m_bsync || !m_bnew)
		return true;

	if(m_bpl && m_bpr && m_bstp && !m_brct){
		if(!(m_brct = rectify_stereo())){
			cerr << "Failed to rectify stereo images." << endl;
			return false;
		}
	}
	
	if(m_sgbm_par.m_update){
		m_sgbm->setBlockSize(m_sgbm_par.blockSize);
		m_sgbm->setDisp12MaxDiff(m_sgbm_par.disp12MaxDiff);
		m_sgbm->setMinDisparity(m_sgbm_par.minDisparity);
		m_sgbm->setMode(m_sgbm_par.mode);
		m_sgbm->setNumDisparities(m_sgbm_par.numDisparities);
		m_sgbm->setP1(m_sgbm_par.P1);
		m_sgbm->setP2(m_sgbm_par.P2);
		m_sgbm->setPreFilterCap(m_sgbm_par.preFilterCap);
		m_sgbm->setSpeckleRange(m_sgbm_par.speckleRange);
		m_sgbm->setSpeckleWindowSize(m_sgbm_par.speckleWindowSize);
		m_sgbm->setUniquenessRatio(m_sgbm_par.uniquenessRatio);
		m_sgbm_par.m_update = false;
	}

	awsFlip(m_img1, m_bflipx, m_bflipy, false);
	awsFlip(m_img2, m_bflipx, m_bflipy, false);

	remap(m_img1, m_img1, m_mapl1, m_mapl2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
	remap(m_img2, m_img2, m_mapr1, m_mapr2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));

	Mat disps16;
	if(m_sgbm_par.m_bsg)
		m_sgbm->compute(m_img1, m_img2, disps16);
	else
		m_bm->compute(m_img1, m_img2, disps16);

	disps16.convertTo(m_disp, CV_8U, 255 / (m_sgbm_par.numDisparities * 16.));

	switch(m_out){
	case DISP:
		m_ch_disp->set_img(m_disp, m_timg1, m_ifrm1);
		break;
	case IMG1:
		m_ch_disp->set_img(m_img1, m_timg1, m_ifrm1);
		break;
	case IMG2:
		m_ch_disp->set_img(m_img2, m_timg2, m_ifrm2);
		break;
	}

	m_bsync = false;
	m_bnew = false;
	return true;
}

bool f_stereo_disp::load_stereo_pars()
{
	if(!m_fstp[0]){
		cerr << "Please specify file of stereo parameters." << endl;
		return false;
	}
	FileStorage fs(m_fstp, FileStorage::READ);
	if(!fs.isOpened()){
		cerr << "Failed to open file " << m_fstp << endl;
		return false;
	}

	FileNode fn;

	fn = fs["Rlr"];
	if(fn.empty())
		return false;
	fn >> m_Rlr;

	fn = fs["Tlr"];
	if(fn.empty())
		return false;
	fn >> m_Tlr;

	fn = fs["E"];
	if(fn.empty())
		return false;
	fn >> m_E;

	fn = fs["F"];
	if(fn.empty())
		return false;
	fn >> m_F;

	fn = fs["Q"];
	if(fn.empty())
		return false;
	fn >> m_Q;

	return true;
}

bool f_stereo_disp::rectify_stereo()
{	
	if(!m_bpl || !m_bpr || !m_bstp)
		return false;
	if(m_img1.empty() || m_img2.empty())
		return false;

	Size sz(m_img1.cols, m_img1.rows);
	Mat Kl, Dl, Kr, Dr;
	if(m_camparl.isFishEye()){
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistFishEyeMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistFishEyeMat().clone();
		fisheye::stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr, 
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, 0);
		fisheye::initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_16SC2, m_mapl1, m_mapl2);
		fisheye::initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_16SC2, m_mapr1, m_mapr2);
	}else{
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistMat().clone();
		stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr, 
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, CALIB_ZERO_DISPARITY, -1);
		initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_16SC2, m_mapl1, m_mapl2);
		initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_16SC2, m_mapr1, m_mapr2);		
	}
	return true;
}


////////////////////////////////////////////////////////// f_debayer members
const char * f_debayer::m_strBayer[UNKNOWN] = {
	"BG8", "GB8", "RG8", "GR8", "GR8NN", "GR8Q", "GR8GQ", "GR8DGQ", "BG16", "GB16", "RG16", "GR16"
};

bool f_debayer::proc(){
	long long timg;
	Mat img = m_pin->get_img(timg);
	if(m_timg == timg){
		return true;
	}
	m_timg = timg;
	Mat bgr;

	if(img.empty()){
		return true;
	}

	switch(m_type){
	case BG8:
		cnvBayerBG8ToBGR8(img, bgr);
		break;
	case GB8:
		cnvBayerGB8ToBGR8(img, bgr);
		break;
	case RG8:
		cnvBayerRG8ToBGR8(img, bgr);
		break;
	case GR8:
		cnvBayerGR8ToBGR8(img, bgr);
		break;
	case GR8NN:
		cnvBayerGR8ToBGR8NN(img, bgr);
		break;
	case GR8Q:
		cnvBayerGR8ToBGR8Q(img, bgr);
		break;
	case GR8GQ:
		cnvBayerGR8ToG8Q(img ,bgr);
		break;
	case GR8DGQ:
		cnvBayerGR8ToDG8Q(img, bgr);
		break;
	case BG16:
		cnvBayerBG16ToBGR16(img, bgr);
		break;
	case GB16:
		cnvBayerGB16ToBGR16(img, bgr);
		break;
	case RG16:
		cnvBayerRG16ToBGR16(img, bgr);
		break;
	case GR16:
		cnvBayerGR16ToBGR16(img, bgr);
		break;
	}

	m_pout->set_img(bgr, timg);
	return true;
}

////////////////////////////////////////////////////////// f_imread members
bool f_imread::proc()
{
	char buf[1024];
	bool raw = false;
	if(m_flist.eof()){
		cout << "File is finished." << endl;
		return false;
	}

	m_flist.getline(buf, 1024);
	char * d = NULL;
	char * u = NULL;

	{
		char * p = buf;
		for(;*p != '\0'; p++){
			if(*p == '.') d = p;
			if(*p == '_') u = p;
		}
		if(d == NULL){
			cerr << "The file does not have any extension." << endl;
			return true;
		}
		if(d[1] == 'r' && d[2] == 'a' && d[3] == 'w')
			raw = true;
	}

	Mat img;
	if(raw)
		read_raw_img(img, buf);
	else
		img = imread(buf);

	long long timg = get_time();
	if(u != NULL){
		*d = '\0';
		timg = atoll(u + 1);
	}

	if(m_verb){
		tmex tm;
		gmtimeex(timg / MSEC  + m_time_zone_minute * 60000, tm);
		snprintf(buf, 32, "[%s %s %02d %02d:%02d:%02d.%03d %d] ", 
			getWeekStr(tm.tm_wday), 
			getMonthStr(tm.tm_mon),
			tm.tm_mday,
			tm.tm_hour,
			tm.tm_min,
			tm.tm_sec,
			tm.tm_msec,
			tm.tm_year + 1900);
		cout << timg << "->" << buf << endl;
	}

	m_pout->set_img(img, timg);
	return true;
}


////////////////////////////////////////////////////////// f_imwrite members
const char * f_imwrite::m_strImgType[eitRAW+1] = {
	"tiff", "jpg", "jp2", "png", "raw"
};

bool f_imwrite::proc()
{
	long long timg;
	Mat img = m_pin->get_img(timg);

	if(m_cur_timg == timg || img.empty()) 
		return true;

	m_cur_timg = timg;

	char buf[1024];
	vector<int> param(2);
	snprintf(buf, 1024, "%s/%s_%lld.%s", m_path, m_name, timg, m_strImgType[m_type]);

	switch(m_type){
	case eitJP2:
	case eitTIFF:
		imwrite(buf, img);
		break;
	case eitJPG:
		param[0] = CV_IMWRITE_JPEG_QUALITY;
		param[1] = m_qjpg;
		imwrite(buf, img, param);
		break;
	case eitPNG:
		param[0] = CV_IMWRITE_PNG_COMPRESSION;
		param[1] = m_qpng;
		imwrite(buf, img, param);
		break;
	case eitRAW:
		write_raw_img(img, buf);
		break;
	}
	if(m_verb)
		cout << "Writing " << buf << endl;
	return true;
}

////////////////////////////////////////////////////////// f_gry member
bool f_gry::proc()
{
  ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
  if(pin == NULL)
    return false;
  
  ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
  if(pout == NULL)
    return false;
  
  ch_image * pclrout = dynamic_cast<ch_image*>(m_chout[1]);
  if(pclrout == NULL)
    return false;
  
  long long timg;
  Mat img = pin->get_img(timg);
  if(img.empty())
    return true;
  
  Mat out;
  cvtColor(img, out, CV_BGR2GRAY);
  
  pout->set_img(out, timg);
  pclrout->set_img(img, timg);
  return true;
}

////////////////////////////////////////////////////////// f_gauss members
bool f_gauss::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "sigma") == 0){
		if(num_args != 4)
			return false;
		m_sigma = atof(args[itok+1]);
		return true;
	}

	return f_base::cmd_proc(cmd);
}


bool f_clip::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "rc") == 0){
		if(num_args != 7)
			return false;
		m_rc_clip = Rect(atoi(args[itok+1]),
			atoi(args[itok+2]), atoi(args[itok+3]), atoi(args[itok+4]));

		return true;
	}

	return f_base::cmd_proc(cmd);
}

bool f_bkgsub::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "rc") == 0){
		return true;
	}

	return f_base::cmd_proc(cmd);
	
}
