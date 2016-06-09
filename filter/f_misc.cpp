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

////////////////////////////////////////////////////////// f_stereo_disp members
f_stereo_disp::f_stereo_disp(const char * name): f_misc(name), m_ch_img1(NULL), m_ch_img2(NULL),
	m_ch_disp(NULL), m_bflipx(false), m_bflipy(false), m_bnew(false), m_bsync(false),
	m_bpl(false), m_bpr(false), m_bstp(false), m_brct(false)
{
	// channels
	register_fpar("ch_caml", (ch_base**)&m_ch_img1, typeid(ch_image_ref).name(), "Left camera channel");
	register_fpar("ch_camr", (ch_base**)&m_ch_img2, typeid(ch_image_ref).name(), "Right camera channel");
	register_fpar("ch_disp", (ch_base**)&m_ch_disp, typeid(ch_image_ref).name(), "Disparity image channel."); 

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

	if(!m_bsync)
		return true;

	if(m_bpl && m_bpr && m_bstp && !m_brct){
		if(m_brct = rectify_stereo()){
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

	if(m_sgbm_par.m_bsg)
		m_sgbm->compute(m_img1, m_img2, m_disp);
	else
		m_bm->compute(m_img1, m_img2, m_disp);

	m_ch_disp->set_img(m_disp, m_timg1, m_ifrm1);

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
		fisheye::initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_32FC1, m_mapl1, m_mapl2);
		fisheye::initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_32FC1, m_mapr1, m_mapr2);
	}else{
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistMat().clone();
		stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr, 
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, CALIB_ZERO_DISPARITY, -1);
		initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_32FC1, m_mapl1, m_mapl2);
		initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_32FC1, m_mapr1, m_mapr2);		
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
