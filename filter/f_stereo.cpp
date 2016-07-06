#include "stdafx.h"
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_stereo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_stereo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_stereo.  If not, see <http://www.gnu.org/licenses/>. 

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

#include "f_stereo.h"

////////////////////////////////////////////////////////// f_stereo members
const char * f_stereo::m_str_out[IMG2 + 1] = {
	"disp", "img1", "img2"
};

f_stereo::f_stereo(const char * name) : f_base(name), m_ch_img1(NULL), m_ch_img2(NULL),
m_ch_disp(NULL), m_bflipx(false), m_bflipy(false), m_bnew(false), m_bsync(false),
m_bpl(false), m_bpr(false), m_bstp(false), m_brct(false),
m_timg1(-1), m_timg2(-1), m_ifrm1(-1), m_ifrm2(-1), m_ifrm_diff(0), m_fm_max_count(300), m_fm_count(0),
m_fm_time_min_dfrm(0), m_fm_time_min(INT_MAX), m_out(DISP)
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

	// Obstacle detection related parameter
	register_fpar("rgnd", &m_odt_par.drange, "Disparity range for obstacle detection.");
	register_fpar("rnw", &m_odt_par.bb_min_n.width, "Minimum bounding box width for obstacle detection in near field");
	register_fpar("rnh", &m_odt_par.bb_min_n.height, "Minimum bounding box height for obstacle detection in near field");
	register_fpar("rfw", &m_odt_par.bb_min_f.width, "Minimum bounding box width for obstacle detection in far field");
	register_fpar("rfh", &m_odt_par.bb_min_f.height, "Minimum bounding box height for obstacle detection in far field");
	register_fpar("rdn", &m_odt_par.dn, "Nearest disparity for obstacle detection");
	register_fpar("rdf", &m_odt_par.df, "Farest disparity for obstacle detection");

}

f_stereo::~f_stereo()
{
}


bool f_stereo::init_run()
{
	if (!m_ch_img1)
	{
		cerr << "Please set img1 channel." << endl;
		return false;
	}

	if (!m_ch_img2)
	{
		cerr << "Please set img2 channel." << endl;
		return false;
	}

	if (!m_ch_disp)
	{
		cerr << "Please set diparity image channel." << endl;
		return false;
	}

	if (!(m_bpl = m_camparl.read(m_fcpl))){
		cerr << "Failed to load camera1's intrinsic parameter." << endl;
		return false;
	}

	if (!(m_bpr = m_camparr.read(m_fcpr))){
		cerr << "Failed to load camera2's instrinsic parameter." << endl;
		return false;
	}

	if (!(m_bstp = load_stereo_pars())){
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

void f_stereo::destroy_run()
{

}

bool f_stereo::proc()
{
	long long timg1, timg2, ifrm1, ifrm2;
	Mat img1 = m_ch_img1->get_img(timg1, ifrm1);
	Mat img2 = m_ch_img2->get_img(timg2, ifrm2);

	if (img1.empty() || img2.empty())
		return true;

	if (m_timg1 != timg1){
		m_img1 = img1.clone();
		m_timg1 = timg1;
		m_ifrm1 = ifrm1;
		m_bnew = true;
	}

	if (m_timg2 != timg2){
		m_img2 = img2.clone();
		m_timg2 = timg2;
		m_ifrm2 = ifrm2;
		m_bnew = true;
	}

	if (ifrm1 != ifrm2 + m_ifrm_diff){
		m_bsync = false;
		int fdiff = (int)(ifrm1 - ifrm2);
		int tdiff = (int)abs(timg1 - timg2);
		if (tdiff < m_fm_time_min){
			m_fm_time_min = tdiff;
			m_fm_time_min_dfrm = fdiff;
		}
		m_fm_count++;
	}
	else{
		m_bsync = true;
		m_fm_count = 0;
		m_fm_time_min_dfrm = 0;
		m_fm_time_min = INT_MAX;
	}

	if (m_fm_count > m_fm_max_count){
		m_ifrm_diff = m_fm_time_min_dfrm;
		cout << "Frame mismatch is fixed. fdiff = " << m_ifrm_diff << " tdiff = " << m_fm_time_min << endl;
		m_fm_count = 0;
		m_fm_time_min_dfrm = 0;
		m_fm_time_min = INT_MAX;
	}

	if (!m_bsync || !m_bnew)
		return true;

	if (m_bpl && m_bpr && m_bstp && !m_brct){
		if (!(m_brct = rectify_stereo())){
			cerr << "Failed to rectify stereo images." << endl;
			return false;
		}
	}

	if (m_sgbm_par.m_update){
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
	if (m_sgbm_par.m_bsg)
		m_sgbm->compute(m_img1, m_img2, disps16);
	else
		m_bm->compute(m_img1, m_img2, disps16);

	calc_obst(m_odt_par, disps16, m_obst);

	if (m_ch_disp){
		disps16.convertTo(m_disp, CV_8U, 255 / (m_sgbm_par.numDisparities * 16.));

		switch (m_out){
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
	}

	m_bsync = false;
	m_bnew = false;
	return true;
}

bool f_stereo::load_stereo_pars()
{
	if (!m_fstp[0]){
		cerr << "Please specify file of stereo parameters." << endl;
		return false;
	}
	FileStorage fs(m_fstp, FileStorage::READ);
	if (!fs.isOpened()){
		cerr << "Failed to open file " << m_fstp << endl;
		return false;
	}

	FileNode fn;

	fn = fs["Rlr"];
	if (fn.empty())
		return false;
	fn >> m_Rlr;

	fn = fs["Tlr"];
	if (fn.empty())
		return false;
	fn >> m_Tlr;

	fn = fs["E"];
	if (fn.empty())
		return false;
	fn >> m_E;

	fn = fs["F"];
	if (fn.empty())
		return false;
	fn >> m_F;

	fn = fs["Q"];
	if (fn.empty())
		return false;
	fn >> m_Q;

	return true;
}

bool f_stereo::rectify_stereo()
{
	if (!m_bpl || !m_bpr || !m_bstp)
		return false;
	if (m_img1.empty() || m_img2.empty())
		return false;

	Size sz(m_img1.cols, m_img1.rows);
	Mat Kl, Dl, Kr, Dr;
	if (m_camparl.isFishEye()){
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistFishEyeMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistFishEyeMat().clone();
		fisheye::stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr,
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, 0);
		fisheye::initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_16SC2, m_mapl1, m_mapl2);
		fisheye::initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_16SC2, m_mapr1, m_mapr2);
	}
	else{
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
