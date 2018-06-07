// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_glfw_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.  If not, see <http://www.gnu.org/licenses/>. 
#include "stdafx.h"
#ifdef GLFW_WINDOW
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <map>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_vobj.h"
#include "../util/aws_vlib.h"

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include "../util/aws_glib.h"
#include "f_glfw_stereo_view.h"


////////////////////////////////////////////////////////////////////////////////f_glfw_stereo_view members
f_glfw_stereo_view::f_glfw_stereo_view(const char * name) : f_glfw_window(name), m_pin1(NULL), m_pin2(NULL),
m_state(NULL),
m_timg1(-1), m_timg2(-1), m_ifrm_diff(0), m_fm_max_count(300), m_fm_count(0), m_fm_time_min_dfrm(0),
m_fm_time_min(INT_MAX),
m_bchsbd(false), m_num_chsbdl(0), m_num_chsbdr(0), m_num_chsbd_com(0), m_bflipx(false), m_bflipy(false),
m_bcpl(false), m_bcpr(false), m_bstp(false), m_brct(false),
m_bsvcp(false), m_bldcp(false), m_budl(false), m_budr(false), m_bdisp(false),
m_bcbl(false), m_bcbr(false), m_bcbst(false),
m_brctst(false), m_bsv_chsbd(false), m_bld_chsbd(false), m_bdet_chsbd(false),
m_bsvstp(false), m_bldstp(false),
m_bupdate_img(false),
m_bfisheye(false), m_bfix_int(false), m_bfix_k1(false), m_bfix_k2(false), m_bfix_k3(false),
m_bfix_k4(false), m_bfix_k5(false), m_bfix_k6(false), m_bguess_int(false), m_bfix_ar(false),
m_bfix_pp(false), m_bzr_tng(false), m_brat_mdl(false), m_bred_chsbd(false), m_num_calib_chsbd(50),
m_bptl(false), m_bptr(false), m_num_com_pts(0),
m_roll0(0), m_pitch0(0), m_yaw0(0), m_dtatt(0), m_iatt(0)
{
	register_fpar("caml", (ch_base**)&m_pin1, typeid(ch_image_ref).name(), "Left camera channel");
	register_fpar("camr", (ch_base**)&m_pin2, typeid(ch_image_ref).name(), "Right camera channel");
	register_fpar("state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");

	m_fcpl[0] = m_fcpr[0] = m_fstp[0] = '\0';
	register_fpar("fcpl", m_fcpl, 1024, "Camera parameter file of left camera.");
	register_fpar("fcpr", m_fcpr, 1024, "Camera parameter file of right camera.");
	register_fpar("fstp", m_fstp, 1024, "Stereo parameter file.");
	register_fpar("udl", &m_budl, "Undistort left camera");
	register_fpar("udr", &m_budr, "Undistort right camera");
	register_fpar("rctst", &m_brctst, "Stereo Rectify");
	register_fpar("disp", &m_bdisp, "Show disparity map.");
	register_fpar("cbl", &m_bcbl, "Calibrate left camera");
	register_fpar("cbr", &m_bcbr, "Calibrate right camera");
	register_fpar("cbst", &m_bcbst, "Calibrate stereo images.");
	register_fpar("sv_chsbd", &m_bsv_chsbd, "Save chessboard.");
	register_fpar("ld_chsbd", &m_bld_chsbd, "Load chessboard.");
	register_fpar("det_chsbd", &m_bdet_chsbd, "Detect chessboard.");
	register_fpar("red_chsbd", &m_bred_chsbd, "Reduce chessboard to calib_chsbds");
	register_fpar("svcp", &m_bsvcp, "Save camera parameter.");
	register_fpar("ldcp", &m_bldcp, "Load camera parameter.");
	register_fpar("svstp", &m_bsvstp, "Save stereo parameter.");
	register_fpar("ldstp", &m_bldstp, "Load stereo parameter.");

	m_fcapture[0] = '\0';
	register_fpar("capture", &m_bcapture, "Capture the screen");
	register_fpar("fcapture", m_fcapture, 1024, "Capture file name");

	register_fpar("flipx", &m_bflipx, "Flip image in x");
	register_fpar("flipy", &m_bflipy, "Flip image in y");

	register_fpar("update_img", &m_bupdate_img, "Update image flag.");
	register_fpar("fchsbd", m_chsbd.fname, 1024, "Chessboard model file");
	m_fchsbdl[0] = m_fchsbdr[0] = m_fchsbdc[0] = '\0';

	register_fpar("fchsbdl", m_fchsbdl, 1024, "Detected chessboard file for left camera.");
	register_fpar("fchsbdr", m_fchsbdr, 1024, "Detected chessboard file for righ camera.");
	register_fpar("fchsbdc", m_fchsbdc, 1024, "Detected common chessboard file for left and right camera.");

	register_fpar("calib_chsbds", &m_num_calib_chsbd, "Number of chessboards used for calibration.");
	register_fpar("guess_int", &m_bguess_int, "Use intrinsic guess.");
	register_fpar("fisheye", &m_bfisheye, "Fisheye model is used.");
	register_fpar("fix_int", &m_bfix_int, "Fix intrinsic parameters");
	register_fpar("fix_pp", &m_bfix_pp, "Fix camera center as specified (cx, cy)");
	register_fpar("fix_aspect_ratio", &m_bfix_ar, "Fix aspect ratio as specified fx/fy. Only fy is optimized.");
	register_fpar("zero_tangent_dist", &m_bzr_tng, "Zeroify tangential distortion (px, py)");
	register_fpar("fix_k1", &m_bfix_k1, "Fix k1 as specified.");
	register_fpar("fix_k2", &m_bfix_k2, "Fix k2 as specified.");
	register_fpar("fix_k3", &m_bfix_k3, "Fix k3 as specified.");
	register_fpar("fix_k4", &m_bfix_k4, "Fix k4 as specified.");
	register_fpar("fix_k5", &m_bfix_k5, "Fix k5 as specified.");
	register_fpar("fix_k6", &m_bfix_k6, "Fix k6 as specified.");
	register_fpar("rat_mdl", &m_brat_mdl, "Enable rational model (k4, k5, k6)");
	register_fpar("fxl", (m_camparl.getCvPrj() + 0), "Focal length in x");
	register_fpar("fyl", (m_camparl.getCvPrj() + 1), "Focal length in y");
	register_fpar("cxl", (m_camparl.getCvPrj() + 2), "Principal point in x");
	register_fpar("cyl", (m_camparl.getCvPrj() + 3), "Principal point in y");
	register_fpar("k1l", (m_camparl.getCvDist() + 0), "k1 for left camera");
	register_fpar("k2l", (m_camparl.getCvDist() + 1), "k2 for left camera");
	register_fpar("p1l", (m_camparl.getCvDist() + 2), "p1 for left camera");
	register_fpar("p2l", (m_camparl.getCvDist() + 3), "p2 for left camera");
	register_fpar("k3l", (m_camparl.getCvDist() + 4), "k3 for left camera");
	register_fpar("k4l", (m_camparl.getCvDist() + 5), "k4 for left camera");
	register_fpar("k5l", (m_camparl.getCvDist() + 6), "k5 for left camera");
	register_fpar("k6l", (m_camparl.getCvDist() + 7), "k6 for left camera");
	register_fpar("k1fl", (m_camparl.getCvDistFishEye() + 0), "k1 for left camera with fisheye");
	register_fpar("k2fl", (m_camparl.getCvDistFishEye() + 1), "k2 for left camera with fisheye");
	register_fpar("k3fl", (m_camparl.getCvDistFishEye() + 2), "k3 for left camera with fisheye");
	register_fpar("k4fl", (m_camparl.getCvDistFishEye() + 3), "k4 for left camera with fisheye");

	register_fpar("fxr", (m_camparr.getCvPrj() + 0), "Focal length in x");
	register_fpar("fyr", (m_camparr.getCvPrj() + 1), "Focal length in y");
	register_fpar("cxr", (m_camparr.getCvPrj() + 2), "Principal point in x");
	register_fpar("cyr", (m_camparr.getCvPrj() + 3), "Principal point in y");
	register_fpar("k1r", (m_camparr.getCvDist() + 0), "k1 for right camera");
	register_fpar("k2r", (m_camparr.getCvDist() + 1), "k2 for right camera");
	register_fpar("p1r", (m_camparr.getCvDist() + 2), "p1 for right camera");
	register_fpar("p2r", (m_camparr.getCvDist() + 3), "p2 for right camera");
	register_fpar("k3r", (m_camparr.getCvDist() + 4), "k3 for right camera");
	register_fpar("k4r", (m_camparr.getCvDist() + 5), "k4 for right camera");
	register_fpar("k5r", (m_camparr.getCvDist() + 6), "k5 for right camera");
	register_fpar("k6r", (m_camparr.getCvDist() + 7), "k6 for right camera");
	register_fpar("k1fr", (m_camparr.getCvDistFishEye() + 0), "k1 for right camera with fisheye");
	register_fpar("k2fr", (m_camparr.getCvDistFishEye() + 1), "k2 for right camera with fisheye");
	register_fpar("k3fr", (m_camparr.getCvDistFishEye() + 2), "k3 for right camera with fisheye");
	register_fpar("k4fr", (m_camparr.getCvDistFishEye() + 3), "k4 for right camera with fisheye");

	// parameter for stereoSGBM
	register_fpar("update_sgbm", &m_sgbm_par.m_update, "Update SGBM parameter.");
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

	// AHRS related parameter
	register_fpar("dtatt", &m_dtatt, "Attitude Time difference against image.");
	register_fpar("roll0", &m_roll0, "Static roll value.");
	register_fpar("pitch0", &m_pitch0, "Static pitch value.");
	register_fpar("yaw0", &m_yaw0, "Static yaw value.");

	// Obstacle detection related parameter
	register_fpar("rgnd", &m_odt_par.drange, "Disparity range for obstacle detection.");
	register_fpar("rnw", &m_odt_par.bb_min_n.width, "Minimum bounding box width for obstacle detection in near field");
	register_fpar("rnh", &m_odt_par.bb_min_n.height, "Minimum bounding box height for obstacle detection in near field");
	register_fpar("rfw", &m_odt_par.bb_min_f.width, "Minimum bounding box width for obstacle detection in far field");
	register_fpar("rfh", &m_odt_par.bb_min_f.height, "Minimum bounding box height for obstacle detection in far field");
	register_fpar("rdn", &m_odt_par.dn, "Nearest disparity for obstacle detection");
	register_fpar("rdf", &m_odt_par.df, "Farest disparity for obstacle detection");
	m_Rl = Mat::eye(3, 3, CV_64FC1);
	m_Rr = Mat::eye(3, 3, CV_64FC1);

	m_pts_chsbdr.reserve(1000);
	m_pts_chsbdl.reserve(1000);
	m_ifrm_chsbdr.reserve(1000);
	m_ifrm_chsbdl.reserve(1000);
	m_pts_chsbdr_com.reserve(1000);
	m_pts_chsbdl_com.reserve(1000);
	m_ifrm_chsbd_com.reserve(1000);

	for (int iatt = 0; iatt < 0; iatt++){
		m_tatt[iatt] = -1;
		m_roll[iatt] = m_pitch[iatt] = m_yaw[iatt] = 0.0;
	}
}

bool f_glfw_stereo_view::proc()
{
	glfwMakeContextCurrent(pwin());

	if (glfwWindowShouldClose(pwin()))
		return false;

	long long timg1, timg2, ifrm1, ifrm2;

	Mat img1, img2;
	if(m_pin1)
	  img1 = m_pin1->get_img(timg1, ifrm1);
	else
	  cerr << "Image channel 1 is empty." << endl;

	if(m_pin2)
	  img2 = m_pin2->get_img(timg2, ifrm2);
	else 
	  cerr << "Image channel 2 is empty." << endl;

	if (img1.empty() || img2.empty()){
		cerr << "One of the image channels are empty." << endl;
		return true;
	}

	if (img1.type() != img2.type()){
		cerr << "Two image channels in " << m_name << " have different image type." << endl;
		return false;
	}

	m_bnew = false;
	if (m_timg1 != timg1){
		m_timg1 = timg1;
		m_ifrm1 = ifrm1;
		m_img1 = img1.clone();
		awsFlip(m_img1, m_bflipx, m_bflipy, false);
		if (m_budl && m_bcpl){
			remap(m_img1, m_img1, m_mapl1, m_mapl2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
		}

		m_bnew = true;
	}


	if (m_timg2 != timg2){
		m_timg2 = timg2;
		m_ifrm2 = ifrm2;
		m_img2 = img2.clone();
		awsFlip(m_img2, m_bflipx, m_bflipy, false);
		if (m_budr && m_bcpr){
			remap(m_img2, m_img2, m_mapr1, m_mapr2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
		}
		m_bnew = true;
	}

	if (m_ifrm1 != m_ifrm2 + m_ifrm_diff){
		if (m_bnew){
			int fdiff = (int)(m_ifrm1 - m_ifrm2);
			int tdiff = (int)abs(timg1 - timg2);
			if (tdiff < m_fm_time_min){
				m_fm_time_min = tdiff;
				m_fm_time_min_dfrm = fdiff;
			}
			m_fm_count++;
		}
	}
	else{
		m_fm_count = 0;
		m_fm_time_min_dfrm = 0;
		m_fm_time_min = INT_MAX;
	}

	if (m_timg1 == m_timg2){
		m_bsync = true;
	}
	else{
		m_bsync = false;
	}

	if (m_fm_count > m_fm_max_count){
		m_ifrm_diff = m_fm_time_min_dfrm;
		cout << "Frame mismatch is fixed. fdiff = " << m_ifrm_diff << " tdiff = " << m_fm_time_min << endl;
		m_fm_count = 0;
		m_fm_time_min_dfrm = 0;
		m_fm_time_min = INT_MAX;
	}

	if (m_bnew && m_bsync)
		m_bupdate_img = true;

	if (m_num_chsbdl && m_bcbl){ //calibrate left camera
		calibrate(0);
		m_bcbl = false;
	}

	if (m_num_chsbdr && m_bcbr){ // calibrate right camera
		calibrate(1);
		m_bcbr = false;
	}

	if (m_bcpl && m_bcpr && m_bcbst){ // stereo calibrate
		calibrate_stereo();
	}

	if (m_bcpl && m_bcpr && m_bstp && m_brctst){
		rectify_stereo();
	}

	if (m_bldcp){
		m_bcpl = m_camparl.read(m_fcpl);
		Size sz(m_img1.cols, m_img1.rows);
		m_Rl = Mat::eye(3, 3, CV_64FC1);
		m_Pl = m_camparl.getCvPrjMat().clone();
		init_undistort(m_camparl, sz, m_Rl, m_Pl, m_mapl1, m_mapl2);
		m_bcpr = m_camparr.read(m_fcpr);
		sz.width = m_img2.cols;
		sz.height = m_img2.rows;
		m_Rr = Mat::eye(3, 3, CV_64FC1);
		m_Pr = m_camparr.getCvPrjMat().clone();
		init_undistort(m_camparr, sz, m_Rr, m_Pr, m_mapr1, m_mapr2);
		m_bldcp = false;
	}

	if (m_bsvcp){
		m_bcpr = m_camparl.write(m_fcpl);
		m_bcpr = m_camparr.write(m_fcpr);
		m_bsvcp = false;
	}

	if (m_bred_chsbd){ // reduce chessboard less than m_num_calib_chsbd
		reduce_chsbd(0);
		reduce_chsbd(1);
		reduce_chsbd(2);
		m_bred_chsbd = false;
	}

	if (m_bchsbd && m_bdet_chsbd && m_bsync && m_bupdate_img){
		s_obj obj;
		bool left = false, right = false;
		// seraching chessboard detected in the frame
		for (int i = 0; i < m_ifrm_chsbdl.size(); i++){
			if (m_ifrm_chsbdl[i] == m_ifrm1){
				left = true;
			}
		}
		for (int i = 0; i < m_ifrm_chsbdr.size(); i++){
			if (m_ifrm_chsbdr[i] == m_ifrm2){
				right = true;
			}
		}

		if (!left || !right){
			if (!left && m_chsbd.detect(m_img1, &obj)){
				m_pts_chsbdl.push_back(obj.pt2d);
				m_ifrm_chsbdl.push_back(ifrm1);
				m_num_chsbdl++;
				cout << "CamL frm[" << ifrm1 << "] chsbd[" << m_num_chsbdl << "] found." << endl;
				left = true;
			}

			if (!right && m_chsbd.detect(m_img2, &obj)){
				m_pts_chsbdr.push_back(obj.pt2d);
				m_ifrm_chsbdr.push_back(ifrm2);
				cout << "CamR frm[" << ifrm2 << "] chsbd[" << m_num_chsbdr << "] found." << endl;
				m_num_chsbdr++;
				right = true;
			}

			if (left && right){ // common chessboard		  
				cout << "frm[" << ifrm1 << "] common chsbd[" << m_num_chsbd_com << "] found." << endl;
				m_pts_chsbdr_com.push_back(m_pts_chsbdr[m_num_chsbdr - 1]);
				m_pts_chsbdl_com.push_back(m_pts_chsbdl[m_num_chsbdl - 1]);
				m_ifrm_chsbd_com.push_back(ifrm1);
				m_num_chsbd_com++;
			}
			m_bdet_chsbd = false;
		}
	}

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!m_img1.empty() && !m_img2.empty())
	{
		// draw images
		Mat wimg1, wimg2;

		m_w = m_sz_win.width >> 1;
		m_h = m_sz_win.height >> 1;
		m_rx1 = (float)((double)m_w / (double)m_img1.cols);
		m_ry1 = (float)((double)m_h / (double)m_img1.rows);
		m_rx2 = (float)((double)m_w / (double)m_img2.cols);
		m_ry2 = (float)((double)m_h / (double)m_img2.rows);
		m_r1 = min(m_rx1, m_ry1);
		m_r2 = min(m_rx2, m_ry2);
		m_sz1.width = (int)(m_r1 * img1.cols);
		m_sz1.height = (int)(m_r1 * img1.rows);
		m_sz2.width = (int)(m_r2 * img2.cols);
		m_sz2.height = (int)(m_r2 * img2.rows);
		m_wn1 = (float)((double)m_sz1.width / (double)m_w);
		m_hn1 = (float)((double)m_sz1.height / (double)m_h);
		m_wn2 = (float)((double)m_sz2.width / (double)m_w);
		m_hn2 = (float)((double)m_sz2.height / (double)m_h);
		m_xscale1 = (float)(m_r1 / (float)m_w);
		m_yscale1 = (float)(m_r1 / (float)m_h);
		m_xscale2 = (float)(m_r2 / (float)m_w);
		m_yscale2 = (float)(m_r2 / (float)m_h);

		resize(m_img1, wimg1, m_sz1);
		resize(m_img2, wimg2, m_sz2);

		glRasterPos2i(-1, 0);
		draw_pixels(wimg1);

		glRasterPos2i(0, 0);
		draw_pixels(wimg2);

		if (m_num_chsbdl){
			draw_chsbd(m_budl && m_bcpl, m_camparl, m_Rl, m_Pl, 1., 0, 0, m_xscale1, m_yscale1, -1, 0,
				m_wn1, m_hn1, m_pts_chsbdl, m_num_chsbdl);
		}

		if (m_num_chsbdr){
			draw_chsbd(m_budr && m_bcpr, m_camparr, m_Rr, m_Pr, 1., 0, 0, m_xscale2, m_yscale2, 0, 0,
				m_wn2, m_hn2, m_pts_chsbdr, m_num_chsbdr);
		}

		if (m_num_chsbd_com){
			draw_com_chsbd(0, 0, 1,
				m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1,
				m_xscale2, m_yscale2, 0, 0, m_wn2, m_hn2);
		}

		if (m_bdisp){ // show disparity map
			calc_and_draw_disparity_map(m_img1, m_img2);
		}

		// draw horizon 
		if (m_state){
			draw_horizon();
		}
	}

	if (m_bsv_chsbd){
		save_chsbd(0);
		save_chsbd(1);
		save_chsbd(2);
		m_bsv_chsbd = false;
	}

	if (m_bld_chsbd){
		load_chsbd(0);
		load_chsbd(1);
		load_chsbd(2);
		m_bld_chsbd = false;
	}

	if (m_bsvstp && m_bstp){
		save_stereo_pars();
		m_bsvstp = false;
	}

	if (m_bldstp){
		load_stereo_pars();
		m_bldstp = false;
	}

	if (m_bptl || m_bptr){
		Point2f pt0;
		float wn, hn, xscale, yscale, xorg, yorg;
		Size sz;

		if (m_bptl){
			pt0 = m_glptl;
			sz = m_sz1;
			wn = m_wn1;
			hn = m_hn1;
			xscale = m_xscale1;
			yscale = m_yscale1;
			xorg = -1.;
			yorg = 0.;
		}
		if (m_bptr){
			pt0 = m_glptr;
			sz = m_sz2;
			wn = m_wn2;
			hn = m_hn2;
			xscale = m_xscale2;
			yscale = m_yscale2;
			xorg = 0.;
			yorg = 0.;
		}
		cnvCvPoint2GlPoint(xscale, yscale, xorg, yorg, wn, hn, pt0, pt0);

		glColor4f(1., 0, 0, 1);
		glBegin(GL_LINES);
		glVertex2f(pt0.x, pt0.y);
		glVertex2f(m_pos_mouse.x, m_pos_mouse.y);
		glEnd();
	}

	if (m_bptl && m_bptr){
		m_ptsl.push_back(m_glptl);
		m_ptsr.push_back(m_glptr);
		m_num_com_pts++;
		m_bptl = m_bptr = false;
	}

	if (m_num_com_pts){
		if (m_budl){
			if (m_camparl.isFishEye()){
				fisheye::undistortPoints(m_ptsl, m_ptsul,
					m_camparl.getCvPrjMat(), m_camparl.getCvDistFishEyeMat(),
					m_Rl, m_Pl);
			}
			else{
				undistortPoints(m_ptsl, m_ptsul,
					m_camparl.getCvPrjMat(), m_camparl.getCvDistMat(),
					m_Rl, m_Pl);
				/*
				check_undistort(m_ptsl, m_ptsul,
				m_camparl.getCvPrjMat(), m_camparl.getCvDistMat(), m_Rl, m_Pl, m_mapl1, m_mapl2);
				*/
			}
		}
		if (m_budr){
			if (m_camparr.isFishEye()){
				fisheye::undistortPoints(m_ptsr, m_ptsur,
					m_camparr.getCvPrjMat(), m_camparl.getCvDistFishEyeMat(),
					m_Rr, m_Pr);
			}
			else{
				undistortPoints(m_ptsr, m_ptsur,
					m_camparr.getCvPrjMat(), m_camparr.getCvDistMat(),
					m_Rr, m_Pr);
			}
		}

		for (int ipt = 0; ipt < m_num_com_pts; ipt++){
			Point2f ptl = m_budl ? m_ptsul[ipt] : m_ptsl[ipt];
			cnvCvPoint2GlPoint(m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1, ptl, ptl);
			Point2f ptr = m_budr ? m_ptsur[ipt] : m_ptsr[ipt];
			cnvCvPoint2GlPoint(m_xscale2, m_yscale2, 0, 0, m_wn2, m_hn2, ptr, ptr);
			glColor4f(1, 1, 0, 1);

			glPointSize(3.f);
			glBegin(GL_POINTS);
			glVertex2f(ptl.x, ptl.y);
			glVertex2f(ptr.x, ptr.y);
			glEnd();

			glBegin(GL_LINES);
			glVertex2f(ptl.x, ptl.y);
			glVertex2f(ptr.x, ptr.y);
			glEnd();
		}
	}

	if (m_bcapture && m_fcapture[0]){
		char buf[1024];
		if (!m_img1.empty()){
			snprintf(buf, 1024, "%sl%lld_%lld.png", m_fcapture, m_timg1, m_ifrm1);
			imwrite(buf, m_img1);
		}

		if (!m_img2.empty()){
			snprintf(buf, 1024, "%sr%lld_%lld.png", m_fcapture, m_timg2, m_ifrm2);
			imwrite(buf, m_img2);
		}

		if (!m_disp.empty()){
			snprintf(buf, 1024, "%sd%lld_%lld.png", m_fcapture, m_timg1, m_ifrm1);
			imwrite(buf, m_disp);
		}

		if (!m_disp16.empty()){
			snprintf(buf, 1024, "%sd%lld_%lld.raw", m_fcapture, m_timg1, m_ifrm1);
			write_raw_img(m_disp16, buf);
		}

		if (!m_dist.empty()){
			snprintf(buf, 1024, "%sD%lld_%lld.png", m_fcapture, m_timg1, m_ifrm1);
			imwrite(buf, m_dist);
		}
		Mat m_simg(m_sz_win.height, m_sz_win.width, CV_8UC3);
		glReadPixels(0, 0, m_sz_win.width, m_sz_win.height, GL_BGR, GL_UNSIGNED_BYTE, m_simg.data);
		awsFlip(m_simg, false, true, false);
		snprintf(buf, 1024, "%sa%lld.png", m_fcapture, m_timg1);
		imwrite(buf, m_simg);

		m_bcapture = false;
	}

	glfwSwapBuffers(pwin());
	glfwPollEvents();

	m_bupdate_img = false;
	return true;
}

bool f_glfw_stereo_view::init_run()
{
	if (!f_glfw_window::init_run())
		return false;

	if (m_bchsbd = m_chsbd.load())
	{
		if (m_chsbd.type == s_model::EMT_CHSBD){
			cout << "Chessboard is loaded." << endl;
			cout << m_chsbd.par_chsbd.w << "x" << m_chsbd.par_chsbd.h << " " << m_chsbd.par_chsbd.p << "m pitch" << endl;
		}
	}

	m_sgbm = StereoSGBM::create(m_sgbm_par.minDisparity, m_sgbm_par.numDisparities,
		m_sgbm_par.blockSize, m_sgbm_par.P1, m_sgbm_par.P2, m_sgbm_par.disp12MaxDiff,
		m_sgbm_par.preFilterCap, m_sgbm_par.uniquenessRatio, m_sgbm_par.speckleWindowSize,
		m_sgbm_par.speckleRange, m_sgbm_par.mode);
	return true;
}

void f_glfw_stereo_view::destroy_run()
{
}

void f_glfw_stereo_view::save_chsbd(int icam)
{
	int num_chsbd = 0;
	int num_chsbd_per_frm = 0;
	const char * fname = NULL;
	vector<long long> *ifrm = NULL;
	vector<vector<Point2f>> *pts[2] = { NULL, NULL };

	switch (icam){
	case 0:
		num_chsbd = m_num_chsbdl;
		num_chsbd_per_frm = 1;
		fname = m_fchsbdl;
		ifrm = &m_ifrm_chsbdl;
		pts[0] = &m_pts_chsbdl;
		break;
	case 1:
		num_chsbd = m_num_chsbdr;
		num_chsbd_per_frm = 1;
		fname = m_fchsbdr;
		ifrm = &m_ifrm_chsbdr;
		pts[0] = &m_pts_chsbdr;
		break;
	case 2:
		num_chsbd = m_num_chsbd_com;
		num_chsbd_per_frm = 2;
		fname = m_fchsbdc;
		ifrm = &m_ifrm_chsbd_com;
		pts[0] = &m_pts_chsbdl_com;
		pts[1] = &m_pts_chsbdr_com;
		break;
	default:
		return;
	}

	FileStorage fs;
	fs.open(fname, FileStorage::WRITE);
	if (!fs.isOpened()){
		cerr << "Failed to open file " << fname << "." << endl;
		return;
	}

	fs << "NumChsbd" << num_chsbd;
	fs << "NumChsbdPerFrm" << num_chsbd_per_frm;
	fs << "ChsbdName" << m_chsbd.name;
	int chsbd_pts = (int)m_chsbd.pts.size();
	char item[1024];
	for (int i = 0; i < num_chsbd; i++){
		snprintf(item, 1024, "frm%d", i);
		fs << item << "{";
		long long val = (*ifrm)[i];
		fs << "FrmIdL" << *((int *)&val);
		fs << "FrmIdH" << *((int *)&val + 1);
		for (int j = 0; j < num_chsbd_per_frm; j++){
			snprintf(item, 1024, "chsbd%d", j);
			fs << item << "[";
			for (int k = 0; k < (*pts[j])[i].size(); k++){
				fs << (*pts[j])[i][k];
			}
			fs << "]";
		}
		fs << "}";
	}
}

void f_glfw_stereo_view::load_chsbd(int icam)
{
	int num_chsbd = 0;
	int num_chsbd_per_frm = 0;
	const char * fname = NULL;
	vector<long long> *ifrm = NULL;
	vector<vector<Point2f>> *pts[2] = { NULL, NULL };
	switch (icam){
	case 0:
		fname = m_fchsbdl;
		ifrm = &m_ifrm_chsbdl;
		pts[0] = &m_pts_chsbdl;
		break;
	case 1:
		ifrm = &m_ifrm_chsbdl;
		fname = m_fchsbdr;
		pts[0] = &m_pts_chsbdr;
		break;
	case 2:
		fname = m_fchsbdc;
		ifrm = &m_ifrm_chsbd_com;
		pts[0] = &m_pts_chsbdl_com;
		pts[1] = &m_pts_chsbdr_com;
		break;
	default:
		return;
	}

	ifrm->clear();
	for (int i = 0; i < 2; i++){
		if (pts[i])
			pts[i]->clear();
	}

	string str;
	FileStorage fs(fname, FileStorage::READ);
	FileNode fn;

	fn = fs["NumChsbd"];
	if (fn.empty()){
		cerr << "Item \"NumChsbd\" cannot be found." << endl;
		return;
	}
	fn >> num_chsbd;

	fn = fs["NumChsbdPerFrm"];
	if (fn.empty()){
		cerr << "Item \"NumChsbdPerFrm\" cannot be found." << endl;
		return;
	}
	fn >> num_chsbd_per_frm;

	if (icam == 2 && num_chsbd_per_frm != 2){
		cerr << "File " << fname << " is not ready for stereo calibration." << endl;
		return;
	}

	fn = fs["ChsbdName"];
	if (fn.empty()){
		cerr << "Item \"ChsbdName\" cannot be found." << endl;
		return;
	}
	fn >> str;

	if (str != m_chsbd.name){
		cerr << "Miss match in chessboard model." << endl;
		return;
	}

	(*ifrm).resize(num_chsbd);
	(*pts[0]).resize(num_chsbd);
	if (pts[1]){
		(*pts[1]).resize(num_chsbd);
	}

	int chsbd_pts = (int)m_chsbd.pts.size();
	char item[1024];
	for (int i = 0; i < num_chsbd; i++){
		snprintf(item, 1024, "frm%d", i);
		FileNode frm = fs[(const char*)item];
		if (frm.empty()){
			cerr << item << " cannot be found." << endl;
			return;
		}

		int valL, valH;
		long long val;
		FileNode frm_id = frm["FrmIdL"];
		if (frm_id.empty()){
			cerr << "\"FrmIdL\" is expected." << endl;
			return;
		}

		frm_id >> valL;

		frm_id = frm["FrmIdH"];
		if (frm_id.empty()){
			cerr << "\"FrmIdH\" is expected." << endl;
		}
		frm_id >> valH;

		*((int *)&val) = valL;
		*((int *)&val + 1) = valH;
		(*ifrm)[i] = val;

		for (int j = 0; j < num_chsbd_per_frm; j++){
			(*pts[j])[i].resize(chsbd_pts);
			snprintf(item, 1024, "chsbd%d", j);
			FileNode chsbd = frm[(const char*)item];
			if (chsbd.empty()){
				cerr << item << " cannot be found." << endl;
				return;
			}

			FileNodeIterator itr = chsbd.begin();
			for (int k = 0; itr != chsbd.end(); itr++, k++){
				*itr >> (*pts[j])[i][k];
			}
		}
	}

	switch (icam){
	case 0:
		m_num_chsbdl = num_chsbd;
		break;
	case 1:
		m_num_chsbdr = num_chsbd;
		break;
	case 2:
		m_num_chsbd_com = num_chsbd;
		break;
	default:
		return;
	}

}

void f_glfw_stereo_view::calibrate_stereo()
{
	Size sz(m_img1.cols, m_img1.rows);
	vector<vector<Point3f>> pt3ds;
	pt3ds.resize(m_num_chsbd_com);
	for (int i = 0; i < m_num_chsbd_com; i++)
		pt3ds[i] = m_chsbd.pts;

	double err;
	Mat Kl, Dl, Kr, Dr;
	if (m_bfisheye){
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistFishEyeMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistFishEyeMat().clone();
		err = fisheye::stereoCalibrate(pt3ds, m_pts_chsbdl_com, m_pts_chsbdr_com,
			Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr);
	}
	else{
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistMat().clone();
		err = stereoCalibrate(pt3ds, m_pts_chsbdl_com, m_pts_chsbdr_com,
			Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr, m_E, m_F);
	}

	double serr = 0;
	int num_pts = 0;
	{ // check epipoler error
		double err;
		vector<Point2f> ptsl, ptsr;
		vector<Vec3f> ll, lr;

		for (int ichsbd = 0; ichsbd < m_num_chsbd_com; ichsbd++){
			if (m_bfisheye){
				fisheye::undistortPoints(m_pts_chsbdl_com[ichsbd], ptsl, Kl, Dl, m_Rl, m_Pl);
				fisheye::undistortPoints(m_pts_chsbdr_com[ichsbd], ptsr, Kr, Dr, m_Rr, m_Pr);
			}
			else{
				undistortPoints(m_pts_chsbdl_com[ichsbd], ptsl, Kl, Dl, m_Rl, Kl);
				undistortPoints(m_pts_chsbdr_com[ichsbd], ptsr, Kr, Dr, m_Rr, Kr);
			}
			computeCorrespondEpilines(ptsl, 1, m_F, ll);
			computeCorrespondEpilines(ptsr, 2, m_F, lr);
			for (int il = 0; il < ptsl.size(); il++){
				err = ptsl[il].x * lr[il][0] + ptsl[il].y * lr[il][1] + lr[il][2];
				serr += err;
				num_pts++;
			}
			for (int il = 0; il < ptsr.size(); il++){
				err = ptsr[il].x * ll[il][0] + ptsr[il].y * ll[il][1] + ll[il][2];
				serr += err;
				num_pts++;
			}
		}
	}
	cout << "Stereo Calibrate done with calibration err=" << err << " epi err = " << serr / (double)num_pts << endl;
	cout << "Rlr" << m_Rlr << endl;
	cout << "Tlr" << m_Tlr << endl;
	m_bcbst = false;
	m_bstp = true;
}


void f_glfw_stereo_view::rectify_stereo()
{
	Size sz(m_img1.cols, m_img1.rows);
	Mat Kl, Dl, Kr, Dr;
	if (m_bfisheye){
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistFishEyeMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistFishEyeMat().clone();
		fisheye::stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr,
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, 0);
		fisheye::initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_32FC1, m_mapl1, m_mapl2);
		fisheye::initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_32FC1, m_mapr1, m_mapr2);
	}
	else{
		Kl = m_camparl.getCvPrjMat().clone();
		Dl = m_camparl.getCvDistMat().clone();
		Kr = m_camparr.getCvPrjMat().clone();
		Dr = m_camparr.getCvDistMat().clone();
		stereoRectify(Kl, Dl, Kr, Dr, sz, m_Rlr, m_Tlr,
			m_Rl, m_Rr, m_Pl, m_Pr, m_Q, CALIB_ZERO_DISPARITY, -1);
		initUndistortRectifyMap(Kl, Dl, m_Rl, m_Pl, sz, CV_32FC1, m_mapl1, m_mapl2);
		initUndistortRectifyMap(Kr, Dr, m_Rr, m_Pr, sz, CV_32FC1, m_mapr1, m_mapr2);
	}

	//	init_undistort(m_camparl, sz, m_Rl, m_Pl, m_mapl1, m_mapl2);
	//	init_undistort(m_camparr, sz, m_Rr, m_Pr, m_mapr1, m_mapr2);
	cout << "Rlr" << m_Rlr << endl;
	cout << "Tlr" << m_Tlr << endl;
	cout << "Rl" << m_Rl << endl;
	cout << "Pl" << m_Pl << endl;
	cout << "Rr" << m_Rr << endl;
	cout << "Pr" << m_Pr << endl;

	double * p = m_Pl.ptr<double>();
	m_odt_par.initD((float)p[0], (float)p[2], (float)p[6], (float)norm(m_Tlr));

	m_brct = true;
	m_brctst = false;
}

void f_glfw_stereo_view::save_stereo_pars()
{
	if (!m_fstp[0]){
		return;
	}
	FileStorage fs(m_fstp, FileStorage::WRITE);
	if (!fs.isOpened()){
		return;
	}
	fs << "Rlr" << m_Rlr;
	fs << "Tlr" << m_Tlr;
	fs << "E" << m_E;
	fs << "F" << m_F;
	fs << "Q" << m_Q;
}

void f_glfw_stereo_view::load_stereo_pars()
{
	if (!m_fstp[0]){
		return;
	}

	FileStorage fs(m_fstp, FileStorage::READ);
	if (!fs.isOpened()){
		return;
	}

	FileNode fn;

	fn = fs["Rlr"];
	if (fn.empty())
		return;
	fn >> m_Rlr;

	fn = fs["Tlr"];
	if (fn.empty())
		return;
	fn >> m_Tlr;

	fn = fs["E"];
	if (fn.empty())
		return;
	fn >> m_E;

	fn = fs["F"];
	if (fn.empty())
		return;
	fn >> m_F;

	fn = fs["Q"];
	if (fn.empty())
		return;
	fn >> m_Q;

	m_bstp = true;
}


void f_glfw_stereo_view::reduce_chsbd(int icam)
{
	int num_chsbd = 0;
	int num_chsbd_per_frm = 0;
	const char * fname = NULL;
	vector<long long> *ifrm = NULL;
	vector<vector<Point2f>> *pts[2] = { NULL, NULL };
	int w, h;
	switch (icam){
	case 0:
		num_chsbd = m_num_chsbdl;
		num_chsbd_per_frm = 1;
		fname = m_fchsbdl;
		ifrm = &m_ifrm_chsbdl;
		pts[0] = &m_pts_chsbdl;
		w = m_img1.cols;
		h = m_img1.rows;
		break;
	case 1:
		num_chsbd = m_num_chsbdr;
		num_chsbd_per_frm = 1;
		fname = m_fchsbdr;
		ifrm = &m_ifrm_chsbdr;
		pts[0] = &m_pts_chsbdr;
		w = m_img2.cols;
		h = m_img2.rows;
		break;
	case 2:
		num_chsbd = m_num_chsbd_com;
		num_chsbd_per_frm = 2;
		fname = m_fchsbdc;
		ifrm = &m_ifrm_chsbd_com;
		pts[0] = &m_pts_chsbdl_com;
		pts[1] = &m_pts_chsbdr_com;
		w = m_img1.cols;
		h = m_img1.rows;
		break;
	default:
		return;
	}


	// dividing image into 8x8 grid, and count the chessboard points found in each grid
	float iws = (float)(1.0 / (double)(w >> 3)), ihs = (float)(1.0 / (double)(h >> 3));
	vector<vector<int>> hist[2];
	vector<s_chsbd_score> cs;
	hist[0].resize(8);
	cs.resize(pts[0]->size());
	for (int i = 0; i < num_chsbd; i++){
		cs[i].idx = i;
		cs[i].score = 0;
	}

	if (num_chsbd_per_frm == 2){
		hist[1].resize(8);
	}

	for (int i = 0; i < 8; i++){
		hist[0][i].resize(8, 0);
		if (num_chsbd_per_frm == 2){
			hist[1][i].resize(8, 0);
		}
	}

	// voting phase
	for (int j = 0; j < num_chsbd_per_frm; j++){
		vector<vector<int>> & _hist = hist[j];
		vector<vector<Point2f>> & _pts = *pts[j];
		for (int k = 0; k < _pts.size(); k++){
			vector<Point2f> & _chsbd = _pts[k];
			for (int l = 0; l < _chsbd.size(); l++){
				Point2f & pt = _chsbd[l];
				int x = (int)(pt.x * iws);
				int y = (int)(pt.y * ihs);
				_hist[x][y]++;
			}
		}
	}

	// remove the number 
	int num_removes = num_chsbd - m_num_calib_chsbd;
	while (num_removes > 0){
		// scoring and find max score chsbd
		s_chsbd_score cs_max;

		cs_max.idx = -1;
		cs_max.score = 0;
		for (int j = 0; j < num_chsbd; j++){
			s_chsbd_score & _cs = cs[j];
			_cs.score = 0.;
			if (_cs.idx == -1)
				continue;

			for (int i = 0; i < num_chsbd_per_frm; i++){
				vector<vector<int>> & _hist = hist[i];
				vector<vector<Point2f>> & _pts = *pts[i];
				vector<Point2f> & _chsbd = _pts[j];
				for (int k = 0; k < _chsbd.size(); k++){
					Point2f & pt = _chsbd[k];
					int x = (int)(pt.x * iws);
					int y = (int)(pt.y * ihs);
					_cs.score += (float)_hist[x][y];
				}
			}
			if (cs_max.score < _cs.score)
				cs_max = _cs;
		}

		// removing and revoting phase		
		for (int i = 0; i < num_chsbd_per_frm; i++){
			vector<vector<int>> & _hist = hist[i];
			vector<vector<Point2f>> & _pts = *pts[i];

			int j = cs_max.idx;
			vector<Point2f> & _chsbd = _pts[j];
			s_chsbd_score & _cs = cs[j];
			_cs.idx = -1;

			// reducing hist count
			for (int k = 0; k < _chsbd.size(); k++){
				Point2f & pt = _chsbd[k];
				int x = (int)(pt.x * iws);
				int y = (int)(pt.y * ihs);
				_hist[x][y]--;
			}

			// clear chessboard
			_chsbd.clear();
		}

		num_removes--;
	}

	// remove chessboard entry removed
	for (int i = 0; i < num_chsbd_per_frm; i++){
		vector<vector<Point2f>> & _pts = *pts[i];
		for (vector<vector<Point2f>>::iterator itr = _pts.begin();
			itr != _pts.end();)
		{
			if ((*itr).size() == 0)
				itr = _pts.erase(itr);
			else
				itr++;
		}
	}

	switch (icam){
	case 0:
		m_num_chsbdl = min(m_num_calib_chsbd, m_num_chsbdl);
		break;
	case 1:
		m_num_chsbdr = min(m_num_calib_chsbd, m_num_chsbdr);
		break;
	case 2:
		m_num_chsbd_com = min(m_num_calib_chsbd, m_num_chsbd_com);
		break;
	default:
		return;
	}

}

void f_glfw_stereo_view::calibrate(int icam)
{
	AWSCamPar * pcp = NULL;
	vector<vector<Point2f>> * ppts = NULL;
	vector<Mat> tvecs;
	vector<Mat> rvecs;
	bool * pbcp;
	Mat * pimg = NULL, *R = NULL, *P = NULL, *map1 = NULL, *map2 = NULL;
	if (icam == 0){
		pbcp = &m_bcpl;
		pcp = &m_camparl;
		ppts = &m_pts_chsbdl;
		pimg = &m_img1;
		P = &m_Pl;
		R = &m_Rl;
		map1 = &m_mapl1;
		map2 = &m_mapl2;
	}
	else{
		pbcp = &m_bcpr;
		pcp = &m_camparr;
		ppts = &m_pts_chsbdr;
		pimg = &m_img2;
		P = &m_Pr;
		R = &m_Rr;
		map1 = &m_mapr1;
		map2 = &m_mapr2;
	}

	Size sz(pimg->cols, pimg->rows);
	vector<vector<Point3f>> pt3ds;
	pt3ds.resize(ppts->size());
	for (int i = 0; i < ppts->size(); i++)
		pt3ds[i] = m_chsbd.pts;

	double err;
	Mat K, D;

	if (m_bfisheye){
		pcp->setFishEye(true);
		int flag = 0;
		if (m_bfix_int)
			flag |= fisheye::CALIB_FIX_INTRINSIC;
		if (m_bguess_int)
			flag |= fisheye::CALIB_USE_INTRINSIC_GUESS;
		if (m_bfix_k1)
			flag |= fisheye::CALIB_FIX_K1;
		if (m_bfix_k2)
			flag |= fisheye::CALIB_FIX_K2;
		if (m_bfix_k3)
			flag |= fisheye::CALIB_FIX_K3;
		if (m_bfix_k4)
			flag |= fisheye::CALIB_FIX_K4;

		err = fisheye::calibrate(pt3ds, *ppts, sz, K, D, rvecs, tvecs, flag);
		pcp->setCvDist(D);
		pcp->setCvPrj(K);
	}
	else{
		int flag = 0;
		pcp->setFishEye(false);
		if (m_bfix_k1)
			flag |= CALIB_FIX_K1;
		if (m_bfix_k2)
			flag |= CALIB_FIX_K2;
		if (m_bfix_k3)
			flag |= CALIB_FIX_K3;
		if (m_bfix_k4)
			flag |= CALIB_FIX_K4;
		if (m_bfix_k5)
			flag |= CALIB_FIX_K5;
		if (m_bfix_k6)
			flag |= CALIB_FIX_K6;
		if (m_bfix_pp)
			flag |= CALIB_FIX_PRINCIPAL_POINT;
		if (m_brat_mdl)
			flag |= CALIB_RATIONAL_MODEL;
		if (m_bzr_tng)
			flag |= CALIB_ZERO_TANGENT_DIST;
		if (m_bguess_int)
			flag |= CALIB_USE_INTRINSIC_GUESS;
		if (m_bfix_ar)
			flag |= CALIB_FIX_ASPECT_RATIO;

		err = calibrateCamera(pt3ds, *ppts, sz, K,
			D, rvecs, tvecs, flag);
		pcp->setCvDist(D);
		pcp->setCvPrj(K);
	}
	cout << "Calibration camera " << icam << " done with err = " << err << endl;
	cout << "K" << K << endl;
	cout << "D" << D << endl;
	*R = Mat::eye(3, 3, CV_64FC1);
	*P = pcp->getCvPrjMat().clone();
	init_undistort(*pcp, sz, *R, *P, *map1, *map2);
	*pbcp = true;
}

void f_glfw_stereo_view::init_undistort(AWSCamPar & par, Size & sz,
	Mat & R, Mat & P, Mat & map1, Mat & map2)
{
	if (par.isFishEye()){
		Mat K, D;
		K = par.getCvPrjMat().clone();
		D = par.getCvDistFishEyeMat().clone();
		//fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, sz, R, P);
		fisheye::initUndistortRectifyMap(K, D, R, P, sz, CV_32FC1, map1, map2);
	}
	else{
		Mat K, D;
		K = par.getCvPrjMat().clone();
		D = par.getCvDistMat().clone();
		//P = getOptimalNewCameraMatrix(K, D, sz, 1.);
		initUndistortRectifyMap(K, D, R, P, sz, CV_32FC1, map1, map2);
	}
}

void f_glfw_stereo_view::draw_horizon()
{
	long long tatt;
	float roll, pitch, yaw;
	m_state->get_attitude(tatt, roll, pitch, yaw);
	if (tatt != m_tatt[m_iatt]){
		m_iatt++;
		if (m_iatt == SZ_ATT_BUF)
			m_iatt = 0;
		m_tatt[m_iatt] = tatt;
		m_roll[m_iatt] = roll;
		m_pitch[m_iatt] = pitch;
		m_yaw[m_iatt] = yaw;
	}

	tatt = m_timg1 + m_dtatt;
	int tdiff, tdiff_min = INT_MAX;
	int iatt;
	for (int i = 0; i < SZ_ATT_BUF; i++){
		tdiff = (int)abs(tatt - m_tatt[i]);
		if (tdiff < tdiff_min){
			tdiff_min = tdiff;
			iatt = i;
		}
	}
	tatt = m_tatt[iatt];
	roll = m_roll[iatt];
	pitch = m_pitch[iatt];
	yaw = m_yaw[iatt];
	Mat R0, R;
	getmatrotRPY((float)(m_roll0 * (CV_PI / 180.)),
		(float)(m_pitch0 * (CV_PI / 180.)), 0, R0);

	getmatrotRPY((float)(roll * (CV_PI / 180.)),
		(float)(pitch * (CV_PI / 180.)), 0, R);

	R *= R0;
	float ifx, ify, fx, fy;
	Point2f pc;
	if (m_brct){
		double * pP = m_Pl.ptr<double>();
		fx = (float)(pP[0]);
		fy = (float)(pP[5]);
		ifx = (float)(1.0 / pP[0]);
		ify = (float)(1.0 / pP[5]);
		pc = Point2f((float)pP[2], (float)pP[6]);
	}
	else{
		fx = (float)(m_camparl.getCvPrj()[0]);
		fy = (float)(m_camparl.getCvPrj()[1]);
		ifx = (float)(1.0 / m_camparl.getCvPrj()[0]);
		ify = (float)(1.0 / m_camparl.getCvPrj()[1]);
		pc = Point2f((float)m_camparl.getCvPrj()[2], (float)m_camparl.getCvPrj()[3]);
	}
	Point2f p0, p1;

	// static horizon line
	p0.x = pc.x - 400;
	p0.y = pc.y;
	p1.x = pc.x + 400;
	p1.y = pc.y;

	p0 -= pc;
	p1 -= pc;
	p0.x *= ifx;
	p0.y *= ify;
	p1.x *= ifx;
	p1.y *= ify;

	double * pR = R.ptr<double>();
	Point2f p0h, p1h; // homography with R0 and R
	float iw;
	iw = (float)(1.0 / (pR[6] * p0.x + pR[7] * p0.y + pR[8]));
	p0h.x = (float)(iw * fx * (pR[0] * p0.x + pR[1] * p0.y + pR[2]) + pc.x);
	p0h.y = (float)(iw * fy * (pR[3] * p0.x + pR[4] * p0.y + pR[5]) + pc.y);

	iw = (float)(1.0 / (pR[6] * p1.x + pR[7] * p1.y + pR[8]));
	p1h.x = (float)(iw * fx * (pR[0] * p1.x + pR[1] * p1.y + pR[2]) + pc.x);
	p1h.y = (float)(iw * fy * (pR[3] * p1.x + pR[4] * p1.y + pR[5]) + pc.y);

	// draw line on p0h and p1h
	cnvCvPoint2GlPoint(m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1, p0h, p0h);
	cnvCvPoint2GlPoint(m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1, p1h, p1h);

	glColor4f(0, 0.5, 0, 1);
	glBegin(GL_LINES);
	glVertex2f(p0h.x, p0h.y);
	glVertex2f(p1h.x, p1h.y);
	glEnd();
	char buf[128];
	snprintf(buf, 128, "p=%3.1f r=%3.1f", pitch, roll);
	drawGlText(p1h.x, p1h.y, buf, 0, 0.5, 0, 1., GLUT_BITMAP_8_BY_13);

	p0h.y += -1.0;
	p1h.y += -1.0;

	snprintf(buf, 128, "timg=%lld tstt=%lld timg-tstt=%lld", m_timg1, tatt, m_timg1 - tatt);
	drawGlText(-1, 0.5, buf, 0, 0.5, 0, 1., GLUT_BITMAP_8_BY_13);

	glBegin(GL_LINES);
	glVertex2f(p0h.x, p0h.y);
	glVertex2f(p1h.x, p1h.y);
	glEnd();
}


void f_glfw_stereo_view::calc_and_draw_disparity_map(Mat & img1, Mat & img2)
{
	int w, h;
	w = m_sz_win.width >> 1;
	h = m_sz_win.height >> 1;
	ushort disp_max = m_sgbm_par.numDisparities * 16;

	if (m_bupdate_img){
		if (m_sgbm_par.m_update){ // updating sgbm parameters
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

		m_sgbm->compute(img1, img2, m_disp16);

		calc_dline(disp_max);
		calc_dmap(disp_max);
		::calc_obst(m_odt_par, m_disp16, m_obst);
		m_disp16.convertTo(m_disp, CV_8U, 255 / (m_sgbm_par.numDisparities * 16.));
	}

	if (m_disp.empty())
		return;

	Mat wdisp;
	double rxd, ryd, rd;
	rxd = (double)w / (double)m_disp.cols;
	ryd = (double)h / (double)m_disp.rows;
	rd = min(rxd, ryd);
	Size sz((int)(m_disp.cols * rd), (int)(m_disp.rows * rd));

	resize(m_disp, wdisp, sz);
	awsFlip(wdisp, false, true, false);

	glRasterPos2i(-1, -1);
	glDrawPixels(wdisp.cols, wdisp.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, wdisp.data);

	if (m_dist.empty())
		return;

	Mat wdist;
	glRasterPos2i(0, -1);
	glDrawPixels(m_dist.cols, m_dist.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_dist.data);

	draw_dline();
	draw_obst(disp_max);
}

void f_glfw_stereo_view::draw_obst(ushort disp_max)
{
	double sd = 1.0 / (double)disp_max;
	double sx = 1.0 / (double)m_disp16.cols;
	double sy = (double)m_disp16.rows / (double)m_sz_win.height;
	for (int iobst = 0; iobst < m_obst.size(); iobst++){
		s_obst & obst = m_obst[iobst];
		float x0, x1, y0, y1;
		y0 = (float)((double)(disp_max - obst.dmin) * sd - 1.0);
		y1 = (float)((double)(disp_max - obst.dmax) * sd - 1.0);
		x0 = (float)(sx * (double)obst.xmin);
		x1 = (float)(sx * (double)obst.xmax);
		drawGlSquare2Df(x0, y0, x1, y1, 1., 0, 0, 1, 1);

		x0 -= 1.0;
		x1 -= 1.0;
		y0 = (float)((double)m_hn1 - obst.ymin * m_yscale1 - 1.);
		y1 = (float)((double)m_hn1 - obst.ymax * m_yscale1 - 1.);
		drawGlSquare2Df(x0, y0, x1, y1, 1, 0, 0, 1, 1);

	}
	glColor4f(1, 1, 0, 1);
}

void f_glfw_stereo_view::draw_dline()
{
	glColor4f(1, 1, 0, 1);
	for (int il = 0; il < m_dline.size(); il++)
	{
		glBegin(GL_LINES);
		glVertex2f(0.0f, m_dline[il]);
		glVertex2f(0.05f, m_dline[il]);
		glVertex2f(0.95f, m_dline[il]);
		glVertex2f(1.0f, m_dline[il]);
		glEnd();
	}
}

void f_glfw_stereo_view::calc_dline(ushort disp_max)
{
	if (!m_dline.size()){
		double idisp_max = 1.0 / (double)disp_max;
		double * pPl = m_Pl.ptr<double>();
		double fx = pPl[0], fy = pPl[5];
		double L = norm(m_Tlr, NORM_L2);
		double Dmax = L * fx;
		double iDmax = 1.0 / Dmax;
		int Dmax_int = (int)(Dmax + 0.5);
		int n100 = Dmax_int / 10;
		m_dline.resize(n100);
		m_dline[0] = 0;
		for (int i = 1; i < n100; i++){
			m_dline[i] = (float)(
				((double)disp_max - (double)(1.6 * Dmax / (double)i)) * idisp_max - 1.0);
		}
	}
}

void f_glfw_stereo_view::calc_dmap(ushort disp_max)
{
	m_dist = Mat::zeros(m_sz_win.height >> 1, m_sz_win.width >> 1, CV_8UC1);

	double sy = m_dist.rows / (double)disp_max;
	double sx = (double)m_dist.cols / (double)m_disp16.cols;

	ushort * pdisp = m_disp16.ptr<ushort>();
	uchar * pdist = m_dist.ptr<uchar>();
	for (int y = 0; y < m_disp16.rows; y++){
		for (int x = 0; x < m_disp16.cols; x++){
			if (*pdisp && *pdisp <= m_odt_par.dn && *pdisp >= m_odt_par.df){
				ushort idisp = disp_max - *pdisp;
				int y2 = (int)(idisp * sy + 0.5);
				int idx = (int)(sx * x + 0.5) + y2 * m_dist.cols;
				ushort val = pdist[idx];
				val += 16;
				pdist[idx] = saturate_cast<uchar>(val);
			}
			else{
				*pdisp = 0;
			}
			pdisp++;
		}
	}
}

void f_glfw_stereo_view::draw_pixels(Mat & img)
{
	awsFlip(img, false, true, false);
	switch (img.type()){
	case CV_8U:
		glDrawPixels(img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);
		break;
	case CV_16U:
		glDrawPixels(img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_SHORT, img.data);
		break;
	case CV_8UC3:
		glDrawPixels(img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
		break;
	}
}


void f_glfw_stereo_view::draw_com_chsbd(const float r, const float g, const float b,
	const float xscalel, const float yscalel,
	const float xorgl, const float yorgl,
	const float wl, const float hl,
	const float xscaler, const float yscaler,
	const float xorgr, const float yorgr,
	const float wr, const float hr)
{
	glLineWidth(1);

	Mat Kl, Kr, Dl, Dr;
	if (m_camparl.isFishEye()){
		Kl = m_camparl.getCvPrjMat();
		Dl = m_camparl.getCvDistFishEyeMat();
	}
	else{
		Kl = m_camparl.getCvPrjMat();
		Dl = m_camparl.getCvDistMat();
	}
	if (m_camparr.isFishEye()){
		Kr = m_camparr.getCvPrjMat();
		Dr = m_camparr.getCvDistFishEyeMat();
	}
	else{
		Kr = m_camparr.getCvPrjMat();
		Dr = m_camparr.getCvDistMat();
	}

	Size sz_chsbd(m_chsbd.par_chsbd.w, m_chsbd.par_chsbd.h);
	int i = 0,
		j = sz_chsbd.width - 1,
		k = sz_chsbd.height * sz_chsbd.width - 1,
		l = (sz_chsbd.height - 1) * sz_chsbd.width;

	vector<Point2f> pt4l, pt4r, ptu4l, ptu4r;
	vector<Vec3f> line4l, line4r;
	pt4l.resize(4);
	pt4r.resize(4);

	for (int ichsbd = 0; ichsbd < m_num_chsbd_com; ichsbd++){
		vector<Point2f> & ptsl = m_pts_chsbdl_com[ichsbd], &ptsr = m_pts_chsbdr_com[ichsbd];
		pt4l[0] = ptsl[i];
		pt4l[1] = ptsl[j];
		pt4l[2] = ptsl[k];
		pt4l[3] = ptsl[l];

		if (m_budl && m_bcpl){
			if (m_camparl.isFishEye()){
				fisheye::undistortPoints(pt4l, ptu4l, Kl, Dl, m_Rl, m_Pl);
			}
			else{
				undistortPoints(pt4l, ptu4l, Kl, Dl, m_Rl, m_Pl);
			}
			for (int ipt = 0; ipt < 4; ipt++){
				cnvCvPoint2GlPoint(xscalel, yscalel, xorgl, yorgl, wl, hl, ptu4l[ipt], pt4l[ipt]);
			}
		}
		else{
			for (int ipt = 0; ipt < 4; ipt++){
				cnvCvPoint2GlPoint(xscalel, yscalel, xorgl, yorgl, wl, hl, pt4l[ipt], pt4l[ipt]);
			}
		}

		pt4r[0] = ptsr[i];
		pt4r[1] = ptsr[j];
		pt4r[2] = ptsr[k];
		pt4r[3] = ptsr[l];
		if (m_budr && m_bcpr){
			if (m_camparr.isFishEye()){
				fisheye::undistortPoints(pt4r, ptu4r, Kr, Dr, m_Rr, m_Pr);
			}
			else{
				undistortPoints(pt4r, ptu4r, Kr, Dr, m_Rr, m_Pr);
			}
			for (int ipt = 0; ipt < 4; ipt++){
				cnvCvPoint2GlPoint(xscaler, yscaler, xorgr, yorgr, wr, hr, ptu4r[ipt], pt4r[ipt]);
			}
		}
		else{
			for (int ipt = 0; ipt < 4; ipt++){
				cnvCvPoint2GlPoint(xscaler, yscaler, xorgr, yorgr, wr, hr, pt4r[ipt], pt4r[ipt]);
			}
		}

		glColor4f(r, g, b, 1);

		glBegin(GL_LINE_LOOP);
		for (int ipt = 0; ipt < 4; ipt++)
			glVertex2f(pt4l[ipt].x, pt4l[ipt].y);
		glEnd();
		glBegin(GL_LINE_LOOP);
		for (int ipt = 0; ipt < 4; ipt++)
			glVertex2f(pt4r[ipt].x, pt4r[ipt].y);
		glEnd();

		if (m_bstp && m_budl && m_budr){// draw epiline
			computeCorrespondEpilines(ptu4l, 1, m_F, line4l);
			computeCorrespondEpilines(ptu4r, 2, m_F, line4r);
			/*
			double err, serr = 0;
			for(int il = 0; il < 4; il++){
			err = ptu4l[il].x * line4r[il][0] + ptu4l[il].y * line4r[il][1] + line4r[il][2];
			serr += err;
			}
			serr = 0;
			for(int il = 0; il < 4; il++){
			err = ptu4r[il].x * line4l[il][0] + ptu4r[il].y * line4l[il][1] + line4l[il][2];
			serr += err;
			}
			*/

			for (int ipt = 0; ipt < 4; ipt++){
				Point2f pt1, pt2;
				double div = 1.0 / line4l[ipt][1];
				pt1.x = 0;
				pt1.y = -(float)(line4l[ipt][2] * div);
				pt2.x = (float)m_img2.cols;
				pt2.y = -(float)((pt2.x * line4l[ipt][0] + line4l[ipt][2]) * div);
				cnvCvPoint2GlPoint(xscaler, yscaler, xorgr, yorgr, wr, hr, pt1, pt1);
				cnvCvPoint2GlPoint(xscaler, yscaler, xorgr, yorgr, wr, hr, pt2, pt2);
				glColor4f(0, 1, 0, 1);
				glBegin(GL_LINES);
				glVertex2f(pt1.x, pt1.y);
				glVertex2f(pt2.x, pt2.y);
				glEnd();

				div = 1.0 / line4r[ipt][1];
				pt1.x = 0;
				pt1.y = -(float)(line4r[ipt][2] * div);
				pt2.x = (float)m_img1.cols;
				pt2.y = -(float)((pt2.x * line4r[ipt][0] + line4r[ipt][2]) * div);
				cnvCvPoint2GlPoint(xscalel, yscalel, xorgl, yorgl, wl, hl, pt1, pt1);
				cnvCvPoint2GlPoint(xscalel, yscalel, xorgl, yorgl, wl, hl, pt2, pt2);
				glBegin(GL_LINES);
				glVertex2f(pt1.x, pt1.y);
				glVertex2f(pt2.x, pt2.y);
				glEnd();
			}
		}
	}
}

void f_glfw_stereo_view::draw_chsbd(bool ud,
	AWSCamPar & cp, Mat & R, Mat & P, const float r, const float g, const float b,
	const float xscale, const float yscale,
	const float xorg, const float yorg,
	const float w, const float h,
	vector<vector<Point2f>> & chsbds, const int num_chsbds)
{
	glColor4f(r, g, b, 1);
	glLineWidth(1);
	Size sz_chsbd(m_chsbd.par_chsbd.w, m_chsbd.par_chsbd.h);
	int i = 0,
		j = sz_chsbd.width - 1,
		k = sz_chsbd.height * sz_chsbd.width - 1,
		l = (sz_chsbd.height - 1) * sz_chsbd.width;

	Mat K, D;
	if (cp.isFishEye()){
		K = cp.getCvPrjMat();
		D = cp.getCvDistFishEyeMat();
	}
	else{
		K = cp.getCvPrjMat();
		D = cp.getCvDistMat();
	}
	vector<Point2f> pt4, ptu4;
	pt4.resize(4);
	for (int ichsbd = 0; ichsbd < num_chsbds; ichsbd++){
		vector<Point2f> & pts = chsbds[ichsbd];

		glBegin(GL_LINE_LOOP);
		{
			Point2f pt;

			pt4[0] = pts[i];
			pt4[1] = pts[j];
			pt4[2] = pts[k];
			pt4[3] = pts[l];
			if (ud){
				if (cp.isFishEye())
					fisheye::undistortPoints(pt4, ptu4, K, D, R, P);
				else
					undistortPoints(pt4, ptu4, K, D, R, P);
				for (int ipt = 0; ipt < 4; ipt++){
					cnvCvPoint2GlPoint(xscale, yscale, xorg, yorg, w, h, ptu4[ipt], pt);
					glVertex2f(pt.x, pt.y);
				}
			}
			else{
				for (int ipt = 0; ipt < 4; ipt++){
					cnvCvPoint2GlPoint(xscale, yscale, xorg, yorg, w, h, pt4[ipt], pt);
					glVertex2f(pt.x, pt.y);
				}
			}
		}
		glEnd();
	}
}

void f_glfw_stereo_view::check_undistort(vector<Point2f> & pts, vector<Point2f> & ptsu,
	Mat & K, Mat & D, Mat & R, Mat & P, Mat & map1, Mat & map2)
{
	Mat PR = P.colRange(0, 3) * R;
	cout << "PR = " << PR;
	Mat iPR = PR.inv();
	cout << "iPR = " << iPR;
	double * _iPR = iPR.ptr<double>(0);

	vector<Point3f> ptsn;
	vector<Point2f> ptsr;
	ptsn.resize(pts.size());

	// back projection to the normalized camera coordinate
	for (int ipt = 0; ipt < pts.size(); ipt++){
		Point2f & ptu = ptsu[ipt];
		Point3f & ptn = ptsn[ipt];
		double iw = 1.0 / (_iPR[6] + _iPR[7] + _iPR[8]);
		ptn.x = (float)((_iPR[0] * ptu.x + _iPR[1] * ptu.y + _iPR[2]) * iw);
		ptn.y = (float)((_iPR[3] * ptu.x + _iPR[4] * ptu.y + _iPR[5]) * iw);
		ptn.z = 1.0;
	}

	projectPoints(ptsn, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), K, D, ptsr);

	for (int ipt = 0; ipt < pts.size(); ipt++){
		Point2f & ptu = ptsu[ipt];
		Point3f & ptn = ptsn[ipt];
		Point2f & ptr = ptsr[ipt];
		Point2f & pt = pts[ipt];
		Point2f ptm(map1.at<float>((int)ptu.y, (int)ptu.x), map2.at<float>((int)ptu.y, (int)ptu.x));
		cout << ptu << "->" << ptn << "->" << ptr << "=" << pt << "=" << ptm << endl;
	}
}


#endif
