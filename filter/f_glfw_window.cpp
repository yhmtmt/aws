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
#include <map>
#include <vector>
#include <list>
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

#include "f_glfw_window.h"

//////////////////////////////////////////////////////////////////////////////////////////// f_glfw_window
MapGLFWin f_glfw_window::m_map_glfwin;

f_glfw_window::f_glfw_window(const char * name) :f_base(name), m_sz_win(640, 480), m_depth_bits(16), m_bfull(false), m_display(0)
{
	m_pwin = NULL;
	register_fpar("width", &m_sz_win.width, "Width of the window.");
	register_fpar("height", &m_sz_win.height, "Height of the window.");
	register_fpar("depth_bits", &m_depth_bits, "Bit depth of depth buffer.");
	register_fpar("full", &m_bfull, "Full screen mode is enabled.");
	register_fpar("display", &m_display, "Display number used for full screen mode");
}

f_glfw_window::~f_glfw_window()
{
}

bool f_glfw_window::init_run()
{
  if(!glfwInit()){
    cerr << "Failed to initialize GLFW." << endl;
    return false;
  }

  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

  glfwWindowHint(GLFW_DEPTH_BITS, m_depth_bits);

  GLFWmonitor * pmonitor = NULL;
  if (m_bfull)
  {
	  int count = 0;
	  GLFWmonitor ** pmonitors = glfwGetMonitors(&count);
	  if (m_display < count)
		  pmonitor = pmonitors[m_display];
	  else
		  pmonitor = pmonitors[0];
	  
	  int min_diff = INT_MAX;
	  int ivm_best = 0;
	  /*
	  const GLFWvidmode * pvidmods = glfwGetVideoModes(pmonitor, &count);
	  for (int ivm = 0; ivm < count; ivm++){
		  int diff = abs(pvidmods[ivm].width - m_sz_win.width) + abs(pvidmods[ivm].height - m_sz_win.height);
		  if (diff < min_diff){
			  min_diff = diff;
			  ivm_best = ivm;
		  }
	  }
	  GLFWvidmode vidmod;  
	  vidmod = pvidmods[ivm_best];

	  m_sz_win.width = vidmod.width;
	  m_sz_win.height = vidmod.height;
	  glfwWindowHint(GLFW_RED_BITS, vidmod.redBits);
	  glfwWindowHint(GLFW_GREEN_BITS, vidmod.greenBits);
	  glfwWindowHint(GLFW_BLUE_BITS, vidmod.blueBits);
	  glfwWindowHint(GLFW_REFRESH_RATE, vidmod.refreshRate);

	  */
	  const GLFWvidmode * pvidmod = glfwGetVideoMode(pmonitor);

	  m_sz_win.width = pvidmod->width;
	  m_sz_win.height = pvidmod->height;
	  glfwWindowHint(GLFW_RED_BITS, pvidmod->redBits);
	  glfwWindowHint(GLFW_GREEN_BITS, pvidmod->greenBits);
	  glfwWindowHint(GLFW_BLUE_BITS, pvidmod->blueBits);
	  glfwWindowHint(GLFW_REFRESH_RATE, pvidmod->refreshRate);
  }
  
  m_pwin = glfwCreateWindow(m_sz_win.width, m_sz_win.height, m_name, pmonitor, NULL);
  if (!pwin())
    {
      cerr << "Failed to create GLFW window." << endl;
      cerr << "Window name = " << m_name << " " << m_sz_win.width << " x " << m_sz_win.height << endl;
      glfwTerminate();
      return false;
    }
  
  m_map_glfwin.insert(pair<GLFWwindow*, f_glfw_window*>(m_pwin, this));
  
  glfwMakeContextCurrent(pwin());
  
  glfwSetKeyCallback(pwin(), key_callback);
  glfwSetCursorPosCallback(pwin(), cursor_position_callback);
  glfwSetMouseButtonCallback(pwin(), mouse_button_callback);
  glfwSetScrollCallback(pwin(), scroll_callback);
  glfwSetErrorCallback(err_cb);
  
  glfwSetWindowTitle(pwin(), m_name);
  
  GLenum err;
  if((err = glewInit()) != GLEW_OK){
    cerr << "Failed to initialize GLEW." << endl;
    cerr << "\tMessage: " << glewGetString(err) << endl;
    return false;
  }

  int argc = 0;
  glutInit(&argc, NULL);

	return true;	
}

void f_glfw_window::destroy_run()
{ 
  if(!m_pwin)
    return;
  glfwTerminate();
  MapGLFWin::iterator itr = m_map_glfwin.find(m_pwin);
  m_map_glfwin.erase(itr);
  m_pwin = NULL;
}

bool f_glfw_window::proc()
{
	if(glfwWindowShouldClose(pwin()))
		return false;

	//glfwMakeContextCurrent(pwin());
	
	// rendering codes >>>>>

	// <<<<< rendering codes

	glfwSwapBuffers(pwin());

	glfwPollEvents();

	return true;
}


//////////////////////////////////////////////////////// f_glfw_imview
bool f_glfw_imview::proc()
{
  glfwMakeContextCurrent(pwin());

	if(glfwWindowShouldClose(pwin()))
		return false;

	if (!m_pin->is_new(m_timg))
		return true;

	long long timg, tfrm;
	Mat img = m_pin->get_img(timg, tfrm);
	if(m_bverb == true){
	  cout << "Newframe[" << tfrm << "] t=" << timg << endl;
	}

	bool bnew = true;
	if(img.empty())
		return true;

	if (m_timg == timg){
		return true;
	}

	m_timg = timg;
	if(m_sz_win.width != img.cols || m_sz_win.height != img.rows){
	  Mat tmp;
	  resize(img, tmp, m_sz_win);
	  img = tmp;
	}

	glRasterPos2i(-1, -1);
	
	if(img.type() == CV_8U){
	  glDrawPixels(img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);
	}
	else{
	  cnvCVBGR8toGLRGB8(img);
	  glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);
	}

	float hfont = (float)(24. / (float) m_sz_win.height);
	float wfont = (float)(24. / (float) m_sz_win.height);
	float x = wfont - 1;
	float y = 1 - 2 * hfont;

	// show time
	drawGlText(x, y, m_time_str, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);

	glfwSwapBuffers(pwin());
	glfwPollEvents();

	return true;
}


bool f_glfw_imview::init_run()
{
  if(m_chin.size() == 0){
    cerr << m_name << " requires at least an input channel." << endl;
    return false;
  }		
	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_pin == NULL){
	  cerr << m_name << "'s first input channel should be image channel." << endl;
	  return false;
	}

	if(!f_glfw_window::init_run())
		return false;

	return true;
}


//////////////////////////////////////////////////////////////////////////////// f_glfw_claib members
f_glfw_calib::f_glfw_calib(const char * name) :f_glfw_window(name), m_b3dview(true),
	m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_campar(false), m_bcalib_fix_focus(false),
	m_bcalib_fix_principal_point(false), m_bcalib_fix_aspect_ratio(false), m_bcalib_zero_tangent_dist(true),
	m_bcalib_fix_k1(false), m_bcalib_fix_k2(false), m_bcalib_fix_k3(false),
	m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false), 
	m_bFishEye(false), m_bcalib_recompute_extrinsic(true), m_bcalib_check_cond(true),
	m_bcalib_fix_skew(false), m_bcalib_fix_intrinsic(false),
	m_bcalib_done(false), m_num_chsbds(30), m_bthact(false), m_hist_grid(10, 10), m_rep_avg(1.0),
	m_bshow_chsbd_all(true), m_bshow_chsbd_sel(true), m_sel_chsbd(-1), m_bshow_chsbd_density(true),
	m_bsave(false), m_bload(false), m_bundist(false), m_bdel(false), m_bdet(false), m_bcalib(false)
{
	m_fcampar[0] = m_fcbdet[0] = m_fsmplimg[0] = '\0';

	register_fpar("fchsbd", m_model_chsbd.fname, 1024, "File path for the chessboard model.");
	register_fpar("fcbdet", m_fcbdet, 1024, "File path for detected chessboard.");
	register_fpar("fsmplimg", m_fsmplimg, 1024, "Sample image file.");
	register_fpar("nchsbd", &m_num_chsbds, "Number of chessboards stocked.");
	register_fpar("fcampar", m_fcampar, 1024, "File path of the camera parameter.");
	register_fpar("Wgrid", &m_hist_grid.width, "Number of horizontal grid of the chessboard histogram.");
	register_fpar("Hgrid", &m_hist_grid.height, "Number of vertical grid of the chessboard histogram.");

	register_fpar("3dv", &m_b3dview, "3D view enable.");

	// normal camera model
	register_fpar("use_intrinsic_guess", &m_bcalib_use_intrinsic_guess, "Use intrinsic guess.");
	register_fpar("recompute_extrinsic", &m_bcalib_recompute_extrinsic, "Recompute Extrinsic in each iteration. (only FishEye)");

	register_fpar("fix_campar", &m_bcalib_fix_campar, "Fix camera parameters");
	register_fpar("fix_focus", &m_bcalib_fix_focus, "Fix camera's focal length");
	register_fpar("fix_principal_point", &m_bcalib_fix_principal_point, "Fix camera center as specified (cx, cy)");
	register_fpar("fix_aspect_ratio", &m_bcalib_fix_aspect_ratio, "Fix aspect ratio as specified fx/fy. Only fy is optimized.");
	register_fpar("zero_tangent_dist", &m_bcalib_zero_tangent_dist, "Zeroify tangential distortion (px, py)");
	register_fpar("fix_k1", &m_bcalib_fix_k1, "Fix k1 as specified.");
	register_fpar("fix_k2", &m_bcalib_fix_k2, "Fix k2 as specified.");
	register_fpar("fix_k3", &m_bcalib_fix_k3, "Fix k3 as specified.");
	register_fpar("fix_k4", &m_bcalib_fix_k4, "Fix k4 as specified.");
	register_fpar("fix_k5", &m_bcalib_fix_k5, "Fix k5 as specified.");
	register_fpar("fix_k6", &m_bcalib_fix_k6, "Fix k6 as specified.");
	register_fpar("rational_model", &m_bcalib_rational_model, "Enable rational model (k4, k5, k6)");
	register_fpar("chk_cnd", &m_bcalib_check_cond, "Check condition (only for FishEye)");
	register_fpar("fx", (m_par.getCvPrj() + 0), "Focal length in x");
	register_fpar("fy", (m_par.getCvPrj() + 1), "Focal length in y");
	register_fpar("cx", (m_par.getCvPrj() + 2), "Principal point in x");
	register_fpar("cy", (m_par.getCvPrj() + 3), "Principal point in y");
	register_fpar("k1", (m_par.getCvDist() + 0), "k1");
	register_fpar("k2", (m_par.getCvDist() + 1), "k2");
	register_fpar("p1", (m_par.getCvDist() + 2), "p1");
	register_fpar("p2", (m_par.getCvDist() + 3), "p2");
	register_fpar("k3", (m_par.getCvDist() + 4), "k3");
	register_fpar("k4", (m_par.getCvDist() + 5), "k4");
	register_fpar("k5", (m_par.getCvDist() + 6), "k5");
	register_fpar("k6", (m_par.getCvDist() + 7), "k6");

	// fisheye camera model
	register_fpar("fisheye", &m_bFishEye, "Yes, use fisheye model.");
	register_fpar("recomp_ext", &m_bcalib_recompute_extrinsic, "Recompute extrinsic parameter.");
	register_fpar("chk_cond", &m_bcalib_check_cond, "Check condition.");
	register_fpar("fix_skew", &m_bcalib_fix_skew, "Fix skew.");
	register_fpar("fix_int", &m_bcalib_fix_intrinsic, "Fix intrinsic parameters.");
	register_fpar("fek1", (m_par.getCvDistFishEye() + 0), "k1 for fisheye");
	register_fpar("fek2", (m_par.getCvDistFishEye() + 1), "k2 for fisheye");
	register_fpar("fek3", (m_par.getCvDistFishEye() + 2), "k3 for fisheye");
	register_fpar("fek4", (m_par.getCvDistFishEye() + 3), "k4 for fisheye");

	// command
	register_fpar("del", &m_bdel, "Delete selected chessboard.");
	register_fpar("det", &m_bdet, "Detect Chessboard");
	register_fpar("calib", &m_bcalib, "Calibrate.");
	register_fpar("save", &m_bsave, "Save parameters.");
	register_fpar("load", &m_bload, "Load parameters.");
	register_fpar("undist", &m_bundist, "Undistort.");

	pthread_mutex_init(&m_mtx, NULL);
}

f_glfw_calib::~f_glfw_calib()
{
}

bool f_glfw_calib::init_run()
{
	if(m_chin.size() == 0)
		return false;

	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_pin == NULL)
		return false;

	if(!m_model_chsbd.load()){
		cerr << "Failed to setup chessboard model with the model file " << m_model_chsbd.fname << endl;
		return false;
	}

	if (!f_glfw_window::init_run())
		return false;

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(120, (double)m_sz_win.width / (double)m_sz_win.height, 0.1, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, -1, 0, 0, 0, 0, -1, 0);

	m_objs.resize(m_num_chsbds, NULL);
	m_score.resize(m_num_chsbds);
	m_upt2d.resize(m_num_chsbds);
	m_upt2dprj.resize(m_num_chsbds);
	m_num_chsbds_det = 0;

	m_dist_chsbd = Mat::zeros(m_hist_grid.height, m_hist_grid.width, CV_32SC1);

	m_par.setFishEye(m_bFishEye);

	return true;
}

bool f_glfw_calib::proc()
{
	if(glfwWindowShouldClose(pwin()))
		return false;

	long long timg;
	Mat img = m_pin->get_img(timg);


	bool bnew = true;
	if (img.empty()){
		if (m_fsmplimg[0]){
			img = imread(m_fsmplimg);
		}
		if (img.empty())
			return true;
		if (m_img_det.empty())
			bnew = true;
		else
			bnew = false;
	}
	if (m_timg == timg)
		bnew = false;
		
	// if the detecting thread is not active, convert the image into a grayscale one, and invoke detection thread.
	if((bnew || m_bcalib || m_bdet) && !m_bthact){
		m_bthact = true;

		if (img.channels() == 3)
			cvtColor(img, m_img_det, CV_RGB2GRAY);
		else
			m_img_det = img.clone();

		pthread_create(&m_thwork, NULL, thwork, (void*) this);
	}

	// calculating contrast
	calc_contrast(img);

	if(m_bsave){
		save();
	}

	if(m_bload){
		load();
	}

	if(m_bdel){
		del();
	}

	if (bnew && m_bundist){
		pthread_mutex_lock(&m_mtx);
		remap(img, img, m_map1, m_map2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
		pthread_mutex_unlock(&m_mtx);
	}

	m_timg = timg;

	// the image is completely fitted to the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	Size sz_img(img.cols, img.rows);
	Size sz_win = m_sz_win;
	if (m_b3dview){
		sz_win.width = m_sz_win.width >> 2;
		sz_win.height = m_sz_win.height >> 2;
	}

	if (sz_img.width != sz_win.width || sz_img.height != sz_win.height){
		Mat tmp;
		resize(img, tmp, sz_win);
		img = tmp;
	}
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2i(-1, -1);

	if (img.channels() == 3){
		cnvCVBGR8toGLRGB8(img);
		glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);
	}
	else{
		cnvCVGRAY8toGLGRAY8(img);
		glDrawPixels(img.cols, img.rows, GL_INTENSITY, GL_UNSIGNED_BYTE, img.data);
	}
	if (!m_b3dview){
		if (m_bshow_chsbd_all){
			for (int iobj = 0; iobj < m_objs.size(); iobj++){
				if (!m_objs[iobj])
					continue;
				drawCvChessboard(sz_img, m_objs[iobj]->pt2d, 0.5, 0.0, 0.0, 0.5, 3, 1);
				if (m_bcalib_done)
					drawCvChessboard(sz_img, m_objs[iobj]->pt2dprj, 0.0, 0.0, 0.5, 0.5, 3, 1);
			}
		}

		if (m_bshow_chsbd_sel){
			pthread_mutex_lock(&m_mtx);
			if (m_sel_chsbd >= 0 && m_sel_chsbd < m_objs.size() && m_objs[m_sel_chsbd]){
				drawCvChessboard(sz_img, m_objs[m_sel_chsbd]->pt2d, 1.0, 0.0, 0.0, 1.0, 3, 2);
				if (m_bcalib_done)
					drawCvChessboard(sz_img, m_objs[m_sel_chsbd]->pt2dprj, 0.0, 0.0, 1.0, 1.0, 3, 2);
			}
			pthread_mutex_unlock(&m_mtx);
		}
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	// overlay chessboard if needed
	if (m_b3dview && m_bcalib_done){
		Point3f p0, p1, p2, p3;
		GLdouble m[16];
		// render camera
		Mat P = m_par.getCvPrjMat();
		double fx = P.at<double>(0, 0), fy = P.at<double>(1, 1);
		double wc = 0.05 * (double)m_img_det.cols / fx, hc = 0.05 * (double)m_img_det.rows / fy;
		p0 = Point3f((float)wc,(float)hc, 0.1f);
		p1 = Point3f((float)-wc, (float)hc, 0.1f);
		p2 = Point3f((float)-wc, (float)-hc, 0.1f);
		p3 = Point3f((float)wc, (float)-hc, 0.1f);
		glColor3f(1., 0., 0.);

		glBegin(GL_LINES);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(0, 0, 0);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(0, 0, 0);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(0, 0, 0);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(0, 0, 0);
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p3.x, p3.y, p3.z);
		glVertex3f(p0.x, p0.y, p0.z);
		glEnd();

		// render chessboard
		int wcb = m_model_chsbd.par_chsbd.w;
		int hcb = m_model_chsbd.par_chsbd.h;
		p0 = m_model_chsbd.pts[0];
		p1 = m_model_chsbd.pts[wcb - 1];
		p2 = m_model_chsbd.pts[(hcb - 1) * wcb];
		p3 = m_model_chsbd.pts[hcb * wcb - 1];
		glMatrixMode(GL_MODELVIEW);
		for (int iobj = 0; iobj < m_objs.size(); iobj++){
			if (!m_objs[iobj])
				continue;
			glPushMatrix();
			/*
			glGetDoublev(GL_PROJECTION_MATRIX, m);
			cout << "Projection Matrix:" << endl;
			printGlMatrix(m);
			glGetDoublev(GL_MODELVIEW_MATRIX, m);
			cout << "Model Matrix:" << endl;
			printGlMatrix(m);
			*/
			cnvCvRTToGlRT(m_objs[iobj]->rvec, m_objs[iobj]->tvec, m);
			glMultMatrixd(m);
			/*
			glGetDoublev(GL_PROJECTION_MATRIX, m);
			cout << "Projection Matrix:" << endl;
			printGlMatrix(m);
			glGetDoublev(GL_MODELVIEW_MATRIX, m);
			cout << "Model Matrix:" << endl;
			printGlMatrix(m);
			*/
			glColor3f(0., 0., 1.);
			glBegin(GL_LINES);
			glVertex3f(p0.x, p0.y, p0.z);
			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p3.x, p2.y, p2.z);
			glVertex3f(p3.x, p2.y, p2.z);
			glVertex3f(p2.x, p3.y, p3.z);
			glVertex3f(p2.x, p3.y, p3.z);
			glVertex3f(p0.x, p0.y, p0.z);
			glEnd();
			glPopMatrix();
		}
	}
	else{
	}

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	// overlay information (Number of chessboard, maximum, minimum, average scores, reprojection error)
	float hfont = (float)(24. / (float)img.rows);
	float wfont = (float)(24. / (float)img.cols);
	float x = (float)(wfont - 1.0);
	float y = (float)(hfont - 1.0);
	char buf[1024];

	// calculating chessboard's stats
	double smax = 0., smin = DBL_MAX, savg = 0.;
	for (int i = 0; i < m_num_chsbds; i++){
		if (!m_objs[i])
			continue;

		double s = m_score[i].tot;
		smax = max(smax, s);
		smin = min(smin, s);
		savg += s;
	}

	savg /= (double)m_num_chsbds_det;

	snprintf(buf, 1024, "Nchsbd= %d/%d, Smax=%2.2e Smin=%2.2e Savg=%2.2f Erep=%2.2f Cnt=%2.2f %s %s",
		m_num_chsbds_det, m_num_chsbds, smax, smin, savg, m_rep_avg, m_contrast, 
		m_bdet ? "det": "nop", m_bcalib ? "cal" : "nop");
	drawGlText(x, y, buf, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
	y += 2 * hfont;

	// indicating infromation about selected chessboard.
	if (m_bshow_chsbd_sel && m_sel_chsbd >= 0 && m_sel_chsbd < m_num_chsbds && m_objs[m_sel_chsbd]){

		snprintf(buf, 255, "Score: %f (Crn: %f Sz %f Angl %f Erep %f  dist: %f)",
			m_score[m_sel_chsbd].tot, m_score[m_sel_chsbd].crn, m_score[m_sel_chsbd].sz,
			m_score[m_sel_chsbd].angl, m_score[m_sel_chsbd].rep, m_score[m_sel_chsbd].dist);
		drawGlText(x, y, buf, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
		y += 2 * hfont;
		snprintf(buf, 255, "Chessboard %d", m_sel_chsbd);
		drawGlText(x, y, buf, 0, 1, 0, 1, GLUT_BITMAP_TIMES_ROMAN_24);
		y += 2 * hfont;
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glfwSwapBuffers(pwin());
	glfwPollEvents();

	return true;
}

void * f_glfw_calib::thwork(void * ptr)
{
	f_glfw_calib * pclb = (f_glfw_calib*) ptr;
	if (pclb->m_bdet){
		pclb->detect();
	}

	//pclb->m_bdet = false;
	if (pclb->m_bcalib){
		pclb->calibrate();
	}
	pclb->m_bcalib = false;
	pclb->m_bthact = false;
	return NULL;
}

void * f_glfw_calib::detect()
{
	if(m_hist_grid.width != m_dist_chsbd.cols || 
		m_hist_grid.height != m_dist_chsbd.rows)
		refresh_chsbd_dist();

	if(m_bdet){
		// detect chessboard 
		s_obj * pobj = m_model_chsbd.detect(m_img_det);
		if(!pobj) 
			return NULL;

		double scrn = calc_corner_contrast_score(m_img_det, pobj->pt2d);

		// replacing the chessboard if possible.
		bool is_changed = false;
		int iobj_worst = -1;
		double sc_worst = DBL_MAX;
		for (int i = 0; i < m_num_chsbds; i++){
			if (m_objs[i] == NULL){
				pthread_mutex_lock(&m_mtx);
				m_objs[i] = pobj;
				m_score[i].crn = scrn;
				m_upt2d[i].clear();
				is_changed = true;
				m_num_chsbds_det++;
				pthread_mutex_unlock(&m_mtx);
				break;
			}
			if (sc_worst > m_score[i].tot){
				sc_worst = m_score[i].tot;
				iobj_worst = i;
			}
		}

		if (!is_changed){// if there is no empty slot
			pthread_mutex_lock(&m_mtx);
			double scrn_tmp = m_score[iobj_worst].crn;
			s_obj * pobj_tmp = m_objs[iobj_worst];
			m_score[iobj_worst].crn = scrn;
			double prev_tot_score = m_tot_score;
			m_objs[iobj_worst] = pobj;
			calc_chsbd_score();

			if (prev_tot_score > m_tot_score){
				m_objs[iobj_worst] = pobj_tmp;	
				m_score[iobj_worst].crn = scrn_tmp;
				calc_chsbd_score();
				delete pobj;
			}
			else {
				delete pobj_tmp;
				m_upt2d[iobj_worst].clear();
				m_upt2d[iobj_worst].clear();
			}
			pthread_mutex_unlock(&m_mtx);
		}
		else{
			calc_chsbd_score();
		}
	}

	return NULL;
}

// The size score is the area of the largest cell.
// The angle score is the ratio of the area of largest cell and that of the smallest cell.
void f_glfw_calib::calc_size_and_angle_score(vector<Point2f> & pts, double & ssz, double & sangl)
{
	Size sz = Size(m_model_chsbd.par_chsbd.w, m_model_chsbd.par_chsbd.h);
	double amax = 0, amin = DBL_MAX;
	double a;
	Point2f v1, v2;

	int i0, i1, i2;

	// left-top corner
	i0 = 0;
	i1 = 1;
	i2 = sz.width;
	v1 = pts[i1] - pts[i0];
	v2 = pts[i2] - pts[i0];

	// Eventhoug the projected grid is not the parallelogram, we now approximate it as a parallelogram.
	a = abs(v1.x * v2.y - v1.y * v2.x); 
	amax = max(a, amax);
	amin = min(a, amin);

	// left-bottom corner
	i0 = sz.width * (sz.height - 1);
	i1 = i0 + 1;
	i2 = i0 - sz.width;
	v1 = pts[i1] - pts[i0];
	v2 = pts[i2] - pts[i0];
	a = abs(v1.x * v2.y - v1.y * v2.x);
	amax = max(a, amax);
	amin = min(a, amin);

	// right-top corner
	i0 = sz.width - 1;
	i1 = i0 - 1;
	i2 = i0 + sz.width;
	v1 = pts[i1] - pts[i0];
	v2 = pts[i2] - pts[i0];
	a = abs(v1.x * v2.y - v1.y * v2.x);
	amax = max(a, amax);
	amin = min(a, amin);

	// right-bottom corner
	i0 = sz.width * sz.height - 1;
	i1 = i0 - 1;
	i2 = i0 - sz.width;
	v1 = pts[i1] - pts[i0];
	v2 = pts[i2] - pts[i0];
	a = abs(v1.x * v2.y - v1.y * v2.x);
	amax = max(a, amax);
	amin = min(a, amin);
	ssz = amax;
	sangl = amax / amin;
}

// calculating average of contrasts of pixels around chessboard's points.
// the contrast of a point is calculated (Imax-Imin) / (Imax+Imin) for pixels around the point
double f_glfw_calib::calc_corner_contrast_score(Mat & img, vector<Point2f> & pts)
{
	uchar * pix = img.data;
	uchar vmax = 0, vmin = 255;
	double csum = 0.;
	//Note that the contrast of the point is calculated in 5x5 image patch centered at the point. 
	
	int org = - 2 * img.cols - 2;
	for(int i = 0; i < pts.size(); i++){
		Point2f & pt = pts[i];

		if (pt.x < 5 || (pt.x >= img.cols - 5) 
			|| pt.y < 5 || (pt.y >= img.rows - 5))
			continue;

		pix = img.data + (int) (img.cols * pt.y + pt.x + 0.5) + org;
		for(int y = 0; y < 5; y++){
			for(int x = 0; x < 5; x++, pix++){
				vmax = max(vmax, *pix);
				vmin = min(vmin, *pix);
			}
			pix += img.cols - 5;
		}
		csum += (double)(vmax  - vmin) / (double)((int)vmax + (int)vmin);
	}

	return csum / (double) pts.size();
}

void f_glfw_calib::add_chsbd_dist(vector<Point2f> & pts)
{
	double wstep = m_img_det.cols / m_hist_grid.width;
	double hstep = m_img_det.rows / m_hist_grid.height;
	for(int i = 0; i < pts.size(); i++){
		int x, y;
		x = (int) (pts[i].x / wstep);
		y = (int) (pts[i].y / hstep);
		m_dist_chsbd.at<int>(x, y) += 1;
	}
}

void f_glfw_calib::sub_chsbd_dist(vector<Point2f> & pts)
{
	double wstep = m_img_det.cols / m_hist_grid.width;
	double hstep = m_img_det.rows / m_hist_grid.height;
	for(int i = 0; i < pts.size(); i++){
		int x, y;
		x = (int) (pts[i].x / wstep);
		y = (int) (pts[i].y / hstep);
		m_dist_chsbd.at<int>(x, y) -= 1;
	}
}

void f_glfw_calib::refresh_chsbd_dist()
{
	m_dist_chsbd = Mat::zeros(m_hist_grid.height, m_hist_grid.width, CV_32SC1);
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		s_obj * pobj = m_objs[iobj];
		if (pobj)
			add_chsbd_dist(pobj->pt2d);
	}
}

double f_glfw_calib::calc_chsbd_dist_score(vector<Point2f> & pts)
{
	double wstep = m_img_det.cols / m_hist_grid.width;
	double hstep = m_img_det.rows / m_hist_grid.height;
	int sum = 0;
	for(int i = 0; i < pts.size(); i++){
		int x, y;
		x = (int) (pts[i].x / wstep);
		y = (int) (pts[i].y / hstep);
		sum += m_dist_chsbd.at<int>(x, y);
	}
	return 1.0 / ((double) sum + 0.001);
}

void f_glfw_calib::calc_chsbd_dist_score()
{
	refresh_chsbd_dist();
	for(int iobj = 0; iobj < m_num_chsbds; iobj++){
		if (!m_objs[iobj])
			continue;
		s_obj * pobj = m_objs[iobj];
		m_score[iobj].dist = calc_chsbd_dist_score(pobj->pt2d);
	}
}

void f_glfw_calib::calc_chsbd_score()
{
	calc_chsbd_dist_score();
	double ssz, sangl;
	for (int iobj = 0; iobj < m_objs.size(); iobj++){
		if (!m_objs[iobj])
			continue;

		calc_size_and_angle_score(m_objs[iobj]->pt2d, ssz, sangl);
		m_score[iobj].sz = ssz;
		m_score[iobj].angl = sangl;
	}

	m_tot_score = 0;
	for (int iobj = 0; iobj < m_objs.size(); iobj++){
		calc_tot_score(m_score[iobj]);
		m_tot_score += m_score[iobj].tot;
	}
}

int f_glfw_calib::gen_calib_flag()
{
	int flag = 0;
	if(m_bFishEye){
		flag |= (m_bcalib_recompute_extrinsic ? fisheye::CALIB_RECOMPUTE_EXTRINSIC : 0);
		flag |= (m_bcalib_check_cond ? fisheye::CALIB_CHECK_COND : 0);
		flag |= (m_bcalib_fix_skew ? fisheye::CALIB_FIX_SKEW : 0);
		flag |= (m_bcalib_fix_intrinsic ? fisheye::CALIB_FIX_INTRINSIC : 0);
		flag |= (m_bcalib_fix_k1 ? fisheye::CALIB_FIX_K1 : 0);
		flag |= (m_bcalib_fix_k2 ? fisheye::CALIB_FIX_K2 : 0);
		flag |= (m_bcalib_fix_k3 ? fisheye::CALIB_FIX_K3 : 0);
		flag |= (m_bcalib_fix_k4 ? fisheye::CALIB_FIX_K4 : 0);
	}else{
		flag |= (m_bcalib_use_intrinsic_guess ? CV_CALIB_USE_INTRINSIC_GUESS : 0);
		flag |= (m_bcalib_fix_principal_point ? CV_CALIB_FIX_PRINCIPAL_POINT : 0);
		flag |= (m_bcalib_fix_aspect_ratio ? CV_CALIB_FIX_ASPECT_RATIO : 0);
		flag |= (m_bcalib_zero_tangent_dist ? CV_CALIB_ZERO_TANGENT_DIST : 0);
		flag |= (m_bcalib_fix_k1 ? CV_CALIB_FIX_K1 : 0);
		flag |= (m_bcalib_fix_k2 ? CV_CALIB_FIX_K2 : 0);
		flag |= (m_bcalib_fix_k3 ? CV_CALIB_FIX_K3 : 0);
		flag |= (m_bcalib_fix_k4 ? CV_CALIB_FIX_K4 : 0);
		flag |= (m_bcalib_fix_k5 ? CV_CALIB_FIX_K5 : 0);
		flag |= (m_bcalib_fix_k6 ? CV_CALIB_FIX_K6 : 0);
		flag |= (m_bcalib_rational_model ? CV_CALIB_RATIONAL_MODEL : 0);
	}
	return flag;
}

void f_glfw_calib::calibrate()
{
	if (m_num_chsbds_det != m_num_chsbds)
		return;

	m_par.setFishEye(m_bFishEye);

	Size sz_chsbd(m_model_chsbd.par_chsbd.w, m_model_chsbd.par_chsbd.h);
	int num_pts = sz_chsbd.width * sz_chsbd.height;

	vector<vector<Point2f>> pt2ds;
	vector<vector<Point3f>> pt3ds;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	int flags = gen_calib_flag();

	pt2ds.resize(m_num_chsbds_det);
	pt3ds.resize(m_num_chsbds_det);
	int i = 0;
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		if(m_objs[iobj] == NULL)
			continue;
		pt2ds[i] = m_objs[i]->pt2d;
		pt3ds[i] = m_model_chsbd.pts;
		i++;
	}
	
	Mat P, D;
	if(m_bFishEye){
		if (m_bcalib_use_intrinsic_guess){
			P = m_par.getCvPrjMat().clone();
			D = m_par.getCvDistFishEyeMat().clone();
		}
		rvecs.resize(m_num_chsbds_det);
		tvecs.resize(m_num_chsbds_det);
		for (int iobj = 0; iobj < m_objs.size(); iobj++){
			rvecs[iobj] = Mat(1, 1, CV_64FC3);
			tvecs[iobj] = Mat(1, 1, CV_64FC3);
		}

		m_Erep = fisheye::calibrate(pt3ds, pt2ds, sz_chsbd, P, D,
			rvecs, tvecs, flags);
	}else{
		if (m_bcalib_use_intrinsic_guess){
			P = m_par.getCvPrjMat().clone();
			D = m_par.getCvDistMat();
		}
		m_Erep = calibrateCamera(pt3ds, pt2ds, sz_chsbd,
			P, D, rvecs, tvecs, flags);
	}

	// copying obtained camera parameter to m_par
	pthread_mutex_lock(&m_mtx);
	m_par.setCvPrj(P);
	m_par.setCvDist(D);

	m_bcalib_done = true;

	for (int iobj = 0, icount = 0; iobj < m_objs.size(); icount++, iobj++){
		if (m_objs[iobj] == NULL)
			continue;

		m_objs[iobj]->rvec = rvecs[icount];
		m_objs[iobj]->tvec = tvecs[icount];
	}
	init_undistort();
	pthread_mutex_unlock(&m_mtx);
}

void f_glfw_calib::init_undistort()
{
	// calculating reprojection error 
	double rep_tot = 0.;
	for (int iobj = 0, icount = 0; iobj < m_objs.size(); iobj++){
		s_obj * pobj = m_objs[iobj];
		if (!pobj)
			continue;

		vector<Point2f> & pt2dprj = pobj->pt2dprj;
		vector<Point2f> & pt2d = pobj->pt2d;
		int num_pts = (int) pt2d.size();

		if (m_bFishEye){
			fisheye::projectPoints(m_model_chsbd.pts, pt2dprj, pobj->rvec, pobj->tvec,
				m_par.getCvPrjMat(), m_par.getCvDistFishEyeMat());
		}
		else{
			prjPts(m_model_chsbd.pts, pt2dprj,
				m_par.getCvPrjMat(), m_par.getCvDistMat(),
				pobj->rvec, pobj->tvec);
		}
		double sum_rep = 0.;
		for (int ipt = 0; ipt < pt2d.size(); ipt++){
			Point2f & ptprj = pt2dprj[ipt];
			Point2f & pt = pt2d[ipt];
			Point2f ptdiff = pt - ptprj;
			double rep = ptdiff.x * ptdiff.x + ptdiff.y * ptdiff.y;
			sum_rep += rep;
		}
		rep_tot += sum_rep;
		m_score[iobj].rep = sqrt(sum_rep / (double)num_pts);
	}

	m_rep_avg = rep_tot / (double)(m_num_chsbds_det);

	Size sz = Size(m_img_det.cols, m_img_det.rows);
	Mat R = Mat::eye(3, 3, CV_64FC1);
	Mat P = m_par.getCvPrjMat();
	if (m_par.isFishEye()){
		Mat K, D;
		K = m_par.getCvPrjMat().clone();
		D = m_par.getCvDistFishEyeMat().clone();

		//fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, sz, R, P);
		fisheye::initUndistortRectifyMap(K, D, R, P, sz, CV_32FC1, m_map1, m_map2);
		for (int iobj = 0; iobj < m_objs.size(); iobj++){
			if (m_objs[iobj]){
				m_upt2d[iobj].resize(m_objs[iobj]->pt2d.size());
				m_upt2dprj[iobj].resize(m_objs[iobj]->pt2d.size());
				fisheye::undistortPoints(m_objs[iobj]->pt2d, m_upt2d[iobj], K, D);
				fisheye::undistortPoints(m_objs[iobj]->pt2dprj, m_upt2dprj[iobj], K, D);
			}
		}
	}
	else{
		Mat K, D;
		K = m_par.getCvPrjMat().clone();
		D = m_par.getCvDistMat().clone();

		//P = getOptimalNewCameraMatrix(K, D, sz, 1.);
		initUndistortRectifyMap(K, D, R, P, sz, CV_32FC1, m_map1, m_map2);
		
		for (int iobj = 0; iobj < m_objs.size(); iobj++){
			if (m_objs[iobj]){
				vector<Point2f> upt2d;
				m_upt2d[iobj].resize(m_objs[iobj]->pt2d.size());
				m_upt2dprj[iobj].resize(m_objs[iobj]->pt2d.size());
				undistortPoints(m_objs[iobj]->pt2d, m_upt2d[iobj], K, D);
				undistortPoints(m_objs[iobj]->pt2dprj, m_upt2dprj[iobj], K, D);
			}
		}
	}
}

void f_glfw_calib::_key_callback(int key, int scancode, int action, int mods)
{
	switch(key){
	case GLFW_KEY_A:
		m_bshow_chsbd_all = !m_bshow_chsbd_all;
		break;
	case GLFW_KEY_B:
	case GLFW_KEY_C:
		m_bcalib = !m_bcalib;
		break;
	case GLFW_KEY_D:
		m_bdet = !m_bdet;
		break;
	case GLFW_KEY_E:
	case GLFW_KEY_F:
	case GLFW_KEY_G:
	case GLFW_KEY_H:
	case GLFW_KEY_I:
	case GLFW_KEY_J:
	case GLFW_KEY_K:
	case GLFW_KEY_L:
		if(mods & GLFW_MOD_CONTROL){
			m_bload = true;
		}
		break;
	case GLFW_KEY_M:
	case GLFW_KEY_N:
	case GLFW_KEY_O:
	case GLFW_KEY_P:
	case GLFW_KEY_Q:
	case GLFW_KEY_R:
	case GLFW_KEY_S:
		if(mods & GLFW_MOD_CONTROL){
			m_bsave = true;
		}
		break;
	case GLFW_KEY_T:
	case GLFW_KEY_U:
	case GLFW_KEY_V:
	case GLFW_KEY_W:
	case GLFW_KEY_X:
	case GLFW_KEY_Y:
	case GLFW_KEY_Z:
	case GLFW_KEY_RIGHT:
		pthread_mutex_lock(&m_mtx);
		m_sel_chsbd = (m_sel_chsbd + 1) % m_objs.size();
		pthread_mutex_unlock(&m_mtx);
		break;
	case GLFW_KEY_LEFT:
		pthread_mutex_lock(&m_mtx);
		m_sel_chsbd = (m_sel_chsbd + (int)m_objs.size() - 1) % (int)m_objs.size();
		pthread_mutex_unlock(&m_mtx);
		break;
	case GLFW_KEY_UP:
	case GLFW_KEY_DOWN:
	case GLFW_KEY_SPACE:
	case GLFW_KEY_BACKSPACE:
	case GLFW_KEY_ENTER:
	case GLFW_KEY_DELETE:
		m_bdel = true;
		break;
	case GLFW_KEY_TAB:
	case GLFW_KEY_HOME:
	case GLFW_KEY_END:
	case GLFW_KEY_F1:
		m_bshow_chsbd_all = !m_bshow_chsbd_all;
		break;
	case GLFW_KEY_F2:
		m_bshow_chsbd_sel = !m_bshow_chsbd_sel;
		break;
	case GLFW_KEY_F3:
		m_bshow_chsbd_density = !m_bshow_chsbd_density;
		break;
	case GLFW_KEY_F4:
	case GLFW_KEY_F5:
	case GLFW_KEY_F6:
	case GLFW_KEY_F7:
	case GLFW_KEY_F8:
	case GLFW_KEY_F9:
	case GLFW_KEY_F10:
	case GLFW_KEY_F11:
	case GLFW_KEY_F12:
	case GLFW_KEY_UNKNOWN:
		break;
	}
}


void f_glfw_calib::save()
{
	pthread_mutex_lock(&m_mtx);
	if (m_fcampar[0] == '\0' || !m_par.write(m_fcampar)){
		cerr << "Failed to save camera parameters into " << m_fcampar << endl;
	}

	FileStorage fs;
	fs.open(m_fcbdet, FileStorage::WRITE);
	if (!fs.isOpened()){
		cerr << "Failed to open file " << m_fcbdet << endl;
	}
	else{
		fs << "NumChsbd" << m_num_chsbds_det;
		fs << "ChsbdName" << m_model_chsbd.name;
		int chsbd_pts = (int)m_model_chsbd.pts.size();
		char item[1024];
		int iobj = 0;
		for (int i = 0; i < m_num_chsbds; i++){
			if (!m_objs[i])
				continue;
			snprintf(item, 1024, "chsbdpt%d", iobj);
			fs << item << "[";
			for (int k = 0; k < m_objs[iobj]->pt2d.size(); k++){
				fs << m_objs[i]->pt2d[k];
			}
			fs << "]";
			snprintf(item, 1024, "chsbdsc%d", iobj);
			fs << item << "[";
			fs << m_score[i].tot << m_score[i].sz << m_score[i].angl
				<< m_score[i].crn << m_score[i].dist << m_score[i].rep;
			fs << "]";
			snprintf(item, 1024, "chsbdat%d", i);
			fs << item << "[";
			fs << m_objs[i]->tvec << m_objs[i]->rvec;
			fs << item << "]";
			iobj++;
		}
	}
	pthread_mutex_unlock(&m_mtx);
	m_bsave = false;

}

void f_glfw_calib::load()
{
	pthread_mutex_lock(&m_mtx);
	if (m_fcampar[0] == '\0' || !m_par.read(m_fcampar)){
		cerr << "Failed to load camera parameters from " << m_fcampar << endl;
	}

	FileStorage fs(m_fcbdet, FileStorage::READ);
	FileNode fn;
	string str;
	if (!fs.isOpened()){
		cerr << "Failed to open " << m_fcampar << endl;
		m_bload = false;
		pthread_mutex_unlock(&m_mtx);
		return;
	}

	fn = fs["NumChsbd"];
	if (fn.empty()){
		cerr << "Item \"NumChsbd\" was not found." << endl;
		goto finish;
	}

	fn >> m_num_chsbds_det;

	fn = fs["ChsbdName"];
	if (fn.empty()){
		cerr << "Item \"ChsbdName\" was not found." << endl;
		goto finish;
	}
	fn >> str;
	if (str != m_model_chsbd.name){
		cerr << "Miss match in chessboard model." << endl;
		goto finish;
	}
	char item[1024];
	for (int i = 0; i < m_num_chsbds_det; i++){
		snprintf(item, 1024, "chsbdpt%d", i);
		fn = fs[(char*)item];
		if (fn.empty()){
			cerr << item << " was not found" << endl;
			goto finish;
		}
		FileNodeIterator itr = fn.begin();
		s_obj * pobj = new s_obj;
		m_objs[i] = pobj;

		pobj->pmdl = &m_model_chsbd;

		pobj->pt2d.resize(m_model_chsbd.pts.size());
		for (int k = 0; itr != fn.end(); itr++, k++){
			*itr >> pobj->pt2d[k];
		}

		snprintf(item, 1024, "chsbdsc%d", i);
		fn = fs[(char*)item];
		if (fn.empty()){
			cerr << item << " was not found" << endl;
			goto finish;
		}
		itr = fn.begin();
		*itr >> m_score[i].tot; itr++;
		*itr >> m_score[i].sz; itr++;
		*itr >> m_score[i].angl; itr++;
		*itr >> m_score[i].crn; itr++;
		*itr >> m_score[i].dist; itr++;
		*itr >> m_score[i].rep;;

		snprintf(item, 1024, "chsbdat%d", i);
		fn = fs[(char*)item];
		if (fn.empty()){
			cerr << item << " was not found" << endl;
			goto finish;
		}
		itr = fn.begin();
		*itr >> pobj->tvec;
		itr++;
		*itr >> pobj->rvec;

	}
finish:
	init_undistort();
	calc_chsbd_score();
	pthread_mutex_unlock(&m_mtx);
	m_bload = false;
	m_bcalib_done = true;
}

void f_glfw_calib::del()
{
	pthread_mutex_lock(&m_mtx);

	if (m_sel_chsbd >= 0 && m_sel_chsbd < m_objs.size() && m_objs[m_sel_chsbd]){
		delete[] m_objs[m_sel_chsbd];
		m_score[m_sel_chsbd] = s_chsbd_score();
		m_objs[m_sel_chsbd] = NULL;
		m_upt2d[m_sel_chsbd].clear();
		m_upt2dprj[m_sel_chsbd].clear();
		m_num_chsbds_det--;
	}
	pthread_mutex_unlock(&m_mtx);
	m_bdel = false;
}

void f_glfw_calib::calc_contrast(Mat & img)
{
	double res;
	if (img.channels() == 3){
		int rmax, gmax, bmax;
		int rmin, gmin, bmin;
		rmax = gmax = bmax = 0;
		rmin = gmin = gmax = 255;
		MatIterator_<Vec3b> itr = img.begin<Vec3b>();
		MatIterator_<Vec3b> end = img.end<Vec3b>();
		for (; itr != end; itr++){
			rmax = max((int)(*itr)[0], rmax);
			gmax = max((int)(*itr)[1], gmax); 
			bmax = max((int)(*itr)[2], bmax);
			rmin = min((int)(*itr)[0], rmin);
			gmin = min((int)(*itr)[1], gmin);
			bmin = min((int)(*itr)[2], bmin);
		}

		res = (double)(rmax + gmax + bmax - rmin - gmin - bmin) /
			(double)(rmax + gmax + bmax + rmin + gmin + bmin);
	}
	else{
		MatIterator_<uchar> itr = img.begin<uchar>();
		MatIterator_<uchar> end = img.end<uchar>();
		int vmax, vmin;
		vmax = 0;
		vmin = 255;
		for (; itr != end; itr++){
			vmax = max((int)(*itr), vmax);
			vmin = min((int)(*itr), vmin);
		}
		res = (double)(vmax - vmin) / (double)(vmax + vmin);
	}

	m_contrast = res;
}


////////////////////////////////////////////////////////////////////// f_gflw_test3d

const char * f_glfw_test3d::strings[5] = {
	"                                                  ",
	"Rotation",
	"Translation",
	"1st line\n2nd line",
	"Color change"
};

f_glfw_test3d::f_glfw_test3d(const char * name) : f_glfw_window(name), n(1), deg(0), tprev(0)
{
	register_fpar("n", &n, "number of division");
	register_fpar("fvs", fvs, 1024, "File of the vertex shader.");
	register_fpar("ffs", ffs, 1024, "File of the fragment shader.");
	register_fpar("fetex", fetex, 1024, "File of the earth texture.");

	register_fpar("ffnttex", ffnttex, 1024, "File of the font texture");
	register_fpar("ffntcfg", ffntcfg, 1024, "File of the font setting file.");

	pthread_mutex_init(&mtx_mouse, NULL);
}

f_glfw_test3d::~f_glfw_test3d()
{
	pthread_mutex_destroy(&mtx_mouse);
}

bool f_glfw_test3d::init_run()
{

	if (!f_glfw_window::init_run()){
		return false;
	}
	
	if (!setup_shader()){
		cerr << "Failed to setup shader." << endl;
		return false;
	}

	c_icosahedron * p0, *p1;
	p0 = new c_icosahedron();

	for (int i = 0; i < n; i++){
		p1 = new c_icosahedron(*p0);
		delete p0;
		p0 = p1;
	}
	nf = p0->get_nf();

	if (!setup_objects(p0)){
		cerr << "Failed to setup bufferobjects." << endl;
		return false;
	}

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_TEXTURE_2D);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	return true;
}

char * f_glfw_test3d::load_glsl_text(const char * fname){
	ifstream fin(fname);
	if (!fin.is_open())
		return NULL;

	fin.seekg(0, fin.end);
	unsigned int flen = fin.tellg();
	fin.seekg(0, fin.beg);

	char * txt = new char [flen+1];
	if (!txt)
		return false;
	
	memset((void*)txt, 0, (size_t)(flen + 1));
	fin.read(txt, flen);
	return txt;
}

bool f_glfw_test3d::setup_shader()
{
	if (!load_glsl_program(ffs, fvs, p))
		return false;

	inv_sz_scrn_loc = glGetUniformLocation(p, "inv_sz_scrn");
	mvpl = glGetUniformLocation(p, "Mmvp");
	ml = glGetUniformLocation(p, "Mm");
	parl = glGetUniformLocation(p, "Lpar");
	smpl = glGetUniformLocation(p, "sampler");
	depth2dl = glGetUniformLocation(p, "depth2d");
	posl = glGetAttribLocation(p, "position");
	nml = glGetAttribLocation(p, "normal");
	texcdl = glGetAttribLocation(p, "texcoord");

	return true;
}

bool f_glfw_test3d::setup_objects(c_icosahedron * pic)
{
	struct s_vertex{
		float x, y, z;
		float nx, ny, nz;
		float u, v;
	} *vtx, *vtx_tmp;
	unsigned int * idx = new unsigned int[pic->get_nf() * 3];

	vtx = new s_vertex[pic->get_nv()];
	for (int iv = 0; iv < pic->get_nv(); iv++){
		Point3f v = pic->getv()[iv];
		vtx[iv].x = v.x;
		vtx[iv].y = v.y;
		vtx[iv].z = v.z;

		Point2f q = pic->getq()[iv];
		q.x += 0.5 * PI;
		q.y += PI;
		q.x = q.x * (1.0 / PI);
		q.y = q.y * (0.5 / PI);
		vtx[iv].u = q.y;
		vtx[iv].v = q.x;
		float inorm = (float)(1.0 / norm(v));
		v = inorm * v;
		vtx[iv].nx = v.x;
		vtx[iv].ny = v.y;
		vtx[iv].nz = v.z;
	}
	vector<s_vertex> vtxb;	
	for (int ifc = 0; ifc < pic->get_nf(); ifc++){
		unsigned int * f = pic->getf()[ifc];
		s_vertex &v0 = vtx[f[0]];
		s_vertex &v1 = vtx[f[1]];
		s_vertex &v2 = vtx[f[2]];
		bool b0 = false, b1 = false, b2 = false;
		
		if (fabs(v0.u - v1.u) > 0.8){
			if (v0.u < v1.u){
				b0 = true;
			}
			else{
				b1 = true;
			}
		}

		if (fabs(v0.u - v2.u) > 0.8){
			if (v0.u < v2.u){
				b0 = true;
			}
			else{
				b2 = true;
			}
		}

		if (fabs(v1.u - v2.u) > 0.8){
			if (v1.u < v2.u){
				b1 = true;
			}
			else{
				b2 = true;
			}
		}

		int bidx = vtxb.size() + pic->get_nv();
		if (b0){
			vtxb.push_back(v0);
			vtxb.back().u += 1.0;
			idx[ifc * 3] = bidx;
			bidx++;
		}
		else{
			idx[ifc * 3] = f[0];
		}

		if (b1){
			vtxb.push_back(v1);
			vtxb.back().u += 1.0;
			idx[ifc * 3 + 1] = bidx;
			bidx++;
		}
		else{
			idx[ifc * 3 + 1] = f[1];
		}

		if (b2){
			vtxb.push_back(v2);
			vtxb.back().u += 1.0;
			idx[ifc * 3 + 2] = bidx;
			bidx++;
		}
		else{
			idx[ifc * 3 + 2] = f[2];
		}
	}

	unsigned int nvertices = pic->get_nv() + vtxb.size();
	vtx_tmp = new s_vertex[nvertices];
	memcpy((void*)vtx_tmp, (const void*)vtx, sizeof(s_vertex)* pic->get_nv());
	delete[] vtx;
	vtx = vtx_tmp;
	vtx_tmp = vtx + pic->get_nv();
	for (int ibvtx = 0; ibvtx < vtxb.size(); ibvtx++){
		vtx_tmp[ibvtx] = vtxb[ibvtx];
	}
	vtxb.clear();

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(2, vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	float * v = (float*)pic->getv();
	glBufferData(GL_ARRAY_BUFFER, sizeof(s_vertex)* nvertices, vtx, GL_STATIC_DRAW);
	glVertexAttribPointer(posl, 3, GL_FLOAT, GL_FALSE,
		sizeof(s_vertex), 0);
	glVertexAttribPointer(nml, 3, GL_FLOAT, GL_FALSE, 
		sizeof(s_vertex), (const void*)(sizeof(float)* 3));
	glVertexAttribPointer(texcdl, 2, GL_FLOAT, GL_FALSE,
		sizeof(s_vertex), (const void*)(sizeof(float)* 6));
	glEnableVertexAttribArray(posl);
	glEnableVertexAttribArray(nml);
	glEnableVertexAttribArray(texcdl);

	delete[] vtx;

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int)* pic->get_nf() * 3, idx, GL_STATIC_DRAW);

	etex = imread(fetex);
	if (etex.empty()){
		cerr << "Error to open texture file " << fetex << endl;
		return false;
	}
	awsFlip(etex, false, true, false);

	// Preparing texture
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &hetex);
	glBindTexture(GL_TEXTURE_2D, hetex);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, /*GL_CLAMP_TO_EDGE*/ GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, /*GL_CLAMP_TO_EDGE*/ GL_REPEAT);

	struct s_pix{
		unsigned char r, g, b, a;
		s_pix() :r(255), g(255), b(255), a(255)
		{
		}
	} *tex = new s_pix[etex.cols * etex.rows];
	int i = 0;
	for (MatIterator_<Vec3b> itr = etex.begin<Vec3b>(); itr != etex.end<Vec3b>(); itr++){
		tex[i].a = 255;
		tex[i].r = (*itr)[2];
		tex[i].g = (*itr)[1];
		tex[i].b = (*itr)[0];
		i++;
	}

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, etex.cols, etex.rows, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, tex);
	delete[] tex;

	GLuint loc_mode_flag = glGetUniformLocation(p, "mode");
	GLuint loc_gcolor = glGetUniformLocation(p, "gcolor");
	GLuint loc_gcolorb = glGetUniformLocation(p, "gcolorb");
	GLuint loc_pos2d = glGetAttribLocation(p, "pos2d");

	otxt.init(ffnttex, ffntcfg, loc_mode_flag, loc_pos2d, texcdl, smpl, loc_gcolor, loc_gcolorb, depth2dl);
//	glm::vec2 sz_fnt = glm::vec2((float)(0.3 * 41.2890625 * 2.0 / (double)m_sz_win.width),
//		(float)(0.3 * 40.0 * 2.0 / (double)m_sz_win.height));
	glm::vec2 sz_fnt = glm::vec2((float)(41.2890625),
				(float)(40.0));
	glm::vec2 mgn = sz_fnt;
	float txt_pos = -100;
	for (int i = 0; i < 5; i++){
		hstr[i] = otxt.reserv(strlen(strings[i]) + 1);
		otxt.set(hstr[i], strings[i]);
		otxt.config(hstr[i], glm::vec4(0., 0., 0., 0.), glm::vec4(0, 0, 1, 1),
			sz_fnt, mgn, c_gl_text_obj::an_lt, glm::vec2(txt_pos, txt_pos), 0.);
		otxt.enable(hstr[i]);
		txt_pos += 100;
	}
	otxt.update_vertices();

	struct s_vertex2{
		float x, y, z;
	};
	s_vertex2  vtx2[8] = {
		{ 4e6, 4e6, 4e6 },		// 0 0 0
		{ -4e6, 4e6, 4e6 },		// 1 0 0
		{ -4e6, -4e6, 4e6 },	// 1 1 0
		{ 4e6, -4e6, 4e6 },		// 0 1 0
		{ 4e6, -4e6, -4e6 },	// 0 1 1
		{ -4e6, -4e6, -4e6 },	// 1 1 1
		{ -4e6, 4e6, -4e6 },	// 1 0 1
		{ 4e6, 4e6, -4e6 }		// 0 0 1
	};

	s_vertex2 vtx3[3] = {
		{ 4e6, 4e6, 4e6 },		// 0 0 0
		{ 4e6, 4e6, -4e6 },		// 0 0 1
		{ 4e6, -4e6, -4e6 }	// 0 1 1
	};

	s_vertex2 vtx4[2] = {
		{ 4e6, 4e6, 4e6 },		// 0 0 0
		{ 4e6, -4e6, 4e6 }		// 0 1 0
	};

	s_vertex2 vtx5[2] = {
		{ -4e6, -4e6, 4e6 },	// 1 1 0
		{ -4e6, -4e6, -4e6 }	// 1 1 1
	};

	oline.init(loc_mode_flag, posl, mvpl, loc_gcolor);
	oline.add(8, (const float*)vtx2);
	oline.add(3, (const float*)vtx3);
	oline.add(2, (const float*)vtx4);
	oline.add(2, (const float*)vtx5);
	oline.enable(0);
	oline.enable(1);
	oline.enable(2);
	oline.enable(3);
	oline.update_vertices();

	opts.init(loc_mode_flag, posl, mvpl, loc_gcolor);
	
	modeloc = loc_mode_flag;

	s_vertex2 vtx6[1000];
	srand(0);

	float r = 6400000, dr = 6800000 - r;
	for (int i = 0; i < 1000; i++)
	{
		float lat, lon, alt;
		lat = (((float)rand() / (float)RAND_MAX) - 0.5) * PI;
		lon = (((float)rand() / (float)RAND_MAX) - 0.5)  * 2.0 * PI;
		alt = (dr * ((float)rand() / (float)RAND_MAX)) + r;

		bihtoecef(lat, lon, alt, vtx6[i].x, vtx6[i].y, vtx6[i].z);
	}
	opts.add(1000, (const float*)vtx6);


	o2d.init_circle(loc_mode_flag, loc_pos2d, loc_gcolor, depth2dl, 10, 
		1.0, 1.0);
	o2d.add(glm::vec4(1.0, 0.0, 0.0, 1.0), glm::vec2(100, 100), 0, 150);
	o2d.add(glm::vec4(0.0, 1.0, 0.0, 1.0), glm::vec2(100, -100), 0, 150);
	o2d.add(glm::vec4(0.0, 0.0, 1.0, 1.0), glm::vec2(-100, -100), 0, 150);
	o2d.add(glm::vec4(1.0, 1.0, 0.0, 1.0), glm::vec2(-100, 100), 0, 150);
	o2d.config_border(0, false, 1.0);
	o2d.config_depth(0, 0);
	o2d.config_border(1, true, 1.0);
	o2d.config_depth(1, 1);
	o2d.config_border(2, true, 2.0);
	o2d.config_depth(2, 2);
	o2d.config_border(3, false, 2.0);
	o2d.config_depth(3, 3);
	o2d.update_vertices();
	return true;
}

void f_glfw_test3d::destroy_run()
{
	glDeleteBuffers(2, vbo);
	glDeleteVertexArrays(1, &vao);
	glDeleteProgram(p);

	return;
}

bool f_glfw_test3d::proc()
{
	if (tprev == 0)
		tprev = m_cur_time;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0, 0, 0, 1);

	glUseProgram(p);
	{
		float inv_sz_scrn[2] = { (float)(2. / (float)m_sz_win.width),
			(float)(2. / (float)m_sz_win.height) };
		glUniform2fv(inv_sz_scrn_loc, 1, inv_sz_scrn);
	}

	float ratio = (float)((float)m_sz_win.width / (float)m_sz_win.height);
	glm::mat4 pm = glm::perspective(55.f, ratio, 1.f, 30e6f);
	glm::mat4 vm = glm::lookAt(glm::vec3(0, 0, 10e6f), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	glm::mat4 rm(1.0);
	deg += 5. * (PI / (180. * (double) SEC)) * (double) (m_cur_time - tprev);
	tprev = m_cur_time;
	if (deg >= 2 * PI){
		deg = 0.0;
	}
	glm::mat4 rym(1.0);
	rym = glm::rotate(rym, (float)(0.5 * PI), glm::vec3(1, 0, 0));
	rm = glm::rotate(rm, deg, glm::vec3(0, 1, 0));
	rm = rm * rym;

	glm::mat4 pvm = pm * vm;
	glm::mat4 m = pvm * rm;
	glm::vec3 Lpar = glm::normalize(glm::vec3(1, 1, 1));
	glUniform1i(modeloc, 0);
	glUniformMatrix4fv(mvpl, 1, GL_FALSE, glm::value_ptr(m));
	glUniformMatrix4fv(ml, 1, GL_FALSE, glm::value_ptr(rm));
	glUniform3fv(parl, 1, glm::value_ptr(Lpar));

	glBindVertexArray(vao);
	glEnableVertexAttribArray(posl);
	glEnableVertexAttribArray(texcdl);
	glEnableVertexAttribArray(nml);

	glActiveTexture(GL_TEXTURE0);
	glUniform1i(smpl, 0);
	glBindTexture(GL_TEXTURE_2D, hetex);

	glDrawElements(GL_TRIANGLES, nf * 3, GL_UNSIGNED_INT, 0);

	otxt.set(hstr[0], m_time_str);
	otxt.config_rotation(hstr[1], deg);
	float c = cos(deg), s = sin(deg);
	glm::vec2 pos2d = glm::vec2((float)(0.2 * c), (float)(0.2 * s));
	otxt.config_position(hstr[2], pos2d);
	otxt.config_color(hstr[4], glm::vec4(c, -s, -c, s), glm::vec4(0, 0, 0, 0));
	otxt.render(0);	

	oline.config_rotation(rm);
	oline.config_position(glm::vec3(0.0, 0.0, 0.0));
	oline.config_width(1.0);
	oline.config_color(glm::vec4(1.0, 1.0, 0.0, 1.0));
	oline.render(pvm);

	opts.config_rotation(rm);
	opts.config_position(glm::vec3(0.0, 0.0, 0.0));
	opts.config_size(1.0);
	opts.config_color(glm::vec4(0.0, 1.0, 1.0, 1.0));
	opts.render(pvm);

	o2d.render();

	glfwSwapBuffers(pwin());
	glfwPollEvents();

	pthread_mutex_lock(&mtx_mouse);
	switch (mbtn){
	case GLFW_MOUSE_BUTTON_LEFT:
		if (mact == GLFW_PRESS){
			int ihandle = otxt.collision(point_cursor);
			if (ihandle >= 0)
				cout << "String[" << ihandle << "] is clicked." << endl;

			ihandle = o2d.collision(point_cursor);
			if (ihandle >= 0)
				cout << "2DObj[" << ihandle << "] is cliced." << endl;
		}
		break;
	case GLFW_MOUSE_BUTTON_RIGHT:
		break;
	}
	mbtn = mact = mmd = -1;
	pthread_mutex_unlock(&mtx_mouse);

	return true;
}
#endif
