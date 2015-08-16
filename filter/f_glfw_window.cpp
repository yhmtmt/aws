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

#include <iostream>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GLFW/glfw3.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glu.h>

#include "f_glfw_window.h"

//////////////////////////////////////////////////////////////////////////////////////////// helper functions
void drawPoints(vector<Point2f> & pts, float r, float g, float b, float alpha, float l)
{
	// drawing Points
	glPointSize(l);
	glBegin(GL_POINTS);
	{
		glColor4f(r, g, b, alpha);
		for(int ipt = 0; ipt < pts.size(); ipt++){
			glVertex2f(pts[ipt].x, pts[ipt].y);
		}
	}
	glEnd();
}

void drawChessboard(vector<Point2f> & pts, float r, float g, float b, float alpha, float l, float w)
{
	drawPoints(pts, r, g, b, alpha, l);
	glLineWidth(w);
	glBegin(GL_LINES);
	{
		Point2f &pt = pts[0];
		glVertex2f(pt.x, pt.y);
		for(int ipt = 1; ipt < pts.size(); ipt++){
			pt = pts[ipt];
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();
}

//////////////////////////////////////////////////////////////////////////////////////////// f_glfw_window
f_glfw_window::f_glfw_window(const char * name):f_base(name), m_pwin(NULL), m_sz_win(640, 480)
{
	register_fpar("width", &m_sz_win.width, "Width of the window.");
	register_fpar("height", &m_sz_win.height, "Height of the window.");
}

f_glfw_window::~f_glfw_window()
{
}

bool f_glfw_window::init_run()
{
	if(!glfwInit())
		return false;

	m_pwin = glfwCreateWindow(m_sz_win.width, m_sz_win.height, "Hello World", NULL, NULL);
	if (!m_pwin)
	{
		glfwTerminate();
		return false;
	}
	glfwSetErrorCallback(err_cb);

	glfwMakeContextCurrent(m_pwin);

	glfwSetKeyCallback(m_pwin, key_callback);
	glfwSetCursorPosCallback(m_pwin, cursor_position_callback);
	glfwSetMouseButtonCallback(m_pwin, mouse_button_callback);
	glfwSetScrollCallback(m_pwin, scroll_callback);

	return true;
}

void f_glfw_window::destroy_run()
{
  
  glfwTerminate();
  m_pwin = NULL;
}

bool f_glfw_window::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	//glfwMakeContextCurrent(m_pwin);

	// rendering codes >>>>>

	// <<<<< rendering codes

	glfwSwapBuffers(m_pwin);

	glfwPollEvents();

	return true;
}


//////////////////////////////////////////////////////// f_glfw_imview
void reshape(int width, int height)
{

	static GLfloat lightPosition[4] = { 0.25f, 1.0f, 0.25f, 0.0f };
	static GLfloat lightDiffuse[3] = { 1.0f, 1.0f, 1.0f };
	static GLfloat lightAmbient[3] = { 0.25f, 0.25f, 0.25f };
	static GLfloat lightSpecular[3] = { 1.0f, 1.0f, 1.0f };



	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glShadeModel(GL_SMOOTH);
	glViewport(0, 0, width, height);



	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0.5, 1.5, 2.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

}


static GLfloat vertices[][3] =
{
	{ -0.5f, -0.5f, 0.5f },
	{ 0.5f, -0.5f, 0.5f },
	{ 0.5f, 0.5f, 0.5f },
	{ -0.5f, 0.5f, 0.5f },
	{ 0.5f, -0.5f, -0.5f },
	{ -0.5f, -0.5f, -0.5f },
	{ -0.5f, 0.5f, -0.5f },
	{ 0.5f, 0.5f, -0.5f }
};

static GLfloat normals[][3] =
{
	{ 0.0f, 0.0f, 1.0f },
	{ 0.0f, 0.0f, -1.0f },
	{ 1.0f, 0.0f, 0.0f },
	{ -1.0f, 0.0f, 0.0f },
	{ 0.0f, 1.0f, 0.0f },
	{ 0.0f, -1.0f, 0.0f }
};


void display(void)
{

	static GLfloat diffuse[3] = { 1.0f, 0.0f, 0.0f };
	static GLfloat ambient[3] = { 0.25f, 0.25f, 0.25f };
	static GLfloat specular[3] = { 1.0f, 1.0f, 1.0f };
	static GLfloat shininess[1] = { 32.0f };

	static float angle = 0.0f;

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess);

	glEnable(GL_DEPTH_TEST);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	angle += 2.5f;
	if (angle > 360.0f) {
		angle -= 360.0f;
	}

	glPushMatrix();
	glRotatef(angle, 0.0f, 1.0f, 0.0f);

	// ‘O
	glBegin(GL_QUADS);
		glNormal3fv(normals[0]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[2]);
		glVertex3fv(vertices[3]);
	glEnd();

	// Œã
	glBegin(GL_QUADS);
		glNormal3fv(normals[1]);
		glVertex3fv(vertices[4]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[6]);
		glVertex3fv(vertices[7]);
	glEnd();

	// ‰E
	glBegin(GL_QUADS);
		glNormal3fv(normals[2]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[4]);
		glVertex3fv(vertices[7]);
		glVertex3fv(vertices[2]);
	glEnd();

	// ¶
	glBegin(GL_QUADS);
		glNormal3fv(normals[3]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[3]);
		glVertex3fv(vertices[6]);
	glEnd();

	// ã
	glBegin(GL_QUADS);
		glNormal3fv(normals[4]);
		glVertex3fv(vertices[3]);
		glVertex3fv(vertices[2]);
		glVertex3fv(vertices[7]);
		glVertex3fv(vertices[6]);
	glEnd();

	// ‰º
	glBegin(GL_QUADS);
		glNormal3fv(normals[5]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[4]);
	glEnd();

	glPopMatrix();
	//sleep(10);

}

bool f_glfw_imview::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	long long timg;
	Mat img = m_pin->get_img(timg);
	if(img.empty())
		return true;
	if(m_timg == timg)
		return true;

	m_timg = timg;

	glRasterPos2i(-1, -1);
	cnvCVBGR8toGLRGB8(img);
	glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	/*
	int width, height;
	glfwGetFramebufferSize(m_pwin, &width, &height);
	reshape(width, height);
	//idle();
	//sleep(10);
	display();
	*/
	glfwSwapBuffers(m_pwin);
	glfwPollEvents();

	return true;
}


bool f_glfw_imview::init_run()
{
	if(m_chin.size() == 0)
		return false;

	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_pin == NULL)
		return false;

	if(!f_glfw_window::init_run())
		return false;

	return true;
}

//////////////////////////////////////////////////////////////////////////////// f_glfw_claib members

bool f_glfw_calib::init_run()
{
	if(m_chin.size() == 0)
		return false;

	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_pin == NULL)
		return false;

	if(!f_glfw_window::init_run())
		return false;

	if(!m_model_chsbd.load()){
		cerr << "Failed to setup chessboard model with the model file " << m_model_chsbd.fname << endl;
		return false;
	}

	m_objs.resize(m_num_chsbds, NULL);
	m_score.resize(m_num_chsbds);
	m_num_chsbds_det = 0;

	m_dist_chsbd = Mat::zeros(m_hist_grid.height, m_hist_grid.width, CV_32SC1);

	m_par.setFishEye(m_bFishEye);

	return true;
}

bool f_glfw_calib::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	long long timg;
	Mat img = m_pin->get_img(timg);
	if(img.empty())
		return true;
	if(m_timg == timg)
		return true;

	// if the detecting thread is not active, convert the image into a grayscale one, and invoke detection thread.
	if(!m_bthact){
		cvtColor(img, m_img_det, CV_RGB2GRAY);
		pthread_create(&m_thdet, NULL, thdet, (void*) this);	
	}

	m_timg = timg;

	glRasterPos2i(-1, -1);
	cnvCVBGR8toGLRGB8(img);
	glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	// overlay chessboard if needed
	// overlay point density if needed
	// overlay information (Number of chessboard, maximum, minimum, average scores, reprojection error)

	glfwSwapBuffers(m_pwin);
	glfwPollEvents();

	return true;
}

void * f_glfw_calib::thdet(void * ptr)
{
	f_glfw_calib * pclb = (f_glfw_calib*) ptr;
	return pclb->_thdet();
}

void * f_glfw_calib::_thdet()
{
	if(m_hist_grid.width != m_dist_chsbd.cols || 
		m_hist_grid.height != m_dist_chsbd.rows)
		refresh_chsbd_dist();

	s_obj * pobj = m_model_chsbd.detect(m_img_det);
	if(!pobj) 
		return NULL;

	// calculating score of the chessboard.
	s_chsbd_score sc;
	sc.rep = 1.0;
	sc.crn = calc_corner_contrast_score(m_img_det, pobj->pt2d);
	calc_size_and_angle_score(pobj->pt2d, sc.sz, sc.angl);

	calc_tot_score(sc);

	// replacing the chessboard if possible.
	bool is_changed = false;
	for(int i = 0; i < m_num_chsbds; i++){
		if(m_objs[i] == NULL){
			m_objs[i] = pobj;
			m_score[i] = sc;
			add_chsbd_dist(pobj->pt2d);
			is_changed = true;
			m_num_chsbds++;
			break;
		}

		if(m_score[i].tot < sc.tot){
			s_obj * ptmp = pobj;
			pobj = m_objs[i];
			m_objs[i] = ptmp;
			sub_chsbd_dist(pobj->pt2d);
			add_chsbd_dist(ptmp->pt2d);
			delete pobj;
			is_changed = true;
			break;
		}
	}
	
	if(is_changed)
		recalc_chsbd_dist_score();
	
	if(m_num_chsbds == m_num_chsbds_det)
		calibrate();

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
		
		pix = img.data + (int) (img.cols * pt.y + pt.x + 0.5) + org;
		for(int y = 0; y < 5; y++){
			for(int x = 0; x < 5; x++, pix++){
				vmax = max(vmax, *pix);
				vmin = min(vmin, *pix);
			}
			pix += org - 5;
		}
		csum += (double)(vmax - vmin) / (double)((int)vmax + (int)vmin);
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

void f_glfw_calib::recalc_chsbd_dist_score()
{
	m_dist_chsbd = Mat::zeros(m_hist_grid.height, m_hist_grid.width, CV_32SC1);
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		s_obj * pobj = m_objs[iobj];
		calc_chsbd_dist_score(pobj->pt2d);
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
	m_par.setFishEye(m_bFishEye);

	Size sz_chsbd(m_model_chsbd.par_chsbd.w, m_model_chsbd.par_chsbd.h);
	int num_pts = sz_chsbd.width * sz_chsbd.height;

	vector<Mat> pt2ds;
	vector<Mat> pt3ds;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	int flags = gen_calib_flag();

	pt2ds.resize(m_objs.size());
	pt3ds.resize(m_objs.size());
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		pt2ds[iobj] = Mat(m_objs[iobj]->pt2d, false);
		pt3ds[iobj] = Mat(m_model_chsbd.pts, false);
	}
	
	if(m_bFishEye){
		m_Erep = fisheye::calibrate(pt3ds, pt2ds, sz_chsbd, m_par.getCvPrjMat(), m_par.getCvDistFishEyeMat(),
			rvecs, tvecs, flags);
	}else{
		m_Erep = calibrateCamera(pt3ds, pt2ds, sz_chsbd,
			m_par.getCvPrjMat(), m_par.getCvDistMat(), rvecs, tvecs, flags);
	}

	m_bcalib_done = true;

	// calculating reprojection error 
	double rep_tot = 0.;
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		m_objs[iobj]->rvec = rvecs[iobj];
		m_objs[iobj]->tvec = tvecs[iobj];
		vector<Point2f> & pt2dprj = m_objs[iobj]->pt2dprj;
		vector<Point2f> & pt2d = m_objs[iobj]->pt2d;

		if(m_bFishEye){
			fisheye::projectPoints(m_model_chsbd.pts, pt2dprj, rvecs[iobj], tvecs[iobj], 
				m_par.getCvPrjMat(), m_par.getCvDistFishEyeMat());
		}else{
			prjPts(m_model_chsbd.pts, pt2dprj,
				m_par.getCvPrjMat(), m_par.getCvDistMat(),
				rvecs[iobj], tvecs[iobj]);
		}
		double sum_rep = 0.;
		for(int ipt = 0; ipt < pt2d.size(); ipt++){
			Point2f & ptprj = pt2dprj[ipt];
			Point2f & pt = pt2d[ipt];
			Point2f ptdiff = pt - ptprj;
			double rep = ptdiff.x * ptdiff.x + ptdiff.y * ptdiff.y;
			sum_rep += rep;
		}
		rep_tot += sum_rep;
		m_score[iobj].rep = sqrt(sum_rep / (double) num_pts);
	}

	m_rep_avg = rep_tot / (double)(num_pts * m_num_chsbds);
}

#endif
