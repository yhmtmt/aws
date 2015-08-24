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
#include <GL/glut.h>
#include <GL/glew.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glu.h>

#include "f_glfw_window.h"

//////////////////////////////////////////////////////////////////////////////////////////// helper functions
void drawCvPoints(const Size & vp, vector<Point2f> & pts,
				  const float r, const float g, const float b, const float alpha, 
				  const float l /*point size*/)
{
	Point2f pt;
	double fac_x = 2.0 / (double) vp.width, fac_y = 2.0 / (double) vp.height;

	// drawing Points
	glPointSize(l);
	glBegin(GL_POINTS);
	{
		glColor4f(r, g, b, alpha);
		for(int ipt = 0; ipt < pts.size(); ipt++){
			cnvCvPoint2GlPoint(fac_x, fac_y, pts[ipt], pt);
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();
}

void drawCvChessboard(const Size & vp, vector<Point2f> & pts, 
					  const float r, const float g, const float b, const float alpha, 
					 const float l /* point size */, const float w /* line width */)
{
	drawCvPoints(vp, pts, r, g, b, alpha, l);
	glLineWidth(w);

	double fac_x = 2.0 / (double) vp.width, fac_y = 2.0 / (double) vp.height;

	glBegin(GL_LINES);
	{
		Point2f pt;
		cnvCvPoint2GlPoint(fac_x, fac_y, pts[0], pt);
		glVertex2f(pt.x, pt.y);
		for(int ipt = 1; ipt < pts.size(); ipt++){
			cnvCvPoint2GlPoint(fac_x, fac_y, pts[ipt], pt);
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();
}

void drawCvPointDensity(Mat hist, const int hist_max, const Size grid,  
				 const float r, const float g, const float b, const float alpha, 
				 const float w /* line width of the grid */)
{
	float fac = (float)(1.0 / (float) hist_max);
	float wgrid = (float)((float)2. / (float) grid.width);
	float hgrid = (float)((float)2. / (float) grid.height);

	glLineWidth(w);
	glBegin(GL_QUADS);
	{
		for(int y = 0; y < grid.height; y++){
			for(int x = 0; x < grid.width; x++){
				glColor4f(r * fac, g * fac, b * fac, alpha);
				float u, v;
				u = (float)(x * wgrid - 1.);
				v = -(float)(y * hgrid - 1.);
				glVertex2f(u, v);
				u = (float)((x + 1) * wgrid - 1.);
				glVertex2f(u, v);
				v = -(float)((y + 1) * hgrid - 1.);
				glVertex2f(u, v);
				u = (float)(x * wgrid - 1.);
				glVertex2f(u, v);
			}
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

	if(glewInit() != GLEW_OK)
		return false;

	m_pwin = glfwCreateWindow(m_sz_win.width, m_sz_win.height, "Hello World", NULL, NULL);
	if (!m_pwin)
	{
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(m_pwin);

	glfwSetKeyCallback(m_pwin, key_callback);
	glfwSetCursorPosCallback(m_pwin, cursor_position_callback);
	glfwSetMouseButtonCallback(m_pwin, mouse_button_callback);
	glfwSetScrollCallback(m_pwin, scroll_callback);
	glfwSetErrorCallback(err_cb);

	glfwSetWindowTitle(m_pwin, m_name);

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
f_glfw_calib::	f_glfw_calib(const char * name):f_glfw_window(name), 	
	m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_campar(false), m_bcalib_fix_focus(false),
	m_bcalib_fix_principal_point(false), m_bcalib_fix_aspect_ratio(false), m_bcalib_zero_tangent_dist(true),
	m_bcalib_fix_k1(true), m_bcalib_fix_k2(true), m_bcalib_fix_k3(true),
	m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false), 
	m_bFishEye(false), m_bcalib_recompute_extrinsic(false), m_bcalib_check_cond(false),
	m_bcalib_fix_skew(false), m_bcalib_fix_intrinsic(false),
	m_bcalib_done(false), m_num_chsbds(30), m_bthact(false), m_hist_grid(10, 10), m_rep_avg(1.0),
	m_vm(VM_CAM), 
	m_bshow_chsbd_all(true), m_bshow_chsbd_sel(true), m_sel_chsbd(-1), m_bshow_chsbd_density(true),
	m_bsave(false), m_bload(false), m_bdel(false), m_bdet(true), m_bcalib(true),
	m_vb_cam(GL_INVALID_VALUE), m_ib_cam(GL_INVALID_VALUE)
{
	m_fcampar[0] = '\0';

	register_fpar("fchsbd", m_model_chsbd.fname, 1024, "File path for the chessboard model.");
	register_fpar("nchsbd", &m_num_chsbds, "Number of chessboards stocked.");
	register_fpar("fcampar", m_fcampar, "File path of the camera parameter.");
	register_fpar("Wgrid", &m_hist_grid.width, "Number of horizontal grid of the chessboard histogram.");
	register_fpar("Hgrid", &m_hist_grid.height, "Number of vertical grid of the chessboard histogram.");

	// normal camera model
	register_fpar("use_intrinsic_guess", &m_bcalib_use_intrinsic_guess, "Use intrinsic guess.");
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

	// initialize camera model's color as transparent gray
	for(int i = 0; i < 16; i++){
		m_cam[i].r = m_cam[i].g = m_cam[i].b = 0.5;
		m_cam[i].a = 0.25;
		m_cam[i].nx = m_cam[i].ny = m_cam[i].nz = 0.;
		m_cam[i].x = m_cam[i].y = m_cam[i].z = 0.;
	}

	for(int i = 0; i < 15; i++){
		m_cam_idx[i] = i;
	}
	m_cam_idx[15] = 14;
	m_cam_idx[16] = 15;
	m_cam_idx[17] = 12;

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
	
	{
		// debugging code. I doubt that the window size specified in glfwCreateWindow is not 
		// the same as the size of the frame buffer.
		int w, h;
		glfwGetFramebufferSize(m_pwin, &w, &h);
		if(m_sz_win.width != w || m_sz_win.height){
			cerr << "Frame buffer size is different from given as the filter's parameter." << endl;
		}
	}
	
	// if the detecting thread is not active, convert the image into a grayscale one, and invoke detection thread.
	if(!m_bthact){
		cvtColor(img, m_img_det, CV_RGB2GRAY);
		pthread_create(&m_thdet, NULL, thdet, (void*) this);
	}

	if(!m_bsave){
		pthread_mutex_lock(&m_mtx);
		if(m_fcampar[0] == '\0' || !m_par.write(m_fcampar)){
			cerr << "Failed to save camera parameters into " << m_fcampar << endl;
		}
		pthread_mutex_unlock(&m_mtx);
		m_bsave = false;
	}

	if(!m_bload){
		pthread_mutex_lock(&m_mtx);
		if(m_fcampar[0] == '\0' || !m_par.read(m_fcampar)){
			cerr << "Failed to load camera parameters from " << m_fcampar << endl;
		}
		pthread_mutex_unlock(&m_mtx);
		m_bload = false;
	}

	if(m_bdel){
		if(m_sel_chsbd >= 0 && m_sel_chsbd < m_objs.size()){
			pthread_mutex_lock(&m_mtx);
			delete [] m_objs[m_sel_chsbd];
			m_score[m_sel_chsbd] = s_chsbd_score();
			for(;m_sel_chsbd >= 0; m_sel_chsbd--)
				if(m_objs[m_sel_chsbd])
					break;
			m_num_chsbds_det--;
			pthread_mutex_unlock(&m_mtx);
		}
	}

	m_timg = timg;

	if(m_vm == VM_CAM){
		glRasterPos2i(-1, -1);
		cnvCVBGR8toGLRGB8(img);
		glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);

		// For 2D rendering mode, the origin is set at left-bottom as (0, 0), clipping rectangle has the 
		// width and height the same as the frame buffer size.
		glViewport(0, 0, m_sz_win.width, m_sz_win.height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1., -1., -1., 1., -1., 1.);		

		// overlay chessboard if needed
		if(m_bshow_chsbd_all){
			for(int iobj = 0; iobj < m_objs.size(); iobj++){
				drawCvChessboard(m_sz_win, m_objs[iobj]->pt2d, 1.0, 0.0, 0.0, 0.5, 3, 1);
			}
		}

		if(m_bshow_chsbd_sel){
			if(m_sel_chsbd < 0 || m_sel_chsbd >= m_objs.size()){
				m_sel_chsbd = (int) m_objs.size() - 1;
			}else{
				drawCvChessboard(m_sz_win, m_objs[m_sel_chsbd]->pt2d, 1.0, 0.0, 0.0, 1.0, 3, 2);
			}
		}

		// overlay point density if needed
		if(m_bshow_chsbd_density){
			drawCvPointDensity(m_dist_chsbd, m_num_chsbds, m_hist_grid, 1.0, 0., 0., 128, 0);
		}

		// overlay information (Number of chessboard, maximum, minimum, average scores, reprojection error)
		float hfont = (float)(24. / (float) img.rows);
		float wfont = (float)(24. / (float) img.cols);
		float x = wfont;
		float y = hfont;
		char buf[256];

		// calculating chessboard's stats
		double smax = 0., smin = DBL_MAX, savg = 0.;
		for(int i = 0; i < m_num_chsbds; i++){
			if(!m_objs[i])
				continue;

			double s = m_score[i].tot;
			smax = max(smax, s);
			smin = min(smin, s);
			savg += s;
		}

		savg /= (double) m_num_chsbds_det;

		snprintf(buf, 255, "Nchsbd= %d/%d, Smax=%f Smin=%f Savg=%f Erep=%f ",
			m_num_chsbds_det, m_num_chsbds_det, smax, smin, savg, m_rep_avg);
		drawGlText(x, y, buf, 0, 1, 0, 1);
		y+= hfont;
		
		// indicating infromation about selected chessboard.
		if(m_bshow_chsbd_sel && m_sel_chsbd >= 0 && m_sel_chsbd < m_num_chsbds && m_objs[m_sel_chsbd]){
			snprintf(buf, 255, "Chessboard %d", m_sel_chsbd);
			drawGlText(x, y, buf, 0, 1, 0, 1);
			y+= hfont;

			snprintf(buf, 255, "Score: %f (Crn: %f Sz %f Angl %f Erep %f)", 
				m_score[m_sel_chsbd].tot, m_score[m_sel_chsbd].crn, m_score[m_sel_chsbd].sz, 
				m_score[m_sel_chsbd].angl, m_score[m_sel_chsbd].rep);
			drawGlText(x, y, buf, 0, 1, 0, 1);
			y+= hfont;
		}
	}else{
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if(m_bcalib){
			// setting projection matrix (Basically setting it the same as the camera parameter.)
			glViewport(0, 0, m_sz_win.width, m_sz_win.height);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			double * pP = m_par.getCvPrj();

			double fx = pP[0], fy = pP[1], cx = pP[2], cy = pP[3];
			double w = m_img_det.cols, h = m_img_det.rows;
			double left = cx / w, right = (1.0 - left);
			double top = cy / h, bottom = (1.0 - top);

			// the scene depth is set as 0.1 to 100 meter
			glFrustum(-cx, w - cx, -h + cy, cy, 0.1, 100.);

			GLfloat mobj[16];

			// setting model-view matrix chessboards and camera
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glTranslatef(m_trx, 0., m_trz);
			glRotatef(m_thx, 1.0, 0., 0.);
			glRotatef(m_thy, 0., 1., 0.);
			for(int i = 0; i < m_num_chsbds; i++){				
				if(!m_objs[i])
					continue;

				glPushMatrix();
				Mat R = Mat(3, 3, CV_64FC1);
				// object transformation
				exp_so3(m_objs[i]->rvec.ptr<double>(), R.ptr<double>());
				reformRtAsGl4x4(R, m_objs[i]->tvec, mobj);
				// multiply cg-cam motion 
				glMultMatrixf(mobj);

				glPointSize(3);
				glBegin(GL_POINTS);
				for(int ipt = 0; ipt < m_model_chsbd.pts.size(); ipt++){
					Point3f & pt = m_model_chsbd.pts[ipt];
					glColor4f(1.0, 0, 0, 1.0);
					glVertex3f(pt.x, pt.y, pt.z);
				}
				glEnd();
				glPopMatrix();
			}

			glEnableClientState(GL_VERTEX_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, m_vb_cam);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ib_cam);
			glInterleavedArrays(GL_C4F_N3F_V3F, 0, NULL);
			glDrawElements(GL_TRIANGLES, sizeof(m_cam_idx), GL_UNSIGNED_INT, NULL);

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		}else{// camera parameters have not been fixed yet. we cant render the objects.
		}
	}
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

	if(m_bdet){
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
				pthread_mutex_lock(&m_mtx);
				m_objs[i] = pobj;
				m_score[i] = sc;
				add_chsbd_dist(pobj->pt2d);
				is_changed = true;
				m_num_chsbds++;
				pthread_mutex_unlock(&m_mtx);
				break;
			}

			if(m_num_chsbds_det == m_num_chsbds && m_score[i].tot < sc.tot){
				pthread_mutex_lock(&m_mtx);
				s_obj * ptmp = pobj;
				pobj = m_objs[i];
				m_objs[i] = ptmp;
				sub_chsbd_dist(pobj->pt2d);
				add_chsbd_dist(ptmp->pt2d);
				pthread_mutex_unlock(&m_mtx);
				delete pobj;
				is_changed = true;
				break;
			}
		}

		if(is_changed)
			recalc_chsbd_dist_score();
	}

	if(m_bcalib && m_num_chsbds == m_num_chsbds_det){
		calibrate();
		// after calibration camera object
		pthread_mutex_lock(&m_mtx);
		resetCameraModel();
		pthread_mutex_unlock(&m_mtx);
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

	pt2ds.resize(m_num_chsbds_det);
	pt3ds.resize(m_num_chsbds_det);
	for(int iobj = 0; iobj < m_objs.size(); iobj++){
		if(m_objs[iobj] == NULL)
			continue;
		pt2ds[iobj] = Mat(m_objs[iobj]->pt2d, false);
		pt3ds[iobj] = Mat(m_model_chsbd.pts, false);
	}
	
	Mat P, D;
	if(m_bFishEye){
		m_Erep = fisheye::calibrate(pt3ds, pt2ds, sz_chsbd, P, D,
			rvecs, tvecs, flags);
	}else{
		m_Erep = calibrateCamera(pt3ds, pt2ds, sz_chsbd,
			P, D, rvecs, tvecs, flags);
	}

	// copying obtained camera parameter to m_par
	pthread_mutex_lock(&m_mtx);
	m_par.setCvPrj(P);
	m_par.setCvDist(D);
	pthread_mutex_unlock(&m_mtx);

	m_bcalib_done = true;

	// calculating reprojection error 
	double rep_tot = 0.;
	
	for(int iobj = 0, icount = 0; iobj < m_objs.size(); iobj++){
		if(m_objs[iobj] == NULL)
			continue;

		m_objs[iobj]->rvec = rvecs[icount];
		m_objs[iobj]->tvec = tvecs[icount];
		vector<Point2f> & pt2dprj = m_objs[iobj]->pt2dprj;
		vector<Point2f> & pt2d = m_objs[iobj]->pt2d;

		if(m_bFishEye){
			fisheye::projectPoints(m_model_chsbd.pts, pt2dprj, rvecs[icount], tvecs[icount], 
				m_par.getCvPrjMat(), m_par.getCvDistFishEyeMat());
		}else{
			prjPts(m_model_chsbd.pts, pt2dprj,
				m_par.getCvPrjMat(), m_par.getCvDistMat(),
				rvecs[icount], tvecs[icount]);
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

void f_glfw_calib::resetCameraModel(){

	if(m_ib_cam != GL_INVALID_VALUE || m_vb_cam != GL_INVALID_VALUE){
		// if the buffers have already been made, delete them.
		glDeleteBuffers(1, &m_ib_cam);
		glDeleteBuffers(1, &m_vb_cam);
		m_ib_cam = GL_INVALID_VALUE;
		m_vb_cam = GL_INVALID_VALUE;
	}

	double * pP = m_par.getCvPrj();
	double fx = pP[0], fy = pP[1], cx = pP[2], cy = pP[3];
	double w = m_img_det.cols, h = m_img_det.rows;
	double sx = 0.1, sy = sx * h / w;
	double left = cx / w, right = (1.0 - left);
	double top = cy / h, bottom = (1.0 - top);
	left *= sx, right *= sx;
	top *= sy, bottom *= sy;
	double z =  sx * fx / w;

	float nx, ny, nz;
	// setting cam's focal point.
	for(int i = 0; i < 12; i += 3){
		m_cam[i].x = m_cam[i].y = m_cam[i].z = 0.;
	}

	// top surface vertices and normals
	m_cam[2].x = -left, m_cam[2].y = top, m_cam[2].z = z;
	m_cam[1].x = right, m_cam[1].y = top, m_cam[1].z = z;
	crossProduct(m_cam[1].x, m_cam[1].y, m_cam[1].z,
		m_cam[2].x, m_cam[2].y, m_cam[2].z,
		nx, ny, nz);
	for(int i = 0; i < 3; i++){
		m_cam[i].nx = nx;
		m_cam[i].ny = ny;
		m_cam[i].nz = nz;
	}

	// bottom 
	m_cam[4].x = -left, m_cam[4].y = -bottom, m_cam[4].z = z;
	m_cam[5].x = right, m_cam[5].y = -bottom, m_cam[5].z = z;
	crossProduct(m_cam[4].x, m_cam[4].y, m_cam[4].z,
		m_cam[5].x, m_cam[5].y, m_cam[5].z,
		nx, ny, nz);
	normalize(nx, ny, nz);
	for(int i = 3; i < 6; i++){
		m_cam[i].nx = nx;
		m_cam[i].ny = ny;
		m_cam[i].nz = nz;
	}

	// left
	m_cam[8].x = -left, m_cam[8].y = -bottom, m_cam[8].z = z;
	m_cam[7].x = -left, m_cam[7].y = top, m_cam[7].z = z;
	crossProduct(m_cam[7].x, m_cam[7].y, m_cam[7].z,
		m_cam[8].x, m_cam[8].y, m_cam[8].z,
		nx, ny, nz);
	normalize(nx, ny, nz);
	for(int i = 6; i < 9; i++){
		m_cam[i].nx = nx;
		m_cam[i].ny = ny;
		m_cam[i].nz = nz;
	}

	// right
	m_cam[10].x = right, m_cam[10].y = -bottom, m_cam[10].z = z;
	m_cam[11].x = right, m_cam[11].y = top, m_cam[11].z = z;
	crossProduct(m_cam[10].x, m_cam[10].y, m_cam[10].z,
		m_cam[11].x, m_cam[11].y, m_cam[11].z,
		nx, ny, nz);
	normalize(nx, ny, nz);
	for(int i = 9; i < 12; i++){
		m_cam[i].nx = nx;
		m_cam[i].ny = ny;
		m_cam[i].nz = nz;
	}

	// sensor surface
	m_cam[12].x = -left, m_cam[12].y = -bottom, m_cam[12].z = z;
	m_cam[13].x = -left, m_cam[13].y = top, m_cam[13].z = z;
	m_cam[14].x = right, m_cam[14].y = top, m_cam[14].z = z;
	m_cam[15].x = right, m_cam[15].y = -bottom, m_cam[15].z = z;
	nx = ny = 0.;
	nz = 1.0;
	for(int i = 12; i < 16; i++){
		m_cam[i].nx = nx;
		m_cam[i].ny = ny;
		m_cam[i].nz = nz;
	}

	glGenBuffers(1, &m_vb_cam);
	glBindBuffer(GL_ARRAY_BUFFER, m_vb_cam);
	glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(AWS_VERTEX), m_cam, GL_STATIC_DRAW);

	glGenBuffers(1, &m_ib_cam);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ib_cam);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 18 * sizeof(GLuint), m_cam_idx, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


void f_glfw_calib::_key_callback(int key, int scancode, int action, int mods)
{
	switch(key){
	case GLFW_KEY_A:
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
		break;
	case GLFW_KEY_S:
		if(mods & GLFW_MOD_CONTROL){
			m_bsave = true;
		}
		break;
	case GLFW_KEY_T:
	case GLFW_KEY_U:
	case GLFW_KEY_V:
		if(m_vm == VM_CAM){
			m_vm = VM_CG;
		}else{
			m_vm = VM_CAM;
		}
		break;
	case GLFW_KEY_W:
	case GLFW_KEY_X:
	case GLFW_KEY_Y:
	case GLFW_KEY_Z:
	case GLFW_KEY_RIGHT:
		m_sel_chsbd = (m_sel_chsbd + 1) % (int) m_objs.size();
		break;
	case GLFW_KEY_LEFT:
		m_sel_chsbd = m_sel_chsbd - 1;
		if(m_sel_chsbd < 0)
			m_sel_chsbd += (int) m_objs.size();
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

#endif
