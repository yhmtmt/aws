#ifndef _F_GLFW_WINDOW_H_
#define _F_GLFW_WINDOW_H_
// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_glfw_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include "../channel/ch_image.h"

// In my OpenGL use, the 2D renderer uses -1 to 1 normalized coordinate, 
// The function convert OpenCV's pixel coordinate into normalized OpenGL coordinate.
inline void cnvCvPoint2GlPoint(const Size & vp, const Point2f & ptcv, Point2f & ptgl)
{
	double fac_x = 2.0 / (double) vp.width, fac_y = 2.0 / (double) vp.height;
	ptgl.x = (float)(ptcv.x * fac_x - 1.0);
	ptgl.y = -(float)(ptcv.y * fac_y - 1.0);
}

inline void cnvCvPoint2GlPoint(const double fac_x, const double fac_y, const Point2f & ptcv, Point2f & ptgl)
{
	ptgl.x = (float)(ptcv.x * fac_x - 1.0);
	ptgl.y = -(float)(ptcv.y * fac_y - 1.0);
}

inline void drawGlText(float x, float y, char * str, 
					   float r, float g, float b , float alpha,
					   void* font = GLUT_BITMAP_TIMES_ROMAN_24)
{
	glRasterPos2f(x, y);
	glColor4f(r, g, b, alpha);
	int l = strlen(str);
	for(int i = 0; i < l; i++){
		glutBitmapCharacter(font, str[i]);
	}
}

void drawCvPoints(const Size & vp, vector<Point2f> & pts,
				  const float r, const float g, const float b, const float alpha, 
				  const float l /*point size*/);
void drawCvChessboard(const Size & vp, vector<Point2f> & pts, 
					  const float r, const float g, const float b, const float alpha, 
					  const float l /* point size */, const float w /* line width */);
void drawCvPointDensity(Mat hist, const int hist_max, const Size grid,  
				   const float r, const float g, const float b, const float alpha, 
				   const float w /* line width of the grid */);


class f_glfw_window: public f_base
{
protected:
	GLFWwindow * m_pwin;
	Size m_sz_win;
	virtual bool init_run();
	virtual void destroy_run();

	virtual void _key_callback(int key, int scancode, int action, int mods)
	{
	}

	static void key_callback(GLFWwindow * pwindow, int key, int scancode, int action, int mods)
	{
		f_glfw_window * ptr = (f_glfw_window*) ((char*)pwindow - offsetof(f_glfw_window, m_pwin));
		ptr->_key_callback(key, scancode, action, mods);
	}

	virtual void _cursor_position_callback(double xpos, double ypos)
	{
	}

	static void cursor_position_callback(GLFWwindow* pwindow, double xpos, double ypos)
	{
		f_glfw_window * ptr = (f_glfw_window*) ((char*)pwindow - offsetof(f_glfw_window, m_pwin));
		ptr->_cursor_position_callback(xpos, ypos);
	}

	void _mouse_button_callback(int button, int action, int mods)
	{
	}

	static void mouse_button_callback(GLFWwindow* pwindow, int button, int action, int mods)
	{
		f_glfw_window * ptr = (f_glfw_window*) ((char*)pwindow - offsetof(f_glfw_window, m_pwin));
		ptr->_mouse_button_callback(button, action, mods);
	}

	void _scroll_callback(double xoffset, double yoffset)
	{
	}

	static void scroll_callback(GLFWwindow* pwindow, double xoffset, double yoffset)
	{
		f_glfw_window * ptr = (f_glfw_window*) ((char*)pwindow - offsetof(f_glfw_window, m_pwin));
		ptr->_scroll_callback(xoffset, yoffset);
	}

	static void framebuffer_size_callback(GLFWwindow * pwindow, int width, int height)
	{
		f_glfw_window * ptr = (f_glfw_window*) ((char*)pwindow - offsetof(f_glfw_window, m_pwin));
		ptr->m_sz_win = Size(width, height);
	}
public:
	f_glfw_window(const char * name);
	virtual ~f_glfw_window();
	
	virtual bool is_main_thread()
	{
		return true;
	}

	virtual bool proc();

	// glfw callbacks
	static void err_cb(int e, const char * dsc)
	{
		fputs(dsc, stderr);
	}
};


// simply shows input image 
class f_glfw_imview: public f_glfw_window
{
protected:
	ch_image * m_pin;
	long long m_timg;

	virtual bool init_run();

public:
	f_glfw_imview(const char * name): f_glfw_window(name), m_pin(NULL), m_timg(-1)
	{
	}
	virtual ~f_glfw_imview()
	{
	}

	virtual bool proc();
};

// single camera calibration
// * Chessboards are detected automatically and collected as a list.
// * A newly detected chessboard is scored by their quality.
// * A newly detected chessboard is replaced with that of lower score.
// * Chessboard's quality factors are, corner clarity, sizes of the grid, angle to the caemra, and density in the view.
class f_glfw_calib: public f_glfw_window
{
protected:
	ch_image * m_pin;
	long long m_timg; // time of the image captured.

	char m_fcampar[1024];
	AWSCamPar m_par; // Camera intrinsic parameters

	// calibration flag. These flags are interpreted into OpenCV's flag of calibrateCamera.
	// For normal camera model (used in cv::calibrateCamera)
	bool m_bcalib_fix_campar;
	bool m_bcalib_fix_focus;
	bool m_bcalib_use_intrinsic_guess;
	bool m_bcalib_fix_principal_point;
	bool m_bcalib_fix_aspect_ratio;
	bool m_bcalib_zero_tangent_dist;
	bool m_bcalib_fix_k1, m_bcalib_fix_k2, 
		m_bcalib_fix_k3, m_bcalib_fix_k4, 
		m_bcalib_fix_k5, m_bcalib_fix_k6; // k1-k4 are shared with fisheye.
	bool m_bcalib_rational_model;

	// For fisheye camera model (used in cv::fisheye::calibrateCamera)
	bool m_bFishEye; // Fish eye model flag.
	bool m_bcalib_recompute_extrinsic;
	bool m_bcalib_check_cond;
	bool m_bcalib_fix_skew;
	bool m_bcalib_fix_intrinsic;

	bool m_bcalib_done; // calibration done flag.
	double m_Erep; // reprojection error. (OpenCV's return value)
	double m_rep_avg; // average reprojection error. (root mean square)

	// Grayscale image passed to the detector. 
	Mat m_img_det;

	// chessboard model
	s_model m_model_chsbd;

	// detection thread objects.
	pthread_t m_thdet;
	pthread_mutex_t m_mtx;
	static void * thdet(void * ptr);
	void * _thdet();
	bool m_bthact;
	
	// Chessboard distribution holder.
	Mat m_dist_chsbd;
	Size m_hist_grid;

	int m_num_chsbds; // number of chessboards stocked as the list.
	int m_num_chsbds_det; // number of chessboards found.

	vector<s_obj*> m_objs; // chessboard list.

	// data structure holding chessboard's score
	struct s_chsbd_score{
		// corner score, size score, angle score, reprojection score, and the accumulated score.
		double crn, sz, angl, rep, tot;
		s_chsbd_score():crn(1.0), sz(1.0), angl(1.0), rep(1.0), tot(1.0)
		{
		}
	};
	vector<s_chsbd_score> m_score;

	void calc_tot_score(s_chsbd_score & score)
	{
		score.tot = score.crn * score.sz * score.angl / (score.rep + 1e-5);
	}

	// Scoring helper functions
	// The average contrast is took as the score
	double calc_corner_contrast_score(Mat & img, vector<Point2f> & pts);

	// The largest grid size is took as the score
	void calc_size_and_angle_score(vector<Point2f> & pts, double & ssz, double & sangl);

	// Point histogram handler
	void add_chsbd_dist(vector<Point2f> & pts);
	void sub_chsbd_dist(vector<Point2f> & pts);
	void refresh_chsbd_dist();
	double calc_chsbd_dist_score(vector<Point2f> & pts);
	void recalc_chsbd_dist_score();

	int gen_calib_flag();
	void calibrate();

	// User interface members
	enum e_view_mode{
		VM_CAM, VM_CG
	} m_vm;

	bool m_bshow_chsbd_all;
	bool m_bshow_chsbd_sel;
	int m_sel_chsbd;
	bool m_bshow_chsbd_density;
	bool m_bsave, m_bload;
	bool m_bdel;
	bool m_bdet;
	bool m_bcalib;

	virtual void _key_callback(int key, int scancode, int action, int mods);

	virtual void _cursor_position_callback(double xpos, double ypos)
	{
	}

	void _mouse_button_callback(int button, int action, int mods)
	{
	}

	void _scroll_callback(double xoffset, double yoffset)
	{
	}

	// OpenGL's vertex and index buffer. Here we use these buffers only for rendering camera object.
	// Chessboards are all rendered as the point cloud.
	GLuint m_vb_cam; // vertex buffer for camera
	GLuint m_ib_cam; // index buffer for camera
	struct AWS_VERTEX{ // we need to specify GL_C4F_N3F_V3F for interpretation.
		float r, g, b, a; // color
		float nx, ny, nz; // normal vector
		float x, y, z;    // vertex
	};
	
	AWS_VERTEX m_cam[16]; // camera object
	GLuint m_cam_idx[18];
	void resetCameraModel()
	{
		double * pP = m_par.getCvPrj();
		double fx = pP[0], fy = pP[1], cx = pP[2], cy = pP[3];
		double w = m_img_det.cols, h = m_img_det.rows;
		double sx = 0.1, sy = sx * h / w;
		double left = sx * cx / w, right = sx * (1.0 - left);
		double top = sy * cy / h, bottom = sy * (1.0 - top);
		double z =  left * fx / cx;
		// setting cam's focal point.
		for(int i = 0; i < 12; i += 3){
			m_cam[i].x = m_cam[i].y = m_cam[i].z = 0.;
		}

		// top surface
		m_cam[2].x = -left, m_cam[2].y = top, m_cam[2].z = z;
		m_cam[1].x = right, m_cam[1].y = top, m_cam[1].z = z;

		// bottom 
		m_cam[4].x = -left, m_cam[4].y = -bottom, m_cam[4].z = z;
		m_cam[5].x = right, m_cam[5].y = -bottom, m_cam[5].z = z;

		// left
		m_cam[8].x = -left, m_cam[8].y = -bottom, m_cam[8].z = z;
		m_cam[7].x = -left, m_cam[7].y = top, m_cam[7].z = z;

		// right
		m_cam[10].x = right, m_cam[10].y = -bottom, m_cam[10].z = z;
		m_cam[11].x = right, m_cam[11].y = top, m_cam[11].z = z;

		// sensor surface
		m_cam[12].x = -left, m_cam[12].y = -bottom, m_cam[12].z = z;
		m_cam[13].x = -left, m_cam[13].y = top, m_cam[13].z = z;
		m_cam[14].x = right, m_cam[14].y = top, m_cam[14].z = z;
		m_cam[15].x = right, m_cam[15].y = -bottom, m_cam[15].z = z;
	}

	virtual bool init_run();
public:
	f_glfw_calib(const char * name);

	virtual ~f_glfw_calib();

	virtual bool proc();
};

#endif
