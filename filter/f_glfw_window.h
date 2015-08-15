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

	virtual bool init_run();
public:
	f_glfw_calib(const char * name):f_glfw_window(name), 	
		m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_campar(false), m_bcalib_fix_focus(false),
		m_bcalib_fix_principal_point(false), m_bcalib_fix_aspect_ratio(false), m_bcalib_zero_tangent_dist(true),
		m_bcalib_fix_k1(true), m_bcalib_fix_k2(true), m_bcalib_fix_k3(true),
		m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false), 
		m_bFishEye(false), m_bcalib_recompute_extrinsic(false), m_bcalib_check_cond(false),
		m_bcalib_fix_skew(false), m_bcalib_fix_intrinsic(false),
		m_bcalib_done(false), m_num_chsbds(30), m_bthact(false), m_hist_grid(10, 10), m_rep_avg(1.0)
	{
		register_fpar("fchsbd", m_model_chsbd.fname, 1024, "File path for the chessboard model.");
		register_fpar("nchsbd", &m_num_chsbds, "Number of chessboards stocked.");
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

	~f_glfw_calib()
	{
	}

	virtual bool proc();
};

#endif
