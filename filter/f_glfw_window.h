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
	bool m_bFishEye; // Fish eye model flag.
	bool m_bcalib_done; // calibration done flag.
	double m_Erep; // reprojection error.

	// Grayscale image passed to the detector. 
	Mat m_img_det;

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

	vector<s_obj*> m_objs; // chessboard list.
	struct s_chsbd_score{
		// corner score, size score, angle score, reprojection score, and the accumulated score.
		double crn, sz, angl, rep, tot;
		s_chsbd_score():crn(1.0), sz(1.0), angl(1.0), rep(1.0), tot(1.0)
		{
		}
	};
	vector<s_chsbd_score> m_score;

	double m_wcrn, m_wsz, m_wangl, m_wrep;
	void calc_tot_score(s_chsbd_score & score)
	{
		score.tot = m_wcrn * score.crn + m_wsz * score.sz 
			+ m_wangl * score.angl + m_wrep * score.rep;
	}

	virtual bool init_run();
public:
	f_glfw_calib(const char * name):f_glfw_window(name), m_bFishEye(false), m_bcalib_done(false), m_num_chsbds(30), m_bthact(false), m_hist_grid(10, 10)
	{
		register_fpar("fisheye", &m_bFishEye, "Yes, use fisheye model.");
		register_fpar("fchsbd", m_model_chsbd.fname, 1024, "File path for the chessboard model.");
		register_fpar("nchsbd", &m_num_chsbds, "Number of chessboards stocked.");
		register_fpar("Wgrid", &m_hist_grid.width, "Number of horizontal grid of the chessboard histogram.");
		register_fpar("Hgrid", &m_hist_grid.height, "Number of vertical grid of the chessboard histogram.");
		register_fpar("wcrn", &m_wcrn, "Weight of the corner score.");
		register_fpar("wsz", &m_wsz, "Weight of the size score.");
		register_fpar("wangle", &m_wangl, "Weight of the angle score.");
		register_fpar("wrep", &m_wrep, "Weight of the reprojection score.");

		pthread_mutex_init(&m_mtx, NULL);
	}

	~f_glfw_calib()
	{
	}

	virtual bool proc();
};

#endif
