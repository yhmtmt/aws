
#ifndef _F_GLFW_STEREO_H_
#define _F_GLFW_STEREO_H_

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "f_glfw_window.h"

#include "../util/aws_vlib.h"
#include "../util/aws_vobj.h"
#include "../channel/ch_image.h"
#include "../channel/ch_state.h"


// four path filter
// 1. find and save chessboard
// 2. calibrate both cameras and undistort independently
// 3. calibrate stereo camera.
// 4. calculate disparity image
class f_glfw_stereo_view : public f_glfw_window
{
protected:
	ch_image_ref * m_pin1, *m_pin2;
	ch_state * m_state;

	Mat m_img1, m_img2;

	long long m_timg1, m_timg2;
	long long m_ifrm1, m_ifrm2;
	long long m_ifrm_diff;
	int m_fm_count;
	int m_fm_max_count;
	int m_fm_time_min_dfrm;
	int m_fm_time_min;

	// Frame state flag
	bool m_bnew; // new frame
	bool m_bsync; // synchronized frame
	bool m_bupdate_img;

	// operation flag
	bool m_bdet_chsbd; // detect chessboard
	bool m_bsv_chsbd, m_bld_chsbd; // save and load chessboard
	bool m_bcbl, m_bcbr, m_bcbst; // calibration flag
	bool m_bred_chsbd; // reduce chessboard less than m_num_calib_chsbd
	bool m_bsvcp, m_bldcp;
	bool m_bsvstp, m_bldstp;
	bool m_bcapture;

	bool m_bchsbd;		 // chessboard model validity flag
	bool m_bflipx, m_bflipy; // image flipping option 
	bool m_bcpl, m_bcpr, m_bstp, m_brct; // camera parameter validity flag
	bool m_budl, m_budr; // undistort flag
	bool m_brctst;		 // rectify
	bool m_bdisp;		 // disparity map

	char m_fcapture[1024];

	// calibration flags
	bool m_bfisheye, /* true if fisheye model is used */
		m_bfix_int, m_bfix_k1, m_bfix_k2, m_bfix_k3, m_bfix_k4, /* common for both fisheye and rational models */
		m_bguess_int, /* Intrinsic guess is enabled */
		m_bfix_ar, m_bfix_pp, m_bzr_tng,
		m_brat_mdl, m_bfix_k5, m_bfix_k6; /* rational model's high order terms are enabled. */
	int m_num_calib_chsbd;
	char m_fcpl[1024], m_fcpr[1024], m_fstp[1024];
	AWSCamPar m_camparl, m_camparr;
	Mat m_Rl, m_Rr;
	Mat m_Pl, m_Pr;
	Mat m_mapl1, m_mapl2, m_mapr1, m_mapr2;
	Mat m_Rlr, m_Tlr;
	Mat m_E, m_F, m_Q;

	// chess board related parameters
	char m_fchsbdl[1024], m_fchsbdr[1024], m_fchsbdc[1024];
	s_model m_chsbd;	 // chessboard model
	int m_num_chsbdl, m_num_chsbdr;
	vector<long long> m_ifrm_chsbdl;
	vector<vector<Point2f>> m_pts_chsbdl;
	vector<double> m_erepl; // reprojection error in left camera
	vector<long long> m_ifrm_chsbdr;
	vector<vector<Point2f>> m_pts_chsbdr;
	vector<double> m_erepr; // reprojection error in right camera

	int m_num_chsbd_com;
	vector<long long> m_ifrm_chsbd_com;
	vector<vector<Point2f>> m_pts_chsbdl_com;
	vector<vector<Point2f>> m_pts_chsbdr_com;
	vector<double> m_ereps; // reprojection error in stereo camera

	struct s_chsbd_score{
		int idx;
		float score;
	};

	void save_chsbd(int icam /* 0 or 1 or 2*/);
	void load_chsbd(int icam /* 0 or 1 or 2*/);
	void reduce_chsbd(int icam /* 0 or 1 or 2*/);
	void calibrate(int icam /* 0 or 1 */);
	void init_undistort(AWSCamPar & par, Size & sz, Mat & R, Mat & P, Mat & map1, Mat & map2);

	int m_w, m_h;
	float m_r1, m_r2, m_rx1, m_ry1, m_rx2, m_ry2, m_wn1, m_hn1, m_wn2, m_hn2, m_xscale1, m_yscale1, m_xscale2, m_yscale2;
	Size m_sz1, m_sz2;

	enum e_show_chsbd{
		ESC_NONE, ESC_MNL, ESC_MNR, ESC_ST, ESC_MN_ALL, ESC_ST_ALL, ESC_ALL, ESC_NULL
	} m_show_chsbd;
	static const char * m_str_show_chsbd[ESC_NULL];

	int m_cur_chsbdl, m_cur_chsbdr, m_cur_chsbdst;

	void calc_window_scale();

	void draw_pixels(Mat & img);
	void draw_chsbds();

	void draw_chsbd(bool ud, AWSCamPar & cp, Mat & R, Mat & P, const float r, const float g, const float b,
		const float xscale, const float yscale,
		const float xorg, const float yorg,
		const float w, const float h,
		vector<vector<Point2f>> & chsbds, const int num_chsbds);
	void draw_a_chsbd(bool ud, AWSCamPar & cp, Mat & R, Mat & P, const float r, const float g, const float b,
		const float xscale, const float yscale,
		const float xorg, const float yorg,
		const float w, const float h,
		vector<vector<Point2f>> & chsbds, vector<double> & erep, const int num_chsbds, const int ichsbd);

	void draw_com_chsbd(const float r, const float g, const float b,
		const float xscalel, const float yscalel,
		const float xorgl, const float yorgl,
		const float wl, const float hl,
		const float xscaler, const float yscaler,
		const float xorgr, const float yorgr,
		const float wr, const float hr
		);

	void draw_a_com_chsbd(const float r, const float g, const float b,
		const float xscalel, const float yscalel,
		const float xorgl, const float yorgl,
		const float wl, const float hl,
		const float xscaler, const float yscaler,
		const float xorgr, const float yorgr,
		const float wr, const float hr
		);

	void calibrate_stereo();
	void rectify_stereo();

	Mat m_disp, m_dist, m_disp16;
	vector<float> m_dline;
	Mat m_odt_work;
	int m_rgn_drange;
	Size m_rgn_bb_min_n, m_rgn_bb_min_f;
	int m_rgn_foot_y;
	ushort m_dn, m_df;

	vector<ushort> m_rgn_disp;
	vector<ushort> m_rgn_pix;
	vector<ushort> m_rgn_ymax;
	vector<Rect> m_rgn_rect;
	vector<int> m_rgn_obst;
	struct s_obst{
		int xmin, xmax, ymin, ymax;
		ushort dmin, dmax;
		s_obst() :xmin(0), xmax(0), ymin(0), ymax(0), dmin(0), dmax(0){}
	};
	vector<s_obst> m_obst;

	void calc_obst();
	void calc_dmap(ushort disp_max);
	void calc_dline(ushort disp_max);
	void draw_dline();
	void draw_obst(ushort disp_max);
	void calc_and_draw_disparity_map(Mat & img1, Mat & img2);

	void save_stereo_pars();
	void load_stereo_pars();

	Ptr<StereoSGBM> m_sgbm;
	struct s_sgbm_par{
		bool m_update;
		int minDisparity; /* normally zero. it depends on rectification algorithm. */
		int numDisparities; /* maximum disparity - minimum disparity. it must be the divisible number by 16*/
		int blockSize; /* 3 to 11 */
		int P1, P2;
		int disp12MaxDiff;
		int preFilterCap;
		int uniquenessRatio;
		int speckleWindowSize;
		int speckleRange;
		int mode;
		s_sgbm_par() :m_update(false), minDisparity(0), numDisparities(64), blockSize(3),
			disp12MaxDiff(1), preFilterCap(0), uniquenessRatio(10), speckleWindowSize(100),
			speckleRange(32), mode(StereoSGBM::MODE_SGBM)
		{
			P1 = 8 * blockSize * blockSize;
			P2 = P1 * 4;
		}
	} m_sgbm_par;

	// draw horizon
#define SZ_ATT_BUF 100
	int m_iatt;
	int m_dtatt;
	long long m_tatt[SZ_ATT_BUF];
	float m_roll[SZ_ATT_BUF], m_pitch[SZ_ATT_BUF], m_yaw[SZ_ATT_BUF];
	float m_roll0, m_pitch0, m_yaw0;
	void draw_horizon();

	virtual bool init_run();
	virtual void destroy_run();

	Point2f m_pos_mouse;
	int m_num_com_pts;
	vector<Point2f> m_ptsl, m_ptsr;  // clicked point
	vector<Point2f> m_ptsul, m_ptsur; // undistort point
	Point2f m_ptl, m_ptr, m_glptl, m_glptr;
	bool m_bptl, m_bptr;
	virtual void _mouse_button_callback(int button, int action, int mods)
	{
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
			if (m_pos_mouse.x > -1 && m_pos_mouse.x < 0 && m_pos_mouse.y > 0 && m_pos_mouse.y < 1 && !m_bptl && !m_budl){
				m_ptl = m_pos_mouse;
				cnvGlPoint2CvPoint(m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1, m_pos_mouse, m_glptl);
				m_bptl = true;
			}
			if (m_pos_mouse.x > 0 && m_pos_mouse.x < 1 && m_pos_mouse.y > 0 && m_pos_mouse.y < 1 && !m_bptr && !m_budr){
				m_ptr = m_pos_mouse;
				cnvGlPoint2CvPoint(m_xscale2, m_yscale2, 0, 0, m_wn2, m_hn2, m_pos_mouse, m_glptr);
				m_bptr = true;
			}
		}
	}

	virtual void _cursor_position_callback(double xpos, double ypos)
	{
		m_pos_mouse.x = (float)(2.0 * xpos / (double)m_sz_win.width - 1.0);
		m_pos_mouse.y = (float)(1.0 - 2.0 * ypos / (double)m_sz_win.height);
	}

	bool m_bevt_key;
	int m_key;
	int m_scancode;
	int m_mods;
	int m_action;
	void handle_key();

	virtual void _key_callback(int key, int scancode, int action, int mods)
	{
		m_key = key;
		m_mods = mods;
		m_scancode = scancode;
		m_action = action;
		switch (key){
		case GLFW_KEY_S:
			if (GLFW_MOD_CONTROL & mods){
				m_bcapture = true;
			}
			break;
		}
	}

	void check_undistort(vector<Point2f> & pts, vector<Point2f> & ptsu,
		Mat & K, Mat & D, Mat & R, Mat & P, Mat & map1, Mat & map2);
public:
	f_glfw_stereo_view(const char * name);
	virtual ~f_glfw_stereo_view()
	{
	}

	virtual bool proc();
};



#endif
