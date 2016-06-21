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

#include "../util/aws_vlib.h"
#include "../util/aws_vobj.h"
#include "../channel/ch_image.h"

#include "f_base.h"

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

inline void cnvGlPoint2CvPoint(const float fac_x, const float fac_y, const float xorg, const float yorg, 
							   const float w, const float h, const Point2f & ptgl, Point2f & ptcv)
{
	ptcv.x = (float)((ptgl.x - xorg) / fac_x);
	ptcv.y = (float)(-(ptgl.y - yorg - h) / fac_y);
}

inline void cnvCvPoint2GlPoint(const float fac_x, const float fac_y, const float xorg, const float yorg, 
							   const float w, const float h, const Point2f & ptcv, Point2f & ptgl)
{
	ptgl.x = (float)(ptcv.x * fac_x + xorg);
	ptgl.y = (float)(h - ptcv.y * fac_y + yorg);
}


void drawGlText(float x, float y, const char * str, 
		float r, float g, float b , float alpha,
		void* font);

void drawGlSquare2Df(float x1, float y1, float x2, float y2, 
		     float r, float g, float b, float alpha, float size);

void drawGlSquare2Df(float x1, float y1, float x2, float y2, 
		     float r, float g, float b, float alpha);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2, 
					float x3,float y3, float r, float g, float b, float alpha, float size);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2, 
					float x3,float y3, float r, float g, float b, float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts, 
					  float r, float g, float b, float alpha, float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts, 
					  float r, float g, float b, float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts, Point2f & offset, 
					  float r, float g, float b, float alpha, float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts, Point2f & offset,
					  float r, float g, float b, float alpha);


void drawGlLine2Df(float x1, float y1, float x2, float y2,
		   float r, float g, float b, float alpha, float size);

void drawCvPoints(const Size & vp, vector<Point2f> & pts,
				  const float r, const float g, const float b, const float alpha, 
				  const float l /*point size*/);
void drawCvChessboard(const Size & vp, vector<Point2f> & pts, 
					  const float r, const float g, const float b, const float alpha, 
					  const float l /* point size */, const float w /* line width */);
void drawCvPointDensity(Mat hist, const int hist_max, const Size grid,  
				   const float r, const float g, const float b, const float alpha, 
				   const float w /* line width of the grid */);

inline void crossProduct(const float x1, const float y1, const float z1, 
				  const float x2, const float y2, const float z2, 
				  float & nx, float & ny, float & nz)
{
	nx = (float)(y1 * z2 - y2 * z1);
	ny = (float)(z1 * x2 - z2 * x1);
	nz = (float)(x1 * y2 - x2 * y1);
}

inline void normalize(float & nx, float & ny, float & nz)
{
	float inorm = (float) (1.0 / sqrt(nx * nx + ny * ny + nz * nz));
	nx *= inorm;
	ny *= inorm;
	nz *= inorm;
}

inline void setEyeGl4x4(GLfloat * m)
{
	memset((void*)m, 0, sizeof(GLfloat) * 16);
	m[0] = m[5] = m[10] = m[15] = 1.0;
}

inline void reformRtAsGl4x4(Mat & R, Mat & t, GLfloat * m)
{
	// note that m is the OpenGL memory layout (say column major order)
	double * pR = R.ptr<double>();
	double * pt = t.ptr<double>();
	int i, j;

	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++){
			m[j * 4 + i] = (float) pR[i * 3 + j];
		}
		m[j * 4 + i] = 0;
	}

	for(int i = 0; i < 3; i++){
		m[12 + i] = (float) pt[i];
	}

	m[15] = 1.0;
}

struct cmpglfwin{
  bool operator () (const GLFWwindow * a, const GLFWwindow * b){
    return reinterpret_cast<unsigned long long>(a) 
      < reinterpret_cast<unsigned long long>(b);
  }
};

class f_glfw_window;

typedef map<GLFWwindow *, f_glfw_window *, cmpglfwin> MapGLFWin;

class f_glfw_window: public f_base
{
protected:
  static MapGLFWin m_map_glfwin;
  GLFWwindow * m_pwin;

  GLFWwindow * pwin(){
    return m_pwin;
  }
  
  Size m_sz_win;
  
  virtual bool init_run();
  virtual void destroy_run();
  
  virtual void _key_callback(int key, int scancode, int action, int mods)
  {
  }
  
  static void key_callback(GLFWwindow * pwindow, int key, int scancode, int action, int mods)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_key_callback(key, scancode, action, mods);
  }
  
  virtual void _cursor_position_callback(double xpos, double ypos)
  {
  }
  
  static void cursor_position_callback(GLFWwindow* pwindow, double xpos, double ypos)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_cursor_position_callback(xpos, ypos);
  }
  
  virtual void _mouse_button_callback(int button, int action, int mods)
  {
  }
  
  static void mouse_button_callback(GLFWwindow* pwindow, int button, int action, int mods)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_mouse_button_callback(button, action, mods);
  }
  
  void _scroll_callback(double xoffset, double yoffset)
  {
  }
  
  static void scroll_callback(GLFWwindow* pwindow, double xoffset, double yoffset)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
    ptr->_scroll_callback(xoffset, yoffset);
  }
  
  static void framebuffer_size_callback(GLFWwindow * pwindow, int width, int height)
  {
    MapGLFWin::iterator itr = m_map_glfwin.find(pwindow);
    f_glfw_window * ptr = itr->second;
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

// four path filter
// 1. find and save chessboard
// 2. calibrate both cameras and undistort independently
// 3. calibrate stereo camera.
// 4. calculate disparity image
class f_glfw_stereo_view: public f_glfw_window
{
protected:
	ch_image_ref * m_pin1, *m_pin2;
	Mat m_img1, m_img2;
	Mat m_disp;

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
	vector<long long> m_ifrm_chsbdr;
	vector<vector<Point2f>> m_pts_chsbdr;

	int m_num_chsbd_com;
	vector<long long> m_ifrm_chsbd_com;
	vector<vector<Point2f>> m_pts_chsbdl_com;
	vector<vector<Point2f>> m_pts_chsbdr_com;
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

	void draw_pixels(Mat & img);
	void draw_chsbd(bool ud, AWSCamPar & cp, Mat & R, Mat & P, const float r, const float g, const float b,
		const float xscale, const float yscale,
		const float xorg, const float yorg,
		const float w, const float h, 
		vector<vector<Point2f>> & chsbds, const int num_chsbds);

	void draw_com_chsbd(const float r, const float g, const float b,
		const float xscalel, const float yscalel,
		const float xorgl, const float yorgl,
		const float wl, const float hl,
		const float xscaler, const float yscaler,
		const float xorgr, const float yorgr,
		const float wr, const float hr
		); 


	void calibrate_stereo();
	void rectify_stereo();
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
		s_sgbm_par():m_update(false), minDisparity(0), numDisparities(64), blockSize(3),
			disp12MaxDiff(1), preFilterCap(0), uniquenessRatio(10), speckleWindowSize(100),
			speckleRange(32), mode(StereoSGBM::MODE_SGBM)
		{
			P1 = 8 * blockSize * blockSize;
			P2 = P1 * 4;
		}
	} m_sgbm_par;
	
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
		  if(m_pos_mouse.x > -1 && m_pos_mouse.x < 0 && m_pos_mouse.y > 0 && m_pos_mouse.y < 1 && !m_bptl && !m_budl){
			  m_ptl = m_pos_mouse;
			  cnvGlPoint2CvPoint(m_xscale1, m_yscale1, -1, 0, m_wn1, m_hn1, m_pos_mouse, m_glptl);
			  m_bptl = true;
		  }
		  if(m_pos_mouse.x > 0 && m_pos_mouse.x < 1 && m_pos_mouse.y > 0 && m_pos_mouse.y < 1 && !m_bptr && !m_budr){
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
  
  virtual void _key_callback(int key, int scancode, int action, int mods)
  {
	  switch(key){
	  case GLFW_KEY_S:
		  if(GLFW_MOD_CONTROL & mods){
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
	
	AWS_VERTEX m_cam[16]; // camera model
	GLuint m_cam_idx[18]; // indices for camera model (triangles with CCW order.)

	void resetCameraModel();

	GLfloat m_thx, m_thy; // 3d camera rotation around x and y. (degree)
	GLfloat m_trx, m_trz; // 3d camera translation toward x and z

	virtual bool init_run();
public:
	f_glfw_calib(const char * name);

	virtual ~f_glfw_calib();

	virtual bool proc();
};

#endif
