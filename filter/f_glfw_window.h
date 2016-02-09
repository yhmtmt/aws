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

void drawGlText(float x, float y, char * str, 
		float r, float g, float b , float alpha,
		void* font);

void drawGlSquare2Df(float x1, float y1, float x2, float y2, 
		     float r, float g, float b, float alpha, float size);

void drawGlSquare2Df(float x1, float y1, float x2, float y2, 
		     float r, float g, float b, float alpha);


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

	void _mouse_button_callback(int button, int action, int mods)
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

class f_glfw_stereo_view: public f_glfw_window
{
protected:
  ch_image * m_pin1, *m_pin2;
	long long m_timg;

	virtual bool init_run();

public:
 f_glfw_stereo_view(const char * name): f_glfw_window(name), m_pin1(NULL), m_pin2(NULL), m_timg(-1)
	{
	}
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
