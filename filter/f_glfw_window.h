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
#include "../channel/ch_state.h"

#include "f_base.h"

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
  bool operator () (const GLFWwindow * a, const GLFWwindow * b) const{
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
	bool m_bverb;
	virtual bool init_run();

public:
 f_glfw_imview(const char * name): f_glfw_window(name), m_pin(NULL), m_timg(-1), m_bverb(false)
	{
	  register_fpar("verb", &m_bverb, "For debug.");
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
	char m_fsmplimg[1024];
	char m_fcbdet[1024];
	char m_fcampar[1024];

	bool m_b3dview;
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
	pthread_t m_thwork;
	pthread_mutex_t m_mtx;
	static void * thwork(void * ptr);
	void * detect();
	bool m_bthact;
	
	// Chessboard distribution holder.
	Mat m_dist_chsbd;
	Size m_hist_grid;

	int m_num_chsbds; // number of chessboards stocked as the list.
	int m_num_chsbds_det; // number of chessboards found.

	vector<s_obj*> m_objs; // chessboard list.
	vector<vector<Point2f> > m_upt2d, m_upt2dprj; // undistort points
	
	// data structure holding chessboard's score
	struct s_chsbd_score{
		// corner score, size score, angle score, reprojection score, and the accumulated score.
		double crn, sz, angl, rep, tot, dist;
		s_chsbd_score() :crn(1.0), sz(1.0), angl(1.0), rep(0.0), tot(1.0), dist(1.0)
		{
		}
	};
	vector<s_chsbd_score> m_score;
	double m_tot_score;


	void calc_tot_score(s_chsbd_score & score)
	{
		score.tot = score.dist * score.crn * score.sz * score.angl / (score.rep + 1e-5);
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
	void calc_chsbd_dist_score();
	void calc_chsbd_score();
	int gen_calib_flag();
	void calibrate();

	void draw_chsbd();
	void init_undistort();
	Mat m_map1, m_map2;
	// User interface members

	bool m_bshow_chsbd_all;
	bool m_bshow_chsbd_sel;
	int m_sel_chsbd;
	bool m_bshow_chsbd_density;
	bool m_bsave, m_bload;
	void save();
	void load();

	bool m_bdel;
	void del();

	void calc_contrast(Mat & img);
	double m_contrast;
	bool m_bdet;
	bool m_bcalib;
	bool m_bundist;

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


	virtual bool init_run();
public:
	f_glfw_calib(const char * name);

	virtual ~f_glfw_calib();

	virtual bool proc();
};

#include "../util/aws_map.h"
using namespace AWSMap;

class f_glfw_test3d : public f_glfw_window
{
protected:
	int n;
	int nf;
	GLuint v, f, p;
	GLuint mvpl, ml, parl, smpl, modeloc;
	GLuint posl, texcdl, nml;
	GLuint vao, vbo[4];// vbo[0]: vetex, vbo[1]: color or texcoord, vbo[2]: normal, vbo[3]: index
	GLuint hetex; // texture handle
	Mat etex; // texture image

	long long tprev;
	float deg;

	char fvs[1024]; // file name of the vertex shader
	char ffs[1024]; // file name of the fragment shader
	char fetex[1024]; // file name of the earth texture


	static const char * strings[5];
	int hstr[5];
	c_gl_text_obj otxt;
	char ffnttex[1024];
	char ffntcfg[1024];

	c_gl_line_obj oline;
	c_gl_point_obj opts;

	char * load_glsl_text(const char * fname);
	bool setup_shader();
	bool setup_objects(c_icosahedron * pic);

	glm::vec2 point_cursor;
	int mbtn, mact, mmd;
	pthread_mutex_t mtx_mouse;
	virtual void _cursor_position_callback(double xpos, double ypos)
	{		
		point_cursor.x = (float)(2.0 * xpos / (double) m_sz_win.width - 1.);
		point_cursor.y = (float)(1. - 2.0 * ypos / (double) m_sz_win.height);
	}

	virtual void _mouse_button_callback(int button, int action, int mods)
	{
		pthread_mutex_lock(&mtx_mouse);
		mbtn = button;
		mact = action;
		mmd = mods;
		pthread_mutex_unlock(&mtx_mouse);
	}
public:
	f_glfw_test3d(const char * name);
	~f_glfw_test3d();
	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

inline void set_identity4x4(float * m) 
{
	for (int i = 0; i < 16; i++){
		m[i] = 0.f;
	}

	m[0] = m[5] = m[10] = m[15] = 1.0f;
}

inline void mult4x4(float * a, float * b) // a = axb
{
	float m[16];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			m[j * 4 + i] = 0.0f;
			for (int k = 0; k < 4; ++k) {
				m[j * 4 + i] += a[k * 4 + i] * b[j * 4 + k];
			}
		}
	}
	memcpy(a, m, 16 * sizeof(float));
}

inline void set_tran(float * m, float x, float y, float z)
{
	set_identity4x4(m);
	m[12] = x;
	m[13] = y;
	m[14] = z;
}

inline void set_pm(float * m, float fov, float ratio, float nearP, float farP)
{
	float f = 1.0f / tan(fov * (PI / 360.0));

	set_identity4x4(m);

	m[0] = f / ratio;
	m[1 * 4 + 1] = f;
	m[2 * 4 + 2] = (farP + nearP) / (nearP - farP);
	m[3 * 4 + 2] = (2.0f * farP * nearP) / (nearP - farP);
	m[2 * 4 + 3] = -1.0f;
	m[3 * 4 + 3] = 0.0f;
}

inline void xproduct(float * a, float * b, float * res)
{
	res[0] = a[1] * b[2] - b[1] * a[2];
	res[1] = a[2] * b[0] - b[2] * a[0];
	res[2] = a[0] * b[1] - b[0] * a[1];
}

inline void norm(float * a)
{
	float imag = 1.0 / sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

	a[0] *= imag;
	a[1] *= imag;
	a[2] *= imag;
}

inline void set_vm(float * m, float posX, float posY, float posZ,
	float lookAtX, float lookAtY, float lookAtZ) 
{
	float dir[3], right[3], up[3];

	up[0] = 0.0f;   up[1] = 1.0f;   up[2] = 0.0f;

	dir[0] = (lookAtX - posX);
	dir[1] = (lookAtY - posY);
	dir[2] = (lookAtZ - posZ);
	norm(dir);

	xproduct(dir, up, right);
	norm(right);

	xproduct(right, dir, up);
	norm(up);

	float aux[16];

	m[0] = right[0];
	m[4] = right[1];
	m[8] = right[2];
	m[12] = 0.0f;

	m[1] = up[0];
	m[5] = up[1];
	m[9] = up[2];
	m[13] = 0.0f;

	m[2] = -dir[0];
	m[6] = -dir[1];
	m[10] = -dir[2];
	m[14] = 0.0f;

	m[3] = 0.0f;
	m[7] = 0.0f;
	m[11] = 0.0f;
	m[15] = 1.0f;

	set_tran(aux, -posX, -posY, -posZ);

	mult4x4(m, aux);
}

void printShaderInfoLog(GLuint obj);
void printProgramInfoLog(GLuint obj);
#endif
