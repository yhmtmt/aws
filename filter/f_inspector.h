#ifndef _F_INSPECTOR_H_
#define _F_INSPECTOR_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_inspector.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_inspector.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_inspector.h.  If not, see <http://www.gnu.org/licenses/>. 
#include "../util/c_imgalign.h"
#include "f_ds_window.h"

bool is_equal(Mat & a, Mat & b);
void mat2csv(ofstream & out, Mat & m);

////////////////// procedure for model pose estimation
// for calibration phase
// 1. load lens parameter table
// 2. chessboard images are loaded. (assuming uniform magnification)
// 3. camera caribration is executed for the magnification
// 4. append the calibration data to the lens parameter table.
// 5. save lens parameter table
//    repeate 1. to 5. for all magnifications.

// for analysis phase
// 6. loading model
// 7. loading lens parameter table
//	(tbl_loaded flag is asserted)
// 8. for each frame
//	8.1 correspondences between model vertices and image vertices are found
//	8.2. bundle adjastment is executed to guesses the model pauses and camera parameters simultaneously
//	(model_pose_fixed flag and cam_par fixed flag are asserted, you can choose from full parameter adjustment to pose only adjustment)
//	8.3 render model
// 9. save model poses

//#define VERB_LM

struct ModelVertex   
{   
    ModelVertex(){}   
    ModelVertex(float x, float y, float z,    
        float nx, float ny, float nz, float u, float v)   
    {   
         _x = x;   _y = y;   _z = z;   
        _nx = nx; _ny = ny; _nz = nz;   
         _u = u;   _v = v;   
    }   
   
    float _x, _y, _z, _nx, _ny, _nz, _u, _v;   
   
    static const DWORD FVF;   
};   

// finding closest 2d point for given (x, y), and returning the index and distance.
void get_cursor_point(vector<Point2f> & pt2ds, float x, float y, int & idx, double & dist);

struct s_model;
void render_prjpts(s_model & mdl, vector<Point2f> & pts,
	LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
	int pttype, int state, int cur_point);

// s_edge represents the edge between points defined in the s_model
// This is used for drawing the wire frame model. 
struct s_edge{
	int s, e;
	s_edge():s(0), e(0){};
};

///////////////////////////////////////////////////////////////// s_model
// s_model represents a 3D model tracked in the scene by f_inspector.
// It contains points and edges, and their projection method.
struct s_obj;
struct s_model
{
	char fname[1024];
	int ref;
	string name;
	vector<Point3f> pts;
	vector<s_edge> edges;

	enum e_model_type{
		EMT_NORMAL, EMT_CHSBD, EMT_UNKNOWN
	} type;

	struct s_chsbd{
		int w, h; // chessboard width and height
		float p; // chessboard pitch in meter
		bool parse(const char * name, e_model_type & type, vector<Point3f> & pts, vector<s_edge> & edges);
		bool detect(Mat & img, vector<Point2f> & pt2d);
	};

	union{
		s_chsbd par_chsbd;
	};

	static const char * m_str_type[ EMT_UNKNOWN + 1];

	float xmin, ymin, zmin, xmax, ymax, zmax;
	s_model():ref(0), xmin(FLT_MAX), ymin(FLT_MAX), zmin(FLT_MAX),
		xmax(-FLT_MAX), ymax(-FLT_MAX), zmax(-FLT_MAX)
	{
		fname[0] = '\0';
	}

	void calc_bounds();

	int get_num_pts()
	{
		return (int) pts.size();
	}

	// get_max_dist calculates size of the bounding box of the model and	
	// returns its diagonal length.
	double get_max_dist();

	double get_xsize()
	{
		return xmax - xmin;
	}

	double get_ysize()
	{
		return ymax - ymin;
	}

	double get_zsize()
	{
		return zmax - zmin;
	}

	void proj(vector<Point2f> & pt2d, Mat & cam_int, Mat & cam_dist, Mat & rvec_cam, Mat & tvec_cam, 
		Mat & rvec_obj, Mat & tvec_obj);

	bool load(const char * afname);

	s_obj * detect(Mat & img, s_obj * pobj = NULL);
};

///////////////////////////////////////////////////////////////// s_obj
// s_obj represents the object in the scene.
// User can specify its feature points, find the correspondance between the model
// and the points. 
struct s_obj
{
	s_model * pmdl;
	long long t;
	char * name;
	vector<Point2f> pt2d;
	vector<Point2f> pt2dprj;
	vector<int> visible; // true if 2d point is visible in the image

	vector<Mat> ptx_tmpl; // point's template images.

	bool is_attitude_fixed;
	Mat tvec, rvec;

	bool is_prj;

	Mat jacobian;
	Mat jmax; // maximum values of jacobian for each parameter
	Mat hessian;
	Mat dp;
	Mat err;
	Mat jterr;
	double ssd;
	int match_count;

	s_obj(): pmdl(NULL), name(NULL), is_attitude_fixed(false), is_prj(false)
	{
		tvec = Mat::zeros(3, 1, CV_64FC1);
		rvec = Mat::zeros(3, 1, CV_64FC1);
	};

	~s_obj()
	{
		delete[] name;
		name = NULL;
	}

	int calc_num_matched_points();
	double calc_ssd();

	int get_num_points(){
		return (int) pt2d.size();
	}

	void get_bb_pt2d(Rect & bb);

	// getting the nearest object point and the distance to (x, y)
	void get_cursor_point(float x, float y, int & idx, double & dist)
	{
		::get_cursor_point(pt2d, x, y, idx, dist);
	}

	// getting the nearest point of the projected model and the distance.
	void get_cursor_point_3d(float x, float y, int & idx, double & dist)
	{
		::get_cursor_point(pt2dprj, x, y, idx, dist);
	}

	// calculate projection
	void proj(Mat & camint, Mat & camdist, bool bjacobian = true, bool fix_aspect_ratio = true);

	// draw the wire frame model
	void render(LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
		int pttype, int state, int cur_point = -1);

	// render vector in comparable size to the object  
	void render_vector(Point3f & vec,
		Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
		LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline);

	// render axis in three direction
	void render_axis(Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
		LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline, int axis = -1);

	void render_axis(Mat & cam_int, Mat & cam_dist,
		LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline, int axis = -1)
	{
		render_axis(rvec, tvec, cam_int, cam_dist, pd3dev, pline, axis);
	}

	bool init(s_model * apmdl, long long at, const Mat & camint, const Mat & camdist,
		const double width, const double height);

	bool init(const s_obj & obj);

	bool load(FileNode & fnobj, vector<s_model*> & mdls);
	bool save(FileStorage & fs);

	void fixAttitude(bool val)
	{
		is_attitude_fixed = val;
	}

	void sample_tmpl(Mat & img, Size & sz); 
	void free_tmpl(){
		ptx_tmpl.clear();
	}
};

///////////////////////////////////////////////////////////////// s_frame_obj
struct s_frame_obj{
	long long tfrm;
	double ssd; // sum of square projection errors
	vector<s_obj*> objs;
	Mat camint, camdist;
	bool is_prj;

	s_frame_obj():is_prj(false)
	{
	}

	~s_frame_obj()
	{
		for (int i = 0; i < objs.size(); i++)
			delete objs[i];
		objs.clear();
	}

	void proj_objs(bool bjacobian = true, bool fix_aspect_ratio = true);

	void sample_tmpl(Mat & img, Size & sz)
	{
		for(int i = 0; i < objs.size(); i++){
			objs[i]->sample_tmpl(img, sz);
		}
	}

	void free_tmpl(){
		for(int i = 0; i < objs.size(); i++){
			objs[i]->free_tmpl();
		}
	}

	bool init(const long long atfrm, s_frame_obj * pfobj0, s_frame_obj * pfobj1, 
		vector<Mat> & impyr, c_imgalign * pia, int & miss_tracks);

	bool save(const char * aname);
	bool load(const char * aname, vector<s_model*> & mdls);
};

//////////////////////////////////////////////////////////////// The filter
// The functions of the filter
// 2D object annotation 
//	- manually specify feature point 
//  - manually bind 3d model
//  - manually bind 2d point to 3d model point
// Optimization for camera parameters and object attitud
class f_inspector: public f_ds_window
{
private:
	//
	// input channel
	//
	ch_image * m_pin;

	bool new_frame(Mat & img, long long & timg);

	// 
	// image information to be processed
	//
	long long m_timg;	// time stamp of the time frame
	Mat m_img;			// image frame
	Mat m_img_s;		// scaled image
	Mat m_img_gry;		// gray scale image
	double m_sig_gb; // variance for gaussian blur
	Mat m_img_gry_blur; // gray scale image with Gaussian blur
	vector<Mat> m_impyr; // image pyramid of the gray image
	int m_lvpyr;		// level of the pyramid
	double m_sh, m_sv; // horizontal and vertical scale. 

	// Tracking module
	bool m_btrack_obj;
	e_warp_type m_wt;
	c_imgalign m_ia;
	int m_miss_tracks;
	Size m_sz_vtx_smpl;

	// Rendering method for whole view
	void render(Mat & imgs, long long timg);
	void renderInfo();
	void renderCursor();

	//
	// operation mode
	//
	enum e_operation {
		MODEL, OBJ, POINT, CAMERA, CAMTBL, ESTIMATE, FRAME, VIEW3D
	} m_op;

	static const char * m_str_op[VIEW3D+1]; 

	// sub operation
	enum e_sub_operation{
		SOP_NULL, SOP_SAVE, SOP_LOAD, SOP_GUESS, SOP_DET, SOP_INST_OBJ, SOP_DELETE, SOP_REINIT_FOBJ, SOP_INS_CPTBL
	} m_sop;

	static const char * m_str_sop[SOP_INS_CPTBL + 1];

	void handle_sop_delete();
	void handle_sop_save();
	void handle_sop_load();
	void handle_sop_guess();
	void handle_sop_inst_obj();
	void handle_sop_reinit_fobj();
	void handle_sop_ins_cptbl();
	void handle_sop_det();

	// helper function for handle_sop_guess()
	void help_guess(s_obj & obj, double z, double cx, double cy, double & sfx, double & sfy);

	bool m_bundistort;	// undistort flag. it cant be used with model handling mode.

	//
	// model 
	//
	char m_fname_model[1024]; // name of the model file
	int m_cur_model; // current selected model
	vector<s_model*> m_models; // storing models
	bool load_model();

	// 
	// model view 
	//
	c_d3d_camview m_model_view; // rendering surface

	// 3D model in the model view
	double m_theta_z_mdl, m_dist_mdl;
	Mat m_rvec_mdl, m_tvec_mdl;
	Mat m_rvec_cam_mdl, m_tvec_cam_mdl;
	Mat m_cam_int_mdl, m_cam_dist_mdl;

	void renderModel(long long timg);

	//
	// Scene view
	//
	enum e_view{
		EV_CAM, EV_OBJX, EV_OBJY, EV_OBJZ, EV_FREE
	} m_ev;
	static const char * m_str_view[EV_FREE + 1];
	Mat m_cam_int_view, m_cam_dist_view, m_tvec_view, m_rvec_view;
	void renderScene(long long timg);

	//
	// frame objects
	// 
	bool m_bauto_load_fobj, m_bauto_save_fobj;
	vector<s_frame_obj*> m_fobjs;
	int m_cur_frm;
	bool save_fobjs();
	bool load_fobjs();

	//
	// Object
	// 
	char m_name_obj[1024]; // name of the object file.
	int m_cur_obj; // current object selected
	int m_num_cur_objs;
	int m_cur_point; // current selected point of the model
	void renderObj();

	//
	// Camera Parameter
	//
	char m_fname_campar[1024]; // name of camera parameter file
	char m_fname_campar_tbl[1024]; // name of camera parameter table for multiple magnifications
	double m_depth_min, m_depth_max;
	Mat m_cam_int, m_cam_dist; // Intrinsic camera parameter.	
	Mat m_rvec_cam, m_tvec_cam; // Extrinsic camera parameter.

	double m_erep;		// reprojection error of last current camera parameter.
	Mat m_jcam_max;
	void calc_jmax();

	// master camera parameter (increasing order in f_x)
	bool m_bcam_tbl_loaded;
	int m_cur_camtbl;
	vector<Mat> m_cam_int_tbl;
	vector<Mat> m_cam_dist_tbl;

	enum e_campar{
		ECP_FX = 0, ECP_FY, ECP_CX, ECP_CY, ECP_K1, ECP_K2, ECP_P1, ECP_P2, ECP_K3, ECP_K4, ECP_K5, ECP_K6
	};

	static const char * m_str_campar[ECP_K6+1];
	int m_cur_campar;

	// for parameter estimation
#define ERROR_TOL 0.01
	enum e_estimation_mode{
		EMD_STOP = 0, EMD_FULL, EMD_RT, EMD_SEL
	} m_emd;

	static const char * m_str_emd[EMD_SEL + 1];

	enum e_estimation_state{
		EES_DIV, EES_CONV, EES_CONT
	} m_eest;

	int m_num_max_itrs;		// maximum gauss newton iteration
	int m_num_max_frms_used; // frame counts used for the parameter estimation
	int m_err_range; // error range of reprojection error of usable frame 
	int m_num_pts_used;
	vector<bool> m_frm_used;
	double m_cam_erep;
	void calc_erep();

	// calibration flag. these flags are interpreted into OpenCV's flag of calibrateCamera.
	CvLevMarq m_solver;

	bool m_bcalib_fix_campar;
	bool m_bcalib_fix_focus;
	bool m_bcalib_use_intrinsic_guess;
	bool m_bcalib_fix_principal_point;
	bool m_bcalib_fix_aspect_ratio;
	bool m_bcalib_zero_tangent_dist;
	bool m_bcalib_fix_k1, m_bcalib_fix_k2, 
		m_bcalib_fix_k3, m_bcalib_fix_k4, 
		m_bcalib_fix_k5, m_bcalib_fix_k6;
	bool m_bcalib_rational_model;

	void update_campar();
	void renderCampar();

	bool addCamparTbl();
	bool delCamparTbl();
	bool saveCamparTbl();
	bool loadCamparTbl();
	void clearCamparTbl();

	void acc_Hcamint(Mat & Hcamint, vector<s_obj*> & objs);
	void copy_Hcamint(Mat & Hcamint, vector<s_obj*> & objs);
	void update_params(vector<s_obj*> & objs);
	void make_param_indices(vector<int> & ipars);
	void estimate();
	void estimate_fulltime();
	void estimate_levmarq();
	void select_opt_campar();
	void estimate_rt_levmarq();
	virtual bool alloc_d3dres();
	virtual void release_d3dres();
public:
	f_inspector(const char * name);
	virtual ~f_inspector();

	virtual bool init_run()
	{
		if(m_chin.size() != 1)
			return false;

		m_pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(m_pin == NULL)
			return false;

		if(!f_ds_window::init_run())
			return false;

		return true;
	}

	virtual void destroy_run()
	{
		f_ds_window::destroy_run();
		m_model_view.release();
		for(int ifobj = 0; ifobj < m_fobjs.size(); ifobj++)
			delete m_fobjs[ifobj];
		m_fobjs.clear();
	}

	virtual bool proc();

	////////////////////////////////////////////////////////// UI related members
	enum e_mmode{
		MM_NORMAL, MM_SCROLL, MM_POINT, MM_CAMINT
	} m_mm;

	// The filter rotates and translates objects around and toward the axis.
	// initially set at AX_X
	enum e_axis{
		AX_X, AX_Y, AX_Z
	} m_axis;
	static const char * m_axis_str[AX_Z + 1];
	int m_adj_pow;
	double m_adj_step;

	Point2i m_mc; // mouse cursor
	Point2i m_pt_sc_start; // scroll start
	Point2f m_main_offset;
	float m_main_scale;
	virtual void handle_lbuttondown(WPARAM wParam, LPARAM lParam);
	virtual void handle_lbuttonup(WPARAM wParam, LPARAM lParam);
	void assign_point2d();

	virtual void handle_lbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousewheel(WPARAM wParam, LPARAM lParam);
	void zoom_screen(short delta);
	void translate_obj(short delta);
	void rotate_obj(short delta);
	void adjust_cam(short delta);

	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam);
	void scroll_screen();
	void shift_cam_center();

	virtual void handle_keydown(WPARAM wParam, LPARAM lParam);
	void handle_vk_left();
	void handle_vk_up();
	void handle_vk_right();
	void handle_vk_down();

	virtual void handle_syskeydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keyup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_char(WPARAM wParam, LPARAM lParam);
};

// Function definition
// * Main window shows always video image
// * Main window can be scrolled and scaled at any operation state
//  Shift+LDrag: scroll
//  Shift+Wheel: scaling
//  R: Reset Window
//  L: Load Frame object
//  S: Save Frame object
//  Up: parameter adjustment step up
//  Down: parameter adjustment step down
//
// State {Frame, Model, Obj, Point, Camera, CamTbl, View3D}
// State transition (always it can work)
// <input>: <Action>
//  M: op <= Model
//  O: op <= Obj
//  E: op <= Estimate
//  C: op <= Camera
//  T: op <= CameraTbl
//  P: op <= Point
//  F: op <= Frame
//  V: op <= View3D
//
// op = Model
//  * Show the model in full screen
//	I: Instantiate and initialize New Object, op <= Obj, cur_obj = new_obj
//  L: Load Model (need to specify model file), cur_model = new_model
//  ->: cur_model++
//  <-: cur_model--
//  Del: Delete the model
//
// op = Obj
//  * Show the bounded model in the sub window
//  * overlay the bounded model with the given attitude
//  L: Load Obj (need to specify object file), cur_obj = new_obj
//  S: Save Obj (need to specify object file)
//  ->: cur_obj++
//  <-: cur_obj--
//  x: axis <= x
//  y: axis <= y
//  z: axis <= z
//  Ctrl+Wheel: rotation around axis
//  Wheel: translation along axis
//  f: fix attitude[cur_obj]
//  Del: Delete the object
// 
// op = Point
//  * overlay the bounded model with the given attitude, and highlight the selected point
//  ->: cur_point++
//  <-: cur_point--
//  LB: point[cur_point] = mc
//  x: axis <= x
//  y: axis <= y
//  z: axis <= z
//  Ctrl+Wheel: rotation around axis
//  Wheel: translation along axis
//  f: fix attitude[cur_obj]
//  Del: Delete the point assigned
// 
// op = Camera
// * show camera center and the distortion grid
//  ->: cur_par++
//  <-: cur_par--
//  Wheel: Change selected parameter value
//
// op = CamTbl
//  ->: cur_camtbl++
//  <-: cur_camtbl--
//  Del: Delete the camera parameter
//  I : Insert current parameter to the table
//  L : Load Camera parameter table
//  S : Save Camera parameter table
//	s : set current campar
//
// op = Estimate
//  1: full parameter estimation EMD_FULL
//  2: Camera parameter selection and attitude optimization mode EMD_SEL
//  3: rotation/translation estimation EMD_RT
//
// op = Frame
//  L : Load Camera parameter table
//  S : Save Camera parameter table
//
// op = View3D
// ParMode {ObjZ, ObjX, ObjY, Free, Cam}
// ->: View mode ++
// <-: View mode --
//  x: axis <= x
//  y: axis <= y
//  z: axis <= z
//  Ctrl+Wheel: rotation around axis (3D view campar, enabled in Free mode)
//  Wheel: translation along axis (3D view campar, enabled in Free mode)
// 
#endif
