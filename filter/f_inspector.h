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

#include "f_ds_window.h"
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

// s_model represents a 3D model tracked in the scene by f_inspector.
// It contains points and edges, and their projection method.
struct s_obj;
struct s_model
{
	const char * fname;
	int ref;
	string name;
	vector<Point3f> pts;
	vector<s_edge> edges;
	float xmin, ymin, zmin, xmax, ymax, zmax;

	s_model():ref(0), xmin(FLT_MAX), ymin(FLT_MAX), zmin(FLT_MAX),
		xmax(-FLT_MAX), ymax(-FLT_MAX), zmax(-FLT_MAX)
	{
	}

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
};

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
	vector<bool> bvisible; // true if 2d point is visible in the image
	bool is_attitude_fixed;
	Mat tvec, rvec;

	s_obj(): pmdl(NULL), name(NULL), is_attitude_fixed(false){
		tvec = Mat::zeros(3, 1, CV_64FC1);
		rvec = Mat::zeros(3, 1, CV_64FC1);
	};

	~s_obj()
	{
		delete[] name;
		name = NULL;
		if(pmdl)
			pmdl->ref--;
	}

	int get_num_points(){
		return (int) pt2d.size();
	}

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

	// draw the wire frame model
	void render(
		Mat & camint, Mat & camdist, Mat & rvec_cam, Mat & tvec_cam,
		LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
		int pttype, int state, int cur_point = -1);

	// render vector in comparable size to the object  
	void render_vector(Point3f & vec,
		Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
		LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline);

	// render axis in three direction
	void render_axis(Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
		LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline, int axis = -1);

	bool init(s_model * apmdl, long long t, const Mat & camint, const Mat & camdist,
		const double width, const double height);
	bool load(const char * aname, long long at, vector<s_model> & mdls);
	bool save();

	void fixAttitude(bool val){
		is_attitude_fixed = val;
	}
};

// The functions of the filter
// 2D object annotation 
//	- manually specify feature point 
//  - manually bind 3d model
//  - manually bind 2d point to 3d model point
// Optimization for camera parameters and object attitude
// Finding chessboard and calibrating camera
// Table based handling of camera parameters with various magnifications
// load and save camera parameters for each time frame
// load and save object information for each time frame
 
class f_inspector: public f_ds_window
{
private:
	//
	// input channel
	//
	ch_image * m_pin;

	// 
	// image information to be processed
	//
	long long m_timg;	// time stamp of the time frame
	Mat m_img;			// image frame
	double m_sh, m_sv; // horizontal and vertical scale. 


	// Rendering method for whole view
	void render(Mat & imgs, long long timg);
	void renderInfo();
	void renderCursor();

	// Parameter calibration
	void calibrate(Mat & img_s, long long timg);

	//
	// operation mode
	//
	enum e_operation {
		MODEL, OBJ, POINT, CAMERA, ESTIMATE
	};

	static const char * m_str_op[ESTIMATE+1]; 
	e_operation m_op;

	bool m_bundistort;	// undistort flag. it cant be used with model handling mode.
	bool m_bpttrack;	// point tracking flag
	bool m_bcampar_fixed;	// indicates the condition of the camera parameter in this frame
	bool m_bpose_fixed;		// indicates the model pause is fixed in this frame

	//
	// chessboard
	// 
	/* now chessboard handling codes are unified into object handling codes
	char m_fname_chsbds[1024]; // name of chsbd collection
	bool m_bcbtrack;	// chessboard tracking
	bool m_bshow_chsbd; // chessboard detected in current frame is marked.
	bool m_bchsbd_found;	// indicates chessboard is found in this frame

	double m_pitch_chsbd;	// chesboard pitch
	Size m_sz_chsbd;		// chesboard size
	vector<Point3f> m_3dchsbd;				// chessboard corners in the world coordinate, automatically constructed by the pitch and size.
	vector<vector<Point2f > > m_2dchsbd;	// chessboard found in the image
	vector<bool> m_bchsbd_pose_fixed;		// true if chessboard pose is fixed.
	vector<Mat> m_rvecs_chsbd;				// chesboard rotation in the time
	vector<Mat> m_tvecs_chsbd;				// chesboard translation in the time
	vector<long long> m_time_chsbd;			// the time chessboard found
	vector<double> m_ereps_chsbd;			// reprojection error for each chessboard
	int m_cur_chsbd;						// index of the current chessboard

	vector<Point2f> m_corners_chsbd;	// temporal data object for findChessboard
	
	void seekChsbdTime(long long timg); // seeks chessboard found at time specified as "timg"
	void findChsbd(Mat & img, long long timg);

	void initChsbd3D();
	void clearChsbds();
	bool saveChsbds();
	bool loadChsbds();

	int m_num_chsbds_calib;
	bool chooseChsbds(vector<vector<Point2f > > & chsbds, vector<int> & id_chsbd);
	void calibChsbd(Mat & img); // calibration is done with chessboard stocked
	void guessCamparPauseChsbd(long long timg);
	*/

	//
	// Object
	// 
	char m_name_obj[1024]; // name of the object file.
	vector<vector<s_obj> > m_obj_trace;
	int m_cur_model; // current selected model
	int m_cur_obj; // current object selected
	int m_cur_point; // current selected point of the model
	vector<s_obj> m_obj; // object points
	void load_obj();
	void save_obj();
	void renderObj();

	//
	// model 
	//
	char m_fname_model[1024]; // name of the model file
	bool m_badd_model;
	vector<s_model> m_models; // storing models
	// loading model when m_badd_model is asserted
	bool load_model();

	// model poses in each time frame
	vector<long long> m_pose_time;		// times corresponding model pose
	vector<bool> m_bmodel_pose_fixed;	// true if model pose is fixed
	vector<vector<vector<Mat > > > m_rvecs_model; // rvec for the models in the time
	vector<vector<vector<Mat > > > m_tvecs_model; // tvec for the models in the time

	void seekModelTime(long long time){};

	// 
	// model view 
	//
	c_d3d_camview m_model_view; // rendering surface

	// 3D model in the model view
	double m_theta_z_mdl, m_dist_mdl;
	Mat m_rvec_mdl, m_tvec_mdl;
	Mat m_rvec_cam_mdl, m_tvec_cam_mdl;
	Mat m_cam_int_mdl, m_cam_dist_mdl;

	LPD3DXMESH m_pmesh_chsbd;
	LPDIRECT3DTEXTURE9 m_ptex_chsbd;

	void renderModel(long long timg);

	//
	// Camera Parameter
	//

	bool m_bload_campar;
	bool m_bload_campar_tbl;

	char m_fname_campar[1024]; // name of camera parameter file
	char m_fname_campar_tbl[1024]; // name of camera parameter table for multiple magnifications
	Mat m_cam_int, m_cam_dist; // Intrinsic camera parameter.	
	Mat m_rvec_cam, m_tvec_cam; // Extrinsic camera parameter.

	double m_erep;		// reprojection error of last current camera parameter.

	// master camera parameter (increasing order in f_x)
	bool m_bcam_tbl_loaded;
	vector<Mat> m_cam_int_tbl;
	vector<Mat> m_cam_dist_tbl;
	vector<double> m_cam_erep;
	
	// calibration flag. these flags are interpreted into OpenCV's flag of calibrateCamera.
	bool m_bcalib_use_intrinsic_guess;
	bool m_bcalib_fix_principal_point;
	bool m_bcalib_fix_aspect_ratio;
	bool m_bcalib_zero_tangent_dist;
	bool m_bcalib_fix_k1, m_bcalib_fix_k2, 
		m_bcalib_fix_k3, m_bcalib_fix_k4, 
		m_bcalib_fix_k5, m_bcalib_fix_k6;
	bool m_bcalib_rational_model;

	bool saveCampar();
	bool loadCampar();
	void clearCampar();
	bool saveCamparTbl();
	bool loadCamparTbl();
	void clearCamparTbl();

	void render3D(long long timg);
	void renderChsbd(long long timg);

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

		/*
		initChsbd3D();

		if(strlen(m_fname_chsbds)){
			loadChsbds();
		}
		*/

		if(strlen(m_fname_campar)){
			loadCampar();
		}

		return true;
	}

	virtual void destroy_run()
	{
		if(m_ptex_chsbd)
			m_ptex_chsbd->Release();
		if(m_pmesh_chsbd)
			m_pmesh_chsbd->Release();
		f_ds_window::destroy_run();
		m_model_view.release();
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
	double m_rot_step;
	double m_trn_step;
	double m_zoom_step;

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
	void translate_cam(short delta);
	void rotate_cam(short delta);
	void zoom_cam(short delta);

	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam);
	void scroll_screen();
	void shift_cam_center();

	virtual void handle_keydown(WPARAM wParam, LPARAM lParam);
	void handle_vk_left();
	void handle_vk_right();

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
//
// State {Model, Obj, Point, Camera}
// State transition (always it can work)
// <input>: <Action>
//  M: op <= Model
//  O: op <= Obj
//  E: op <= Estimate
//  C: op <= Camera
//  P: op <= Point
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
//  L: Load Camera Parameter (need to specify parameter file)
//  S: Save Camera Parameter (need to specify parameter file)
//  ->: cur_par++
//  <-: cur_par--
//  Wheel: Change focal length
//  LDrag: Move Principal Point
//  f : fix par[cur_par]
//  Del: Delete the camera parameter
//
// op = Estimate
//  E: Estimate
//
// render : render whole screen with correct scroll and scale
// renderObj : render overlay model, corresponding points, highlight selected point
// renderModel : render model window
// renderInfo : render Information of the filter state
// instObj : initialize new object instance by a model 
// estimate: 


#endif
