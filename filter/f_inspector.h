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
#include "../util/aws_vlib.h"
#include "../util/aws_vobj.h"
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
//#include "../util/aws_cminpack.h"
#include "../util/aws_coord.h"

#include "../util/c_ship.h"

//#include "../util/aws_nmea.h"
#include "../channel/ch_image.h"

#include "f_ds_window.h"

///////////////////////////////////////////////////////////////////////////////////////////////// AWSLevMarq
// AWSLevMarq is the extension of CvLevMarq.
// Because we need to evaluate Hessian inverse of the last iteration, the 
class AWSLevMarq: public CvLevMarq
{
public:
	AWSLevMarq():CvLevMarq()
	{
		Cov = Ptr<CvMat>();
	}

	AWSLevMarq(int nparams, int nerrs, CvTermCriteria criteria0 =
              cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON), 
			  bool _completeSymmFlag = false):CvLevMarq()
	{
		Cov = Ptr<CvMat>();
		initEx(nparams, nerrs, criteria0, _completeSymmFlag);
	}

	~AWSLevMarq()
	{
		clearEx();
	}

	cv::Ptr<CvMat> Cov;

	void initEx(int nparams, int nerrs, CvTermCriteria criteria0 =
              cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON), 
			  bool _completeSymmFlag = false)
	{
		init(nparams, nerrs, criteria0, _completeSymmFlag);
		Cov = cvCreateMat(nparams, nparams, CV_64F);
	}

    void clearEx();

	bool updateAltEx( const CvMat*& param, CvMat*& JtJ, CvMat*& JtErr, double*& errNorm );

	// call immediately after the iteration terminated.
	void calcCov();
};

////////////////////////////////////////////////////////////////////////////////////////////////// 3D model tracking 
#define DEBUG_MODELTRACK

// 1. set initial parameter p = (r, t), and transformation T(p) 
// 2. calculate point matching error, jacobian, hessian inverse, and delta_p  
// 3. set new transformation T(p) = T(delta_p)T(p)
class ModelTrack
{
private:
	AWSLevMarq solver;	// LM solver
	CvTermCriteria tc;

	// Rini and tini are the initial transformation (Rotation and Translation)
	// If the parameter is not given (means empty) initialized as an identity matrix and a zero vector.
	Mat Rini, tini; 

	// Rnew ant tnew are the current transformation in the iteration. 
	// It contains the component given in Rini and tini. Therefore, they are initialized by Rini and tini.
	Mat Rnew, tnew;
	double * pRnew, * ptnew;

	// Racc and tacc are the transformation component accumulating all the update.
	// Note that these do not include Rini and tini, and are initialized with an identity matrix and a zero vector.
	// These are required to calculate the move of pixels around points approximated as planer in the previous frame.
	Mat Racc, tacc; 
	double * pRacc, * ptacc;

	// Rdelta and tdelta are the update of the transformation in each iteration.
	// They are multiplied to both [Rnew|tnew] and [Racc|tacc] from their left side.
	// For multiplication with [Rnew|tnew], because CvLevMarq requires convergence check, temporalily we need 
	// to treate new transformation as temporal value, and rewind it to the original if the error was not reduced.
	// For the purpose, Rtmp and ttmp are prepared below.
	Mat Rdelta, tdelta;
	double * pRdelta, *ptdelta;

	// Rtmp and ttmp are the temporal variables for Rnew and tnew during error checking phase.
	Mat Rtmp, ttmp;
	double * pRtmp, *pttmp;

	// Racctmp and tacctmp are the temporal variables for Racc and tacc during error checking phsase. 
	Mat Racctmp, tacctmp;
	double * pRacctmp, * ptacctmp;

	// building point patch pyramid and its derivative
	// Ppyr is the pyramid image of the point patch. 
	// (We use rectangler region around the object points as point patch to be tracked."
	vector<vector<Mat>> Ppyr;
	vector<double> Pssd;

	Mat JRtrt0acc;
	Mat JUrt0acc;
	Mat J, E;

	// Differential filter kernel.
	// dxr and dxc are the row and column filter for x derivative.
	// dyr and dyc are those for y derivative.
	Mat dxr, dxc, dyr, dyc; 

	void reprj(int ilv, const uchar * pI, int w, int h, int sx, int sy, double ox, double oy, 
		double fx, double fy, double ifx, double ify, double cx, double cy, const vector<Point3f> & Mo, 
		vector<Point3f> & Mcold, vector<Point2f> & m, int neq, double * pE, vector<int> & valid, 
		vector<Point3f> & Mc, double * pIx, double * pIy, double * pJ, CvMat * _JtJ, CvMat * _JtErr);

	void reprjNoJ(int ilv, const uchar * pI, int w, int h, int sx, int sy, double ox, double oy, 
		double fx, double fy, double ifx, double ify, double cx, double cy, const vector<Point3f> & Mo, 
		vector<Point3f> & Mcold, vector<Point2f> & m, int neq, double * pE, vector<int> & valid);

	// debug method
	// saves reprojected area 
#ifdef DEBUG_MODELTRACK
	char fsmpl[128];
	vector<vector<Mat>> Pyrsmpl;
	vector<vector<Mat>> Pyrsmplx;
	vector<vector<Mat>> Pyrsmply;
#endif
public:
	ModelTrack()
	{
		tc.epsilon = FLT_EPSILON;
		tc.max_iter = 30;
		tc.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;

		// preparing differential filter kernel
		getDerivKernels(dxr, dxc, 1, 0, 3, true, CV_64F);
		getDerivKernels(dyr, dyc, 0, 1, 3, true, CV_64F);
	}

	// Ipyr is the grayscaled Image pyramid.
	// P is the set of image patches around model points.
	// D is the set of depth maps corresponding to patches in P.
	// M is the set of 3D points in the model.
	// T is the initial value of the transformation, and the resulting transformation.
	// m is tracked points. 
	bool align(const vector<Mat> & Ipyr, const vector<Point3f> & M, vector<int> & valid, 
		const vector<Mat> & P, Mat & camint, Mat & R, Mat & t, vector<Point2f> & m, int & miss);
};


inline double rerr(double a, double b){
	return fabs((a - b) / max(fabs(b), (double)1e-12));
}

///////////////////////////////////////////////////////////////// s_frame
struct s_frame{
	bool kfrm;
	Mat img; // Image data. This field is used only for key frame

	union {
		long long tfrm;
		s_frame * ptr;
	};

	double ssd; // sum of square projection errors
	vector<s_obj*> objs;
	Mat camint, camdist;

	// Other possible frame objects
	// key points and the descriptors, coners, edges, something like that.
	// gray image, differential image, image pyramid, 
	bool update;

	s_frame():update(false), kfrm(false)
	{
	}

	~s_frame()
	{
		release();
	}

	void proj_objs(bool bjacobian = true, bool fix_aspect_ratio = true);
	void calc_rpy(int base_obj = 0, bool xyz = true);
	void calc_rpy_and_pts(vector<vector<Point3f> > & pts, Mat & Rcam, Mat & Tcam, vector<Point3f> & acam, int base_obj = 0, bool xyz = true);
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

	void release()
	{
		update = false;
		kfrm = false;
		for (int i = 0; i < objs.size(); i++)
			delete objs[i];
		objs.clear();
		img.release();
	}

	bool init(const long long atfrm, s_frame * pfobj0, s_frame * pfobj1, 
		vector<Mat> & impyr, c_imgalign * pia, int & miss_tracks);
	bool init(const long long atfrm, s_frame * pfobj0, 
		vector<Mat> & impyr, ModelTrack * pmdlTrck, int & miss_tracks);

	bool init(const long long atfrm, Mat & acamint, Mat & acamdist)
	{
		tfrm = atfrm;
		acamint.copyTo(camint);
		acamdist.copyTo(camdist);
		return true;
	}

	bool save(const char * aname);
	bool load(const char * aname, long long atfrm, vector<s_model*> & mdls);

	void set_update()
	{
		update = true;
	}

	void set_as_key(Mat & _img){
		kfrm = true;
		img = _img.clone();
	}

	// memory pool for frame object
	static s_frame * pool;
	static void free(s_frame * pfobj)
	{
		if(pfobj == NULL)
			return;
		
		pfobj->release();
		pfobj->ptr = pool;
		pool = pfobj;
	}

	static s_frame * alloc(){
		if(pool != NULL){
			s_frame * pfobj = pool;
			pool = pool->ptr;
			return pfobj;
		}

		return new s_frame;
	}

};

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

	bool m_bnew_frm;
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
	bool m_btrack_obj_3d;

	e_warp_type m_wt;
	c_imgalign m_ia;
	ModelTrack m_mdlTrck;

	int m_miss_tracks;
	Size m_sz_vtx_smpl;

	// Rendering method for whole view
#define DEBUG_IMALIGN
	bool m_brender_grid;

	void render(Mat & imgs);
	void renderInfo();
	void renderGrid();
	void renderModelInfo(char * buf, int len, int & y);
	void renderObjInfo(vector<s_obj*> & objs, char * buf, int len, int & y);
	void renderPartsInfo(vector<s_obj*> & objs, char * buf, int len, int & y);
	void renderPointInfo(vector<s_obj*> & objs, char * buf, int len, int & y);
	void renderCamparInfo(char * buf, int len, int & y);
	void renderCamparTblInfo(char * buf, int len, int & y);
	void renderEstimateInfo(char * buf, int len, int & y);
	void renderFrameInfo(char * buf, int len, int & y);
	void renderKeyFrameInfo(char * buf, int len, int & y);
	void renderCursor();

	//
	// operation mode
	//
	enum e_operation {
		MODEL, OBJ, PARTS, POINT, CAMERA, CAMTBL, ESTIMATE, FRAME, VIEW3D
	} m_op;

	bool m_bkfrm;

	static const char * m_str_op[VIEW3D+1]; 

	// sub operation
	enum e_sub_operation{
		SOP_NULL, SOP_SAVE, SOP_LOAD, SOP_GUESS, SOP_DET, SOP_INST_OBJ, SOP_DELETE, 
		 SOP_SET_KF, SOP_SEEK_KF_FWD, SOP_SEEK_KF_BK, SOP_INS_CPTBL, SOP_AWSCMD
	} m_sop;

	static const char * m_str_sop[SOP_AWSCMD + 1];
	
	void handle_sop_delete();
	void handle_sop_save();
	void handle_sop_load();
	void handle_sop_guess();
	void handle_sop_inst_obj();

	bool m_bstep_issued;
	void handle_sop_seek_kf(bool fwd = true);
	void handle_sop_ins_cptbl();
	void handle_sop_det();

	void handle_sop_awscmd();
	char m_cmd_buf[1024]; // storing command string. used in awscmd.
	char m_cmd_ret[1024]; // storing returned string. used in awscmd.

	void handle_sop_set_kf();

	// helper function for handle_sop_guess()
	void help_guess(s_obj & obj, double z, double cx, double cy, double & sfx, double & sfy, bool fix_aspect_ratio = true);

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

	void renderModel();

	//
	// Scene view
	//
	bool m_bdecomp_xyz; // decompose rotation matrix in x-y-z order.
	enum e_view{
		EV_CAM, EV_OBJX, EV_OBJY, EV_OBJZ, EV_FREE
	} m_ev;
	static const char * m_str_view[EV_FREE + 1];
	Mat m_cam_int_view, m_cam_dist_view, m_tvec_view, m_rvec_view;
	double m_rat_z;
	void renderScene();
	void renderSceneInfo(char * buf, int len, int & y);

	// Key frames 
	// * Key frames are automatically allocated with the interval m_int_kfrms, 
	//   and automatically deallocated if the buffer is full.
	// * Key frames are stored in ring buffer m_kfrms, and m_cur_kfrm points the current key frame
	// * we can view all key frames by key frame view mode 'K'
	// * we can set arbitraly current frame as a key frame by 'k'. Next key frame is just after m_int_kfrms.
	bool m_bald_kfrms, m_basv_kfrms;// flags for auto load and save key frames
	int m_num_kfrms;				// number of key frames cached in the memory.
	int m_cur_kfrm;					// current key frame
	int m_sel_kfrm;					// selected key frame in KFRAME mode.
	int m_int_cyc_kfrm;				// it defines the keyframe's interval as the number of aws cycles. The number of cycle skiped by right vk_left and vk_right in frame operation.
	long long  m_int_kfrms;			// Keframe's interval in 10e-7 second. Subject to m_int_cyc_kfrm

	vector<s_frame*> m_kfrms;		// Key frames (ring buffer)
	bool save_kfrms();
	bool save_analytics(ofstream & file, char * fname, int ikf0, long long tmin, 
		vector<s_obj*> & objptr, int num_objs, int base_obj, bool bxyz);
	bool load_kfrms();
	bool m_bsave_objpts;			// flag enables transformed object points. 

	s_frame * m_pfrm;				// temporal frame object
	s_frame * m_pfrm_int;			// frame object of interest

	//
	// frame objects
	// 
	bool m_bauto_load_fobj, m_bauto_save_fobj;

	//
	// Object
	// 
	char m_name_obj[1024];	// name of the object file.
	int m_cur_obj;			// current object selected
	int m_num_cur_objs;		// number of objects in current frame
	int m_cur_point;		// current selected point of the model
	int m_cur_part;			// number of parts in current object.

	// Drawing object in the view.
	void renderObj(s_frame * pfrm);

	//
	// Camera Parameter
	//
	static const char * m_str_campar[ECP_K6+1];

	char m_fname_campar[1024]; // name of camera parameter file
	char m_fname_campar_tbl[1024]; // name of camera parameter table for multiple magnifications
	double m_depth_min, m_depth_max;
	Mat m_cam_int, m_cam_dist; // Intrinsic camera parameter.	
	Mat m_rvec_cam, m_tvec_cam; // Extrinsic camera parameter.

	double m_erep;		// reprojection error of last current camera parameter.
	Mat m_jcam_max;
	void calc_jmax();

	bool m_bcam_tbl_loaded;
	int m_cur_camtbl;			// Selected parameter in the camera parameter table here
	vector<Mat> m_cam_int_tbl;  // Table of camera intrinsic parameters
	vector<Mat> m_cam_dist_tbl; // Table of camera distortion parameters

	int m_cur_campar;			// currently selected camera parameter. Valid only if the operation is CAMERA.

	// for parameter estimation
#define ERROR_TOL 0.01
	enum e_estimation_mode{
		EMD_STOP = 0, EMD_FULL, EMD_RT, EMD_RT_FULL, EMD_SEL
	} m_emd;

	static const char * m_str_emd[EMD_SEL + 1];

	enum e_estimation_state{
		EES_DIV, EES_CONV, EES_CONT
	} m_eest;

	int m_num_max_itrs;			// maximum gauss newton iteration
	int m_num_max_frms_used;	// frame counts used for the parameter estimation (now used only in estimate_fulltime. this is old code to be discarded in the future)
 	int m_err_range;			// error range of reprojection error of usable frame 
	int m_num_pts_used;			// number of points used for the estimation
	vector<bool> m_kfrm_used;	// number of key frames used for the estimation
	double m_cam_erep;			// RMSE of the estimation (sqrt(SSD / num_points))
	void calc_erep();			// SSD calcuration method

	//CvLevMarq m_solver;			// LM solver from openCV
	AWSLevMarq m_solver;		// Modified LM solver oriinaly from openCV

	// calibration flag. these flags are interpreted into OpenCV's flag of calibrateCamera.
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
	void estimate_rt_levmarq(s_frame * pfrm);
	void estimate_rt_full_levmarq();
	void estimate_rt_full_and_sel_cptbl();
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

		m_main_offset = Point2d(0, 0);
		m_main_scale = (float) m_rat;
		m_main_scale_inv = (float)(1.0 / m_main_scale);

		m_kfrms.resize(m_num_kfrms, NULL);
		m_cur_kfrm = -1;

		return true;
	}

	virtual void destroy_run()
	{
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
	int m_adj_pow;
	double m_adj_step;

	Point2i m_mc; // mouse cursor
	Point2i m_pt_sc_start; // scroll start
	Point2f m_main_offset;
	float m_main_scale, m_main_scale_inv;

	Size m_sz_img_view, m_sz_img;
	// coordinate conversion. Screen(View) coordinate to Image coordinate.
	void cnv_view2img(const Point2f & pview, Point2f & pimg){
		pimg = (pview - m_main_offset);
		pimg *= m_main_scale_inv;
	}

	// coordinate conversion. Image coordinate to Screen(View) coordinate.
	void cnv_img2view(const Point2f & pimg, Point2f & pview){
		pview = m_main_scale * pimg;
		pview += m_main_offset;
	}

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
	void adjust_part(short delta);

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
	void handle_char_f();
};

// img = (view - ofst) / (scale * rat)
//  
// m_main_cam.show() changes scale as (scale * rat)
//      (rendering surface always be the same size as the original image)
//      (rendering coordinate is right-top coordinate. the rendering offset should be careful (ofst.x, ofst.y+height)
//
// changes are in
// zoom_screen
// assign_point2d
// handle_cahr 'R' m_main_scale returns to rat

// Function definition
// * Main window shows always video image
// * Main window can be scrolled and scaled at any operation state
//  Shift+LDrag: scroll
//  Shift+Wheel: scaling
//  R: Reset Window
//  h: help overlay mode enable
//	k : Set as Key frame
//  g : Show grid
//  < : step backward
//  > : step forward
//  Up: parameter adjustment step up
//  Down: parameter adjustment step down
//
// State {Frame, Model, Obj, Point, Camera, CamTbl, View3D}
// State transition (always it can work)
// <input>: <Action>
//  M: op <= Model
//  O: op <= Obj
//  Q: op <= Parts
//  E: op <= Estimate
//  C: op <= Camera
//  T: op <= CameraTbl
//  P: op <= Point
//  F: op <= Frame
//  K: op <= KeyFrame
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
// op = Part
// * overlay the object, and highlight the selected part
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
// -> : step to the later frame. (depending on m_int_cyc_kfrm)
// <- : step back to the former frame. (depending on m_int_cyc_kfrm)
// 
// op = KeyFrame
// L : Load key frame
// S : Save key frame
// -> : See Next Key Frame
// <- : See Previous Key Frame

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
