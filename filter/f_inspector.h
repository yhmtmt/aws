
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
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

	//
	// operation mode
	//
	enum e_operation {
		NORMAL, MODEL,
		DET_CHSBD, SAVE_CHSBDS, LOAD_CHSBDS,CLEAR_CHSBDS,
		CALIB, SAVE_CAMPAR, LOAD_CAMPAR, CLEAR_CAMPAR,
		DET_POSE_CAM, DET_POSE_CAM_TBL, DET_POSE, UNKNOWN
	};
	static const char * m_str_op[UNKNOWN]; 
	e_operation m_op;

	bool m_bundistort;	// undistort flag. it cant be used with model handling mode.
	bool m_bpttrack;	// point tracking flag
	bool m_bcbtrack;	// chessboard tracking
	bool m_bshow_chsbd; // chessboard detected in current frame is marked.
	bool m_bcampar_fixed;	// indicates the condition of the camera parameter in this frame
	bool m_bpose_fixed;		// indicates the model pause is fixed in this frame
	bool m_bchsbd_found;	// indicates chessboard is found in this frame

	// 
	// file name for read/write
	//
	char m_fname_chsbds[1024]; // name of chsbd collection
	char m_fname_model[1024]; // name of model file
	char m_fname_campar[1024]; // name of camera parameter file

	//
	// chessboard
	//
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

	//
	// model 
	//
	vector<string> m_name_model; // name of the model
	vector<vector<vector<Point2f> > > m_point_2d;
	vector<vector<Point3f > > m_point_3d; // model points in model coordinate
	vector<vector<vector<int> > > m_edge_model; // model edge list
	int m_cur_model; // current selected model
	int m_cur_model_point; // current selected point of the model
	vector<vector<Point2f > > m_cur_point_2d;

	// model poses in each time frame
	vector<long long> m_pose_time;		// times corresponding model pose
	vector<bool> m_bmodel_pose_fixed;	// true if model pose is fixed
	vector<vector<vector<Mat > > > m_rvecs_model; // rvec for the models in the time
	vector<vector<vector<Mat > > > m_tvecs_model; // tvec for the models in the time

	void seekModelTime(long long time){};

	//
	// Camera Parameter
	//
	Mat m_cam_int, m_cam_dist;// current camera parameter.	
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

	int m_num_chsbds_calib;
	bool chooseChsbds(vector<vector<Point2f > > & chsbds, vector<int> & id_chsbd);
	void calibChsbd(Mat & img); // calibration is done with chessboard stocked
	void guessCamparPauseChsbd(long long timg);
	void guessCamparPauseModel(long long timg);

	bool saveCampar();
	bool loadCampar();
	void clearCampar();

	// 
	// 3D view 
	//
	c_d3d_camview m_3dscene; // rendering surface
	enum e_3dmode{
		SUB, FULL, OVLY, NONE3D, UNKNOWN3D
	} m_3dmode;
	static const char * m_str_3dmode[UNKNOWN3D];

	LPD3DXMESH m_pmesh_chsbd;
	LPDIRECT3DTEXTURE9 m_ptex_chsbd;

	void render3D(long long timg);
	void renderChsbd(long long timg);
	void renderModel(long long timg);
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

		initChsbd3D();

		if(strlen(m_fname_chsbds)){
			loadChsbds();
		}

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
		m_3dscene.release();
	}

	virtual bool proc();


	enum e_mmode{
		MM_NORMAL, MM_SCROLL
	} m_mm;

	Point2i m_mc; // mouse cursor
	Point2i m_pt_sc_start; // scroll start
	Point2i m_main_offset;
	float m_main_scale;

	virtual void handle_lbuttondown(WPARAM wParam, LPARAM lParam);
	virtual void handle_lbuttonup(WPARAM wParam, LPARAM lParam);

	virtual void handle_lbuttondblclk(WPARAM wParam, LPARAM lParam);
	virtual void handle_rbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousewheel(WPARAM wParam, LPARAM lParam);
	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam);
	virtual void handle_keydown(WPARAM wParam, LPARAM lParam);
	virtual void handle_syskeydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keyup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_char(WPARAM wParam, LPARAM lParam);
};