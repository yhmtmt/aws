#ifndef _AWS_VOBJ_H_
#define _AWS_VOBJ_H_

// s_edge represents the edge between points defined in the s_model
// This is used for drawing the wire frame model. 
struct s_edge{
	int s, e;
	s_edge():s(0), e(0){};
};

// s_part describe the part's structure. translation, rotation axis and the origin relative to the body
// part's nesting is not supported yet. it could be the future issue.
struct s_part{
	vector<int> pts; // point indices of the parts (the points are that in the s_model object contains this object)
	int org; // point index for the rotation origin. 
	Point3f axis; // translation or rotation vector
	bool rot, trn; // translation / rotation flag
	s_part():org(0), axis(0., 0., 0.), rot(true), trn(true){	}
};

///////////////////////////////////////////////////////////////// s_model
// s_model represents a 3D model tracked in the scene by f_inspector.
// It contains points and edges, and their projection method.
#define DEBUG_CHSBDDET

class c_imgalign;
class ModelTrack;

struct s_obj;
struct s_model
{
	char fname[1024];
	int ref;
	string name;
	vector<Point3f> pts; // model points
	vector<Point3f> pts_deformed; // temporal data strucuture. immediately before projection, the data structure should have deformed model points.
	vector<s_edge> edges; // model edges
	vector<s_part> parts;
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
	bool load();

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
	Rect bb2d;
	vector<Point2f> pt2d;
	vector<Point2f> pt2dprj;
	vector<int> visible; // true if 2d point is visible in the image

	vector<Mat> ptx_tmpl; // point's template images.

	bool is_attitude_fixed;
	Mat R;
	Mat tvec, rvec;

	double roll, pitch, yaw;
	Point3f pos;

	// note: The part's deformation code is now under construction. I need to modify followings:
	// s_obj::load, save, init, proj
	vector<double> dpart; // part's deformation value.

	bool update;

	Mat jacobian;
	Mat jmax; // maximum values of jacobian for each parameter
	Mat hessian;
	Mat dp;
	Mat err;
	Mat jterr;
	double ssd;
	int match_count;

	s_obj(): pmdl(NULL), name(NULL), is_attitude_fixed(false), 
		roll(0.0), pitch(0.0), yaw(0.0), pos(0., 0., 0.),
		update(false), bb2d(0, 0, 0, 0)
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

	// calculate projection
	void proj(Mat & camint, Mat & camdist, bool bjacobian = true, bool fix_aspect_ratio = true);

	void render(Mat & img);


	bool init(s_model * apmdl, long long at, const Mat & camint, const Mat & camdist,
		const double width, const double height);

	bool init(const s_obj & obj);

	void calc_part_deformation();

	bool load(FileNode & fnobj, vector<s_model*> & mdls);
	bool save(FileStorage & fs);

	void fixAttitude(bool val)
	{
		is_attitude_fixed = val;
	}

	void sample_tmpl(Mat & img, Size & sz); 
	void sample_pt_tmpl(int ipt, Mat & img, Size & sz);
	void free_tmpl(){
		ptx_tmpl.clear();
	}
};

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
	void calc_rpy_and_pts(vector<vector<Point3f> > & pts, int base_obj = 0, bool xyz = true);
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

#endif