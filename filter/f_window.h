// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_window.h.  If not, see <http://www.gnu.org/licenses/>. 


////////////////////////////////Alert////////////////////////////////////
// these highgui windows won't work because of the recent parallelization.
class f_window: public f_base
{
protected:
	ch_image * m_pin;
public:
	f_window(const char * name):f_base(name)
	{
		namedWindow(m_name);
	};

	virtual ~f_window()
	{
		destroyWindow(m_name);
	}

	virtual bool check()
	{
		return m_chin[0] != NULL;
	}

	virtual bool proc(){
		long long timg;
		Mat img = m_pin->get_img(timg);
		if(img.empty())
			return true;
		imshow(m_name, img);
		return true;
	}
};

////////////////////////////////Alert////////////////////////////////////
// these highgui windows won't work because of the recent parallelization.

class f_mark_window: public f_window
{
protected:
	vector<Point2i> m_point;
	int m_scale;

	void push(int x, int y)
	{
		m_point.push_back(Point2i(x, y));
	}

	void write(const char * fname){
		ofstream file;
		file.open(fname, ios_base::trunc);

		if(!file.is_open()){
			cerr << "Failed to open file " << fname << endl;
			return;
		}

		for(int i = 0; i < m_point.size(); i++)
			file << m_point[i].x << " " << m_point[i].y << endl;
	}

	void render(){
		long long timg;
		Mat img = m_pin->get_img(timg);
		if(img.empty())
			return;

		Mat img_show;
		resize(img, img_show, Size(img.cols*m_scale, img.rows*m_scale));

		Point p1, p2;

		for(int i = 0; i < m_point.size(); i++){
			p1.x = m_point[i].x * m_scale - 5;
			p2.y = p1.y = m_point[i].y * m_scale;
			p2.x = p1.x + 10;
			line(img_show, p1, p2, CV_RGB(255, 0, 0), 1);

			p2.x = p1.x = m_point[i].x * m_scale;
			p1.y = m_point[i].y * m_scale - 5;
			p2.y = p1.y + 10;
			line(img_show, p1, p2, CV_RGB(255, 0, 0), 1);
		}
		imshow(m_name, img_show);
	}

	static void on_mouse(int event, int x, int y, int flags, void * param=NULL)
	{
		f_mark_window * pwin = (f_mark_window*) param;
		switch(event){
		case CV_EVENT_LBUTTONDOWN:
			pwin->push(x / pwin->m_scale, y / pwin->m_scale);
		}
		pwin->render();
	}

public:
	f_mark_window(const char * name);
	virtual ~f_mark_window();

	virtual bool proc(){
		render();
		return true;
	}

	virtual bool cmd_proc(s_cmd & cmd);
};

#ifdef _WIN32
////////////////////////////////////////////////////////////////
/////////////// direct X based window //////////////////////////
#define SYS_BUF_SIZE 512

class f_ds_window:public f_base
{
protected:
	pthread_t m_th_msg;

	int m_Hfull, m_Vfull;
	void fit_full_mode();

	int m_Vwin, m_Hwin; 
	void fit_win_mode();

	double m_rat;

	HINSTANCE m_hinst;	// instance handle of the application
	TCHAR * m_name_t;	// window title
	TCHAR * m_clsname_t; //registered window class name 
	WNDCLASSEX m_wclsx; // window class registered
	HWND m_hwnd;		// handle of the created window 

	bool create_wnd();

	/////////////////////////////////// direct3d objs

	UINT m_iadapter;	// video card index to be used
	bool m_blost;		// flag for device lost
	bool m_bwin;	// flag for window mode (false if fullscreen)
	pthread_mutex_t m_d3d_mtx;

	LPDIRECT3D9 m_pd3d; // direct3d object
	D3DPRESENT_PARAMETERS m_d3dppfull, m_d3dppwin; // creation parameter of direct3d (one for window mode, another for fullscreen)
	LPDIRECT3DDEVICE9 m_pd3dev; // direct3d device
	LPDIRECT3DSURFACE9 m_pgrabsurf;

	ID3DXLine * m_pline;// line object of direct3dx
	c_d3d_dynamic_text m_d3d_txt; // text object of direct3d

	D3DVIEWPORT9 m_ViewPort;
	D3DDISPLAYMODE m_d3ddm;
	c_d3d_camview m_maincam;

	bool init_d3d();	// initialize all direct3d objects
	virtual bool alloc_d3dres(); // helper for init_d3d
	bool init_viewport(Mat & img);
	virtual void release_d3dres(); // release all direct3d objects
	void change_dispmode(); // change furll or window mode.
	bool reset_d3dev();

	/////////////////////////////////// for grabbing rendered surface
	int m_num_grab_frms, m_num_grabbed_frms;
	wchar_t * m_grab_name;

	void grab();

	// helper functions
	void blt_offsrf(Mat & img);

	union{
		char m_buf[SYS_BUF_SIZE];
		wchar_t m_wbuf[SYS_BUF_SIZE/2];
	};

#define SIZE_WNDTBL 16 // sizeof window table. the number limits the number of windows user can make.
	struct s_wnd_entry{
		f_ds_window * pwin;
		HWND hwnd;
		s_wnd_entry():pwin(NULL), hwnd(NULL){};
	};

	static s_wnd_entry m_wndtbl[SIZE_WNDTBL];
	bool add_wnd(f_ds_window * pwin, HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(!m_wndtbl[iw].hwnd && !m_wndtbl[iw].pwin){
				m_wndtbl[iw].hwnd = hwnd;
				m_wndtbl[iw].pwin = pwin;
				return true;
			}
		}
		return false;
	}
	static void rm_wnd(f_ds_window * pwin, HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(m_wndtbl[iw].hwnd == hwnd && m_wndtbl[iw].pwin == pwin){
				m_wndtbl[iw].hwnd = NULL;
				m_wndtbl[iw].pwin = NULL;
			}
		}
	}

	static f_ds_window * get_window(HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(m_wndtbl[iw].hwnd == hwnd){
				return m_wndtbl[iw].pwin;
			}
		}
		return NULL;
	}

public:
	f_ds_window(const char * name);
	~f_ds_window();

	virtual bool is_active();

	virtual bool check(){
		return dynamic_cast<ch_image*>(m_chin[0]) != NULL; 
	};

	virtual bool run(long long start_time, long long end_time)
	{
		return f_base::run(start_time, end_time);
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();

	// Message Processor
	static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg,
		WPARAM wParam, LPARAM lParam);

	virtual void handle_lbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_lbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_lbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousewheel(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_syskeydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keyup(WPARAM wParam, LPARAM lParam){};

	virtual bool init_run(){
		pthread_mutex_init(&m_d3d_mtx, NULL); 
		if(!create_wnd()){
			cerr << "failed to create window." << endl;
			return false;
		}

		if(!init_d3d()){
			cerr << "failed to initialize direct3d." << endl;
			return false;
		}

		return (m_hwnd != NULL) && (m_pd3d != NULL) && (m_pd3dev != NULL);
	}

	virtual void destroy_run(){
		if(m_hwnd == NULL)
			return;

		pthread_mutex_destroy(&m_d3d_mtx);
		delete[] m_name_t;
		m_name_t = NULL;
		release_d3dres();

		if(m_pd3dev != NULL){
			m_pd3dev->Release();
			m_pd3dev = NULL;
		}

		if(m_pd3d != NULL){
			m_pd3d->Release();
			m_pd3d = NULL;
		}

		rm_wnd(this, m_hwnd);

		DestroyWindow(m_hwnd);
		UnregisterClass(m_clsname_t, m_hinst);
		m_hwnd = NULL;
	}
};


class f_ptz_window: public f_ds_window
{
protected:
	/////////////////////////////////////// ptz control value
	bool m_baiscap;
	char m_logfname[SYS_BUF_SIZE];
	c_ship * m_ptgt_ship;
	c_ship * m_ptgt_prev;

	unsigned short m_pan;
	vector<c_ship*> m_candidate_ship;
	long long m_capture_interval;
	long long m_capture_time;
	long long m_rotation_wait_time;
	long long m_shutter_time;
	double m_dir_tgt;

	bool select_capture_target();
	bool capture_target(unsigned short p, Mat & img);

	double m_dlim;
	int m_v, m_vlim;
	vector<int> m_jpg_param;

	void update_ais(long long cur_time);

	/////////////////////////////////////// own ship status
	int m_own_mmsi;
	short m_tz;		// time zone
	short m_h, m_m;	// hour, minute
	float m_s;		// second
	double m_vel, m_crs, m_crs_var; // speed, course, course variation
	Point3d m_Vwrld;
	Point3d m_Vecef;

	/////////////////////////////////////// geometric members
	Mat m_Rwrld; // world rotation for ECEF cordinate
	s_rotpar m_rot_own; // own ship rotation parameters
	Point3d m_pos_own;	// own ship translation parameters
	Mat m_Rown; // 3x3 rotation matrix (ecef to world cordinate)
	Mat m_Rcam; // 3x3 rotation matrix (world to camera cordinate)
	Mat m_Tcam; // camera translation vector

	s_bihpos m_Xbih;  // bih cordinate position of own ship
	Point3d m_Xecef; // ecef cordinate position of own ship
	
	void update_pvt(float dt);

	///////////////////////////////////// system member
	union{
		char m_buf[SYS_BUF_SIZE];
		wchar_t m_wbuf[SYS_BUF_SIZE/2];
	};

	long long m_prev_time;
	double m_avg_cycle_time;

	//////////////////////////////////// windows oobjs
	virtual bool alloc_d3dres(); // helper for init_d3d
	virtual void release_d3dres(); // release all direct3d objects
	void render(Mat & img);

	int m_grid_width;
	double m_grid3d_width;
	void render_grid();

	/////////////////////////////////// for child view
	bool m_bdisp;
	c_d3d_camview m_maincam;

	Size m_sz_map;
	c_d3d_childview m_map;

	/////////////////////////////////// for 2d ship marker
	c_d3d_ship2d m_d3d_ship2d;

	//////////////////////////////////// mouse pointer
	POINT m_mouse;

public:
	f_ptz_window(const char * name);
	virtual ~f_ptz_window();

	virtual bool check();

	virtual bool run(long long start_time, long long end_time)
	{
		c_ship::destroy();
		c_ship::init();

		m_prev_time = start_time;
		return f_base::run(start_time, end_time);
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();
};


class f_sys_window: public f_ds_window
{
protected:
	/////////////////////////////////////// aws system members
	void update_ais(long long cur_time);

	/////////////////////////////////////// own ship status
	int m_own_mmsi;
	short m_tz;		// time zone
	short m_h, m_m;	// hour, minute
	float m_s;		// second
	double m_vel, m_crs, m_crs_var; // speed, course, course variation
	Point3d m_Vwrld;
	Point3d m_Vecef;

	/////////////////////////////////////// geometric members
	Mat m_Rwrld; // world rotation for ECEF cordinate
	s_rotpar m_rot_own; // own ship rotation parameters
	Point3d m_pos_own;	// own ship translation parameters
	Mat m_Rown; // 3x3 rotation matrix (ecef to world cordinate)
	Mat m_Rcam; // 3x3 rotation matrix (world to camera cordinate)
	Mat m_Tcam; // camera translation vector

	s_bihpos m_Xbih;  // bih cordinate position of own ship
	Point3d m_Xecef; // ecef cordinate position of own ship
	
	void update_pvt(float dt);

	///////////////////////////////////// system member
	union{
		char m_buf[SYS_BUF_SIZE];
		wchar_t m_wbuf[SYS_BUF_SIZE/2];
	};

	long long m_prev_time;
	double m_avg_cycle_time;

	/////////////////////////////////// direct3d objs
	virtual bool alloc_d3dres(); // helper for init_d3d
	virtual void release_d3dres(); // release all direct3d objects

	void render();

	int m_grid_width;
	double m_grid3d_width;
	void render_grid();

	/////////////////////////////////// for child view
	bool m_bdisp;
	c_d3d_camview m_maincam;

	Size m_sz_subcam;
	c_d3d_camview m_subcam;

	Size m_sz_map;
	c_d3d_childview m_map;

	/////////////////////////////////// for 2d ship marker
	c_d3d_ship2d m_d3d_ship2d;

	//////////////////////////////////// for tracker
	bool m_btrck;
	SIZE m_trck_size;
	Rect m_trck_rc;
	void render_trck_rc(LPDIRECT3DDEVICE9 pd3dev);

	//////////////////////////////////// mouse pointer
	POINT m_mouse;

public:
	f_sys_window(const char * name);
	virtual ~f_sys_window();

	virtual bool check();

	virtual bool run(long long start_time, long long end_time)
	{
		c_ship::destroy();
		c_ship::init();
		m_prev_time = start_time;
		return f_base::run(start_time, end_time);
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();

	//////////////////////////////////// direct 3D version code
	virtual void handle_lbuttondown(WPARAM wParam, LPARAM lParam);
	virtual void handle_lbuttonup(WPARAM wParam, LPARAM lParam)
	{};
	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam);
};

class ch_nmea;

#define ERR_SPROT_WINDOW_UNKNOWN 1

class f_sprot_window : public f_ds_window
{
private:
	char m_ftrail_log[128];
	ofstream m_trail_log;

	long long m_tint_ttm2ais;
	long long m_tint_trail;
	long long m_tprev_ttm2ais;
	long long m_tprev_trail;
	int m_bm_ver;
	int m_max_trail;
	double m_range;
	double m_circle;
	unsigned int m_bmch;
	c_d3d_ship2d m_d3d_ship2d;
	
	virtual bool alloc_d3dres(); // helper for init_d3d
	virtual void release_d3dres(); // release all direct3d objects
	ch_nmea * m_bmout;
	char m_toker[3];
	char m_nmea[85];
	int m_seq_id;
	virtual bool send_bm(unsigned int mmsi_dst, int & seq_id, int id, int chan, 
	const unsigned char *  buf, int bits);

	D3DXVECTOR2 m_pt_circle[128];
	void render_circle(int cx, int cy, double rat)
	{
		double c, s;
		m_pline->Begin();
		for(double r = m_circle; r < m_range; r+=m_circle){
			for(int i = 0; i < 127; i++){
				double th = i * (1./127.) * 2 * PI;
				c = cos(th);
				s = sin(th);
				m_pt_circle[i].x = (FLOAT) (c * r * rat + cx);
				m_pt_circle[i].y = (FLOAT) (s * r * rat + cy);
			}
			m_pt_circle[127] = m_pt_circle[0];
			m_pline->Draw(m_pt_circle, 128, D3DCOLOR_RGBA(128, 255, 255, 255));
		}
		m_pline->End();
	}

public:
	f_sprot_window(const char * name);
	virtual ~f_sprot_window();

	virtual const char * get_err_msg(int code);
	virtual bool check(){
		return true;
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif