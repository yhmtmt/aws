
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_ship.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_ship.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_ship.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "aws_coord.h"

#ifdef _WIN32
#include "c_d3d_2dobj.h"
#endif

class c_vdm_msg1;
class c_vdm_msg5;
class c_vdm_msg8;
class c_vdm_msg18;
class c_vdm_msg19;
class c_vdm_msg24;
class c_ttm;
class c_rmc;

enum e_ship_data_type{
	ESDT_VDM, ESDT_ARPA, ESDT_BM, ESDT_CAM, ESDT_UNDEF
};


class c_ship{
private:
	e_ship_data_type m_data_type;
	unsigned int m_mmsi;
	unsigned int m_ms_mmsi; // mother ship mmsi

	char m_name[21];
	char m_callsign[8];
	bool m_ais_class_A;

	unsigned char m_type;
	s_bihpos m_Xbih;
	vector<s_bihpos> m_trail;
	int m_trail_head, m_trail_tail;

#ifdef _WIN32
	DWORD m_clr;
#endif
	double m_bear;
	double m_dist, m_inv_dist;

	double m_cog, m_vel, m_hdg, m_turn;

	unsigned int m_to_bow, m_to_stern, m_to_port, m_to_starboard;

	bool m_update;
	long long m_tabs; // position update time in 100 nsec
	Point3d m_Vwrld;
	Point3d m_Xwrld; // world cordinate (own ship centric)
	Point2d m_Vview;
	Point2d m_Xview;

	c_ship * m_pnext;
	long long m_last_bmt;

public:
	c_ship():m_data_type(ESDT_UNDEF), m_mmsi(UINT_MAX), m_pnext(NULL), m_tabs(0),
			m_to_bow(0), m_to_stern(0), m_to_port(0), m_to_starboard(0), m_last_bmt(0),
			m_update(false), m_trail_head(0), m_trail_tail(0)
#ifdef _WIN32
			// for Direct 3D
			,m_pvb(NULL), m_pidb(NULL)
#endif
	{
		strcpy(m_name, "null");
		m_type = -1;
	}

	~c_ship(){
		delete m_pnext;
	}

	void init(unsigned int mmsi)
	{
		m_mmsi = mmsi;
	}

	void set_ttmid(unsigned int ttmid){
		m_mmsi = ttmid | 0x80000000;
	}

	unsigned int get_ttmid(){
		if(ESDT_ARPA)
			return m_mmsi ^ 0x80000000;
		return 0;
	}

	void set_bmid(unsigned int bmid){
		m_mmsi = bmid | 0xC0000000;
	}

	unsigned int get_bmid(){
		if(m_data_type == ESDT_BM)
			return m_mmsi ^ 0xC0000000;
		return 0;
	}

	void set_name(const char * name)
	{
		// copying ship name (ais record has no termination character)
		for(int i = 0; i < 21; i++){
			m_name[i] = name[i];
		}
	}

	void set_callsign(const char * cs)
	{

		for(int i = 0; i < 8; i++){
			m_callsign[i] = cs[i];
		}
	}

	void set_dimension(unsigned int to_bow, unsigned int to_stern, 
		unsigned int to_port, unsigned int to_starboard){
			m_to_bow = to_bow;
		m_to_stern = to_stern;
		m_to_port = to_port;
		m_to_starboard = to_starboard;
	}

#ifdef _WIN32
	void set_clr(DWORD clr){
		m_clr = clr;
	}
#endif

	void set_time(long long cur_time){
		m_tabs = cur_time;
	}

	void set_bm_time(long long cur_time){
		m_last_bmt = cur_time;
	}

	void set_pos(double lat/*rad*/, double lon/*rad*/, double alt, 
		double vel/*knot*/, double crs /*rad*/, double hdg/*rad*/, double turn)
	{
		m_cog = crs;
		m_vel = vel;
		m_hdg = hdg;
		m_turn = turn;
		m_Vwrld.x = vel * sin(crs) * KNOT; 
		m_Vwrld.y = vel * cos(crs) * KNOT;
		m_Vwrld.z = 0;

		m_Xbih.lat = lat;
		m_Xbih.lon = lon;
		m_Xbih.alt = alt;
		m_update = true;
	}

	bool is_timeout(long long curtime){
		return curtime - m_tabs > m_time_limit;
	}

	double get_cog() /* in rad relative to true north*/{
		return m_cog;
	}
		
	double get_sog() /* in knot */{
		return m_vel;
	}	

	double get_bear() /* in rad */{
		return m_bear;
	}
	
	const s_bihpos & get_bihpos() const{
		return m_Xbih;
	}

	double get_lon() /* longitude in rad */{
		return m_Xbih.lon;
	}

	double get_lat() /* latitude in rad */{
		return m_Xbih.lat;
	}

	bool calc_Xwrld_own(Point3d & org, Mat & rot);

	bool calc_Xwrld(Point3d & Xorg, Mat & Rwrld, long long tabs);

	void save_trail(int max_trail = 100){
		if(m_update){
			if(m_trail.size() != max_trail){
				m_trail.resize(max_trail);
				m_trail_head = 0; 
				m_trail_tail = 0;
			}

			m_trail[m_trail_tail] = m_Xbih;
			
			m_trail_tail = (m_trail_tail + 1) % max_trail;
			if(m_trail_tail == m_trail_head)
				m_trail_head++;

			m_update = false;
		}
	}

	void log_trail(ofstream & file){
		if(m_update){
			file << m_tabs << " ";
			file << m_mmsi << " ";
			file << m_Xbih.lat << " ";
			file << m_Xbih.lon << " ";
			file << m_Xbih.alt << " ";
			file << m_Xwrld.x << " ";
			file << m_Xwrld.y << " ";
			file << m_Xwrld.z << " ";
			file << m_Vwrld.x << " ";
			file << m_Vwrld.y << " ";
			file << m_Vwrld.z << " ";
			file << endl;
		}
	}

	bool is_insight(Mat & vec_cam, double clip_cos)
	{
		double * ptr = vec_cam.ptr<double>(0);
		return (clip_cos < (ptr[0] * m_Xwrld.x + ptr[1] * m_Xwrld.y + ptr[2] * m_Xwrld.z) * m_inv_dist);
	}

	bool is_inrange(double clip_dist)
	{
		return clip_dist > m_dist;
	}

	e_ship_data_type get_data_type(){
		return m_data_type;
	}

	long long get_last_bmt(){
		return m_last_bmt;	
	}

	double get_dist()
	{
		return m_dist;
	}

	double get_dir(long long t)
	{
		double dt = (t - m_tabs) * 1e-7;
		Point3d X = m_Xwrld + m_Vwrld * (double) dt * 1e-7;

		return atan2(X.x, X.y);
	}

	unsigned int get_mmsi()
	{	
		return m_mmsi;
	}

	const char * get_name()
	{
		return m_name;
	}

	const char * get_ship_type();


#ifdef _WIN32
	////////////////////////////// for direct 3D //////////////////////////
private:
	static LPDIRECT3DVERTEXBUFFER9 m_pvb_std; // vertex buffer for standard model
	static LPDIRECT3DINDEXBUFFER9 m_pidb_std; // index buffer for standard model
	static D3DMATERIAL9 m_mtrl_std;
	static D3DLIGHT9 m_light_std;
	LPDIRECT3DVERTEXBUFFER9 m_pvb;			  // vertex buffer for org model
	LPDIRECT3DINDEXBUFFER9 m_pidb;			  // index buffer for org model

public:
	static bool load_std_model(LPDIRECT3DDEVICE9 pd3dev);
	static void release_std_model();
	bool load_original_model(LPDIRECT3DDEVICE9 pd3dev);
	void release_original_model();

	// render triangle of the ship on map

	bool render2d(LPDIRECT3DDEVICE9 pd3dev, 
		ID3DXLine * pline, 
		c_d3d_ship2d & ship2d, 
		Point3d & Xorg, Mat & Rwrld, 
		double rat, float cx, float cy, bool trail = true);

	// render box of the ship on camera view
	bool render(LPDIRECT3DDEVICE9 pd3dev, ID3DXLine * pLine,
		c_d3d_dynamic_text & txt, Mat & Mtrn);

	bool set_render_state(LPDIRECT3DDEVICE9 pd3dev);
	bool render3d(LPDIRECT3DDEVICE9 pd3dev);
#endif

	////////////////////////////// static members /////////////////////////
private:
	static pthread_mutex_t m_list_mtx;
	static unsigned int m_mmsi_own;
	static c_ship m_ship_own;

	// ship list
	static list<c_ship*> m_ship_list;
	static void push_ship(c_ship * pship); // push a ship to ship list and htbl if possible.

	static vector<c_ship*> m_ship_list_recent; // ship list for recent position update. 
	static int m_head_slrpv; // head of the recent list
	static int m_max_slrpv; // size of the recent list
	static void push_recent(c_ship * pship);
	static void pop_recent(c_ship * pship);

	static double m_time_limit; // deleting time limit from last update

	// hash table by mmsi number
	static char m_buf[128];
	static c_ship * m_htbl[1024];
	static size_t m_htbl_size;
	static unsigned int get_hash_value(unsigned int mmsi)
	{
		return mmsi % m_htbl_size;
	}

	// helping hash table's linked list
	c_ship * get_htbl(unsigned int mmsi);
	void push_htbl(c_ship * pship);
	static c_ship * pop_htbl(unsigned int mmsi);
	c_ship * pop_htbl_link(unsigned int mmsi);

	// memory pool and its management methods
	static c_ship * m_ship_pool;
	static c_ship * alloc();
	static void free(c_ship * pship);
public:
	// initialize global ship data table
	static void init();
	static void destroy();
	static void delete_timeout_ship(long long curtime);
	static void list_lock();
	static void list_unlock();

	// for own ship
	static void set_mmsi_own(unsigned int mmsi_own);
	static void set_own_rmc(c_rmc * prmc, long long cur_time);
	static c_ship & get_own_ship();

	// get ship obj by mmsi
	static const c_ship * get_ship_by_mmsi(unsigned int mmsi);
	static const c_ship * get_ship_by_ttmid(unsigned int ttmid);
	static const list<c_ship*> & get_ship_list();
	static const c_ship * get_recent(int i);
	static void delete_ship_by_mmsi(unsigned int mmsi);
	static void delete_ship(c_ship * pship);

	// register ship by vdm
	static const c_ship * register_ship_by_vdm1(c_vdm_msg1 * pvdm1, long long cur_time);
	static const c_ship * register_ship_by_vdm5(c_vdm_msg5 * pvdm5);
	static const c_ship * register_ship_by_vdm8(c_vdm_msg8 * pvdm8, long long cur_time, int bm_ver = 1);
	static const c_ship * register_ship_by_vdm18(c_vdm_msg18 * pvdm18, long long cur_time);
	static const c_ship * register_ship_by_vdm19(c_vdm_msg19 * pvdm19, long long cur_time);
	static const c_ship * register_ship_by_vdm24(c_vdm_msg24 * pvdm24);
	
	// register ship by ttm
	static const c_ship * register_ship_by_ttm(c_ttm * pttm, long long cur_time);
};

////////////////////////////////////////////////////////////////////// c_track_obj
class c_track_obj
{
protected:
	char * m_name;
	Rect m_rc; // bounding box
	vector<Mat> m_apyr; // pyramid of appearance img
	Mat m_M; // motion (affine transform)
	bool m_bapyr;
	bool m_blost;
public:
	c_track_obj(): m_name(NULL), m_bapyr(false), m_blost(false)
	{
		m_M = Mat::eye(2, 3, CV_64F);
	}

	~c_track_obj()
	{
		delete [] m_name;
	}

	void set_name(const char * name)
	{
		m_name = new char[strlen(name) + 1];
		strcpy(m_name, name);
	}

	void set_apimg(Mat & img, int num_pyr_levels = 1)
	{
		Mat img_cpy;
		img.copyTo(img_cpy);
		m_apyr.resize(num_pyr_levels);
		// making image pyramid
		buildPyramid(img_cpy, m_apyr, (int) m_apyr.size());
		m_bapyr = true;
	}

	bool is_apyr_set()
	{
		return m_bapyr;
	}

	void set_rc(Rect & rc)
	{
		m_rc = rc;
		m_blost = false;
	}

	void set_lost()
	{
		m_blost = true;
	}

	bool is_lost()
	{
		return m_blost;
	}

	const char * get_name()
	{
		return m_name;
	}

	vector<Mat> & get_apyr()
	{ 
		return m_apyr;
	}

	Rect & get_rc()
	{
		return m_rc;
	}

#ifdef _WIN32
	void render(LPD3DXLINE pline, double rat){

		D3DXVECTOR2 pt[5];	
		{
			double x0 = m_rc.x;
			double x1 = m_rc.x + m_rc.width;
			double y0 = m_rc.y;
			double y1 = m_rc.y + m_rc.height;
			x0 *= rat;
			x1 *= rat;
			y0 *= rat;
			y1 *= rat;

			// calculating affin transformation 
			double * ptr = m_M.ptr<double>(0);
			double t = ptr[2] * rat;
			pt[0].x = (float)(ptr[0] * x0 + ptr[1] * y0 + t);
			pt[1].x = (float)(ptr[0] * x0 + ptr[1] * y1 + t);
			pt[2].x = (float)(ptr[0] * x1 + ptr[1] * y1 + t);
			pt[3].x = (float)(ptr[0] * x1 + ptr[1] * y0 + t);
			pt[4].x = (float)(pt[0].x);
			ptr = m_M.ptr<double>(1);
			t = ptr[2] * rat;
			pt[0].y = (float)(ptr[0] * x0 + ptr[1] * y0 + t);
			pt[1].y = (float)(ptr[0] * x0 + ptr[1] * y1 + t);
			pt[2].y = (float)(ptr[0] * x1 + ptr[1] * y1 + t);
			pt[3].y = (float)(ptr[0] * x1 + ptr[1] * y0 + t);
			pt[4].y = (float)(pt[0].y);
		}

		pline->Begin();
		pline->Draw(pt, 5, D3DXCOLOR(1., 0, 0, 1.));
		pline->End();
	}
#endif
	Mat & get_motion()
	{
		return m_M;
	}

	void set_motion(Mat & M)
	{
		M.copyTo(m_M);
	}
};
