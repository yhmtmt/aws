#ifndef _F_PTZ_WINDOW_H_
#define _F_PTZ_WINDOW_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ptz_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ptz_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ptz_window.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/aws_coord.h"

#include "../util/c_ship.h"
//#include "../util/aws_nmea.h"

#include "../channel/ch_image.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_scalar.h"
#include "../channel/ch_campar.h"
#include "../channel/ch_navdat.h"
#include "../channel/ch_nmea.h"

#include "f_ds_window.h"

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
		m_prev_time = start_time;
		return f_base::run(start_time, end_time);
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();
};

#endif