#ifndef _F_SYS_WINDOW_H_
#define _F_SYS_WINDOW_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_sys_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sys_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sys_window.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_ds_window.h"

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

#endif