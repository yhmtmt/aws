// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_sprot_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sprot_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sprot_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_SPROT_WINDOW_
#define _F_SPROT_WINDOW_
#ifdef _WIN32

#include "f_base.h"
#include "f_ds_window.h"

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
#endif
