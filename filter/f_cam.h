#ifndef _F_CAM_H_
#define _F_CAM_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_cam.h.  If not, see <http://www.gnu.org/licenses/>. 
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_base.h"

enum e_frm_state{
	EFS_NONE, EFS_NEW, EFS_SHOW,
};

extern bool g_kill;

class f_cam: public f_base
{
protected:
	Mat m_frm; // frame to be loaded for each grab

	bool m_bstream;

	// camera parameters
	bool m_intpar;
	Mat m_cam_mat;
	Mat m_dist_coeff;
	Mat m_rvec_base; // rotation matrix from wold to camera
	Mat m_tvec_base; // translation vector from wold center to the camera center.

	long long m_time_shot;
	ch_image * m_pout; // filter output pin
	ch_campar * m_pcamparout;
public:
	f_cam(const char * name);
	virtual ~f_cam();

	// as the filter ///////////////////////
	virtual bool check()
	{
		return m_chout[0] != NULL;
	}

	virtual bool init_run(){
			m_time_shot = -1;
		return true;
	}

	virtual bool proc();
	/////////////////////////// as the filter

	virtual bool grab(Mat & img) = 0;

	virtual bool write_intpar(const char * fname);
	virtual bool read_intpar(const char * fname);

	virtual bool cmd_proc(s_cmd & cmd);
};


#endif