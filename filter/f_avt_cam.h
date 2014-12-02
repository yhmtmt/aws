// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_cam.  If not, see <http://www.gnu.org/licenses/>. 

#include "PvApi.h"

class f_avt_cam: public f_base
{
protected:
	vector<bool> m_frm_done;
	ch_image_ref * pout;
	static bool m_bready_api;
	char m_host[1024];

	int m_num_buf;
	tPvAccessFlags m_access;
	int m_size_buf;
	tPvFrame * m_frame;
	unsigned char ** m_img_buf;
	tPvHandle m_hcam;

	int m_cur_frm;
	
	virtual bool init_run();
	virtual void destroy_run();
public:
	static bool init_interface();
	static void destroy_interface();
	f_avt_cam(const char * name);
	virtual ~f_avt_cam();

	virtual const char * get_err_msg(int code);
	virtual bool proc();

	void set_new_frm(tPvFrame * pfrm);
};
