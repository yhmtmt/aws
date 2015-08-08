#ifndef _F_AVT_MONO_H_
#define _F_AVT_MONO_H_
// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_avt_mono.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_avt_mono.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_avt_mono.  If not, see <http://www.gnu.org/licenses/>. 


#include "f_avt_cam.h"

class f_avt_mono: public f_avt_cam
{
protected:
	s_cam_params m_cam;
	virtual bool init_run();
	virtual void destroy_run();
public:
	static bool init_interface();
	static void destroy_interface();
	f_avt_mono(const char * name);
	virtual ~f_avt_mono();

	virtual bool proc();

	void set_new_frm(tPvFrame * pfrm);
};
#endif