// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_core.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_core.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_core.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_UI_CORE_H_
#define _F_AWS1_UI_CORE_H_

class f_aws1_ui;

class c_aws1_ui_core
{
protected:
	f_aws1_ui * pui;

	const long long get_cur_time();

	 ch_state * get_ch_state();
	 ch_aws1_ctrl_inst * get_ch_ctrl_inst();
	 ch_aws1_ctrl_stat * get_ch_ctrl_stat();
	 ch_wp * get_ch_wp();
	 ch_ais_obj * get_ch_ais_obj();
	 ch_obj * get_ch_obj();
	 ch_image * get_ch_img();
	 ch_map * get_ch_map();
	 ch_obst * get_ch_obst();
	 ch_aws1_ap_inst * get_ch_ap_inst();

	 s_aws1_ctrl_inst & get_ctrl_inst();
	 const Size & get_window_size();
	 const char * get_path_storage();
	// normal coordinate to pixel coordinate transformation
	void nml2pix(const float xnml, const float ynml, float & xpix, float & ypix);

	// pixel coordinate to normal coordinate transformation
	void pix2nml(const float xpix, const float ypix, float & xnml, float & ynml);

public:
	c_aws1_ui_core(f_aws1_ui * _pui):pui(_pui)
	{
	}

	virtual ~c_aws1_ui_core()
	{
		pui = NULL;
	}

	virtual void js(const s_jc_u3613m  & js)
	{
	}

	virtual void draw()
	{
	};

	virtual void key(int key, int scancode, int action, int mods)
	{
	};
};

#include "c_aws1_ui_normal.h"
#include "c_aws1_ui_map.h"
#include "c_aws1_ui_dev.h"

#endif