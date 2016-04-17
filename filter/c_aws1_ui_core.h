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

	 ch_state * get_state();
	 ch_aws1_ctrl * get_ctrl_in();
	 ch_aws1_ctrl * get_ctrl_out();
	 ch_wp * get_wp();
	 ch_obj * get_obj();
	 ch_image * get_img();
	 s_aws1_ctrl_pars & get_acp();
	 const Size & get_window_size();
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

	virtual void draw(float xscale, float yscale)
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