// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_dev.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_dev.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_dev.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _C_AWS1_UI_DEV_H_
#define _C_AWS1_UI_DEV_H_

class c_aws1_ui_dev: public c_aws1_ui_core
{
protected:
public:
	c_aws1_ui_dev(f_aws1_ui * _pui):c_aws1_ui_core(_pui)
	{
	}

	virtual void js(const s_jc_u3613m  & js);
	virtual void draw(float xscale, float yscale);
	virtual void key(int key, int scancode, int action, int mods);
};

#endif
