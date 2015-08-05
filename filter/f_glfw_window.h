#ifndef _F_GLFW_WINDOW_H_
#define _F_GLFW_WINDOW_H_
// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_glfw_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include "../channel/ch_image.h"

class f_glfw_window: public f_base
{
protected:
	GLFWwindow * m_pwin;
	Size m_sz_win;
	virtual bool init_run();
	virtual void destroy_run();

public:
	f_glfw_window(const char * name);
	virtual ~f_glfw_window();
	
	virtual bool is_main_thread()
	{
		return true;
	}

	virtual bool proc();

	// glfw callbacks
	static void err_cb(int e, const char * dsc)
	{
		fputs(dsc, stderr);
	}
};


class f_glfw_imview: public f_glfw_window
{
protected:
	ch_image * m_pin;
	long long m_timg;

	virtual bool init_run();

public:
	f_glfw_imview(const char * name): f_glfw_window(name), m_pin(NULL), m_timg(-1)
	{
	}
	virtual ~f_glfw_imview()
	{
	}

	virtual bool proc();
};

#endif
