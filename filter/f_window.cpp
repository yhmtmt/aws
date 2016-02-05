#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <WindowsX.h>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_window.h"



///////////////////////////////////////////////////////////////// f_mark_window

f_mark_window::f_mark_window(const char * name):f_window(name), m_scale(1)
{
	setMouseCallback(m_name, on_mouse, this);
}

f_mark_window::~f_mark_window()
{
}

bool f_mark_window::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;
	if(strcmp(args[itok], "save") == 0){
		if(num_args != 4)
			return false;
		write(args[itok+1]);
		return true;
	}else if(strcmp(args[itok], "sc") == 0){
		if(num_args != 4)
			return false;
		m_scale = atoi(args[itok+1]);
		return true;
	}

	return f_window::cmd_proc(cmd);
}

