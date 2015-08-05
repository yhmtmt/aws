// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_glfw_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_glfw_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_glfw_window.  If not, see <http://www.gnu.org/licenses/>. 
#include "stdafx.h"
#ifdef GLFW_WINDOW

#include <iostream>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GLFW/glfw3.h>

#include "f_glfw_window.h"

f_glfw_window::f_glfw_window(const char * name):f_base(name), m_pwin(NULL), m_sz_win(640, 480)
{
	register_fpar("width", &m_sz_win.width, "Width of the window.");
	register_fpar("height", &m_sz_win.height, "Height of the window.");
}

f_glfw_window::~f_glfw_window()
{
}

bool f_glfw_window::init_run()
{
	if(!glfwInit())
		return false;

	m_pwin = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!m_pwin)
    {
        glfwTerminate();
        return false;
    }

	 glfwMakeContextCurrent(m_pwin);
	 return true;
}

void f_glfw_window::destroy_run()
{
	glfwTerminate();
	m_pwin = NULL;
}

bool f_glfw_window::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	glfwMakeContextCurrent(m_pwin);

	// rendering codes >>>>>

	// <<<<< rendering codes

	glfwSwapBuffers(m_pwin);

	glfwPollEvents();

	return true;
}


//////////////////////////////////////////////////////// f_glfw_imview
bool f_glfw_imview::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	long long timg;
	Mat img = m_pin->get_img(timg);
	if(m_timg == timg)
		return true;
	m_timg = timg;

	glfwMakeContextCurrent(m_pwin);

	// rendering codes >>>>>
	glRasterPos2f(-1, 1);
	glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);
	// <<<<< rendering codes

	glfwSwapBuffers(m_pwin);

	glfwPollEvents();
}

#endif