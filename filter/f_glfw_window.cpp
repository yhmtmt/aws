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

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glu.h>

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

	m_pwin = glfwCreateWindow(m_sz_win.width, m_sz_win.height, "Hello World", NULL, NULL);
	if (!m_pwin)
	{
		glfwTerminate();
		return false;
	}
	glfwSetErrorCallback(err_cb);

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

	//glfwMakeContextCurrent(m_pwin);

	// rendering codes >>>>>

	// <<<<< rendering codes

	glfwSwapBuffers(m_pwin);

	glfwPollEvents();

	return true;
}


//////////////////////////////////////////////////////// f_glfw_imview
void reshape(int width, int height)
{

	static GLfloat lightPosition[4] = { 0.25f, 1.0f, 0.25f, 0.0f };
	static GLfloat lightDiffuse[3] = { 1.0f, 1.0f, 1.0f };
	static GLfloat lightAmbient[3] = { 0.25f, 0.25f, 0.25f };
	static GLfloat lightSpecular[3] = { 1.0f, 1.0f, 1.0f };



	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glShadeModel(GL_SMOOTH);
	glViewport(0, 0, width, height);



	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0.5, 1.5, 2.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

}


static GLfloat vertices[][3] =
{
	{ -0.5f, -0.5f, 0.5f },
	{ 0.5f, -0.5f, 0.5f },
	{ 0.5f, 0.5f, 0.5f },
	{ -0.5f, 0.5f, 0.5f },
	{ 0.5f, -0.5f, -0.5f },
	{ -0.5f, -0.5f, -0.5f },
	{ -0.5f, 0.5f, -0.5f },
	{ 0.5f, 0.5f, -0.5f }
};

static GLfloat normals[][3] =
{
	{ 0.0f, 0.0f, 1.0f },
	{ 0.0f, 0.0f, -1.0f },
	{ 1.0f, 0.0f, 0.0f },
	{ -1.0f, 0.0f, 0.0f },
	{ 0.0f, 1.0f, 0.0f },
	{ 0.0f, -1.0f, 0.0f }
};


void display(void)
{

	static GLfloat diffuse[3] = { 1.0f, 0.0f, 0.0f };
	static GLfloat ambient[3] = { 0.25f, 0.25f, 0.25f };
	static GLfloat specular[3] = { 1.0f, 1.0f, 1.0f };
	static GLfloat shininess[1] = { 32.0f };

	static float angle = 0.0f;

	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess);

	glEnable(GL_DEPTH_TEST);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	angle += 2.5f;
	if (angle > 360.0f) {
		angle -= 360.0f;
	}

	glPushMatrix();
	glRotatef(angle, 0.0f, 1.0f, 0.0f);

	// ‘O
	glBegin(GL_QUADS);
		glNormal3fv(normals[0]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[2]);
		glVertex3fv(vertices[3]);
	glEnd();

	// Œã
	glBegin(GL_QUADS);
		glNormal3fv(normals[1]);
		glVertex3fv(vertices[4]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[6]);
		glVertex3fv(vertices[7]);
	glEnd();

	// ‰E
	glBegin(GL_QUADS);
		glNormal3fv(normals[2]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[4]);
		glVertex3fv(vertices[7]);
		glVertex3fv(vertices[2]);
	glEnd();

	// ¶
	glBegin(GL_QUADS);
		glNormal3fv(normals[3]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[3]);
		glVertex3fv(vertices[6]);
	glEnd();

	// ã
	glBegin(GL_QUADS);
		glNormal3fv(normals[4]);
		glVertex3fv(vertices[3]);
		glVertex3fv(vertices[2]);
		glVertex3fv(vertices[7]);
		glVertex3fv(vertices[6]);
	glEnd();

	// ‰º
	glBegin(GL_QUADS);
		glNormal3fv(normals[5]);
		glVertex3fv(vertices[1]);
		glVertex3fv(vertices[0]);
		glVertex3fv(vertices[5]);
		glVertex3fv(vertices[4]);
	glEnd();

	glPopMatrix();
	//sleep(10);

}

bool f_glfw_imview::proc()
{
	if(glfwWindowShouldClose(m_pwin))
		return false;

	long long timg;
	Mat img = m_pin->get_img(timg);
	if(img.empty())
		return true;
	if(m_timg == timg)
		return true;

	m_timg = timg;

	glRasterPos2i(-1, -1);
	glDrawPixels(img.cols, img.rows, GL_RGB, GL_UNSIGNED_BYTE, img.data);

	/*
	int width, height;
	glfwGetFramebufferSize(m_pwin, &width, &height);
	reshape(width, height);
	//idle();
	//sleep(10);
	display();
	*/
	glfwSwapBuffers(m_pwin);
	glfwPollEvents();

	return true;
}


bool f_glfw_imview::init_run()
{
	if(m_chin.size() == 0)
		return false;

	m_pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(m_pin == NULL)
		return false;

	if(!f_glfw_window::init_run())
		return false;

	return true;
}

#endif
