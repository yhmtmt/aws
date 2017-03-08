// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// aws1_glib.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws1_glib.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws1_glib.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <map>
using namespace std;


#ifdef _WIN32
#include <Windows.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_vobj.h"
#include "../util/aws_vlib.h"

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include "aws_glib.h"


//////////////////////////////////////////////////////////////////////////////////////////// helper functions
void drawCvPoints(const Size & vp, vector<Point2f> & pts,
	const float r, const float g, const float b, const float alpha,
	const float l /*point size*/)
{
	Point2f pt;
	double fac_x = 2.0 / (double)vp.width, fac_y = 2.0 / (double)vp.height;

	// drawing Points
	glPointSize(l);
	glBegin(GL_POINTS);
	{
		glColor4f(r, g, b, alpha);
		for (int ipt = 0; ipt < pts.size(); ipt++){
			cnvCvPoint2GlPoint(fac_x, fac_y, pts[ipt], pt);
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();
}

void drawCvPoints(const float fac_x, const float fac_y, const float xorg, const float yorg, const float w, const float h,
	vector<Point2f> & pts,
	const float r, const float g, const float b, const float alpha,
	const float l /*point size*/)
{

	Point2f pt;
	// drawing Points
	glPointSize(l);
	glBegin(GL_POINTS);
	{
		glColor4f(r, g, b, alpha);
		for (int ipt = 0; ipt < pts.size(); ipt++){
			cnvCvPoint2GlPoint(fac_x, fac_y, xorg, yorg, w, h, pts[ipt], pt);
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();

}


void drawCvChessboard(const Size & vp, vector<Point2f> & pts,
	const float r, const float g, const float b, const float alpha,
	const float l /* point size */, const float w /* line width */)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(w);

	double fac_x = 2.0 / (double)vp.width, fac_y = 2.0 / (double)vp.height;

	glBegin(GL_LINE_LOOP);
	{
		Point2f pt;
		cnvCvPoint2GlPoint(fac_x, fac_y, pts[0], pt);
		glVertex2f(pt.x, pt.y);
		for (int ipt = 1; ipt < pts.size(); ipt++){
			cnvCvPoint2GlPoint(fac_x, fac_y, pts[ipt], pt);
			glVertex2f(pt.x, pt.y);
		}
	}
	glEnd();
}

void drawCvPointDensity(Mat hist, const int hist_max, const Size grid,
	const float r, const float g, const float b, const float alpha,
	const float w /* line width of the grid */)
{
	float fac = (float)(1.0 / (float)hist_max);
	float wgrid = (float)((float)2. / (float)grid.width);
	float hgrid = (float)((float)2. / (float)grid.height);

	glLineWidth(w);
	glBegin(GL_QUADS);
	{
		for (int y = 0; y < grid.height; y++){
			for (int x = 0; x < grid.width; x++){
				glColor4f(r * fac, g * fac, b * fac, alpha);
				float u, v;
				u = (float)(x * wgrid - 1.);
				v = -(float)(y * hgrid - 1.);
				glVertex2f(u, v);
				u = (float)((x + 1) * wgrid - 1.);
				glVertex2f(u, v);
				v = -(float)((y + 1) * hgrid - 1.);
				glVertex2f(u, v);
				u = (float)(x * wgrid - 1.);
				glVertex2f(u, v);
			}
		}
	}
	glEnd();
}

void drawGlText(float x, float y, const char * str,
	const float r, const float g, const float b, const float alpha,
	void* font)
{
	glColor4f(r, g, b, alpha);
	int l = (int)strlen(str);
	glRasterPos2f(x, y);
	for (int i = 0; i < l; i++){
		glutBitmapCharacter(font, str[i]);
	}
}

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha, const float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	glVertex2f(x1, y1);
	glVertex2f(x2, y1);
	glVertex2f(x2, y2);
	glVertex2f(x1, y2);
	glEnd();
}

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_QUADS);
	glVertex2f(x1, y1);
	glVertex2f(x2, y1);
	glVertex2f(x2, y2);
	glVertex2f(x1, y2);
	glEnd();
}

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, const float r, const float g, const float b, const float alpha, const float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glVertex2f(x3, y3);
	glEnd();
}

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, const float r, const float g, const float b, const float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_TRIANGLES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glVertex2f(x3, y3);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const float r, const float g, const float b, const float alpha, const float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i< num_pts; i++)
		glVertex2f(pts[i].x, pts[i].y);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const float r, const float g, const float b, const float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_POLYGON);
	for (int i = 0; i < num_pts; i++)
		glVertex2f(pts[i].x, pts[i].y);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const Point2f & offset,
	const float r, const float g, const float b, const float alpha, const float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i< num_pts; i++)
		glVertex2f((float)(pts[i].x + offset.x), (float)(pts[i].y + offset.y));
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const Point2f & offset,
	const float r, const float g, const float b, const float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_POLYGON);
	for (int i = 0; i < num_pts; i++)
		glVertex2f((float)(pts[i].x + offset.x), (float)(pts[i].y + offset.y));
	glEnd();
}


void drawGlLine2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha, const float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glEnd();
}


void cnvCvRTToGlRT(const Mat & r, const Mat & t, GLdouble * m)
{
	// m[0] m[4] m[8]  m[12]
	// m[1] m[5] m[9]  m[13]
	// m[2] m[6] m[10] m[14]
	// m[3] m[7] m[11] m[15]
	m[3] = m[7] = m[11] = 0.;
	m[15] = 1.0;
	{// load rotation matrix
		Mat R;
		Rodrigues(r, R);

		if (R.type() == CV_64FC1){ // doulbe precision
			double * p = R.ptr<double>();
			m[0] = p[0];
			m[4] = p[1];
			m[8] = p[2];
			m[1] = p[3];
			m[5] = p[4];
			m[9] = p[5];
			m[2] = p[6];
			m[6] = p[7];
			m[10] = p[8];
		}
		else{ // single precision
			float * p = R.ptr<float>();
			m[0] = p[0];
			m[4] = p[1];
			m[8] = p[2];
			m[1] = p[3];
			m[5] = p[4];
			m[9] = p[5];
			m[2] = p[6];
			m[6] = p[7];
			m[10] = p[8];
		}
	}

	// load translation
	if(t.type() == CV_64FC1){
		const double * p = t.ptr<double>();
		m[12] = p[0];
		m[13] = p[1];
		m[14] = p[2];
	}
	else{
		const float * p = t.ptr<float>();
		m[12] = p[0];
		m[13] = p[1];
		m[14] = p[2];
	}
}

void cnvCvRTToGlRT(const Mat & r, const Mat & t, GLfloat * m)
{
	// m[0] m[4] m[8]  m[12]
	// m[1] m[5] m[9]  m[13]
	// m[2] m[6] m[10] m[14]
	// m[3] m[7] m[11] m[15]
	m[3] = m[7] = m[11] = 0.;
	m[15] = 1.0;
	{// load rotation matrix
		Mat R;
		Rodrigues(r, R);

		if (R.type() == CV_64FC1){ // doulbe precision
			double * p = R.ptr<double>();
			m[0] = (GLfloat)p[0];
			m[4] = (GLfloat)p[1];
			m[8] = (GLfloat)p[2];
			m[1] = (GLfloat)p[3];
			m[5] = (GLfloat)p[4];
			m[9] = (GLfloat)p[5];
			m[2] = (GLfloat)p[6];
			m[6] = (GLfloat)p[7];
			m[10] = (GLfloat)p[8];
		}
		else{ // single precision
			float * p = R.ptr<float>();
			m[0] = p[0];
			m[4] = p[1];
			m[8] = p[2];
			m[1] = p[3];
			m[5] = p[4];
			m[9] = p[5];
			m[2] = p[6];
			m[6] = p[7];
			m[10] = p[8];
		}
	}

	// load translation
	if (t.type() == CV_64FC1){
		const double * p = t.ptr<double>();
		m[12] = (GLfloat)p[0];
		m[13] = (GLfloat)p[1];
		m[14] = (GLfloat)p[2];
	}
	else{
		const float * p = t.ptr<float>();
		m[12] = p[0];
		m[13] = p[1];
		m[14] = p[2];
	}

}


void printGlMatrix(const float * m)
{
	cout << m[0] << " " << m[4] << " " << m[8] << " " << m[12] << endl;
	cout << m[1] << " " << m[5] << " " << m[9] << " " << m[13] << endl;
	cout << m[2] << " " << m[6] << " " << m[10] << " " << m[14] << endl;
	cout << m[3] << " " << m[7] << " " << m[11] << " " << m[15] << endl;
}

void printGlMatrix(const double * m)
{
	cout << m[0] << " " << m[4] << " " << m[8] << " " << m[12] << endl;
	cout << m[1] << " " << m[5] << " " << m[9] << " " << m[13] << endl;
	cout << m[2] << " " << m[6] << " " << m[10] << " " << m[14] << endl;
	cout << m[3] << " " << m[7] << " " << m[11] << " " << m[15] << endl;
}