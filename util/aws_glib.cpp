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

	glBegin(GL_LINES);
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
	float r, float g, float b, float alpha,
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
	float r, float g, float b, float alpha, float size)
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
	float r, float g, float b, float alpha)
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
	float x3, float y3, float r, float g, float b, float alpha, float size)
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
	float x3, float y3, float r, float g, float b, float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_TRIANGLES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glVertex2f(x3, y3);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	float r, float g, float b, float alpha, float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i< num_pts; i++)
		glVertex2f(pts[i].x, pts[i].y);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	float r, float g, float b, float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_POLYGON);
	for (int i = 0; i < num_pts; i++)
		glVertex2f(pts[i].x, pts[i].y);
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	Point2f & offset,
	float r, float g, float b, float alpha, float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i< num_pts; i++)
		glVertex2f((float)(pts[i].x + offset.x), (float)(pts[i].y + offset.y));
	glEnd();
}

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	Point2f & offset,
	float r, float g, float b, float alpha)
{
	glColor4f(r, g, b, alpha);
	glBegin(GL_POLYGON);
	for (int i = 0; i < num_pts; i++)
		glVertex2f((float)(pts[i].x + offset.x), (float)(pts[i].y + offset.y));
	glEnd();
}


void drawGlLine2Df(float x1, float y1, float x2, float y2,
	float r, float g, float b, float alpha, float size)
{
	glColor4f(r, g, b, alpha);
	glLineWidth(size);
	glBegin(GL_LINES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glEnd();
}
