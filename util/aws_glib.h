// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// aws1_glib.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws1_glib.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws1_glib.h.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef AWS_GLIB_H
#define AWS_GLIB_H

// In my OpenGL use, the 2D renderer uses -1 to 1 normalized coordinate, 
// The function convert OpenCV's pixel coordinate into normalized OpenGL coordinate.
inline void cnvCvPoint2GlPoint(const Size & vp, const Point2f & ptcv, Point2f & ptgl)
{
	double fac_x = 2.0 / (double)vp.width, fac_y = 2.0 / (double)vp.height;
	ptgl.x = (float)(ptcv.x * fac_x - 1.0);
	ptgl.y = -(float)(ptcv.y * fac_y - 1.0);
}

inline void cnvCvPoint2GlPoint(const double fac_x, const double fac_y, const Point2f & ptcv, Point2f & ptgl)
{
	ptgl.x = (float)(ptcv.x * fac_x - 1.0);
	ptgl.y = -(float)(ptcv.y * fac_y - 1.0);
}

inline void cnvGlPoint2CvPoint(const float fac_x, const float fac_y, const float xorg, const float yorg,
	const float w, const float h, const Point2f & ptgl, Point2f & ptcv)
{
	ptcv.x = (float)((ptgl.x - xorg) / fac_x);
	ptcv.y = (float)(-(ptgl.y - yorg - h) / fac_y);
}

inline void cnvCvPoint2GlPoint(const float fac_x, const float fac_y, const float xorg, const float yorg,
	const float w, const float h, const Point2f & ptcv, Point2f & ptgl)
{
	ptgl.x = (float)(ptcv.x * fac_x + xorg);
	ptgl.y = (float)(h - ptcv.y * fac_y + yorg);
}


void drawGlText(float x, float y, const char * str,
	float r, float g, float b, float alpha,
	void* font);

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	float r, float g, float b, float alpha, float size);

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	float r, float g, float b, float alpha);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, float r, float g, float b, float alpha, float size);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, float r, float g, float b, float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	float r, float g, float b, float alpha, float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	float r, float g, float b, float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts, Point2f & offset,
	float r, float g, float b, float alpha, float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts, Point2f & offset,
	float r, float g, float b, float alpha);


void drawGlLine2Df(float x1, float y1, float x2, float y2,
	float r, float g, float b, float alpha, float size);

void drawCvPoints(const Size & vp, vector<Point2f> & pts,
	const float r, const float g, const float b, const float alpha,
	const float l /*point size*/);
void drawCvChessboard(const Size & vp, vector<Point2f> & pts,
	const float r, const float g, const float b, const float alpha,
	const float l /* point size */, const float w /* line width */);
void drawCvPointDensity(Mat hist, const int hist_max, const Size grid,
	const float r, const float g, const float b, const float alpha,
	const float w /* line width of the grid */);

void cnvCvRTToGlRT(const Mat & r, const Mat & t, GLdouble * m);
void cnvCvRTToGlRT(const Mat & r, const Mat & t, GLfloat * m);
void printGlMatrix(const GLfloat * m);
void printGlMatrix(const GLdouble * m);

#endif
