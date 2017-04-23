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


/////////////////////////////////////////////////////////////////////////////// c_gl_text_obj
void c_gl_text_obj::parse_var_name(ifstream & ifile, char * str_var, int & len_str_var)
{
	len_str_var = 0;
	while (1){
		if (ifile.eof())
			break;

		char c = ifile.get();
		if (c == '"'){
			break;
		}

		str_var[len_str_var] = c;
		len_str_var++;
	}

	str_var[len_str_var] = '\0';
}

int c_gl_text_obj::parse_int(ifstream & ifile, char * str_var, int & len_str_var)
{
	len_str_var = 0;
	while (1){
		if (ifile.eof())
			break;

		char c = ifile.get();
		if (c == ',' || c == '}')
			break;

		str_var[len_str_var] = c;
		len_str_var++;
	}
	str_var[len_str_var] = '\0';

	return atoi(str_var);
}

float c_gl_text_obj::parse_float(ifstream & ifile, char * str_var, int & len_str_var)
{
	len_str_var = 0;
	while (1){
		if (ifile.eof())
			break;

		char c = ifile.get();
		if (c == ',' || c == '}')
			break;

		str_var[len_str_var] = c;
		len_str_var++;
	}
	str_var[len_str_var] = '\0';

	return atof(str_var);
}

c_gl_text_obj::c_gl_text_obj() : vertices(NULL), texcoord(NULL)
{
}

c_gl_text_obj::~c_gl_text_obj()
{
	if (vertices)
		delete[] vertices;

	vertices = NULL;

	if (indices)
		delete[] indices;
	indices = NULL;

	if (texcoord)
		delete[] texcoord;

	texcoord = NULL;
}

bool c_gl_text_obj::init(const char * ftex, const char * finf, GLuint _modeloc,
	 GLuint _posloc, GLuint _txcloc, GLuint _smploc, GLuint _clrloc, unsigned int _sz_buf)
{
	modeloc = _modeloc;
	posloc = _posloc;
	txcloc = _txcloc;
	smploc = _smploc;
	clrloc = _clrloc;
	sz_buf = _sz_buf;

	vector <s_texcoord> _texcoord(128);

	// loop until eof
	// seeking for "index":<uint>
	// seeking for "x":<float> "y":<float> "width":<width> "height":<height>
	// add _texcoord
	ifstream ifile(finf);
	if (!ifile.is_open()){
		cerr << "Failed to open file " << finf << endl;
		return false;
	}

	parser_state = ps_none;
	char str_var[128];
	int len_str_var = 0;
	int index;
	float x, y, w, h;

	bool bfind = false;
	while (!ifile.eof()){
		char c;
		c = ifile.get();
		switch (c){
		case '"':
			parse_var_name(ifile, str_var, len_str_var);
			if (strcmp("index", str_var) == 0){
				parser_state = ps_index;
				bfind = true;
			}
			else if (strcmp("x", str_var) == 0 && bfind){
				parser_state = ps_x;
			}
			else if (strcmp("y", str_var) == 0 && bfind){
				parser_state = ps_y;
			}
			else if (strcmp("width", str_var) == 0 && bfind){
				parser_state = ps_w;
			}
			else if (strcmp("height", str_var) == 0 && bfind){
				parser_state = ps_h;
			}
			break;
		case ':':
			if (!bfind)
				break;
			switch (parser_state){
			case ps_index:
				index = parse_int(ifile, str_var, len_str_var);
				break;
			case ps_x:
				x = parse_float(ifile, str_var, len_str_var);
				break;
			case ps_y:
				y = parse_float(ifile, str_var, len_str_var);
				break;
			case ps_w:
				w = parse_float(ifile, str_var, len_str_var);
				break;
			case ps_h:
				h = parse_float(ifile, str_var, len_str_var);
				_texcoord[index].u = x;
				_texcoord[index].v = y + 0.02;
				_texcoord[index].w = w;
				_texcoord[index].h = h;
				bfind = false;
				break;
			}
			parser_state = ps_none;
			break;
		}
	}

	Mat tex = imread(ftex);
	if (tex.empty()){
		cerr << "Failed to open file " << ftex << endl;
		return false;
	}

	Mat tex_r = Mat::zeros(tex.rows, tex.cols, CV_8UC1);
	MatIterator_<Vec3b> itr = tex.begin<Vec3b>();
	MatIterator_<uchar> itr_dst = tex_r.begin<uchar>();
	for (; itr != tex.end<Vec3b>(); itr++, itr_dst++){
		*itr_dst = (*itr)[0];
	}

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &htex);
	glBindTexture(GL_TEXTURE_2D, htex);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	awsFlip(tex_r, false, true, false);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, tex_r.cols, tex_r.rows, 0,
		GL_RED, GL_UNSIGNED_BYTE, tex_r.data);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(2, vbo);
	vertices = new s_vertex[sz_buf * 4];
	indices = new unsigned short[sz_buf * 6];

	texcoord = new s_texcoord[_texcoord.size()];
	for (int itcd = 0; itcd < _texcoord.size(); itcd++){
		texcoord[itcd] = _texcoord[itcd];
	}

	sbis.reserve(128);

	return true;
}

const int c_gl_text_obj::reserv(const unsigned int len)
{
	int isbis = -1;
	for (int i = 0; i < sbis.size(); i++){
		if (!sbis[i].bvalid){
			isbis = i;
		}
	}
	if (isbis < 0){
		sbis.push_back(s_string_buffer_inf());
		isbis =(int)(sbis.size() - 1);
	}

	sbis[isbis].length = len;
	sbis[isbis].str = new char[len + 1];
	sbis[isbis].bvalid = true;
	return isbis;
}

void c_gl_text_obj::set(int handle, const char * str)
{
	if (handle < sbis.size()){
		strncpy(sbis[handle].str, str, sbis[handle].length);
		sbis[handle].str[sbis[handle].length - 1] = '\0';
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config(int handle, const glm::vec4 & clr,
	const glm::vec2 & sz_fnt, const glm::vec2 & mgn, const e_anchor anch,
	const glm::vec2 & t, const float  rot)
{
	if (handle < sbis.size()){
		sbis[handle].sz_fnt = sz_fnt;
		float s = (float)sin(rot), c = (float)cos(rot);
		sbis[handle].rot = glm::mat2(c, -s, s, c);
		sbis[handle].t = t;
		sbis[handle].clr = clr;
		sbis[handle].anch = anch;
		sbis[handle].mgn = mgn;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_font_size(const int handle, glm::vec2 & sz_fnt)
{
	if (handle < sbis.size()){
		sbis[handle].sz_fnt = sz_fnt;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_font_space(const int handle, const glm::vec2 & mgn)
{
	if (handle < sbis.size()){
		sbis[handle].mgn = mgn;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_color(const int handle, const glm::vec4 & clr)
{
	if (handle < sbis.size()){
		sbis[handle].clr = clr;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_position(const int handle, glm::vec2 & t)
{
	if (handle < sbis.size()){
		sbis[handle].t = t;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_rotation(const int handle, const float rot)
{
	if (handle < sbis.size()){
		float s = (float)sin(rot), c = (float)cos(rot);
		sbis[handle].rot = glm::mat2(c, -s, s, c);
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::config_anchor(const int handle, const e_anchor anch)
{
	if (handle < sbis.size()){
		sbis[handle].anch = anch;
		sbis[handle].bupdate = false;
	}
}

void c_gl_text_obj::remove(int handle)
{
	if (handle < sbis.size()){
		sbis[handle].bvalid = false;
		if (sbis[handle].str)
			delete[] sbis[handle].str;
	}
}

void c_gl_text_obj::update_vertices(bool bfull)
{
	// creating vertices
	unsigned int offset = 0;
	s_vertex * _vtx = vertices;
	unsigned short * _idx = indices;
	for (int istr = 0; istr < sbis.size(); istr++){
		s_string_buffer_inf & sbi = sbis[istr];
		if (!sbis[istr].bvalid)
			continue;
		if (bfull){
			sbi.offset = offset;
			sbi.vtx = _vtx;
			sbi.idx = _idx;
			update_vertices(sbi);
			_vtx = sbi.vtx + sbi.length * 4;
			_idx = sbi.idx + sbi.length * 6;
			offset = sbi.offset + sbi.length;
		}
		else{
			if (sbi.bupdate)
				continue;
			update_vertices(sbi);
		}
	}
	if (bfull){
		num_vertices = offset * 4;
		total_str_len = offset;
	}
}

void c_gl_text_obj::update_vertices(s_string_buffer_inf & sbi)
{
	sbi.chars = 0;
	char * str = sbi.str;
	float x = 0.f, y = 0.f;
	float xmin = FLT_MAX, xmax = -FLT_MAX, ymin = FLT_MAX, ymax = -FLT_MAX;
	s_vertex * _vtx = sbi.vtx;
	unsigned short * _idx = sbi.idx;
	unsigned int offset = sbi.offset;
	for (int ic = 0; ic < sbi.length; ic++){
		if (str[ic] == '\n'){
			y -= sbi.mgn[1];
			x = 0.f;
		}
		else{
			s_texcoord & tc = texcoord[str[ic]];
			if (tc.w != 0.f){
				_vtx[0].x = _vtx[2].x = x;
				_vtx[1].x = _vtx[3].x = x + sbi.sz_fnt[0];
				_vtx[0].y = _vtx[1].y = y;
				_vtx[2].y = _vtx[3].y = y - sbi.sz_fnt[1];
				_vtx[0].u = _vtx[2].u = tc.u;
				_vtx[1].u = _vtx[3].u = tc.u + tc.w;
				_vtx[0].v = _vtx[1].v = tc.v + tc.h;
				_vtx[2].v = _vtx[3].v = tc.v;
				_idx[0] = (unsigned short)(4 * offset);
				_idx[1] = (unsigned short)(_idx[0] + 2);
				_idx[2] = (unsigned short)(_idx[0] + 1);
				_idx[3] = (unsigned short)(_idx[0] + 1);
				_idx[4] = (unsigned short)(_idx[0] + 2);
				_idx[5] = (unsigned short)(_idx[0] + 3);
				xmin = min(xmin, _vtx[0].x);
				ymin = min(ymin, _vtx[2].y);
				xmax = max(xmax, _vtx[1].x);
				ymax = max(ymax, _vtx[0].y);
				_vtx += 4;
				_idx += 6;
				offset++;
				sbi.chars++;
			}

			x += sbi.mgn[0];
		}
	}
	// shifting center position
	float cx, cy;
	switch (sbi.anch & 0x0F)
	{
	case an_hl:
		cx = xmin;
		break;
	case an_hc:
		cx = (float)(0.5 * (xmax + xmin));
		break;
	case an_hr:
		cx = xmax;
	}

	switch (sbi.anch & 0xF0)
	{
	case an_vb:
		cy = ymin;
		break;
	case an_vc:
		cy = (float)(0.5 * (ymin + ymax));
		break;
	case an_vt:
		cy = ymax;
	}

	int n = sbi.chars * 4;
	for (int ic = 0; ic < n; ic++){
		s_vertex & v = sbi.vtx[ic];
		v.x -= cx;
		v.y -= cy;
		glm::vec2 tmp;
		tmp.x = v.x * sbi.rot[0][0] + v.y * sbi.rot[0][1] + sbi.t.x;
		tmp.y = v.x * sbi.rot[1][0] + v.y * sbi.rot[1][1] + sbi.t.y;
		v.x = tmp.x;
		v.y = tmp.y;
	}

	xmin -= cx;
	xmax -= cx;
	ymin -= cy;
	ymax -= cy;

	sbi.box[0].x = xmin; sbi.box[0].y = ymin;
	sbi.box[1].x = xmin; sbi.box[1].y = ymax;
	sbi.box[2].x = xmax; sbi.box[2].y = ymin;
	sbi.box[3].x = xmax; sbi.box[3].y = ymax;
	for (int ib = 0; ib < 4; ib++){
		glm::vec2 tmp;
		tmp.x = sbi.box[ib].x * sbi.rot[0][0] + sbi.box[ib].y * sbi.rot[0][1] + sbi.t.x;
		tmp.y = sbi.box[ib].x * sbi.rot[1][0] + sbi.box[ib].y * sbi.rot[1][1] + sbi.t.y;
		sbi.box[ib] = tmp;
	}

	sbi.bupdate = true;
}

bool c_gl_text_obj::render(unsigned int texture_unit)
{
	update_vertices(false);
	glUniform1i(modeloc, 1);

	// vertex buffer
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(s_vertex)*num_vertices, vertices, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(posloc, 2, GL_FLOAT, GL_FALSE,
		sizeof(s_vertex), 0);
	glVertexAttribPointer(txcloc, 2, GL_FLOAT, GL_FALSE,
		sizeof(s_vertex), (const void*)(sizeof(float)* 2));
	glEnableVertexAttribArray(posloc);
	glEnableVertexAttribArray(txcloc);
	// index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)* 6 * total_str_len, indices, GL_DYNAMIC_DRAW);

	glActiveTexture(GL_TEXTURE0 + texture_unit);
	glUniform1i(smploc, texture_unit);
	glBindTexture(GL_TEXTURE_2D, htex);

	// rendering
	for (int istr = 0; istr < sbis.size(); istr++){
		if (!sbis[istr].bact)
			continue;
		glUniform4fv(clrloc, 1, glm::value_ptr(sbis[istr].clr));
		glDrawElements(GL_TRIANGLES, sbis[istr].chars * 6, GL_UNSIGNED_SHORT, (const void*)(sizeof(unsigned short)* sbis[istr].offset * 6));
	}

	return true;
}

int c_gl_text_obj::collision(const glm::vec2 & pt)
{
	for (int ih = 0; ih < sbis.size(); ih++){
		s_string_buffer_inf & sbi = sbis[ih];
		if (!sbi.bvalid || !sbi.bact)
			continue;
		glm::vec2 vbase[2];
		vbase[0] = sbi.box[2] - sbi.box[0];
		vbase[1] = sbi.box[1] - sbi.box[0];

		glm::vec2 vpt[4];
		vpt[0] = pt - sbi.box[0];
		vpt[1] = pt - sbi.box[3];
		if (glm::dot(vpt[0], vbase[0]) < 0)
			continue;

		if (glm::dot(vpt[0], vbase[1]) < 0)
			continue;

		if (glm::dot(vpt[1], vbase[0]) > 0)
			continue;

		if (glm::dot(vpt[1], vbase[1]) > 0)
			continue;

		return ih;
	}
	return -1;
}

////////////////////////////////////////////////////////////////////// c_gl_line_obj
c_gl_line_obj::c_gl_line_obj() :bupdated(false), vertices(NULL)
{
}

c_gl_line_obj::~c_gl_line_obj()
{
	if (vertices)
		delete[] vertices;
	vertices = NULL;

}

bool c_gl_line_obj::init(GLuint _modeloc, GLuint _posloc, GLuint _Mmvploc, GLuint _clrloc, unsigned int buffer_size)
{
	vertices = new s_vertex[buffer_size];
	if (!vertices)
		return false;

	modeloc = _modeloc;
	posloc = _posloc;
	Mmvploc = _Mmvploc;
	clrloc = _clrloc;
	bupdated = false;

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);

	glGetFloatv(GL_LINE_WIDTH_RANGE, wrange);
	width = wrange[2];

	num_total_vertices = 0;
	return true;
}

int c_gl_line_obj::set(const int npts, const float * pts)
{
	if (num_total_vertices + npts >= buffer_size)
		return -1;

	int handle = -1;
	for (int ih = 0; ih < lbis.size(); ih++)
	{
		if (!lbis[ih].bvalid){
			handle = ih;
			break;
		}
	}

	if (handle < 0){
		lbis.push_back(s_line_buffer_inf());
		handle = (int)(lbis.size() - 1);
	}

	s_line_buffer_inf & lbi = lbis[handle];
	lbi.bvalid = true;	
	lbi.npts = npts;
	unsigned int offset = 0;
	for (int ih = 0; ih < lbis.size(); ih++){
		lbis[ih].offset = offset;
		offset += lbis[ih].npts;
	}

	for (int ih = lbis.size() - 1; ih >= 0; ih--)
	{
		if (lbis[ih].offset + vertices == lbis[ih].vtx)
			continue;
		if (ih == handle){			
			s_vertex * vtx = vertices + lbis[ih].offset;
			memcpy((void*)vtx, (void*)pts, sizeof(float)* npts * 3);
			lbis[ih].vtx = vtx;
		}
		else{
			s_vertex * vtx = lbis[ih].vtx + lbis[ih].npts - 1;
			s_vertex * vtx_src = vertices + lbis[ih].offset + lbis[ih].npts - 1;
			for (int iv = 0; iv < lbis[ih].npts; iv++)
			{
				*vtx = *vtx_src;
				vtx--;
				vtx_src--;
			}
			lbis[ih].vtx = vertices + lbis[ih].offset;
		}
	}

	num_total_vertices += npts;

	bupdated = false;
	return handle;
}

void c_gl_line_obj::remove(const int handle)
{
	if (handle < lbis.size()){
		num_total_vertices -= lbis[handle].npts;
		lbis[handle].bvalid = false;
		lbis[handle].npts = 0;
		lbis[handle].offset = 0;
		lbis[handle].vtx = 0;
	}
}

void c_gl_line_obj::update_vertices()
{
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(s_vertex) * num_total_vertices, (void*)vertices, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(posloc, 3, GL_FLOAT, GL_FALSE, sizeof(s_vertex), 0);
	glEnableVertexAttribArray(posloc);
	bupdated = true;
}

void c_gl_line_obj::render(const glm::mat4 & PV)
{
	if (!bupdated)
		update_vertices();

	glLineWidth(width);
	glEnable(GL_LINE_SMOOTH);

	glm::mat4 T(1.0);
	T = glm::translate(T, t);
	glm::mat4 m = PV * T * R;
	glUniformMatrix4fv(Mmvploc, 1, GL_FALSE, glm::value_ptr(m));
	glUniform4fv(clrloc, 1, glm::value_ptr(clr));
	glUniform1i(modeloc, 2);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(posloc);

	for (int ih = 0; ih < lbis.size(); ih++){
		s_line_buffer_inf & lbi = lbis[ih];
		if (!lbi.bvalid || !lbi.bactive)
			continue;

		glDrawArrays(GL_LINE_STRIP, lbi.offset, lbi.npts);
	}
}


/////////////////////////////////////////////////////////////// c_gl_point_obj
bool c_gl_point_obj::init(GLuint _modeloc, GLuint _posloc, GLuint _Mmvploc, GLuint _clrloc)
{
	modeloc = _modeloc;
	posloc = _posloc;
	Mmvploc = _Mmvploc;
	clrloc = _clrloc;

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);

	glGetFloatv(GL_POINT_SIZE_RANGE, szrange);
	size = szrange[2];
	return true;
}

void c_gl_point_obj::set(const int num_points, const float * points)
{
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	num_total_vertices = num_points;
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 *  num_total_vertices, (void*)points, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(posloc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);
	glEnableVertexAttribArray(posloc);
}

void c_gl_point_obj::render(const glm::mat4 & PV)
{
	glPointSize(size);

	glm::mat4 T(1.0);
	T = glm::translate(T, t);
	glm::mat4 m = PV * T * R;
	glUniformMatrix4fv(Mmvploc, 1, GL_FALSE, glm::value_ptr(m));
	glUniform4fv(clrloc, 1, glm::value_ptr(clr));
	glUniform1i(modeloc, 2);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(posloc);

	glDrawArrays(GL_POINTS, 0, num_total_vertices);

}