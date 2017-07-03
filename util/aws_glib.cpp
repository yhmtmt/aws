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
/////////////////////////////////////////////////////////////////////////////// glsl source loader

void printShaderInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten = 0;
	char *infoLog;

	glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n", infoLog);
		free(infoLog);
	}
}

void printProgramInfoLog(GLuint obj)
{
	int infologLength = 0;
	int charsWritten = 0;
	char *infoLog;

	glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 0)
	{
		infoLog = (char *)malloc(infologLength);
		glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
		printf("%s\n", infoLog);
		free(infoLog);
	}
}


char * load_glsl_text(const char * fname)
{
	ifstream fin(fname);
	if (!fin.is_open())
		return NULL;

	fin.seekg(0, fin.end);
	unsigned int flen = fin.tellg();
	fin.seekg(0, fin.beg);

	char * txt = new char[flen + 1];
	if (!txt)
		return false;

	memset((void*)txt, 0, (size_t)(flen + 1));
	fin.read(txt, flen);
	return txt;
}

bool load_glsl_program(const char * ffs, const char * fvs,  GLuint & p)
{
	char *vs = NULL, *fs = NULL;
	GLuint f, v;
	v = glCreateShader(GL_VERTEX_SHADER);
	if (!v)
		return false;
	f = glCreateShader(GL_FRAGMENT_SHADER);
	if (!f)
		return false;


	vs = load_glsl_text(fvs);
	fs = load_glsl_text(ffs);

	const char * vv = vs;
	const char * ff = fs;

	glShaderSource(v, 1, &vv, NULL);
	glShaderSource(f, 1, &ff, NULL);

	delete[] vs;
	delete[] fs;

	glCompileShader(v);
	glCompileShader(f);

	printShaderInfoLog(v);
	printShaderInfoLog(f);

	p = glCreateProgram();
	glAttachShader(p, v);
	glAttachShader(p, f);
	glLinkProgram(p);
	printProgramInfoLog(p);

	glDeleteShader(v);
	glDeleteShader(f);

	return true;
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

c_gl_text_obj::c_gl_text_obj() : vao(0), vertices(NULL), texcoord(NULL), bupdated(false)
{
}

c_gl_text_obj::~c_gl_text_obj()
{
	destroy();
}

void c_gl_text_obj::destroy()
{
	if (vao != 0){
		glDeleteBuffers(2, vbo);
		glDeleteVertexArrays(1, &vao);
		vao = 0;
	}

	if (vertices)
		delete[] vertices;

	vertices = NULL;

	if (indices)
		delete[] indices;
	indices = NULL;

	if (texcoord)
		delete[] texcoord;

	texcoord = NULL;

	for (int isbi = 0; isbi < sbis.size(); isbi++){
		delete[] sbis[isbi].str;
		sbis[isbi].str = NULL;
	}
	sbis.clear();
}

bool c_gl_text_obj::init(const char * ftex, const char * finf, GLuint _modeloc,
	 GLuint _posloc, GLuint _txcloc, GLuint _smploc, GLuint _clrloc, GLuint _bkgclrloc, GLuint _depthloc,
	 unsigned int _sz_buf)
{
	modeloc = _modeloc;
	posloc = _posloc;
	txcloc = _txcloc;
	smploc = _smploc;
	clrloc = _clrloc;
	bkgclrloc = _bkgclrloc;
	sz_buf = _sz_buf;
	depthloc = _depthloc;

	int depth;
	glGetIntegerv(GL_DEPTH_BITS, &depth);
	zstep = (float)(2.0 / (float)(1 << depth));

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
	int num_chars = len;
	for (int i = 0; i < sbis.size(); i++){
		if (!sbis[i].bvalid){
			isbis = i;
		}
		else {
			num_chars += sbis[i].length;
		}
	}

	if (num_chars >= sz_buf) {
		cerr << "c_gl_text_obj::reserv cannot allocate sufficient size for buffer. Buffer size should be increased in initialization phase. " << endl;
		return -1;
	}

	if (isbis < 0){
		sbis.push_back(s_string_buffer_inf());
		isbis =(int)(sbis.size() - 1);
	}

	sbis[isbis].length = len + 1;
	sbis[isbis].str = new char[len + 1];
	sbis[isbis].bvalid = true;
	bupdated = false;
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

void c_gl_text_obj::config(int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & mgn, const e_anchor anch,
	const glm::vec2 & t, const float  rot, const int depth)
{
	if (handle < sbis.size()){
		sbis[handle].sz_fnt = sz_fnt;
		float s = (float)sin(rot), c = (float)cos(rot);
		sbis[handle].rot = glm::mat2(c, -s, s, c);
		sbis[handle].t = t;
		sbis[handle].bkgclr = bkgclr;
		sbis[handle].clr = clr;
		sbis[handle].anch = anch;
		sbis[handle].mgn = mgn;
		sbis[handle].bupdate = false;
		config_depth(handle, depth);
	}
}

void c_gl_text_obj::config_font_size(const int handle, const glm::vec2 & sz_fnt)
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

void c_gl_text_obj::config_color(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr)
{
	if (handle < sbis.size()){
		sbis[handle].clr = clr;
		sbis[handle].bkgclr = bkgclr;
		//sbis[handle].bupdate = false;
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

void c_gl_text_obj::config_rotation(const int handle, const float c, const float s)
{
	if (handle < sbis.size())
	{
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


void c_gl_text_obj::config_depth(const int handle, const int depth)
{
	if (handle < sbis.size())
	{
		sbis[handle].z = -1.0 + (float)(depth * zstep);
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
	update_vertices(!bupdated);
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
		glUniform4fv(bkgclrloc, 1, glm::value_ptr(sbis[istr].bkgclr));
		glUniform1f(depthloc, sbis[istr].z);
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
c_gl_line_obj::c_gl_line_obj() :vao(0), bupdated(false), vertices(NULL)
{
}

c_gl_line_obj::~c_gl_line_obj()
{
	destroy();
}

void c_gl_line_obj::destroy()
{
	if (vao != 0)
	{
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
		vao = 0;
	}

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

	num_total_vertices = 0;
	return true;
}

int c_gl_line_obj::add(const int npts, const float * pts)
{
	if (num_total_vertices + npts >= buffer_size){
		cerr << "c_gl_line_obj::add cannot assign sufficient size of buffer. Buffer size should be increased in initialization phase." << endl;
		return -1;
	}
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


	glm::mat4 T(1.0);
	glUniform1i(modeloc, 2);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(posloc);

	for (int ih = 0; ih < lbis.size(); ih++){
		s_line_buffer_inf & lbi = lbis[ih];
		if (!lbi.bvalid || !lbi.bactive)
			continue;
		T = glm::translate(T, lbi.t);
		glm::mat4 m = PV * T * lbi.R;
		glUniformMatrix4fv(Mmvploc, 1, GL_FALSE, glm::value_ptr(m));
		glUniform4fv(clrloc, 1, glm::value_ptr(lbi.clr));

		glLineWidth(lbi.w);
		glEnable(GL_LINE_SMOOTH);
		glDrawArrays(GL_LINE_STRIP, lbi.offset, lbi.npts);
	}
}


/////////////////////////////////////////////////////////////// c_gl_point_obj
void c_gl_point_obj::destroy()
{
	if (vao != 0){
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
		vao = 0;
	}
}
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

void c_gl_point_obj::add(const int num_points, const float * points)
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

//////////////////////////////////////////////////////////////////// c_gl_2d_line_obj
c_gl_2d_line_obj::c_gl_2d_line_obj(): vao(0), bupdated(false), vertices(NULL)
{

}

c_gl_2d_line_obj::~c_gl_2d_line_obj()
{
	destroy();
}


void c_gl_2d_line_obj::destroy()
{
	if (vao != 0)
	{
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
		vao = 0;
	}

	if (vertices)
		delete[] vertices;
	vertices = NULL;
}


void c_gl_2d_line_obj::config_depth(const int handle, const int depth)
{
	if (handle < lbis.size())
	{
		lbis[handle].z = 1.0 - (float)(depth * zstep);
	}
}


bool c_gl_2d_line_obj::init(GLuint _modeloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc, unsigned int _buffer_size)
{
	buffer_size = _buffer_size;
	vertices = new s_vertex[buffer_size * 2];
	vertices_trn = &vertices[buffer_size];

	if (!vertices)
		return false;

	modeloc = _modeloc;
	posloc = _posloc;
	clrloc = _clrloc;
	depthloc = _depthloc;
	bupdated = false;

	int depth;
	glGetIntegerv(GL_DEPTH_BITS, &depth);
	zstep = (float)(2.0 / (float)(1 << depth));

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);

	glGetFloatv(GL_LINE_WIDTH_RANGE, wrange);

	num_total_vertices = 0;
	return true;
}

int c_gl_2d_line_obj::add(const int npts, const float * pts, bool blines)
{
	if (num_total_vertices + npts >= buffer_size) {
		cerr << "c_gl_2d_line_obj::add cannot assign sufficient size of buffer. Buffer size should be extended in initialization phase." << endl;
		return -1;
	}

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
	lbi.bactive = true;
	lbi.bvalid = true;
	lbi.blines = blines;
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
			memcpy((void*)vtx, (void*)pts, sizeof(float)* npts * 2);
			lbis[ih].vtx = vtx;
			lbis[ih].vtx_trn = lbis[ih].vtx + buffer_size;
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
			lbis[ih].vtx_trn = lbis[ih].vtx + buffer_size;
		}
	}

	num_total_vertices += npts;

	bupdated = false;
	return handle;
}

void c_gl_2d_line_obj::config_points(const int handle, const float * pts)
{
	if (handle < lbis.size()){
		s_vertex * vtx = lbis[handle].vtx;
		int npts = lbis[handle].npts;
		memcpy((void*)vtx, (void*)pts, sizeof(float)* 2 * npts);
		lbis[handle].bupdated = false;
		bupdated = false;
	}
}

void c_gl_2d_line_obj::remove(const int handle)
{
	if (handle < lbis.size()){
		num_total_vertices -= lbis[handle].npts;
		lbis[handle].bvalid = false;
		lbis[handle].npts = 0;
		lbis[handle].offset = 0;
		lbis[handle].vtx = 0;
	}
}

void c_gl_2d_line_obj::update_vertices()
{

	for (int ilbi = 0; ilbi < lbis.size(); ilbi++){
		s_line_buffer_inf & lbi = lbis[ilbi];
		if (!lbi.bactive || !lbi.bvalid || lbi.bupdated)
			continue;
		
		s_vertex * vtx = lbi.vtx;
		s_vertex * vtx_trn = lbi.vtx_trn;
		glm::mat2 & r = lbi.R;
		for (int iv = 0; iv < lbi.npts; iv++){
			vtx_trn[iv].x = vtx[iv].x * r[0][0] + vtx[iv].y * r[0][1] + lbi.t.x;
			vtx_trn[iv].y = vtx[iv].x * r[1][0] + vtx[iv].y * r[1][1] + lbi.t.y;
		}
	}

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(s_vertex)* num_total_vertices, (void*)vertices_trn, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(posloc, 2, GL_FLOAT, GL_FALSE, sizeof(s_vertex), 0);
	glEnableVertexAttribArray(posloc);

	bupdated = true;
}

void c_gl_2d_line_obj::render()
{
	if (!bupdated)
		update_vertices();

	glm::mat4 T(1.0);
	glUniform1i(modeloc, 3);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(posloc);

	for (int ih = 0; ih < lbis.size(); ih++){
		s_line_buffer_inf & lbi = lbis[ih];
		if (!lbi.bvalid || !lbi.bactive)
			continue;
		glUniform4fv(clrloc, 1, glm::value_ptr(lbi.clr));
		glUniform1f(depthloc, lbi.z);

		glLineWidth(lbi.w);
		glEnable(GL_LINE_SMOOTH);
		if (lbi.blines)
			glDrawArrays(GL_LINES, lbi.offset, lbi.npts);
		else
			glDrawArrays(GL_LINE_STRIP, lbi.offset, lbi.npts);
	}
}


//////////////////////////////////////////////////////////////////// c_gl_2d_obj
c_gl_2d_obj::c_gl_2d_obj() :vtxbuf(NULL), idxbuf(NULL)
{
}

c_gl_2d_obj::~c_gl_2d_obj()
{
	destroy();
}

void c_gl_2d_obj::destroy()
{
	if (vao != 0){
		glDeleteBuffers(2, vbo);
		glDeleteVertexArrays(1, &vao);
		vao = 0;
	}

	prottype.destroy();

	if (vtxbuf)
		delete[] vtxbuf;

	if (idxbuf)
		delete[] idxbuf;

	vtxbuf = NULL;
	idxbuf = NULL;

}

bool c_gl_2d_obj::init_rectangle(GLuint _modeloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
	const glm::vec2 & plb, glm::vec2 & sz, const unsigned int _buffer_size)
{
	type = rectangle;

	modeloc = _modeloc;
	posloc = _posloc;
	clrloc = _clrloc;
	depthloc = _depthloc;

	buffer_size = _buffer_size;

	int depth;
	glGetIntegerv(GL_DEPTH_BITS, &depth);
	zstep = (float)(2.0 / (float)(1 << depth));

	prottype.nvtx = 4;
	prottype.nidxs = 5;
	prottype.nidxt = 6;
	prottype.vtx = new s_vertex[4];
	prottype.idxs = new unsigned short[prottype.nidxs + prottype.nidxt];
	prottype.idxt = prottype.idxs + prottype.nidxs;
	prottype.vtx[0].x = plb.x;
	prottype.vtx[0].y = plb.y;
	prottype.vtx[1].x = plb.x + sz.x;
	prottype.vtx[1].y = plb.y;
	prottype.vtx[2].x = plb.x + sz.x;
	prottype.vtx[2].y = plb.y + sz.y;
	prottype.vtx[3].x = plb.x;
	prottype.vtx[3].y = plb.y + sz.y;

	prottype.idxs[0] = 0;
	prottype.idxs[1] = 1;
	prottype.idxs[2] = 2;
	prottype.idxs[3] = 3;
	prottype.idxs[4] = 0;

	prottype.idxt[0] = 0;
	prottype.idxt[1] = 1;
	prottype.idxt[2] = 2;
	prottype.idxt[3] = 2;
	prottype.idxt[4] = 3;
	prottype.idxt[5] = 0;

	vtxbuf = new s_vertex[buffer_size * prottype.nvtx];
	idxbuf = new unsigned short[buffer_size * (prottype.nidxt + prottype.nidxs)];

	// allocating vertex buffer
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(2, vbo);

	return true;
}

bool c_gl_2d_obj::init_circle(GLuint _modeloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
	const unsigned int npts, const float rx, const float ry, const unsigned int _buffer_size)
{
	type = circle;

	modeloc = _modeloc;
	posloc = _posloc;
	clrloc = _clrloc;
	depthloc = _depthloc;

	buffer_size = _buffer_size;

	int depth;
	glGetIntegerv(GL_DEPTH_BITS, &depth);
	zstep = (float)(2.0 / (float) (1 << depth));

	prottype.nvtx = npts;
	prottype.nidxs = npts + 1;
	prottype.nidxt = 3 * (npts - 2);
	prottype.vtx = new s_vertex[npts];
	prottype.idxs = new unsigned short[prottype.nidxs + prottype.nidxt];
	prottype.idxt = prottype.idxs + prottype.nidxs;
	prottype.r = glm::vec2(rx, ry);

	float th, thstep = (float)(2.0 * CV_PI / (float) npts);
	for (int iv = 0; iv < npts; iv++){
		th = (float)(thstep * (float) iv);
		prottype.vtx[iv].x = (float)(rx * cos(th));
		prottype.vtx[iv].y = (float)(ry * sin(th));
	}

	// triangle index
	unsigned short p0 = 0, p1 = 1, p2 = 2;
	for (int itri = 0; itri < npts - 2; itri++){
		int idx0 = itri * 3, idx1 = idx0 + 1, idx2 = idx0 + 2;
		prottype.idxt[idx0] = p0;
		prottype.idxt[idx1] = p1;
		prottype.idxt[idx2] = p2;
		p1 = p2;
		p2++;
	}

	// shape index
	for (int idx = 0; idx < prottype.nidxs - 1; idx++){
		prottype.idxs[idx] = (unsigned short) idx;
	}
	prottype.idxs[prottype.nidxs - 1] = 0;

	vtxbuf = new s_vertex[buffer_size * prottype.nvtx];
	idxbuf = new unsigned short[buffer_size * (prottype.nidxt + prottype.nidxs)];

	// allocating vertex buffer
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(2, vbo);

	return true;
}

bool c_gl_2d_obj::init(GLuint _modeloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
	const unsigned int npts, const float * points,
	const unsigned int nids, const unsigned short * indices,
	const unsigned int _buffer_size)
{
	type = custom;

	modeloc = _modeloc;
	posloc = _posloc;
	clrloc = _clrloc;
	depthloc = _depthloc;

	buffer_size = _buffer_size;

	int depth;
	glGetIntegerv(GL_DEPTH_BITS, &depth);
	zstep = (float)(2.0 / (float)(1 << depth));

	if (nids % 3 != 0) // triangle set is assumed.
		return false;

	prottype.nvtx = npts;
	prottype.nidxt = nids;

	// calculating shape indices
	{
		struct s_edge{
			int b, e;
		};

		vector<s_edge> edges(nids);
		for (int i = 0; i < nids; i += 3){
			edges[i].b = indices[i]; edges[i].e = indices[i + 1];
			edges[i + 1].b = indices[i + 1]; edges[i + 1].e = indices[i + 2];
			edges[i + 2].b = indices[i + 2]; edges[i + 2].e = indices[i];
		}

		// eliminating identical edges
		for (int i = 0; i < nids; i++){
			for (int j = i; j < nids; j++){
				if (edges[j].b < 0)
					continue;
				if ((edges[j].b == edges[i].b && edges[j].e == edges[i].e) ||
					(edges[j].b == edges[i].e && edges[j].e == edges[i].b))
					edges[j].b = -1;
			}
		}

		// relocating edges
		int nidxs = 0, idx_min = INT_MIN;
		for (int i = 0, j = 0; i < nids && j < nids; i++){
			if (edges[i].b < 0){
				for (; j < nids; j++){
					if (edges[j].b < 0){
						edges[i] = edges[j];
						edges[j].b = -1;
					}
				}
			}

			if (edges[i].b >= 0){
				idx_min = min(idx_min, edges[i].b);
				idx_min = min(idx_min, edges[i].e);
			}
			nidxs = i;
		}
		nidxs += 1;

		// allocating and loading shape index array
		prottype.nidxs = nidxs;
		prottype.idxs = new unsigned short[nidxs + nids];
		prottype.idxt = prottype.idxs + nidxs;
		prottype.idxs[0] = idx_min;
		for (int i = 0; i < nidxs - 1; i++){
			for (int iedge = 0; iedge < nidxs - 1; iedge++){
				if (edges[iedge].b == prottype.idxs[i]){
					prottype.idxs[i + 1] = edges[iedge].e;
					break;
				}
				else if (edges[iedge].e == prottype.idxs[i]){
					prottype.idxs[i + 1] = edges[iedge].b;
					break;
				}
			}
		}
		if (prottype.idxs[0] != prottype.idxs[nidxs - 1]){
			delete[] prottype.idxs;
			cerr << "Given 2d object is not closed shape." << endl;
			return false;
		}
	}

	prottype.vtx = new s_vertex[npts];
	memcpy((void*)prottype.vtx, (const void*)points, sizeof(s_vertex)* npts);
	memcpy((void*) prottype.idxt, (const void*)indices, sizeof(unsigned short)* nids);

	// allocating instance buffer
	buffer_size = _buffer_size;
	iis.reserve(buffer_size);

	vtxbuf = new s_vertex[buffer_size * prottype.nvtx];
	idxbuf = new unsigned short[buffer_size * (prottype.nidxt + prottype.nidxs)];

	// allocating vertex buffer
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(2, vbo);

	return true;
}

int c_gl_2d_obj::add(const glm::vec4 & clr, const glm::vec2 & pos, 
	const float rot, const glm::vec2 & scale)
{
	int handle = -1;
	if (buffer_size == iis.size()){
		cerr << "c_gl_2d_obj::add run out of buffer. The buffer size should be larger in initialization." << endl;
		return handle;
	}

	for (int i = 0; i < iis.size(); i++)
	{
		if (!iis[i].bvalid){
			handle = i;
			break;
		}
	}

	if (handle < 0){
		if (iis.size() == buffer_size)
			return -1;
		handle = (int)iis.size();
		iis.push_back(s_inst_inf());
		iis[handle].order = handle;
		iis_sorted_by_depth.push_back(handle);
		iis[handle].order = handle;
		reorder(handle);
	}

	iis[handle].bactive = true;
	iis[handle].bvalid = true;
	iis[handle].bupdated = false;
	iis[handle].bborder = false;
	iis[handle].scale = scale;
	iis[handle].rot = rot;
	iis[handle].clr = clr;
	iis[handle].pos = pos;

	float s = (float)(sin(rot)), c = (float)(cos(rot));
	iis[handle].r = glm::mat2(c, -s, s, c);
	iis[handle].sr = iis[handle].r * glm::mat2(scale.x, 0, 0, scale.y);

	if (type == circle) {
		glm::vec2 & scale = iis[handle].scale;
		glm::vec2 & r = prottype.r;
		glm::vec2 & relps2inv = iis[handle].relps2inv;
		relps2inv.x = (float)(scale.x * r.x);
		relps2inv.y = (float)(scale.y * r.y);
		relps2inv.x *= relps2inv.x;
		relps2inv.y *= relps2inv.y;
		relps2inv.x = (float)(1.0 / relps2inv.x);
		relps2inv.y = (float)(1.0 / relps2inv.y);
	}
	return handle;
}

int c_gl_2d_obj::add(const glm::vec4 & clr, const glm::vec2 & pos, 
	const float rot, const float scale)
{
	int handle = -1;
	for (int i = 0; i < iis.size(); i++)
	{
		if (!iis[i].bvalid){
			handle = i;
			break;
		}
	}

	if (handle < 0){
		if (iis.size() == buffer_size)
			return -1;
		handle = (int)iis.size();
		iis.push_back(s_inst_inf());
		iis[handle].order = handle;
		iis_sorted_by_depth.push_back(handle);
		reorder(handle);
	}

	iis[handle].bactive = true;
	iis[handle].bvalid = true;
	iis[handle].bupdated = false;
	iis[handle].bborder = false;
	iis[handle].scale.x = scale;
	iis[handle].scale.y = scale;
	if (type == circle){
		glm::vec2 & scale = iis[handle].scale;
		glm::vec2 & r = prottype.r;
		glm::vec2 & relps2inv = iis[handle].relps2inv;
		relps2inv.x = (float)(scale.x * r.x);
		relps2inv.y = (float)(scale.y * r.y);
		relps2inv.x *= relps2inv.x;
		relps2inv.y *= relps2inv.y;
		relps2inv.x = (float)(1.0 / relps2inv.x);
		relps2inv.y = (float)(1.0 / relps2inv.y);
	}

	iis[handle].rot = rot;
	iis[handle].clr = clr;
	iis[handle].pos = pos;

	float s = (float)(sin(rot)), c = (float)(cos(rot));
	iis[handle].r = glm::mat2(c, -s, s, c);
	iis[handle].sr = iis[handle].r * glm::mat2(scale, 0, 0, scale);
	return handle;
}

void c_gl_2d_obj::remove(const int handle)
{
	if (handle < iis.size()){
		iis[handle].bvalid = false;
		iis[handle].bupdated = false;
	}
}

void c_gl_2d_obj::config_color(const int handle, const glm::vec4 & clr)
{
	if (handle < iis.size()){
		iis[handle].clr = clr;
	}
}

void c_gl_2d_obj::config_position(const int handle, const glm::vec2 & pos)
{
	if (handle < iis.size()){
		iis[handle].pos = pos;
		iis[handle].bupdated = false;
	}
}

void c_gl_2d_obj::config_rotation(const int handle, const float rot)
{
	if (handle < iis.size())
	{
		float scalex = iis[handle].scale.x;
		float scaley = iis[handle].scale.y;
		iis[handle].rot = rot;
		float s = (float)(sin(rot)), c = (float)(cos(rot));
		iis[handle].r = glm::mat2((float)(c), (float)(-s),
			(float)(s), (float)(c));
		iis[handle].sr = glm::mat2(scalex, 0, 0, scaley) * iis[handle].r;
		iis[handle].bupdated = false;
	}
}

void c_gl_2d_obj::config_scale(const int handle, const float scale)
{
	if (handle < iis.size())
	{
		iis[handle].scale.x = scale;
		iis[handle].scale.y = scale;
		float rot = iis[handle].rot;
		float s = (float)(sin(rot)), c = (float)(cos(rot));
		iis[handle].sr = glm::mat2(scale, 0, 0, scale) * iis[handle].r;
		iis[handle].bupdated = false;
		if (type == circle){
			glm::vec2 & scale = iis[handle].scale;
			glm::vec2 & r = prottype.r;
			glm::vec2 & relps2inv = iis[handle].relps2inv;
			relps2inv.x = (float)(scale.x * r.x);
			relps2inv.y = (float)(scale.y * r.y);
			relps2inv.x *= relps2inv.x;
			relps2inv.y *= relps2inv.y;
			relps2inv.x = (float)(1.0 / relps2inv.x);
			relps2inv.y = (float)(1.0 / relps2inv.y);
		}
	}
}

void c_gl_2d_obj::config_scale(const int handle, const glm::vec2 & scale)
{
	if (handle < iis.size())
	{
		iis[handle].scale = scale;
		float rot = iis[handle].rot;
		float s = (float)(sin(rot)), c = (float)(cos(rot));
		iis[handle].sr = glm::mat2((float)(scale.x * c), (float)(-scale.y * s),
			(float)(scale.x * s), (float)(scale.y * c));
		iis[handle].bupdated = false;
		if (type == circle){
			glm::vec2 & scale = iis[handle].scale;
			glm::vec2 & r = prottype.r;
			glm::vec2 & relps2inv = iis[handle].relps2inv;
			relps2inv.x = (float)(scale.x * r.x);
			relps2inv.y = (float)(scale.y * r.y);
			relps2inv.x *= relps2inv.x;
			relps2inv.y *= relps2inv.y;
			relps2inv.x = (float)(1.0 / relps2inv.x);
			relps2inv.y = (float)(1.0 / relps2inv.y);
		}
	}
}

void c_gl_2d_obj::config_border(const int handle, const bool b, const float w)
{
	if (handle < iis.size())
	{
		iis[handle].w = w;
		iis[handle].bborder = b;
		iis[handle].bupdated = false;
	}
}

void c_gl_2d_obj::config_depth(const int handle, const int depth)
{
	if (handle < iis.size())
	{
		iis[handle].z = 1.0 - (float)(depth * zstep);
		reorder(handle);
	}
}

void c_gl_2d_obj::reorder(const int handle)
{
	int order = iis[handle].order;
	float z = iis[handle].z;
	s_inst_inf & ii1 = iis[handle];
	for (int iorder = order + 1; iorder < iis_sorted_by_depth.size(); iorder++){
		s_inst_inf & ii2 = iis[iis_sorted_by_depth[iorder]];
		if (ii1.z < ii2.z){
			int handle2 = iis_sorted_by_depth[iorder];
			iis_sorted_by_depth[iorder] = handle;
			iis_sorted_by_depth[iorder - 1] = handle2;
			ii1.order = iorder;
			ii2.order = iorder - 1;
	
		}
		else{
			break;
		}
	}

	for (int iorder = order - 1; iorder >= 0; iorder--){
		s_inst_inf & ii2 = iis[iis_sorted_by_depth[iorder]];
		if (ii1.z > ii2.z){
			int handle2 = iis_sorted_by_depth[iorder];
			iis_sorted_by_depth[iorder] = handle;
			iis_sorted_by_depth[iorder + 1] = handle2;
			ii1.order = iorder;
			ii2.order = iorder + 1;
		}
	}
}

bool c_gl_2d_obj::collision(const glm::vec2 & pt, const int handle)
{
	switch (type){
	case circle:
	{
				   float px, py;
				   s_inst_inf & ii = iis[handle];
				   if (!ii.bvalid || !ii.bactive)
					   return false;
				   glm::vec2 & pos = ii.pos;
				   px = pt.x - pos.x;
				   py = pt.y - pos.y;
				   float sxx, syy;
				   glm::mat2 & r = ii.r;
				   if (ii.rot != 0.0)
				   {
					   sxx = r[0][0] * px + r[1][0] * py;
					   syy = r[0][1] * px + r[1][1] * py;
				   }
				   else{
					   sxx = px;
					   syy = py;
				   }
				   float n = (float)(sxx * sxx * ii.relps2inv.x +
					   syy * syy * ii.relps2inv.y);
				   if (n <= 1.0){
					   return true;
				   }
	}
		break;
	case rectangle:
	{
					  s_inst_inf & ii = iis[handle];
					  if (!ii.bvalid || !ii.bactive)
						  return false;
					  s_vertex * vtx = vtxbuf + ii.vtxoffset;

					  if (ii.rot == 0){
						  if (vtx[0].x < pt.x && vtx[1].x > pt.x &&vtx[0].y < pt.y && vtx[2].y > pt.y){
							  return true;
						  }
					  }

					  glm::vec2 vbase[2];
					  vbase[0] = glm::vec2(vtx[3].x - vtx[0].x, vtx[3].x - vtx[0].y);
					  vbase[1] = glm::vec2(vtx[1].x - vtx[0].x, vtx[1].y - vtx[0].y);
					  glm::vec2 vpt[4];
					  vpt[0] = glm::vec2(pt.x - vtx[2].y, pt.y - vtx[2].y);
					  vpt[1] = glm::vec2(pt.x - vtx[0].x, pt.y - vtx[0].y);
					  if (glm::dot(vpt[0], vbase[0]) < 0)
						  return false;

					  if (glm::dot(vpt[0], vbase[1]) < 0)
						  return false;

					  if (glm::dot(vpt[1], vbase[0]) > 0)
						  return false;

					  if (glm::dot(vpt[1], vbase[1]) > 0)
						  return false;

					  return true;
	}
	}
	return false;
}

int c_gl_2d_obj::collision(const glm::vec2 & pt, const vector<int> & lst)
{

	for (int ilst = 0; ilst < lst.size(); ilst){
		if (lst[ilst] >= iis.size())
			continue;

		iis[lst[ilst]].binlst = true;
	}

	int handle;
	switch (type){
	case circle:
		for (int i = 0; i < iis_sorted_by_depth.size(); i++){
			float px, py;
			s_inst_inf & ii = iis[iis_sorted_by_depth[i]];
			if (!ii.bvalid || !ii.bactive)
				continue;
			glm::vec2 & pos = ii.pos;
			px = pt.x - pos.x;
			py = pt.y - pos.y;
			float sxx, syy;
			glm::mat2 & r = ii.r;
			if (ii.rot != 0.0)
			{
				sxx = r[0][0] * px + r[1][0] * py;
				syy = r[0][1] * px + r[1][1] * py;
			}
			else{
				sxx = px;
				syy = py;
			}
			float n = (float)(sxx * sxx * ii.relps2inv.x +
				syy * syy * ii.relps2inv.y);
			if (n <= 1.0){
				handle = i;
				break;
			}
		}

		break;
	case rectangle:
		for (int i = 0; i < iis_sorted_by_depth.size(); i++){
			s_inst_inf & ii = iis[iis_sorted_by_depth[i]];
			if (!ii.bvalid || !ii.bactive)
				continue;
			s_vertex * vtx = vtxbuf + ii.vtxoffset;

			if (ii.rot == 0){
				if (vtx[0].x < pt.x && vtx[1].x > pt.x &&vtx[0].y < pt.y && vtx[2].y > pt.y){
					handle = i;
					break;
				}
			}

			glm::vec2 vbase[2];
			vbase[0] = glm::vec2(vtx[3].x - vtx[0].x, vtx[3].x - vtx[0].y);
			vbase[1] = glm::vec2(vtx[1].x - vtx[0].x, vtx[1].y - vtx[0].y);
			glm::vec2 vpt[4];
			vpt[0] = glm::vec2(pt.x - vtx[2].y, pt.y - vtx[2].y);
			vpt[1] = glm::vec2(pt.x - vtx[0].x, pt.y - vtx[0].y);
			if (glm::dot(vpt[0], vbase[0]) < 0)
				continue;

			if (glm::dot(vpt[0], vbase[1]) < 0)
				continue;

			if (glm::dot(vpt[1], vbase[0]) > 0)
				continue;

			if (glm::dot(vpt[1], vbase[1]) > 0)
				continue;

			handle = i;
			break;
		}

		break;
	default:
		handle =  -1;
		break;
	}

	for (int ilst = 0; ilst < lst.size(); ilst){
		if (lst[ilst] >= iis.size())
			continue;

		iis[lst[ilst]].binlst = false;
	}

	return handle;
}

void c_gl_2d_obj::render()
{
	update_vertices();
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glVertexAttribPointer(posloc, 2, GL_FLOAT, GL_FALSE, sizeof(s_vertex), 0);
	glEnableVertexAttribArray(posloc);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);

	glUniform1i(modeloc, 3);

	for (int ih = 0; ih < iis.size(); ih++){
		if (!iis[ih].bactive || !iis[ih].bvalid)
			continue;
		glUniform4fv(clrloc, 1, glm::value_ptr(iis[ih].clr));
		glUniform1f(depthloc, iis[ih].z);
		if (!iis[ih].bborder){
			glDrawElements(GL_TRIANGLES, prottype.nidxt,
				GL_UNSIGNED_SHORT, (const void*)(sizeof(unsigned short)* iis[ih].idxtoffset));
		}
		else{
			glLineWidth(iis[ih].w);
			glEnable(GL_LINE_SMOOTH);

			glDrawElements(GL_LINE_STRIP, prottype.nidxs,
				GL_UNSIGNED_SHORT, (const void*)(sizeof(unsigned short)* iis[ih].idxsoffset));
		}
	}
}

void c_gl_2d_obj::update_vertices()
{
	s_vertex * vtx = vtxbuf;
	unsigned short * idxt = idxbuf;
	unsigned short * idxs = idxbuf + prottype.nidxt;
	unsigned int vtxoffset = 0;
	unsigned int idxtoffset = 0;
	unsigned int idxsoffset = prottype.nidxt;
	unsigned int nids = prottype.nidxs + prottype.nidxt;
	for (int ih = 0; ih < iis.size(); ih++){
		if (!iis[ih].bvalid || !iis[ih].bactive || iis[ih].bupdated == true){
			vtx += prottype.nvtx;
			vtxoffset += prottype.nvtx;
			idxtoffset += nids;
			idxsoffset += nids;
			idxt += nids;
			idxs += nids;
			continue;
		}

		glm::mat2 sr = iis[ih].sr;
		glm::vec2 pos = iis[ih].pos;
		for (int iv = 0; iv < prottype.nvtx; iv++){
			s_vertex & v = prottype.vtx[iv];
			vtx[iv].x = v.x * sr[0][0] + v.y * sr[0][1] + pos.x;
			vtx[iv].y = v.x * sr[1][0] + v.y * sr[1][1] + pos.y;
		}

		for (int iid = 0; iid < prottype.nidxt; iid++){
			idxt[iid] = prottype.idxt[iid] + vtxoffset;
		}

		for (int iid = 0; iid < prottype.nidxs; iid++){
			idxs[iid] = prottype.idxs[iid] + vtxoffset;
		}

		iis[ih].vtxoffset = vtxoffset;
		iis[ih].idxtoffset = idxtoffset;
		iis[ih].idxsoffset = idxsoffset;
		iis[ih].bupdated = true;

		vtx += prottype.nvtx;
		vtxoffset += prottype.nvtx;
		idxtoffset += nids;
		idxsoffset += nids;
		idxt += nids;
		idxs += nids;
	}
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(s_vertex) * iis.size() * prottype.nvtx, vtxbuf, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(posloc, 2, GL_FLOAT, GL_FALSE, sizeof(s_vertex), 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short)* iis.size() * nids, idxbuf, GL_DYNAMIC_DRAW);
}