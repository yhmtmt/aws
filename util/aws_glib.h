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

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>

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
	const float r, const float g, const float b, const float alpha,
	void* font);

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha, const float size);

void drawGlSquare2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, const float r, const float g, const float b, const float alpha, const float size);

void drawGlTriangle2Df(float x1, float y1, float x2, float y2,
	float x3, float y3, const float r, const float g, const float b, const float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const float r, const float g, const float b, const float alpha, const float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts,
	const float r, const float g, const float b, const float alpha);

void drawGlPolygon2Df(Point2f * pts, int num_pts, const Point2f & offset,
	const float r, const float g, const float b, const float alpha, const float size);

void drawGlPolygon2Df(Point2f * pts, int num_pts, const Point2f & offset,
	const float r, const float g, const float b, const float alpha);


void drawGlLine2Df(float x1, float y1, float x2, float y2,
	const float r, const float g, const float b, const float alpha, const float size);

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


class c_gl_2d_obj
{
private:
	GLuint vao, vbo[2]; // veterx buffer objects: vertex and index

	struct s_vertex{
		float x, y;
	};

	s_vertex * v;

public:
	virtual bool init();

};

class c_gl_point_obj
{
private:
	GLuint vao, vbo;
	int num_total_vertices;
	glm::vec4 clr;
	glm::vec3 t;
	glm::mat4 R;
	float size, szrange[2];
	GLuint modeloc, posloc, Mmvploc, clrloc;

public:
	c_gl_point_obj(): size(1.0){}
	~c_gl_point_obj(){
	}

	bool init(GLuint _modeloc, GLuint posloc, GLuint Mmvploc, GLuint clrloc);

	void set(const int num_points, const float * points);

	void config_color(const glm::vec4 & _clr)
	{
		clr = _clr;
	}

	void config_position(const glm::vec3 & _t)
	{
		t = _t;
	}

	void config_rotation(const glm::mat4 & _R)
	{
		R = _R;
	}

	void config_size(const float _size)
	{
		size = _size;
	}

	void render(const glm::mat4 & PV);

};

class c_gl_line_obj
{
private:
	GLuint vao, vbo;
	struct s_vertex
	{
		float x, y, z;
	};

	unsigned int buffer_size;
	int num_total_vertices;
	s_vertex * vertices;
	glm::vec4 clr;
	glm::vec3 t;
	glm::mat4 R;
	float width;
	float wrange[2];
	GLuint modeloc, posloc, Mmvploc, clrloc;

	struct s_line_buffer_inf
	{
		bool bvalid;
		bool bactive;
		s_vertex * vtx;
		unsigned int offset;
		unsigned int npts;
		s_line_buffer_inf() :offset(0), npts(0), vtx(NULL), bvalid(false), bactive(false)
		{}
	};

	vector<s_line_buffer_inf> lbis;

	bool bupdated;
public:
	c_gl_line_obj();
	~c_gl_line_obj();

	bool init(GLuint modeloc, GLuint posloc, GLuint Mmvploc, GLuint clrloc, unsigned int buffer_size = 4096);
	int set(const int npts, const float * pts);
	void remove(const int handle);

	void enable(const int handle)
	{
		if (handle < lbis.size())
		{
			lbis[handle].bactive = true;
		}
	}

	void disable(const int handle)
	{
		if (handle < lbis.size())
		{
			lbis[handle].bactive = false;
		}
	}

	void config_color(const glm::vec4 & _clr)
	{
		clr = _clr;
	}

	void config_position(const glm::vec3 & _t)
	{
		t = _t;
	}

	void config_rotation(const glm::mat4 & _R)
	{
		R = _R;
	}

	void config_width(const float _width)
	{
		width = _width;
	}

	void update_vertices();

	void render(const glm::mat4 & PV);
};

class c_gl_text_obj
{
public:
	enum e_anchor{
		an_hl = 0x1,
		an_hc = 0x2,
		an_hr = 0x4,
		an_vb = 0x10,
		an_vc = 0x20,
		an_vt = 0x40,
		an_lb = 0x11, an_lc = 0x21, an_lt = 0x41,
		an_cb = 0x12, an_cc = 0x22, an_ct = 0x42,
		an_rb = 0x14, an_rc = 0x24, an_rt = 0x44
	};

private:

	GLuint modeloc, posloc, txcloc, smploc, clrloc;
	GLuint htex;
	GLuint vao, vbo[2];
	enum e_parser_state{
		ps_none,
		ps_index,
		ps_x,
		ps_y,
		ps_w,
		ps_h
	} parser_state;

	struct s_vertex
	{
		float x, y;
		float u, v;
	};

	struct s_string_buffer_inf
	{
		char * str;
		unsigned int offset;
		unsigned int length;
		unsigned int chars;
		e_anchor anch;
		glm::vec2 box[4];
		glm::vec2 t;
		glm::mat2 rot;
		glm::vec2 sz_fnt;
		glm::vec2 sz_str;
		glm::vec2 mgn;
		glm::vec4 clr;
		bool bact;
		bool bvalid;
		bool bupdate;
		s_vertex * vtx;
		unsigned short * idx;
		s_string_buffer_inf() :str(NULL), offset(0), length(0), bact(false), bvalid(false), bupdate(false), 
			vtx(NULL), idx(NULL)
		{}
		~s_string_buffer_inf()
		{
			if (str)
				delete[] str;
			str = NULL;
		}
	};

	vector<s_string_buffer_inf> sbis;
	unsigned int num_vertices, sz_buf, total_str_len;

	s_vertex * vertices; // 4 times the sz_buf
	unsigned short * indices; // 6 times the sz_buf
	void parse_var_name(ifstream & ifile, char * str_var, int & len_str_var);
	int parse_int(ifstream & ifile, char * str_var, int & len_str_var);
	float parse_float(ifstream & ifile, char * str_var, int & len_str_var);

	struct s_texcoord{
		float u, v, w, h;
		s_texcoord() :u(0.f), v(0.f), w(0.f), h(0.f){};
	};

	s_texcoord * texcoord;
	void update_vertices(s_string_buffer_inf & sbi);

public:
	c_gl_text_obj();
	~c_gl_text_obj();

	bool init(const char * ftex, const char * finf, GLuint _modeloc,
		GLuint _posloc, GLuint _txcloc, GLuint _smploc, GLuint _clrloc, unsigned int _sz_buf = 4096);

	const int reserv(const unsigned int len);
	void set(const int handle, const char * str);

	void config(const int handle, const glm::vec4 & clr, const glm::vec2 & sz_fnt,
		const glm::vec2 & mgn, const e_anchor sc,
		const glm::vec2 & t, const float  rot);
	void config_font_size(const int handle, glm::vec2 & sz_fnt);
	void config_font_space(const int handle, const glm::vec2 & mgn);
	void config_color(const int handle, const glm::vec4 & clr);
	void config_position(const int handle, glm::vec2 & t);
	void config_rotation(const int handle, const float rot);
	void config_anchor(const int handle, const e_anchor sc);
	int collision(const glm::vec2 & pt);
	void remove(const int handle);
	void enable(const int handle){
		if (handle < sbis.size())
			sbis[handle].bact = true;
	}
	void disable(const int handle){
		if (handle < sbis.size())
			sbis[handle].bact = false;
	}
	bool render(unsigned int texture_unit);

	void update_vertices(bool bfull = true);
};


#endif
