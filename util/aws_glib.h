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

bool load_glsl_program(const char * ffs, const char * fvs, GLuint & p);

class c_gl_obj{
 private:
 public:
  c_gl_obj(){};
  virtual ~c_gl_obj(){};
  virtual size_t get_reserved_resource_size() = 0;
  virtual size_t get_used_resource_size() = 0;
};

class c_gl_radar
{
 private:
  bool benable;
  int spokes, spoke_len_max;
  GLuint modeloc, trnloc, scloc, posloc, txcloc, smploc, clrloc, bkgclrloc, depthloc;
  GLuint htex;
  GLuint vao, vbo[2];
  float zstep, z, scl;
  glm::vec2 pos;
  long long tprev_update;
  int bearing_prev, range_meters;
  struct s_vertex
  {
    float x, y;
    float u, v;
  };
  Mat texture_buffer;
  int num_vertices, num_indices;
  s_vertex * vertices; // built as _spokes * (3(in arc) + 1(in center))
  unsigned short * indices;
  
  glm::vec4 clr, bkgclr;
 public:
 c_gl_radar():benable(false), spokes(0), spoke_len_max(0), htex(0), vao(0), vertices(NULL), indices(NULL), range_meters(1852)
    {
      
  }

  ~c_gl_radar()
    {
      destroy();
    }
  
  bool init(int _spokes, int _spoke_len_max, GLuint _modeloc,
	    GLuint _trnloc, GLuint _scloc,
	    GLuint _posloc, GLuint _txcloc, GLuint _smploc,
	    GLuint _clrloc, GLuint _bkgclrloc, GLuint _depthloc,
	    glm::vec4 & _clr, glm::vec4 & _bkgclr);

  void destroy();

  void set_scale(const float pix_per_meter){
    scl = range_meters * pix_per_meter;
  }

  void set_pos(glm::vec2 & _pos){
    pos = _pos;
  }

  const int get_range_meters()
  {
    return range_meters;
  }

  const long long get_time_last_update()
  {
    return tprev_update;
  }

  const int get_bearing_last_update()
  {
    return bearing_prev;
  }
  
  void set_depth(int depth)
  {
      z = -1.0 + (float)(depth * zstep);
  }
  
  void update_spoke(const long long _t,
		    const double _lat, const double _lon,
		    const int _range_meters,
		    const int _bearing, const int _len,
		    const unsigned char * _line);
  void update_spokes(const long long _t,
		     const int _range_meters,
		     const int _bearing_from, const int _bearing_to,
		     const unsigned char * _lines);
  void render();

  void enable()
  {
    benable=true;
  }
  
  void disable()
  {
    benable=false;
  }
};

class c_gl_text_obj: public c_gl_obj
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
  bool bupdated;
  GLuint modeloc, trnloc, scloc,  posloc, txcloc, smploc, clrloc, bkgclrloc, depthloc;
  GLuint htex;
  GLuint vao, vbo[2];

  float zstep;
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
    float z;
    glm::vec2 sz_fnt;
    glm::vec2 sz_str;
    glm::vec2 mgn;
    glm::vec4 clr, bkgclr;
    bool bact;
    bool bvalid;
    bool bupdate;
    s_vertex * vtx;
    unsigned short * idx;
    s_string_buffer_inf() :str(NULL), offset(0), length(0), bact(false), bvalid(false), bupdate(false),
      vtx(NULL), idx(NULL), z(-1.0)
    {}
    ~s_string_buffer_inf()
    {
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
  virtual ~c_gl_text_obj();

  bool init(const char * ftex, const char * finf, GLuint _modeloc,
	    GLuint _trnloc, GLuint _scloc,
    GLuint _posloc, GLuint _txcloc, GLuint _smploc,
    GLuint _clrloc, GLuint _bkgclrloc, GLuint _depthloc,
    unsigned int _sz_buf = 4096);

  void destroy();

  const int reserv(const unsigned int len);
  void set(const int handle, const char * str);

  void config(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr, const glm::vec2 & sz_fnt,
    const glm::vec2 & mgn, const e_anchor sc,
    const glm::vec2 & t, const float  rot, const int depth = 0);
  void config_font_size(const int handle, const glm::vec2 & sz_fnt);
  void config_font_space(const int handle, const glm::vec2 & mgn);
  void config_color(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr);

  void config_position(const int handle, glm::vec2 & t);
  void config_rotation(const int handle, const float rot);
  void config_rotation(const int handle, const float c, const float s);
  void config_anchor(const int handle, const e_anchor sc);
  void config_depth(const int handle, const int depth);

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

  virtual size_t get_reserved_resource_size()
  {
    return (size_t) sz_buf;
  }
  
  virtual size_t get_used_resource_size()
  {
    int num_chars = 0;
    for(int i = 0; i < sbis.size(); i++){
      if(sbis[i].bvalid)
	num_chars += sbis[i].length;
    }
    return (size_t) num_chars;
  }
};

class c_gl_line_obj: public c_gl_obj
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
  float wrange[2];
  GLuint modeloc, posloc, Mmvploc, clrloc;

  struct s_line_buffer_inf
  {
    bool bvalid;
    bool bactive;
    s_vertex * vtx;
    unsigned int offset;
    unsigned int npts;
    float w;
    glm::vec4 clr;
    glm::vec3 t;
    glm::mat4 R;

    s_line_buffer_inf() :offset(0), npts(0), vtx(NULL),
      bvalid(false), bactive(false), w(1.f), t(0,0,0), R(1.0)
    {}
  };

  vector<s_line_buffer_inf> lbis;

  bool bupdated;
public:
  c_gl_line_obj();
  virtual ~c_gl_line_obj();

  bool init(GLuint modeloc, GLuint posloc, GLuint Mmvploc, GLuint clrloc,
    unsigned int buffer_size = 4096);
  int add(const int npts, const float * pts);

  void destroy();

  void clear();

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
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].clr = _clr;
    }
  }

  void config_color(const int handle, const glm::vec4 & _clr)
  {
    if (handle >= lbis.size())
      return;

    lbis[handle].clr = _clr;
  }

  void config_points(const int handle, const float * pts);

  void config_position(const glm::vec3 & _t)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].t = _t;
    }
  }

  void config_position(const int handle, const glm::vec3 & _t)
  {
    if (handle >= lbis.size())
      return;

    lbis[handle].t = _t;
  }

  void config_rotation(const glm::mat4 & _R)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].R = _R;
    }
  }

  void config_rotation(const int handle, const glm::mat4 & _R)
  {
    if (handle >= lbis.size())
      return;

    lbis[handle].R = _R;
  }

  void config_width(const float _width)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].w = _width;
    }
  }

  void config_width(const int handle, const float _width)
  {
    if (lbis.size() >= handle)
      return;

    lbis[handle].w = _width;
  }

  void update_vertices();

  void render(const glm::mat4 & PV);
  virtual size_t get_reserved_resource_size()
  {
    return (size_t) buffer_size;
  }
  
  virtual size_t get_used_resource_size()
  {
    return (size_t) num_total_vertices;
  }
};

class c_gl_point_obj: public c_gl_obj
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
  c_gl_point_obj() : vao(0), size(1.0){}
  virtual ~c_gl_point_obj(){
    destroy();
  }

  bool init(GLuint _modeloc, GLuint posloc, GLuint Mmvploc, GLuint clrloc);
  void destroy();

  void add(const int num_points, const float * points);

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
  virtual size_t get_reserved_resource_size()
  {
    return 0;
  }
  virtual size_t get_used_resource_size()
  {
    return 0;
  }
};

class c_gl_2d_line_obj: public c_gl_obj
{
  GLuint vao, vbo;
  struct s_vertex
  {
    float x, y;
  };

  unsigned int buffer_size;
  int num_total_vertices;
  s_vertex * vertices, *vertices_trn;
  float wrange[2];
  GLuint modeloc, trnloc, scloc, posloc, clrloc, depthloc;
  float zstep;

  struct s_line_buffer_inf
  {
    bool bvalid;
    bool bactive;
    bool bupdated;
    bool blines;
    s_vertex * vtx;
    s_vertex * vtx_trn;
    float w;
    float z;
    float rot;
    glm::vec4 clr;
    glm::vec2 t;
    glm::mat2 R;
    unsigned int offset;
    unsigned int npts;
  s_line_buffer_inf() :offset(0), npts(0), vtx(NULL), bvalid(false), bactive(false), bupdated(false), w(1.0), z(1.0), rot(0.f), t(0,0)
    {}
  };

  vector<s_line_buffer_inf> lbis;

  bool bupdated;
public:
  c_gl_2d_line_obj();
  virtual ~c_gl_2d_line_obj();

  bool init(GLuint modeloc, GLuint trnloc, GLuint scloc, GLuint posloc, GLuint clrloc, GLuint _depthloc,
    unsigned int buffer_size = 4096);
  int add(const int npts, const float * pts, bool blines = false);

  void destroy();

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
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].clr = _clr;
    }
  }

  void config_color(const int handle, const glm::vec4 & _clr)
  {
    if (handle >= lbis.size())
      return;

    lbis[handle].clr = _clr;
  }

  void config_points(const int handle, const float * pts);

  void config_position(const glm::vec2 & _t)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].t = _t;
      lbis[ih].bupdated = false;
    }
    bupdated = false;
  }

  void config_position(const int handle, const glm::vec2 & _t)
  {
    if (handle >= lbis.size())
      return;

    lbis[handle].t = _t;
    lbis[handle].bupdated = false;
    bupdated = false;
  }

  void config_rotation(const float & _rot)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].rot = _rot;
      float s = (float)(sin(lbis[ih].rot)), c = (float)(cos(lbis[ih].rot));
      lbis[ih].R = glm::mat2(c, -s, s, c);
      lbis[ih].bupdated = false;
    }
    bupdated = false;
  }

  void config_rotation(const int handle, const float & _rot)
  {
    if (handle >= lbis.size())
      return;
    lbis[handle].rot = _rot;
    float s = (float)(sin(lbis[handle].rot)), c = (float)(cos(lbis[handle].rot));
    lbis[handle].R = glm::mat2(c, -s, s, c);
    lbis[handle].bupdated = false;
    bupdated = false;
  }

  void config_rotation(const int handle, const float & rot, const float & c, const float s)
  {
    if (handle >= lbis.size())
      return;
    lbis[handle].rot = rot;
    lbis[handle].R = glm::mat2(c, -s, s, c);
    lbis[handle].bupdated = false;

    bupdated = false;
  }

  void config_width(const float _width)
  {
    for (int ih = 0; ih < lbis.size(); ih++){
      lbis[ih].w = _width;
    }
  }

  void config_width(const int handle, const float _width)
  {
    if (lbis.size() >= handle)
      return;

    lbis[handle].w = _width;
  }

  void config_depth(const int handle, const int depth = 0);

  void update_vertices();

  void render();

  virtual size_t get_reserved_resource_size(){
    return (size_t) buffer_size;
  }
  virtual size_t get_used_resource_size(){
    return (size_t) num_total_vertices;
  }
};

class c_gl_2d_obj: c_gl_obj
{
public:
  enum e_type{
    custom,
    circle,
    rectangle
  } type;

private:
  GLuint vao, vbo[2]; // veterx buffer objects: vertex and index
  GLuint modeloc, trnloc, scloc, posloc, clrloc, depthloc;
  float zstep;
  struct s_vertex{
    float x, y;
  };

  struct s_prottype{
    unsigned int nvtx, nidxt, nidxs;
    s_vertex * vtx;
    glm::vec2 r;
    unsigned short *idxt; // for triangles
    unsigned short *idxs; // for shape
    s_prottype() :vtx(NULL), idxt(NULL), idxs(NULL){}
    ~s_prottype()
    {
      destroy();
    }
    void destroy()
    {
      if (vtx)
        delete[] vtx;
      if (idxs)
        delete[] idxs;
      vtx = NULL;
      idxt = idxs = NULL;
    }
  } prottype;

  unsigned int buffer_size;
  s_vertex * vtxbuf;
  unsigned short * idxbuf;

  struct s_inst_inf{
    bool bvalid, bactive, bupdated, bborder, binlst;
    int order;
    float rot;
    glm::vec2 scale, relps2inv /* inverse squared radius of ellipsoid */;
    glm::vec4 clr; // color
    glm::vec2 pos; // position
    glm::mat2 r;
    glm::mat2 sr;  // scale and rotation
    float w;		// border width
    float z;		// depth
    unsigned int vtxoffset;
    unsigned int idxtoffset, idxsoffset;
    s_inst_inf() :bvalid(false), bactive(false), bupdated(false), bborder(false), binlst(false), z(1.), w(1.0)
    {}
  };

  vector<s_inst_inf> iis;
  vector<int> iis_sorted_by_depth;
  void reorder(const int handle);
public:
  c_gl_2d_obj();
  virtual ~c_gl_2d_obj();

  void destroy();

  void enable(const int handle)
  {
    if (handle < iis.size())
    {
      iis[handle].bactive = true;
    }
  }

  bool is_enabled(const int handle)
  {
    if (handle < iis.size())
    {
      return iis[handle].bactive;
    }
    return false;
  }

  void disable(const int handle)
  {
    if (handle < iis.size())
    {
      iis[handle].bactive = false;
    }
  }

  bool init(GLuint _modeloc, GLuint _trnloc, GLuint _scloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
    const unsigned int npts, const float * points,
    const unsigned int nids, const unsigned short * indices,
    const unsigned int buffer_size = 64);
  
  bool init_rectangle(GLuint _modeloc, GLuint _trnloc, GLuint _scloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
    const glm::vec2 & plb, glm::vec2 & sz, const unsigned int buffer_size = 64);
  bool init_circle(GLuint _modeloc, GLuint _trnloc, GLuint _scloc, GLuint _posloc, GLuint _clrloc, GLuint _depthloc,
    const unsigned int npts, const float rx, const float ry, const unsigned int buffer_size = 64);

  int add(const glm::vec4 & clr, const glm::vec2 & pos, const float rot = 0.0, const float scale = 1.0);
  int add(const glm::vec4 & clr, const glm::vec2 & pos, const float rot = 0.0, const glm::vec2 & scale = glm::vec2(1.0, 1.0));

  void remove(const int handle);

  void config_color(const int handle, const glm::vec4 & clr);
  void config_position(const int handle, const  glm::vec2 & pos);
  void config_rotation(const int handle, const float rot);
  void config_scale(const int handle, const float scl);
  void config_scale(const int handle, const glm::vec2 & scl);
  void config_border(const int handle, const bool b, const float w);
  void config_depth(const int handle, const int depth = 0);

  int collision(const glm::vec2 & pt, const vector<int> & lst = vector<int>());
  bool collision(const glm::vec2 & pt, const int handle);
  void render();
  void update_vertices();

  virtual size_t get_reserved_resource_size()
  {
    return (size_t) buffer_size; 
  }
  virtual size_t get_used_resource_size()
  {
    return (size_t) iis.size();
  }
  
};

#endif
