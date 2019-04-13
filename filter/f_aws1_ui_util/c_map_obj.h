// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// c_map_obj.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_map_obj.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_map_obj.h.  If not, see <http://www.gnu.org/licenses/>. 

class c_map_obj
{
protected:
  static int display_depth;
  e_ui_mode mode;
  glm::mat4 pv;
  glm::vec2 sz_scrn;
  float pix_per_meter;
  float meter_per_pix;
  Mat Rmap;
  float xmap, ymap, zmap;
  
  glm::vec3 calc_fpv_pos(const float rx, const float ry, const float rz)
    {
      glm::vec3 pos;
      glm::vec4 x(rx, ry, rz, 1), xprj;
      xprj = pv * x;
      float iw = (float)(1.0 / xprj.w);
      pos.x = xprj.x * iw * sz_scrn.x;
      pos.y = xprj.y * iw * sz_scrn.y;
      pos.z = xprj.z * iw;
      return pos;
    }
  
  glm::vec2 calc_map_pos(const float rx, const float ry, const float rz)
    {
      glm::vec2 pos;
      pos.x = pix_per_meter * rx;
      pos.y = pix_per_meter * ry;
      return pos;
    }
 public:
 c_map_obj() :mode(ui_mode_fpv)
  {
  }
  
  virtual int collision(const glm::vec2 pos) = 0;
  
  void set_ui_mode(e_ui_mode _mode = ui_mode_fpv)
  {
    mode = _mode;
  }
  
  void set_fpv_param(
		     const glm::mat4 & _pv /* camera projection x camera rotation and translation*/,
		     const glm::vec2 & _sz_scrn)
  {
    pv = _pv;
    sz_scrn.x = (float)(_sz_scrn.x * 0.5);
    sz_scrn.y = (float)(_sz_scrn.y * 0.5);
  }
  
  void set_map_param(const float _pix_per_meter, const Mat & Rorg, const float xorg, const float yorg, const float zorg)
  {
    pix_per_meter = _pix_per_meter;
    meter_per_pix = (float)(1.0 / pix_per_meter);
    Rmap = Rorg;
    xmap = xorg;
    ymap = yorg;
    zmap = zorg;
  }
};

class c_map_waypoint_obj : public c_map_obj
{
private:
  c_gl_2d_obj * pocirc;
  c_gl_text_obj * potxt;
  c_gl_2d_line_obj * poline;
  c_gl_line_obj * poline3d;
  glm::vec4 clr;
  vector<s_wp> wps;
  struct s_marker{
    int hmark, hstr, hline_inf, hline_next_3d;
  };
  vector<s_marker> hmarks;
  int focus, next;
  float dist, crs, xerr;
  int nmaxwps;
  float rmark;
 public:
  c_map_waypoint_obj();
  bool init(c_gl_2d_obj * pocirc, c_gl_text_obj * potxt, c_gl_2d_line_obj * poline,
	    c_gl_line_obj * poline3d, 
	    const glm::vec4 & clr, const glm::vec2 & sz_fnt, const float _rmark,
	    const unsigned int _nmaxwps = 100);
  void update_wps(const int iwp, const s_wp & wp);
  void update_drawings();
  void enable(const int iwp);
  void disable(const int iwp);
  void disable();
  void set_focus(const int iwp);
  void set_next(const int iwp, const float dist, const float crs, const float xerr);
  virtual int collision(const glm::vec2 pos);
};

class c_map_ais_obj : public c_map_obj
{
private:
  c_gl_2d_obj * porect, *potri;
  c_gl_text_obj * potxt;
  c_gl_2d_line_obj * poline;
  
  glm::vec2 sz_rect;
  glm::vec4 clr;
  struct s_marker{
    int hmark, hship2d, hstr, hline_vel;
  };
  vector<s_marker> hmarks;
  vector<c_ais_obj> objs;
  int nmax_objs;
  int focus;
  float tvel;
 public:
 c_map_ais_obj() : tvel(300)
	{
	}
  
  bool init(c_gl_2d_obj * porect, c_gl_2d_obj * potri, c_gl_text_obj * potxt, c_gl_2d_line_obj * poline,
	    const glm::vec4 & clr, const glm::vec2 & sz_fnt, const glm::vec2 & sz_rect,
	    const unsigned int _nmax_objs = 100);
  void update_ais_obj(const int iobj, const c_ais_obj & ais_obj);
  void update_drawings();
  void enable(const int iobj);
  void disable(const int iobj);
  void disable();
  void set_focus(const int iobj);
  void set_vel_len(const float t = 300){ tvel = t; };
  virtual int collision(const glm::vec2 pos);
};

class c_map_coast_line_obj : public c_map_obj
{
private:
  c_gl_line_obj * poline;
  glm::vec4 clr;
  struct s_line{
    int index;
    int handle;
  s_line(const int _index, const int _handle) : index(_index), handle(_handle)
    {}
  };
  
  vector<s_line> handle;
  
  bool add_new_line(int index, int npts, const float * pts)
  {
    int h = poline->add(npts, pts);
    if (h < 0)
      return false;
    handle.push_back(s_line(index, h));
    poline->config_width(h, 1.0);
    poline->config_color(h, clr);
    return true;
  }
 public:
  c_map_coast_line_obj()
    {
    }
  
  virtual ~c_map_coast_line_obj()
    {
    }
  
  bool init(c_gl_line_obj * poline, const glm::vec4 & clr,
	    const unsigned int max_num_points);
  bool update_points(list<AWSMap2::LayerDataPtr> & coast_lines);
  
  void update_drawings();
  virtual int collision(const glm::vec2 pos)
  {
    return -1;
  }
};

class c_own_ship: public c_map_obj
{
private:
  c_gl_2d_obj * potri;
  c_gl_2d_obj * pocirc;
  c_gl_2d_line_obj * poline;
  c_gl_text_obj * potxt;
  
  int hship, hline_vel, hstay_point, hstay_line ;

  float radius;
  int hbearing, hbearing_tgt;
  float tvel;
public:
 c_own_ship() :tvel(300)
    {      
    }
  
  virtual int collision(const glm::vec2 pos)
  {
    potri->collision(pos, hship);
  }
  
  bool init(c_gl_2d_obj * _potri, c_gl_2d_line_obj * _poline,
	    c_gl_2d_obj * _pocirc, c_gl_text_obj * _potxt,
	    const glm::vec4 & clr, const glm::vec2 & sz);
  void set_param(const float rx, const float ry, const float rz,
		 const float rxs, const float rys, const float rzs,
		 const float hdg, const float vx, const float vy,
		 const float cog_tgt);

  void set_vel_len(const float t = 300) {
    tvel = t;
  }
  void enable();
  void disable();
};

class c_cursor
{
private:
	c_gl_2d_line_obj * poline;
	c_gl_text_obj * potxt;
	int harrow, hpos, hpos_str;
	glm::vec2 pos_str;
public:
	bool init(c_gl_2d_line_obj * poline, c_gl_text_obj * potxt, const glm::vec4 & clr, glm::vec2 sz_fnt, glm::vec2 & sz);
	void set_cursor_position(const glm::vec2 & _pos_mouse, const glm::vec2 & _pos_bih);
	void enable_arrow();
	void enable_pos();
	void disable();
};

