// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// c_map_obj.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_map_obj.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_map_obj.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include "../stdafx.h"
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <list>
#include <map>
using namespace std;

#include "../../util/aws_stdlib.h"
#include "../../util/aws_thread.h"
#include "../../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include "../../util/aws_glib.h"
#include "../f_aws1_ui.h"


/////////////////////////////////////////////////////////////////// c_map_waypoint_obj
c_map_waypoint_obj::c_map_waypoint_obj() :pocirc(NULL), potxt(NULL)
{
}

bool c_map_waypoint_obj::init(c_gl_2d_obj * _pocirc, c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline,
	const glm::vec4 & _clr, const glm::vec2 & sz_fnt, const float _rmark, const unsigned int _nmaxwps)
{
	nmaxwps = _nmaxwps;
	rmark = _rmark;
	pocirc = _pocirc;
	potxt = _potxt;
	poline = _poline;

	clr = _clr;
	glm::vec2 pos(0.f, 0.f), r(_rmark, _rmark), mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y);
	wps.resize(nmaxwps);
	hmarks.resize(nmaxwps);
	for (int i = 0; i < nmaxwps; i++){
		hmarks[i].hmark = pocirc->add(clr, pos, 0.0f, r);
		pocirc->config_border(hmarks[i].hmark, true, 1.0);
		pocirc->config_depth(hmarks[i].hmark, 0);
		pocirc->disable(hmarks[i].hmark);
		hmarks[i].hstr = potxt->reserv(20);
		potxt->config(hmarks[i].hstr, clr, glm::vec4(0, 0, 0, 0), sz_fnt, mgn_fnt, c_gl_text_obj::an_cb, pos, 0, 0);
		potxt->disable(hmarks[i].hstr);

		float pts[4] = { 0, 0, 0, (float)(rmark * 2.0) };
		hmarks[i].hline_next = poline->add(2, pts);
		poline->config_color(hmarks[i].hline_next, clr);
		poline->config_depth(hmarks[i].hline_next);
		poline->config_position(hmarks[i].hline_next, pos);
		poline->disable(hmarks[i].hline_next);

		hmarks[i].hline_inf = poline->add(2, pts);
		poline->config_color(hmarks[i].hline_inf, clr);
		poline->config_depth(hmarks[i].hline_inf);
		poline->config_position(hmarks[i].hline_inf, pos);
		poline->disable(hmarks[i].hline_inf);
	}
	return true;
}

void c_map_waypoint_obj::update_wps(const int iwp, const s_wp & wp)
{
	if (iwp < nmaxwps){
		wps[iwp] = wp;
	}
}

void c_map_waypoint_obj::update_drawings()
{
  if (mode == ui_mode_sys) {
    for (int iwp = 0; iwp < nmaxwps; iwp++) {
      disable(iwp);
    }
    return;
  }
  
  char buf[20];
  glm::vec2 pos0, pos1;
  int iwp_last = 0;
  for (int iwp = 0; iwp < nmaxwps; iwp++) {
    if (!pocirc->is_enabled(hmarks[iwp].hmark)) {
      break;
    }
    iwp_last = iwp;
    
    s_wp & wp = wps[iwp];
    
    if (mode == ui_mode_map) {
      pos1 = calc_map_pos(wp.rx, wp.ry, wp.rz);
    }
    else if (mode == ui_mode_fpv) {
      glm::vec3 pos_tmp = calc_fpv_pos(wp.rx, wp.ry, wp.rz);
      
      if (pos_tmp.z > 1.0) { // back side
	disable(iwp);
	continue;
      }
      pos1.x = pos_tmp.x;
      pos1.y = pos_tmp.y;
    }
    
    pocirc->config_position(hmarks[iwp].hmark, pos1);
    if (iwp > 0) {
      float x[4] = { pos0.x, pos0.y, pos1.x, pos1.y };
      poline->config_points(hmarks[iwp].hline_next, x);
    }
    else {
      poline->disable(hmarks[iwp].hline_next);
    }
    
    if (iwp == focus) {
      pocirc->config_border(hmarks[iwp].hmark, true, 2.0);
    }
    else {
      pocirc->config_border(hmarks[iwp].hmark, true, 1.0);
    }
    
    glm::vec2 pos_inf = pos1;
    pos_inf.x += (float)(2.0 * rmark);
    pos_inf.y += (float)(2.0 * rmark);
    if (iwp == next) {
      snprintf(buf, 20, "WP%03d\nD%4.0f\nC%3.1f", iwp, dist, crs);
    }
    else {
      snprintf(buf, 20, "WP%03d", iwp);
    }
    potxt->set(hmarks[iwp].hstr, buf);
    potxt->config_position(hmarks[iwp].hstr, pos_inf);
    poline->config_position(hmarks[iwp].hline_inf, pos1);
    
    pos0 = pos1;
  }
}

void c_map_waypoint_obj::enable(const int iwp)
{
  pocirc->enable(hmarks[iwp].hmark);
  poline->enable(hmarks[iwp].hline_next);
  poline->enable(hmarks[iwp].hline_inf);
  potxt->enable(hmarks[iwp].hstr);
}

void c_map_waypoint_obj::disable(const int iwp)
{
  pocirc->disable(hmarks[iwp].hmark);
  poline->disable(hmarks[iwp].hline_next);
  poline->disable(hmarks[iwp].hline_inf);
  potxt->disable(hmarks[iwp].hstr);
}

void c_map_waypoint_obj::disable()
{
  for (int i = 0; i < nmaxwps; i++){
    disable(i);
  }
}

void c_map_waypoint_obj::set_focus(const int iwp)
{
	focus = iwp;
}

void c_map_waypoint_obj::set_next(const int iwp, const float _dist, const float _crs)
{
	next = iwp;
	dist = _dist;
	crs = _crs;
}

int c_map_waypoint_obj::collision(const glm::vec2 pos)
{
  for (int i = 0; i < nmaxwps; i++){
    if (pocirc->is_enabled(hmarks[i].hmark)){
      if (pocirc->collision(pos, hmarks[i].hmark))
	return i;
    }
  }
  return -1;
}

/////////////////////////////////////////////////////////////////// c_ais_obj
bool c_map_ais_obj::init(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline,
	const glm::vec4 & _clr, const glm::vec2 & sz_fnt,
	const glm::vec2 & _sz_rect, const unsigned int _nmax_objs)
{
  nmax_objs = _nmax_objs;
  porect = _porect;
  potri = _potri;
  potxt = _potxt;
  poline = _poline;
  
  sz_rect = _sz_rect;
  clr = _clr;
  glm::vec2 pos(0.f, 0.f), mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y);
  glm::vec2 sz_ship2d(sz_fnt.x, (float)(sz_fnt.y * 0.5));
  
  hmarks.resize(nmax_objs);
  objs.resize(nmax_objs);
  for (int i = 0; i < nmax_objs; i++){
    hmarks[i].hmark = porect->add(clr, pos, 0.0f, sz_rect);
    porect->config_border(hmarks[i].hmark, true, 1.0);
    porect->config_depth(hmarks[i].hmark, 0);
    porect->disable(hmarks[i].hmark);
    
    hmarks[i].hship2d = potri->add(clr, pos, 0.0f, sz_ship2d);
    potri->config_border(hmarks[i].hship2d, false, 1.0);
    potri->config_depth(hmarks[i].hship2d, 1);
    potri->disable(hmarks[i].hship2d);
    
    hmarks[i].hstr = potxt->reserv(64);
    potxt->config(hmarks[i].hstr, clr, glm::vec4(0, 0, 0, 0),
		  sz_fnt, mgn_fnt, c_gl_text_obj::an_lb, pos, 0, 0);
    
    float pts[4] = { 0, 0, 0, (float)(sz_rect.y) };
    hmarks[i].hline_inf = poline->add(2, pts);
    poline->config_color(hmarks[i].hline_inf, clr);
    poline->config_depth(hmarks[i].hline_inf);
    poline->config_position(hmarks[i].hline_inf, pos);
    
    hmarks[i].hline_vel = poline->add(2, pts);
    poline->config_color(hmarks[i].hline_vel, clr);
    poline->config_depth(hmarks[i].hline_vel);
    poline->config_position(hmarks[i].hline_vel, pos);
  }
  
  return true;
}

int c_map_ais_obj::collision(const glm::vec2 pos)
{
  for (int i = 0; i < nmax_objs; i++)
    {
      if (porect->is_enabled(hmarks[i].hmark)){
	if (porect->collision(pos, hmarks[i].hmark))
	  return i;
      }
    }
  return -1;
}

void c_map_ais_obj::update_ais_obj(const int iobj, const c_ais_obj & ais_obj)
{
  if (objs.size() > iobj && iobj >= 0){
    objs[iobj] = ais_obj;
  }
}

void c_map_ais_obj::update_drawings()
{
  if (mode == ui_mode_sys) {
    for (int iobj = 0; iobj < nmax_objs; iobj++) {
      disable(iobj);
    }
    return;
  }
  
  char buf[64];
  for (int iobj = 0; iobj < nmax_objs; iobj++){
    if (!porect->is_enabled(hmarks[iobj].hmark)){
      continue;
    }
    
    c_ais_obj & obj = objs[iobj];
    float x, y, z, rx, ry, rz, rxf, ryf, rzf;
    obj.get_pos_ecef(x, y, z);
    eceftowrld(Rmap, xmap, ymap, zmap, x, y, z, rx, ry, rz);

    unsigned int mmsi = obj.get_mmsi();
    float bear, dist, tcpa, dcpa, nvx, nvy, vxr, vyr, vzr, cog, sog, roll, pitch, yaw;
    obj.get_pos_bd(bear, dist);
    obj.get_tdcpa(tcpa, dcpa);
    obj.get_vel_rel(vxr, vyr, vzr);
    obj.get_vel_vec2d(nvx, nvy);
    obj.get_vel_bih(cog, sog);
    obj.get_att(roll, pitch, yaw);
    
    rxf = rx + vxr * tvel;
    ryf = ry + vyr * tvel;
    rzf = rz + vzr * tvel;
    
    glm::vec2 pos, pos_future;
    if (mode == ui_mode_map){
      potri->config_rotation(hmarks[iobj].hship2d, (float)((90.0f - yaw) * PI / 180.));
      potri->enable(hmarks[iobj].hship2d);
      
      pos = calc_map_pos(rx, ry, rz);
      pos_future = calc_map_pos(rxf, ryf, rzf);
      potri->config_position(hmarks[iobj].hship2d, pos);
    }
    else if (mode == ui_mode_fpv){
      potri->disable(hmarks[iobj].hship2d);
      glm::vec3 pos_tmp = calc_fpv_pos(rx, ry, rz);
      glm::vec3 pos_future_tmp = calc_fpv_pos(rxf, ryf, rzf);
      if (pos_tmp.z > 1.0) {
	disable(iobj);
      }
      pos.x = pos_tmp.x;
      pos.y = pos_tmp.y;
      pos_future.x = pos_future_tmp.x;
      pos_future.y = pos_future_tmp.y;
    }
    glm::vec2 pos_rect((float)(pos.x - 0.5 * sz_rect.x), (float)(pos.y - 0.5 * sz_rect.y));
    glm::vec2 pos_inf = pos;
    pos_inf.y += sz_rect.y;
    
    if (iobj == focus){
      snprintf(buf, 64, "AIS%09u\nD%4.0f V%02.1f\nCPA T%04.0f D%04.0f",
	       mmsi, dist, sog, tcpa, dcpa);
      porect->config_border(hmarks[iobj].hmark, true, 2.0);
    }
    else{
      snprintf(buf, 64, "D%4.0f", dist);
      porect->config_border(hmarks[iobj].hmark, true, 1.0);
    }
    
    potxt->set(hmarks[iobj].hstr, buf);
    potxt->config_position(hmarks[iobj].hstr, pos_inf);
    poline->config_position(hmarks[iobj].hline_inf, pos);
    porect->config_position(hmarks[iobj].hmark, pos_rect);
    
    float pts[4] = { pos.x, pos.y, pos_future.x, pos_future.y };
    poline->config_points(hmarks[iobj].hline_vel, pts);
  }
}

void c_map_ais_obj::enable(const int iobj)
{
  porect->enable(hmarks[iobj].hmark);
  poline->enable(hmarks[iobj].hline_inf);
  poline->enable(hmarks[iobj].hline_vel);
  potxt->enable(hmarks[iobj].hstr);
}

void c_map_ais_obj::disable(const int iobj)
{
  potri->disable(hmarks[iobj].hship2d);
  porect->disable(hmarks[iobj].hmark);
  poline->disable(hmarks[iobj].hline_inf);
  poline->disable(hmarks[iobj].hline_vel);
  potxt->disable(hmarks[iobj].hstr);
}

void c_map_ais_obj::disable()
{
  for (int iobj = 0; iobj < nmax_objs; iobj++)
    {
      disable(iobj);
    }
}

void c_map_ais_obj::set_focus(const int iobj)
{
  focus = iobj;
}

/////////////////////////////////////////////////////////////////// c_map_coast_line_obj
bool c_map_coast_line_obj::init(c_gl_2d_line_obj * _poline, const glm::vec4 & _clr, unsigned int max_num_points)
{
  clr = _clr;
  poline = _poline;
  return true;
}

bool c_map_coast_line_obj::update_points(list<const AWSMap2::LayerData*> & coast_lines)
{
  // clear line object
  for (auto itr = handle.begin(); itr != handle.end(); itr++)
    poline->remove(itr->handle);
  handle.clear();
  
  // add new lines
  struct s_vertex{
    float x, y;
    s_vertex() :x(0), y(0){}
  };
  
  vector<s_vertex> pts;
  vector<bool> bcull; // In fpv mode, we need to cull a line partially.
  
  if (mode == ui_mode_fpv)
    bcull.reserve(128);
  
  pts.reserve(128);
  int index = 0;
  for (auto itr = coast_lines.begin(); itr != coast_lines.end(); itr++){
    const AWSMap2::CoastLine * pcl = dynamic_cast<const AWSMap2::CoastLine*>(*itr);
    unsigned int num_lines = pcl->getNumLines();
    for (unsigned int iline = 0; iline < num_lines; iline++, index++){
      const vector<AWSMap2::vec3> & pts_ecef = pcl->getPointsECEF(iline);
      pts.resize(pts_ecef.size());
      bcull.resize(pts_ecef.size());
      
      for (int ipt = 0; ipt < pts.size(); ipt++){
	const AWSMap2::vec3 & pte = pts_ecef[ipt];
	float rx, ry, rz;
	eceftowrld(Rmap, xmap, ymap, zmap,
		   (const float)pte.x, (const float)pte.y, (const float)pte.z,
		   rx, ry, rz);
	
	glm::vec2 pt;
	if (mode == ui_mode_map){
	  pt = calc_map_pos(rx, ry, rz);
	}
	else if (mode == ui_mode_fpv){
	  glm::vec3 pt_tmp = calc_fpv_pos(rx, ry, rz);
	  pt.x = pt_tmp.x;
	  pt.y = pt_tmp.y;
	  if (pt_tmp.z > 1.0)
	    bcull[ipt] = true;
	  else
	    bcull[ipt] = false;
	}
	pts[ipt].x = pt.x;
	pts[ipt].y = pt.y;
      }
      
      // add lines
      if (mode == ui_mode_map){
	if (!add_new_line(index, pts.size(), (const float*)pts.data()))
	  return false;
      }
      else if (mode == ui_mode_fpv)
	{
	  int start = -1;
	  for (int i = 0; i < pts.size(); i++){
	    if (bcull[i]){
	      if (start >= 0){
		if (!add_new_line(index, i - start, (const float*)(pts.data() + start)))
		  return false;
		start = -1;
	      }
	      continue;
	    }
	    
	    if (start < 0){
	      start = i;
	    }
	  }
	  
	  if (start >= 0)
	    {
	      if (!add_new_line(index, pts.size() - start, (const float*)(pts.data() + start)))
		return false;
	    }
	}
      
    }
  }
  
  return true;
}

/////////////////////////////////////////////////////////////////// c_own_ship
bool c_own_ship::init(c_gl_2d_obj * _potri, c_gl_2d_line_obj * _poline,
	const glm::vec4 & clr, const glm::vec2 & sz)
{
  potri = _potri;
  poline = _poline;
  
  glm::vec2 pos(0, 0);
  hship = potri->add(clr, pos, 0, sz);
  potri->config_depth(hship, 0);
  potri->config_border(hship, true, 1.0);
  
  float pts[4] = { 0, 0, 1, 1 };
  hline_vel = poline->add(2, pts);
  poline->config_color(clr);
  poline->config_depth(hline_vel, 0);
  return true;
}

void c_own_ship::enable()
{
  potri->enable(hship);
  poline->enable(hline_vel);
}

void c_own_ship::disable()
{
  potri->disable(hship);
  poline->disable(hline_vel);
}

void c_own_ship::set_param(const float rx, const float ry, const float rz, const float hdg, const float vx, const float vy, const float pix_per_meter)
{
  float th = (float)((90.f - hdg) * PI / 180.);
  potri->config_rotation(hship, th);
  float pts[4] = {
    (float)(rx * pix_per_meter), (float)(ry * pix_per_meter),
    (float)((rx + vx * tvel) * pix_per_meter), (float)((ry + vy * tvel) * pix_per_meter)
  };
  glm::vec2 pos(pts[0], pts[1]);
  potri->config_position(hship, pos);
  poline->config_points(hline_vel, pts);
}

/////////////////////////////////////////////////////////////////// c_cursor
bool c_cursor::init(c_gl_2d_line_obj * _poline, c_gl_text_obj * _potxt,
	const glm::vec4 & clr, glm::vec2 sz_fnt, glm::vec2 & sz)
{
  poline = _poline;
  potxt = _potxt;
  struct s_vertex{
    float x, y;
  };
  
  
  s_vertex arrow[8] = {
    { (float)(sz.x * 0.25), (float)(sz.y * 0.25) }, { (float)(sz.x * 0.5), (float)(sz.y * 0.5) },
    { (float)(-sz.x * 0.25), (float)(sz.y * 0.25) }, { (float)(-sz.x * 0.5), (float)(sz.y * 0.5) },
    { (float)(-sz.x * 0.25), (float)(-sz.y * 0.25) }, { (float)(-sz.x * 0.5), (float)(-sz.y * 0.5) },
    { (float)(sz.x * 0.25), (float)(-sz.y * 0.25) }, { (float)(sz.x * 0.5), (float)(-sz.y * 0.5) }
	};
  harrow = poline->add(8, (float*)arrow, true);
  poline->config_color(harrow, clr);
  poline->config_depth(harrow, 0);
  poline->config_width(harrow, 1.0);
  
  
  s_vertex pos[3] = {
    { 0, 0 }, { (float)(-sz.x), sz.y }, { (float)(-sz.x * 1.5), sz.y }
  };
  
  hpos = poline->add(3, (float*)pos, false);
  poline->config_color(hpos, clr);
  poline->config_depth(hpos, 0);
  poline->config_width(hpos, 1.0);
  
  hpos_str = potxt->reserv(32);
  pos_str = glm::vec2(pos[2].x, pos[2].y);
  glm::vec2 sz_mgn(sz_fnt.x * 0.6, sz_fnt.y);
  potxt->config(hpos_str, clr, glm::vec4(0, 0, 0, 0),
		sz_fnt, sz_mgn, c_gl_text_obj::an_rc, pos_str, 0, 0);
  
  return true;
}


void c_cursor::set_cursor_position(const glm::vec2 & _pos_mouse, const glm::vec2 & _pos_bih)
{
  glm::vec2 pos = _pos_mouse + pos_str;
  potxt->config_position(hpos_str, pos);
  poline->config_position(hpos, _pos_mouse);
  poline->config_position(harrow, _pos_mouse);
  char buf[64];
  snprintf(buf, 64, "%3.8f\n%3.8f\n", _pos_bih.x * 180.f / PI, _pos_bih.y * 180.0f / PI);
  potxt->set(hpos_str, buf);
}

void c_cursor::enable_arrow()
{
  potxt->disable(hpos_str);
  poline->disable(hpos);
  poline->enable(harrow);
}

void c_cursor::enable_pos()
{
  potxt->enable(hpos_str);
  poline->enable(hpos);
  poline->disable(harrow);
}

void c_cursor::disable()
{
  potxt->disable(hpos_str);
  poline->disable(hpos);
  poline->disable(harrow);
}
