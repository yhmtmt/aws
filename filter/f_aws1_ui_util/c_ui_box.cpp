// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// c_ui_box.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_ui_box.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_ui_box.cpp.  If not, see <http://www.gnu.org/licenses/>. 

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

/////////////////////////////////////////////////////////////////// c_aws_ui_button
c_gl_2d_obj * c_aws_ui_button::porect = NULL;
c_gl_text_obj * c_aws_ui_button::potxt = NULL;

bool c_aws_ui_button::init(const glm::vec2 & _pos, const glm::vec2 _sz, const unsigned int _txt_len,
  const glm::vec4 & _clr, const glm::vec4 & _bkgclr)
{
  clr = _clr;
  bkgclr = _bkgclr;
  sz = _sz;

  hbox = porect->add(clr, _pos, 0, _sz);
  porect->config_depth(hbox, 1);
  htxt = potxt->reserv(_txt_len);
  potxt->config_depth(htxt, 0);

  glm::vec2 sz_fnt;
  sz_fnt.x = (float)(0.8 * _sz.x / (double)_txt_len);
  sz_fnt.y = (float)(0.8 * _sz.y);
  if (sz_fnt.x < sz_fnt.y)
    sz_fnt.y = sz_fnt.x;
  else
    sz_fnt.x = sz_fnt.y;

  glm::vec2 pos_txt((float)(_pos.x + _sz.x * 0.5), (float)(_pos.y + _sz.y * 0.5));
  potxt->config(htxt, clr, bkgclr, sz_fnt, sz_fnt, c_gl_text_obj::an_cc, pos_txt, 0);

  set_normal();

  return true;
}

void c_aws_ui_button::set_position(const glm::vec2 & _pos)
{
  porect->config_position(hbox, _pos);
  glm::vec2 pos_txt((float)(_pos.x + sz.x * 0.5), (float)(_pos.y + sz.y * 0.5));
  potxt->config_position(htxt, pos_txt);
}

void c_aws_ui_button::set_text(const char * _text)
{
  potxt->set(htxt, _text);
}

void c_aws_ui_button::set_visible()
{
  porect->enable(hbox);
  potxt->enable(htxt);
}

void c_aws_ui_button::set_invisible()
{
  porect->disable(hbox);
  potxt->disable(htxt);
}

void c_aws_ui_button::set_select()
{
  glm::vec4 sel_clr(clr.x * 0.5, clr.y * 0.5, clr.z * 0.5, clr.w * 0.5);
  porect->config_color(hbox, sel_clr);
  porect->config_border(hbox, false, 1.0);
  potxt->config_color(htxt, clr, bkgclr);
  state = es_select;
}

void c_aws_ui_button::set_check()
{
  glm::vec4 txt_clr(0, 0, 0, 1);

  porect->config_color(hbox, clr);
  porect->config_border(hbox, false, 1.0);
  potxt->config_color(htxt, txt_clr, bkgclr);
  state = es_check;
}

void c_aws_ui_button::set_normal()
{
  porect->config_color(hbox, clr);
  porect->config_border(hbox, true, 1.0);
  potxt->config_color(htxt, clr, bkgclr);
  state = es_normal;
}

void c_aws_ui_button::set_disable()
{
  porect->config_color(hbox, clr);
  porect->config_border(hbox, true, 1.0);
  glm::vec4 txt_clr(clr.r * 0.3, clr.g * 0.3, clr.b * 0.3, clr.a);
  potxt->config_color(htxt, clr, bkgclr);
  state = es_disable;
}

/////////////////////////////////////////////////////////////////// c_aws_ui_box_manager
c_aws_ui_box_manager::c_aws_ui_box_manager() : box_pushed(nul), box_released(nul),
					       blpushed(false), blreleased(false), box_updated(nul)
{
}

c_aws_ui_box_manager::~c_aws_ui_box_manager()
{
  if (pboxes.size()){
    for (int ibox = 0; ibox < pboxes.size(); ibox++)
      delete pboxes[ibox];
    pboxes.clear();
    hboxes.clear();
  }
}

bool c_aws_ui_box_manager::init(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
				c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline,
				const glm::vec4 & _clr, const glm::vec4 & _bkgclr, const glm::vec2 & _sz_font,
				const float fovx, const glm::vec2 & _sz_screen)
{
  clr = _clr;
  bkgclr = _bkgclr;
  sz_font = _sz_font;
  sz_screen = _sz_screen;
  porect = _porect;
  potri = _potri;
  potxt = _potxt;
  poline = _poline;
  c_aws_ui_box::set_gl_objs(porect, potri, potxt, poline);
  
  {
    glm::vec2 lb(0, 0);
    glm::vec2 scale(0, 0);
    porect->add(clr, lb, 0, scale);
    porect->add(clr, lb, 0, scale);
    porect->add(clr, lb, 0, scale);
    porect->add(clr, lb, 0, scale);
  }
  
  pboxes.resize(4, NULL);
  hboxes.resize(4);
  
  pboxes[view_mode] = ((c_aws_ui_box*) new c_view_mode_box());
  pboxes[ctrl_mode] = ((c_aws_ui_box*) new c_ctrl_mode_box());
  pboxes[map_cfg] = ((c_aws_ui_box*) new c_map_cfg_box());
  pboxes[route_cfg] = ((c_aws_ui_box*) new c_route_cfg_box());
  hboxes[view_mode] = 0;
  hboxes[ctrl_mode] = 1;
  hboxes[map_cfg] = 2;
  hboxes[route_cfg] = 3;

  float xmax = (float)(0.5 * sz_screen.x), xmin = -xmax,
    ymax = (float)(0.5 * sz_screen.y), ymin = -ymax;
  float y;
  
  // view_mode box and ctrl_mode box are placed at the left top side.
  y = (float)(ymax - pboxes[view_mode]->get_box_size(sz_font).y);
  pboxes[view_mode]->init(view_mode, clr, bkgclr, sz_font, sz_screen, y, true);
  y -= pboxes[ctrl_mode]->get_box_size(sz_font).y;
  pboxes[ctrl_mode]->init(ctrl_mode, clr, bkgclr, sz_font, sz_screen, y, true);
  
  // map_cfg box and route_cfg box are placed at the right top side.
  y = (float)(ymax - pboxes[map_cfg]->get_box_size(sz_font).y);
  pboxes[map_cfg]->init(map_cfg, clr, bkgclr, sz_font, sz_screen, y, false);
  y -= pboxes[route_cfg]->get_box_size(sz_font).y;
  pboxes[route_cfg]->init(route_cfg, clr, bkgclr, sz_font, sz_screen, y, false);
  
  return true;
}

bool c_aws_ui_box_manager::set_mouse_event(const glm::vec2 & pt,
	const int button, const int action, const int modifier)
{
  // asserting flags in the boxes mouse event happened.
  bool handled = false; // this flag is the return value, if the event is handled on the boxes the manager hold.
  if (button == GLFW_MOUSE_BUTTON_LEFT){
    if (action == GLFW_PRESS){
      for (int ibox = 0; ibox < nul; ibox++){
	if (porect->collision(pt, hboxes[ibox])){
	  box_pushed = (e_box)ibox;
	  pboxes[box_pushed]->set_mouse_event(pt, button, action, modifier);
	  handled = true;
	  blpushed = true;
	}
      }
    }
    
    if (action == GLFW_RELEASE){
      blreleased = true;
      for (int ibox = 0; ibox < nul; ibox++){
	if (porect->collision(pt, hboxes[ibox])){
	  box_released = (e_box)ibox;
	  if (box_pushed != nul)
	    pboxes[box_pushed]->set_mouse_event(pt, button, action, modifier);
	  else
	    pboxes[box_released]->set_mouse_event(pt, button, action, modifier);
	  handled = true;
				}
      }
    }
  }
  
  // process the mouse event
  if (blpushed){
    if (box_pushed != nul){
      if (pboxes[box_pushed]->proc(blpushed, blreleased))
	box_updated = box_pushed;
    }
  }
  else{
    if (box_released != nul){
      if (pboxes[box_released]->proc(blpushed, blreleased))
	box_updated = box_released;
    }
  }
  
  if (blreleased){
    box_pushed = nul;
    box_released = nul;
    blreleased = false;
    blpushed = false;
  }
  
  return handled;
}


/////////////////////////////////////////////////////////////////////////////// ui controls

///////////////////////////////////////////////////////////////////////// c_aws_ui_box
c_gl_2d_obj * c_aws_ui_box::porect = NULL;
c_gl_2d_obj * c_aws_ui_box::potri = NULL;
c_gl_2d_line_obj * c_aws_ui_box::poline = NULL;
c_gl_text_obj * c_aws_ui_box::potxt = NULL;

void c_aws_ui_box::set_gl_objs(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline)
{
	porect = _porect;
	potri = _potri;
	potxt = _potxt;
	poline = _poline;
}

void c_aws_ui_box::add_btn(int & hbtn, int & hstr, const char * str,
	const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_fnt)
{
	hbtn = porect->add(clr, pos, 0, sz_btn);
	porect->config_border(hbtn, true, 1.0f);
	porect->config_depth(hbtn, 1);

	hstr = potxt->reserv((unsigned int)(strlen(str) + 1));
	potxt->set(hstr, str);
	glm::vec2 pos_txt((float)(pos.x + sz_btn.x * 0.5), (float)(pos.y + sz_btn.y * 0.5));
	potxt->config(hstr, clr, bkgclr, sz_fnt, sz_fnt, c_gl_text_obj::an_cc, pos_txt, 0.0);
	potxt->config_depth(hstr, 0);

	set_normal_color(hbtn, hstr);
}

void c_aws_ui_box::add_select_box(int & hlbtn, int & hlstr, const char * lstr,
	int & hrbtn, int & hrstr, const char * rstr, int & hvalstr,
	const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_box,
	const glm::vec2 & sz_fnt, const unsigned int len_str)
{
	glm::vec2 pos_btn;
	pos_btn.x = pos.x;
	pos_btn.y = pos.y;
	add_btn(hlbtn, hlstr, lstr, pos_btn, sz_btn, sz_fnt);

	pos_btn.x += (float)(sz_box.x * 0.5);
	pos_btn.y += (float)(sz_fnt.y * 0.5);
	hvalstr = potxt->reserv(len_str);
	glm::vec2 mgn_fnt((float)(sz_fnt.x * 0.5), sz_fnt.y);

	potxt->config(hvalstr, clr, bkgclr, sz_fnt, mgn_fnt, c_gl_text_obj::an_cc, pos_btn, 0.0);

	pos_btn.x = (float)(pos.x + sz_box.x - sz_btn.x);
	pos_btn.y = pos.y;
	add_btn(hrbtn, hrstr, rstr, pos_btn, sz_btn, sz_fnt);
}

void c_aws_ui_box::setup_frame(const float y, const bool left,
	const glm::vec2 & sz_scrn, const glm::vec2 &sz_box,
	const glm::vec2  & sz_fnt, const glm::vec4 & clr)
{
	float xmax = (float)(0.5 * sz_scrn.x), xmin = (float)(-xmax),
		ymax = (float)(0.5 * sz_scrn.y), ymin = (float)(-ymax);

	sz_close.x = sz_fnt.x;
	sz_close.y = sz_box.y;
	sz_open.x = (float)(sz_box.x + sz_close.x);
	sz_open.y = sz_box.y;
	pos_open.x = (left ? xmin : (float)(xmax - sz_open.x));
	pos_open.y = y;
	pos_close.x = (left ? xmin : (float)(xmax - sz_close.x));
	pos_close.y = y;

	glm::vec2 pos;
	pos.y = (float)(y + sz_box.y * 0.5);
	pos.x = (left ? (float)(xmin + sz_box.x + 0.5 * sz_fnt.x) : (float)(xmax - sz_box.x - 0.5 * sz_fnt.x));
	hopen = potri->add(clr, pos, (left ? (float)(PI) : 0.0f), (float)(sz_fnt.x * 0.5));
	pos.x = (left ? (float)(xmin + 0.5 * sz_fnt.x) : (float)(xmax - 0.5 * sz_fnt.x));
	hclose = potri->add(clr, pos, (left ? 0.0f : (float)(PI)), (float)(sz_fnt.x * 0.5));

	// appearance setting
	porect->config_border(hbox, true, 1.0);
	porect->config_color(hbox, clr);
	porect->config_depth(hbox, 2);

	potri->config_border(hopen, false, 1.0);
	potri->config_color(hopen, clr);
	potri->config_depth(hopen, 1);

	potri->config_border(hclose, false, 1.0);
	potri->config_color(hclose, clr);
	potri->config_depth(hclose, 1);
}

bool c_aws_ui_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		if (potri->collision(pt, hopen)){
			btn_oc_pushed = true;
		}
	}
	else{
		if (potri->collision(pt, hclose)){
			btn_oc_pushed = true;
		}
	}
	return btn_oc_pushed;
}

bool c_aws_ui_box::handle_left_release(const glm::vec2 & pt)
{
	pt_mouse = pt;
	if (bopened){
		if (potri->collision(pt, hopen)){
			btn_oc_released = true;
		}
	}
	else{
		if (potri->collision(pt, hclose)){
			btn_oc_released = true;
		}
	}

	return btn_oc_released;
}

void c_aws_ui_box::open()
{
	potri->enable(hopen);
	potri->disable(hclose);
	porect->config_position(hbox, pos_open);
	porect->config_scale(hbox, sz_open);
}

void c_aws_ui_box::close()
{
	potri->enable(hclose);
	potri->disable(hopen);
	porect->config_position(hbox, pos_close);
	porect->config_scale(hbox, sz_close);
}

bool c_aws_ui_box::proc(const bool bpushed, const bool breleased)
{
	if (bpushed){
		if (btn_oc_pushed){
			if (breleased){
				if (btn_oc_released){
					bopened = !bopened;
					if (bopened){
						open();
					}
					else{
						close();
					}
				}
				set_normal_color(hbox);
				btn_oc_pushed = btn_oc_released = false;
				return true;
			}
			else{
				set_selected_color(hbox);
				return true;
			}
		}
	}
	else{
		if (breleased && btn_oc_released){
			bopened = !bopened;
			if (bopened){
				open();
			}
			else{
				close();
			}

			set_normal_color(hbox);
			btn_oc_pushed = btn_oc_released = false;
		}
	}
	return false;
}

void c_aws_ui_box::set_selected_color(const int hrect)
{
	glm::vec4 box_clr(clr.x * 0.5, clr.y * 0.5, clr.z * 0.5, clr.w * 0.5);
	porect->config_color(hrect, box_clr);
	porect->config_border(hrect, false, 1.0);
}

void c_aws_ui_box::set_normal_color(const int hrect)
{
	glm::vec4 box_clr = clr;
	porect->config_color(hrect, box_clr);
	porect->config_border(hrect, true, 1.0);
}

void c_aws_ui_box::set_checked_color(const int hbtn, const int hstr)
{
	glm::vec4 box_clr = clr;
	glm::vec4 txt_clr(0, 0, 0, 1);

	porect->config_color(hbtn, box_clr);
	porect->config_border(hbtn, false, 1.0);
	potxt->config_color(hstr, txt_clr, bkgclr);
}

void c_aws_ui_box::set_normal_color(const int hbtn, const int hstr)
{
	glm::vec4 box_clr = clr;
	porect->config_color(hbtn, box_clr);
	porect->config_border(hbtn, true, 1.0);
	potxt->config_color(hstr, clr, bkgclr);
}

bool c_aws_ui_box::set_mouse_event(const glm::vec2 & pt, const int button, const int action, const int modifier)
{
	switch (button){
	case GLFW_MOUSE_BUTTON_LEFT:
		switch (action){
		case GLFW_PRESS:
			if (handle_left_push(pt))
				return true;
			break;
		case GLFW_RELEASE:
			if (handle_left_release(pt))
				return true;
			break;
		}
		break;
	}

	return false;
};

////////////////////////////////////////////////////////////////////////// c_map_cfg_box
const char * c_view_mode_box::str_btn[nul] =
{
	"FPV", "MAP", "SYS"
};

bool c_view_mode_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = (float)(-xmax),
		ymax = (float)(0.5 * sz_scrn.y), ymin = (float)(-ymax);

	glm::vec2 pos, sz_btn, sz_box;
	pos.x = (float)(-sz_scrn.x * 0.5);
	pos.y = y;
	sz_btn.x = (float)(4 * sz_fnt.x);
	sz_btn.y = (float)(1.5 * sz_fnt.y);
	sz_box.x = sz_btn.x;
	sz_box.y = (float)(sz_btn.y * (float)nul);

	pos.x = xmin;
	pos.y = y;
	for (int ibtn = 0; ibtn < nul; ibtn++){
		add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
		pos.y += sz_btn.y;
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}

void c_view_mode_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (mode == ibtn){
			set_checked_color(hbtn[ibtn], hstr[ibtn]);
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
}

void c_view_mode_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
}

bool c_view_mode_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}
	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_view_mode_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_view_mode_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){

		if (breleased){
			if (btn_released == btn_pushed){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_pushed;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			else{
				if (btn_pushed == mode)
					set_checked_color(hbtn[mode], hstr[mode]);
				else
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
			}

			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_released;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			btn_released = nul;
			return true;
		}
	}


	return false;
}

/////////////////////////////////////////////////////////////////// c_ctrl_mode_box
const char * c_ctrl_mode_box::str_btn[nul] =
{
	"CRZ", "CTL", "CSR", "FWP", "STY", "FTG"
};

bool c_ctrl_mode_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = (float)(-xmax),
		ymax = (float)(0.5 * sz_scrn.y), ymin = (float)(-ymax);

	glm::vec2 pos, sz_btn, sz_box;
	pos.x = (float)(-sz_scrn.x * 0.5);
	pos.y = y;
	sz_btn.x = (float)(4 * sz_fnt.x);
	sz_btn.y = (float)(1.5 * sz_fnt.y);
	sz_box.x = sz_btn.x;
	sz_box.y = sz_btn.y * (float)nul;

	pos.x = xmin;
	pos.y = y;

	for (int ibtn = 0; ibtn < nul; ibtn++){
		add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
		pos.y += sz_btn.y;
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_ctrl_mode_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (mode == ibtn){
			set_checked_color(hbtn[ibtn], hstr[ibtn]);
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
}

void c_ctrl_mode_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
}

bool c_ctrl_mode_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_ctrl_mode_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_ctrl_mode_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			if (btn_released == btn_pushed){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_pushed;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			else{
				if (btn_pushed == mode)
					set_checked_color(hbtn[mode], hstr[mode]);
				else
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);

			}
			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				set_normal_color(hbtn[mode], hstr[mode]);
				mode = btn_released;
				set_checked_color(hbtn[mode], hstr[mode]);
			}
			btn_released = nul;
			return true;
		}
	}


	return false;
}


//////////////////////////////////////////////////////////////////// c_map_cfg_box
const char * c_map_cfg_box::str_btn[nul] =
{
	"WP", "VSL", "MRK", "CL", "-", "+"
};

bool c_map_cfg_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = (float)(-xmax),
		ymax = (float)(0.5 * sz_scrn.y), ymin = (float)(-ymax);

	glm::vec2 sz_btn, sz_box, sz_udbtn;
	int rows = nul / 3 + (nul % 3 ? 1 : 0);
	sz_btn.x = (float)(4 * sz_fnt.x);
	sz_btn.y = (float)(1.5 * sz_fnt.y);
	sz_udbtn.x = (float)(2 * sz_fnt.x);
	sz_udbtn.y = (float)(1.5 * sz_fnt.y);

	sz_box.x = (float)(sz_btn.x * 3);
	sz_box.y = (float)(sz_btn.y * (rows + 1));

	glm::vec2 pos;
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y;

	add_select_box(hbtn[range_down], hstr[range_down], str_btn[range_down],
		hbtn[range_up], hstr[range_up], str_btn[range_up], hstr_range, pos, sz_udbtn, sz_box, sz_fnt, 7);

	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = (float)(y + sz_udbtn.y);
	{
		int ibtn = 0;
		while (ibtn < range_down)
		{
			pos.x = (float)(left ? xmin : xmax - sz_box.x);
			for (int i = 0; i < 3 && ibtn < range_down; i++){
				add_btn(hbtn[ibtn], hstr[ibtn], str_btn[ibtn], pos, sz_btn, sz_fnt);
				ibtn++;
				pos.x += sz_btn.x;

			}
			pos.y += sz_btn.y;
		}
	}

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_map_cfg_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		if (ibtn < range_down){
			if (check[ibtn]){
				set_checked_color(hbtn[ibtn], hstr[ibtn]);
			}
			else{
				set_normal_color(hbtn[ibtn], hstr[ibtn]);
			}
		}
		else{
			set_normal_color(hbtn[ibtn], hstr[ibtn]);
		}
		potxt->enable(hstr[ibtn]);
	}
	potxt->enable(hstr_range);
}

void c_map_cfg_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
	potxt->disable(hstr_range);
}


bool c_map_cfg_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_map_cfg_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_map_cfg_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			if (btn_released == btn_pushed){
				command = btn_pushed;
				if (btn_pushed < range_down){
					check[btn_pushed] = !check[btn_pushed];
				}
			}

			if (btn_pushed < range_down){
				if (check[btn_pushed]){
					set_checked_color(hbtn[btn_pushed], hstr[btn_pushed]);
				}
				else{
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
				}
			}
			else{
				set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
			}
			btn_released = btn_pushed = nul;
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				command = btn_released;
				if (btn_released < range_down){
					check[btn_released] = !check[btn_released];
					if (check[btn_released]){
						set_checked_color(hbtn[btn_released], hstr[btn_released]);
					}
					else{
						set_normal_color(hbtn[btn_released], hstr[btn_released]);
					}
				}
			}
			btn_released = nul;
			return true;
		}
	}

	return false;
}

/////////////////////////////////////////////////////////////////// c_route_cfg_box
const char * c_route_cfg_box::str_btn[nul] =
{
	"<", ">", "-", "+", "ADD", "DEL", "<", ">", "LOAD", "SAVE"
};

bool c_route_cfg_box::init(const int handle, const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
{
	hbox = handle;
	bopened = false;
	bkgclr = _bkgclr;
	clr = _clr;

	float xmax = (float)(0.5 * sz_scrn.x), xmin = (float)(-xmax),
		ymax = (float)(0.5 * sz_scrn.y), ymin = (float)(-ymax);

	glm::vec2 sz_btn, sz_box, sz_udbtn;
	sz_btn.x = (float)(5 * sz_fnt.x);
	sz_btn.y = sz_udbtn.y = (float)(1.5 * sz_fnt.y);
	sz_udbtn.x = (float)(2 * sz_fnt.x);

	sz_box.x = (float)(sz_btn.x * 2);
	sz_box.y = (float)(sz_btn.y * 5);

	glm::vec2 pos;

	// waypoint selection interface
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y + sz_box.y - sz_btn.y;
	add_select_box(hbtn[wp_prev], hstr[wp_prev], str_btn[wp_prev],
		hbtn[wp_next], hstr[wp_next], str_btn[wp_next], hstr_wp, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// waypoint speed selection interface
	pos.y = y + sz_box.y - 2 * sz_btn.y;
	add_select_box(hbtn[wp_spd_down], hstr[wp_spd_down], str_btn[wp_spd_down],
		hbtn[wp_spd_up], hstr[wp_spd_up], str_btn[wp_spd_up], hstr_spd, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// add/delete button
	pos.y = y + sz_box.y - 3 * sz_btn.y;
	add_btn(hbtn[wp_add], hstr[wp_add], str_btn[wp_add], pos, sz_btn, sz_fnt);
	pos.x += sz_btn.x;
	add_btn(hbtn[wp_del], hstr[wp_del], str_btn[wp_del], pos, sz_btn, sz_fnt);

	// route selection interface
	pos.x = (float)(left ? xmin : xmax - sz_box.x);
	pos.y = y + sz_box.y - 4 * sz_btn.y;
	add_select_box(hbtn[rt_prev], hstr[rt_prev], str_btn[rt_prev],
		hbtn[rt_next], hstr[rt_next], str_btn[rt_next], hstr_rt, pos, sz_udbtn, sz_box, sz_fnt, 5);

	// save/load button
	pos.y = y + sz_box.y - 5 * sz_btn.y;
	add_btn(hbtn[rt_save], hstr[rt_save], str_btn[rt_save], pos, sz_btn, sz_fnt);
	pos.x += sz_btn.x;
	add_btn(hbtn[rt_load], hstr[rt_load], str_btn[rt_load], pos, sz_btn, sz_fnt);

	return c_aws_ui_box::init(handle, clr, bkgclr, sz_fnt, sz_scrn, y, left);
}


void c_route_cfg_box::open()
{
	c_aws_ui_box::open();

	for (int ibtn = 0; ibtn < nul; ibtn++){
		porect->enable(hbtn[ibtn]);
		porect->config_border(hbtn[ibtn], true, 1.0);
		porect->config_color(hbtn[ibtn], clr);
		potxt->enable(hstr[ibtn]);
	}
	potxt->enable(hstr_wp);
	potxt->enable(hstr_spd);
	potxt->enable(hstr_rt);
}

void c_route_cfg_box::close()
{
	c_aws_ui_box::close();

	for (int ibtn = 0; ibtn < nul; ibtn++)
	{
		porect->disable(hbtn[ibtn]);
		potxt->disable(hstr[ibtn]);
	}
	potxt->disable(hstr_wp);
	potxt->disable(hstr_spd);
	potxt->disable(hstr_rt);
}


bool c_route_cfg_box::handle_left_push(const glm::vec2 & pt)
{
	if (bopened){
		for (int ibtn = 0; ibtn < (int)nul; ibtn++){
			if (porect->collision(pt, hbtn[ibtn])){
				btn_pushed = (e_btn)ibtn;
				break;
			}
		}
	}

	if (btn_pushed == nul)
		return c_aws_ui_box::handle_left_push(pt);
	else
		return true;
}

bool c_route_cfg_box::handle_left_release(const glm::vec2 & pt)
{
	for (int ibtn = 0; ibtn < (int)nul; ibtn++){
		if (porect->collision(pt, hbtn[ibtn])){
			btn_released = (e_btn)ibtn;
			break;
		}
	}

	if (btn_released == nul)
		return c_aws_ui_box::handle_left_release(pt);
	else
		return true;
}

bool c_route_cfg_box::proc(const bool bpushed, const bool breleased)
{
	if (c_aws_ui_box::proc(bpushed, breleased))
		return true;

	if (btn_pushed != nul){
		if (breleased){
			set_normal_color(hbtn[btn_pushed]);
			if (btn_released == btn_pushed){
				command = btn_pushed;
				if (command == wp_add)
					set_checked_color(hbtn[btn_pushed], hstr[btn_pushed]);
				else
					set_normal_color(hbtn[btn_pushed], hstr[btn_pushed]);
				btn_released = btn_pushed = nul;
			}
			return true;
		}
		else{
			set_selected_color(hbtn[btn_pushed]);
			return true;
		}
	}
	else{
		if (breleased){
			if (btn_released != nul){
				command = btn_released;
				if (command == wp_add)
					set_checked_color(btn_pushed, hstr[btn_released]);
				else
					set_normal_color(btn_pushed, hstr[btn_released]);
			}
			btn_released = nul;
			return true;
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////// c_indicator
c_indicator::c_indicator() : porect(NULL), potri(NULL), poline(NULL), potxt(NULL),
meng(127), seng(127), rud(0), cog(0.1f), sog(10), yaw(0.05f), pitch(0.5f), roll(0.5f),
veng_n(0x7f), veng_nf(0x7f + 0x19), veng_nb(0x7f - 0x19), dir_cam(0.f), mode(ui_mode_fpv)
{
}

c_indicator::~c_indicator()
{
}

void c_indicator::create_engine_indicator(int & heng_in, int & heng_out,
	int & heng_n, int & heng_f, int & heng_b,
	glm::vec2 & pos, const glm::vec2 & sz_fnt,
	const glm::vec4 & clr)
{
  glm::vec2
    pos_eng,
    pos_eng_out,
    pos_eng_in,
    scl_eng_out,
    scl_eng_in,
    pos_eng_g, pos_eng_n, sz_g, sz_n;
  pos_eng = pos;
  sz_n.x = (float)(sz_fnt.x * 0.8);
  sz_n.y = (float)(sz_fnt.y * 0.8);
  sz_g.x = (float)(sz_fnt.x * 0.4);
  sz_g.y = (float)(sz_fnt.y * 0.4);
  pos_eng_in.x = (float)(pos_eng.x);
  pos_eng_in.y = (float)(pos_eng.y + sz_fnt.y);
  pos_eng_out = pos_eng_in;
  pos_eng_n.x = (float)(pos_eng.x + 0.1 * sz_fnt.x);
  pos_eng_n.y = (float)(pos_eng.y + 0.1 * sz_fnt.y);
  pos_eng_g.x = (float)(pos_eng.x + 0.5 * sz_fnt.x);
  pos_eng_g.y = (float)(pos_eng.y + 0.5 * sz_fnt.y);
  scl_eng_in.x = scl_eng.x;
  scl_eng_in.y = 0.f;
  scl_eng_out.x = scl_eng.x;
  scl_eng_out.y = scl_eng.y;
  heng_in = porect->add(clr, pos_eng_in, 0.f, 1.f);
  porect->config_border(heng_in, false, 1.f);
  porect->config_depth(heng_in, 0);
  porect->config_scale(heng_in, scl_eng_in);
  
  heng_out = porect->add(clr, pos_eng_out, 0.f, 1.f);
  porect->config_scale(heng_out, scl_eng_out);
  porect->config_border(heng_out, true, 1.f);
  porect->config_depth(heng_out, 0);
  
  heng_n = porect->add(clr, pos_eng_n, 0.f, sz_n);
  porect->config_border(heng_n, true, 1.f);
  porect->config_depth(heng_n, 0);
  heng_f = potri->add(clr, pos_eng_g, (float)(PI * 0.5), sz_g);
  potri->config_border(heng_b, false, 1.f);
  potri->config_depth(heng_f, 0);
  heng_b = potri->add(clr, pos_eng_g, (float)(-PI * 0.5), sz_g);
  potri->config_border(heng_b, false, 1.f);
  potri->config_depth(heng_b, 0);
}

void c_indicator::update_engine_indicator(int & heng_in, int & heng_n, int & heng_f, int & heng_b, const unsigned char val)
{
  glm::vec2 scl(scl_eng.x, scl_eng.y * abs((int)val - (int)127) * (1 / 127.));
  porect->config_scale(heng_in, scl);
  
  if (val == veng_n){
    porect->config_border(heng_n, false, 1.0);
    porect->enable(heng_n);
    potri->disable(heng_f);
    potri->disable(heng_b);
  }
  else if (val > veng_n){
    if (val > veng_nf){
      porect->disable(heng_n);
      potri->enable(heng_f);
      potri->disable(heng_b);
    }
    else{
      porect->config_border(heng_n, true, 1.0);
      porect->enable(heng_n);
      potri->enable(heng_f);
      potri->disable(heng_b);
    }
  }
  else{
    if (val < veng_nb){
      porect->disable(heng_n);
      potri->disable(heng_f);
      potri->enable(heng_b);
    }
    else{
      porect->config_border(heng_n, true, 1.0);
      porect->enable(heng_n);
      potri->disable(heng_f);
      potri->enable(heng_b);
    }
  }
}

void c_indicator::create_rudder_indicator(glm::vec2 & pos,
					  const glm::vec2 & sz_fnt,
					  const glm::vec4 & clr)
{
  glm::vec2 pos_rud_in, pos_rud_out, scl_rud_in, scl_rud_out;
  scl_rud_in.x = 0;
  scl_rud_in.y = sz_fnt.y;
  scl_rud_out = scl_rud;
  
  pos_rud_in.x = pos.x;
  pos_rud_in.y = pos.y;
  pos_rud_out.x = (float)(pos.x - scl_rud_out.x * 0.5);
  pos_rud_out.y = pos.y;
  
  hrud_in = porect->add(clr, pos_rud_in, 0.f, 1.f);
  porect->config_scale(hrud_in, scl_rud_in);
  porect->config_border(hrud_in, false, 1.f);
  porect->config_depth(hrud_in, 0);
  hrud_out = porect->add(clr, pos_rud_out, 0.f, 1.f);
  porect->config_scale(hrud_out, scl_rud_out);
  porect->config_border(hrud_out, true, 1.f);
  porect->config_depth(hrud_out, 0);
}

void c_indicator::update_rudder_indicator()
{
  int srud = -(int)rud + (int)127;
  
  glm::vec2 scl(scl_rud.x * abs(srud) * (1.0f / 255.0f), scl_rud.y);
  glm::vec2 pos;
  if (srud < 0)
    pos.x = (float)(pos_rud.x - scl.x);
  else
    pos.x = pos_rud.x;
  pos.y = pos_rud.y;
  porect->config_position(hrud_in, pos);
  porect->config_scale(hrud_in, scl);
}

void c_indicator::create_sog_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt, const glm::vec4 & clr)
{
#define RAD_SOG_ARC 6.5
#define NUM_SOG_SCALE (2*(SOG_STEP-1)+1)
#define NUM_SOG_ARC_PTS (2*(NUM_SOG_SCALE-1)+1)
  rad_sog = sz_fnt;
  rad_sog.x *= (RAD_SOG_ARC + 0.5);
  rad_sog.y *= (RAD_SOG_ARC + 0.5);
  
  pos_sog = pos;
  glm::vec2 mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y);
  struct s_vertex{
    float x, y;
  };
  // arc
  {
    s_vertex pts[NUM_SOG_ARC_PTS];
    float c, s, ths = (float)(PI / (float)(NUM_SOG_ARC_PTS - 1));
    for (int i = 0; i < NUM_SOG_ARC_PTS; i++)
      {
	float th = (float)(i * ths);
	c = cos(th);
	s = sin(th);
	pts[i].x = (float)(c * sz_fnt.x * RAD_SOG_ARC);
	pts[i].y = (float)(s * sz_fnt.y * RAD_SOG_ARC);
      }
    
    hsog_arc = poline->add(NUM_SOG_ARC_PTS, (float*)pts);
    poline->config_position(hsog_arc, pos);
    poline->config_rotation(hsog_arc, 0);
    poline->config_width(hsog_arc, 1.0);
    poline->config_color(hsog_arc, clr);
    poline->config_depth(hsog_arc, 0);
  }
  
  // scale
  {
    float c, s, ths = (float)(PI / (float)(NUM_SOG_SCALE - 1));
    s_vertex pts[NUM_SOG_SCALE * 2];
    
    for (int i = 0, iscl = SOG_STEP - 1; i < NUM_SOG_SCALE; i++){
      float th = (float)(ths * i);
      c = (float)(cos(th) * sz_fnt.x);
      s = (float)(sin(th) * sz_fnt.y);
      s_vertex & vtx0 = pts[2 * i];
      s_vertex & vtx1 = pts[2 * i + 1];
      vtx0.x = (float)(c * RAD_SOG_ARC);
      vtx0.y = (float)(s * RAD_SOG_ARC);
      float rscl = (i % 2 == 0 ? (float)(RAD_SOG_ARC - 0.5) : (float)(RAD_SOG_ARC - 0.25));
      vtx1.x = (float)(c * rscl);
      vtx1.y = (float)(s * rscl);
      
      if (i % 2 == 0){ // creating scale string
	char str[4];
	float rstr = (float)(rscl - 0.25);
	glm::vec2 pos_str(c * rstr + pos.x, s * rstr + pos.y);
	snprintf(str, 4, "%d", iscl * 10);
	hstr_sog_scale[iscl] = potxt->reserv(2);
	potxt->set(hstr_sog_scale[iscl], str);
	potxt->config(hstr_sog_scale[iscl], clr, glm::vec4(0, 0, 0, 0),
		      sz_fnt, mgn_fnt, c_gl_text_obj::an_ct, pos_str, (float)(th - 0.5 * PI));
	potxt->config_depth(hstr_sog_scale[iscl], 0);
	potxt->enable(hstr_sog_scale[iscl]);
	iscl--;
      }
    }
    
    hsog_scale = poline->add(NUM_SOG_SCALE * 2, (float*)pts, true);
    poline->config_depth(hsog_scale, 0);
    poline->config_position(hsog_scale, pos);
    poline->config_rotation(hsog_scale, 0.0);
    poline->config_color(hsog_scale, clr);
    poline->config_width(hsog_scale, 1.0f);
  }
  //indicator
  hsog_ptr = potri->add(clr, pos, 0.0f, 0.0f);
  potri->config_border(hsog_ptr, false, 1.0);
  glm::vec2 sz_ptr((float)(sz_fnt.x * 0.5), (float)(sz_fnt.y * 0.5));
  potri->config_scale(hsog_ptr, sz_ptr);
  potri->config_depth(hsog_ptr, 0);
  potri->enable(hsog_ptr);
}

void c_indicator::update_sog_indicator()
{
  float thtri = (float)(-sog * PI / 40.0);
  potri->config_rotation(hsog_ptr, thtri);
  float th = (float)(PI + thtri);
  glm::vec2 pos((float)(cos(th) * rad_sog.x + pos_sog.x),
		(float)(sin(th) * rad_sog.y + pos_sog.y));
  potri->config_position(hsog_ptr, pos);
}

void c_indicator::create_rp_indicator(glm::vec2 & pos,
				      const glm::vec2 & sz_fnt,
				      const glm::vec4 & clr)
{
  // pitch measure
  struct s_vertex{
    float x, y;
  };
  pos_rp = pos;
  lpmeas = 4 * sz_fnt.y;
  glm::vec2 sz_sfnt((float)(sz_fnt.x * 0.75), (float)(sz_fnt.y * 0.75));
  glm::vec2 mgn_sfnt((float)(sz_sfnt.x * 0.6), sz_fnt.y);
  
  {
    s_vertex pts[5] = {
      { -sz_fnt.x, lpmeas },
      { -sz_fnt.x, -lpmeas },
      { 0, (float)(-(lpmeas + 2 * sz_fnt.y)) },
      { sz_fnt.x, -lpmeas },
      { sz_fnt.x, lpmeas }
    };
    
    hpmeas = poline->add(5, (float*)pts);
    poline->config_depth(hpmeas, 0);
    poline->config_position(hpmeas, pos);
    poline->config_rotation(hpmeas, 0.0);
    poline->config_color(hpmeas, clr);
    poline->config_width(hpmeas, 1.0f);
  }
  
  {
    s_vertex pts[5] = {
      { (float)(-sz_fnt.x), 0 },
      { (float)(-0.5 * sz_fnt.x), 0 },
      { 0, (float)(-sz_fnt.y) },
      { (float)(0.5 * sz_fnt.x), 0 },
      { sz_fnt.x, 0 }
    };
    hpptr = poline->add(5, (float*)pts);
    poline->config_depth(hpptr, 0);
    poline->config_position(hpptr, pos);
    poline->config_rotation(hpptr, 0.0);
    poline->config_color(hpptr, clr);
    poline->config_width(hpptr, 1.0f);
  }
  
  {
    s_vertex pts[((PITCH_STEP - 1) * 2 + 1) * 2];
    glm::vec2 ptstart(sz_fnt.x, lpmeas);
    float sstep = (float)(lpmeas * 2 / (float)((PITCH_STEP - 1) * 2));
    for (int i = 0, ip = PITCH_STEP - 1; i < PITCH_STEP * 2 - 1; i++){
      s_vertex & pt0 = pts[i * 2];
      s_vertex & pt1 = pts[i * 2 + 1];
      float l = (i % 2 == 0 ? (float)(0.5 * sz_fnt.x) : (float)(0.25 * sz_fnt.y));
      pt0.x = ptstart.x;
      pt1.x = (float)(pt0.x + l);
      pt0.y = pt1.y = (float)(ptstart.y - sstep * i);
      
      if (i % 2 == 0){
	hstr_pscale[ip] = potxt->reserv(3);
	pos_pscale[ip].x = pt1.x;
	pos_pscale[ip].y = pt1.y;
	char str[4];
	snprintf(str, 4, "%d", abs((ip - PITCH_STEP / 2) * 10));
	potxt->set(hstr_pscale[ip], str);
	potxt->config(hstr_pscale[ip], clr, glm::vec4(0, 0, 0, 0),
		      sz_sfnt, mgn_sfnt, c_gl_text_obj::an_lc,
		      pos_pscale[ip] + pos_rp, 0.f);
	potxt->enable(hstr_pscale[ip]);
	ip--;
      }
    }
    
    hpscale = poline->add(PITCH_STEP * 4, (float*)pts, true);
    poline->config_depth(hpscale, 0);
    poline->config_position(hpscale, pos);
    poline->config_rotation(hpscale, 0.0);
    poline->config_color(hpscale, clr);
    poline->config_width(hpscale, 1.0f);
  }
#define NUM_RARC_PTS ((ROLL_STEP-1)*4+1)
#define RAD_RARC 6
  // roll arc
  {
    s_vertex pts[NUM_RARC_PTS];
    float c, s, ths = (float)(PI / (float)(NUM_RARC_PTS - 1));
    for (int i = 0; i < NUM_RARC_PTS; i++){
      float th = (float)(PI + i * ths);
      c = cos(th);
      s = sin(th);
      pts[i].x = (c * sz_fnt.x * RAD_RARC);
      pts[i].y = (s * sz_fnt.y * RAD_RARC);
    }
    
    hrarc = poline->add(NUM_RARC_PTS, (float*)pts);
    poline->config_position(hrarc, pos);
    poline->config_rotation(hrarc, 0);
    poline->config_width(hrarc, 1.0);
    poline->config_color(hrarc, clr);
    poline->config_depth(hrarc, 0);
  }
#define NUM_RSCL_PTS ((ROLL_STEP-1)*2+1)
  // roll scale
  {
    s_vertex pts[NUM_RSCL_PTS * 2];
    float rr;
    float c, s, ths = (float)(PI / (float)(NUM_RSCL_PTS - 1));
    for (int i = 0, ir = ROLL_STEP; i < NUM_RSCL_PTS; i++){
      float th = (float)(PI + i * ths);
      c = (float)(cos(th) * sz_fnt.x);
      s = (float)(sin(th) * sz_fnt.y);
      s_vertex & pt0 = pts[i * 2];
      s_vertex & pt1 = pts[i * 2 + 1];
      pt0.x = (float)(c * RAD_RARC);
      pt0.y = (float)(s * RAD_RARC);
      rr = (i % 2 == 0 ? (float)(RAD_RARC + 0.5) : (float)(RAD_RARC + 0.25));
      pt1.x = (float)(c * rr);
      pt1.y = (float)(s * rr);
      if (i % 2 == 0){
	char buf[4];
	glm::vec2 pos_str((float)(pt1.x + pos.x), (float)(pt1.y + pos.y));
	hstr_rscale[ir - 1] = potxt->reserv(3);
	snprintf(buf, 4, "%d", abs((ir - 10) * 10));
	potxt->set(hstr_rscale[ir - 1], buf);
	potxt->config(hstr_rscale[ir - 1], clr, glm::vec4(0, 0, 0, 0),
		      sz_sfnt, mgn_sfnt, c_gl_text_obj::an_ct, pos_str, (float)(-0.5 * PI + ths * i));
	potxt->enable(hstr_rscale[ir - 1]);
	ir--;
      }
    }
    hrscale = poline->add(NUM_RSCL_PTS * 2, (float*)pts, true);
    poline->config_position(hrscale, pos);
    poline->config_rotation(hrscale, 0);
    poline->config_width(hrscale, 1.0);
    poline->config_color(hrscale, clr);
    poline->config_depth(hrscale, 0);
  }
}

void c_indicator::update_rp_indicator()
{
	float thr = -roll;
	float c = cos(thr), s = sin(thr);

	poline->config_rotation(hpmeas, roll, c, s);
	poline->config_rotation(hpscale, roll, c, s);
	poline->config_rotation(hpptr, roll, c, s);
	glm::vec2 pos_pptr(0, lpmeas * pitch * (1.0 / (PI * 40. / 180.)));
	pos_pptr.x = -s * pos_pptr.y + pos_rp.x;
	pos_pptr.y = c * pos_pptr.y + pos_rp.y;
	poline->config_position(hpptr, pos_pptr);

	for (int i = 0; i < PITCH_STEP; i++)
	{
		glm::vec2 pos;
		pos.x = c * pos_pscale[i].x - s * pos_pscale[i].y + pos_rp.x;
		pos.y = s * pos_pscale[i].x + c * pos_pscale[i].y + pos_rp.y;
		potxt->config_rotation(hstr_pscale[i], c, s);
		potxt->config_position(hstr_pscale[i], pos);
	}
}

void c_indicator::create_hc_indicator(const float fov,
	const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const glm::vec4 & clr)
{
	fxcam = (float)(0.5 * sz_scrn.x / tan(0.5 * fov * PI / 180.f));
	float xmin = (float)(-sz_scrn.x * 0.5), xmax = -xmin,
		ymin = (float)(-sz_scrn.y * 0.5), ymax = -ymin;

	struct s_vertex{
		float x, y;
	};

	glm::vec2 mgn_fnt((float)(sz_fnt.x * 0.6), sz_fnt.y), rad_ptr((float)(sz_fnt.x * 0.3), sz_fnt.y);
	glm::vec2 pos_str, pos_scale, pos_ptr;
	{
		s_vertex vtx[2] = {
			{ xmin, 0 }, { xmax, 0 }
		};
		pos_scale.x = pos_scale.y = 0;
		hhlzn = poline->add(2, (float*)vtx);
		poline->config_position(hhlzn, pos_scale);
		poline->config_rotation(hhlzn, 0);
		poline->config_width(hhlzn, 1.0);
		poline->config_color(hhlzn, clr);
		poline->config_depth(hhlzn, 0);

		vtx[0].x = vtx[1].x = pos_str.x = pos_ptr.x = 0;
		vtx[0].y = 0;
		vtx[1].y = pos_str.y = pos_ystr = (float)(sz_fnt.y * 0.5);
		pos_ptr.y = pos_yptr = (float)(-sz_fnt.y);
		float ths = (float)(2 * PI / (float)YAW_STEP);
		for (int i = 0; i < YAW_STEP; i++)
		{
			char str[3];

			hstr_yscale[i] = potxt->reserv(3);
			hyscale[i] = poline->add(2, (float*)vtx);
			snprintf(str, 3, "%02d", i);
			potxt->set(hstr_yscale[i], str);
			potxt->config(hstr_yscale[i], clr, glm::vec4(0, 0, 0, 0),
				sz_fnt, mgn_fnt, c_gl_text_obj::an_cb, pos_str, 0);
			potxt->config_depth(hstr_yscale[i], 0);
			potxt->enable(hstr_yscale[i]);

			poline->config_position(hyscale[i], pos_scale);
			poline->config_rotation(hyscale[i], 0);
			poline->config_width(hyscale[i], 1.0);
			poline->config_color(hyscale[i], clr);
			poline->config_depth(hyscale[i], 0);

			// here pos_yscl has the normalized circular vector for the yaw direction.
			pos_yscl[i].x = sin((float)(ths * i));
			pos_yscl[i].y = cos((float)(ths * i));
		}
	}

	hhptr = potri->add(clr, pos_ptr, (float)(0.5 * PI), rad_ptr);
	potri->config_border(hhptr, false, 1.0);
	potri->config_depth(hhptr, 0);
	potri->enable(hhptr);
	hcptr = potri->add(clr, pos_ptr, (float)(0.5 * PI), rad_ptr);
	potri->config_border(hcptr, true, 1.0);
	potri->config_depth(hcptr, 0);
	potri->enable(hcptr);
}

void c_indicator::update_hc_indicator()
{
	float dir_cam_abs = yaw + dir_cam;

	// unit vectors are calculated ( all of them are defined in the ENU world coordinate)
	// pos_cam : camera direction vector
	// pos_crs : course direction vector
	// pos_yaw : heding vector
	glm::vec2 pos_cam((float)sin(dir_cam_abs), (float)cos(dir_cam_abs));
	glm::vec2 pos_crs((float)sin(cog), (float)cos(cog));
	glm::vec2 pos_yaw((float)sin(yaw), (float)cos(yaw));


	if (mode == ui_mode_fpv){// calculating hc indicator for first person view mode
		poline->enable(hhlzn);
		// converting the course vector to the camera coordinate
		glm::vec2 pos_crs_tmp(
			(float)(pos_cam.y * pos_crs.x - pos_cam.x * pos_crs.y),
			(float)(pos_cam.x * pos_crs.x + pos_cam.y * pos_crs.y));
		if (pos_crs_tmp.y > 0){ // don't calculate if the indicator is not in the camera direction
			// projecting the vector's x position using the focal length fxcam given in the initialization.
			pos_crs_tmp.y /= pos_crs_tmp.x;
			pos_crs_tmp.x *= fxcam;
			pos_crs_tmp.y = pos_yptr;
			potri->config_position(hcptr, pos_crs_tmp);
			potri->enable(hcptr);
		}
		else{
			potri->disable(hcptr);
		}

		// converting the heding vector to the camera coordinate
		glm::vec2 pos_yaw_tmp(
			(float)(pos_cam.y * pos_yaw.x - pos_cam.x * pos_yaw.y),
			(float)(pos_cam.x * pos_yaw.x + pos_cam.y * pos_yaw.y));

		if (pos_yaw_tmp.y > 0){ // don't calculate if the indicator is not in the camera direction
			// projecting the vector's x position using the focal length fxcam given in the initialization.
			pos_yaw_tmp.x /= pos_yaw_tmp.y;
			pos_yaw_tmp.x *= fxcam;
			pos_yaw_tmp.y = pos_yptr;
			potri->config_position(hhptr, pos_yaw_tmp);
			potri->enable(hhptr);
		}
		else{
			potri->disable(hhptr);
		}

		for (int i = 0; i < YAW_STEP; i++){
			// projectig the scale vectors in the camera coordinate
			pos_yscl_tmp[i].x = (float)(pos_cam.y * pos_yscl[i].x - pos_cam.x * pos_yscl[i].y);
			pos_yscl_tmp[i].y = (float)(pos_cam.x * pos_yscl[i].x + pos_cam.y * pos_yscl[i].y);
			pos_yscl_tmp[i].x /= pos_yscl_tmp[i].y;
			pos_yscl_tmp[i].x *= fxcam;
			if (pos_yscl_tmp[i].y < 0){ // back side of the camera is not calculated.
				poline->disable(hyscale[i]);
				potxt->disable(hstr_yscale[i]);
			}
			else{
				poline->enable(hyscale[i]);
				pos_yscl_tmp[i].y = 0; // scale is always rendered in the center of the display.
				poline->config_position(hyscale[i], pos_yscl_tmp[i]);
				potxt->enable(hstr_yscale[i]);
				pos_yscl_tmp[i].y = pos_ystr;
				potxt->config_position(hstr_yscale[i], pos_yscl_tmp[i]);
			}
		}
	}
	else{ // for map mode, simply disable the indicator, in this implementaion.
		poline->disable(hhlzn);
		potri->disable(hhptr);
		potri->disable(hcptr);
		for (int i = 0; i < YAW_STEP; i++){
			poline->disable(hyscale[i]);
			potxt->disable(hstr_yscale[i]);
		}
	}
}

bool c_indicator::init(c_gl_2d_line_obj * _poline, c_gl_text_obj * _potxt,
	c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
	const glm::vec2 & sz_fnt, const glm::vec4 & clr,
	const float fovx, const glm::vec2 & sz_scrn)
{
	poline = _poline;
	potxt = _potxt;
	porect = _porect;
	potri = _potri;

	float xmin = (float)(-sz_scrn.x * 0.5), xmax = (float)(-xmin),
		ymin = (float)(-sz_scrn.y * 0.5), ymax = (float)(-ymin);
	// create meng/seng indicator (left bottom corner) 

	scl_eng.x = sz_fnt.x;
	scl_eng.y = (float)(0.25 * sz_scrn.y);
	scl_rud.x = (float)(sz_scrn.x * 0.4);
	scl_rud.y = sz_fnt.y;

	glm::vec2 pos_meng, pos_seng;
	pos_meng.x = xmin;
	pos_meng.y = ymin;
	pos_seng.x = xmin + sz_fnt.x;
	pos_seng.y = ymin;
	pos_rud.x = 0.f;
	pos_rud.y = ymin;
	create_engine_indicator(hmeng_in, hmeng_out, hmeng_n, hmeng_f, hmeng_b, pos_meng, sz_fnt, clr);
	create_engine_indicator(hseng_in, hseng_out, hseng_n, hseng_f, hseng_b, pos_seng, sz_fnt, clr);

	// create rudder indicator (bottom center)
	create_rudder_indicator(pos_rud, sz_fnt, clr);

	// create sog indicator (right bottom corner)
	glm::vec2 pos_sog_indicator(xmax - 8 * sz_fnt.x, ymin + 10 * sz_fnt.y);
	create_sog_indicator(pos_sog_indicator, sz_fnt, clr);

	// create roll/pitch indicator (right bottom corner)
	glm::vec2 pos_rp_indicator(xmax - 8 * sz_fnt.x, ymin + 8 * sz_fnt.y);
	create_rp_indicator(pos_rp_indicator, sz_fnt, clr);

	// create hdg/cog indicator (top center, only in fpv)
	create_hc_indicator(fovx, sz_fnt, sz_scrn, clr);

	update_engine_indicator(hmeng_in, hmeng_n, hmeng_f, hmeng_b, meng);
	update_engine_indicator(hseng_in, hseng_n, hseng_f, hseng_b, seng);
	update_sog_indicator();
	update_rp_indicator();
	update_hc_indicator();
	update_rudder_indicator();

	return true;
}

void c_indicator::set_param(
	const unsigned char _meng, const unsigned char _seng, const unsigned char _rud,
	const float _cog, const float _sog,
	const float _yaw, const float _pitch, const float _roll)
{
	meng = _meng;
	seng = _seng;
	rud = _rud;
	cog = _cog;
	sog = _sog;
	yaw = _yaw;
	pitch = _pitch;
	roll = _roll;

	update_engine_indicator(hmeng_in, hmeng_n, hmeng_f, hmeng_b, meng);
	update_engine_indicator(hseng_in, hseng_n, hseng_f, hseng_b, seng);
	update_sog_indicator();
	update_rp_indicator();
	update_hc_indicator();
	update_rudder_indicator();
}
