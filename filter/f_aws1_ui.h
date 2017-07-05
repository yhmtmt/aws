// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ui.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ui.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ui.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_UI_H_
#define _F_AWS1_UI_H_

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "f_aws1_ctrl.h"
#include "f_glfw_window.h"

#include "../channel/ch_image.h"
#include "../channel/ch_state.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_aws1_sys.h"
#include "../channel/ch_map.h"
#include "../channel/ch_wp.h"
#include "../channel/ch_obj.h"

#include "../util/aws_jpad.h"

#define MAX_RT_FILES 10

enum e_ui_mode {
	ui_mode_fpv, ui_mode_map, ui_mode_sys, ui_mode_undef
};

class c_aws_ui_box
{
protected:
	static c_gl_2d_obj * porect, * potri;
	static c_gl_2d_line_obj * poline;
	static c_gl_text_obj * potxt;
	bool bopened;
	int hbox, hopen, hclose;
	bool btn_oc_pushed, btn_oc_released;
	glm::vec2 pt_mouse;
	glm::vec2 pos_close, pos_open; // box position
	glm::vec2 sz_close, sz_open; // box size

	glm::vec4 clr, bkgclr;
	
	void add_btn(int & hbtn, int & hstr, const char * str,
		const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_fnt);
	void add_select_box(int & hlbtn, int & hlstr, const char * lstr, int & hrbtn, int & hrstr, const char * rstr, int & hvalstr, 
		const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_box, const glm::vec2 & sz_fnt, const unsigned int len_str);
	void setup_frame(const float y, const bool left, const glm::vec2 & sz_scrn, 
		const glm::vec2 &sz_box, const glm::vec2  & sz_fnt, const glm::vec4 & clr);

	void set_selected_color(const int hrect);
	void set_normal_color(const int hrect);
	void set_checked_color(const int hbtn, const int hstr);
	void set_normal_color(const int hbtn, const int hstr);

	virtual void open();
	virtual void close();

	virtual bool handle_left_push(const glm::vec2 & pt);
	virtual bool handle_left_release(const glm::vec2 & pt);

public:
	c_aws_ui_box() : bopened(false), hbox(-1), hopen(-1), hclose(-1), btn_oc_pushed(false), btn_oc_released(false)
	{
	}

	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
	{
		setup_frame(y, left, sz_scrn, get_box_size(sz_fnt), sz_fnt, clr);

		bopened = false;
		close();
		return true;
	};

	virtual const glm::vec2 get_box_size(const glm::vec2 sz_font)
	{
		return sz_open;
	};

	virtual bool set_mouse_event(const glm::vec2 & pt, 
		const int button, const int action, const int modifier);

	virtual bool proc(const bool bpushed, const bool breleased);

	static void set_gl_objs(c_gl_2d_obj * _porect, c_gl_2d_obj * potri, 
		c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline);
};

class c_view_mode_box : public c_aws_ui_box
{
public:
	enum e_btn{
		fpv = 0, // first person view
		map, // map view
		sys, // system menu
		nul
	};
private:
	static const char * str_btn[nul];
	vector<int> hstr;
	vector<int> hbtn;
	e_btn btn_pushed, btn_released;

	e_btn mode;
	virtual void open();
	virtual void close();
	virtual bool handle_left_push(const glm::vec2 & pt);
	virtual bool handle_left_release(const glm::vec2 & pt);
public:
	c_view_mode_box() :c_aws_ui_box(), mode(fpv), btn_pushed(nul), btn_released(nul)
	{
		hstr.resize(nul);
		hbtn.resize(nul);
	}

	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left);
	virtual const glm::vec2 get_box_size(const glm::vec2 sz_fnt)
	{
		glm::vec2 pos, sz_btn, sz_box;
		sz_btn.x = (float)(4 * sz_fnt.x);
		sz_btn.y = (float)(1.5 * sz_fnt.y);
		sz_box.x = sz_btn.x;
		sz_box.y = (float)(sz_btn.y * (float)nul);
		return sz_box;
	}

	virtual bool proc(const bool bpushed, const bool breleased);

	void set_mode(const e_btn _mode){
		mode = _mode;
	}

	e_ui_mode get_mode(){
		switch (mode) {
		case fpv:
			return ui_mode_fpv;
		case map:
			return ui_mode_map;
		case sys:
			return ui_mode_sys;
		}
		return ui_mode_undef;
	}
};

class c_ctrl_mode_box : public c_aws_ui_box
{
public:
	enum e_btn{
		crz = 0, // manual cruise mode
		ctl,	// manual control mode
		csr,	// cursor control mode
		fwp,	// follow way point mode
		sty,	// stay mode
		ftg,	// follow target mode
		nul
	};
private:
	static const char * str_btn[nul];
	vector<int> hstr;
	vector<int> hbtn;
	e_btn btn_pushed, btn_released;

	e_btn mode;
	virtual void open();
	virtual void close();
	virtual bool handle_left_push(const glm::vec2 & pt);
	virtual bool handle_left_release(const glm::vec2 & pt);

public:
	c_ctrl_mode_box() :c_aws_ui_box(), mode(crz), btn_pushed(nul), btn_released(nul)
	{
		hstr.resize(nul);
		hbtn.resize(nul);
	};

	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left);

	virtual const glm::vec2 get_box_size(const glm::vec2 sz_fnt)
	{
		glm::vec2 pos, sz_btn, sz_box;
		sz_btn.x = (float)(4 * sz_fnt.x);
		sz_btn.y = (float)(1.5 * sz_fnt.y);
		sz_box.x = sz_btn.x;
		sz_box.y = (float)(sz_btn.y * (float)nul);
		return sz_box;
	}

	virtual bool proc(const bool bpushed, const bool breleased);

	void set_mode(const e_btn _mode){
		set_normal_color(hbtn[mode], hstr[mode]);
		mode = _mode;
		set_checked_color(hbtn[mode], hstr[mode]);
	}

	const e_btn get_mode(){
		return mode;
	}
};

class c_obj_cfg_box : public c_aws_ui_box
{
public:
	enum e_btn{
		wp,		// waypoint
		vsl,	// vessel
		mrk,	// mark
		cl,		// cast line
		range_down,
		range_up,
		nul
	};

private:
	static const char * str_btn[nul];
	vector<int> hstr;
	vector<int> hbtn;
	e_btn btn_pushed, btn_released;
	e_btn command;

	vector<bool> check;
	int hstr_range;
	char range_str[8];
	float range;
	virtual void open();
	virtual void close();
	virtual bool handle_left_push(const glm::vec2 & pt);
	virtual bool handle_left_release(const glm::vec2 & pt);
public:
	c_obj_cfg_box() : c_aws_ui_box(), range(1000), btn_pushed(nul), btn_released(nul), command(nul)
	{
		hstr.resize(nul);
		hbtn.resize(nul);
		check.resize(range_down, true);
	}

	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left);

	virtual const glm::vec2 get_box_size(const glm::vec2 sz_fnt)
	{
		glm::vec2 sz_btn, sz_box;
		int rows = nul / 3 + (nul % 3 ? 1 : 0);
		sz_btn.x = (float)(4 * sz_fnt.x);
		sz_btn.y = (float)(1.5 * sz_fnt.y);
		sz_box.x = (float)(sz_btn.x * 3);
		sz_box.y = (float)(sz_btn.y * (rows + 1));
		return sz_box;
	}

	virtual bool proc(const bool bpushed, const bool breleased);

	void set_params(const float _range, vector<bool> _check)
	{
		range = _range;
		if(range > 1000000)
			snprintf(range_str, 8, "%dM", (int)(range * 0.000001));
		else if (range > 1000)
			snprintf(range_str, 8, "%dK", (int)(range * 0.001));
		else
			snprintf(range_str, 8, "%d", (int)(range));
		potxt->set(hstr_range, range_str);

		for (int ibtn = 0; ibtn < range_down; ibtn++){
			check[ibtn] = _check[ibtn];
			if (check[ibtn]){
				set_checked_color(hbtn[ibtn], hstr[ibtn]);
			}
			else{
				set_normal_color(hbtn[ibtn], hstr[ibtn]);
			}
		}
		command = nul;
	}

	void get_params(float & _range, vector<bool> & _check)
	{
		_range = range;
		if (_check.size() != check.size())
			_check.resize(check.size());
		for (int ibtn = 0; ibtn < range_down; ibtn++){
			_check[ibtn] = check[ibtn];
		}
	}

	e_btn get_command()
	{
		return command;
	}
};

class c_route_cfg_box : public c_aws_ui_box
{
public:
	enum e_btn{
		wp_prev, wp_next,
		wp_spd_down, wp_spd_up,
		wp_add, wp_del,
		rt_prev, rt_next,
		rt_load, rt_save,
		nul
	};
private:
	static const char * str_btn[nul];
	vector<int> hstr;
	vector<int> hbtn;
	e_btn btn_pushed, btn_released;
	e_btn command;
	char str_val[8];
	int hstr_wp, hstr_spd, hstr_rt;
	unsigned int wp;
	unsigned int spd;
	unsigned int rt;
	virtual void open();
	virtual void close();
	virtual bool handle_left_push(const glm::vec2 & pt);
	virtual bool handle_left_release(const glm::vec2 & pt);
public:
	c_route_cfg_box() :c_aws_ui_box(), wp(0), spd(0), rt(0), btn_pushed(nul), btn_released(nul), command(nul)
	{
		hstr.resize(nul);
		hbtn.resize(nul);
	}

	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left);

	virtual const glm::vec2 get_box_size(const glm::vec2 sz_fnt)
	{
		glm::vec2 sz_btn, sz_box;
		sz_btn.x = (float)(5 * sz_fnt.x);
		sz_btn.y = (float)(1.5 * sz_fnt.y);

		sz_box.x = (float)(sz_btn.x * 2);
		sz_box.y = (float)(sz_btn.y * 5);
		return sz_box;
	}
	virtual bool proc(const bool bpushed, const bool breleased);

	const e_btn get_command()
	{
		return command;
	}

	void reset_command()
	{
		command = nul;
	}

	void command_processed(e_btn _command)
	{
		if (_command != nul)
		{
			set_normal_color(hbtn[_command], hstr[_command]);
		}
	}

	void set_params(const unsigned int _wp, const unsigned int _spd, const unsigned int _rt)
	{
		wp = _wp;
		snprintf(str_val, 8, "WP%03d", wp);
		potxt->set(hstr_wp, str_val);

		spd = _spd;
		snprintf(str_val, 8, "%03d", spd);
		potxt->set(hstr_spd, str_val);

		rt = _rt;
		snprintf(str_val, 8, "R%d", rt);
		potxt->set(hstr_rt, str_val);
	}

	void get_params(unsigned int & _wp, unsigned int & _spd, unsigned int & _rt)
	{
		_wp = wp;
		_spd = spd;
		_rt = rt;
	}
};

class c_indicator
{
public:
	// main/sub engine throttle
	// main/sub engine gear
	// rudder 
	// roll/pitch/yaw
	// cog/sog
private:
	e_ui_mode mode;
	c_gl_2d_obj * porect, * potri;
	c_gl_2d_line_obj * poline;
	c_gl_text_obj * potxt;

	const unsigned char veng_n, veng_nf, veng_nb;
	unsigned char meng, seng, rud;
	float cog /*radian*/, sog, yaw /*radian*/, pitch /*radian*/, roll /*radian*/;

	int hmeng_in, hmeng_out, hseng_in, hseng_out, hrud_in, hrud_out;
	int hmeng_n, hmeng_f, hmeng_b, hseng_n, hseng_f, hseng_b;
	glm::vec2 scl_eng, scl_rud, pos_rud;
	void create_engine_indicator(int & heng_in, int & heng_out, 
		int & heng_n, int & heng_f, int & heng_b, 
		glm::vec2 & pos, const glm::vec2 & sz_fnt,
		const glm::vec4 & clr);
	void update_engine_indicator(int & heng_in, int & heng_n, int & heng_f, int & heng_b,
		const unsigned char val);
	void create_rudder_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt,
		const glm::vec4 & clr);
	void update_rudder_indicator();
#define SOG_STEP 5
	glm::vec2 pos_sog;
	glm::vec2 rad_sog;
	int hsog_arc, hsog_scale, hstr_sog_scale[SOG_STEP], hsog_ptr;
	void create_sog_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt, const glm::vec4 & clr);
	void update_sog_indicator();

#define PITCH_STEP 9
#define ROLL_STEP 19
	glm::vec2 pos_rp;
	float lpmeas;
	int hrarc, hrscale, hstr_rscale[ROLL_STEP], hpmeas, hpscale, hstr_pscale[PITCH_STEP], hpptr;
	glm::vec2 pos_pscale[PITCH_STEP];
	void create_rp_indicator(glm::vec2 & pos, const glm::vec2 & sz_fnt, const glm::vec4 & clr);
	void update_rp_indicator();

#define YAW_STEP 36
	float dir_cam, pos_ystr, pos_yptr, fxcam;
	
	glm::vec2 pos_yscl[YAW_STEP], pos_yscl_tmp[YAW_STEP];
	int hhlzn, hyscale[YAW_STEP], hstr_yscale[YAW_STEP], hhptr, hcptr;
	void create_hc_indicator(const float fovx, const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const glm::vec4 & clr);
	void update_hc_indicator();
public:
	c_indicator();
	~c_indicator();
	bool init(c_gl_2d_line_obj * _poline, c_gl_text_obj * _potxt, 
		c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
		const glm::vec2 & sz_fnt, const glm::vec4 & clr, 
		const float fovx, const glm::vec2 & sz_scrn);

	void set_param(
		const unsigned char _meng, const unsigned char _seng, const unsigned char _rud,
		const float _cog /*radian*/, const float _sog,
		const float _yaw /*radian*/, const float _pitch/*radian*/, const float _roll/*radian*/);

	void set_dir_cam(const float _dir_cam)
	{
		dir_cam = _dir_cam;
	}

	void set_mode(e_ui_mode _mode = ui_mode_fpv)
	{
		mode = _mode;
	}
};

class c_ui_obj
{
protected:
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
	c_ui_obj() :mode(ui_mode_fpv)
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

class c_ui_waypoint_obj: public c_ui_obj
{
private:

	c_gl_2d_obj * pocirc;
	c_gl_text_obj * potxt;
	c_gl_2d_line_obj * poline;
	glm::vec4 clr;
	vector<s_wp> wps;
	struct s_marker{
		int hmark, hstr, hline_inf, hline_next;
	};
	vector<s_marker> hmarks;
	int focus, next;
	float dist, crs;
	int nmaxwps;
	float rmark;
public:
	c_ui_waypoint_obj();
	bool init(c_gl_2d_obj * pocirc, c_gl_text_obj * potxt, c_gl_2d_line_obj * poline, 
		const glm::vec4 & clr,  const glm::vec2 & sz_fnt, const float _rmark, 
		const unsigned int _nmaxwps = 100);
	void update_wps(const int iwp, const s_wp & wp);
	void update_drawings();
	void enable(const int iwp);
	void disable(const int iwp);
	void disable();
	void set_focus(const int iwp);
	void set_next(const int iwp, const float dist, const float crs);
	virtual int collision(const glm::vec2 pos);
};

class c_ui_ais_obj : public c_ui_obj
{
private:
	c_gl_2d_obj * porect, * potri;
	c_gl_text_obj * potxt;
	c_gl_2d_line_obj * poline;

	glm::vec2 sz_rect;
	glm::vec4 clr;
	struct s_marker{
		int hmark, hship2d, hstr, hline_inf, hline_vel;
	};
	vector<s_marker> hmarks;
	vector<c_ais_obj> objs;
	int nmax_objs;
	int focus;
	float tvel;
public:
	c_ui_ais_obj() : tvel(300)
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

class c_own_ship
{
private:
	c_gl_2d_obj * potri;
	c_gl_2d_line_obj * poline;
	int hship, hline_vel;
	float tvel;
public:
	c_own_ship() :tvel(300)
	{

	}

	bool init(c_gl_2d_obj * potri, c_gl_2d_line_obj * poline,
		const glm::vec4 & clr, const glm::vec2 & sz);
	void set_param(const float rx, const float ry, const float rz,
		const float hdg, const float vx, const float vy, const float pix_per_meter);
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

class c_aws_ui_box_manager
{
public:
	enum e_box{
		view_mode = 0,
		ctrl_mode = 1,
		obj_cfg = 2,
		route_cfg = 3,
		nul
	};
private:
	vector<c_aws_ui_box*> pboxes;
	vector<int> hboxes;

	e_box box_pushed, box_released, box_updated;
	bool blpushed, blreleased;
	c_gl_2d_obj * porect, *potri;
	c_gl_text_obj * potxt;
	c_gl_2d_line_obj * poline;

	glm::vec4 clr, bkgclr;
	glm::vec2 sz_font, sz_screen;
public:
	c_aws_ui_box_manager();
	~c_aws_ui_box_manager();

	bool init(c_gl_2d_obj * _porect, c_gl_2d_obj * _potri,
		c_gl_text_obj * _potxt, c_gl_2d_line_obj * poline, 
		const glm::vec4 & _clr, const glm::vec4 & _bkgclr, const glm::vec2 & _sz_font, const float fovx, const glm::vec2 & _sz_screen);

	bool set_mouse_event(const glm::vec2 & pt, const int button, const int action, const int modifier);

	c_aws_ui_box * get_ui_box(const e_box & box)
	{
		return pboxes[box];
	}

	e_box get_box_updated()
	{
		return box_updated;
	}
};


class f_aws1_ui: public f_glfw_window
{
public:
	enum e_obj_type{
		ot_wp, ot_cl, ot_ais, ot_mark, ot_nul
	};
	struct s_obj{
		e_obj_type type;
		int handle;
		s_obj():type(ot_nul), handle(-1){}
	};

	enum e_mouse_state{
		ms_add_wp, ms_drag, ms_normal
	} mouse_state;

 private:
	 void cnv_img_to_view(Mat & img, float av, Size & sz, bool flipx, bool flipy);
	 s_aws1_ctrl_inst m_inst;
	 s_aws1_ctrl_stat m_stat;

	 // send control packet to m_acd_socket or m_ch_ctrl_out
	 void snd_ctrl_inst();
	 // Recive control state (rudder angle) from m_acd_socket or m_ch_ctrl_in.
	 void rcv_ctrl_stat();

	 ////////////////////////////////////////// current version
  ch_state * m_state;
  ch_aws1_sys * m_ch_sys;
  ch_aws1_ctrl_inst * m_ch_ctrl_inst;
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;
  ch_wp * m_ch_wp;
  ch_map * m_ch_map;
  ch_obj * m_ch_obj;
  ch_ais_obj * m_ch_ais_obj;
  ch_image_ref * m_ch_img, * m_ch_img2, * m_ch_disp;
  ch_obst * m_ch_obst;
  ch_aws1_ap_inst * m_ch_ap_inst;

  char m_path_storage[1024];

  // shader related members
  char fvs[1024], ffs[1024], ftex[1024], ftexinf[1024];
  GLuint p;
  GLuint loc_mode, loc_gcolor, loc_gcolorb, loc_pos2d, loc_inv_sz_half_scrn,
	  loc_Mmvp, loc_Mm, loc_Lpar, loc_sampler, 
	  loc_depth2d, loc_position, loc_normal, loc_texcoord;
  float inv_sz_half_scrn[2], fov_cam_x, fov_cam_y, fcam, ifcam, height_cam, dir_cam_hdg, dir_cam_hdg_drag;
  float iRE, height_cam_ec, dhorizon_cam, dhorizon_arc, zhorizon, th_horizon;
  bool setup_shader();

  // visual elements
  c_gl_2d_obj orect, otri, ocirc;
  c_gl_text_obj otxt;
  c_gl_2d_line_obj oline;
  void render_gl_objs()
  {
	  orect.render();
	  otri.render();
	  ocirc.render();
	  otxt.render(0);
	  oline.render();
  }

  // ui elements
  c_aws_ui_box_manager uim;
  c_indicator ind;
  float sz_mark;
  c_ui_waypoint_obj owp;
  int num_max_wps;
  void update_route();

  c_ui_ais_obj oais;
  int num_max_ais;
  void update_ais_objs();
  vector<bool> visible_obj;
  c_own_ship own_ship;
  c_cursor ocsr;

  void update_view_mode_box(c_view_mode_box * pvm_box);
  void update_ctrl_mode_box(c_ctrl_mode_box * pcm_box);
  void update_obj_cfg_box(c_obj_cfg_box * poc_box);
  void update_route_cfg_box(c_route_cfg_box * prc_box, e_mouse_state mouse_state_new);

  // joypad related members
  bool m_verb;
  s_jc_u3613m m_js;
  int m_js_id;
  const char * m_js_name;
  float m_rud_f, m_meng_f, m_seng_f;
  enum e_ctrl_mode{
	  cm_crz, cm_ctl, cm_csr
  } ctrl_mode;

  void handle_ctrl_crz(); // cruise mode: sticks are used to increase/decrease engine throttle and rudder angle 
  void handle_ctrl_ctl(); // control mode: positions of sticks are the throttle values. 
  void handle_ctrl_csr(); // cursor mode: follows cursor position 

  // mouse related members
  glm::vec2 pt_mouse, pt_mouse_drag_begin, pt_mouse_bih;
  glm::vec3 pt_mouse_ecef, pt_mouse_enu;
  int mouse_button, mouse_action, mouse_mods;
  s_obj obj_mouse_on;

  void calc_mouse_enu_and_ecef_pos(e_ui_mode vm, Mat & Rown,
	  const float lat, const float lon, 
	  const float xown, const float yown, const float zown, const float yaw);
  void handle_mouse_lbtn_push(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	  c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box);
  void handle_mouse_lbtn_up(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	  c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box);
  void handle_mouse_mv(c_view_mode_box * pvm_box, c_ctrl_mode_box * pcm_box,
	  c_obj_cfg_box * poc_box, c_route_cfg_box * prc_box);
  void handle_mouse_drag(c_view_mode_box * pvm_box, s_obj & obj_tmp);
  void add_waypoint(c_route_cfg_box * prc_box);
  void drag_waypoint();
  void drag_cam_dir();
  void drag_map();
  void det_obj_collision();

  // map related members
  bool bmap_center_free;
  Mat Rmap;
  glm::vec2 pt_map_center_bih;
  glm::vec3 pt_map_center_ecef;
  float map_range, meter_per_pix, pix_per_meter;
  void recalc_range()
  {
	  meter_per_pix = (float)(map_range / (float)(m_sz_win.height >> 1));
	  pix_per_meter = (float)(1.0 / meter_per_pix);
  }

  // FPV related member
  glm::mat4 pm, vm, pvm;

  // video/screen capture related members
  VideoWriter m_vw;
  bool m_bsvw; // screen video write
  bool m_bss; // screen shot
  Mat m_simg;	// screen img
  void write_screen();

  // glfw event handler 
  virtual void _cursor_position_callback(double xpos, double ypos);
  virtual void _mouse_button_callback(int button, int action, int mods);
  virtual void _key_callback(int key, int scancode, int action, int mods);

 public:
  f_aws1_ui(const char * name);
  virtual ~f_aws1_ui();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();

  // If LT+LB+RT+RB is detected, the system forces the controls to be nutral state. Called by default.
  void ui_force_ctrl_stop(c_ctrl_mode_box * pcm_box);
  bool m_quit;
};

//////////////////////////////////////////////////////// f_aws1_ui_test
// This class is a test class interfacing only with f_aws1_ui via the compatible channel set.
// This class is used to manipulate the channel values by fset parameters. 
class f_aws1_ui_test: public f_base
{
protected:
	ch_state * m_state;
	ch_ais_obj * m_ch_ais_obj;
	ch_aws1_ctrl_stat * m_ch_ctrl_stat;

	ch_aws1_ctrl_inst * m_ch_ctrl_ui, * m_ch_ctrl_ap1, * m_ch_ctrl_ap2;
	ch_image_ref * m_ch_img;

	bool m_ahrs, m_gps; 
	float r, p, y; // roll(deg), pitch(deg), yaw(deg)
	float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	float cog, sog; // Course over ground(deg), Speed over ground (kts)
	float depth; // water depth
	s_aws1_ctrl_stat m_stat;

	bool m_add_ais_ship;
	unsigned int ais_mmsi;
	float ais_lat, ais_lon, ais_cog, ais_sog, ais_yaw;

	float m_rud_sta_sim;
public:
	f_aws1_ui_test(const char * name);

	void select_control_input()
	{
		// Control input selection
		s_aws1_ctrl_inst acp;
		if(m_ch_ctrl_ui)
			m_ch_ctrl_ui->get(acp);
		m_stat.tcur = acp.tcur;
		m_stat.ctrl_src = acp.ctrl_src;
		switch(m_stat.ctrl_src){
		case ACS_UI:
			m_stat.rud_aws = acp.rud_aws;
			m_stat.meng_aws = acp.meng_aws;
			m_stat.seng_aws = acp.seng_aws;
			break;
		case ACS_AP1:
			if(m_ch_ctrl_ap1){
				m_ch_ctrl_ap1->get(acp);
				m_stat.rud_aws = acp.rud_aws;
				m_stat.meng_aws = acp.meng_aws;
				m_stat.seng_aws = acp.seng_aws;
			}
			break;
		case ACS_AP2:
			if(m_ch_ctrl_ap2){
				m_ch_ctrl_ap2->get(acp);
				m_stat.rud_aws = acp.rud_aws;
				m_stat.meng_aws = acp.meng_aws;
				m_stat.seng_aws = acp.seng_aws;
			}
			break;
		default:
			break;
		}
	}

	void simulate_rudder(){
		// Rudder response simulation
		unsigned rud_inst = map_oval(m_stat.rud,
			m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min,
			m_stat.rud_sta_max, m_stat.rud_sta_nut, m_stat.rud_sta_min);
#define RUD_PER_CYCLE 0.45f
		if(rud_inst > m_stat.rud_sta){
			m_rud_sta_sim += RUD_PER_CYCLE;
		}else{
			m_rud_sta_sim -= RUD_PER_CYCLE;
		}

		m_stat.rud_sta = (unsigned char) m_rud_sta_sim;
	}

	void set_control_output()
	{
		switch(m_stat.ctrl_src){
		case ACS_UI:
		case ACS_AP1:
		case ACS_AP2:
		case ACS_FSET:
		case ACS_NONE:
			m_stat.rud = map_oval(m_stat.rud_aws, 
				0xff, 0x7f, 0x00, 
				m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
			m_stat.meng = map_oval(m_stat.meng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, 
				m_stat.meng_nub, m_stat.meng_min);  
			m_stat.seng = map_oval(m_stat.seng_aws, 
				0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
				m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, 
				m_stat.seng_nub, m_stat.seng_min);
			break;
		case ACS_RMT:
			m_stat.rud = map_oval(m_stat.rud_rmc, 
				m_stat.rud_max_rmc, m_stat.rud_nut_rmc, m_stat.rud_min_rmc,
				m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
			m_stat.meng = map_oval(m_stat.meng_rmc, 
				m_stat.meng_max_rmc, m_stat.meng_nuf_rmc, m_stat.meng_nut_rmc, 
				m_stat.meng_nub_rmc, m_stat.meng_min_rmc,
				m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, m_stat.meng_nub, 
				m_stat.meng_min);  
			m_stat.seng = map_oval(m_stat.seng_rmc, 
				m_stat.seng_max_rmc, m_stat.seng_nuf_rmc, m_stat.seng_nut_rmc, 
				m_stat.seng_nub_rmc, m_stat.seng_min_rmc,
				m_stat.seng_max, m_stat.seng_nuf, m_stat.seng_nut, m_stat.seng_nub, 
				m_stat.seng_min);
			break;
		}
		if(m_ch_ctrl_stat){
			m_ch_ctrl_stat->set(m_stat);
		}
	}

	void set_state(){
		if(m_state){
			long long t = 0;
			if(!m_ahrs)
				m_state->set_attitude(t, r, p, y);

			if(!m_gps){
				m_state->set_position(t, lat, lon, alt, galt);		
				m_state->set_velocity(t, cog, sog);
			}

			m_state->set_depth(t, depth);
		}
	}

	void simulate_dynamics()
	{
	}

	void add_ais_ship()
	{
		if(m_ch_ais_obj && m_add_ais_ship){
			m_add_ais_ship = false;
			m_ch_ais_obj->push(m_cur_time, ais_mmsi, ais_lat, ais_lon, ais_cog, ais_sog, ais_yaw);
			if(m_state){
				long long t = 0;
				Mat R = m_state->get_enu_rotation(t);
				float x, y, z;
				m_state->get_position_ecef(t, x, y, z);
				m_ch_ais_obj->update_rel_pos_and_vel(R, x, y, z);
			}
		}
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};
#endif
