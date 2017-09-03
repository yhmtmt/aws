// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// c_ui_box.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_ui_box.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_ui_box.h.  If not, see <http://www.gnu.org/licenses/>. 

////////////////////////////////////////////////////////////////////////////////// ui control box
// The base class c_aws_ui_box provides some basic methods: constructing buttons, handling event's , and changing the looks.
// currentlly four subclasses are created.  : c_view_mode_box, c_ctrl_mode_box, c_map_cfg_box, c_route_cfg_box
class c_aws_ui_box
{
protected:
	static c_gl_2d_obj * porect, *potri;
	static c_gl_2d_line_obj * poline;
	static c_gl_text_obj * potxt;
	bool bopened;
	int hbox, hopen, hclose;
	bool btn_oc_pushed, btn_oc_released;
	glm::vec2 pt_mouse;
	glm::vec2 pos_close, pos_open; // box position
	glm::vec2 sz_close, sz_open; // box size

	glm::vec4 clr, bkgclr;

	// setup frame of the box. frame can be constructed at left or right side of the screen.
	void setup_frame(const float y, const bool left, const glm::vec2 & sz_scrn,
		const glm::vec2 &sz_box, const glm::vec2  & sz_fnt, const glm::vec4 & clr);

	// create a button to the box
	void add_btn(int & hbtn, int & hstr, const char * str,
		const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_fnt);

	// create a select box includes two buttons to increment or decrement the value, and an box indicating the value.
	void add_select_box(int & hlbtn, int & hlstr, const char * lstr, int & hrbtn, int & hrstr, const char * rstr, int & hvalstr,
		const glm::vec2 & pos, const glm::vec2 & sz_btn, const glm::vec2 & sz_box, const glm::vec2 & sz_fnt, const unsigned int len_str);

	// set the color of the rectangle as seletcted one.
	void set_selected_color(const int hrect);

	// set the color of the rectangle box as normal one.
	void set_normal_color(const int hrect);

	// set the color of the button as checked one.  
	void set_checked_color(const int hbtn, const int hstr);

	// set the color of the buton as normal one.
	void set_normal_color(const int hbtn, const int hstr);

	// open the box
	virtual void open();

	// close the box
	virtual void close();

	// mouse event handler handles left button push. The function only assert the flag which button is pressed.
	virtual bool handle_left_push(const glm::vec2 & pt);

	// mouse event handler handles left button release. The function only assert the flag which button the mouse cursor is on when the button is released.
	virtual bool handle_left_release(const glm::vec2 & pt);

public:
	c_aws_ui_box() : bopened(false), hbox(-1), hopen(-1), hclose(-1), btn_oc_pushed(false), btn_oc_released(false)
	{
	}

	// initialization method. sub class should call this at the final line.
	virtual bool init(const int handle, const glm::vec4 & clr, const glm::vec4 & bkgclr,
		const glm::vec2 & sz_fnt, const glm::vec2 & sz_scrn, const float y, const bool left)
	{
		setup_frame(y, left, sz_scrn, get_box_size(sz_fnt), sz_fnt, clr);

		bopened = false;
		close();
		return true;
	};

	// the method returns the box size.
	virtual const glm::vec2 get_box_size(const glm::vec2 sz_font)
	{
		return sz_open;
	};

	// set mouse event with glfw's modifier codes. The method calls handle_left_push and handle_left_release.
	// The method only assert the flag which button is pressed or released if the cursor is hitting that.
	bool set_mouse_event(const glm::vec2 & pt,
		const int button, const int action, const int modifier);

	// The method should implement the action to the flag asserted by set_mouse_event. 
	// In the base class, only open/close are handled.
	virtual bool proc(const bool bpushed, const bool breleased);

	// Setup the drawing resources. This should be called before initializing the instance of the class and the subclasses. 
	static void set_gl_objs(c_gl_2d_obj * _porect, c_gl_2d_obj * potri,
		c_gl_text_obj * _potxt, c_gl_2d_line_obj * _poline);
};

// c_view_mode_box is to be used for selecting the view mode. Currently three modes are implemented.
// 1. FPV, "First Person View" mode, provides the vision viewing from my own ship.
// 2. Map mode provides the 2D view looking from the sky.
// 3. SYS mode provides system menu. This blocks all the ui boxes, the use should be restricted with certain safety conditions.
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

// c_ctrl_mode_box provides the capability to select the controling mode of my own ship.
// There are six controlling modes planned to be implemented.
// CRZ, the cruise mode, provides the controllability suite for high speed cruise.
// CTL, the control mode, provides the slow and accurate controllability.
// CSR, the cursor mode, provides the controllability that the own ship follows the map point the mouse cursor is pointing on. 
// FWP, the follow waypint mode, makes my own ship follows a set of waypoints.
// STY, the stay mode,  makes my own ship staying at the current point and pose. 
// FTG, the follow target mode, makes my own ship to follow a target selected.
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

// c_map_cfg_box configures the objects to be drawn on the view. 
// Currentlly four objects and the range can be configured.
// wp: Waypoints, vsl: Vessel includes AIS and others detected by sensors, mrk: Navigational marks.
// cl: Coastline.
class c_map_cfg_box : public c_aws_ui_box
{
public:
	enum e_btn{
		wp = 0,		// waypoint
		vsl,	// vessel
		mrk,	// mark
		cl,		// coast line
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
	c_map_cfg_box() : c_aws_ui_box(), range(1000), btn_pushed(nul), btn_released(nul), command(nul)
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
		if (range > 1000000)
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

// c_route_cfg_box edits, saves and loads a route, a set of waypoints.
// * Identification numbers of waypoints and routes can be selected.
// * Once the add button pushed, mouse is captured and the cursor position of the  next click is set as the new way point. 
// * New waypoint is created with the identification number selected 
// * Delete button deletes a waypoint selected. 
// * Save button saves a set of waypoints currentlly on the map as the route of the selected identification number.
// * Load button loads a set of waypoints saved as the route of the selected identification number.
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

/////////////////////////////////////////////////////////////////////// Container for ui boxes
// Creating and managing four ui_boxes.
class c_aws_ui_box_manager
{
public:
	enum e_box{
		view_mode = 0,
		ctrl_mode = 1,
		map_cfg = 2,
		route_cfg = 3,
		nul
	};
private:
	vector<c_aws_ui_box*> pboxes;
	vector<int> hboxes;

	// Following four members are used in set_mouse_event and cleared when the mouse left button released
	e_box
		box_pushed,  // the box where the mouse left button clicked
		box_released;// the box where the mouse left button released
	bool blpushed;  // asserted when the mouse is clicked on the boxes.
	bool blreleased;// asserted when the mouse is released.

	e_box box_updated; // the box updated in set_mouse_event()

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
		const glm::vec4 & _clr, const glm::vec4 & _bkgclr,
		const glm::vec2 & _sz_font, const float fovx, const glm::vec2 & _sz_screen);

	bool set_mouse_event(const glm::vec2 & pt, const int button, const int action, const int modifier);

	c_aws_ui_box * get_ui_box(const e_box & box)
	{
		return pboxes[box];
	}

	e_box get_box_updated()
	{
		return box_updated;
	}

	void reset_box_updated()
	{
		box_updated = nul;
	}
};

/////////////////////////////////////////////////////////////////////////// State indicator 
// Indicates main/sub engine's throttle/gear, rudder angle, ship states (roll/pitch/yaw/cog/sog)
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
	c_gl_2d_obj * porect, *potri;
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
