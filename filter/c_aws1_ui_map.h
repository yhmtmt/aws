// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_map.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_UI_MAP_H_
#define _F_AWS1_UI_MAP_H_




// provides 2D map of objects and map near around 
class c_aws1_ui_map: public c_aws1_ui_core
{
protected:
	enum e_map_operation{
		EMO_EDT_WP, EMD_EDT_MAP
	} m_op;

	// EMD_EDT_WP: Edit way point
	//		A: Add waypoint to the cursor point
	//		B: Delete selected waypoint 
	//		Left/Right: Waypoint selection

	// EMD_EDT_MAP: Edit map (Still not defined)
	//

	// Window scale parameters
	float fx, fy, fxmeter, fymeter, ifxmeter, ifymeter;

	void pix2mtr(const float xpix, const float ypix, float & xmtr, float & ymtr){		
	}

	void mtr2pix(const float xmtr, const float ymtr, float & xpix, float & ypix){
	}

	void nml2mtr(const float xnml, const float ynml, float & xmtr, float & ymtr){
		xmtr = (float)(fxmeter * xnml);
		ymtr = (float)(fymeter * ynml);
	}

	void mtr2nml(const float xmtr, const float ymtr, float & xnml, float & ynml){
		xnml = (float)(ifxmeter * xmtr);
		ynml = (float)(ifymeter * ymtr);
	}

	// Own ship coordinate transformation 
	Mat Rorg;		// Rotation matrix of the own ship coordinate
	Point3f Porg;   // Origin of the own ship coordinate

	Point2f m_ship_pts[3];
	Point2f m_circ_pts[36];

	float m_map_range; // range in meter (default 10km)

	Point2f m_cur_pos; // in pixel
	float cp_rx, cp_ry;
	float cp_x, cp_y, cp_z;
	float cp_lat, cp_lon, cp_alt;

	Point2f m_map_pos; // relative position to my own ship (in meter)
	float mp_x, mp_y, mp_z;
	float mp_lat, mp_lon, mp_alt;

	Point2f offset;

	void draw_ship_object(c_obj & obj);

public:
	c_aws1_ui_map(f_aws1_ui * _pui);

	virtual ~c_aws1_ui_map()
	{
	}

	virtual void js(const s_jc_u3613m  & js);
	virtual void draw();
	virtual void key(int key, int scancode, int action, int mods);
};

#endif
