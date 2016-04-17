// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// c_aws1_ui_map.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws1_ui_map.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws1_ui_map.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include <GL/glew.h>

#include <GLFW/glfw3.h>
#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glut.h>
#include <GL/glu.h>

#include "f_aws1_ui.h"

/////////////////////////////////////////////////////////////////////////// c_aws1_ui_map

c_aws1_ui_map::c_aws1_ui_map(f_aws1_ui * _pui):c_aws1_ui_core(_pui), m_op(EMO_EDT_WP),
	m_map_range(10000)
{
	m_cur_pos.x = m_cur_pos.y = 0.;
	m_map_pos.x = m_map_pos.y = 0.;

	// points for the ship object
	m_ship_pts[0].x = 0.f;
	m_ship_pts[0].y = 1.f;
	m_ship_pts[1].x = 0.5f;
	m_ship_pts[1].y = -1.f;
	m_ship_pts[2].x = -0.5f;
	m_ship_pts[2].y = -1.f;

	// calculating circular points
	const float step = (float)(2 * PI / 36.);
	float theta = 0.;
	for (int i = 0; i < 36; i++, theta += step){
		m_circ_pts[i].x = (float)cos(theta);
		m_circ_pts[i].y = (float)sin(theta);
	}
}


void c_aws1_ui_map::js(const s_jc_u3613m & js)
{
	// Operation selection (cross key)

	// Cursor move (right stick)
	m_cur_pos.x += (float)(js.lr2 * (1.0/60.0));
    m_cur_pos.y += (float)(js.ud2 * (1.0/60.0) );		

	// Map move (left stick)	
	m_map_pos.x += (float)(js.lr2 * (1.0/60.0));
    m_map_pos.y += (float)(js.ud2 * (1.0/60.0) );	
	if(js.elst & s_jc_u3613m::EB_STDOWN){ // back to the own ship 
		m_map_pos.x = m_map_pos.y = 0.;
	}

}

void c_aws1_ui_map::draw(float xscale, float yscale)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	const Size & sz_win = get_window_size();
	float lw = xscale;

	float fxmeter, fymeter, ifxmeter, ifymeter;

	if(yscale > xscale){ // 1 in y direction equals to m_map_range
		fxmeter = (float)(((float) sz_win.height / (float)sz_win.width) * m_map_range);
		fymeter = m_map_range;
	}else{ // 1 in x direction equals to m_map_range
		fxmeter = m_map_range;
		fymeter =  (float)(((float) sz_win.width / (float)sz_win.height) * m_map_range);
	}

	ifxmeter = (float)(1.0 / fxmeter);
	ifymeter = (float)(1.0 / fymeter);

	// draw circle centered at my own ship 
	{
		Point2f pts[36]; // scaled points
		for(int i = 0; i < 36; i++){
			pts[i].x = m_circ_pts[i].x * fxmeter;
			pts[i].y = m_circ_pts[i].y * fymeter;
		}

		drawGlPolygon2Df(pts, 10, 0, 1, 0, 0, lw); // own ship triangle
	}

	// draw own ship
	{
		Point2f offset = Point2f((float)(-m_map_pos.x * ifxmeter), (float)(-m_map_pos.y * ifymeter));
		ch_state * pstate = get_state();
		float cog, sog, roll, pitch, yaw;
		Point2f pts[3]; // rotated version of the ship points
		pstate->get_velocity(cog, sog);
		pstate->get_attitude(roll, pitch, yaw);
		float theta = (float)(yaw * (PI / 180.));
		float c = (float) cos(theta), s = (float) sin(theta);
		float ws = (float)(xscale * 10), hs = (float)(yscale * 10);
		for(int i = 0; i < 3; i++){
			// rotating the points
			// multiply [ c -s; s c]
			pts[i].x = (float)(ws * (m_ship_pts[i].x * c - m_ship_pts[i].y * s + offset.x));
			pts[i].y = (float)(hs * (m_ship_pts[i].x * s + m_ship_pts[i].y * c + offset.y));
		}

		drawGlPolygon2Df(pts, 3, 0, 1, 0, 0, lw); // own ship triangle

		theta = (float)(cog * (PI / 180.));
		c = (float)cos(theta);
		s = (float)sin(theta);
		float v = (float)(sog * KNOT * 180.); // vector is 180 sec length
		float vy = v * c * ifxmeter; 
		float vx = v * s * ifymeter;
		drawGlLine2Df(offset.x, offset.y, (float)(offset.x + vx), (float)(offset.y + vy), 0., 1., 0., 0., lw);
	}

	// draw waypoints

	// draw objects

	// draw operation lists (right bottom)

	// draw Engine and Rudder status 
	pui->ui_show_meng(xscale, yscale);
	pui->ui_show_seng(xscale, yscale);
	pui->ui_show_rudder(xscale, yscale);

	// draw own ship status
	pui->ui_show_state(xscale, yscale);
}

void c_aws1_ui_map::key(int key, int scancode, int action, int mods)
{
}
