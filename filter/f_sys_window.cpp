#include "stdafx.h"

// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_sys_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sys_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sys_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <WindowsX.h>
#ifdef _WIN32
#include <DShow.h>
#include <uuids.h>

//#define DS_DEBUG

//#include <Qedit.h>

// From http://msdn2.microsoft.com/en-us/library/ms786691.aspx:

// Include Qedit.h. This header file is not compatible with Microsoft Direct3D headers later than version 7.

// Since we are using DX9, we cannot include this header. Necessary API elements, copied below.

EXTERN_C const CLSID CLSID_SampleGrabber;

EXTERN_C const CLSID CLSID_NullRenderer;

EXTERN_C const IID IID_ISampleGrabberCB;

MIDL_INTERFACE("0579154A-2B53-4994-B0D0-E773148EFF85")

ISampleGrabberCB : public IUnknown {
public:
	virtual HRESULT STDMETHODCALLTYPE SampleCB( double SampleTime,IMediaSample *pSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE BufferCB( double SampleTime,BYTE *pBuffer,long BufferLen) = 0;
};

EXTERN_C const IID IID_ISampleGrabber;

MIDL_INTERFACE("6B652FFF-11FE-4fce-92AD-0266B5D7C78F")

ISampleGrabber : public IUnknown {
public:
	virtual HRESULT STDMETHODCALLTYPE SetOneShot( BOOL OneShot) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetMediaType( const AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType( AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetBufferSamples( BOOL BufferThem) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer( /* [out][in] */ long *pBufferSize,/* [out] */ long *pBuffer) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentSample( /* [retval][out] */ IMediaSample **ppSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetCallback( ISampleGrabberCB *pCallback,long WhichMethodToCallback) = 0;
};


#if WINVER != 0x603 && WINVER != 0x602
// if not windows 8, the direct 3d is not included in the windows sdk.
//#include <d2d1.h>
//#include <dwrite.h>
#include <d3d9.h>
#endif

#include <d3dx9.h>

#endif
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_sys_window.h"

///////////////////////////////////////////////////////////////// f_sys_window
#define ALPHA 0.1

f_sys_window::f_sys_window(const char * name):f_ds_window(name), 
	m_prev_time(0), m_avg_cycle_time(0), m_tz(9), 
	m_bdisp(true), m_grid_width(-1),
	m_grid3d_width(-1.0), m_own_mmsi(-1),
	m_btrck(false), m_sz_map(300, 300), m_sz_subcam(1600/3, 300)
{
	// initialize geometry parameters
	m_Rown = Mat::zeros(3, 3, CV_64FC1);
	m_Rwrld = Mat::zeros(3, 3, CV_64FC1);
};

f_sys_window::~f_sys_window()
{
	release_d3dres();
}

bool f_sys_window::alloc_d3dres()
{	
	HRESULT hr;

	hr = m_pd3dev->CreateOffscreenPlainSurface(
		m_ViewPort.Width,
		m_ViewPort.Height, 
		D3DFMT_X8R8G8B8, D3DPOOL_SYSTEMMEM, &m_pgrabsurf, NULL);

	if(FAILED(hr)){
		cerr << "Failed to create surface for grab" << endl;
		return false;
	}

	if(!m_d3d_txt.init(m_pd3dev,
		(float) (m_ViewPort.Width),
		(float) (m_ViewPort.Height), 18))
		return false;

	if(!m_d3d_ship2d.init(m_pd3dev, 
		(float) m_sz_map.width, (float) m_sz_map.height, 16))
		return false;

	if(!m_map.init(m_pd3dev, (float) m_sz_map.width, (float) m_sz_map.height, 
		(float) (m_ViewPort.Width),
		(float) (m_ViewPort.Height)))
		return false;

	int hsize, vsize;
	Mat Cam;
	s_rotpar rot;
	Point3d Pos;

	if(m_chin[8] != NULL){
		ch_campar * psubcpin = dynamic_cast<ch_campar*>(m_chin[8]);
		if(psubcpin == NULL){
			cerr << "8th channel is bad in " << m_name << endl;
			return false;
		}

		psubcpin->get_Size(hsize, vsize);
		psubcpin->get_Int(Cam);
		psubcpin->get_pos(Pos);
		psubcpin->get_rot(rot);
		if(!m_subcam.init(m_pd3dev, Cam, rot, Pos, (float) hsize, (float) vsize, 
			(float) m_sz_subcam.width, (float) m_sz_subcam.height,
			(float) (m_ViewPort.Width), (float) m_ViewPort.Height))
			return false;
	}

	if(m_chin[1] == NULL)
		return false;

	ch_campar * pmaincpin = dynamic_cast<ch_campar*>(m_chin[1]);
	if(pmaincpin == NULL){
		cerr << "1st channel is bad in " << m_name << endl;
		return false;
	}

	pmaincpin->get_Size(hsize, vsize);
	pmaincpin->get_Int(Cam);
	pmaincpin->get_pos(Pos);
	pmaincpin->get_rot(rot);
	if(!m_maincam.init(m_pd3dev, Cam, rot, Pos, (float) hsize, (float) vsize, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height))
		return false;

	D3DXCreateLine(m_pd3dev, &m_pline);

	c_ship::load_std_model(m_pd3dev);

	return true;
}


void f_sys_window::release_d3dres()
{
	if(m_pgrabsurf != NULL){
		m_pgrabsurf->Release();
		m_pgrabsurf = NULL;
	}

	m_d3d_ship2d.release();
	m_d3d_txt.release();
	m_map.release();
	m_subcam.release();
	m_maincam.release();
	if(m_pline != NULL){
		m_pline->Release();
		m_pline = NULL;
	}
	c_ship::release_std_model();
}


bool f_sys_window::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "trck") == 0){
		if(num_args == 5){ // on
			m_trck_size.cx = atoi(args[itok+1]);
			m_trck_size.cy = atoi(args[itok+2]);
			m_btrck = true;
		}else{
			m_btrck = false;
		}
		return true;
	}else if(strcmp(args[itok], "disp") == 0){
		m_bdisp = !m_bdisp;
		return true;
	}else if(strcmp(args[itok], "grid") == 0){
		if(num_args == 4)
			m_grid_width = atoi(args[itok+1]);
		else 
			m_grid_width = -1;
		return true;
	}else if(strcmp(args[itok], "grid3d") == 0){
		if(num_args == 4)
			m_grid3d_width = atoi(args[itok+1]);
		else
			m_grid3d_width = -1.0;
		return true;
	}else if(strcmp(args[itok], "own_mmsi") ==0){
		if(num_args == 4)
			m_own_mmsi = atoi(args[itok+1]);
		else
			m_own_mmsi = -1;

		c_ship::set_mmsi_own(m_own_mmsi);

		return true;
	}

	return f_ds_window::cmd_proc(cmd);
}

bool f_sys_window::check()
{
	return m_chin[0] != NULL;
}

bool f_sys_window::proc()
{
	unique_lock<mutex> lock(m_d3d_mtx);
//	pthread_lock lock(&m_d3d_mtx);

	float dt = (float) ((m_cur_time - m_prev_time) * 1e-7);
	m_avg_cycle_time = ALPHA * dt + (1 - ALPHA) * m_avg_cycle_time;
	////////////////// updating pvt information ///////////////////////
	update_pvt(dt);

	ch_navdat * pshipin = dynamic_cast<ch_navdat*>(m_chin[4]);
	// update heading
	if(pshipin != NULL){
		if(pshipin->is_valid()){
			m_rot_own.yaw = pshipin->m_hdg;
			pshipin->get_done();
		}

	}else{
		m_rot_own.yaw = m_crs;
	}

	// this is the temporal code. ship translation is set to zero.
	m_pos_own = Point3d(0., 0., 0.);

	////////////////// update cam par ///////////////////////////////
	Point3d pos;
	Mat Int;
	int hsize, vsize;
	s_rotpar rot;
	if(m_chin[8] != NULL){
		ch_campar * psubcpin = dynamic_cast<ch_campar*>(m_chin[8]);
		if(psubcpin == NULL){
			cerr << "8th channel is bad in " << m_name << endl;
			return false;
		}

		if(psubcpin->get_pos(pos)){
			m_subcam.set_campar(pos);
		}

		if(psubcpin->get_rot(rot)){
			m_subcam.set_campar(rot);
		}

		if(psubcpin->get_Int(Int)){
			m_subcam.set_campar(Int);
		}

		if(psubcpin->get_Size(hsize, vsize)){
			m_subcam.release();
			if(!m_subcam.init(m_pd3dev, Int, rot, pos, (float) hsize, (float) vsize, 
				(float) m_sz_subcam.width, (float) m_sz_subcam.height,
				(float) (m_ViewPort.Width), (float) m_ViewPort.Height))
				return false;
		}

		// This is the temporal code.
		// translation between camera center and pan-tilt center set as zero.
		pos = Point3d(0., 0., 0.); 
		m_subcam.set_campar(pos, true);

		/////////////////// update ptz control of ptz camera /////////////
		ch_ptz * ptzin = dynamic_cast<ch_ptz*>(m_chin[6]);
		if(ptzin == NULL){
			cerr << "6th channel is bad in " << m_name << endl;
			return false;
		}

		unsigned short p,z;
		short t;
		if(ptzin->get(p, t, z)){
			//		cout << "PTZ:" << p << "," << t << "," << z << endl;
			rot.yaw = -0.01 * (float) p * PI/180;
			rot.pitch = 0.01 * (float) t * PI/180.;
			rot.roll = 0.;
			m_subcam.set_campar(rot, true);
		}
	}

	ch_campar * pmaincpin = dynamic_cast<ch_campar*>(m_chin[1]);
	if(pmaincpin == NULL){
		cerr << "1st channel is bad in " << m_name << endl;
		return false;
	}

	if(pmaincpin->get_pos(pos)){
		m_maincam.set_campar(pos);
	}

	if(pmaincpin->get_rot(rot)){
		m_maincam.set_campar(rot);
	}

	if(pmaincpin->get_Int(Int)){
		m_maincam.set_campar(Int);
	}

	if(pmaincpin->get_Size(hsize, vsize)){
		m_maincam.release();
		if(!m_maincam.init(m_pd3dev, Int, rot, pos, (float) hsize, (float) vsize, 
			(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
			(float) m_ViewPort.Width, (float) m_ViewPort.Height))
			return false;
	}

	////////////////// loading ais data //////////////////////////////
	update_ais(m_cur_time);

	//////////////////// clear back buffer ///////////////////////////
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);

	/////////////////////rendering sequence/////////////////////////// 
	render();

	////////////////////// grab rendered back surface ////////////////
	if(m_grab_name)
		grab();

	////////////////////// presentation //////////////////////////////
	if(m_pd3dev->Present(NULL, NULL, 
		NULL, NULL) == D3DERR_DEVICELOST){
			cerr << "device lost" << endl;
			m_blost = true;
	}

	m_prev_time = m_cur_time;

	return true;
}

void f_sys_window::update_pvt(float dt)
{
	ch_pvt * ppvtin = dynamic_cast<ch_pvt*>(m_chin[2]);
	if(ppvtin == NULL){
		cerr << "2th channel is bad in "<< m_name << endl;
		return;
	}

	{// update time
		// update pvt data
		short h, m;
		float s;
		bool bt;

		ppvtin->get_time(h, m, s, bt);

		if(bt){
			m_h = (h + m_tz) % 24;
			m_m = m;
			m_s = s;
		}else{
			m_s += dt;
			if(m_s >= 60.0){
				m_s = (float)(m_s - 60.0);
				m_m++;
			}
			if(m_m >= 60){
				m_m = 0;
				m_h++;
			}
			if(m_h >= 24){
				m_h = 0;
			}
		}
	}

	{ // update velocity
		double vel, crs, crs_var;
		bool bv;
		ppvtin->get_vel(vel, crs, crs_var, bv);
		if(bv){
			m_vel = vel;
			m_crs = crs * (PI / 180);
			m_crs_var = crs_var;

			m_Vwrld.x = m_vel * sin(m_crs) * KNOT;
			m_Vwrld.y = m_vel * cos(m_crs) * KNOT;
			m_Vwrld.z = 0.;
		}
	}

	{ //update position
		double lon, lat, alt;
		e_gp_dir lon_dir, lat_dir;
		bool bp;
		ppvtin->get_pos(lon, lon_dir, lat, lat_dir, alt, bp);

		if(bp){
			m_Xbih.lat = lat * (lat_dir == EGP_N ? 1 : -1) * (PI/180);
			m_Xbih.lon = lon * (lon_dir == EGP_E ? 1 : -1) * (PI/180);
			m_Xbih.alt = 0;
			getwrldrot(m_Xbih, m_Rwrld);
			bihtoecef(m_Xbih, m_Xecef);
		}else{
			Point3d Xecef_new;
			wrldtoecef(m_Xecef, m_Rown, dt * m_Vwrld, Xecef_new);

			m_Xecef = Xecef_new;
		}
	}
}

void f_sys_window::update_ais(long long cur_time)
{
	ch_ais * paisin = dynamic_cast<ch_ais*>(m_chin[3]);
	if(paisin == NULL){
		cerr << "3rd channel is bad in " << m_name << endl;
		return;
	}

	c_vdm_msg1 * pmsg1;
	while(pmsg1 = paisin->pop_msg1()){
		c_ship::register_ship_by_vdm1(pmsg1, cur_time);
		delete pmsg1;
	}

	c_vdm_msg18 * pmsg18;
	while(pmsg18 = paisin->pop_msg18()){
		c_ship::register_ship_by_vdm18(pmsg18, cur_time);
		delete pmsg18;
	}

	c_vdm_msg19 * pmsg19;
	while(pmsg19 = paisin->pop_msg19()){
		c_ship::register_ship_by_vdm19(pmsg19, cur_time);
		delete pmsg19;
	}
	
	c_vdm_msg5 * pmsg5;
	while(pmsg5 = paisin->pop_msg5()){
		c_ship::register_ship_by_vdm5(pmsg5);
		delete pmsg5;
	}

	c_vdm_msg24 * pmsg24;
	while(pmsg24 = paisin->pop_msg24()){
		c_ship::register_ship_by_vdm24(paisin->pop_msg24());
		delete pmsg24;
	}

	c_ship::delete_timeout_ship(cur_time);

	const list<c_ship*> & ships = c_ship::get_ship_list();
	for(auto itr = ships.begin(); itr != ships.end(); itr++)
		(*itr)->calc_Xwrld(m_Xecef, m_Rwrld, cur_time);
}

void f_sys_window::render()
{
	const list<c_ship*> & ships = c_ship::get_ship_list();

	// for ais 2d map
	m_pd3dev->BeginScene();
	m_maincam.calc_prjmtx(m_rot_own, m_pos_own);

	m_maincam.SetAsRenderTarget(m_pd3dev);

	ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pin == NULL){
		cerr << "0th channel is bad in " << m_name << endl;
		return;
	}
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.5f, 0.5f, 0.6f, 1.0f), 1.0f, 0);

	long long timg;
	Mat img = pin->get_img(timg);
	if(!img.empty()){
		m_maincam.blt_offsrf(m_pd3dev, img);
	}
	//m_maincam.render_sea(m_pd3dev);
	//m_maincam.render_ship(m_pd3dev, ships, 10000);

	if(m_bdisp){
		m_maincam.render_hrzn(m_pd3dev, m_d3d_txt, m_pline);
		m_maincam.render_ais(m_pd3dev, m_d3d_txt, m_pline, ships, 3000);
	}

	m_maincam.ResetRenderTarget(m_pd3dev);

	m_maincam.show(m_pd3dev, 0, (float) m_ViewPort.Height);

	if(!m_bdisp){
		m_pd3dev->EndScene();
		return;
	}

	m_map.SetAsRenderTarget(m_pd3dev);

	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.f, 0.f, 0.f, 1.0f), 1.0f, 0);
	int cx = m_sz_map.width >> 1;
	int cy = m_sz_map.height >> 1;
	double range = 3000;
	double rat = (double) cx / range;

	m_d3d_ship2d.render(m_pd3dev, (float) cx, 
		(float) cy, 1., (float) -m_rot_own.yaw, D3DCOLOR_RGBA(0, 255, 0, 0));

	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		if(!(*itr)->is_inrange(range))
			continue;
		(*itr)->render2d(m_pd3dev, m_pline, m_d3d_ship2d, m_Xecef, m_Rwrld, rat, (float) cx, (float) cy);
	}

	m_map.ResetRenderTarget(m_pd3dev);
	m_map.show(m_pd3dev, (float) (m_ViewPort.Width - 300), (float) m_ViewPort.Height);

	// for sub camera
	if(m_chin[7] != NULL){
		ch_image * psubin = dynamic_cast<ch_image*>(m_chin[7]);
		if(psubin == NULL){
			cerr << "7th channel is bad in " << m_name << endl;
			return;
		}

		long long timg;
		Mat img = psubin->get_img(timg);
		if(!img.empty()){
			m_subcam.calc_prjmtx(m_rot_own, m_pos_own);
			m_subcam.SetAsRenderTarget(m_pd3dev);
			// here the rendering code
			m_subcam.blt_offsrf(m_pd3dev, img);

			m_subcam.render_hrzn(m_pd3dev, m_d3d_txt, m_pline);
			m_subcam.render_ais(m_pd3dev, m_d3d_txt, m_pline, ships, 3000);
			m_subcam.ResetRenderTarget(m_pd3dev);
		}
		m_subcam.show(m_pd3dev,(float) (m_ViewPort.Width - m_sz_map.width - m_sz_subcam.width), (float) m_ViewPort.Height);
	}

	m_d3d_txt.set_prjmtx((float)m_ViewPort.Width, (float)m_ViewPort.Height);

	// draw time
	if(m_h < 0 || m_h > 23 || m_m < 0 || m_m > 59
		|| m_s < 0 || m_s >= 60){
			sprintf(m_buf, "UTC+%2d %2d:%2d:%2d", 0, 0, 0, 0.);
	}else{
		sprintf(m_buf, "UTC+%2d %2d:%2d:%2d", m_tz, m_h, m_m, (int)m_s);
	}

	float sx, sy, x, y;
	m_d3d_txt.get_text_size(sx, sy, m_buf);
	x = (float)m_ViewPort.X;
	y = (float)m_ViewPort.Y + m_ViewPort.Height;
	m_d3d_txt.render(m_pd3dev, m_buf, x, y, 1., 0.,  EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	if(fabs(m_Xbih.lon) > 2*PI || fabs(m_Xbih.lat) > 0.5*PI){
		sprintf(m_buf, "POS %3.6f%s %2.6f%s", 0, "X", 0, "X");
	}else{
		sprintf(m_buf, "POS %3.6f%s %2.6f%s", 
			m_Xbih.lon * (180/PI), (m_Xbih.lon > 0 ? "E":"W"),
			m_Xbih.lat * (180/PI), (m_Xbih.lat > 0 ? "N":"S"));
	}
	y -= sy;
	m_d3d_txt.render(m_pd3dev, m_buf, x, y, 1., 0.,  EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	// draw velosity and course
	if(m_vel < 0 || m_vel >= 100 || m_crs < 0 || m_crs >= 360){
		sprintf(m_buf, "SOG %2.1f COG %2.1f HDG %2.1f", 0., 0., 0.);
	}else{
		sprintf(m_buf, "SOG %2.1f COG %2.1f HDG %2.1f", m_vel, m_crs * (180/PI),
			m_rot_own.yaw * (180/PI));
	}
	y -= sy;
	m_d3d_txt.render(m_pd3dev, m_buf, x, y, 1., 0.,  EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	// draw frame rate
	if(m_avg_cycle_time < 0){
		sprintf(m_buf, "FPS %2.2f", 0);
	}else{
		sprintf(m_buf, "FPS %2.2f", 1.0 / m_avg_cycle_time);
	}
	y -= sy;
	m_d3d_txt.render(m_pd3dev, m_buf, x, y, 1., 0.,  EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	if(m_grid_width > 0)
		render_grid();

	if(m_btrck){
		render_trck_rc(m_pd3dev);
	}

	if(m_chin[5]){
		ch_vector<c_track_obj> * ptrckin = dynamic_cast<ch_vector<c_track_obj> *>(m_chin[5]);
		if(ptrckin == NULL){
			cerr << "5th channel is bad in " << m_name << endl;
			return;
		}

		ch_vector<c_track_obj> * ptrckout = dynamic_cast<ch_vector<c_track_obj> *>(m_chout[0]);
		if(ptrckout == NULL){
			cerr << "0th channel is bad in " << m_name << endl;
			return;
		}

		c_track_obj * pobj;
		while((pobj = ptrckin->pop()) != NULL){
			if(pobj->is_lost()){
				delete pobj;
				continue;
			}else{
				pobj->render(m_pline, m_rat);
			}
			ptrckout->push(pobj);
		}
	}

	m_pd3dev->EndScene();
}

void f_sys_window::render_trck_rc(LPDIRECT3DDEVICE9 pd3dev)
{
	D3DXVECTOR2 box[5] = {
		D3DXVECTOR2((float)m_mouse.x, (float)m_mouse.y),
		D3DXVECTOR2((float)m_mouse.x, (float)(m_mouse.y + m_trck_size.cy)),
		D3DXVECTOR2((float)(m_mouse.x + m_trck_size.cx), (float)(m_mouse.y + m_trck_size.cy)),
		D3DXVECTOR2((float)(m_mouse.x + m_trck_size.cx),(float) m_mouse.y),
		D3DXVECTOR2((float)m_mouse.x, (float)m_mouse.y)
	};

	m_pline->Begin();
	m_pline->Draw(box, 5, D3DXCOLOR(1., 1., 0, 1.));
	m_pline->End();
}

void f_sys_window::render_grid()
{
	D3DXVECTOR2 v[2];
	m_pline->Begin();
	int top = m_ViewPort.Y;
	int bottom = m_ViewPort.Y + m_ViewPort.Height;
	int left = m_ViewPort.X;
	int right = m_ViewPort.X + m_ViewPort.Width;
	for(int x = left; x < right; x += m_grid_width){
		v[0] = D3DXVECTOR2((float) x, (float) top);
		v[1] = D3DXVECTOR2((float) x, (float) bottom);
		m_pline->Draw(v, 2, D3DCOLOR_ARGB(128, 255, 255, 255));
	}
	for(int y = top; y < bottom; y += m_grid_width){
		v[0] = D3DXVECTOR2((float) left, (float) y);
		v[1] = D3DXVECTOR2((float) right, (float) y);
		m_pline->Draw(v, 2, D3DCOLOR_ARGB(128, 255, 255, 255));
	}
	m_pline->End();
}


void f_sys_window::handle_lbuttondown(WPARAM wParam, LPARAM lParam)
{
	m_mouse.x = LOWORD(lParam);
	m_mouse.y = HIWORD(lParam);
	if(m_btrck){
		double rat_inv = 1./m_rat;
		m_trck_rc.x = (int) (m_mouse.x * rat_inv);
		m_trck_rc.y = (int) (m_mouse.y * rat_inv);
		m_trck_rc.width = (int) (m_trck_size.cx * rat_inv);
		m_trck_rc.height = (int) (m_trck_size.cy * rat_inv);
		c_track_obj * pobj = new c_track_obj;
		pobj->set_rc(m_trck_rc);
		pobj->set_name("tracking");

		ch_vector<c_track_obj> * ptrckout = dynamic_cast<ch_vector<c_track_obj> *>(m_chout[0]);
		if(ptrckout == NULL){
			cerr << "0th channel is bad in " << m_name << endl;
			return;
		}
		ptrckout->push(pobj);
	}
}

void f_sys_window::handle_mousemove(WPARAM wParam, LPARAM lParam)
{
	m_mouse.x = LOWORD(lParam);
	m_mouse.y = HIWORD(lParam);
}


