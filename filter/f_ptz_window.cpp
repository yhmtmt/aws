#include "stdafx.h"

// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ptz_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ptz_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ptz_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 

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


#include "f_ptz_window.h"


//////////////////////////////////////////////////////////////// f_ptz_window
#define ALPHA 0.1

f_ptz_window::f_ptz_window(const char * name):f_ds_window(name),
	m_prev_time(0), m_avg_cycle_time(0), m_tz(9), 
	m_bdisp(true), m_grid_width(-1),
	m_grid3d_width(-1.0), m_own_mmsi(-1), m_sz_map(300, 300), m_ptgt_ship(NULL),
	m_capture_interval(50000000), m_rotation_wait_time(20000000), 
	m_capture_time(0), m_shutter_time(0),
	m_baiscap(false), m_ptgt_prev(NULL)
{
	// initialize geometry parameters
	m_Rown = Mat::zeros(3, 3, CV_64FC1);
	m_Rwrld = Mat::zeros(3, 3, CV_64FC1);
	m_jpg_param.resize(2);
	m_jpg_param[0] = CV_IMWRITE_JPEG_QUALITY;
	m_jpg_param[1] = 100;
};

f_ptz_window::~f_ptz_window()
{
	release_d3dres();
}

bool f_ptz_window::alloc_d3dres()
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

	if(!m_d3d_ship2d.init(m_pd3dev, (float) m_sz_map.width, (float) m_sz_map.height, 16))
		return false;

	if(!m_map.init(m_pd3dev, (float) m_sz_map.width, (float) m_sz_map.height, 
		(float) (m_ViewPort.Width),
		(float) (m_ViewPort.Height)))
		return false;

	int hsize, vsize;
	Mat Cam;
	s_rotpar rot;
	Point3d Pos;

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

	return true;
}


void f_ptz_window::release_d3dres()
{
	if(m_pgrabsurf != NULL){
		m_pgrabsurf->Release();
		m_pgrabsurf = NULL;
	}

	m_d3d_ship2d.release();
	m_d3d_txt.release();
	m_map.release();
	m_maincam.release();
	if(m_pline != NULL){
		m_pline->Release();
		m_pline = NULL;
	}
}

bool f_ptz_window::select_capture_target()
{
	ch_ptzctrl * ptzout = dynamic_cast<ch_ptzctrl*>(m_chout[0]);
	if(ptzout == NULL){
		cerr << "0th channel is bad in " << m_name << endl;
		return false;
	}

	const list<c_ship*> & ships = c_ship::get_ship_list();

	if(m_capture_time < m_cur_time){
		int theta;
		m_shutter_time = m_cur_time + m_rotation_wait_time;
		double hdg = m_rot_own.yaw;
		for(auto itr = ships.begin(); itr != ships.end(); itr++){
			m_dir_tgt = (*itr)->get_dir(m_shutter_time);

			theta = (int)((m_dir_tgt - hdg) * 180. / PI);
			theta -= m_v;
			theta %= 360;
			if(theta > 180)
				theta -= 360;
			if(theta < -180)
				theta += 360;

			if(abs(theta) > m_vlim)
				continue;

			if((*itr)->get_dist() > m_dlim)
				continue;

			m_candidate_ship.push_back((*itr));
		}

		if(m_candidate_ship.size() != 0){
			int itgt_ship = rand() % m_candidate_ship.size();
			m_ptgt_ship = m_candidate_ship[itgt_ship];
			m_dir_tgt = m_ptgt_ship->get_dir(m_cur_time);
			if(m_ptgt_ship != NULL){
				theta = (int) ((m_dir_tgt - hdg) * 18000. / PI);
				cout << "dir_tgt=" << m_dir_tgt * 180 / PI << endl;
				cout << "hdg_own=" << hdg * 180 / PI << endl;
				cout << "cam_rot= " << theta << endl;
				theta = -theta;
				theta %= 36000;
				if(theta < 0)
					theta += 36000;

				m_pan = (unsigned short) theta;
				cout << "pan deg = " << m_pan << endl;
				ptzout->inst_pan(m_pan);
			}
		}

		m_candidate_ship.clear();
	}
	return true;
}

bool f_ptz_window::capture_target(unsigned short p, Mat & img)
{
	if(abs(p - m_pan) < 100){
		cout << "Capture at " << m_pan << " deg " << endl;
		char fname[128];
		m_capture_time = m_cur_time + m_capture_interval;
		sprintf(fname, "%d_%d_%f2.1_%u.jpg", m_h, m_m, m_s, m_ptgt_ship->get_mmsi());
		imwrite(fname, img, m_jpg_param);
		ofstream file(m_logfname, ios_base::app);

		file << fname << ", " << m_h << ", " << m_m << ", " << m_s << ", "
			<< m_ptgt_ship->get_mmsi() << ", "
			<< m_ptgt_ship->get_name() << ", "
			<< m_ptgt_ship->get_ship_type() << ", "
			<< m_ptgt_ship->get_cog()  * 180 / PI << ", "
			<< m_ptgt_ship->get_dist() << ", "
			<< m_dir_tgt * 180 / PI << ", "
			<< m_rot_own.yaw * 180 / PI << ", " << m_pan << endl;
		file.close();
		m_ptgt_prev = m_ptgt_ship;
		m_ptgt_ship = NULL;
	}else if(m_cur_time - m_capture_time > m_capture_interval){
		m_ptgt_ship = NULL;
		m_capture_time = m_cur_time + m_capture_interval;
	}
	return true;
}

bool f_ptz_window::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "disp") == 0){
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
	}else if(strcmp(args[itok], "cap") == 0){
		if(num_args != 4)
			m_baiscap = false;

		m_baiscap = true;
		strcpy(m_logfname, args[itok+1]);

		return true;
	}else if(strcmp(args[itok], "int") == 0){
		if(num_args != 4 )
			return false;

		m_capture_interval = (long long) (atof(args[itok+1]) * 10000000);
		return true;
	}else if(strcmp(args[itok], "dlim") == 0){
		if(num_args != 4)
			return false;
		m_dlim = atof(args[itok+1]);
		return false;
	}else if(strcmp(args[itok], "vlim") == 0){
		if(num_args != 5)
			return false;
		double low = atof(args[itok+1]);
		double high = atof(args[itok+2]);

		if(low > high){
			cerr << "vlim <low> <high> requires <low> < <high>" << endl;
			return false;
		}
		if(low < -360. || low > 360.){
			cerr << "vlim <low> <high> requires -360 < theta < 360" << endl;
			return false;
		}

		if(high < -360. || high > 360.){
			cerr << "vlim <low> <high> requires -360 < theta < 360" << endl;
			return false;
		}

		double v = 0.5 * (low + high);
		if(v < 0)
			v += 360.;

		if(v < 0. || v > 360.){
			cerr << "vlim <low> <high> requires 0 < <high> - <low> < 360" << endl;
			return false;
		}

		double vlim = 0.5 * (high - low);
		if(0 > vlim || 180 < vlim){
			cerr << "vlim <low> <high> requires 0 < <high> - <low> < 360" << endl;
			return false;
		}

		m_v = (int) v;
		m_vlim = (int) vlim;
		return true;
	}

	return f_ds_window::cmd_proc(cmd);
}

bool f_ptz_window::check()
{
	return m_chin[0] != NULL;
}

bool f_ptz_window::proc()
{
	unique_lock<mutex> lock(m_d3d_mtx);

	float dt = (float) ((m_cur_time - m_prev_time) * 1e-7);
	m_avg_cycle_time = ALPHA * dt + (1 - ALPHA) * m_avg_cycle_time;
	////////////////// updating pvt information ///////////////////////
	update_pvt(dt);

	ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pin == NULL){
		cerr << "0th channel is bad in " << m_name << endl;
		return false;
	}

	long long timg;
	Mat img = pin->get_img(timg);

	// update heading
	ch_navdat * pshipin = dynamic_cast<ch_navdat*>(m_chin[4]);

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

	// This is the temporal code.
	// translation between camera center and pan-tilt center set as zero.
	pos = Point3d(0., 0., 0.); 
	m_maincam.set_campar(pos, true);

	unsigned short p,z;
	short t;
	ch_ptz * ptzin = dynamic_cast<ch_ptz*>(m_chin[5]);
	if(ptzin == NULL){
		cerr << "5th channel is bad in " << m_name << endl;
		return false;
	}

	if(ptzin->get(p, t, z)){
		//		cout << "PTZ:" << p << "," << t << "," << z << endl;
		rot.yaw = -0.01 * (float) p * PI/180;
		rot.pitch = 0.01 * (float) t * PI/180.;
		rot.roll = 0.;
		m_maincam.set_campar(rot, true);
	}


	////////////////// loading ais data //////////////////////////////
	update_ais(m_cur_time);

	/////////////// handling device lost /////////////////////////////
	if(m_blost){
		if(m_pd3dev->TestCooperativeLevel() == D3DERR_DEVICELOST){
			Sleep(10);
			return true;
		}

		if(!reset_d3dev())
			return false;
	}

	//////////////////// clear back buffer ///////////////////////////
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);

	////////////////// ais ship capture routine //////////////////////
	// if no target set then
	//		chose target in sight
	//		calc and set cam direction
	// else 
	//	 if cam stopped panning then
	//		save the target inf, picture, and ownship state
	//		reset target
	//	 else 
	//		continue
	// end if
	ch_ptzctrl * ptzout = dynamic_cast<ch_ptzctrl*>(m_chout[0]);
	if(ptzout == NULL){
		cerr << "0th channel is bad in " << m_name << endl;
		return false;
	}

	if(m_baiscap){
		if(m_ptgt_ship)
			m_ptgt_ship->set_clr(D3DCOLOR_RGBA(255, 0, 0, 255));
		if(m_ptgt_prev)
			m_ptgt_prev->set_clr(D3DCOLOR_RGBA(0, 255, 0, 255));

		if(m_ptgt_ship == NULL){
			if(!select_capture_target())
				return false;
		}else if(m_shutter_time < m_cur_time){
			capture_target(p, img);
		}
	}

	/////////////////////rendering sequence/////////////////////////// 
	render(img);

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

void f_ptz_window::update_pvt(float dt)
{
	ch_pvt * ppvtin = dynamic_cast<ch_pvt*>(m_chin[2]);
	if(ppvtin == NULL){
		cerr << "2nd channel is bad in " << m_name << endl;
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
	/*
	m_Xbih.lat = m_pshipin->m_lat;
	m_Xbih.lon = m_pshipin->m_lon;
	m_vel = m_pshipin->m_gpspd;
	m_crs = m_pshipin->m_gpdir;
	m_crs_var = 0.;
	m_h = m_pshipin->m_hour;
	m_m = m_pshipin->m_min;
	m_s = m_pshipin->m_sec;
	*/
}

void f_ptz_window::update_ais(long long cur_time)
{
	ch_ais * paisin = dynamic_cast<ch_ais*>(m_chin[3]);
	if(paisin == NULL){
		cerr << "3rd channel is bad in " << m_name << endl;
		return;
	}

	while(c_ship::register_ship_by_vdm1(paisin->pop_msg1(), cur_time));
	while(c_ship::register_ship_by_vdm18(paisin->pop_msg18(), cur_time));
	while(c_ship::register_ship_by_vdm19(paisin->pop_msg19(), cur_time));
	while(c_ship::register_ship_by_vdm5(paisin->pop_msg5()));
	while(c_ship::register_ship_by_vdm24(paisin->pop_msg24()));

	c_ship::delete_timeout_ship(cur_time);

	const list<c_ship*> & ships = c_ship::get_ship_list();
	for(auto itr = ships.begin(); itr != ships.end(); itr++)
		(*itr)->calc_Xwrld(m_Xecef, m_Rwrld, cur_time);
}


void f_ptz_window::render(Mat & img)
{
	const list<c_ship*> & ships = c_ship::get_ship_list();

	// for ais 2d map
	m_pd3dev->BeginScene();
	/////////////////////// render main view //////////////////////////
	m_maincam.calc_prjmtx(m_rot_own, m_pos_own);

	m_maincam.SetAsRenderTarget(m_pd3dev);

	m_maincam.blt_offsrf(m_pd3dev, img);

	if(m_bdisp){
		m_maincam.render_hrzn(m_pd3dev, m_d3d_txt, m_pline);
		m_maincam.render_ais(m_pd3dev, m_d3d_txt, m_pline, ships, 3000);
	}

	m_maincam.ResetRenderTarget(m_pd3dev);

	m_maincam.show(m_pd3dev, 0, (float) m_ViewPort.Height);

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

	m_pd3dev->EndScene();
}

void f_ptz_window::render_grid()
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
