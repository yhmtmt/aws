#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <WindowsX.h>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/thread_util.h"

#include "../util/coord.h"
#include "../util/c_ship.h"
#include "../util/c_clock.h"
//#include "../util/c_nmeadec.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_scalar.h"
#include "../channel/ch_campar.h"
#include "../channel/ch_navdat.h"
#include "../channel/ch_nmea.h"

#include "f_base.h"
#include "f_window.h"



#define ALPHA 0.1
///////////////////////////////////////////////////////////////// f_mark_window

f_mark_window::f_mark_window(const char * name):f_window(name), m_scale(1)
{
	setMouseCallback(m_name, on_mouse, this);
}

f_mark_window::~f_mark_window()
{
}

bool f_mark_window::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;
	if(strcmp(args[itok], "save") == 0){
		if(num_args != 4)
			return false;
		write(args[itok+1]);
		return true;
	}else if(strcmp(args[itok], "sc") == 0){
		if(num_args != 4)
			return false;
		m_scale = atoi(args[itok+1]);
		return true;
	}

	return f_window::cmd_proc(cmd);
}


#ifdef _WIN32

///////////////////////////////////////////////////////////////// f_ds_window
f_ds_window::s_wnd_entry f_ds_window::m_wndtbl[SIZE_WNDTBL];

f_ds_window::f_ds_window(const char * name): f_base(name), 	
	m_name_t(NULL),
	m_bwin(true), m_blost(false), m_hwnd(NULL),
	m_Hfull(1920), m_Vfull(1080), m_iadapter(D3DADAPTER_DEFAULT),
	m_Hwin(1920), m_Vwin(1080),
	m_pd3d(NULL), m_pd3dev(NULL), 
	m_pgrabsurf(NULL), m_pline(NULL), m_grab_name(NULL)
{
	register_fpar("wmd", &m_bwin, "yes: window mode, no: full screen mode");
	register_fpar("Hfull", &m_Hfull, "Horizontal screen size in full screen mode.");
	register_fpar("Vfull", &m_Vfull, "Vertical screen size in full screen mode.");
	register_fpar("Hwin", &m_Hwin, "Horizontal screen size in window mode.");
	register_fpar("Vwin", &m_Vwin, "Vertical screen size in window mode.");

	// get instance handle
	m_hinst = GetModuleHandle(NULL);

	// create window
	m_name_t = new TCHAR[strlen(m_name)+1];
	mbstowcs(m_name_t, m_name, strlen(m_name) + 1);
	m_clsname_t = new TCHAR[strlen(m_name) +  5]; // adding prefix "aws_"
	wsprintf(m_clsname_t, _T("%s_%s"), _T("aws"), m_name_t);

	m_wclsx.lpszClassName = m_clsname_t;
	m_wclsx.cbSize = sizeof(WNDCLASSEX);
	m_wclsx.style = CS_HREDRAW | CS_VREDRAW;
	m_wclsx.lpfnWndProc = WindowProc;
	m_wclsx.hInstance = m_hinst;
	m_wclsx.hIcon = NULL;
	m_wclsx.hIconSm = NULL;
	m_wclsx.hCursor = LoadCursor(NULL, IDC_ARROW);
	m_wclsx.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	m_wclsx.lpszMenuName = NULL;
	m_wclsx.cbClsExtra = 0;
	m_wclsx.cbWndExtra = 0;
	m_client_org.x = GetSystemMetrics(SM_CXSIZEFRAME);
	m_client_org.y = GetSystemMetrics(SM_CYCAPTION) + GetSystemMetrics(SM_CYSIZEFRAME);
}

f_ds_window::~f_ds_window()
{
}

bool f_ds_window::create_wnd()
{
	if(!RegisterClassEx(&m_wclsx))
		return false;

	m_hwnd = CreateWindowEx(NULL, m_clsname_t,
		m_name_t, (m_bwin ? WS_OVERLAPPEDWINDOW | WS_VISIBLE : WS_EX_TOPMOST | WS_POPUP | WS_VISIBLE), 0, 0, m_Hwin, m_Vwin, 
		NULL, NULL, m_hinst, NULL);

	if(m_hwnd == NULL)
		return false;

	if(!add_wnd(this, m_hwnd))
		return false;

	ShowWindow(m_hwnd, SW_SHOWNORMAL);
	UpdateWindow(m_hwnd);

	return true;
}

void f_ds_window::fit_full_mode(){
	D3DDISPLAYMODE d3ddm;
	double min_diff = DBL_MAX;
	int m_Hbest = m_Hfull, m_Vbest = m_Vfull;

	int num_disp_modes = m_pd3d->GetAdapterModeCount(m_iadapter, D3DFMT_X8R8G8B8);
	int i;
	for(i = 0; i < num_disp_modes; i++){
		m_pd3d->EnumAdapterModes(m_iadapter, D3DFMT_X8R8G8B8, i, &d3ddm);
		double diff = abs((int) (d3ddm.Width - m_Hfull))
			+ abs((int) (d3ddm.Height - m_Vfull));
		if(diff < min_diff){
			m_Hbest = d3ddm.Width;
			m_Vbest = d3ddm.Height;
		}
	}

	m_Hfull = m_Hbest;
	m_Vfull = m_Vbest;
	m_d3dppfull.BackBufferWidth = m_Hfull;
	m_d3dppfull.BackBufferHeight = m_Vfull;
}

void f_ds_window::fit_win_mode(){
	RECT rc;
	rc.top = rc.left = 0;
	rc.bottom = m_Vwin;
	rc.right = m_Hwin; 
	AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW | WS_VISIBLE, FALSE);
	rc.right = min((int) (rc.right - rc.left), (int)m_d3ddm.Width);
	rc.bottom = min((int) (rc.bottom - rc.top), (int) m_d3ddm.Height);
	rc.top = rc.left = 0;
	SetWindowPos(m_hwnd, HWND_NOTOPMOST, rc.left, rc.top, rc.right, rc.bottom, SWP_SHOWWINDOW);
	GetClientRect(m_hwnd, &rc);
	m_Vwin = rc.bottom - rc.top;
	m_Hwin = rc.right - rc. left;
}

bool f_ds_window::init_d3d()
{
	m_pd3d = Direct3DCreate9(D3D_SDK_VERSION);

	if(m_pd3d == NULL)
		return false;

	HRESULT hr;
	hr = m_pd3d->GetAdapterDisplayMode(m_iadapter, &m_d3ddm);

	// initialize d3d device
	SetWindowLong(m_hwnd, GWL_STYLE, (m_bwin ? WS_OVERLAPPEDWINDOW | WS_VISIBLE : WS_EX_TOPMOST | WS_POPUP | WS_VISIBLE));

	if(m_bwin){
		fit_win_mode();
	}else{
		fit_full_mode();
	}

	// check if current display mode is X8R8G8B8
	if(FAILED(hr))
		return false;

	if(m_d3ddm.Format != D3DFMT_X8R8G8B8){
		cerr << "Please change display mode to 32bit color" << endl;
		return false;
	}

	m_Hwin = (m_Hwin < 0 ? m_d3ddm.Width : m_Hwin);
	m_Vwin = (m_Vwin < 0 ? m_d3ddm.Height : m_Vwin);

	// check if device supports fullscreen X8R8G8B8 
	hr = m_pd3d->CheckDeviceType(m_iadapter, D3DDEVTYPE_HAL,
		D3DFMT_X8R8G8B8, D3DFMT_X8R8G8B8, FALSE);

	if(FAILED(hr))
		return false;

	// check if device supports these surface format
	hr = m_pd3d->CheckDeviceFormat(m_iadapter, 
		D3DDEVTYPE_HAL, D3DFMT_X8R8G8B8, D3DUSAGE_DEPTHSTENCIL,
		D3DRTYPE_SURFACE, D3DFMT_D16);

	if(FAILED(hr))
		return false;

	// creating device
	m_blost = false;

	D3DCAPS9 d3dCaps;
	hr = m_pd3d->GetDeviceCaps(m_iadapter,
		D3DDEVTYPE_HAL, &d3dCaps);

	if(FAILED(hr))
		return false;

	DWORD dwBehaviorFlags = 0;

	if(d3dCaps.VertexProcessingCaps != 0)
		dwBehaviorFlags |= D3DCREATE_HARDWARE_VERTEXPROCESSING;
	else
		dwBehaviorFlags |= D3DCREATE_SOFTWARE_VERTEXPROCESSING;

	// set d3d present parameter for fullscreenmode 
	memset(&m_d3dppfull, 0, sizeof(m_d3dppfull));
	m_d3dppfull.SwapEffect = D3DSWAPEFFECT_DISCARD;
	m_d3dppfull.Windowed = FALSE;
	m_d3dppfull.BackBufferWidth = m_Hfull;
	m_d3dppfull.BackBufferHeight = m_Vfull;
	m_d3dppfull.BackBufferFormat = D3DFMT_X8R8G8B8;
	m_d3dppfull.EnableAutoDepthStencil = TRUE;
	m_d3dppfull.AutoDepthStencilFormat = D3DFMT_D16;
	m_d3dppfull.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;

	// set d3d present parameter for window mode
	memset(&m_d3dppwin, 0, sizeof(m_d3dppwin));
	m_d3dppwin.SwapEffect = D3DSWAPEFFECT_DISCARD;
	m_d3dppwin.Windowed = TRUE;
	m_d3dppwin.BackBufferWidth = 0;
	m_d3dppwin.BackBufferHeight = 0;
	m_d3dppwin.BackBufferFormat = D3DFMT_X8R8G8B8;
	m_d3dppwin.EnableAutoDepthStencil = TRUE;
	m_d3dppwin.AutoDepthStencilFormat = D3DFMT_D16;
	m_d3dppwin.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;

	if(m_bwin){
		hr = m_pd3d->CreateDevice(m_iadapter, D3DDEVTYPE_HAL, m_hwnd,
			dwBehaviorFlags, &m_d3dppwin, &m_pd3dev);
	}else{
		hr = m_pd3d->CreateDevice(m_iadapter, D3DDEVTYPE_HAL, m_hwnd,
			dwBehaviorFlags, &m_d3dppfull, &m_pd3dev);
	}

	if(FAILED(hr)){
		return false;
	}

	if(!init_viewport(Mat()))
		return false;

	return alloc_d3dres();
}

bool f_ds_window::init_viewport(Mat & img)
{
	int H, V;

	if(m_bwin){
		H = m_Hwin;
		V = m_Vwin;
	}else{
		H = m_Hfull;
		V = m_Vfull;
	}

	m_rat = 1.0;
	m_pd3dev->GetViewport(&m_ViewPort);

	m_ViewPort.Width = H;
	m_ViewPort.Height = V;
	if(!img.empty()){
		m_rat = (double) V / (double) img.rows;
		m_rat = min(m_rat, (double) H / (double) img.cols);
		m_ViewPort.Width = (int) ((double) img.cols * m_rat);
		m_ViewPort.Height = (int) ((double) img.rows * m_rat);
	}

	m_pd3dev->SetViewport(&m_ViewPort);

	return true;
}


bool f_ds_window::alloc_d3dres()
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

	if(!m_maincam.init(m_pd3dev,
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height))
		return false;

	D3DXCreateLine(m_pd3dev, &m_pline);

	return true;
}

bool f_ds_window::is_active()
{
	if(m_blost){
		if(m_pd3dev->TestCooperativeLevel() == D3DERR_DEVICELOST){
			Sleep(10);
			return true;
		}
		pthread_lock lock(m_d3d_mtx);
		if(!reset_d3dev())
			return m_bactive = false;
		m_blost = false;
	}

	return m_bactive;
}

void f_ds_window::release_d3dres()
{
	if(m_pgrabsurf != NULL){
		m_pgrabsurf->Release();
		m_pgrabsurf = NULL;
	}
	m_d3d_txt.release();
	m_maincam.release();
	if(m_pline != NULL){
		m_pline->Release();
		m_pline = NULL;
	}
}

void f_ds_window::change_dispmode()
{
	m_bwin = !m_bwin;
	pthread_mutex_lock(&m_d3d_mtx);
	if(!reset_d3dev())
		cerr << "Failed to change display mode." << endl;
	pthread_mutex_unlock(&m_d3d_mtx);
}

bool f_ds_window::reset_d3dev()
{
	release_d3dres();

	HRESULT hr;
	if(m_bwin){
		fit_win_mode();
		hr = m_pd3dev->Reset(&m_d3dppwin);
	}else{
		fit_full_mode();
		hr = m_pd3dev->Reset(&m_d3dppfull);
	}

	if(FAILED(hr)){
		cerr << "failed to reset D3DDevice" << endl;
		return false;
	}

	SetWindowLong(m_hwnd, GWL_STYLE, (m_bwin ? WS_OVERLAPPEDWINDOW | WS_VISIBLE : WS_EX_TOPMOST | WS_POPUP | WS_VISIBLE));

	if(m_bwin){
		fit_win_mode();
	}	

	cout << "Change screen mode ";
	if(m_bwin){
		cout << "to " << m_Hwin << "x" << m_Vwin << " window." << endl;
	}else{
		cout << "to " << m_Hfull << "x" << m_Vfull << " full." << endl;
	}

	if(!init_viewport(Mat())){
		cerr << "failed to initialize viewport" << endl;
		return false;
	}

	if(!alloc_d3dres()){
		cerr << "failed to allocate D3D resources" << endl;
		return false;
	}

	return true;
}

bool f_ds_window::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "grab") == 0){
		if(num_args < 3)
			return false;
		m_grab_name = new wchar_t[strlen(args[itok+1])+1];
		mbstowcs(m_grab_name, args[itok+1], strlen(args[itok+1]) + 1);

		if(num_args > 4)
			m_num_grab_frms = atoi(args[itok+2]);
		else
			m_num_grab_frms = 1;
		m_num_grabbed_frms = 0;

		return true;
	}

	return true;
}

bool f_ds_window::proc()
{
	pthread_lock lock(m_d3d_mtx);
	
	////////////////// updating pvt information ///////////////////////
	ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pin == NULL){
		cerr << "0th channel is bad in " << m_name << endl;
		return false;
	}

	long long timg;
	Mat img = pin->get_img(timg);
	if(img.empty())
		return true;
	if(img.cols != m_ViewPort.Width ||
		img.rows != m_ViewPort.Height){
			if(!init_viewport(img)){
				return false;
			}
	}

	if(img.cols != m_maincam.get_surface_width() ||
		img.rows != m_maincam.get_surface_height()){
			m_maincam.release();
			if(!m_maincam.init(m_pd3dev,  
				(float) img.cols, (float) img.rows, 
				(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
				(float) m_ViewPort.Width, (float) m_ViewPort.Height))
				return false;
	}

	//////////////////// clear back buffer ///////////////////////////
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);

	/////////////////////// render main view //////////////////////////
	m_maincam.SetAsRenderTarget(m_pd3dev);

	m_maincam.blt_offsrf(m_pd3dev, img);

	m_maincam.ResetRenderTarget(m_pd3dev);

	m_pd3dev->BeginScene();

	m_maincam.show(m_pd3dev, 0, (float) m_ViewPort.Height);

	m_pd3dev->EndScene();
	////////////////////// grab rendered back surface ////////////////
	if(m_grab_name)
		grab();

	////////////////////// presentation //////////////////////////////
	if(m_pd3dev->Present(NULL, NULL, 
		NULL, NULL) == D3DERR_DEVICELOST){
			cerr << "device lost" << endl;
			m_blost = true;
	}

	return true;
}


void f_ds_window::grab()
{
	LPDIRECT3DSURFACE9 pbksr;
	m_pd3dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &pbksr);
	snwprintf(m_wbuf, SYS_BUF_SIZE >> 1,  _T("%s%06d.bmp"), m_grab_name, m_num_grabbed_frms);

	HRESULT hr = m_pd3dev->GetRenderTargetData(pbksr, m_pgrabsurf);
	if(FAILED(hr))
		cerr << "Failed to Get Render Target Data" << endl;

	D3DXSaveSurfaceToFile(m_wbuf, D3DXIFF_BMP,
		m_pgrabsurf, NULL, NULL);

	m_num_grabbed_frms++;
	if(m_num_grabbed_frms == m_num_grab_frms){
		delete[] m_grab_name;
		m_grab_name = NULL;
	}

	pbksr->Release();
}


LRESULT CALLBACK f_ds_window::WindowProc(HWND hwnd, UINT uMsg,
	WPARAM wParam, LPARAM lParam)
{
	f_ds_window * pwin = get_window(hwnd); 
	if(pwin == NULL)
		return DefWindowProc(hwnd, uMsg, wParam, lParam);

	switch(uMsg){
	case WM_KEYDOWN:
		switch(wParam){
		case VK_ESCAPE:
			PostQuitMessage(0);
			break;
		default:
			pwin->handle_keydown(wParam, lParam);
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
		break;
	case WM_KEYUP:
		switch(wParam){
		default:
			pwin->handle_keyup(wParam, lParam);
		}
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
		break;
	case WM_CHAR:
		pwin->handle_char(wParam, lParam);
		break;
	case WM_LBUTTONDOWN:
		pwin->handle_lbuttondown(wParam, lParam);
		break;
	case WM_MBUTTONDOWN:
		pwin->handle_mbuttondown(wParam, lParam);
		break;
	case WM_RBUTTONDOWN:
		pwin->handle_rbuttondown(wParam, lParam);
		break;
	case WM_MOUSEMOVE:
		pwin->handle_mousemove(wParam, lParam);
		break;
	case WM_LBUTTONUP:
		pwin->handle_lbuttonup(wParam, lParam);
		break;
	case WM_MBUTTONUP:
		pwin->handle_mbuttonup(wParam, lParam);
		break;
	case WM_RBUTTONUP:
		pwin->handle_rbuttonup(wParam, lParam);
		break;
	case WM_LBUTTONDBLCLK:
		pwin->handle_lbuttondblclk(wParam, lParam);
		break;
	case WM_MBUTTONDBLCLK:
		pwin->handle_mbuttondblclk(wParam, lParam);
		break;
	case WM_RBUTTONDBLCLK:
		pwin->handle_rbuttondblclk(wParam, lParam);
		break;
	case WM_MOUSEWHEEL:
		pwin->handle_mousewheel(wParam, lParam);
		break;
	case WM_SYSKEYDOWN:
		switch(wParam){
		case VK_RETURN:
			pwin->change_dispmode();
			break;
		default:
			pwin->handle_syskeydown(wParam, lParam);
		}
		break;
	case WM_CLOSE:
		{
			//PostQuitMessage(0);
		}
		break;
	case WM_DESTROY:
		{
			//PostQuitMessage(0);
		}
		break;
	case WM_KILLFOCUS:
		if(!pwin->m_bwin)
			pwin->change_dispmode();
		break;
	default:
		{
			return DefWindowProc(hwnd, uMsg, wParam, lParam);
		}
		break;
	}

	return 0;
}

void f_ds_window::extractPointlParam(LPARAM lParam, Point2i & pt){
	pt.x = GET_X_LPARAM(lParam)/* - m_client_org.x*/;
	pt.y = GET_Y_LPARAM(lParam)/* - m_client_org.y*/;
}

///////////////////////////////////////////////////////////////// f_sys_window

f_sys_window::f_sys_window(const char * name):f_ds_window(name), 
	m_prev_time(0), m_avg_cycle_time(0), m_tz(9), 
	m_bdisp(true), m_grid_width(-1),
	m_grid3d_width(-1.0), m_own_mmsi(-1),
	m_btrck(false), m_sz_map(300, 300), m_sz_subcam(1600/3, 300)
{
	c_ship::init();

	// initialize geometry parameters
	m_Rown = Mat::zeros(3, 3, CV_64FC1);
	m_Rwrld = Mat::zeros(3, 3, CV_64FC1);
};

f_sys_window::~f_sys_window()
{
	c_ship::destroy();
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
	pthread_lock lock(m_d3d_mtx);

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


//////////////////////////////////////////////////////////////// f_ptz_window

f_ptz_window::f_ptz_window(const char * name):f_ds_window(name),
	m_prev_time(0), m_avg_cycle_time(0), m_tz(9), 
	m_bdisp(true), m_grid_width(-1),
	m_grid3d_width(-1.0), m_own_mmsi(-1), m_sz_map(300, 300), m_ptgt_ship(NULL),
	m_capture_interval(50000000), m_rotation_wait_time(20000000), 
	m_capture_time(0), m_shutter_time(0),
	m_baiscap(false), m_ptgt_prev(NULL)
{
	c_ship::init();

	// initialize geometry parameters
	m_Rown = Mat::zeros(3, 3, CV_64FC1);
	m_Rwrld = Mat::zeros(3, 3, CV_64FC1);
	m_jpg_param.resize(2);
	m_jpg_param[0] = CV_IMWRITE_JPEG_QUALITY;
	m_jpg_param[1] = 100;
};

f_ptz_window::~f_ptz_window()
{
	c_ship::destroy();
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
	pthread_lock lock(m_d3d_mtx);

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



//////////////////////////////////////////////////////////////// f_sprot_window

f_sprot_window::f_sprot_window(const char * name): f_ds_window(name), m_bmout(NULL), 
	m_tint_ttm2ais(5 * SEC), m_bmch(0), m_tprev_ttm2ais(0), m_range(10000), m_circle(1852), m_seq_id(0),
	m_tint_trail(10 * SEC), m_tprev_trail(0), m_bm_ver(1), m_max_trail(100)
{
	m_toker[0] = 'E';
	m_toker[1] = 'I';
	m_ftrail_log[0] = '\0';
	register_fpar("toker_bm", m_toker, 3, "Toker for binary message transfer.");
	register_fpar("tlog", m_ftrail_log, 128, "File name to save trail log.");
	register_fpar("tint_bm", &m_tint_ttm2ais, "Time interval of binary message transfer in 100nsec.");
	register_fpar("bm_ch", &m_bmch, "Channel for binary message (0: no preference, 1: channel A, 2: channel B, 3: Both.");
	register_fpar("range", &m_range, "Display range in meter.");
	register_fpar("circle", &m_circle, "Circle range in meter.");
	register_fpar("tint_trail", &m_tint_trail, "Time interval of saving trail.");
	register_fpar("max_trail", &m_max_trail, "Number of positions saved as trail.");
	register_fpar("bm_ver", &m_bm_ver, "Version of the binary message");
}

f_sprot_window::~f_sprot_window()
{
}

const char * f_sprot_window::get_err_msg(int code)
{
	const char * msg = f_base::get_err_msg(code);
	if(msg)
		return msg;

	switch(code){
	case FERR_SPROT_WINDOW_SNDBBM:
		return "Failed to send binary message.";
	case FERR_SPROT_WINDOW_OSPOS:
		return "Position of own ship has not been fixed yet.";
	case FERR_SPROT_WINDOW_FTRAIL_LOG_OPEN:
		return "Failed to open trail log file.";
	}
	return NULL;
}

bool f_sprot_window::send_bm(unsigned int mmsi_dst, int & seq_id,
	int id, int chan, 
	const unsigned char *  buf, 
	int bits)
{
	if(bits > 952){
		cout << "Irregal message length in send_bbm." << endl;
		return false;
	}

	if(chan >= 4){
		cout << "Irregal channel specification in send_bbm." << endl;
		return false;
	}

	if(id != 8 && id != 14 && id != 6 && id != 12){
		cout << "Irregal message id in send_bbm." << endl;
		return false;
	}

	int num_sends;

	m_nmea[0] = '!'; m_nmea[1] = m_toker[0]; m_nmea[2] = m_toker[1];

	int bit_lim;
	// Message type "ABM," or "BBM," filled in the m_nmea buffer
	if(id == 6 || id == 12){ //ABM
		m_nmea[3] = 'A';
		bit_lim = 288;
		seq_id %= 4;
	}else{ // BBM
		m_nmea[3] = 'B'; 
		bit_lim = 348;
		seq_id %= 10;
	}
	m_nmea[4] = 'B'; m_nmea[5] = 'M'; m_nmea[6] = ',';

	// calculating number of sentences needed 
	if(bits <= bit_lim){
		num_sends = 1;
	}else{
		num_sends = 1 + (bits - bit_lim) / 360 + (((bits - bit_lim) % 360) == 0 ? 0 : 1);
		num_sends = max(num_sends, 9);
	}

	m_nmea[7] = num_sends + '0'; m_nmea[8] = ',';  // Total number of sentences
	m_nmea[11] = seq_id + '0'; m_nmea[12] = ','; // sequential message identifier
	int ibit = 0;
	int ibuf = 0;
	int im = 0;
	for(int isend = 0; isend < num_sends; isend++){
		// first 58x6=348bit
		// subsequent 60x6=360bit
		int i;
		m_nmea[9] = (1 + isend) + '0'; m_nmea[10] = ','; // sentense number (upto num_sends)
		if(isend == 0){
			if(id == 6 || id == 12){
				sprintf(&m_nmea[13], "%09d", mmsi_dst);
				m_nmea[22] = ',';
				m_nmea[23] = chan + '0'; m_nmea[24] = ',';
				if(id == 6){
					m_nmea[25] = '6';
					i = 26;
				}else{
					m_nmea[25] = '1'; m_nmea[26] = '2';
					i = 27;
				}
			}else{
				m_nmea[13] = chan + '0'; m_nmea[14] = ',';
				if(id == 8){
					m_nmea[15] = '8';
					i = 16;
				}else{
					m_nmea[15] = '1'; m_nmea[16] = '4';
					i = 17;
				}
			}
				m_nmea[i] = ',';
			i++;
		}else{
			m_nmea[13] = ','; m_nmea[14] = ',';
			if(id == 6 || id == 12){
				m_nmea[15] = ',';
				i = 16;
			}else
				i = 15;
		}
			
		// copy message
		while(1){
			unsigned char uc = 0;

			switch(im){
			case 0:
				uc = (buf[ibuf] >> 2) & 0x3F;
				ibit += 6;
				im = 1;
				break;
			case 1:
				uc = (buf[ibuf] << 4);
				ibit += 2;
				if(ibit < bits){
					ibuf++;
					uc |= ((buf[ibuf] & 0xF0) >> 4); 
					ibit += 4;
				}
				im = 2;
				break;
			case 2:
				uc = (buf[ibuf] << 2);
				ibit += 4;
				if(ibit < bits){
					ibuf++;
					uc |= (buf[ibuf] & 0xC0) >> 6;
					ibit += 2;
				}
				im = 3;
				break;
			case 3:
				uc = buf[ibuf];
				ibit += 6;
				ibuf++;
				im = 0;
				break;
			}

			m_nmea[i] = armor(uc & 0x3F);
			i++;
			if(ibit >= bits || ibit >= bit_lim)
				break;
		}

		m_nmea[i] = ',';
		i++;
		if(isend == num_sends - 1){
			int pad = (bits % 6);
			if(pad)
				pad = 6 - pad;

			m_nmea[i] = pad + '0';	
		}else{
			m_nmea[i] = '0';
		}
		i++;
		m_nmea[i] = '*';
		unsigned char chksum = calc_nmea_chksum(m_nmea);
		char c;
		i++;
		c = (chksum >> 4) & 0x0F;
		m_nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');
		i++;
		c = chksum & 0x0F;
		m_nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');

		i++;
		m_nmea[i] = 13; // cr
		i++;
		m_nmea[i] = 10; // lf
		i++;
		m_nmea[i] = '\0';

		cout << "sending: " << m_nmea << endl;
		if(m_bmout){
			m_bmout->push(m_nmea);
		}

		bit_lim += 360;
	}
	seq_id = seq_id + 1;

	return true;
}

bool f_sprot_window::init_run()
{
	if(!f_ds_window::init_run())
		return false;

	if(m_chout.size())
		m_bmout = dynamic_cast<ch_nmea*>(m_chout[0]);
	if(m_ftrail_log[0]){
		m_trail_log.open(m_ftrail_log);
		if(!m_trail_log.is_open()){
			f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_FTRAIL_LOG_OPEN);
			return false;
		}
	}
	return true;
}

void f_sprot_window::destroy_run()
{
	f_ds_window::destroy_run();

	if(m_trail_log.is_open()){
		m_trail_log.close();
	}
}

bool f_sprot_window::alloc_d3dres()
{
	 c_ship::init();

	if(!f_ds_window::alloc_d3dres()){
		return false;
	}
	if(!m_d3d_ship2d.init(m_pd3dev,
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 16))
		return false;
	return true;
}

void f_sprot_window::release_d3dres()
{
	m_d3d_ship2d.release();
	f_ds_window::release_d3dres();
}

bool f_sprot_window::proc()
{
	Mat Rwrld;
	Point3d Xorg;

	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);
	

//	m_maincam.SetAsRenderTarget(m_pd3dev);
	int cx = m_ViewPort.Width >> 1;
	int cy = m_ViewPort.Height >> 1;
	double rat = (double) cy / m_range;

	c_ship::list_lock();
	c_ship & ship_own = c_ship::get_own_ship();
	if(!ship_own.calc_Xwrld_own(Xorg, Rwrld)){
		//f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_OSPOS);
		c_ship::list_unlock();
		return true;
	}


	const list<c_ship*> & ships = c_ship::get_ship_list();
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		(*itr)->calc_Xwrld(Xorg, Rwrld, m_cur_time);
	}
	c_ship::list_unlock();

//	m_d3d_ship2d.render(m_pd3dev, (float)cx,
//		(float)cy, 1.0, (float) -ship_own.get_cog(), D3DCOLOR_RGBA(0, 255, 0, 0));
	
	c_ship * pship_bm =  NULL;
	long long bmt_min = m_cur_time;

	m_pd3dev->BeginScene();
	render_circle(cx, cy, rat);
	c_ship::list_lock();
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		if(!(*itr)->is_inrange(m_range))
			continue;

		(*itr)->render2d(m_pd3dev, m_pline, m_d3d_ship2d, Xorg, Rwrld, 
			rat, (float)cx, (float) cy);
	}
	c_ship::list_unlock();
		
	ship_own.render2d(m_pd3dev, m_pline, m_d3d_ship2d, Xorg, Rwrld, 
		rat, (float) cx, (float)cy);


//	m_maincam.ResetRenderTarget(m_pd3dev);
//	m_maincam.show(m_pd3dev, 0, (float) m_ViewPort.Height);
	m_d3d_txt.set_prjmtx((float) m_ViewPort.Width, (float) m_ViewPort.Height);

	m_d3d_txt.render(m_pd3dev, get_time_str(),
		10, 20, 1.,0., EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	m_pd3dev->EndScene();

	if(m_grab_name)
		grab();

	if(m_pd3dev->Present(NULL, NULL, NULL, NULL) == D3DERR_DEVICELOST){
		cerr << "device lost" << endl;
		m_blost = true;
	}

	/////////////////////////// saving trail
	if(m_tprev_trail + m_tint_trail <= m_cur_time){
		if(m_trail_log.is_open()){
			ship_own.log_trail(m_trail_log);
		}
		
	//	ship_own.save_trail(m_max_trail);
		
		for(auto itr = ships.begin(); itr != ships.end(); itr++){
			if(!(*itr)->is_inrange(m_range))
				continue;
			
			if(m_trail_log.is_open()){
				(*itr)->log_trail(m_trail_log);
			}
			
	//		(*itr)->save_trail(m_max_trail);
		}
		m_tprev_trail = m_cur_time;
	}

	///////////////////////// sending ttm
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		if((*itr)->get_data_type() == ESDT_ARPA){
			long long bmt = (*itr)->get_last_bmt();
			if(bmt < bmt_min){
				bmt_min = bmt;
				pship_bm = (*itr);
			}
		}
	}
	if(m_bmout && pship_bm && (m_tprev_ttm2ais + m_tint_ttm2ais < m_cur_time)){
		// packing position data into bm
		s_binary_message bm;
		bm.id = 8;
		bm.ch = m_bmch;
		unsigned char sec;
		unsigned char id;
		unsigned short cog, sog, dist, bear;
		int lon, lat;
		id = pship_bm->get_ttmid();
		sog = (unsigned short) ((pship_bm->get_sog()) * 10 + 0.5);
		cog = (unsigned short) ((pship_bm->get_cog() * 180. / PI) * 10. + 0.5);
		dist = (unsigned short) (pship_bm->get_dist() * (10. / (double) MILE));
		bear = (unsigned short) ((pship_bm->get_bear() * 180. /PI) * 10. + 0.5);
		lon = (int)(pship_bm->get_lon() * (180. / PI) * 600000 + 0.5);
		lat = (int)(pship_bm->get_lat() * (180. / PI) * 600000 + 0.5);
		sec = m_tm.tm_sec;

		cout << "Encode: ";
		cout << "id " << (int) id 
			<< " sog " << sog
			<< " cog " << cog
			<< " dist " << dist
			<< " bear " << bear
			<< " lon " << lon 
			<< " lat " << lat  
			<< " sec " << (unsigned int) sec << endl;

		switch(m_bm_ver){
		case 5:
			bm.set_msg_pvc5(id, sog, cog, dist, bear, sec);
			break;
		case 4:
			bm.set_msg_pvc4(id, sog, cog, lon, lat, sec);
			break;
		case 3:
			bm.set_msg_pvc3(id, sog, cog, dist, bear);
			break;
		case 2:
			bm.set_msg_pvc2(id, sog, cog, lon, lat);
			break;
		case 1:
		default:
			bm.set_msg_pvc(id, sog, cog, lon, lat);
			break;
		}

		// sending bm to channel
		if(!send_bm(bm.mmsi, m_seq_id, bm.id, bm.ch, bm.msg, bm.len)){
			f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_SNDBBM);
		}else{
			m_tprev_ttm2ais = m_cur_time;
			pship_bm->set_bm_time(m_cur_time);
		}
	}

	return true;
}

#endif