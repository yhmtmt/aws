#include "stdafx.h"

// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ds_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <WindowsX.h>

#if WINVER != 0x603 && WINVER != 0x602
// if not windows 8, the direct 3d is not included in the windows sdk.
//#include <d2d1.h>
//#include <dwrite.h>
#include <d3d9.h>
#endif

#include <d3dx9.h>

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

#include "f_ds_window.h"


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
	AdjustWindowRect(&rc, (WS_OVERLAPPEDWINDOW ^ WS_THICKFRAME) | WS_VISIBLE, FALSE);
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
	SetWindowLong(m_hwnd, GWL_STYLE, (m_bwin ? (WS_OVERLAPPEDWINDOW ^ WS_THICKFRAME) | WS_VISIBLE : WS_EX_TOPMOST | WS_POPUP | WS_VISIBLE));

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
		unique_lock<mutex> lock(m_d3d_mtx);
///		pthread_lock lock(&m_d3d_mtx);
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
	unique_lock<mutex> lock(m_d3d_mtx);
//	pthread_mutex_lock(&m_d3d_mtx);
	if(!reset_d3dev())
		cerr << "Failed to change display mode." << endl;
//	pthread_mutex_unlock(&m_d3d_mtx);
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
//	pthread_lock lock(&m_d3d_mtx);
	unique_lock<mutex> lock(m_d3d_mtx);
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

#endif