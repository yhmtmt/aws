#ifndef _F_DS_WINDOW_
#define _F_DS_WINDOW_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ds_window.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifdef _WIN32
////////////////////////////////////////////////////////////////
/////////////// direct X based window //////////////////////////
#define SYS_BUF_SIZE 512

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_d3d_2dobj.h"
#include "../channel/ch_image.h"

#include "f_base.h"

class f_ds_window:public f_base
{
protected:
	thread * m_th_msg;
	//pthread_t m_th_msg;

	int m_Hfull, m_Vfull;
	void fit_full_mode();

	int m_Vwin, m_Hwin; 
	void fit_win_mode();

	double m_rat;

	HINSTANCE m_hinst;	// instance handle of the application
	TCHAR * m_name_t;	// window title
	TCHAR * m_clsname_t; //registered window class name 
	WNDCLASSEX m_wclsx; // window class registered
	HWND m_hwnd;		// handle of the created window 
	POINT m_client_org; // client origin in the window
	bool create_wnd();

	//////////////////////////////////// Windows helper
	void extractPointlParam(LPARAM lParam, Point2i & pt);

	/////////////////////////////////// direct3d objs

	UINT m_iadapter;	// video card index to be used
	bool m_blost;		// flag for device lost
	bool m_bwin;	// flag for window mode (false if fullscreen)
	mutex m_d3d_mtx;
	//pthread_mutex_t m_d3d_mtx;

	LPDIRECT3D9 m_pd3d; // direct3d object
	D3DPRESENT_PARAMETERS m_d3dppfull, m_d3dppwin; // creation parameter of direct3d (one for window mode, another for fullscreen)
	LPDIRECT3DDEVICE9 m_pd3dev; // direct3d device
	LPDIRECT3DSURFACE9 m_pgrabsurf;

	ID3DXLine * m_pline;// line object of direct3dx
	c_d3d_dynamic_text m_d3d_txt; // text object of direct3d

	D3DVIEWPORT9 m_ViewPort;
	D3DDISPLAYMODE m_d3ddm;
	c_d3d_camview m_maincam;

	bool init_d3d();	// initialize all direct3d objects
	virtual bool alloc_d3dres(); // helper for init_d3d
	bool init_viewport(Mat & img);
	virtual void release_d3dres(); // release all direct3d objects
	void change_dispmode(); // change furll or window mode.
	bool reset_d3dev();

	/////////////////////////////////// for grabbing rendered surface
	int m_num_grab_frms, m_num_grabbed_frms;
	wchar_t * m_grab_name;

	void grab();

	// helper functions
	void blt_offsrf(Mat & img);

	union{
		char m_buf[SYS_BUF_SIZE];
		wchar_t m_wbuf[SYS_BUF_SIZE/2];
	};

#define SIZE_WNDTBL 16 // sizeof window table. the number limits the number of windows user can make.
	struct s_wnd_entry{
		f_ds_window * pwin;
		HWND hwnd;
		s_wnd_entry():pwin(NULL), hwnd(NULL){};
	};

	static s_wnd_entry m_wndtbl[SIZE_WNDTBL];
	bool add_wnd(f_ds_window * pwin, HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(!m_wndtbl[iw].hwnd && !m_wndtbl[iw].pwin){
				m_wndtbl[iw].hwnd = hwnd;
				m_wndtbl[iw].pwin = pwin;
				return true;
			}
		}
		return false;
	}
	static void rm_wnd(f_ds_window * pwin, HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(m_wndtbl[iw].hwnd == hwnd && m_wndtbl[iw].pwin == pwin){
				m_wndtbl[iw].hwnd = NULL;
				m_wndtbl[iw].pwin = NULL;
			}
		}
	}

	static f_ds_window * get_window(HWND hwnd){
		for(int iw = 0; iw < SIZE_WNDTBL; iw++){
			if(m_wndtbl[iw].hwnd == hwnd){
				return m_wndtbl[iw].pwin;
			}
		}
		return NULL;
	}

public:
	f_ds_window(const char * name);
	~f_ds_window();

	virtual bool is_active();

	virtual bool check(){
		return dynamic_cast<ch_image*>(m_chin[0]) != NULL; 
	};

	virtual bool run(long long start_time, long long end_time)
	{
		return f_base::run(start_time, end_time);
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();

	// Message Processor
	static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg,
		WPARAM wParam, LPARAM lParam);

	virtual void handle_lbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_lbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_lbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_rbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttonup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mbuttondblclk(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousewheel(WPARAM wParam, LPARAM lParam){};
	virtual void handle_mousemove(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_syskeydown(WPARAM wParam, LPARAM lParam){};
	virtual void handle_keyup(WPARAM wParam, LPARAM lParam){};
	virtual void handle_char(WPARAM wParam, LPARAM lParam){};
	virtual bool init_run(){
//		pthread_mutex_init(&m_d3d_mtx, NULL); 
		if(!create_wnd()){
			cerr << "failed to create window." << endl;
			return false;
		}
		if(!init_d3d()){
			release_d3dres();

			if(m_pd3dev != NULL){
				m_pd3dev->Release();
				m_pd3dev = NULL;
			}

			if(m_pd3d != NULL){
				m_pd3d->Release();
				m_pd3d = NULL;
			}

			cerr << "failed to initialize direct3d." << endl;
			return false;
		}

		return (m_hwnd != NULL) && (m_pd3d != NULL) && (m_pd3dev != NULL);
	}

	virtual void destroy_run(){
		if(m_hwnd == NULL)
			return;

//		pthread_mutex_destroy(&m_d3d_mtx);
		delete[] m_name_t;
		m_name_t = NULL;
		release_d3dres();

		if(m_pd3dev != NULL){
			m_pd3dev->Release();
			m_pd3dev = NULL;
		}

		if(m_pd3d != NULL){
			m_pd3d->Release();
			m_pd3d = NULL;
		}

		rm_wnd(this, m_hwnd);

		DestroyWindow(m_hwnd);
		UnregisterClass(m_clsname_t, m_hinst);
		m_hwnd = NULL;
	}
};

#endif
#endif


