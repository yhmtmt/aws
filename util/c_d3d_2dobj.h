#ifndef _C_D3D_2DOBJ_H_
#define _C_D3D_2DOBJ_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_d3d_2dobj.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_d3d_2dobj.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_d3d_2dobj.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_coord.h"

struct CUSTOMVERTEX
{
	FLOAT x, y, z, rhw;
	DWORD color;
	float u,v;
};

#define D3DFVF_CUSTOMVERTEX (D3DFVF_XYZRHW|D3DFVF_DIFFUSE|D3DFVF_TEX1)

struct CUSTOMVERTEX2
{
	FLOAT x, y, z;
	DWORD color;
	float u, v;
};

#define D3DFVF_CUSTOMVERTEX2 (D3DFVF_XYZ|D3DFVF_DIFFUSE|D3DFVF_TEX1)

struct CUSTOMVERTEX3
{
	FLOAT x, y, z;
	DWORD color;
};

#define D3DFVF_CUSTOMVERTEX3 (D3DFVF_XYZ|D3DFVF_DIFFUSE)

struct CUSTOMVERTEX4
{
	FLOAT x, y, z;
	D3DVECTOR n;
};

#define D3DFVF_CUSTOMVERTEX4 (D3DFVF_XYZ|D3DFVF_NORMAL)


/////////////////////////////////////////////////////////////// c_d3d_2dobj
class c_d3d_2dobj
{
protected:
	D3DXMATRIX m_Mview;
	D3DXMATRIX m_Mproj;
	float m_Hpix, m_Vpix;
public:
	virtual bool set_prjmtx(float hsize, float vsize);
};


/////////////////////////////////////////////////////////////// c_d3d_text
class c_d3d_text: public c_d3d_2dobj
{
protected:
	LPDIRECT3DTEXTURE9 m_pTx[95];
	short m_sx, m_sy;


	LPDIRECT3DTEXTURE9 gettx(char c)
	{
		if(c < 32 || c > 127)
			return NULL;
		return m_pTx[c - ' '];
	}
public:	
	c_d3d_text(): m_sx(0), m_sy(0)
	{
		for(int i = 0; i < 95; i++){
			m_pTx[i] = NULL;
		}
	}

	virtual ~c_d3d_text()
	{
		release();
	}

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev, unsigned int size = 24);

	virtual void release()
	{
		for(int i = 0; i < 95; i++){
			if(m_pTx[i] != NULL){
				m_pTx[i]->Release();
				m_pTx[i] = NULL;
			}
		}
	};
};



/////////////////////////////////////////////////////////////// c_d3d_dynamic_text
enum e_d3d_txt_center{
	EDTC_LB, EDTC_LC, EDTC_LT,
	EDTC_CB, EDTC_CC, EDTC_CT, 
	EDTC_RB, EDTC_RC, EDTC_RT
};

class c_d3d_dynamic_text: public c_d3d_text
{
private:
	D3DMATERIAL9 m_mtrl;
	LPDIRECT3DVERTEXBUFFER9 m_pvb;
public:
	c_d3d_dynamic_text():m_pvb(NULL)
	{
	}

	virtual ~c_d3d_dynamic_text()
	{
		release();
	}

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev,
		float Hpix, float Vpix,
		unsigned int size = 24);

	virtual void release(){
		c_d3d_text::release();

		if(m_pvb != NULL){
			m_pvb->Release();
			m_pvb = NULL;
		}
	}

	void get_text_size(float & x, float & y, const char * str);

	void set_starting_point(float & x, float & y, float w, float h,
		e_d3d_txt_center center);
	virtual bool render(LPDIRECT3DDEVICE9 pd3dev, const char * str,
		float x, float y, float scale = 1., float rot = 0.,
		e_d3d_txt_center center = EDTC_LB, DWORD clr = 0xFFFFFFFF);
};


/////////////////////////////////////////////////////////////// c_d3d_ship2d
class c_d3d_ship2d: public c_d3d_2dobj
{
private:
	D3DMATERIAL9 m_mtrl;
	LPDIRECT3DVERTEXBUFFER9 m_pvb;
public:
	c_d3d_ship2d():m_pvb(NULL)
	{
	};

	virtual ~c_d3d_ship2d()
	{
	}

	virtual void release(){
		if(m_pvb != NULL){
			m_pvb->Release();
			m_pvb = NULL;
		}
	}

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev,
		float Hpix, float Vpix, unsigned int size = 10);

	virtual bool render(LPDIRECT3DDEVICE9 pd3dev,
		float x, float y, float scale = 1., float rot = 0, DWORD clr = 0xFFFFFFFF);
};

/////////////////////////////////////////////////////////////// c_d3d_childview
class c_d3d_childview: public c_d3d_2dobj
{
protected:
	float m_Htx, m_Vtx; // size for the render target texture
	D3DMATERIAL9 m_mtrl;
	LPDIRECT3DVERTEXBUFFER9 m_pvb;
	LPDIRECT3DSURFACE9 m_pzbuf;
	LPDIRECT3DTEXTURE9 m_ptex;

	LPDIRECT3DSURFACE9 m_pzbuf_back;
	LPDIRECT3DSURFACE9 m_psurf_back;
	D3DVIEWPORT9 m_vp_back;
public:
	c_d3d_childview():m_pvb(NULL)
	{
	}

	virtual ~c_d3d_childview()
	{
		release();
	}

	virtual void release();

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev, 
		float Htx, float Vtx,
		float Hpix, float Vpix);

	bool SetAsRenderTarget(LPDIRECT3DDEVICE9 pd3dev);
	bool ResetRenderTarget(LPDIRECT3DDEVICE9 pd3dev);

	bool show(LPDIRECT3DDEVICE9 pd3dev, float x, float y, 
		float scale = 1.0, float rot = 0.0);
};

/////////////////////////////////////////////////////////////// c_d3d_camview
class c_ship;

// note: the world coordinate system is ECEF system which is right handed system.
// however, the camera coordinate system is left handed system its line of sight 
// vector is z-axis. 
// We mount the camera on the ship, and the GPS position is set to be the origin 
// of our ECEF coordinate system. Now, at the same origin, we set the camera base
//  coordinate system. 
// Camera's extrinsic parameter should be given relative to the base coordinate.
// here're the camera's extrinsic parameters r1, t1, r2, t2.
// t1 is the translation to the first cam coordinate from base coordinate.
// r1 is the rotation to the first cam coordinate from base coordinate.
// t2 is the translation to the second cam coordinate from first coordinate.
// r2 is the rotation to the second cam coordinate from first coordinate.
// second cam coordinate shoud be used if the camera is ptz camera.

class c_d3d_camview: public c_d3d_childview
{
protected:
	D3DXMATRIX m_M3dprj; // d3d projection matrix
	D3DXMATRIX m_M3dview; // d3d view matrix
	
	LPDIRECT3DVERTEXBUFFER9 m_pvb_sea; // virtex and index
	D3DMATERIAL9 m_mtrl_sea;
	D3DLIGHT9 m_sun_light;
	//LPDIRECT3DINDEXBUFFER9 m_pib_sea;  // buffers for sea surface

	Mat m_Rown;		// Ship rotation matrix (3x3)

	bool m_bptz;
	s_rotpar m_r1, m_r2; // Camera rotation parameter (roll, pitch, yaw)
	Point3d m_t1, m_t2;	 // Camera translation parameter (x, y, z in camera coordinate)

	Mat m_Rcam;		// Camera rotation matrix (3x3)
	Mat m_Tcam;		// Camera translation vector (3x1)
	Mat m_Vcam;		// Normalized vector of the camera direction (3rd row of m_Rcam)
	Mat m_Cam;		// Camera Intrinsic Matrix
	Mat m_View;		// View transform matrix
	Mat m_Mtrn;		// Final transformation matrix

	double m_clip_cos;
	Point3d m_hrzn[72];
	Point2d m_hrzn2[72];
	double m_dip_hrzn;
	double m_hrzn_dist;

	float m_Himg, m_Vimg;
	LPDIRECT3DSURFACE9 m_poffsrf;
	D3DCOLOR m_sys_clr;
public:

	c_d3d_camview():m_sys_clr(D3DCOLOR_ARGB(255, 0, 255, 0)), 
		m_poffsrf(NULL), m_bptz(false), m_pvb_sea(NULL)
	{
		m_View = Mat::eye(3, 3, CV_64FC1);
	}

	virtual ~c_d3d_camview(){
	}

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev, 
		Mat & Cam, s_rotpar & rot, Point3d & Xcam,
		float Himg, float Vimg,
		float Htx, float Vtx,
		float Hscrn, float Vscrn);

	virtual bool init(LPDIRECT3DDEVICE9 pd3dev, 
		float Himg, float Vimg,
		float Htx, float Vtx,
		float Hscrn, float Vscrn);


	virtual void release();

	float get_surface_width(){
		return m_Himg;
	}

	float get_surface_height(){
		return m_Vimg;
	}

	bool blt_offsrf(LPDIRECT3DDEVICE9 pd3dev, Mat & img);

	virtual void calc_prjmtx(s_rotpar & rot_own, Point3d & t_own);

	virtual bool render_hrzn(LPDIRECT3DDEVICE9 pd3dev, 
		c_d3d_dynamic_text & txt, LPD3DXLINE pline);

	virtual bool render_ais(LPDIRECT3DDEVICE9 pd3dev, 
		c_d3d_dynamic_text & txt, LPD3DXLINE pline,
		const list<c_ship*> & ships, double range);

	virtual bool render_sea(LPDIRECT3DDEVICE9 pd3dev);

	virtual bool render_ship(LPDIRECT3DDEVICE9 pd3dev, 
		const list<c_ship*> & ships, double range);

	virtual bool set_campar(s_rotpar & rot, bool bptz = false)
	{	
		if(bptz){
			m_bptz = bptz;
			m_r2 = rot;
		}else
			m_r1 = rot;
		return true;
	}

	virtual bool set_campar(Point3d & t, bool bptz = false){
		if(bptz){
			m_bptz = bptz;
			m_t2 = t;
		}else
			m_t1 = t;

		Point3d ttot = m_t1 + m_t2;

		m_dip_hrzn = (1.0 - TERREF)*sqrt(2.0*ttot.y/AE) * (PI/180);
		m_hrzn_dist = TERREF2 * sqrt(ttot.y) * MILE;
		return true;
	}

	virtual bool set_campar(Mat & Cam){
		Cam.copyTo(m_Cam);

		calc_clip();
		calc_hrzn();
		return true;
	}

	void calc_clip()
	{
		// clip
		double hmax = (m_Himg/2 - m_Cam.at<double>(0, 2)) / m_Cam.at<double>(0, 0);
		double hmin = (-m_Himg/2 - m_Cam.at<double>(0, 2)) / m_Cam.at<double>(0, 0);
		double vmax = (m_Vimg/2 - m_Cam.at<double>(1, 2)) / m_Cam.at<double>(1, 1);
		double vmin = (- m_Vimg/2 - m_Cam.at<double>(1, 2)) / m_Cam.at<double>(1, 1);

		double hamax = max(fabs(hmax), fabs(hmin));
		double vamax = max(fabs(vmax), fabs(vmin));
		m_clip_cos = cos(atan(sqrt(hamax * hamax + vamax * vamax)));
	}

	void calc_hrzn()
	{
		// calculate horizon
		for(int i = 0; i < 72; i++){
			double theta = i * (PI / 36);
			m_hrzn[i].x = m_hrzn_dist * sin(theta);
			m_hrzn[i].y = m_hrzn_dist * cos(theta);
			m_hrzn[i].z = 0.0;
		}
	}
};

/////////////////////////////////////////////////// D3DXLINE based librarys
// These functions should be used inside the Begin()/End()

void drawPoint2d(LPDIRECT3DDEVICE9 pd3dev,	
		c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
		vector<Point2f> & points, vector<int> & valid, 
		int pttype, const int state = 0, const int cur_point = -1);
void xsquare(LPD3DXLINE pline, Point2f & pt, float radius, D3DCOLOR color);
void xcross(LPD3DXLINE pline,  Point2f & pt, float radius, D3DCOLOR color);
void xdiamond(LPD3DXLINE pline,  Point2f & pt, float radius, D3DCOLOR color);
void xdiagonal(LPD3DXLINE pline,  Point2f & pt, float radius, D3DCOLOR color);

#endif