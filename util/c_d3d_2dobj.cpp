#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_d3d_2dobj.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_d3d_2dobj.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_d3d_2dobj.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#define _USE_MATH_DEFINES
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_thread.h"

#include "../util/aws_coord.h"
#include "../util/c_ship.h"

//////////////////////////////////////////////////////////////// c_d3d_2dobj member
bool c_d3d_2dobj:: set_prjmtx(float hsize, float vsize)
{
	m_Hpix = hsize;
	m_Vpix = vsize;
	D3DXMatrixIdentity(&m_Mproj);

	// Note:
	// The Direct3D projection matrix projects points into normalized 2.0x2.0x1.0 box.
	// The Direct3D projection matrix is multiplied from the right side of the vertices. 
	
	// zn  near field limit
	// zf far field limit
	// final z is to be calculated as q(Z- Zf)/Z = [Zf(Z-Zn)]/[Z(Zf-Zn)].
	// this means Z->Zf, z->1 and Z->Zn, z->0 
	float zn = 100., zf = 1000.;
	float q = zf / (zf - zn);

	// focal distance should be specified in pixel, and noromalized by hsize/2 or vsize/2
	// the normalization makes objects in sight projected in 2.0x2.0 x-y area.
	m_Mproj(0, 0) = (FLOAT)(zn / (0.5 * hsize)); // focal distance in x
	m_Mproj(1, 1) = (FLOAT)(zn / (0.5 * vsize)); // focal distance in y

	m_Mproj(2, 0) = 0.;
	m_Mproj(2, 1) = 0.;
	m_Mproj(2, 2) = (FLOAT)q;
	m_Mproj(3, 2) = (FLOAT)(-q * zn);
	m_Mproj(3, 0) = 0.;
	m_Mproj(3, 1) = 0.;
	m_Mproj(2, 3) = 1.0; // save original z value as w in forth dimension
	m_Mproj(3, 3) = 0.0;

	D3DXMatrixIdentity(&m_Mview);
	m_Mview(1, 1) = 1;
	m_Mview(3, 0) = (FLOAT)(-hsize / 2.0);
	m_Mview(3, 1) = (FLOAT)(vsize / 2.0);

	return true;
}


/////////////////////////////////////////////////////////////// c_d3d_text member
bool c_d3d_text::init(LPDIRECT3DDEVICE9 pd3dev, unsigned int size)
{
	CONST MAT2 m = {{0,1},{0,0},{0,0},{0,1}};
	HFONT hFont = CreateFont(size, 0, 0, 0, 0, FALSE, FALSE, FALSE,
		ANSI_CHARSET, OUT_TT_ONLY_PRECIS, CLIP_DEFAULT_PRECIS, PROOF_QUALITY,
		FIXED_PITCH | FF_MODERN, _T("Terminal"));
	HDC hdc = GetDC(NULL);
	HFONT oldFont = (HFONT) SelectObject(hdc, hFont);
	m_sx = m_sy = 0;
	for(unsigned char c = ' '; c <127; c++){
		GLYPHMETRICS gm;
		TEXTMETRIC tm;
		LPDIRECT3DTEXTURE9 pTx;
		DWORD size = GetGlyphOutline(hdc, (UINT)c, GGO_GRAY4_BITMAP, &gm, 0, NULL, &m);
		BYTE * ptr = new BYTE[size];
		GetGlyphOutline(hdc, (UINT)c, GGO_GRAY4_BITMAP, &gm, size, ptr, &m);
		GetTextMetrics(hdc, &tm);
		pd3dev->CreateTexture(gm.gmCellIncX, tm.tmHeight, 1, D3DUSAGE_DYNAMIC, 
			D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &pTx, NULL);
	
		int org_x, org_y, sx, sy;
		org_x = gm.gmptGlyphOrigin.x;
		org_y = tm.tmAscent - gm.gmptGlyphOrigin.y;
		sx = gm.gmBlackBoxX + (4 - (gm.gmBlackBoxX % 4))%4;
		sy = gm.gmBlackBoxY;
		m_sx = max(gm.gmCellIncX, m_sx);
		m_sy = max((short) tm.tmHeight, m_sy);
		D3DLOCKED_RECT lrc;
		pTx->LockRect(0, &lrc, NULL, D3DLOCK_DISCARD);
		FillMemory(lrc.pBits, lrc.Pitch * tm.tmHeight, 0);
		for(int y = org_y; y < org_y+sy; y++){
			for(int x = org_x; x < org_x+sx; x++){
				DWORD Trans = (255*ptr[x-org_x+sx*(y-org_y)]) / 16;
				DWORD clr = 0x00ffffff | (Trans << 24);
				//DWORD clr =Trans;
				memcpy((BYTE*)lrc.pBits + lrc.Pitch*y+4*x, &clr, sizeof(DWORD));
			}
		}
		pTx->UnlockRect(0);

		m_pTx[c - ' '] = pTx;
		delete[] ptr;
	}

	SelectObject(hdc, oldFont);
	DeleteObject(hFont);
	ReleaseDC(NULL, hdc);	

	return true;
}


///////////////////////////////////////////////////////////// c_d3d_ship2d members

bool c_d3d_ship2d::init(LPDIRECT3DDEVICE9 pd3dev,
	float Hpix, float Vpix,
	unsigned int size)
{

	set_prjmtx(Hpix, Vpix);

	CUSTOMVERTEX3 vtx[3];

	int top, left, right, bottom;
	top = size >> 1;
	right = (size >> 2);
	left = -right;
	bottom = -top;


	vtx[0].x = 0.;
	vtx[0].y = (FLOAT) top;
	vtx[0].z = 0.;
	vtx[0].color = 0xffffffff;

	vtx[1].x = (FLOAT) right;
	vtx[1].y = (FLOAT) bottom;
	vtx[1].z = 0.;
	vtx[1].color = 0xffffffff;

	vtx[2].x = (FLOAT) left;
	vtx[2].y = (FLOAT) bottom;
	vtx[2].z = 0.;
	vtx[2].color = 0xffffffff;

	int bytes = 3 * sizeof(CUSTOMVERTEX3);
	HRESULT hr = pd3dev->CreateVertexBuffer(bytes, 0
		,D3DFVF_CUSTOMVERTEX3, D3DPOOL_DEFAULT, &m_pvb, 0);
	if(FAILED(hr))
		return false;

	VOID * ptr;
	hr = m_pvb->Lock(0, bytes, (void**)&ptr, 0);
	if(FAILED(hr)){
		return false;
	}

	ZeroMemory(&m_mtrl, sizeof(m_mtrl));

	memcpy(ptr, vtx, bytes);
	m_pvb->Unlock();
	return true;
}

bool c_d3d_ship2d::render(LPDIRECT3DDEVICE9 pd3dev,
		float x, float y, float scale, float rot, DWORD clr)
{
	m_mtrl.Ambient.r = 1.f;
	m_mtrl.Ambient.g = 1.f;
	m_mtrl.Ambient.b = 1.f;
	m_mtrl.Ambient.a = 0.f;
	pd3dev->SetMaterial(&m_mtrl);
	pd3dev->SetRenderState(D3DRS_LIGHTING, TRUE);
	pd3dev->SetRenderState(D3DRS_AMBIENT, clr);

	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	pd3dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
	pd3dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
	//pd3dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dev->SetFVF(D3DFVF_CUSTOMVERTEX3);

	D3DXMATRIX Mwrld, Tmp;

	pd3dev->SetTransform(D3DTS_VIEW, &m_Mview);

	pd3dev->SetTransform(D3DTS_PROJECTION, &m_Mproj);
	
	pd3dev->SetStreamSource(0, m_pvb, 0, sizeof(CUSTOMVERTEX3));

	D3DXMatrixScaling(&Mwrld, scale, scale, 1.0);
	D3DXMatrixRotationZ(&Tmp, rot);
	Mwrld *= Tmp;
	D3DXMatrixTranslation(&Tmp, x, -y, 100);
	Mwrld *= Tmp;

	pd3dev->SetTransform(D3DTS_WORLD, &Mwrld);
	pd3dev->DrawPrimitive(D3DPT_TRIANGLELIST, 0, 1);

	return true;
}


////////////////////////////////////////////////////////////// c_d3d_dynamic_text members


bool c_d3d_dynamic_text::init(LPDIRECT3DDEVICE9 pd3dev,
	float Hpix, float Vpix,
	unsigned int size)
{
	if(!c_d3d_text::init(pd3dev, size))
		return false;

	set_prjmtx(Hpix, Vpix);

	CUSTOMVERTEX2 vtx[4];

	float top,left,bottom,right;
	left = bottom = 0;
	right = (float) m_sx;
	top = (float) m_sy;

	vtx[0].x = right;
	vtx[0].y = top;
	vtx[0].z = 0.;
	vtx[0].color = 0xffffffff;
	vtx[0].u = 1.;
	vtx[0].v = 0.;

	vtx[1].x = right;
	vtx[1].y = bottom;
	vtx[1].z = 0.;
	vtx[1].color = 0xffffffff;
	vtx[1].u = 1.;
	vtx[1].v = 1.;

	vtx[2].x = left;
	vtx[2].y = top;
	vtx[2].z = 0.;
	vtx[2].color = 0xffffffff;
	vtx[2].u = 0.;
	vtx[2].v = 0.;

	vtx[3].x = left;
	vtx[3].y = bottom;
	vtx[3].z = 0.;
	vtx[3].color = 0xffffffff;
	vtx[3].u = 0.;
	vtx[3].v = 1.;

	int bytes = 4 * sizeof(CUSTOMVERTEX2);
	HRESULT hr = pd3dev->CreateVertexBuffer(bytes, 0
		,D3DFVF_CUSTOMVERTEX2, D3DPOOL_DEFAULT, &m_pvb, 0);
	if(FAILED(hr))
		return false;

	VOID * ptr;
	hr = m_pvb->Lock(0, bytes, (void**)&ptr, 0);
	if(FAILED(hr)){
		return false;
	}

	ZeroMemory(&m_mtrl, sizeof(m_mtrl));

	memcpy(ptr, vtx, bytes);
	m_pvb->Unlock();
	return true;
}

bool c_d3d_dynamic_text::render(LPDIRECT3DDEVICE9 pd3dev,
	const char * str, float x, float y, float scale, float rot,
	e_d3d_txt_center center, DWORD clr)
{
	m_mtrl.Ambient.r = 1.f;
	m_mtrl.Ambient.g = 1.f;
	m_mtrl.Ambient.b = 1.f;
	m_mtrl.Ambient.a = 0.f;
	m_mtrl.Diffuse.r = 0.f;
	m_mtrl.Diffuse.g = 0.f;
	m_mtrl.Diffuse.b = 0.f;
	m_mtrl.Diffuse.a = 0.f;

	pd3dev->SetMaterial(&m_mtrl);
	pd3dev->SetRenderState(D3DRS_LIGHTING, TRUE);
	pd3dev->SetRenderState(D3DRS_AMBIENT, clr);

	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	pd3dev->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
	pd3dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
	//pd3dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dev->SetFVF(D3DFVF_CUSTOMVERTEX2);

	D3DXMATRIX Mwrld, Tmp;

	pd3dev->SetTransform(D3DTS_VIEW, &m_Mview);

	pd3dev->SetTransform(D3DTS_PROJECTION, &m_Mproj);
	
	pd3dev->SetStreamSource(0, m_pvb, 0, sizeof(CUSTOMVERTEX2));
	int len = (int) strlen(str);

	float wx = (float) m_sx * scale;
	float wy = (float) m_sy * scale;
	float ox = 0, oy = 0;
	set_starting_point(ox, oy, wx * (float)len, wy, center);
	for(;*str != '\0'; str++){
		if(*str == ' '){
			ox += wx;
			continue;
		}

		D3DXMatrixScaling(&Mwrld, scale, scale, 1.0);
		D3DXMatrixTranslation(&Tmp, ox, oy, 0);
		Mwrld *= Tmp;
		D3DXMatrixRotationZ(&Tmp, rot);
		Mwrld *= Tmp;
		D3DXMatrixTranslation(&Tmp, x, -y, 100);
		Mwrld *= Tmp;

		pd3dev->SetTransform(D3DTS_WORLD, &Mwrld);
		LPDIRECT3DTEXTURE9 ptx = gettx(*str);
		pd3dev->SetTexture(0, ptx);
		pd3dev->DrawPrimitive(D3DPT_TRIANGLESTRIP, 0, 2);
		ox += wx;
	}

	pd3dev->SetTexture(0, NULL);
	return true;
}

void c_d3d_dynamic_text::get_text_size(float & x, float & y,
	const char * str)
{
	x = (float)(m_sx * strlen(str));
	y = (float) m_sy;
}

void c_d3d_dynamic_text::set_starting_point(float & x, float & y,
	float w, float h, e_d3d_txt_center center)
{
	switch(center){
	case EDTC_LB:
	case EDTC_LC:
	case EDTC_LT:
		break;
	case EDTC_CB:
	case EDTC_CC:
	case EDTC_CT:
		x -= (float)(0.5 * w);
		break;
	case EDTC_RB:
	case EDTC_RC:
	case EDTC_RT:
		x -= w;
	}

	switch(center){
	case EDTC_LB:
	case EDTC_CB:
	case EDTC_RB:
		break;
	case EDTC_LC:
	case EDTC_CC:
	case EDTC_RC:
		y -= (float)(0.5 * h);
		break;
	case EDTC_LT:
	case EDTC_CT:
	case EDTC_RT:
		y -= h;
	}
}

////////////////////////////////////////////////////////////// c_d3d_childview

bool c_d3d_childview::init(LPDIRECT3DDEVICE9 pd3dev, 
		float Htx, float Vtx,
		float Hpix, float Vpix)
{

	set_prjmtx(Hpix, Vpix);

	float top,left,bottom,right;
	left = bottom = 0;
	right = (float) Htx;
	top = (float) Vtx;
	m_Htx = Htx;
	m_Vtx = Vtx;

	CUSTOMVERTEX2 vtx[4];

	vtx[0].x = right;
	vtx[0].y = top;

	vtx[0].z = 0.;
	vtx[0].color = 0xffffffff;
	vtx[0].u = 1.;
	vtx[0].v = 0.;

	vtx[1].x = right;
	vtx[1].y = bottom;
	vtx[1].z = 0.;
	vtx[1].color = 0xffffffff;
	vtx[1].u = 1.;
	vtx[1].v = 1.;

	vtx[2].x = left;
	vtx[2].y = top;
	vtx[2].z = 0.;
	vtx[2].color = 0xffffffff;
	vtx[2].u = 0.;
	vtx[2].v = 0.;

	vtx[3].x = left;
	vtx[3].y = bottom;
	vtx[3].z = 0.;
	vtx[3].color = 0xffffffff;
	vtx[3].u = 0.;
	vtx[3].v = 1.;

	int bytes = 4 * sizeof(CUSTOMVERTEX2);
	HRESULT hr = pd3dev->CreateVertexBuffer(bytes, 0
		,D3DFVF_CUSTOMVERTEX2, D3DPOOL_DEFAULT, &m_pvb, 0);
	if(FAILED(hr))
		return false;

	VOID * ptr;
	hr = m_pvb->Lock(0, bytes, (void**)&ptr, 0);
	if(FAILED(hr)){
		return false;
	}

	ZeroMemory(&m_mtrl, sizeof(m_mtrl));

	memcpy(ptr, vtx, bytes);
	m_pvb->Unlock();

	hr = pd3dev->CreateTexture((UINT)Htx, (UINT)Vtx, 1, D3DUSAGE_RENDERTARGET, 
		D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &m_ptex, NULL);
	if(FAILED(hr))
		return false;

	hr = pd3dev->CreateDepthStencilSurface((UINT)Htx, (UINT)Vtx, 
		D3DFMT_D16, D3DMULTISAMPLE_NONE, 0, TRUE, &m_pzbuf, NULL);
	if(FAILED(hr))
		return false;

	return true;
}

void c_d3d_childview::release()
{
	if(m_pvb != NULL){
		m_pvb->Release();
		m_pvb = NULL;
		m_pzbuf->Release();
		m_pzbuf = NULL;
		m_ptex->Release();
		m_ptex = NULL;
	}
}

// notice: 
// this function hacks the geo-pipe to map objects's triangle size directly to pixel.
// in this function x, y is view coordinate which is left top corner is the origin.
// the trick is below.
// 1. world transformation flips the sign of y. object's Z is set to clipping Zn.
// 2. view(camera) transformation adds the offset values -h/2 and v/2. 
// 3. The focal length of the projection matrix is also set as Zn.
bool c_d3d_childview::show(LPDIRECT3DDEVICE9 pd3dev, float x, float y, float scale, float rot)
{
	m_mtrl.Ambient.r = 1.f;
	m_mtrl.Ambient.g = 1.f;
	m_mtrl.Ambient.b = 1.f;
	m_mtrl.Ambient.a = 0.f;
	pd3dev->SetMaterial(&m_mtrl);
	pd3dev->SetRenderState(D3DRS_LIGHTING, TRUE);
	pd3dev->SetRenderState(D3DRS_AMBIENT, D3DCOLOR_RGBA(255, 255, 255, 0));

	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	pd3dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
	pd3dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
//	pd3dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dev->SetFVF(D3DFVF_CUSTOMVERTEX2);

	D3DXMATRIX Mwrld, Tmp;

	pd3dev->SetTransform(D3DTS_VIEW, &m_Mview);

	pd3dev->SetTransform(D3DTS_PROJECTION, &m_Mproj);
	
	pd3dev->SetStreamSource(0, m_pvb, 0, sizeof(CUSTOMVERTEX2));
//	float ox = 0, oy = 0;

	D3DXMatrixScaling(&Mwrld, scale, scale, 1.0);
//	D3DXMatrixTranslation(&Tmp, ox, oy, 0);
//	Mwrld *= Tmp;
	D3DXMatrixRotationZ(&Tmp, rot);
	Mwrld *= Tmp;
	D3DXMatrixTranslation(&Tmp, x, -y, 100);
	Mwrld *= Tmp;

	pd3dev->SetTransform(D3DTS_WORLD, &Mwrld);
	pd3dev->SetTexture(0, m_ptex);
	pd3dev->DrawPrimitive(D3DPT_TRIANGLESTRIP, 0, 2);
	pd3dev->SetTexture(0, NULL);
	return true;
}

bool c_d3d_childview::SetAsRenderTarget(LPDIRECT3DDEVICE9 pd3dev)
{
	HRESULT hr;
	hr = pd3dev->GetRenderTarget(0, &m_psurf_back);
	if(FAILED(hr))
		return false;
	hr = pd3dev->GetDepthStencilSurface(&m_pzbuf_back);
	if(FAILED(hr))
		return false;

	hr = pd3dev->GetViewport(&m_vp_back);
	if(FAILED(hr))
		return false;

	LPDIRECT3DSURFACE9 psurf;
	hr = m_ptex->GetSurfaceLevel(0, &psurf); 
	if(FAILED(hr))
		return false;

	hr = pd3dev->SetRenderTarget(0, psurf);
	if(FAILED(hr))
		return false;

	hr = pd3dev->SetDepthStencilSurface(m_pzbuf);
	if(FAILED(hr))
		return false;

	psurf->Release();
	return true;
}

bool c_d3d_childview::ResetRenderTarget(LPDIRECT3DDEVICE9 pd3dev)
{
	HRESULT hr;

	hr = pd3dev->SetRenderTarget(0, m_psurf_back);
	if(FAILED(hr))
		return false;

	hr = pd3dev->SetDepthStencilSurface(m_pzbuf_back);
	if(FAILED(hr))
		return false;

	hr = pd3dev->SetViewport(&m_vp_back);
	if(FAILED(hr))
		return false;
	m_psurf_back->Release();
	m_pzbuf_back->Release();

	return true;
}

//////////////////////////////////////////////////////////////////////// c_d3d_camview members

bool c_d3d_camview::init(LPDIRECT3DDEVICE9 pd3dev, 
				Mat & Cam, s_rotpar & rot, Point3d & Xcam,
				float Himg, float Vimg,
				float Htx, float Vtx,
				float Hscrn, float Vscrn)
{
	if(!c_d3d_childview::init(pd3dev, Htx, Vtx, Hscrn, Vscrn))
		return false;

	m_Himg = Himg;
	m_Vimg = Vimg;
	HRESULT hr = pd3dev->CreateOffscreenPlainSurface((UINT) Himg, (UINT) Vimg, 
			D3DFMT_X8R8G8B8,  D3DPOOL_DEFAULT, &m_poffsrf, NULL);

	if(FAILED(hr))
		return false;

	m_View.at<double>(0, 0) = Htx/Himg; 
	m_View.at<double>(1, 1) = -Vtx/Vimg;
	m_View.at<double>(0, 2) = 0.5 * (double) Htx;
	m_View.at<double>(1, 2) = 0.5 * (double) Vtx;

	m_Cam = Mat::eye(3, 3, CV_64FC1);
	set_campar(rot);
	set_campar(Xcam);
	set_campar(Cam);


	CUSTOMVERTEX4 v[38];

	const float range = 100000;

	for(int i = 1; i < 38; i++){
		double theta = (double) (i - 1) * 2 * PI / 36.;
		v[i].x = (float) (range * sin(theta));
		v[i].y = (float) (range * cos(theta));
		v[i].z = 0.0;
		v[i].n = D3DXVECTOR3(0, 0, -1);
	}

	v[0].x = 0;
	v[0].y = 0;
	v[0].z = 0;
	v[0].n = D3DXVECTOR3(0, 0, -1);

	int bytes = 38 * sizeof(CUSTOMVERTEX4);
	hr = pd3dev->CreateVertexBuffer(bytes, 0, 
		D3DFVF_CUSTOMVERTEX4, D3DPOOL_DEFAULT, 
		&m_pvb_sea, 0);
	if(FAILED(hr))
		return false;
	VOID * ptr;
	hr = m_pvb_sea->Lock(0, bytes, (void**) & ptr, 0);
	if(FAILED(hr))
		return false;

	memcpy(ptr, v, bytes);

	m_pvb_sea->Unlock();

	ZeroMemory(&m_mtrl_sea, sizeof(m_mtrl_sea));
	m_mtrl_sea.Ambient.r = 0.f;
	m_mtrl_sea.Ambient.g = 0.f;
	m_mtrl_sea.Ambient.b = 1.f;
	m_mtrl_sea.Ambient.a = 1.f;
	m_mtrl_sea.Diffuse.r = 0.f;
	m_mtrl_sea.Diffuse.g = 0.f;
	m_mtrl_sea.Diffuse.b = 1.f;
	m_mtrl_sea.Diffuse.a = 1.f;

	ZeroMemory(&m_sun_light, sizeof(m_sun_light));

	m_sun_light.Diffuse.r = 0.5f;
	m_sun_light.Diffuse.g = 0.5f;
	m_sun_light.Diffuse.b = 0.5f;
	m_sun_light.Diffuse.a = 0.5f;
//	m_sun_light.Range = 500000;
	m_sun_light.Type = D3DLIGHT_DIRECTIONAL;
	m_sun_light.Direction.x = 0.f;
	m_sun_light.Direction.y = 0.f;
	m_sun_light.Direction.z = 1.f;
	
	calc_clip();
	calc_hrzn();

	return true;
}

bool c_d3d_camview::init(LPDIRECT3DDEVICE9 pd3dev, 
				float Himg, float Vimg,
				float Htx, float Vtx,
				float Hscrn, float Vscrn)
{
	if(!c_d3d_childview::init(pd3dev, Htx, Vtx, Hscrn, Vscrn))
		return false;

	m_Himg = Himg;
	m_Vimg = Vimg;
	HRESULT hr = pd3dev->CreateOffscreenPlainSurface((UINT) Himg, (UINT) Vimg, 
			D3DFMT_X8R8G8B8,  D3DPOOL_DEFAULT, &m_poffsrf, NULL);

	if(FAILED(hr))
		return false;

	return true;
}

void c_d3d_camview::release()
{
	c_d3d_childview::release();
	if(m_poffsrf != NULL){
		m_poffsrf->Release();
		m_poffsrf = NULL;
	}

	if(m_pvb_sea != NULL){
		m_pvb_sea->Release();
		m_pvb_sea = NULL;
	}
}

bool c_d3d_camview::blt_offsrf(LPDIRECT3DDEVICE9 pd3dev, Mat & img)
{
	if(img.empty())
		return true;

	D3DLOCKED_RECT lrc;
	RECT rc;

	int Vimg = (int) m_Vimg;
	int Himg = (int) m_Himg;
	SetRect(&rc, 0, 0, Himg, Vimg);

	HRESULT hr;
	hr = m_poffsrf->LockRect(&lrc, &rc, D3DLOCK_DISCARD);
	if(FAILED(hr))
		return false;
	unsigned char * pdst = (unsigned char*) lrc.pBits;
	unsigned char * psrc = (unsigned char*) img.data;
	if(img.type() == CV_8UC3){
		for(int i = 0; i < Vimg; i++){
			unsigned char * ptmp = pdst;
			for(int j = 0; j < Himg; j++){
				*ptmp = *psrc; // Blue
				ptmp++;
				psrc++;
				*ptmp = *psrc; // Green
				ptmp++;
				psrc++;
				*ptmp = *psrc; // Red
				ptmp++;
				psrc++;
				*ptmp = 255; // alpha channel
				ptmp++;
			}
			pdst += lrc.Pitch;
		}
	}else if(img.type() == CV_8UC1){
		for(int i = 0; i < Vimg; i++){
			unsigned char * ptmp = pdst;
			for(int j = 0; j < Himg; j++){
				*ptmp = *psrc; // Blue
				ptmp++;
				*ptmp = *psrc; // Green
				ptmp++;
				*ptmp = *psrc; // Red
				ptmp++;
				psrc++;
				*ptmp = 255; // alpha channel
				ptmp++;
			}
			pdst += lrc.Pitch;
		}
	}else{
		return false;
	}

	m_poffsrf->UnlockRect();
	
	// transfer input image to back surface from offscreen surface
	LPDIRECT3DSURFACE9 pbksr;
	D3DVIEWPORT9 Vp;
	pd3dev->GetViewport(&Vp);
	SetRect(&rc, Vp.X, Vp.Y,
		Vp.X + Vp.Width,	Vp.Y + Vp.Height);
	pd3dev->GetRenderTarget(0, &pbksr);
	//m_pd3dev->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &pbksr);
	pd3dev->StretchRect(m_poffsrf, NULL, pbksr, &rc, D3DTEXF_LINEAR);
	pbksr->Release();
	return true;
}


void c_d3d_camview::calc_prjmtx(s_rotpar & rot_own, Point3d & t_own)
{
	// R0 : world to camera base coordinate (z-y swapping) matrix
	// t_own: translation of the ship
	// R1 : ship rotation matrix
	// m_t1: translation of the camera first coordinate from camera base coordinate 
	// R2 : first rotation matrix of the cam
	// m_t2: translation to the camera second coordinate (pan-tilt centered) from camera first coordinate
	// R3 : second rotation matrix of the cam
	// m_t3: translation to the camera center from the pan-tilt center 
	// m_t2, R3, m_t3 are only for pt camera
	Mat R0 = Mat::zeros(3, 3, CV_64FC1); 
	R0.at<double>(0, 0) = 1.;
	R0.at<double>(1, 2) = 1.;
	R0.at<double>(2, 1) = 1.;
	Mat R1;
	getmatrotRPY(rot_own, R1); // ship rotation 
	
	Mat R2; 
	getmatrotRPY(m_r1, R2); // getting camera rotation matrix on the ship
	
	Mat R3;
	if(m_bptz)
		getmatrotRPY(m_r2, R3); 
	else
		R3 = Mat::eye(3, 3, CV_64FC1);
	
	m_Tcam = Mat(t_own);

	m_Tcam -= R1 * Mat(m_t1);

	R2 *= R1;
	R3 *= R2;
	if(m_bptz){
		m_Tcam -= R3 * Mat(m_t2);
	}

	m_Rcam = R3 * R0;

	// setting rotation and translation to view transformation matrix
	m_Mtrn = Mat(3, 4, CV_64FC1);
	m_Rcam.copyTo(m_Mtrn(Range(0, 3), Range(0, 3)));

	double * ptr = m_Tcam.ptr<double>(0);
	m_Mtrn.at<double>(0, 3) = ptr[0];
	m_Mtrn.at<double>(1, 3) = ptr[1];
	m_Mtrn.at<double>(2, 3) = ptr[2];

	ptr = m_Mtrn.ptr<double>(0);
	m_M3dview._11 = (float) ptr[0]; m_M3dview._21 = (float) ptr[1]; m_M3dview._31 = (float) ptr[2]; m_M3dview._41 = (float) ptr[3];
	
	ptr = m_Mtrn.ptr<double>(1);
	m_M3dview._12 = (float) ptr[0]; m_M3dview._22 = (float) ptr[1]; m_M3dview._32 = (float) ptr[2]; m_M3dview._42 = (float) ptr[3];

	ptr = m_Mtrn.ptr<double>(2);
	m_M3dview._13 = (float) ptr[0]; m_M3dview._23 = (float) ptr[1]; m_M3dview._33 = (float) ptr[2]; m_M3dview._43 = (float) ptr[3];
	m_M3dview._14 = m_M3dview._24 = m_M3dview._34 = 0.;
	m_M3dview._44 = 1.;

	// calcurating camera direction vector (normalized)
	m_Vcam = m_Rcam(Range(2, 3), Range(0, 3));

	// setting direct3d prj matrix
	float zn = 10., zf = 100000.;
	float q = zf / (zf - zn);
	D3DXMatrixIdentity(&m_M3dprj);
	m_M3dprj._11 = (FLOAT)(2. * m_Cam.at<double>(0, 0) / (double) m_Himg);
	m_M3dprj._22 = (FLOAT)(2. * m_Cam.at<double>(1, 1) / (double) m_Vimg);
	m_M3dprj._33 = (FLOAT) q;
	m_M3dprj._43 = (FLOAT) -q * zn;
	m_M3dprj._34 = 1.0;
	m_M3dprj._44 = 0.0;

	// multiply pojection matrix
	m_Mtrn = m_Cam * m_Mtrn;
	m_Mtrn = m_View * m_Mtrn;
}

bool c_d3d_camview::render_ais(LPDIRECT3DDEVICE9 pd3dev, 
		c_d3d_dynamic_text & txt, LPD3DXLINE pline,
		const list<c_ship*> & ships, double range)
{
	txt.set_prjmtx(m_Htx, m_Vtx);
	for(list<c_ship*>::const_iterator itr = ships.begin(); itr != ships.end(); itr++){
		if(!(*itr)->is_inrange(range))
			continue;

		if(!(*itr)->is_insight(m_Vcam, m_clip_cos))
			continue;

		(*itr)->render(pd3dev, pline, txt, m_Mtrn);
	}
	return true;
}

bool c_d3d_camview::render_sea(LPDIRECT3DDEVICE9 pd3dev)
{
	pd3dev->SetMaterial(&m_mtrl_sea);
	pd3dev->SetRenderState(D3DRS_AMBIENT, D3DXCOLOR(0.5, 0.5, 0.5, 0.5));
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	pd3dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
	pd3dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
	pd3dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dev->SetFVF(D3DFVF_CUSTOMVERTEX4);

	pd3dev->SetStreamSource(0, m_pvb_sea, 0, sizeof(CUSTOMVERTEX4));

	pd3dev->SetTransform(D3DTS_VIEW, &m_M3dview);
	pd3dev->SetTransform(D3DTS_PROJECTION, &m_M3dprj);

	D3DXMATRIX Mwrld;
	D3DXMatrixIdentity(&Mwrld);
	pd3dev->SetTransform(D3DTS_WORLD, &Mwrld);
	pd3dev->DrawPrimitive(D3DPT_TRIANGLEFAN, 0, 36);

	return true;
}

bool c_d3d_camview::render_ship(LPDIRECT3DDEVICE9 pd3dev, 
	const list<c_ship*> & ships, double range)
{
	pd3dev->SetLight(0, &m_sun_light);
	pd3dev->LightEnable(0, TRUE);
	pd3dev->SetRenderState(D3DRS_LIGHTING, TRUE);
	pd3dev->SetTransform(D3DTS_VIEW, &m_M3dview);
	pd3dev->SetTransform(D3DTS_PROJECTION, &m_M3dprj);
	float fd = 0.0001f;
	float fs = 100.f;
	float fe = 5000.f;
	pd3dev->SetRenderState(D3DRS_FOGENABLE, TRUE);
	pd3dev->SetRenderState(D3DRS_FOGCOLOR, 0x007F7F7F);
//	pd3dev->SetRenderState(D3DRS_FOGTABLEMODE, D3DFOG_EXP);
//	pd3dev->SetRenderState(D3DRS_FOGDENSITY, *(DWORD*)&fd);
	pd3dev->SetRenderState(D3DRS_FOGTABLEMODE, D3DFOG_LINEAR);
	pd3dev->SetRenderState(D3DRS_FOGSTART, *(DWORD*) &fs);
	pd3dev->SetRenderState(D3DRS_FOGEND, *(DWORD*) &fe);

	for(list<c_ship*>::const_iterator itr = ships.begin();
		itr != ships.end(); itr++){
		if(!(*itr)->is_inrange(range))
			continue;

		if(!(*itr)->is_insight(m_Vcam, m_clip_cos))
			continue;

		(*itr)->set_render_state(pd3dev);
		(*itr)->render3d(pd3dev);
	}

	return true;
}

bool c_d3d_camview::render_hrzn(LPDIRECT3DDEVICE9 pd3dev,
	c_d3d_dynamic_text & txt, LPD3DXLINE pline)
{
	txt.set_prjmtx(m_Htx, m_Vtx);
	char buf[4];
	double inv_dist_hrzn = 1.0 / m_hrzn_dist;
	D3DXVECTOR2 v[2];
	double * ptr = m_Vcam.ptr<double>(0);

	for(int i = 0; i < 72; i++){
		double clip_cos = ptr[0] * m_hrzn[i].x + ptr[1] * m_hrzn[i].y + ptr[2] * m_hrzn[i].z;
		clip_cos *= inv_dist_hrzn;
		//clip_cos = fabs(clip_cos);
		if(clip_cos < m_clip_cos)
			continue;
		

		trans(m_Mtrn, m_hrzn[i], m_hrzn2[i]);
		int x = (int) m_hrzn2[i].x;
		int y = (int) m_hrzn2[i].y;
		if(i % 2 == 0){
			sprintf(buf, "%d", i * 5);
			v[0] = D3DXVECTOR2((float)m_hrzn2[i].x, (float)m_hrzn2[i].y - 50);
			v[1] = D3DXVECTOR2((float)m_hrzn2[i].x, (float)m_hrzn2[i].y - 30);
			pline->Begin();
			pline->Draw(v, 2, m_sys_clr);
			pline->End();
			txt.render(pd3dev, buf,
				(float)m_hrzn2[i].x, (float)(m_hrzn2[i].y - 50),
				1.0, 0.0, EDTC_CB, m_sys_clr);
		}else{
			v[0] = D3DXVECTOR2((float)m_hrzn2[i].x, (float)m_hrzn2[i].y - 45);
			v[1] = D3DXVECTOR2((float)m_hrzn2[i].x, (float)m_hrzn2[i].y - 35);
			pline->Begin();
			pline->Draw(v, 2, m_sys_clr);
			pline->End();
			continue;
		}
	}

	return true;
}

/////////////////////////////////////////////////////////////////D3DXLINE based libraries
void drawPoint2d(LPDIRECT3DDEVICE9 pd3dev,	
		c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
		vector<Point2f> & points, vector<int> & valid, 
		int pttype, const int state, const int cur_point)
{
	// state 0: NORMAL -> 127
	// state 1: STRONG -> 255
	// state 2: GRAY -> 127,127,127
	// state 3: WHITE -> 255,255,255

	pttype %= 12; // {sq, dia, x, cross} x {red, green, blue}
	int shape = pttype % 4;
	int val;
	if(state == 0 || state == 2){
		val = 128;
	}else if(state == 1 || state == 3){
		val = 255;
	}

	D3DCOLOR color;
	if(state < 2){
		switch(pttype / 4){
		case 0:
			color = D3DCOLOR_RGBA(val, 0, 0, 255);
			break;
		case 1:
			color = D3DCOLOR_RGBA(0, val, 0, 255);
			break;
		case 2:
			color = D3DCOLOR_RGBA(0, 0, val, 255);
			break;
		}
	}else{
		color = D3DCOLOR_RGBA(val, val, val, 255);
	}

	D3DXVECTOR2 v[5];

	int size = 5;
	pline->Begin();
	for(int ipt = 0; ipt < points.size(); ipt++){
		if(!valid[ipt])
			continue;
		Point2f & pt = points[ipt];
		if(ipt == cur_point){
			v[0] = D3DXVECTOR2((float)(pt.x - 1.0), (float)(pt.y));
			v[1] = D3DXVECTOR2((float)(pt.x - 3.0), (float)(pt.y));
			pline->Draw(v, 2, color);
			v[0].x += 4.0;
			v[1].x += 4.0;
			pline->Draw(v, 2, color);
			v[0] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - 1.0));
			v[1] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - 3.0));
			pline->Draw(v, 2, color);
			v[0].y += 4.0;
			v[1].y += 4.0;
			pline->Draw(v, 2, color);
			continue;
		}


		switch(shape){
		case 0: // square
			xsquare(pline, pt, 2, color);
			break;
		case 1: // diamond
			xdiamond(pline, pt, 2, color);
			break;
		case 2: // X
			xdiagonal(pline, pt, 2, color);
			break;
		case 3:
			xcross(pline, pt, 2, color);
			break;
		}

		if(ptxt != NULL){
			char buf[10];
			sprintf(buf, "%d", ipt);
			ptxt->render(pd3dev, buf, pt.x, (float)(pt.y + 3.0), 1.0, 0.0, EDTC_CB, color); 
		}
	}
	pline->End();
}


void xsquare(LPD3DXLINE pline, Point2f & pt, float radius, D3DCOLOR color)
{
	D3DXVECTOR2 v[5];
	v[0] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y - radius));
	v[1] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y + radius));
	v[2] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y + radius));
	v[3] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y - radius));
	v[4] = v[0];
	pline->Draw(v, 5, color);
}

void xcross(LPD3DXLINE pline, Point2f & pt, float radius, D3DCOLOR color)
{
	D3DXVECTOR2 v[5];
	v[0] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - radius));
	v[1] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y));
	v[2] = D3DXVECTOR2((float)(pt.x), (float)(pt.y + radius));
	v[3] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y));
	v[4] = v[0];
	pline->Draw(v, 5, color);
}

void xdiamond(LPD3DXLINE pline, Point2f & pt, float radius, D3DCOLOR color)
{
	D3DXVECTOR2 v[2];
	v[0] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y - radius));
	v[1] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y + radius));
	pline->Draw(v, 2, color);
	v[0] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y + radius));
	v[1] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y - radius));
	pline->Draw(&v[2], 2, color);
}

void xdiagonal(LPD3DXLINE pline, Point2f & pt, float radius, D3DCOLOR color)
{		
	D3DXVECTOR2 v[4];
	v[0] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - radius));
	v[1] = D3DXVECTOR2((float)(pt.x), (float)(pt.y + radius));
	pline->Draw(v, 2, color);
	v[2] = D3DXVECTOR2((float)(pt.x - radius), (float)(pt.y));
	v[3] = D3DXVECTOR2((float)(pt.x + radius), (float)(pt.y));
	pline->Draw(&v[2], 2, color);
}

