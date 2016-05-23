#ifndef _F_DS_VIDEO_H_
#define _F_DS_VIDEO_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_video.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_video.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

#include "f_cam.h"

class f_ds_video: public f_cam
{
protected:
	AM_MEDIA_TYPE m_mt;
	IGraphBuilder * m_pGraph;
	IMediaControl * m_pControl;
	IMediaEvent * m_pEvent;
	IMediaSeeking * m_pSeek;
	IBaseFilter * m_pGrabberF;
	IBaseFilter * m_pNullF;
//	IBaseFilter * m_pSNullF;
	IBaseFilter * m_pSrcF;
	ISampleGrabber * m_pGrabber;

	long m_cbBuffer;
	unsigned char * m_pBuffer;
	unsigned char * m_src;
	unsigned int m_wsize;

	VIDEOINFOHEADER * m_pVih;

#ifdef DS_DEBUG
	DWORD m_dwReg;
#endif

public:
	f_ds_video(const char * name);
	virtual ~f_ds_video();

	virtual void close()
	{};	
};

void util_dbg2(int argc, char * argv[]);

#define BUFSIZE_MONIKER_INF 256

// helper for connecting graph pin
HRESULT AddToRot(IUnknown *pUnkGraph, DWORD *pdwRegister);
void RemoveFromRot(DWORD pdwRegister);
IBaseFilter * GetFilter(const char * fname);

HRESULT GetUnconnectedPin(IBaseFilter *pFilter, PIN_DIRECTION PinDir, IPin **ppPin);
HRESULT ConnectFilters(IGraphBuilder * pGraph, IBaseFilter * pSrc, IBaseFilter * pDest);

#endif