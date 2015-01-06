// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_video.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_video.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ds_video.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include "f_ds_video.h"

/////////////////////////////////////////////////////f_ds_video members
f_ds_video::f_ds_video(const char * name):f_cam(name), m_pGraph(NULL),
	m_pControl(NULL), m_pEvent(NULL), m_pGrabberF(NULL), m_pNullF(NULL), 
	m_pSrcF(NULL), m_pSeek(NULL), m_pGrabber(NULL), m_pBuffer(NULL)
{
}

f_ds_video::~f_ds_video()
{
	delete [] m_pBuffer;
}

void util_dbg2(int argc, char * argv[])
{
	if(argc != 2)
		return;

	//////////////////////////////////////////////////// construction

	IGraphBuilder * pGraph = NULL;
	AM_MEDIA_TYPE mt;
	IMediaControl * pControl = NULL;
	IMediaEvent * pEvent = NULL;

	IBaseFilter * pGrabberF = NULL;
	IBaseFilter * pNullF = NULL;
	IBaseFilter * pSNullF = NULL;
	IBaseFilter * pSrcF = NULL;

	ISampleGrabber * pGrabber = NULL;

	int length = (int) strlen(argv[1]) + 1;
	wchar_t * wargv = new wchar_t[length];
	mbstowcs(wargv, argv[1], length);

	HRESULT hr = CoInitialize(NULL);
	if(FAILED(hr))
		return;

	// Create GraphBuilder
	hr = CoCreateInstance(CLSID_FilterGraph, NULL,
		CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void**) & pGraph);
	if(FAILED(hr))
		return;

#define GRABBER
#ifdef GRABBER
	// creating GrabberFilter
	hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER,
		IID_IBaseFilter, (void**) & pGrabberF);
	if(FAILED(hr)){
		cout << "Failed to create sample grabber." << endl;
		return;
	}

	hr = pGraph->AddFilter(pGrabberF,L"Sample Grabber");
	if(FAILED(hr)){
		cout << "AddFilter Failed" << endl;
		return;
	}

	// retrieve Grabber from GrabberFilter
	pGrabberF->QueryInterface(IID_ISampleGrabber, (void**) &pGrabber);

	ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
	mt.majortype = MEDIATYPE_Video;
	mt.subtype = MEDIASUBTYPE_RGB24;

	hr = pGrabber->SetMediaType(&mt);
	if(FAILED(hr)){
		cout << "SetMediaType Failed" << endl;
		return;
	}
#endif
	// retrieve Control from GraphBuilder
	hr = pGraph->QueryInterface(IID_IMediaControl, (void**) & pControl);

	// retrieve Event from GraphBuilder
	hr = pGraph->QueryInterface(IID_IMediaEvent, (void**) & pEvent);

#define OPTFILTER
#ifdef OPTFILTER
	// create and connect source filter
	cout << "Creating source filter" << endl;
	hr = pGraph->AddSourceFilter(wargv, L"Source", &pSrcF);
	if(FAILED(hr))
		return;

	hr = ConnectFilters(pGraph, pSrcF, pGrabberF);
	if(FAILED(hr))
		return;

	// create and connect graphics renderer
	cout << "Creating null graphics renderer" << endl;
	hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, 
		IID_IBaseFilter, (void**) &pNullF);
	if(FAILED(hr)){
		cout << "Failed to creating null graphics renderer" << endl;
		return;
	}

	hr = pGraph->AddFilter(pNullF, NULL);
	if(FAILED(hr)){
		cout << "Failed to add null graphics renderer" << endl;
		return;
	}

	hr = ConnectFilters(pGraph, pGrabberF, pNullF);
	if(FAILED(hr)){
		cout << "Failed to connect null graphics renderer" << endl;
		return;
	}

	// create and connect null filter for sound renderer
	/*
	cout << "Creating null sound renderer" << endl;
	hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, 
		IID_IBaseFilter, (void**) &pSNullF);
	if(FAILED(hr)){
		cout << "Failed to create null sound renderer" << endl;
		return;
	}

	hr = pGraph->AddFilter(pSNullF, NULL);
	if(FAILED(hr)){
		cout << "Failed to add null sound renderer" << endl;
		return;
	}

	hr = ConnectFilters(pGraph, pSrcF, pSNullF);
	if(FAILED(hr)){
		cout << "Failed to connect nulll sound renderer" << endl;
		return;
	}
	*/
#else
	hr = pGraph->RenderFile(wargv, NULL);
	if(FAILED(hr)){
		cout << "RenderFile failed" << endl;
		return;
	}
#endif

#ifdef GRABBER
//	hr = pGrabber->SetOneShot(TRUE);

	hr = pGrabber->SetBufferSamples(TRUE);
	if(FAILED(hr))
		return;

	hr = pGrabber->GetConnectedMediaType(&mt);
	if(FAILED(hr))
		return;

	VIDEOINFOHEADER *pVih;
	if((mt.formattype == FORMAT_VideoInfo) &&
		(mt.cbFormat >= sizeof(VIDEOINFOHEADER)) &&
		(mt.pbFormat != NULL))
		pVih = (VIDEOINFOHEADER*) mt.pbFormat;
	else
		return;
	cout << "media size (" << pVih->bmiHeader.biHeight << "," << 
		pVih->bmiHeader.biWidth << ")" << endl;

	Mat frm(pVih->bmiHeader.biHeight, pVih->bmiHeader.biWidth, CV_8UC3);
	long cbBuffer = 0;
	OAFilterState fs;
	hr = pControl->Run();
	hr = pControl->GetState(INFINITE, &fs);
	for(int i= 0; i<1000; i++){
		hr = pGrabber->GetCurrentBuffer(&cbBuffer, NULL);
		cout << "cbBuffer=" << cbBuffer << endl;
		hr = pGrabber->GetCurrentBuffer(&cbBuffer, (long*) frm.data);
		cvWaitKey(2);
	}
/*
	if(SUCCEEDED(hr)){
		long evCode;
		pEvent->WaitForCompletion(INFINITE, &evCode);
	}
*/

#else
	pControl->Run();
	long evCode;
	pEvent->WaitForCompletion(INFINITE, &evCode);
#endif

	/////////////////////////////////////////////////// destruction

#ifdef GRABBER
	pGrabberF->Release();
	pGrabber->Release();
#endif
#ifdef OPTFILTER
	pNullF->Release();
//	pSNullF->Release();
	pSrcF->Release();
#endif

	pControl->Release();
	pEvent->Release();
	pGraph->Release();
	CoUninitialize();
}