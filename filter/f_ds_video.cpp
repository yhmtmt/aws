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

#include "../util/thread_util.h"
#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_base.h"
#include "f_cam.h"
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

/////////////////////////////////////////////////////f_ds_vdev members
f_ds_vdev::f_ds_vdev(const char * name):f_ds_video(name), m_pYUVConv(NULL), m_dev(0)
{
	register_fpar("device", &m_dev, "Video device number in integer.");
}

f_ds_vdev::~f_ds_vdev()
{
	close();
}

bool f_ds_vdev::get_src_filter(int dev)
{
	ICreateDevEnum * pCreateDevEnum = NULL;
	IEnumMoniker * pEnumMoniker = NULL;
	IMoniker * pMoniker = NULL;

	try{
		HRESULT hr;

		hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, 
				IID_ICreateDevEnum, (PVOID *)&pCreateDevEnum);
		if(FAILED(hr))
			throw "Failed to create ICreateDevEnum object.";

		hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
				&pEnumMoniker, 0);

		if (pEnumMoniker == NULL) 
			throw "No video device found.";

		pEnumMoniker->Reset();
		ULONG nFetched;
		int idev = 0;
		while(pEnumMoniker->Next(1, &pMoniker, &nFetched) == S_OK){

			if(idev == dev){
				dump_moniker_inf(pMoniker);
				break;
			}

			if(dev < 0){// show device name
				cout << idev << "th device: " << endl;
				dump_moniker_inf(pMoniker);
			}

			pMoniker->Release();
			pMoniker = NULL;
			idev++;
		}

		if(dev < 0)
			throw "choose one.";

		if(idev != dev)
			throw "specified device number is not in range.";

	}catch(char * msg){
		cout << msg << endl;
		if(pCreateDevEnum != NULL)
			pCreateDevEnum->Release();

		if(pEnumMoniker != NULL)
			pEnumMoniker->Release();

		if(pMoniker != NULL)
			pMoniker->Release();

		return false;
	}

	pMoniker->BindToObject(0, 0, IID_IBaseFilter, (void**)&m_pSrcF );
		
	pCreateDevEnum->Release();
	pEnumMoniker->Release();
	pMoniker->Release();


	return true;
}

void f_ds_vdev::dump_moniker_inf(IMoniker * pMoniker)
{
	IPropertyBag *pPropertyBag;
	char str[BUFSIZE_MONIKER_INF];

   // IPropertyBag‚Ébind‚·‚é
	pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pPropertyBag);

	VARIANT var;

	var.vt = VT_BSTR;
	pPropertyBag->Read(L"FriendlyName", &var, 0);
	wcstombs(str, var.bstrVal, BUFSIZE_MONIKER_INF);
	cout << "FriendlyName: " << str << endl;

	VariantClear(&var);
	var.vt = VT_BSTR;
	pPropertyBag->Read(L"Description", &var, 0);
	wcstombs(str, var.bstrVal, BUFSIZE_MONIKER_INF);
	cout <<"Description: " << str << endl;

	VariantClear(&var);

	var.vt = VT_BSTR;
	pPropertyBag->Read(L"DevicePath", &var, 0);
	wcstombs(str, var.bstrVal, BUFSIZE_MONIKER_INF);
	cout << "DevicePath: " << str << endl;

	VariantClear(&var);

   pPropertyBag->Release();
}

bool f_ds_vdev::open(int dev)
{
	try{
		if(!get_src_filter(dev))
			throw "Failed to create source filter.";

		// Create GraphBuilder
		HRESULT hr = CoCreateInstance(CLSID_FilterGraph, NULL,
			CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void**) & m_pGraph);
		if(FAILED(hr))
			throw "Failed to create IGraphBuilder object";

		// creating GrabberFilter
		hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER,
			IID_IBaseFilter, (void**) & m_pGrabberF);
		if(FAILED(hr))
			throw "Failed to create SampleGrabber Filter";

		hr = m_pGraph->AddFilter(m_pGrabberF,L"Sample Grabber");
		if(FAILED(hr))
			throw "Failed to Add SampleGrabber Filter";

		// retrieve Grabber from GrabberFilter
		hr = m_pGrabberF->QueryInterface(IID_ISampleGrabber, (void**) &m_pGrabber);
		if(FAILED(hr))
			throw "Failed to retrieve ISampleGrabber";

		ZeroMemory(&m_mt, sizeof(AM_MEDIA_TYPE));
		m_mt.majortype = MEDIATYPE_Video;
		m_mt.subtype = MEDIASUBTYPE_RGB24;

		hr = m_pGrabber->SetMediaType(&m_mt);
		if(FAILED(hr))
			throw "Failed to set Media type to SampleGrabber";

		// retrieve Control from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaControl, (void**) & m_pControl);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaControl";

		// retrieve Event from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaEvent, (void**) & m_pEvent);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaEvent";

		// retrieve IMediaSeeking from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaSeeking, (void**) & m_pSeek);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaSeeking";
	
		// create and connect source filter
		cout << "Creating source filter" << endl;
/*
		hr = m_pGraph->AddSourceFilter(m_fname, L"Source", &m_pSrcF);
		if(FAILED(hr))
			throw "Failed to create source filter";
*/
		hr = m_pGraph->AddFilter(m_pSrcF, L"Device Filter");
		if(FAILED(hr))
			throw "Failed to create source filter";

		m_pYUVConv = GetFilter("ffdshow raw video filter");
		if(m_pYUVConv == NULL)
			throw "Failed to create YUV Transform filter" ;

		hr = m_pGraph->AddFilter(m_pYUVConv, L"ffdshow raw video filter");
		if(FAILED(hr))
			throw "Failed to add YUV Transform filter";

		hr = ConnectFilters(m_pGraph, m_pSrcF, m_pYUVConv);
		if(FAILED(hr))
			throw "Failed to connect source filter to YUV Transform filter";

		hr = ConnectFilters(m_pGraph, m_pYUVConv, m_pGrabberF);
		if(FAILED(hr))
			throw "Failed to connect source filter";

		// create and connect graphics renderer
		hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, 
			IID_IBaseFilter, (void**) &m_pNullF);
		if(FAILED(hr))
			throw "Failed to creating null renderer filter";
		hr = m_pGraph->AddFilter(m_pNullF, NULL);
		if(FAILED(hr))
			throw "Failed to add null renderer filter";

		hr = ConnectFilters(m_pGraph, m_pGrabberF, m_pNullF);
		if(FAILED(hr))
			throw "Failed to connect null graphics renderer";

		hr = m_pGrabber->SetBufferSamples(TRUE);
		if(FAILED(hr))
			throw "Failed to set buffer samples";

		hr = m_pGrabber->GetConnectedMediaType(&m_mt);
		if(FAILED(hr))
			throw "Failed to get media type";

		m_cbBuffer = m_mt.lSampleSize;

		if((m_mt.formattype == FORMAT_VideoInfo) &&
			(m_mt.cbFormat >= sizeof(VIDEOINFOHEADER)) &&
			(m_mt.pbFormat != NULL))
			m_pVih = (VIDEOINFOHEADER*) m_mt.pbFormat;
		else
			throw "Invalid media type was retrieved from grabber";

		cout << "Graphics size (" << m_pVih->bmiHeader.biHeight << "," << 
			m_pVih->bmiHeader.biWidth << ")" << endl;
		if(m_chout[1]){
			ch_campar * pcamparout = dynamic_cast<ch_campar*>(m_chout[1]);
			if(pcamparout == NULL){
				throw "1st channel is bad ";
			}

			pcamparout->set_Pix(m_pVih->bmiHeader.biWidth, m_pVih->bmiHeader.biHeight);
		}

		hr = m_pControl->Run();

		OAFilterState fs = State_Stopped;
		while(fs != State_Running){
			hr = m_pControl->GetState(10000, &fs);
		}
		///hr = m_pSeek->SetTimeFormat(&TIME_FORMAT_FRAME);
		
		//if(FAILED(hr))
		//	throw "time format frame is not valid";
	}catch(char * msg){
		cout << msg << endl;
		close();
		return false;
	}

	return true;
}

void f_ds_vdev::close()
{
	if(m_pGrabberF != NULL)
		m_pGrabberF->Release();

	if(m_pGrabber != NULL)
		m_pGrabber->Release();
	
	if(m_pNullF != NULL)
		m_pNullF->Release();

	// if(m_pSNullF != NULL)
	//	m_pSNullF->Release();

	if(m_pSrcF != NULL)
		m_pSrcF->Release();

	if(m_pYUVConv != NULL)
		m_pYUVConv->Release();

	if(m_pControl != NULL)
		m_pControl->Release();

	if(m_pEvent != NULL)
		m_pEvent->Release();

	if(m_pSeek != NULL)
		m_pSeek->Release();

	if(m_pGraph != NULL)
		m_pGraph->Release();

	m_pGrabberF = NULL;
	m_pGrabber = NULL;
	m_pNullF = NULL;
	//m_pSNullF = NULL;
	m_pSrcF = NULL;
	m_pControl = NULL;
	m_pEvent = NULL;
	m_pGraph = NULL;
}

bool f_ds_vdev::grab(Mat & img)
{
	HRESULT hr;
	if(img.rows != m_pVih->bmiHeader.biHeight 
		&& img.cols != m_pVih->bmiHeader.biWidth){
		img = Mat(m_pVih->bmiHeader.biHeight, m_pVih->bmiHeader.biWidth, CV_8UC3);
	}

	if(m_pBuffer == NULL){
		m_pBuffer = new unsigned char[m_cbBuffer];
		m_wsize = 3 * m_pVih->bmiHeader.biWidth;
		m_src = m_pBuffer + m_wsize * (m_pVih->bmiHeader.biHeight - 1);
	}

	hr = m_pGrabber->GetCurrentBuffer(&m_cbBuffer, 
		(long*) m_pBuffer);

	unsigned char * dst = img.data;
	unsigned char * src = m_src; 

	for(unsigned int icol = 0; icol < (unsigned int) m_pVih->bmiHeader.biHeight; icol++){
		memcpy(dst, src, m_wsize);
		dst += m_wsize;
		src -= m_wsize;
	}

	if(FAILED(hr))
		return false;

	return true;
}

bool f_ds_vdev::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "open") == 0){
		if(num_args != 4)
			return false;
		return open(atoi(args[itok+1]));
	}

	return f_ds_video::cmd_proc(cmd);
}

////////////////////////////////////////////////////////f_ds_vfile members
f_ds_vfile::f_ds_vfile(const char * name):m_phsplt(NULL), m_pffdec(NULL),
	f_ds_video(name), m_oneshot(false), m_bshot(false), m_fnamew(NULL), m_start_time(0)
{
	register_fpar("file", m_fname, 1024, "File path of the video.");
	register_fpar("abs_time", m_start_time_str, 64, "Absolute start time of the video.");
	register_fpar("oneshot", &m_oneshot, "yes: one shot mode enabled.  no: continuous mode enabled.");
	register_fpar("shot", &m_bshot, "yes: single image grabbed.");
}

f_ds_vfile::~f_ds_vfile()
{
	close();
}

bool f_ds_vfile::open(const char * fname)
{
	int length = (int) strlen(fname) + 1;
	m_fnamew = new wchar_t[length];
	mbstowcs(m_fnamew, fname, length);
	tmex tm;
	if(decTmStr(m_start_time_str, tm))
		m_start_time = mkgmtimeex_tz(tm, m_time_zone_minute) * MSEC;
	else
		m_start_time = (long long) atol(m_start_time_str) * (long long) SEC;

	HRESULT hr;
	try{
		// Create GraphBuilder
		hr = CoCreateInstance(CLSID_FilterGraph, NULL,
			CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void**) & m_pGraph);
		if(FAILED(hr))
			throw "Failed to create IGraphBuilder object";

#ifdef DS_DEBUG
		hr = AddToRot(m_pGraph, &m_dwReg);
		if(FAILED(hr))
			throw "Failed to registor rot table.";
#endif

		// creating GrabberFilter
		hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER,
			IID_IBaseFilter, (void**) & m_pGrabberF);
		if(FAILED(hr))
			throw "Failed to create SampleGrabber Filter";

		hr = m_pGraph->AddFilter(m_pGrabberF,L"Sample Grabber");
		if(FAILED(hr))
			throw "Failed to Add SampleGrabber Filter";

		// retrieve Grabber from GrabberFilter
		hr = m_pGrabberF->QueryInterface(IID_ISampleGrabber, (void**) &m_pGrabber);
		if(FAILED(hr))
			throw "Failed to retrieve ISampleGrabber";

		ZeroMemory(&m_mt, sizeof(AM_MEDIA_TYPE));
		m_mt.majortype = MEDIATYPE_Video;
		m_mt.subtype = MEDIASUBTYPE_RGB24;
		m_mt.formattype = FORMAT_VideoInfo;

		hr = m_pGrabber->SetMediaType(&m_mt);
		if(FAILED(hr))
			throw "Failed to set Media type to SampleGrabber";

		// retrieve Control from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaControl, (void**) & m_pControl);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaControl";

		// retrieve Event from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaEvent, (void**) & m_pEvent);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaEvent";

		// retrieve IMediaSeeking from GraphBuilder
		hr = m_pGraph->QueryInterface(IID_IMediaSeeking, (void**) & m_pSeek);
		if(FAILED(hr))
			throw "Failed to retrieve IMediaSeeking";
	
		// create and connect source filter
		cout << "Creating source filter" << endl;
		hr = m_pGraph->AddSourceFilter(m_fnamew, L"Source", &m_pSrcF);
		if(FAILED(hr))
			throw "Failed to create source filter";
		hr = ConnectFilters(m_pGraph, m_pSrcF, m_pGrabberF);

		if(FAILED(hr)){
			// if sample grabber cannot connect source filter, try inserting ffdshow video decoder
			m_pffdec = GetFilter("ffdshow Video Decoder");
			if(m_pffdec == NULL)
				throw "Failed to create IFFDecoder";

			hr = m_pGraph->AddFilter(m_pffdec,L"ffdshow Video Decoder");
			if(FAILED(hr))
				throw "Failed to Add ffdshow Video Decoder Filter";

			hr = ConnectFilters(m_pGraph, m_pSrcF, m_pffdec);

			if(FAILED(hr))
				throw "Failed to connect source to ffdec filter";

			hr = ConnectFilters(m_pGraph, m_pffdec, m_pGrabberF);
			if(FAILED(hr))
				throw "Failed to connect ffdec to grabber filter";
		}

		// create and connect graphics renderer
		hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, 
			IID_IBaseFilter, (void**) &m_pNullF);
		if(FAILED(hr))
			throw "Failed to creating null renderer filter";
		hr = m_pGraph->AddFilter(m_pNullF, NULL);
		if(FAILED(hr))
			throw "Failed to add null renderer filter";

		hr = ConnectFilters(m_pGraph, m_pGrabberF, m_pNullF);
		if(FAILED(hr))
			throw "Failed to connect null graphics renderer";

		if(m_oneshot){
			hr = m_pGrabber->SetOneShot(TRUE);
		}

		hr = m_pGrabber->SetBufferSamples(TRUE);
		if(FAILED(hr))
			throw "Failed to set buffer samples";

		hr = m_pGrabber->GetConnectedMediaType(&m_mt);
		if(FAILED(hr))
			throw "Failed to get media type";

		if((m_mt.formattype == FORMAT_VideoInfo) &&
			(m_mt.cbFormat >= sizeof(VIDEOINFOHEADER)) &&
			(m_mt.pbFormat != NULL))
			m_pVih = (VIDEOINFOHEADER*) m_mt.pbFormat;
		else
			throw "Invalid media type was retrieved from grabber";

		cout << "Graphics size (" << m_pVih->bmiHeader.biHeight << "," << 
			m_pVih->bmiHeader.biWidth << ")" << endl;
		
		if(m_chout.size() >= 2){
			ch_campar * pcamparout = dynamic_cast<ch_campar*>(m_chout[1]);
			if(pcamparout == NULL){
				throw "1st channel is bad ";
			}

			pcamparout->set_Pix(m_pVih->bmiHeader.biWidth, m_pVih->bmiHeader.biHeight);
		}

		hr = m_pSeek->SetTimeFormat(&TIME_FORMAT_MEDIA_TIME);
		if(FAILED(hr))
			throw "time format frame is not valid";
		
		m_pSeek->GetDuration(&m_duration);
		cout << "length = " << m_duration << endl;

	}catch(char * msg){
		TCHAR ebuf[255];
		AMGetErrorText(hr, ebuf, 255);
		cout << msg << endl;
		delete[] m_fnamew;
		m_fnamew = NULL;
		close();
		return false;
	}

	delete[] m_fnamew;
	m_fnamew = NULL;
	return true;
}

void f_ds_vfile::close()
{
	if(m_pGrabberF != NULL)
		m_pGrabberF->Release();

	if(m_pGrabber != NULL)
		m_pGrabber->Release();
	
	if(m_pNullF != NULL)
		m_pNullF->Release();

	// if(m_pSNullF != NULL)
	//	m_pSNullF->Release();

	if(m_pSrcF != NULL)
		m_pSrcF->Release();

	if(m_pControl != NULL)
		m_pControl->Release();

	if(m_pEvent != NULL)
		m_pEvent->Release();

	if(m_pSeek != NULL)
		m_pSeek->Release();

#ifdef DS_DEBUG
	RemoveFromRot(m_dwReg);
#endif

	if(m_pGraph != NULL)
		m_pGraph->Release();

	m_pGrabberF = NULL;
	m_pGrabber = NULL;
	m_pNullF = NULL;
	//m_pSNullF = NULL;
	m_pSrcF = NULL;
	m_pControl = NULL;
	m_pEvent = NULL;
	m_pSeek = NULL;
	m_pGraph = NULL;
}

bool f_ds_vfile::grab(Mat & img)
{

	HRESULT hr;
	if(img.rows != m_pVih->bmiHeader.biHeight 
		&& img.cols != m_pVih->bmiHeader.biWidth){
		img.create(m_pVih->bmiHeader.biHeight, m_pVih->bmiHeader.biWidth, CV_8UC3);
	}

	long evCode;
	if(m_duration < (m_cur_time - m_start_time)){
		cerr << "Video source filter " << m_name << " finished." << endl;
		return false;
	}

	if(m_oneshot && m_bshot){
		m_pSeek->SetPositions(&m_cur_time_shot, AM_SEEKING_AbsolutePositioning, 
			&m_duration, AM_SEEKING_AbsolutePositioning);
		m_pControl->Run();
		m_pEvent->WaitForCompletion(INFINITE,&evCode);
		m_bshot = false;
	}

	if(m_pBuffer == NULL){
		hr = m_pGrabber->GetCurrentBuffer(&m_cbBuffer, NULL);
		if(FAILED(hr)){
			cerr << "Cannot grab frame." << endl;
			return false;
		}
		m_pBuffer = new unsigned char[m_cbBuffer];
		m_wsize = 3 * m_pVih->bmiHeader.biWidth;
		m_src = m_pBuffer + m_wsize * (m_pVih->bmiHeader.biHeight - 1);
	}

	hr = m_pGrabber->GetCurrentBuffer(&m_cbBuffer, 
		(long*) m_pBuffer);
	if(FAILED(hr)){
		cerr << "Cannot grab frame." << endl;
		return false;
	}

	unsigned char * dst = img.data;
	unsigned char * src = m_src; 

	for(unsigned int icol = 0; icol < (unsigned int) m_pVih->bmiHeader.biHeight; icol++){
		memcpy(dst, src, m_wsize);
		dst += m_wsize;
		src -= m_wsize;
	}

	if(FAILED(hr))
		return false;

	return true;
}

bool f_ds_vfile::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;
	return f_ds_video::cmd_proc(cmd);
}

bool f_ds_vfile::seek(long long seek_time)
{
	seek_time -= m_start_time;
	m_pSeek->SetPositions(&seek_time, AM_SEEKING_AbsolutePositioning, 
		&m_duration, AM_SEEKING_AbsolutePositioning);	
	return true;
}

bool f_ds_vfile::run(long long start_time, long long end_time)
{
	if(!init_run()){
		return false;
	}

	m_prev_time = start_time;

	if(m_pGraph == NULL)
		return false;
	HRESULT hr;
	start_time -= m_start_time;
	end_time -= m_start_time;
	if(end_time < 0)
		end_time = LLONG_MAX;

	end_time = min(m_duration, end_time);
	m_cur_time_shot = start_time;
	hr = m_pSeek->SetPositions(&start_time, AM_SEEKING_AbsolutePositioning, 
		&end_time, AM_SEEKING_AbsolutePositioning);	

	if(FAILED(hr))
		return false;

	hr = m_pControl->Run();

	OAFilterState fs = State_Stopped;
	while(fs != State_Running){
		hr = m_pControl->GetState(10000, &fs);
	}

	if(FAILED(hr))
		return false;

	m_bactive = true;
	m_bstopped = false;

	pthread_create(&m_fthread, NULL, fthread, (void*) this);
	return true;
}

bool f_ds_vfile::stop()
{
	if(f_base::stop()){
		if(m_pGraph != NULL)
			m_pControl->Stop();
		return true;
	}
	return false;
}


////////////////////////////////////////////////////////f_ds_video helpers
HRESULT AddToRot(IUnknown *pUnkGraph, DWORD *pdwRegister) 
{
    IMoniker * pMoniker;
    IRunningObjectTable *pROT;
    if (FAILED(GetRunningObjectTable(0, &pROT))) {
        return E_FAIL;
    }
    WCHAR wsz[256];
    wsprintfW(wsz, L"FilterGraph %08x pid %08x", (DWORD_PTR)pUnkGraph, GetCurrentProcessId());
    HRESULT hr = CreateItemMoniker(L"!", wsz, &pMoniker);
    if (SUCCEEDED(hr)) {
        hr = pROT->Register(ROTFLAGS_REGISTRATIONKEEPSALIVE, pUnkGraph,
            pMoniker, pdwRegister);
        pMoniker->Release();
    }
    pROT->Release();
    return hr;
}

void RemoveFromRot(DWORD pdwRegister)
{
    IRunningObjectTable *pROT;
    if (SUCCEEDED(GetRunningObjectTable(0, &pROT))) {
        pROT->Revoke(pdwRegister);
        pROT->Release();
    }
}


HRESULT GetUnconnectedPin(IBaseFilter *pFilter, PIN_DIRECTION PinDir, IPin **ppPin)
{
    *ppPin = 0;
    IEnumPins *pEnum = 0;
    IPin *pPin = 0;
    HRESULT hr = pFilter->EnumPins(&pEnum);
    if (FAILED(hr))
    {
        return hr;
    }
    while (pEnum->Next(1, &pPin, NULL) == S_OK)
    {
        PIN_DIRECTION ThisPinDir;
        pPin->QueryDirection(&ThisPinDir);
        if (ThisPinDir == PinDir)
        {
            IPin *pTmp = 0;
            hr = pPin->ConnectedTo(&pTmp);
            if (SUCCEEDED(hr))
            {
                pTmp->Release();
            }
            else 
            {
                pEnum->Release();
                *ppPin = pPin;
                return S_OK;
            }
        }
        pPin->Release();
    }
    pEnum->Release();

    return E_FAIL;
}

HRESULT ConnectFilters(IGraphBuilder * pGraph, IBaseFilter * pSrc, IBaseFilter * pDest)
{
    IPin *pOut = 0, *pIn = 0;
    HRESULT hr = GetUnconnectedPin(pSrc, PINDIR_OUTPUT, &pOut);
    if (FAILED(hr)) 
    {
        return hr;
    }
    hr = GetUnconnectedPin(pDest, PINDIR_INPUT, &pIn);
    if (FAILED(hr))
    {
        pIn->Release();
        return hr;
    }
    hr = pGraph->Connect(pOut, pIn);
    pOut->Release();
    pIn->Release();
    return hr;	
}

IBaseFilter * GetFilter(const char * fname)
{
	HRESULT hr;
	IFilterMapper3 * pFM3;
	hr = CoCreateInstance(CLSID_FilterMapper2, NULL, CLSCTX_INPROC_SERVER, 
		IID_IFilterMapper3, (void **)&pFM3);
	
	ICreateDevEnum *pSysDevEnum = NULL;
	hr = pFM3->GetICreateDevEnum(&pSysDevEnum);
	/*
	hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
		IID_ICreateDevEnum, (void **)&pSysDevEnum);
	*/

	if (FAILED(hr))
	{
		return NULL;
	}

	IEnumMoniker *pEnumCat = NULL;
	hr = pSysDevEnum->CreateClassEnumerator(CLSID_LegacyAmFilterCategory, &pEnumCat, 0);
	char str[BUFSIZE_MONIKER_INF];
	IBaseFilter *pFilter = NULL;

	if (hr == S_OK) 
	{
		// Enumerate the monikers.
		IMoniker *pMoniker = NULL;
		ULONG cFetched;
//		cout << "Seeking ffdshow Video Decoder" << endl;
		int ifilter = 0;
		while(pEnumCat->Next(1, &pMoniker, &cFetched) == S_OK && pFilter == NULL)
		{
			IPropertyBag *pPropBag;
			hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, 
				(void **)&pPropBag);
			if (SUCCEEDED(hr))
			{
				VARIANT varName;
				VariantInit(&varName);
				hr = pPropBag->Read(L"FriendlyName", &varName, 0);
				if (SUCCEEDED(hr))
				{
					wcstombs(str, varName.bstrVal, BUFSIZE_MONIKER_INF);
//					cout << ifilter << ":" << str << endl;
					if(strcmp(str, fname) == 0){
						hr = pMoniker->BindToObject(NULL, NULL, IID_IBaseFilter,
							(void**)&pFilter);
					}

				}
				VariantClear(&varName);
				
				pPropBag->Release();
			}
			ifilter++;
			pMoniker->Release();
		}
		pEnumCat->Release();
	}
	pSysDevEnum->Release();
	pFM3->Release();
	return pFilter;
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