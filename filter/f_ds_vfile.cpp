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

#include "f_ds_vfile.h"

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

	if(m_phsplt != NULL)
		m_phsplt->Release();

	if(m_pffdec != NULL)
		m_pffdec->Release();

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

	m_phsplt = NULL;
	m_pffdec = NULL;
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
	m_bactive = true;
	m_bstopped = false;
	m_count_proc =  0;	
	m_max_cycle = 0;
	m_cycle = 0;
	m_count_pre = m_count_post = m_count_clock;

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
