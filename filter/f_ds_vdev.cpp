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

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_ds_vdev.h"

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
	m_pYUVConv = NULL;
	m_pNullF = NULL;
	//m_pSNullF = NULL;
	m_pSrcF = NULL;
	m_pControl = NULL;
	m_pEvent = NULL;
	m_pSeek = NULL;
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

