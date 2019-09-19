#include "stdafx.h"
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// factory is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// factory is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with factory.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#ifndef _WIN32
#include <linux/videodev2.h>
#endif

#ifdef _WIN32
#include <DShow.h>
#include <uuids.h>

//#define DS_DEBUG

//#include <Qedit.h>

// From http://msdn2.microsoft.com/en-us/library/ms786691.aspx:

// Include Qedit.h. This header file is not compatible with Microsoft Direct3D headers later than version 7.

// Since we are using DX9, we cannot include this header. Necessary API elements, copied below.

EXTERN_C const CLSID CLSID_SampleGrabber;

EXTERN_C const CLSID CLSID_NullRenderer;

EXTERN_C const IID IID_ISampleGrabberCB;

MIDL_INTERFACE("0579154A-2B53-4994-B0D0-E773148EFF85")

ISampleGrabberCB : public IUnknown{
public:
	virtual HRESULT STDMETHODCALLTYPE SampleCB(double SampleTime,IMediaSample *pSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE BufferCB(double SampleTime,BYTE *pBuffer,long BufferLen) = 0;
};

EXTERN_C const IID IID_ISampleGrabber;

MIDL_INTERFACE("6B652FFF-11FE-4fce-92AD-0266B5D7C78F")

ISampleGrabber : public IUnknown{
public:
	virtual HRESULT STDMETHODCALLTYPE SetOneShot(BOOL OneShot) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetMediaType(const AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType(AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetBufferSamples(BOOL BufferThem) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer( /* [out][in] */ long *pBufferSize,/* [out] */ long *pBuffer) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentSample( /* [retval][out] */ IMediaSample **ppSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetCallback(ISampleGrabberCB *pCallback,long WhichMethodToCallback) = 0;
};


#if WINVER != 0x603 && WINVER != 0x602
// if not windows 8, the direct 3d is not included in the windows sdk.
//#include <d2d1.h>
//#include <dwrite.h>
#include <d3d9.h>
#endif

#include <d3dx9.h>

#endif

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <thread>
using namespace std;

#define XMD_H
#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif
#include "util/aws_stdlib.h"
#include "util/aws_sock.h"
#include "util/aws_serial.h"
#include "util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;
#include "util/aws_thread.h"
#include "util/aws_coord.h"
#include "util/c_ship.h"
#include "util/c_imgalign.h"

///////////////////////////////////////////////// setting up channel factory
// Include file list. If you add newly designed your channel, please insert
// your channel's header file here. 
#include "channel/ch_base.h"
#include "channel/ch_image.h"
#include "channel/ch_campar.h"
#include "channel/ch_scalar.h"
#include "channel/ch_vector.h"
#include "channel/ch_nmea.h"
#include "channel/ch_ais.h"
#include "channel/ch_navdat.h"
#include "channel/ch_state.h"
#include "channel/ch_aws1_sys.h"
#include "channel/ch_aws1_ctrl.h"
#include "channel/ch_map.h"
#include "channel/ch_obj.h"
#include "channel/ch_wp.h"

#ifdef ORB_SLAM
#include "channel/ch_orb_slam.h"
#endif
#include "channel/ch_aws3.h"

#ifdef RADAR
#include "channel/ch_radar.h"
#endif

// Initialization function. 
// This function is called at the begining of the aws process start. If you
// need to initialize global and static data structure related to channel
// please insert your initialization code here.
void ch_base::init()
{
	register_factory();
}

// Uninitialization function. 
// This function is called at the end of the aws process. If you add your
// initialization code, correspodning destruction code should be added to
// this function.
void ch_base::uninit()
{
}

// Registration function.
// This function is called at the beginning of the aws process. If you add
// your own channels to the system, please register your channel by inserting
// the code "register_factory<class>("class string").
void ch_base::register_factory()
{
	register_factory<ch_sample>("sample");
	register_factory<ch_image_cln>("imgc");
	register_factory<ch_image_ref>("imgr");
	register_factory<ch_pvt>("pvt");
	register_factory<ch_nmea>("nmea");
	register_factory<ch_ais>("ais");
	register_factory<ch_vector<s_binary_message> >("bmsg");
	register_factory<ch_navdat>("ship");
	register_factory<ch_ship_ctrl>("ship_ctrl");

	register_factory<ch_aws1_ctrl_inst>("aws1_ctrl_inst");
	register_factory<ch_aws1_ctrl_stat>("aws1_ctrl_stat");
	register_factory<ch_aws1_ap_inst>("aws1_ap_inst");
	register_factory<ch_vector<Rect>>("vrect");
	register_factory<ch_vector<c_track_obj>>("trck");
	register_factory<ch_ptz>("ptz");
	register_factory<ch_ptzctrl>("ptzc");
	register_factory<ch_campar>("campar");
	register_factory<ch_ring<char, 1024> >("crbuf");
	register_factory<ch_ring<char, 2048> >("crbuf2k");
	register_factory<ch_ring<char, 4096> >("crbuf4k");
	register_factory<ch_ring<char, 8192> >("crbuf8k");
	register_factory<ch_state>("state");
	register_factory<ch_estate>("estate");
	register_factory<ch_eng_state>("engstate");
	register_factory<ch_env>("env");
	register_factory<ch_volt>("volt");
	register_factory<ch_map>("map");
	register_factory<ch_obj>("obj");
	register_factory<ch_obst>("obst");
	register_factory<ch_ais_obj>("ais_obj");
	register_factory<ch_wp>("wp");
	register_factory<ch_route>("route");
	register_factory<ch_aws1_sys>("aws1_sys");

#ifdef ORB_SLAM	
	// orb slam2
	register_factory<ORB_SLAM2::ch_keyframe>("orb_slam_kf");
	register_factory<ORB_SLAM2::ch_keyframeDB>("orb_slam_kfdb");
	register_factory<ORB_SLAM2::ch_map>("orb_slam_map");
	register_factory<ORB_SLAM2::ch_sys>("orb_slam_sys");
	register_factory<ORB_SLAM2::ch_trj>("orb_slam_trj");
	register_factory<ORB_SLAM2::ch_frm>("orb_slam_frm");
#endif
	register_factory<ch_aws3_param>("aws3par");
	register_factory<ch_aws3_cmd>("aws3cmd");
	register_factory<ch_aws3_state>("aws3state");
}
