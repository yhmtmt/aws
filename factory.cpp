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

ISampleGrabberCB : public IUnknown {
public:
	virtual HRESULT STDMETHODCALLTYPE SampleCB( double SampleTime,IMediaSample *pSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE BufferCB( double SampleTime,BYTE *pBuffer,long BufferLen) = 0;
};

EXTERN_C const IID IID_ISampleGrabber;

MIDL_INTERFACE("6B652FFF-11FE-4fce-92AD-0266B5D7C78F")

ISampleGrabber : public IUnknown {
public:
	virtual HRESULT STDMETHODCALLTYPE SetOneShot( BOOL OneShot) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetMediaType( const AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetConnectedMediaType( AM_MEDIA_TYPE *pType) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetBufferSamples( BOOL BufferThem) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentBuffer( /* [out][in] */ long *pBufferSize,/* [out] */ long *pBuffer) = 0;
	virtual HRESULT STDMETHODCALLTYPE GetCurrentSample( /* [retval][out] */ IMediaSample **ppSample) = 0;
	virtual HRESULT STDMETHODCALLTYPE SetCallback( ISampleGrabberCB *pCallback,long WhichMethodToCallback) = 0;
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
  register_factory<ch_env>("env");
	register_factory<ch_map>("map");
	register_factory<ch_obj>("obj");
	register_factory<ch_obst>("obst");
	register_factory<ch_ais_obj>("ais_obj");
	register_factory<ch_wp>("wp");
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

////////////////////////////////////////////////// setting up filter factory
// include file list. If you add newly designed your filter, please insert
// your filter's header file here. 
#include "filter/f_base.h"
#include "filter/f_sample.h"
#include "filter/f_misc.h"
#ifdef STEREO
#include "filter/f_stereo.h"
#endif
#include "filter/f_stabilizer.h"
#include "filter/f_cam.h"
#ifdef AVT_CAM
#include "filter/f_avt_cam.h"
#include "filter/f_avt_stereo.h"
#include "filter/f_avt_mono.h"
#endif
#ifdef AVT_VMB_CAM
#include "filter/f_avt_vmb_cam.h"
#endif

#ifdef SANYO_HD5400
#include "filter/f_netcam.h"
#endif
#include "filter/f_imgshk.h"
#ifdef _WIN32
#include "filter/f_imgs.h"
#include "filter/f_ds_vdev.h"
#include "filter/f_ds_vfile.h"
#else
#include "filter/f_uvc_cam.h"
#endif
#ifdef FWINDOW
#include "filter/f_window.h"
#include "filter/f_ds_window.h"
#include "filter/f_sprot_window.h"
#include "filter/f_sys_window.h"
#include "filter/f_ptz_window.h"
#include "filter/f_inspector.h"
#endif

#ifdef GLFW_WINDOW
#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>

#include "util/aws_glib.h"
#include "filter/f_glfw_window.h"
#include "filter/f_glfw_stereo_view.h"
#include "filter/f_aws1_ui.h"
#include "filter/f_aws1_sim.h"
#include "filter/f_aws3_ui.h"
#include "filter/f_state_estimator.h"
#endif

#include "filter/f_nmea.h"
#include "filter/f_aws1_nmea_sw.h"
#include "filter/f_aws1_ctrl.h"
#include "filter/f_shioji.h"
#include "filter/f_ship_detector.h"
#ifdef CAMCALIB
#include "filter/f_camcalib.h"
#endif
#include "filter/f_com.h"
#include "filter/f_event.h"
#include "filter/f_fep01.h"
#include "filter/f_ahrs.h"
#include "filter/f_map.h"
#include "filter/f_time.h"
#include "filter/f_aws1_ap.h"
#include "filter/f_obj_manager.h"
#include "filter/f_wp_manager.h"
#include "filter/f_env_sensor.h"

#ifdef ORB_SLAM
#include "filter/f_orb_slam.h"
#endif

#include "filter/f_aws3_com.h"


#ifdef GST_CAM
#include "filter/f_gst_cam.h"
#endif

// Initialization function. 
// This function is called at the begining of the aws process start. If you
// need to initialize global and static data structure please insert your 
// initialization code here.
void f_base::init(c_aws * paws){
	// initializing basic members of the filter graph.
	m_err_head = 0;
	m_err_tail = 0;
	m_file_err.open(FILE_FERR_LOG);
	m_paws = paws;

	// registering filters
	f_base::register_factory();

	// initializing AVT PvAPI
#ifdef AVT_CAM
	f_avt_cam::init_interface();
#endif

	// Insert your own initialization code
}

// Uninitialization function. 
// This function is called at the end of the aws process. If you add your
// initialization code, correspodning destruction code should be added to
// this function.
void f_base::uninit()
{
	// destructing AVT PvAPI
#ifdef AVT_CAM
	f_avt_cam::destroy_interface();
#endif

	flush_err_buf();
	m_file_err.close();
}

// Registration function.
// This function is called at the beginning of the aws process. If you add
// your own filters to the system, please register your filter by inserting
// the code "register_factory<class>("class string").
void f_base::register_factory()
{
	register_factory<f_sample>("sample");
	register_factory<f_nmea>("nmea");
	register_factory<f_aws1_nmea_sw>("aws1_nmea_sw");
#ifndef _WIN32
	register_factory<f_aws1_ctrl>("aws1_ctrl");
#endif

	// image processing
#ifdef IMGSHK
	register_factory<f_imgshk>("imgshk");
#endif
#ifdef MISC
	register_factory<f_debayer>("debayer");
	register_factory<f_gry>("gry");
	register_factory<f_edge>("edge");
	register_factory<f_imreg>("imreg");
	register_factory<f_bkgsub>("bkgsub");
	register_factory<f_houghp>("hough");
	register_factory<f_gauss>("gauss");
	register_factory<f_clip>("clip");
	register_factory<f_imwrite>("imwrite");
	register_factory<f_imread>("imread");
	register_factory<f_lcc>("lcc");
	register_factory<f_bkg_mask>("bkgmsk");
#endif
#ifdef STABILIZER
	register_factory<f_stabilizer>("stab");
#endif
#ifdef SHIP_DETECTOR
	register_factory<f_ship_detector>("shipdet");
#endif

#ifdef CAMCALIB
	register_factory<f_camcalib>("camcalib");
#endif
#ifdef STEREO
	register_factory<f_stereo>("stereod");
#endif

	// windows
#ifdef FWINDOW
	register_factory<f_window>("window");
	register_factory<f_mark_window>("mwin");
#ifdef _WIN32
	register_factory<f_ds_window>("dswin");
	register_factory<f_sys_window>("syswin");
	register_factory<f_sprot_window>("spwin");
	register_factory<f_ptz_window>("ptzwin");
	register_factory<f_inspector>("inspector");
#endif
#endif

#ifdef GLFW_WINDOW
	register_factory<f_glfw_window>("glwin");
	register_factory<f_glfw_stereo_view>("glstvw");
	register_factory<f_glfw_imview>("glimv");
	register_factory<f_aws1_ui>("aws1_ui");
	register_factory<f_aws1_sim>("aws1_sim");
	register_factory<f_aws3_ui>("aws3_ui");
	register_factory<f_glfw_calib>("gcalib");
	register_factory<f_state_estimator>("stest");
	register_factory<f_est_viewer>("estv");
	register_factory<f_glfw_test3d>("test3d");
#endif

	// video sources
#ifdef SANYO_HD5400
	register_factory<f_netcam>("hd5400");
#endif

#ifdef AVT_CAM
	register_factory<f_avt_stereo>("avtstereo");
	register_factory<f_avt_mono>("avtmono");
#endif

#ifdef AVT_VMB_CAM
	register_factory<avt_vmb_cam::f_avt_vmb_cam_impl<1>>("vmbmono");
#endif

#ifdef UVC_CAM
	register_factory<f_uvc_cam>("uvcam");
#endif

#ifdef _WIN32
	register_factory<f_ds_vfile>("vfile");
	register_factory<f_ds_vdev>("vdev");
#endif
	// communication
	register_factory<f_trn_img>("trnimg");
	register_factory<f_rcv_img>("rcvimg");
	register_factory<f_trn>("trn");
	register_factory<f_rcv>("rcv");

	// event 
	register_factory<f_event>("evt");

	register_factory<f_fep01>("fep01");
	register_factory<f_serial>("ser");
	register_factory<f_udp>("udp");
	register_factory<f_ch_share>("ch_share");
	register_factory<f_write_ch_log>("write_ch_log");
	register_factory<f_read_ch_log>("read_ch_log");
	register_factory<f_dummy_data>("dd");
	register_factory<f_rec_data>("rd");

	register_factory<f_time>("tsync");

	register_factory<f_ahrs>("ahrs");
  register_factory<f_env_sensor>("env");

	register_factory<f_aws1_ap>("aws1_ap");
	register_factory<f_obj_manager>("obj_manager");

	register_factory<f_map>("map");
	register_factory<f_wp_manager>("wp_manager");
#ifdef ORB_SLAM
	// orb slam
	register_factory<ORB_SLAM2::f_local_mapper>("orb_slam_local_mapper");
	register_factory<ORB_SLAM2::f_loop_closer>("orb_slam_loop_closer");
	register_factory<ORB_SLAM2::f_tracker>("orb_slam_tracker");
	register_factory<ORB_SLAM2::f_viewer>("orb_slam_viewer");
#endif

	register_factory<f_aws3_com>("aws3c");
#ifdef GST_CAM
	register_factory<f_gst_cam>("gstcam");
#endif
}
