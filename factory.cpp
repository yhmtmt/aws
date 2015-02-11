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
#ifndef _WIN32
#include <linux/videodev2.h>
#endif

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
using namespace std;

#define XMD_H
#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;
#include "util/util.h"
#include "util/c_clock.h"
#include "util/coord.h"
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
	register_factory<ch_image_cln>("imgc");
	register_factory<ch_image_ref>("imgr");
	register_factory<ch_pvt>("pvt");
	register_factory<ch_nmea>("nmea");
	register_factory<ch_ais>("ais");
	register_factory<ch_vector<s_binary_message> >("bmsg");
	register_factory<ch_navdat>("ship");
	register_factory<ch_ship_ctrl>("ship_ctrl");
	register_factory<ch_vector<Rect>>("vrect");
	register_factory<ch_vector<c_track_obj>>("trck");
	register_factory<ch_ptz>("ptz");
	register_factory<ch_ptzctrl>("ptzc");
	register_factory<ch_campar>("campar");
	register_factory<ch_ring<char> >("crbuf");
}

////////////////////////////////////////////////// setting up filter factory
// include file list. If you add newly designed your filter, please insert
// your filter's header file here. 
#include "filter/f_base.h"
#include "filter/f_sample.h"
#include "filter/f_misc.h"
#include "filter/f_stabilizer.h"
#include "filter/f_cam.h"
#ifdef AVT_CAM
#include "filter/f_avt_cam.h"
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
#include "filter/f_nmea.h"
#include "filter/f_shioji.h"
#include "filter/f_ship_detector.h"
#include "filter/f_camcalib.h"
#include "filter/f_com.h"
#include "filter/f_event.h"
#include "filter/f_fep01.h"

// Initialization function. 
// This function is called at the begining of the aws process start. If you
// need to initialize global and static data structure please insert your 
// initialization code here.
void f_base::init(c_aws * paws){
	// initializing basic members of the filter graph.
	pthread_mutex_init(&m_mutex, NULL);
	pthread_cond_init(&m_cond,  NULL);
	pthread_mutex_init(&m_err_mtx, NULL);
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

	pthread_mutex_destroy(&m_mutex);
	pthread_cond_destroy(&m_cond);
	pthread_mutex_destroy(&m_err_mtx);
}

// Registration function.
// This function is called at the beginning of the aws process. If you add
// your own filters to the system, please register your filter by inserting
// the code "register_factory<class>("class string").
void f_base::register_factory()
{
	register_factory<f_sample>("sample");
	register_factory<f_nmea>("nmea");
	register_factory<f_nmea_proc>("nmea_proc");

	// image processing
	register_factory<f_imgshk>("imgshk");
	register_factory<f_debayer>("debayer");
	register_factory<f_gry>("gry");
	register_factory<f_edge>("edge");
	register_factory<f_imreg>("imreg");
	register_factory<f_bkgsub>("bkgsub");
	register_factory<f_houghp>("hough");
	register_factory<f_gauss>("gauss");
	register_factory<f_clip>("clip");
	register_factory<f_stabilizer>("stab");
	register_factory<f_tracker>("trck");
	register_factory<f_ship_detector>("shipdet");
	register_factory<f_camcalib>("camcalib");

	register_factory<f_imwrite>("imwrite");

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
	// video sources
#ifdef SANYO_HD5400
	register_factory<f_netcam>("hd5400");
#endif

#ifdef AVT_CAM
	register_factory<f_avt_cam>("avtcam");
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

}
