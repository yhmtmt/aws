#include <cstdio>
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

#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
using namespace std;
#include "../util/aws_stdlib.h"
#include "../util/aws_sock.h"
#include "../util/aws_serial.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;
#include "../util/aws_thread.h"
#include "../util/aws_coord.h"
#include "../util/c_ship.h"
#include "../util/c_imgalign.h"

///////////////////////////////////////////////// setting up channel factory
// Include file list. If you add newly designed your channel, please insert
// your channel's header file here. 
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"
#include "../channel/ch_scalar.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_nmea.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_navdat.h"
#include "../channel/ch_state.h"
#include "../channel/ch_aws1_sys.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_map.h"
#include "../channel/ch_obj.h"
#include "../channel/ch_wp.h"

// include file list. If you add newly designed your filter, please insert
// your filter's header file here. 
#include "../filter/f_base.h"
#include "../filter/f_sample.h"
#include "../filter/f_misc.h"
#include "../filter/f_stereo.h"
#include "../filter/f_stabilizer.h"
#include "../filter/f_cam.h"
#ifdef AVT_CAM
#include "../filter/f_avt_cam.h"
#include "../filter/f_avt_stereo.h"
#include "../filter/f_avt_mono.h"
#endif
#ifdef SANYO_HD5400
#include "../filter/f_netcam.h"
#endif
#include "../filter/f_imgshk.h"
#ifdef _WIN32
#include "../filter/f_imgs.h"
#include "../filter/f_ds_vdev.h"
#include "../filter/f_ds_vfile.h"
#else
#include "../filter/f_uvc_cam.h"
#endif
#ifdef FWINDOW
#include "../filter/f_window.h"
#include "../filter/f_ds_window.h"
#include "../filter/f_sprot_window.h"
#include "../filter/f_sys_window.h"
#include "../filter/f_ptz_window.h"
#include "../filter/f_inspector.h"
#endif

#ifdef GLFW_WINDOW
#include <GLFW/glfw3.h>
#include "../util/aws_glib.h"
#include "../filter/f_glfw_window.h"
#include "../filter/f_glfw_stereo_view.h"
#include "../filter/f_aws1_ui.h"
#endif

#include "../filter/f_nmea.h"
#include "../filter/f_aws1_nmea_sw.h"
#include "../filter/f_aws1_ctrl.h"
#include "../filter/f_shioji.h"
#include "../filter/f_ship_detector.h"
#include "../filter/f_camcalib.h"
#include "../filter/f_com.h"
#include "../filter/f_event.h"
#include "../filter/f_fep01.h"
#include "../filter/f_ahrs.h"
#include "../filter/f_map.h"
#include "../filter/f_time.h"
#include "../filter/f_aws1_ap.h"
#include "../filter/f_obj_manager.h"

#include "aws_stdlib.h"

bool g_kill;

int main(int argc, char ** argv)
{
	if(argc != 3){
		printf("Usage: log2txt <channel type> <log file>\n");
		return 1;
	}

	cout << "Channel type: " << argv[1] << endl;
	cout << "Log file:" << argv[2] << endl;
	char chname[1024];
	char fname[1024];
	
	char * p = argv[2];
	char * q = fname;
	char * u = NULL;
	char * d = NULL;
	char * b = NULL;
	for(; *p != '\0'; p++, q++){
		if(*p == '.')
			d = q;

		if(*p == '_')
			u = q;

		if(*p == '/' || *p == '\\')
			b = q;

		*q = *p;
	}
	*q = '\0';

	if(!d){
		cerr << "Irregal file name " << argv[2] << endl;
		return 1;
	}

	q = d + 1;
	q[0] = 't'; q[1] = 'x'; q[2] = 't'; q[3] = '\0';

	q = chname;
	for(p = b ? b + 1 : argv[2]; p != u; p++, q++){
		*q = *p;
	}
	*q = '\0';

	ch_base::init();

	FILE * pbfile = fopen(argv[2], "rb");
	FILE * ptfile = fopen(fname, "w");

	if(!pbfile){
		cerr << "Failed to open " << argv[2] << endl;
		return 1;
	}

	if(!ptfile){
		cerr << "Failed to open " << fname << endl;
		return 1;
	}

	ch_base * pchan = ch_base::create(argv[1], chname);
	if(!pchan){
		cerr << "Channel type " << argv[1] << " cannot be found." << endl;
		return 1;
	}

	if(!pchan->log2txt(pbfile, ptfile)){
		cerr << "Failed to convert " << argv[2] << "." << endl;
		return 1;
	}

	return 0;
}
