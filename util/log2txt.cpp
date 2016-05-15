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

#include <opencv2/opencv.hpp>
using namespace cv;
#include "../util/c_clock.h"
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
#include "../filter/f_glfw_window.h"
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

	ch_base * pchan = ch_base::create(argv[1], chname);
	if(!pchan->log2txt(pbfile, ptfile)){
		cerr << "Failed to convert " << argv[2] << "." << endl;
		return 1;
	}

	return 0;
}