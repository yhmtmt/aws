
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_base is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_base is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_base.  If not, see <http://www.gnu.org/licenses/>. 


#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#ifndef _WIN32
#include <linux/videodev2.h>
#endif

#define XMD_H

#ifdef SANYO_HD5400
#include <jpeglib.h>
#include <curl/curl.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"

#include "../util/thread_util.h"
#include "../util/c_clock.h"

#include "../util/c_ship.h"
#include "../util/c_imgalign.h"
#include "../channel.h"
#include "../filter.h"

ch_base * ch_base::create(const char * type_name, const char * chan_name)
{
	if(strcmp(type_name, "imgc")==0){
		return new ch_image_cln(chan_name);
	}	

	if(strcmp(type_name, "imgr") == 0){
		return new ch_image_ref(chan_name);
	}

	if(strcmp(type_name, "pvt") == 0){
		return new ch_pvt(chan_name);
	}

	if(strcmp(type_name, "nmea") == 0){
		return new ch_nmea(chan_name);
	}

	if(strcmp(type_name, "ais") == 0){
		return new ch_ais(chan_name);
	}

	if(strcmp(type_name, "bmsg") == 0){
		return new ch_vector<s_binary_message>(chan_name);
	}

	if(strcmp(type_name, "ship") == 0){
		return new ch_navdat(chan_name);
	}

	if(strcmp(type_name, "ship_ctrl") == 0){
		return new ch_ship_ctrl(chan_name);
	}

	if(strcmp(type_name, "vrect") == 0){
		return new ch_vector<Rect>(chan_name);
	}

	if(strcmp(type_name, "trck") == 0){
		return new ch_vector<c_track_obj>(chan_name);
	}

	if(strcmp(type_name, "ptz") == 0){
		return new ch_ptz(chan_name);
	}

	if(strcmp(type_name, "ptzc") == 0){
		return new ch_ptzctrl(chan_name);
	}

	if(strcmp(type_name, "campar") == 0){
		return new ch_campar(chan_name);
	}
	return NULL;
}