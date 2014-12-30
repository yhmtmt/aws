
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

CHMap ch_base::m_chmap;

ch_base * ch_base::create(const char * type_name, const char * chan_name)
{	
	ch_base * ptr = NULL;
	try{
		ptr = m_chmap[type_name](chan_name);
	}catch(...){
		ptr = NULL;
	}
	return ptr;
}