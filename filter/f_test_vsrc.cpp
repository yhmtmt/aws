#include "stdafx.h"
// Copyright(c) 2017 Yohei Matsumoto,  All right reserved. 

// f_test_vsrc.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_test_vsrc.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_test_vsrc.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_test_vsrc.h"

f_test_vsrc::f_test_vsrc(const char * name):f_base(name), ch_img(NULL), fmt(IMF_RGB8), width(640), height(480), rdrop(0.0)
{
  register_fpar("ch_img", (ch_base**)&ch_img, typeid(ch_image_ref).name(), "Output Image Channel");
  register_fpar("width", &width, "Image width");
  register_fpar("height", &height, "Image height");
  register_fpar("fmt", (int*)&fmt, (int)IMF_Undef, str_imfmt, "Image format");
  register_fpar("rdrop", &rdrop, "Rate of frame drop.");
  register_fpar("verb", &verb, "Verbose for debug.");
}

f_test_vsrc::~f_test_vsrc()
{
}


bool f_test_vsrc::init_run()
{
  frm = 0;
  if(!ch_img){
    cerr << "Output image channel is not given." << endl;
    return false;
  }
  return true;
}

void f_test_vsrc::destroy_run()
{
}

bool f_test_vsrc::proc()
{
  double rdrop_try = ((double) rand() / (double) RAND_MAX);
  if(rdrop_try < rdrop){
    if(verb)
      cout << "frame[" << frm << "] at " << m_time_str << " dropped." << endl;    
    return true;
  }   
  
  Mat img = Mat::zeros(height, width, CV_8UC3);
  uchar * pdata = img.data;
  uchar r, g, b, sd = (uchar)(get_time() % 256);
  for(int y = 0; y < img.rows; y++){
    for(int x = 0; x < img.cols; x++){
      pdata[0] = (x + sd) % 256;
      pdata[1] = (y + sd) % 256;
      pdata[2] = sd;
      pdata += 3;
    }
  }
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  int baseline = 0;
  Size textSize = getTextSize(m_time_str, fontFace, fontScale, thickness, &baseline);
  Point org((width - textSize.width)/2, (height - textSize.height)/2);
  
  putText(img, m_time_str, org, fontFace, fontScale,
	  Scalar::all(255), thickness);
  
  ch_img->set_fmt(fmt);
  ch_img->set_img(img, get_time(), frm);
  
  frm++;
  return true;
}
		       
