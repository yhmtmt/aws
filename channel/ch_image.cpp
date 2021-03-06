// Copyright(c) 2012-2017 Yohei Matsumoto, All right reserved. 

// ch_image.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_image.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_image.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;
#include "../util/aws_thread.h"
#include "ch_image.h"

const char * str_imfmt[IMF_Undef] = 
{
  "GRAY8", "GRAY10", "GRAY12", "GRAY14", "GRAY16",
  "RGB8", "RGB10", "RGB12", "RGB14", "RGB16",
  "BGR8", "BGR10", "BGR12", "BGR14", "BGR16",
  "BayerBG8", "BayerGB8", "BayerGR8", "BayerRG8",  
  "BayerBG10", "BayerGB10", "BayerGR10", "BayerRG10",
  "BayerBG12", "BayerGB12", "BayerGR12", "BayerRG12",
  "NV12", "I420"
};

#define TIME_VERSION_1_00 14663001811536897L
 
int ch_image::write(FILE * pf, long long tcur)
{
  if(pf){
    unique_lock<mutex> lock(m_mtx_fr);
    Mat img;
    if (!m_img[m_front].empty() && m_tfile < m_time[m_front]){
      img = m_img[m_front].clone();
    }
    lock.unlock();
    if (!img.empty()){
      m_tfile = m_time[m_front];
      int r, c, type, size;
      r = img.rows;
      c = img.cols;
      type = img.type();
      size = (int)(r * c * img.elemSize());
      fwrite((void*)&m_tfile, sizeof(long long), 1, pf);
      fwrite((void*)&m_ifrm[m_front], sizeof(long long), 1, pf);
      fwrite((void*)&type, sizeof(int), 1, pf);
      fwrite((void*)&m_offset, sizeof(m_offset), 1, pf); // from ver.1.00	      
      fwrite((void*)&m_sz_sensor, sizeof(m_sz_sensor), 1, pf); // from ver.1.00	      
      fwrite((void*)&r, sizeof(int), 1, pf);
      fwrite((void*)&c, sizeof(int), 1, pf);
      fwrite((void*)&size, sizeof(int), 1, pf);
      fwrite((void*)img.data, sizeof(char), size, pf);
      return sizeof(long long) + sizeof(m_offset) + sizeof(m_sz_sensor) + 4 * sizeof(int)+size;
    }
  }
  return 0;
}

int ch_image::read(FILE * pf, long long tcur)
{
  if(!pf)
    return 0;
  size_t sz = 0;
  while(m_tfile <= tcur && !feof(pf)){
    long long tsave, ifrm;
    int r, c, type, size;
    size_t res;
    r = c = type = size = 0;
    res = fread((void*)&tsave, sizeof(long long), 1, pf);
    if(!res)
      return 0;
    sz += res;
    
    unique_lock<mutex> lock_bk(m_mtx_bk);
    res = fread((void*)&ifrm, sizeof(long long), 1, pf);
    if(!res)
      goto failed;
    sz += res;
    
    m_ifrm[m_back] = ifrm;
    m_time[m_back] = m_tfile = tsave;
    
    res = fread((void*)&type, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    sz += res;
    
    if(tsave > TIME_VERSION_1_00){
      res = fread((void*)&m_offset, sizeof(m_offset), 1, pf);
      if(!res)
	goto failed;
      sz += res;
      res = fread((void*)&m_sz_sensor, sizeof(m_sz_sensor), 1, pf);
      if(!res)
	goto failed;
      sz += res;
    }
    
    res = fread((void*)&r, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    sz += res;
    
    res = fread((void*)&c, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    sz += res;
    
    res = fread((void*)&size, sizeof(int), 1, pf);
    if(!res)
      goto failed;
    sz += res;
    
    Mat & img = m_img[m_back];
    if(img.type() != type || img.rows != r || img.cols != c){
      img.create(r, c, type);
    }
    res = fread((void*)img.data, sizeof(char), size, pf);
    if(!res)
      goto failed;
    sz += res;

    cout << m_name << " time " << m_time[m_back] << " frm " << m_ifrm[m_back] << " loaded." << endl;
    
    unique_lock<mutex> lock_fr(m_mtx_fr);
    
    int tmp = m_front;
    m_front = m_back;
    m_back = tmp;
    lock_fr.unlock();
    lock_bk.unlock();
  }
  return (int) sz;
 failed:
  return 0;
}

bool ch_image::log2txt(FILE * pbf, FILE * ptf)
{
  char fname[1024];
  long long tprev = 0;
  fprintf(ptf, "t, filename\n");
  while(!feof(pbf)){
    long long tsave, ifrm;
    int r, c, type, size;
    size_t res;
    r = c = type = size = 0;
    
    res = fread((void*)&tsave, sizeof(long long), 1, pbf);
    if(tsave == tprev)
      continue;
    
    res = fread((void*)&ifrm, sizeof(long long), 1, pbf);
    if(!res)
      goto failed;
    
    res = fread((void*)&type, sizeof(int), 1, pbf);
    if(!res)
      goto failed;
    
    if(tsave > TIME_VERSION_1_00){
      res = fread((void*)&m_offset, sizeof(m_offset), 1, pbf);
      if(!res)
	goto failed;
		     
      res = fread((void*)&m_sz_sensor, sizeof(m_sz_sensor), 1, pbf);
      if(!res)
	goto failed;
    }
    
    res = fread((void*)&r, sizeof(int), 1, pbf);
    if(!res)
      goto failed;
    
    res = fread((void*)&c, sizeof(int), 1, pbf);
    if(!res)
      goto failed;
    
    res = fread((void*)&size, sizeof(int), 1, pbf);
    if(!res)
      goto failed;
    
    Mat & img = m_img[m_back];
    if(img.type() != type || img.rows != r || img.cols != c){
      img.create(r, c, type);
    }
    res = fread((void*)img.data, sizeof(char), size, pbf);
    if(!res)
      goto failed;
    snprintf(fname, 1024, "%s_%lld.png", get_name(), tsave);
    fprintf(ptf, "%lld, %s\n", tsave, fname);
    imwrite(fname, img);
    tprev = tsave;
  }
  return true;
 failed:
  return false;
}
