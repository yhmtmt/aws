// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

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

int ch_image::write(FILE * pf)
{
	if(pf){
	  lock_fr();
	  if(!m_img[m_front].empty()){
	    if(m_tfile < m_time[m_front]){
	      m_tfile = m_time[m_front];
	      Mat & img = m_img[m_front];
	      int r, c, type, size;
	      r = img.rows;
	      c = img.cols;
	      type = img.type();
	      size = (int)(r * c * img.channels() * img.elemSize());
	      fwrite((void*)&m_tfile, sizeof(long long), 1, pf);
	      fwrite((void*)&type, sizeof(int), 1, pf);
	      fwrite((void*)&r, sizeof(int), 1, pf);
	      fwrite((void*)&c, sizeof(int), 1, pf);
	      fwrite((void*)&size, sizeof(int), 1, pf);
	      fwrite((void*)img.data, sizeof(char), size, pf);
	      unlock_fr();
	      return sizeof(long long) + 4 * sizeof(int) + size;
	    }
	  }
	  unlock_fr();
	}
	return 0;
}

int ch_image::read(FILE * pf, long long tcur)
{
  if(!pf)
		return 0;
  size_t sz = 0;
	while(m_tfile <= tcur && !feof(pf)){
		long long tsave;
		int r, c, type, size;
		size_t res;
		r = c = type = size = 0;
		res = fread((void*)&tsave, sizeof(long long), 1, pf);
		if(!res)
			return 0;
		sz += res;
		lock_bk();
		m_time[m_back] = m_tfile = tsave;

		res = fread((void*)&type, sizeof(int), 1, pf);
		if(!res)
			goto failed;
		sz += res;

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

		lock_fr();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		unlock_fr();

		unlock_bk();
	}
	return (int) sz;
failed:
	unlock_bk();
	return 0;
}

bool ch_image::log2txt(FILE * pbf, FILE * ptf)
{
	char fname[1024];
	long long tprev = 0;
	fprintf(ptf, "t, filename\n");
	while(!feof(pbf)){
		long long tsave;
		int r, c, type, size;
		size_t res;
		r = c = type = size = 0;
		res = fread((void*)&tsave, sizeof(long long), 1, pbf);
		if(tsave == tprev)
			continue;

		res = fread((void*)&type, sizeof(int), 1, pbf);
		if(!res)
			goto failed;
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
