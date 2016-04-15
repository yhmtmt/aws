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

#include <opencv2/opencv.hpp>
using namespace cv;
#include "../util/aws_thread.h"
#include "ch_image.h"

bool ch_image_cln::read(f_base * pf, ifstream & fin, long long t)
{
  lock_bk();
  unsigned long long ul;
  m_time[m_back] = t;
  fin.read((char*) &m_ifrm[m_back], sizeof(long long));
  //  cout << "Frame index " << m_ifrm[m_back] << endl;
  fin.read((char*) &ul, sizeof(unsigned long long));
  //  cout << "Compressed Frame size " << ul << endl;
  Mat bufm(1, (int) ul, CV_8UC1);
  fin.read((char*) bufm.data, (streamsize) ul);

  imdecode(bufm, CV_LOAD_IMAGE_ANYDEPTH, &m_img[m_back]);
  //  cout << "Resulting image size " << m_img[m_back].cols << "x" << m_img[m_back].rows << endl; 
  //  cout << "Exiting ch_image::read" << endl;
  lock_fr();

  unlock_fr();
  int tmp = m_front;
  m_front = m_back;
  m_back = tmp;

  unlock_bk();
  return true;
}

bool ch_image_cln::write(f_base * pf, ofstream & fout, long long t)
{
  lock_fr();  
  vector<uchar> buf;
  imencode(".png", m_img[m_front], buf);
  //  cout << "Buffer size " << buf.size() << endl;
  Mat bufm(buf);
  uchar * ptr = bufm.ptr<uchar>();
  unsigned long long ul = (unsigned long long) (bufm.cols * bufm.rows);
  //  cout << "Mat Buffer size " << bufm.cols  << "x" << bufm.rows << endl;
  fout.write((const char*) &m_ifrm[m_front], sizeof(long long));
  fout.write((const char*) &ul, sizeof(unsigned long long));
  fout.write((const char*) ptr, (streamsize) ul);
  //  cout << "Exiting ch_image::write" << endl;
  unlock_fr();
  return true;
}

bool ch_image_ref::read(f_base * pf, ifstream & fin, long long t)
{
  lock_bk();
  unsigned long long ul;
  m_time[m_back] = t;
  fin.read((char*) &m_ifrm[m_back], sizeof(long long));
  //  cout << "Frame index " << m_ifrm[m_back] << endl;
  fin.read((char*) &ul, sizeof(unsigned long long));
  //  cout << "Compressed Frame size " << ul << endl;
  Mat bufm(1, (int) ul, CV_8UC1);
  fin.read((char*) bufm.data, (streamsize) ul);

  imdecode(bufm, CV_LOAD_IMAGE_ANYDEPTH, &m_img[m_back]);
  //  cout << "Resulting image size " << m_img[m_back].cols << "x" << m_img[m_back].rows << endl; 
  //  cout << "Exiting ch_image::read" << endl;
  lock_fr();

  unlock_fr();
  int tmp = m_front;
  m_front = m_back;
  m_back = tmp;

  unlock_bk();
  return true;
}

bool ch_image_ref::write(f_base * pf, ofstream & fout, long long t)
{
  lock_fr();  
  vector<uchar> buf;
  imencode(".png", m_img[m_front], buf);
  //  cout << "Buffer size " << buf.size() << endl;
  Mat bufm(buf);
  uchar * ptr = bufm.ptr<uchar>();
  unsigned long long ul = (unsigned long long) (bufm.cols * bufm.rows);
  //  cout << "Mat Buffer size " << bufm.cols  << "x" << bufm.rows << endl;
  fout.write((const char*) &m_ifrm[m_front], sizeof(long long));
  fout.write((const char*) &ul, sizeof(unsigned long long));
  fout.write((const char*) ptr, (streamsize) ul);
  //  cout << "Exiting ch_image::write" << endl;
  unlock_fr();
  return true;
}
