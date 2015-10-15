#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;
#include "../util/aws_thread.h"
#include "ch_image.h"

bool ch_image_ref::read(f_base * pf, ifstream & fin, long long t)
{
  lock();
  //  cout << "Entering ch_image::read" << endl;
  fin.read((char*) &m_bnew, sizeof(bool));
  if(!m_bnew){
    unlock();
    return true;
  }
  //  cout << "New Image found" << endl;
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
  unlock();
  return true;
}

bool ch_image_ref::write(f_base * pf, ofstream & fout, long long t)
{
  lock();
  //  cout << "Entering ch_image::write" << endl;
  fout.write((const char*) &m_bnew, sizeof(bool));
  if(!m_bnew){
    unlock();
    return true;
  }
  
  vector<uchar> buf;
  imencode(".png", m_img[m_back], buf);
  //  cout << "Buffer size " << buf.size() << endl;
  Mat bufm(buf);
  uchar * ptr = bufm.ptr<uchar>();
  unsigned long long ul = (unsigned long long) (bufm.cols * bufm.rows);
  //  cout << "Mat Buffer size " << bufm.cols  << "x" << bufm.rows << endl;
  fout.write((const char*) &m_ifrm[m_back], sizeof(long long));
  fout.write((const char*) &ul, sizeof(unsigned long long));
  fout.write((const char*) ptr, (streamsize) ul);
  //  cout << "Exiting ch_image::write" << endl;
  unlock();
  return true;
}
