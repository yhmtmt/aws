#ifndef _CH_IMAGE_H_
#define _CH_IMAGE_H_
// Copyright(c) 2012-2017 Yohei Matsumoto,  All right reserved. 

// ch_image.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_image.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_image.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_vlib.h"
#include "ch_base.h"

enum e_imfmt{
  IMF_GRAY8, IMF_GRAY10, IMF_GRAY12, IMF_GRAY14, IMF_GRAY16,
  IMF_RGB8, IMF_RGB10, IMF_RGB12, IMF_RGB14, IMF_RGB16,
  IMF_BGR8, IMF_BGR10, IMF_BGR12, IMF_BGR14, IMF_BGR16,
  IMF_BayerBG8, IMF_BayerGB8, IMF_BayerGR8, IMF_BayerRG8,
  IMF_BayerBG10, IMF_BayerGB10, IMF_BayerGR10, IMF_BayerRG10,
  IMF_BayerBG12, IMF_BayerGB12, IMF_BayerGR12, IMF_BayerRG12,
  IMF_NV12, IMF_I420, IMF_Undef
};

extern const char* str_imfmt[IMF_Undef];

enum e_campar{
  ECP_FX = 0, ECP_FY, ECP_CX, ECP_CY, ECP_K1,
  ECP_K2, ECP_P1, ECP_P2, ECP_K3, ECP_K4, ECP_K5, ECP_K6
};

enum e_campar_fish
{
  ECPF_FX = 0, ECPF_FY, ECPF_CX, ECPF_CY, ECPF_K1, ECPF_K2, ECPF_K3, ECPF_K4
};

class ch_image: public ch_base
{
protected:
  e_imfmt fmt;
  int m_back, m_front;
  
  Point2i m_offset;
  Size m_sz_sensor;
  
  Mat m_img[2];
  long long m_time[2]; // frame time (aws time)
  long long m_ifrm[2]; // frame index (if available.)
  long long m_tfile;	 // time fwrite called
  
  mutex m_mtx_bk, m_mtx_fr;

  AWSCamPar m_campar;
  AWSAttitude m_camatt;
	
 public:
 ch_image(const char * name) :ch_base(name), m_front(0), m_back(1), m_tfile(0), m_offset(0, 0), m_sz_sensor(0, 0), fmt(IMF_Undef)
  {
    m_time[0] = m_time[1] = 0;
    m_ifrm[0] = m_ifrm[1] = -1;
  }
  
  virtual ~ch_image()
    {
    }
  
  bool is_new(const long long t)
  {
    return t < m_time[m_front];
  }
  
  void set_offset(const int ox, const int oy){
    m_offset.x = ox;
    m_offset.y = oy;
  }
  
  void get_offset(int & ox, int & oy){
    ox = m_offset.x;
    oy = m_offset.y;
  }
  
  void set_sz_sensor(const int sw, const int sh){
    m_sz_sensor.width = sw;
    m_sz_sensor.height = sh;
  }
  
  void get_sz_sensor(int & sw, int & sh){
    sw = m_sz_sensor.width;
    sh = m_sz_sensor.height;
  }
  
  
  void set_int_campar(const AWSCamPar & _campar)
  {
    lock();
    m_campar = _campar;
    unlock();
  }
  
  void get_int_campar(AWSCamPar & _campar)
  {
    lock();
    _campar = m_campar;
    unlock();
  }
  
  void set_ext_campar(const AWSAttitude & _camatt)
  {
    lock();
    m_camatt = _camatt;
    unlock();
  }
  
  void get_ext_campar(AWSAttitude & _camatt)
  {
    lock();
    _camatt = m_camatt;
    unlock();
  }
   
  virtual Mat get_img(long long & t) = 0;
  virtual Mat get_img(long long & t, long long & ifrm) = 0;
  virtual void set_img(Mat & img, long long t) = 0;
  virtual void set_img(Mat & img, long long t, long long ifrm) = 0;
  
  void set_fmt(const e_imfmt & _fmt)
  {
    lock();
    fmt = _fmt;
    unlock();
  }
  
  const e_imfmt get_fmt()
  {
    return fmt;
  }
  
  // file writer method
  virtual int write(FILE * pf, long long tcur);
  // file reader method
  virtual int read(FILE * pf, long long tcur);
  
  virtual bool log2txt(FILE * pbf, FILE * ptf);
};

// ch_image_cln output clone of the image for get_img
class ch_image_cln: public ch_image
{
 protected:
 public:
 ch_image_cln(const char * name):ch_image(name)
  {
  }
  
  virtual ~ch_image_cln()
    {
    }
  
  virtual Mat get_img(long long & t){
    Mat img;
    unique_lock<mutex> lock(m_mtx_fr);
    if(m_img[m_front].empty()){
      return img;
    }

    img = m_img[m_front].clone();
    t = m_time[m_front];
    return img;
  }
  
  virtual Mat get_img(long long & t, long long & ifrm){
    Mat img;
    unique_lock<mutex> lock(m_mtx_bk);
    if(m_img[m_front].empty()){
      return img;
    }
    
    img = m_img[m_front].clone();
    t = m_time[m_front];
    ifrm = m_ifrm[m_front];
    return img;
  }
  
  virtual void set_img(Mat & img, long long t){
    unique_lock<mutex> lock_bk(m_mtx_bk);
    m_img[m_back] = img;
    m_time[m_back] = t;
    unique_lock<mutex> lock_fr(m_mtx_fr);
    int tmp = m_front;
    m_front = m_back;
    m_back = tmp;
  }
  
  virtual void set_img(Mat & img, long long t, long long ifrm){
    unique_lock<mutex> lock_bk(m_mtx_bk);
    m_img[m_back] = img;
    m_time[m_back] = t;
    m_ifrm[m_back] = ifrm;
    unique_lock<mutex> lock_fr(m_mtx_fr);
    int tmp = m_front;
    m_front = m_back;
    m_back = tmp;
  }
};

// ch_image_ref returns reference of the image for get_img. Don't change the data if you use this as inputs for multiple filters.
class ch_image_ref: public ch_image
{
 protected:
 public:
 ch_image_ref(const char * name): ch_image(name)
  {
  }
  
  virtual ~ch_image_ref(){
  }
  
  virtual Mat get_img(long long & t){
    unique_lock<mutex> lock(m_mtx_fr);
    Mat img = m_img[m_front];
    t = m_time[m_front];
    return img;
  }
  
  virtual Mat get_img(long long & t, long long & ifrm){
    unique_lock<mutex> lock(m_mtx_fr);
    Mat img = m_img[m_front];
    t = m_time[m_front];
    ifrm = m_ifrm[m_front];
    return img;
  }

  virtual Mat get_img_clone(long long & t, long long & ifrm){
    unique_lock<mutex> lock(m_mtx_fr);
    Mat img = m_img[m_front].clone();
    t = m_time[m_front];
    ifrm = m_ifrm[m_front];
    return img;   
  }
  
  virtual void set_img(Mat & img, long long t){
    unique_lock<mutex> lock_bk(m_mtx_bk);
    m_img[m_back] = img;
    m_time[m_back] = t;
    unique_lock<mutex> lock_fr(m_mtx_fr);
    int tmp = m_front;
    m_front = m_back;
    m_back = tmp;
  }
  
  virtual void set_img(Mat & img, long long t, long long ifrm){
    unique_lock<mutex> lock_bk(m_mtx_bk);
    m_img[m_back] = img;
    m_time[m_back] = t;
    m_ifrm[m_back] = ifrm;
    unique_lock<mutex> lock_fr(m_mtx_fr);
    int tmp = m_front;
    m_front = m_back;
    m_back = tmp;
  }
};

#endif
