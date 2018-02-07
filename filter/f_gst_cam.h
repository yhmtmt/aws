#ifndef _F_GST_CAM_H_
#define _F_GST_CAM_H_
// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_gst_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_gst_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_gst_cam.h  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include "../channel/ch_image.h"

//#define CV_VIDEO

bool cnv_imf(const Mat & src, Mat & dst, const e_imfmt & fmt_in, const e_imfmt & fmt_out);

class f_gst_cam: public f_base
{
 protected:
  ch_image_ref * m_ch_out;
  long long m_frm_count;
  Size m_sz;
  char m_fppl[1024];
  char m_fts[1024]; 
  FILE * m_pfts;

  bool live;
  bool paused;
  unsigned int m_sz_frmbuf;
  unsigned int m_num_frmbuf;
  unsigned int m_head_frmbuf;
  unsigned int m_tail_frmbuf;
  struct s_frmbuf{
    long long t;
    long long frm;    
    Mat img;
  };
  vector<s_frmbuf> m_frmbuf;
  mutex mtxbuf, mtxppl;
  void lock_buf()
  {
    mtxbuf.lock();
  }

  void unlock_buf()
  {
    mtxbuf.unlock();
  }

  void lock_ppl()
  {
    mtxppl.lock();
  }

  void unlock_ppl()
  {
    mtxppl.unlock();
  }
  
  void init_frmbuf()
  {
    m_frm_count = m_head_frmbuf = m_tail_frmbuf = m_num_frmbuf = 0;
    m_frmbuf.resize(m_sz_frmbuf);
  }

  void control_ppl()
  { 
    if(m_num_frmbuf > (3 * m_sz_frmbuf / 4)){
      if(m_verb)
	cout << "f_gst_cam::control_ppl() paused. num_frmbuf=" << m_num_frmbuf << " sz_frmbuf=" << m_sz_frmbuf << endl;
      gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_PAUSED);
      
      paused = true;
    }else if(paused && m_num_frmbuf < m_sz_frmbuf / 4){
      if(m_verb)
       	cout << "f_gst_cam::control_ppl() restarted." << endl;
      gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_PLAYING);
      paused = false;
    }
  }
  
  bool push_frmbuf(Mat & img, const long long t, const long long frm)
  {
    lock_buf();
    if(m_num_frmbuf == m_sz_frmbuf){
      unlock_buf();
      return false;
    }

    s_frmbuf & frmbuf = m_frmbuf[m_tail_frmbuf];
    frmbuf.img = img;
    frmbuf.t = t;
    frmbuf.frm = frm;
    m_tail_frmbuf++;
    if(m_tail_frmbuf == m_sz_frmbuf)
      m_tail_frmbuf = 0;
    m_num_frmbuf++;

    unlock_buf();
    return true;
  }

  bool pop_frmbuf(Mat & img, long long & t, long long & frm)
  {
    lock_buf();
    bool success = false;
    long long tcur = get_time();
    while(m_head_frmbuf != m_tail_frmbuf &&
	  m_frmbuf[m_head_frmbuf].t <= tcur){
      s_frmbuf & frmbuf = m_frmbuf[m_head_frmbuf];
      img = frmbuf.img;
      t = frmbuf.t;
      frm = frmbuf.frm;
      success = true;
      
      m_head_frmbuf++;
      if(m_head_frmbuf == m_sz_frmbuf)
	m_head_frmbuf = 0;
      
      m_num_frmbuf--;
    }
    unlock_buf();
    return success;
  }
  
  
  gchar * m_descr;
  GstElement * m_ppl;
  GstElement * m_sink;
  GstBus * m_bus;
  guint m_bus_watch_id;
  GError * m_error;
  bool m_verb;
  
  e_imfmt fmt_in, fmt_out; 
  void set_img(Mat & img)
  {
    if(!m_pfts)
      push_frmbuf(img, m_cur_time, m_frm_count);
    else{
      long long t;
      int res = fread((void*)&t, sizeof(t), 1, m_pfts);
      if(res)
	push_frmbuf(img, t, m_frm_count);
      else
	cout << "Time stamp file is in EOF" << endl;
    }
      
    m_frm_count++;
  }

  const Size & get_sz(){
    return m_sz;
  }

 public:
  f_gst_cam(const char * name);
  virtual ~f_gst_cam();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();

  static GstFlowReturn new_preroll(GstAppSink * appsink, gpointer data);
  static GstFlowReturn new_sample(GstAppSink * appsink, gpointer data);
  static gboolean bus_callback(GstBus * bus, GstMessage * message,
			       gpointer data);
};

class f_gst_enc: public f_base
{
 protected:
  ch_image_ref * m_ch_in;
  char m_fppl[1024];
  char m_fts[1024]; // timestamp file
  FILE * m_pfts;
  gchar * m_descr;
  GstElement * m_ppl;
  GstElement * m_src;
  GError * m_error;
  GstBus * m_bus;
  guint m_bus_watch_id;
  int m_fps;
  Size m_sz;
  e_imfmt fmt_in, fmt_out;
  long long tstart;
 public:
  f_gst_enc(const char * name);
  virtual ~f_gst_enc();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();

  static gboolean bus_callback(GstBus * bus, GstMessage * message, gpointer data);
};

#endif
