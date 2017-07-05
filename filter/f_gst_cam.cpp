
// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_gst_cam.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_gst_cam.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_gst_cam.cpp  If not, see <http://www.gnu.org/licenses/>. 
#include "stdafx.h"
#include <cmath>
#include <cstring>

#include <iostream>
#include <fstream>
#include <vector>

#include <list>

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_gst_cam.h"


// image format convertor
inline bool cnv_imf(const Mat & src, Mat & dst, const e_imfmt & fmt_in, const e_imfmt & fmt_out)
{
  if(src.empty())
    return false;
  
  if(fmt_in == fmt_out){
    dst = src;
    return true;
  }
  
  switch(fmt_in){
  case FMT_NV12:
    switch(fmt_out){
    case FMT_BGR8:
      cvtColor(src, dst, COLOR_YUV2BGR_NV12);
      return true;
    case FMT_I420:
      cvtColor(src, dst, COLOR_YUV2BGR_I420);
      return true;
    }
  case FMT_BGR8:
    switch(fmt_out){
    case FMT_I420:
      cvtColor(src, dst, COLOR_BGR2YUV_I420);
      return true;
    }
  }
  return false;
}

// Call backs

GstFlowReturn f_gst_cam::new_preroll(GstAppSink * appsink, gpointer data)
{
  f_gst_cam * pcam = (f_gst_cam*) data;
  return GST_FLOW_OK;
}

GstFlowReturn f_gst_cam::new_sample(GstAppSink * appsink, gpointer data)
{
  f_gst_cam * pcam = (f_gst_cam*) data;
  GstSample * sample =  gst_app_sink_pull_sample(appsink);
  if(sample == NULL)
    return GST_FLOW_OK;
  
  GstCaps * caps = gst_sample_get_caps(sample);
  GstStructure * str = gst_caps_get_structure(caps, 0);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  //const GstStructure * info = gst_sample_get_info(sample);
  
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);
  
  Size sz = pcam->get_sz();
  int width, height;
  if(!gst_structure_get_int(str, "width", &width) || 
     !gst_structure_get_int(str, "height", &height)){
    if(sz.width == 0 && sz.height == 0){
      g_print("No width/height available\n");
      return GST_FLOW_OK;
    }
  }else{
    sz.width = width;
    sz.height = height;
  }

  // assuming input is nv12. converting it to bgr
  Mat src;
  switch(fmt_in){
  case IMF_NV12:
  case IMF_I420:
    src = Mat(sz.height + sz.height / 2, sz.width, CV_8UC1, map.data);
    break;
  case IMF_GRAY8:
    src = Mat(sz.height, sz.width, CV_8UC3, map.data);
    break;
  case IMF_RGB8:
  case IMF_BGR8:
    src = Mat(sz.height, sz.width, CV_8UC3, map.data);
    break;
  }
  
  Mat dst;
  if(cnv_imf(src, dst, fmt_in, fmt_out)){
    pcam->set_img(img);
  }
  
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);

  return GST_FLOW_OK;
}

gboolean f_gst_cam::bus_callback(GstBus * bus, GstMessage * message, gpointer data)
{
    f_gst_cam * pcam = (f_gst_cam*) data;
    g_print("Got %s message\n", GST_MESSAGE_TYPE_NAME(message));
    switch(GST_MESSAGE_TYPE(message)){
    case GST_MESSAGE_ERROR:
      {
	GError * err;
	gchar * debug;
	
	gst_message_parse_error(message, &err, &debug);
	g_print("Error: %s\n", err->message);
	g_error_free(err);
	g_free(debug);
	break;
      }
    case GST_MESSAGE_EOS:
      break;
    default:
      break;
    }
    return true;
}

f_gst_cam::f_gst_cam(const char * name): f_base(name), m_ch_out(NULL), m_sz(0, 0), m_descr(NULL), m_ppl(NULL), m_sink(NULL), m_bus(NULL), m_bus_watch_id(-1), m_error(NULL), m_fmt_in(IMF_Undef), m_fmt_out(IMF_Undef)
{
  m_fppl[0] = '\0';
  register_fpar("ch_out", (ch_base**)&m_ch_out, typeid(ch_image_ref).name(), "Channel for image output.");
  register_fpar("width", &m_sz.width, "Width of the image.");
  register_fpar("height", &m_sz.height, "Height of the image.");
  register_fpar("fppl", m_fppl, 1024, "File describes pipe line composition.");
    register_fpar("fmt_in", (int*)&m_fmt_in, (int)IMF_Undef, str_imfmt, "Input image format.");
  register_fpar("fmt_out", (int*)&m_fmt_out, (int)IMF_Undef, str_imfmt, "Output image format.");

}

f_gst_cam::~f_gst_cam()
{
}

bool f_gst_cam::init_run()
{

  // loading m_fppl to m_descr and add sink as "! appsink name=sink sync=true"
  ifstream fppl(m_fppl);
  if(!fppl.is_open()){
    cerr << "Failed to open pipeline description file " << m_fppl << "." << endl;
    return false;
  }

  {
    char tmp[2048], descr[2048];
    fppl.getline(tmp, 2048);
    snprintf(descr, 2048, "%s ! appsink name=sink sync=true", tmp);
    
    m_descr = g_strdup(descr);
    cout << "gst launch with: " << m_descr << endl;
  }

  m_error = NULL;
  
  gst_init(NULL, NULL);
  m_ppl = gst_parse_launch(m_descr, &m_error);

  if(m_error != NULL){
    g_print("Could not construct pipeline %s\n", m_error->message);
    g_error_free(m_error);
    return false;
  }

  m_sink = gst_bin_get_by_name(GST_BIN(m_ppl), "sink");
  gst_app_sink_set_drop((GstAppSink*)m_sink, true);
  gst_app_sink_set_max_buffers((GstAppSink*)m_sink, 1);
  GstAppSinkCallbacks callbacks = {NULL, new_preroll, new_sample};
  gst_app_sink_set_callbacks(GST_APP_SINK(m_sink), &callbacks, (gpointer)this, NULL);
  
  m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_ppl));
  m_bus_watch_id = gst_bus_add_watch(m_bus, bus_callback, (gpointer)this);
  gst_object_unref(m_bus);
  
  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_PLAYING);

  return true;
}

void f_gst_cam::destroy_run()
{
  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_NULL);
  gst_object_unref(m_src); 
  gst_object_unref(GST_OBJECT(m_ppl));
}

bool f_gst_cam::proc()
{
  return true;
}



///////////////////////////////////////////////////////////////// f_gst_cam
void f_gst_enc::need_data(GstAppSrc * src, guint length, gpointer user_data)
{
  ((f_gst_enc*)user_data)->need_data(src, length);
}

void f_gst_enc::enough_data(GstAppSrc * src, gpointer user_data)
{
    ((f_gst_enc*)user_data)->enough_data(src);
}

gboolean f_gst_enc::seek_data(GstAppSrc * src, guint64 offset, gpointer user_data)
{
  return ((f_gst_enc*)user_data)->seek_data(src, offset);
}

gboolean f_gst_enc::bus_callback(GstBus * bus, GstMessage * message, gpointer data)
{
    g_print("Got %s message\n", GST_MESSAGE_TYPE_NAME(message));
    switch(GST_MESSAGE_TYPE(message)){
    case GST_MESSAGE_ERROR:
      {
	GError * err;
	gchar * debug;
	
	gst_message_parse_error(message, &err, &debug);
	g_print("Error: %s\n", err->message);
	g_error_free(err);
	g_free(debug);
	break;
      }
    case GST_MESSAGE_EOS:
      break;
    default:
      break;
    }
    return true;  
}

f_gst_enc::f_gst_enc(const char * name):m_ch_in(NULL), m_sz(0, 0), m_descr(NULL), m_ppl(NULL), m_src(NULL), m_bus(NULL), m_bus_watch_id(-1), m_error(NULL), m_fmt_in(IMF_Undef), m_fmt_out(IMF_Undef)
{
  m_fppl[0] = '\0';
  register_fpar("ch_in", (ch_base**)&m_ch_in, typeid(ch_image_ref).name(), "Channel for image input.");
  register_fpar("fppl", m_fppl, 1024, "File describes pipe line composition.");
  register_fpar("fmt_in", (int*)&m_fmt_in, (int)IMF_Undef, str_imfmt, "Input image format.");
  register_fpar("fmt_out", (int*)&m_fmt_out, (int)IMF_Undef, str_imfmt, "Output image format.");
  register_fpar("width", &m_sz.width, "Width of the image.");
  register_fpar("height", &m_sz.height, "Height of the image.");
  register_fpar("fps", &m_fps, "Frame per second.");
  
}

f_gst_enc::~f_gst_enc()
{
}

bool f_gst_enc::init_run()
{
  if(!m_ch_in){
    cerr << "No input image channel is connected." << endl;
    return false;
  }
  
  ifstream fppl(m_fppl);
  if(!fppl.is_open()){
    cerr << "Failed to open pipeline description file " << m_fppl << "." << endl;
    return false;
  }

  {
    char tmp[2048], descr[2048];
    fppl.getline(tmp, 2048);
    snprintf(descr, 2048, "appsrc name=src ! %s", tmp);
    
    m_descr = g_strdup(descr);
    cout << "gst launch with: " << m_descr << endl;
  }

  m_error = NULL;

  gst_init(NULL, NULL);
  m_ppl = gst_parse_launch(m_descr, &m_error);

  if(m_error != NULL){
    g_print("Could not construct pipeline %s\n", m_error->message);
    g_error_free(m_error);
    return false;
  }

  m_src = gst_bin_get_by_name(GST_BIN(m_ppl), "src");

  const char * sfmt;
  switch(fmt_out){
  case IMF_I420:
  case IMF_NV12:
    sfmt = str_imfmt[IMF_I420];
    break;   
  }
  GstCaps * caps = gst_caps_new_simple("video/x-raw",
				       "format", G_TYPE_STRING, sfmt,
				       "framerate", GST_TYPE_FRACTION, fps, 1,
				       "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1,
				       "width", GTYPE_INT, m_sz.width,
				       "height", GTYPE_INT, m_sz.height);

  gst_app_src_set_caps(GST_APP_SRC(m_src), caps);
  
  GstAppSrcCallbacks callbacks = {nejed_data, enough_data, seek_data};
  gst_app_src_set_callbacks(GST_APP_SRC(m_src), &callbacks, (gpointer)this, NULL);
  
  m_bus = gst_pipeline_get_buf(GST_PIPELINE(m_ppl));
  m_bus_watch_id = gst_bus_add_watch(m_bus, bus_callback, (gpointer)this);
  gst_obj_unref(m_bus);

  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_PLAYING);
  
  return true;
}

void f_gst_enc::destroy_run()
{
  gst_app_src_end_of_stream(m_src);

  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(m_src));  
  gst_object_unref(GST_OBJECT(m_ppl));
}

bool f_gst_enc::proc()
{
  return true;
}

void f_gst_enc::need_data(GstAppSrc * src, guint length)
{
  
  Mat src, dst;
  m_ch_in->get_img(src);
  if(src.empty())
    return ;

  if(!cnv_imf(src, dst, fmt_in, fmt_out)){
    return;
  }

  size_t sz_mem = dst.cols * dst.rows * dst.channels();
  GstBuffer * buf = gst_buffer_new_allocate(Null, sz_mem, NULL);
  GstMapInfo map;
  
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  
  // copy buffer data
  memcpy(map.data, dst.data, sz_mem);
  
  gst_buffer_unmap(buf, &map);

  gst_app_src_push_buffer(src, buf);  
}

void f_gst_enc::enough_data(GstAppSrc * src)
{
  return;
}

gboolean f_gst_enc::seek_data(GstAppSrc * src, guint64 offset)
{
  return true;
}
