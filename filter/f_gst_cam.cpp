
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
  Mat yuv(sz.height + sz.height / 2, sz.width, CV_8UC1, map.data);
  Mat img;
  cvtColor(yuv, img, CV_YUV2BGR_NV12);
  pcam->set_img(img);
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

f_gst_cam::f_gst_cam(const char * name): f_base(name), m_ch_out(NULL), m_sz(0, 0), m_descr(NULL), m_ppl(NULL), m_sink(NULL), m_bus(NULL), m_bus_watch_id(-1), m_error(NULL)
{
  m_fppl[0] = '\0';
  register_fpar("ch_out", (ch_base**)&m_ch_out, typeid(ch_image_ref).name(), "Channel for image output.");
  register_fpar("width", &m_sz.width, "Width of the image.");
  register_fpar("height", &m_sz.height, "Height of the image.");
  register_fpar("fppl", m_fppl, 1024, "File describes pipe lien compsition.");
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
    //    snprintf(descr, 2048, "%s ! video/x-raw,format=RGB ! videoconvert ! appsink name=sink sync=true", tmp);
    snprintf(descr, 2048, "%s ! appsink name=sink sync=true", tmp);
    
    m_descr = g_strdup(descr);
    cout << "gst launch with: " << m_descr << endl;
  }

  m_error = NULL;
  
  // maybe these three lines are to be in the application initialization code.
  gst_init(NULL, NULL);
  m_ppl = gst_parse_launch(m_descr, &m_error);

  if(m_error != NULL){
    g_print("Could not construct pipeline %s\n", m_error->message);
    g_error_free(m_error);
    return false;
  }

  m_sink = gst_bin_get_by_name(GST_BIN(m_ppl), "sink");
  //  gst_app_sink_set_emit_signals((GstAppSink*)m_sink, true);
  gst_app_sink_set_drop((GstAppSink*)m_sink, true);
  gst_app_sink_set_max_buffers((GstAppSink*)m_sink, 1);
  GstAppSinkCallbacks callbacks = {NULL, new_preroll, new_sample};
  gst_app_sink_set_callbacks(GST_APP_SINK(m_sink), &callbacks, (gpointer)this, NULL);
  
  guint bus_watch_id;
  m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_ppl));
  m_bus_watch_id = gst_bus_add_watch(m_bus, bus_callback, NULL);
  gst_object_unref(m_bus);
  
  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_PLAYING);

  return true;
}

void f_gst_cam::destroy_run()
{
  gst_element_set_state(GST_ELEMENT(m_ppl), GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(m_ppl));
}

bool f_gst_cam::proc()
{
  return true;
}



