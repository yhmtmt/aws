// Copyright(c) 2019 Yohei Matsumoto, All right reserved.
// f_radar.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_radar.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_radar.cpp.  If not, see <http://www.gnu.org/licenses/>.

// This program is based on radar_pi developed under GPLv2.
// See https://github.com/opencpn-radar-pi/radar_i

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <queue>
#include <map>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include "f_radar.h"

const char * f_radar::str_radar_command_id[RC_NONE] = {
  "txoff", "txon", "range", "bearing_alignment",
  "no_transmit_start", "no_transmit_end",
  "gain", "sea", "rain", "interference_rejection",
  "scan_speed", "timed_idle", "timed_run", "img"
};

const int f_radar::range_vals[16] = { 250, 500, 750, 1000, 1500, 2000, 3000, 4000, 6000, 8000, 12000, 16000, 24000, 36000, 48000, 64000 };


f_radar::f_radar(const char * name): f_base(name),
				     interface_address(172,16,1,1,0),
				     receive(this, interface_address, gx_report, gx_data),
				     control(gx_send), cmd(RC_NONE, 0, RCS_OFF)
{
  register_fpar("state", (ch_base**)&state, typeid(ch_state).name(), "State channel");
  register_fpar("radar_state", (ch_base**)&radar_state, typeid(ch_radar_state).name(), "Radar state channel");
  register_fpar("radar_image", (ch_base**)&radar_image, typeid(ch_radar_image).name(), "Radar image channel");
  register_fpar("radar_ctrl", (ch_base**)&radar_ctrl, typeid(ch_radar_ctrl).name(), "Radar control channel");
  
  register_fpar("cmd_id", (int*)&cmd.id, (int)RC_NONE, str_radar_command_id, "Command ID");
  register_fpar("cmd_val", &cmd.val, "Command value");
  register_fpar("cmd_state", &cmd_state, "Radar Control State OFF:-1, MANUAL:0, AUTO1-9:1-9");
  
}

bool f_radar::proc()
{
  if(cmd.id != RC_NONE){
    radar_ctrl->push(cmd.id, cmd.val, cmd.state);
    cmd.id = RC_NONE;      
  }
  
  radar_command_id id;
  int val;
  RadarControlState state;
  while(radar_ctrl->pop(id, val, state))
    {
      switch(id){
      case RC_TXOFF:
	control.RadarTxOff();
	break;
      case RC_TXON:
	control.RadarTxOn();
	break;
      case RC_RANGE:
	control.SetRange(find_nearest_range(val));
	break;
      case RC_BEARING_ALIGNMENT:
	control.SetControlValue(CT_BEARING_ALIGNMENT, val, state);
	break;
      case RC_NO_TRANSMIT_START:
	control.SetControlValue(CT_NO_TRANSMIT_START, val, state);
	break;	  
      case RC_NO_TRANSMIT_END:
	control.SetControlValue(CT_NO_TRANSMIT_END, val, state);
	break;
      case RC_GAIN:
	control.SetControlValue(CT_GAIN, val, state);
	break;
      case RC_SEA:
	control.SetControlValue(CT_SEA, val, state);
	break;
      case RC_RAIN:
	control.SetControlValue(CT_RAIN, val, state);
	break;
      case RC_INTERFERENCE_REJECTION:
	control.SetControlValue(CT_INTERFERENCE_REJECTION, val, state);
	break;
      case RC_SCAN_SPEED:
	control.SetControlValue(CT_SCAN_SPEED, val, state);
	break;
      case RC_TIMED_IDLE:
	control.SetControlValue(CT_TIMED_IDLE, val, state);
	break;
      case RC_TIMED_RUN:
	  control.SetControlValue(CT_TIMED_RUN, val, state);
	  break;
      case RC_IMG:
	write_radar_image(val);
	break;
      default:	
	break;
      }
    }
  return receive.Loop();
}



void f_radar::write_radar_image(int val)
{
  // B-scope (x: range, y: azimuth)
  Mat img(GARMIN_XHD_SPOKES, GARMIN_XHD_MAX_SPOKE_LEN, CV_8UC1);  
  radar_image->get_spoke_data(img.data);
    
  if(val == 0){
    char buf[64];
    long long t = get_time();
    snprintf(buf, 64, "radar_%lld.png", t);
    imwrite(buf, img);
  }

  // PPI (North Up)
  if(val == 1){
    Mat ppi=Mat::zeros(GARMIN_XHD_MAX_SPOKE_LEN*2,
		       GARMIN_XHD_MAX_SPOKE_LEN*2,CV_8UC1);
    double spoke_angle = (double)(2 * PI) / (double)GARMIN_XHD_SPOKES;
    for (int r = 0; r < img.rows; r++){
      unsigned char * ptr = ppi.ptr<unsigned char>(r);
      for (int c = 0; c < img.cols; c++){
	int x = r - GARMIN_XHD_MAX_SPOKE_LEN;
	int y = GARMIN_XHD_MAX_SPOKE_LEN - c;
	int ispoke = (int)((double)GARMIN_XHD_SPOKES * atan2(x, y) * ( 0.5 / PI));
	ispoke = (ispoke + GARMIN_XHD_SPOKES * 2) % GARMIN_XHD_SPOKES;
	
	int d = (int)(0.5 + sqrt((double)(x * x + y * y)));
	unsigned char val;
	if(d >= GARMIN_XHD_MAX_SPOKE_LEN || d < 0)
	  val = 255;
	else
	  val = *(img.data + ispoke * GARMIN_XHD_MAX_SPOKE_LEN + d);
	*(ptr + c) = val;	
      }
    }
    char buf[64];
    long long t = get_time();
    snprintf(buf, 64, "radar_%lld.png", t);
    
    imwrite(buf, ppi);     
  }
}
