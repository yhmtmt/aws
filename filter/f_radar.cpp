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
  "scan_speed", "timed_idle", "timed_run"
};
