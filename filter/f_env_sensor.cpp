// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_env_sensor.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_env_sensor.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_env_sensor.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/aws_serial.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_env_sensor.h"

f_env_sensor::f_env_sensor(const char * name):f_base(name), m_chan(NULL), m_rbuf_head(0), m_rbuf_tail(0), m_port(0), m_verb(false), m_hserial(-1), m_br(9600)
{
  m_dname[0] = '¥0';
  register_fpar("ch", (ch_base**)&m_chan, typeid(ch_env).name(), "Channel for enviromental parameters.");
  register_fpar("dev", &m_dname, 1024, "Device file path of the serial port.");
  register_fpar("port", &m_port, "Port number.");
  register_fpar("br", &m_br, "Baudrate.");
  register_fpar("verb", &m_verb, "Verbose mode.");
}

f_env_sensor::~f_env_sensor()
{
}

bool f_env_sensor::init_run()
{
#ifdef _WIN32
  m_hserial = open_serial(m_port, m_br);
#else
  m_hserial = open_serial(m_dname, m_br);
#endif
  if(m_hserial == NULL_SERIAL)
    return false;
  return true;
}

void f_env_sensor::destroy_run()
{
}

bool f_env_sensor::proc()
{
  return true;
}
