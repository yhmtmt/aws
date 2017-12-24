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

f_env_sensor::f_env_sensor(const char * name) :f_base(name), m_chan(NULL), m_rbuf_head(0), m_rbuf_tail(0), m_port(0), m_verb(false), m_hserial(NULL_SERIAL), m_br(9600)
{
  m_dname[0] = 0;
  register_fpar("ch", (ch_base**)&m_chan, typeid(ch_env).name(), "Channel for enviromental parameters.");
  register_fpar("dev", m_dname, 1024, "Device file path of the serial port.");
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
  if (m_hserial == NULL_SERIAL)
    return false;
  return true;
}

void f_env_sensor::destroy_run()
{
}

bool f_env_sensor::proc()
{
  m_rbuf_tail +=
    read_serial(m_hserial, m_rbuf + m_rbuf_tail, MLB_BUF - m_rbuf_tail);
  int dat_start = -1, dat_end = -1;
  for (int i = m_rbuf_head; i < m_rbuf_tail; i++){
    if (m_rbuf[i] == 'm' && m_rbuf[i + 1] == 'l' && m_rbuf[i + 2] == 'b'){     
      dat_start = i;
      break;
    }
  }

  if (dat_start < 0){
    return true;
  }
  else{
    for (int i = dat_start; i < m_rbuf_tail; i++){
      if (m_rbuf[i] == 0x0d || m_rbuf[i] == 0x0a){
        m_rbuf[i] = 0;
        dat_end = i;
        break;
      }
    }
  }

  double baro, temp, humd, ilum;

  if (dat_start >= 0 && dat_end >= 0){
    int i = 3 + dat_start;
    int j = i;
    int ipar = 0;
    for (; j <= dat_end; j++){
      if (m_rbuf[j] == ',' || m_rbuf[j] == 0){
        m_rbuf[j] = 0;
        switch (ipar){
        case 0:
          temp = atof(&m_rbuf[i]);
          break;
        case 1:
          baro = atof(&m_rbuf[i]);
          break;
        case 2:
          humd = atof(&m_rbuf[i]);
          break;
        case 3:
          ilum = atof(&m_rbuf[i]);
          break;
        }
        ipar++;
        j++;
        i = j;
      }
    }
    if(m_verb){
      cout << m_time_str;
      cout << " baro=" << baro << " temp=" << temp << " humd=" << humd << " ilum=" << ilum << endl;
    }
    m_chan->set(get_time(), (float)baro, (float)temp, (float)humd, (float)ilum);
    
    // repacking buffer
    for (i = 0, j = dat_end + 1; j < m_rbuf_tail; i++, j++){
      m_rbuf[i] = m_rbuf[j];
    }
    m_rbuf_head = 0;
    m_rbuf_tail = i;
  }

  return true;
}


f_volt_sensor::f_volt_sensor(const char * name) :f_base(name), m_chan(NULL), m_rbuf_head(0), m_rbuf_tail(0), m_port(0), m_verb(false), m_hserial(NULL_SERIAL), m_br(9600)
{
  m_dname[0] = 0;
  register_fpar("ch", (ch_base**)&m_chan, typeid(ch_volt).name(), "Channel for voltage parameters.");
  register_fpar("dev", m_dname, 1024, "Device file path of the serial port.");
  register_fpar("port", &m_port, "Port number.");
  register_fpar("br", &m_br, "Baudrate.");
  register_fpar("verb", &m_verb, "Verbose mode.");
}

f_volt_sensor::~f_volt_sensor()
{
}


bool f_volt_sensor::init_run()
{
#ifdef _WIN32
  m_hserial = open_serial(m_port, m_br);
#else
  m_hserial = open_serial(m_dname, m_br);
#endif
  if (m_hserial == NULL_SERIAL)
    return false;
  return true;
}

void f_volt_sensor::destroy_run()
{
}

bool f_volt_sensor::proc()
{
  m_rbuf_tail +=
    read_serial(m_hserial, m_rbuf + m_rbuf_tail, MLB_BUF - m_rbuf_tail);
  int dat_start = -1, dat_end = -1;
  for (int i = m_rbuf_head; i < m_rbuf_tail; i++){
    if (m_rbuf[i] == 'v' && m_rbuf[i + 1] == 's' 
	&& m_rbuf[i + 2] == 'e' && m_rbuf[i + 3] == 'n'){
      dat_start = i;
      break;
    }
  }

  if (dat_start < 0){
    return true;
  }
  else{
    for (int i = dat_start; i < m_rbuf_tail; i++){
      if (m_rbuf[i] == 0x0d || m_rbuf[i] == 0x0a){
        m_rbuf[i] = 0;
        dat_end = i;
        break;
      }
    }
  }

  if (dat_start >= 0 && dat_end >= 0){
    int i = 4 + dat_start;
    int j = i;
    int ipar = 0;
    for (; j <= dat_end; j++){
      if (m_rbuf[j] == ',' || m_rbuf[j] == 0){
        m_rbuf[j] = 0;
        m_val[ipar] = (float)(atoi(&m_rbuf[i]));
        ipar++;
        j++;
        i = j;
      }
    }
    if (m_verb){
      cout << m_time_str;
      for (int iprobe = 0; iprobe < 8; iprobe++){
        cout << " v[" << iprobe << "]:" << m_val[iprobe];
      }
      cout << endl;
    }
    m_chan->set(get_time(), m_val);

    // repacking buffer
    for (i = 0, j = dat_end + 1; j < m_rbuf_tail; i++, j++){
      m_rbuf[i] = m_rbuf[j];
    }
    m_rbuf_head = 0;
    m_rbuf_tail = i;
  }

  return true;
}

