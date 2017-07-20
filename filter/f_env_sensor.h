// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_env_sensor.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_env_sensor.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_env_sensor.h.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef _F_BASE_H_
#define _F_BASE_H_

#include "f_base.h"
#include "ch_state"

#define MLB_BUF 1024

class f_env_sensor: public f_base
{
 protected:
  ch_env * m_chan;
  char m_dname[1024];
  unsigned short m_port;
  unsigned int m_br;
  char m_rbuf[MLB_BUF];
  AWS_SERIAL m_hserial;

  int m_rbuf_head, m_rbuf_tail;
  bool m_verb;
 public:
  f_env_sensor(const char * name);
  virtual ~f_env_sensor();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();

};
