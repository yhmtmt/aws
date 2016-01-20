// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_aws1_ctrl.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ctrl.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ctrl.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
#include <cstdio>
#include <cstring>

using namespace std;
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

#include  "../driver/zgpio.h"
#include "f_aws1_ctrl.h"

f_aws1_ctrl::f_aws1_ctrl(const char * name): 
  f_base(name), m_fd(NULL), m_aws_ctrl(false), m_meng_max_rmt(0x7f),
  m_meng_nuf_rmt(0x7f),  m_meng_nut_rmt(0x7f),  m_meng_nub_rmt(0x7f),
  m_meng_min_rmt(0x7f),  m_seng_max_rmt(0x7f),  m_seng_nuf_rmt(0x7f),
  m_seng_nut_rmt(0x7f),  m_seng_nub_rmt(0x7f),  m_seng_min_rmt(0x7f),
  m_rud_max_rmt(0x7f),  m_rud_nut_rmt(0x7f),  m_rud_min_rmt(0x7f),
  m_meng(0x7f),  m_seng(0x7f),  m_rud(0x7f),  m_meng_max(0x7f),
  m_meng_nuf(0x7f),  m_meng_nut(0x7f),  m_meng_nub(0x7f),
  m_meng_min(0x7f),  m_seng_max(0x7f),  m_seng_nuf(0x7f),
  m_seng_nut(0x7f),  m_seng_nub(0x7f),  m_seng_min(0x7f),
  m_rud_max(0x7f),  m_rud_nut(0x7f),  m_rud_min(0x7f)					     
{
  strcpy(m_dev, "/dev/zgpio0");
  register_fpar("device", m_dev, 1023, "AWS1's control gpio device path");
  register_fpar("awsctrl", &m_aws_ctrl, "Yes if aws controls AWS1 (default no)");
}

f_aws1_ctrl::~f_aws1_ctrl()
{
}

bool f_aws1_ctrl::init_run()
{
  fd = open(m_dev, O_RDWR);
  if(fd == -1){
    cerr << "Error in f_aws1_ctrl::init_run, opening device " << m_dev << "." << endl; 
    cerr << "    Message: " << strerr(errno) << endl;
    fd = NULL;
    return false;
  }

 
  return true;
}

void f_aws1_ctrl::destroy_run()
{
  close(m_fd);
}

void f_aws1_ctrl::get_gpio()
{
  unsigned int val;
  ioctl(fd, ZGPIO_IOGET, &val);
  m_rud_rmt = ((unsigned char*) &val)[0];
  m_meng_rmt = ((unsigned char*) &val)[1];
  m_seng_rmt = ((unsigned char*) &val)[2];
  m_rud_sta= ((unsigned char*) &val)[3];
}

void f_aws1_ctrl::set_gpio()
{
  unsigned int val;

  if(m_aws_ctrl){
    m_rud = map_oval(m_rud_aws, 
		     0xff, 0x7f, 0x00, 
		     m_rud_max, m_rud_nut, m_rud_min);
    m_meng = map_oval(m_meng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_meng_max, m_meng_nuf, m_meng_nut, m_meng_nub, m_meng_min);  
    m_seng = map_oval(m_seng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_seng_max, m_seng_nuf, m_seng_nut, m_seng_nub, m_seng_min);
  }else{
    m_rud = map_oval(m_rud_rmt, 
		     m_rud_max_rmt, m_rud_nut_rmt, m_rud_min_rmt,
		     m_rud_max, m_rud_nut, m_rud_min);
    m_meng = map_oval(m_meng_rmt, 
		      m_meng_max_rmt, m_meng_nuf_rmt, m_meng_nut_rmt, m_meng_nub_rmt, m_meng_min_rmt,
		      m_meng_max, m_meng_nuf, m_meng_nut, m_meng_nub, m_meng_min);  
    m_seng = map_oval(m_seng_rmt, 
		      m_seng_max_rmt, m_seng_nuf_rmt, m_seng_nut_rmt, m_seng_nub_rmt, m_meng_min_rmt,
		      m_seng_max, m_seng_nuf, m_seng_nut, m_seng_nub, m_seng_min);    
  }
  
  ((unsigned char *) &val)[0] = m_rud;
  ((unsigned char *) &val)[1] = m_meng;
  ((unsigned char *) &val)[2] = m_seng;
  
  m_rud_sta_out = map_oval(m_rud_sta, 
			   m_rud_sta_max, m_rud_sta_nut, m_rud_sta_min,
			   m_rud_sta_out_max, m_rud_sta_out_nut, m_rud_sta_out_min);
  ((unsigned char *) &val)[3] = m_rud_sta_out;
}

bool f_aws1_ctrl::proc()
{
  
  return true;
}
