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
#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

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
  f_base(name), m_fd(-1), m_verb(false),  m_aws_ctrl(false), m_t_rmc_avg(10),
  m_meng_max_rmc(0x81), m_meng_nuf_rmc(0x80),  m_meng_nut_rmc(0x7f),  
  m_meng_nub_rmc(0x7e), m_meng_min_rmc(0x7d),  
  m_seng_max_rmc(0x81),  m_seng_nuf_rmc(0x80), m_seng_nut_rmc(0x7f),
  m_seng_nub_rmc(0x7e),  m_seng_min_rmc(0x7d),
  m_rud_max_rmc(0x80),  m_rud_nut_rmc(0x7f),  m_rud_min_rmc(0x7e),
  m_rud_sta_max(0xff), m_rud_sta_nut(0x7f), m_rud_sta_min(0x00),
  m_meng(0x7f),  m_seng(0x7f),  m_rud(0x7f),  
  m_meng_max(0x81),m_meng_nuf(0x80),  m_meng_nut(0x7f),  
  m_meng_nub(0x7e), m_meng_min(0x7d),  
  m_seng_max(0x81),  m_seng_nuf(0x80), m_seng_nut(0x7f),
  m_seng_nub(0x7e),  m_seng_min(0x7d),
  m_rud_max(0x80),  m_rud_nut(0x7f),  m_rud_min(0x7e),
  m_rud_sta_out_max(0xff), m_rud_sta_out_nut(0x7f), m_rud_sta_out_min(0x00)
{
  strcpy(m_dev, "/dev/zgpio1");
  register_fpar("device", m_dev, 1023, "AWS1's control gpio device path");
  register_fpar("verb", &m_verb, "For debug.");
  register_fpar("ctrl", &m_aws_ctrl, "Yes if aws controls AWS1 (default no)");
  register_fpar("tavg", &m_t_rmc_avg, "Averaging time window. (default 10)");
  register_fpar("awsrud", &m_rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_seng_aws, "Control value of AWS1's sub engine.");
  register_fpar("rmcrud", &m_rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_rud_sta, "Rudder Status of AWS1's.");

  register_fpar("meng_max_rmc", &m_meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  register_fpar("meng_max", &m_meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_meng_min, "Minimum control value for AWS1's main engine.");

  register_fpar("seng_max_rmc", &m_seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  register_fpar("seng_max", &m_seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_seng_min, "Minimum control value for AWS1's sub engine.");

  register_fpar("rud_max_rmc", &m_rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  register_fpar("rud_max", &m_rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_rud_min, "Minimum control value for AWS1's rudder.");

  register_fpar("rud_sta_max", &m_rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  register_fpar("rud_sta_out_max", &m_rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

  register_fpar("meng", &m_meng, "Output value for main engine.");
  register_fpar("seng", &m_meng, "Output value for sub engine.");
  register_fpar("rud", &m_rud, "Output value for rudder.");
  register_fpar("rud_sta_out", &m_rud_sta_out, "Output value for rudder status.");
}

f_aws1_ctrl::~f_aws1_ctrl()
{
}

bool f_aws1_ctrl::init_run()
{
  m_fd = open(m_dev, O_RDWR);
  if(m_fd == -1){
    cerr << "Error in f_aws1_ctrl::init_run, opening device " << m_dev << "." << endl; 
    cerr << "    Message: " << strerror(errno) << endl;
    return false;
  }

  m_rud_rmc_sum = m_meng_rmc_sum = m_seng_rmc_sum = m_rud_sta_sum = 0;
  m_t_rmc_cnt = 0;
 
  return true;
}

void f_aws1_ctrl::destroy_run()
{
  if(m_fd != -1)
    close(m_fd);
}

void f_aws1_ctrl::get_gpio()
{
  unsigned int val;

  ioctl(m_fd, ZGPIO_IOCGET, &val);
  m_rud_rmc = ((unsigned char*) &val)[0];
  m_meng_rmc = ((unsigned char*) &val)[1];
  m_seng_rmc = ((unsigned char*) &val)[2];
  m_rud_sta = ((unsigned char*) &val)[3];
 
  if(!m_aws_ctrl){
    m_rud_aws = map_oval(m_rud_rmc, 
			 m_rud_max_rmc, m_rud_nut_rmc, m_rud_min_rmc,
			 0xff, 0x7f, 0x00);
    m_meng_aws = map_oval(m_meng_rmc,
			  m_meng_max_rmc, m_meng_nuf_rmc, m_meng_nut_rmc, 
			  m_meng_nub_rmc, m_meng_min_rmc,
			  0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
    m_seng_aws = map_oval(m_seng_rmc,
			  m_seng_max_rmc, m_seng_nuf_rmc, m_seng_nut_rmc, 
			  m_seng_nub_rmc, m_seng_min_rmc,
			  0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
  }
}

// In this method, aws's control values are assumed to be 0-255.
// Then the values are mapped to actual control values.
// 
// Rudder
// 0-127: port (min to nutral)
// 127-255: starboard (nutral to max)
//
// Engine 
// 0-102: Astern (min to nutral backword)
// 102-127: Deadslow Astern (nutral backword to nutral)
// 127-132: Deadslow Ahead (nutral to nutral forward)
// 132-255: Ahead (nutral forward to max)

void f_aws1_ctrl::set_gpio()
{
  unsigned int val;

  if(m_aws_ctrl){
    m_rud = map_oval(m_rud_aws, 
		     0xff, 0x7f, 0x00, 
		     m_rud_max, m_rud_nut, m_rud_min);
    m_meng = map_oval(m_meng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_meng_max, m_meng_nuf, m_meng_nut, 
		      m_meng_nub, m_meng_min);  
    m_seng = map_oval(m_seng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_seng_max, m_seng_nuf, m_seng_nut, 
		      m_seng_nub, m_seng_min);
  }else{
    m_rud = map_oval(m_rud_rmc, 
		     m_rud_max_rmc, m_rud_nut_rmc, m_rud_min_rmc,
		     m_rud_max, m_rud_nut, m_rud_min);
    m_meng = map_oval(m_meng_rmc, 
		      m_meng_max_rmc, m_meng_nuf_rmc, m_meng_nut_rmc, 
		      m_meng_nub_rmc, m_meng_min_rmc,
		      m_meng_max, m_meng_nuf, m_meng_nut, m_meng_nub, 
		      m_meng_min);  
    m_seng = map_oval(m_seng_rmc, 
		      m_seng_max_rmc, m_seng_nuf_rmc, m_seng_nut_rmc, 
		      m_seng_nub_rmc, m_seng_min_rmc,
		      m_seng_max, m_seng_nuf, m_seng_nut, m_seng_nub, 
		      m_seng_min);
  }
  
  ((unsigned char *) &val)[0] = m_rud;
  ((unsigned char *) &val)[1] = m_meng;
  ((unsigned char *) &val)[2] = m_seng;
  
  m_rud_sta_out = map_oval(m_rud_sta, 
			   m_rud_sta_max, m_rud_sta_nut, m_rud_sta_min,
			   m_rud_sta_out_max, m_rud_sta_out_nut, m_rud_sta_out_min);
  ((unsigned char *) &val)[3] = m_rud_sta_out;

  ioctl(m_fd, ZGPIO_IOCSET2, &val);
}

bool f_aws1_ctrl::proc()
{
  
  get_gpio();

  set_gpio();

  
  if(m_verb){
    
  }
  return true;
}
