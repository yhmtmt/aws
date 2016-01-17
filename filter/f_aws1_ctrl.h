// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_aws1_ctrl.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ctrl.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ctrl.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_CTRL_H_
#define _F_AWS1_CTRL_H_

#include "f_base.h"

class f_aws1_ctrl: public f_base
{
protected:
  char m_dev[1024];         // device path, e.g. "/dev/zgpio0"
  int m_fd;                 // file descriptor for zgpio

  bool m_aws_ctrl;          // remote control flag. 
                            //    true: control values from aws are sat. 
                            //    false: control values from remote controller are sat.

  unsigned char m_meng_rmt; // main engine control value from remote control
  unsigned char m_seng_rmt; // sub engine control value from remote control
  unsigned char m_rud_rmt;  // rudder control value from remote control

  unsigned char m_rud_sta;  // rudder status value.
  // remote controler's values corresponding positions 
  unsigned char m_meng_max_rmt; 
  unsigned char m_meng_nuf_rmt;
  unsigned char m_meng_nut_rmt;
  unsigned char m_meng_nub_rmt;
  unsigned char m_meng_min_rmt;

  unsigned char m_seng_max_rmt;
  unsigned char m_seng_nuf_rmt;
  unsigned char m_seng_nut_rmt;
  unsigned char m_seng_nub_rmt;
  unsigned char m_seng_min_rmt;

  unsigned char m_rud_max_rmt;
  unsigned char m_rud_nut_rmt;
  unsigned char m_rud_min_rmt;
  
  unsigned char m_meng;     // main engine control value
  unsigned char m_seng;     // sub engine control value
  unsigned char m_rud;      // rudder control value

  // digital potentiometer's values corressponding positions
  unsigned char m_meng_max;
  unsigned char m_meng_nuf;
  unsigned char m_meng_nut;
  unsigned char m_meng_nub;
  unsigned char m_meng_min;

  unsigned char m_seng_max;
  unsigned char m_seng_nuf;
  unsigned char m_seng_nut;
  unsigned char m_seng_nub;
  unsigned char m_seng_min;

  unsigned char m_rud_max;
  unsigned char m_rud_nut;
  unsigned char m_rud_min;

public:
	f_aws1_ctrl(const char * name);

	virtual ~f_aws1_ctrl();

	virtual bool init_run();

	virtual void destroy_run();


	virtual bool proc();
};

#endif
