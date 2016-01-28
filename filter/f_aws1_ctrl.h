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

  unsigned char m_rud_aws;
  unsigned char m_meng_aws;
  unsigned char m_seng_aws;

  unsigned char m_meng_rmc; // main engine control value from remote control
  unsigned char m_seng_rmc; // sub engine control value from remote control
  unsigned char m_rud_rmc;  // rudder control value from remote control

  unsigned char m_rud_sta;  // rudder status value.

  // remote controller's values corresponding positions 
  unsigned char m_meng_max_rmc; 
  unsigned char m_meng_nuf_rmc;
  unsigned char m_meng_nut_rmc;
  unsigned char m_meng_nub_rmc;
  unsigned char m_meng_min_rmc;

  unsigned char m_seng_max_rmc;
  unsigned char m_seng_nuf_rmc;
  unsigned char m_seng_nut_rmc;
  unsigned char m_seng_nub_rmc;
  unsigned char m_seng_min_rmc;

  unsigned char m_rud_max_rmc;
  unsigned char m_rud_nut_rmc;
  unsigned char m_rud_min_rmc;

  unsigned char m_rud_sta_max;
  unsigned char m_rud_sta_nut;
  unsigned char m_rud_sta_min;
  
  unsigned char m_meng;     // main engine control value
  unsigned char m_seng;     // sub engine control value
  unsigned char m_rud;      // rudder control value
  unsigned char m_rud_sta_out; // rudder status output

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

  unsigned char m_rud_sta_out_max;
  unsigned char m_rud_sta_out_nut;
  unsigned char m_rud_sta_out_min;

  int map_oval(int val, 
	      int vmax, int vnut, int vmin, 
	      int omax, int onut, int omin)
  {
    int dvmax = val - vmax;
    int dvnut = val - vnut;
    int dvmin = val - vmin;
    int dvmax_vnut = vmax - vnut;
    int dvnut_vmin = vnut - vmin;

    if(abs(dvmax) < abs(dvmax_vnut) && abs(dvfnut) < abs(dvmax_vnut))
      return (int) ((double)((omax - onut) * (dvnut)) / (double) (dvmax_vnut)) + onut;
    else if(abs(dvnut) < abs(dvnut_vmin) && abs(dvmin) < abs(dvnut_vmin))
      return (int) ((double)((ofnut - onut) * (dvnut)) / (double) (dvfnut_vnut)) + onut;
    else if(abs(dvmax) < abs(dvmin))
      return omax;
    else
      return omin;
  }

  int map_oval(int val, 
	       int vmax, int vfnut, 
	       int vnut, int vbnut, 
	       int vmin,
	       int omax, int ofnut, 
	       int onut, int obnut, 
	       int omin
	       )
  {
    int dvmax = val - vmax;
    int dvfnut = val - vfnut;
    int dvnut = val - vnut;
    int dvbnut = val - vbnut;
    int dvmin = val - vmin;
    int dvmax_vfnut = vmax - vfnut;
    int dvfnut_vnut = vfnut - vnut;
    int dvnut_vbnut = vnut - vbnut;
    int dvbnut_vmin = vbnut - vmin;

    if(abs(dvmax) < abs(dvmax_vfnut) && abs(dvfnut) < abs(dvmax_vfnut))
      return (int) ((double)((omax - ofnut) * (dvfnut)) / (double) (dvmax_vfnut)) + ofnut;
    else if(abs(dvnut) < abs(dvfnut_vnut) && abs(dvfnut) < abs(dvfnut_vnut))
      return (int) ((double)((ofnut - onut) * (dvnut)) / (double) (dvfnut_vnut)) + onut;
    else if(abs(dvnut) < abs(dvnut_dvbnut) && abs(dvbnut) < abs(dvnut_vbnut))
      return (int) ((double)((onut - obnut) * (dvbnut)) / (double) (dvnut_vbnut)) + obnut;
    else if(abs(dvbnut) < abs(dvbnut_vmin) && abs(dvmin) < abs(dvbnut_vmin))
      return (int) ((double)((obnut - omin) * dvbmin) / (double) (dvbnut_vmin)) + omin;
    else if(abs(dvmax) < abs(dvmin))
      return omax;
    else
      return omin;
  }

  void get_gpio();
  void set_gpio();
public:
	f_aws1_ctrl(const char * name);

	virtual ~f_aws1_ctrl();

	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

#endif
