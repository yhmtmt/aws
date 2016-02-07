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

const char * f_aws1_ctrl:: m_str_adclpf_type[ADCLPF_NONE] = {
  "avg", "gauss"
};

const char * str_aws1_ctrl_src[ACS_NONE] = {
  "fset", "udp", "chan"
};

f_aws1_ctrl::f_aws1_ctrl(const char * name): 
  f_base(name), m_fd(-1), m_verb(false),  m_aws_ctrl(false),
  m_aws1_ctrl_src(ACS_FSET), m_acs_sock(-1), m_acs_port(20100), 
  m_pacs_in(NULL), m_pacs_out(NULL),
  m_adclpf(false), m_sz_adclpf(5), m_cur_adcsmpl(0), m_sigma_adclpf(3.0),
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
  m_flog_name[0] = 0;

  register_fpar("device", m_dev, 1023, "AWS1's control gpio device path");
  register_fpar("flog", m_flog_name, 1023, "Control log file.");
  register_fpar("verb", &m_verb, "For debug.");
  register_fpar("ctrl", &m_aws_ctrl, "Yes if aws controls AWS1 (default no)");

  register_fpar("acs", (int*) &m_aws1_ctrl_src, ACS_NONE, str_aws1_ctrl_src,  "AWS control source.");
  register_fpar("acsport", &m_acs_port, "Port number for AWS1 UDP control.");
  // LPF related parameters
  register_fpar("adclpf", &m_adclpf, "LPF is applied for the ADC inputs.");
  register_fpar("sz_adclpf", &m_sz_adclpf, "Window size of the ADC-LPF.");
  register_fpar("type_adclpf", (int*) &m_type_adclpf, ADCLPF_NONE, m_str_adclpf_type, "Type of ADCLPF.");
  register_fpar("sigma_adclpf", &m_sigma_adclpf, "Standard deviation of the gaussian kernel of the ADC-LPF (This can only be used in the case of the filter type is gauss)");

  // aws's control parameters
  register_fpar("awsrud", &m_rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_seng_aws, "Control value of AWS1's sub engine.");

  // remote controller's control parameters (Read Only)
  register_fpar("rmcrud", &m_rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_rud_sta, "Rudder Status of AWS1's.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  // Each control points of the sub engine output
  register_fpar("seng_max", &m_seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_seng_min, "Minimum control value for AWS1's sub engine.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  // Control points as the rudder indicator output.
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

  if(m_flog_name[0] != 0){
    m_flog.open(m_flog_name);
    if(!m_flog.is_open()){
      cerr << m_name <<  " failed to open log file." << endl;
      return false;
    }
  }

  if(m_chin.size() > 0){
    m_pacs_in = dynamic_cast<ch_ring<char>*>(m_chin[0]);
    if(m_pacs_in == NULL){
      cerr << "The first input channel should be ch_ring<char>." << endl;
      return false;
    }
  }

  if(m_chout.size() > 0){
    m_pacs_out = dynamic_cast<ch_ring<char>*>(m_chout[0]);
    if(m_pacs_out == NULL){
      cerr << "The first output channel should be ch_ring<char>." << endl;
      return false;
    }
  }

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
  if(m_aws_ctrl){
    switch(m_aws1_ctrl_src){
    case ACS_UDP:
      proc_acs_udp();
      break;
    case ACS_CHAN:
      proc_acs_chan();
      break;
    case ACS_FSET:
    default:
      // nothing to do
      break;
    }
  }

  get_gpio();

  if(m_adclpf)
    lpf();

  set_gpio();

  
  if(m_verb){
    cout << "Control Values." << endl;
    cout << "    rmc rud " << (int) m_rud_rmc << " meng " << (int) m_meng_rmc << " seng " << (int) m_seng_rmc << endl;
    cout << "    aws rud " << (int) m_rud_aws << " meng " << (int) m_meng_aws << " seng " << (int) m_seng_aws << endl;
    cout << "    out rud " << (int) m_rud << " meng " << (int) m_meng << " seng " << (int) m_seng << endl;
    cout << "    rud stat in " << (int) m_rud_sta << " out " << (int) m_rud_sta_out << endl;

  }
  if(m_flog.is_open()){
    m_flog << m_cur_time << " ";
    m_flog << (int) m_rud_rmc << " " << (int) m_meng_rmc << " " << (int) m_seng_rmc << " ";
    m_flog << (int) m_rud_aws << " " << (int) m_meng_aws << " " << (int) m_seng_aws << " ";
    m_flog << (int) m_rud << " " << (int) m_meng << " " << (int) m_seng << " ";
    m_flog << (int) m_rud_sta << " " << (int) m_rud_sta_out << endl;
  }
  return true;
}

void f_aws1_ctrl::lpf()
{
  if(m_sz_adclpf != m_kern_adclpf.size()){ // initialize filter
    if(m_verb)
    // allocating memory
    m_kern_adclpf.resize(m_sz_adclpf);
    m_rud_smpl.resize(m_sz_adclpf, (int) m_rud_rmc);
    m_meng_smpl.resize(m_sz_adclpf, (int) m_meng_rmc);
    m_seng_smpl.resize(m_sz_adclpf, (int) m_seng_rmc);
    //m_rud_sta_smpl.resize(m_sz_adclpf, (int) m_rud_sta);

    // building filter kernel.
    switch(m_type_adclpf){
    case ADCLPF_GAUSS:
      {
	double c = (double)(m_sz_adclpf - 1) / 2.0;
	double sum = 0.;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] = (float) gauss(c, m_sigma_adclpf, (double) i);
	  sum += m_kern_adclpf[i];
	}

	// normalize the filter.
	sum = 1.0 / sum;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] *= sum;
	}
      }
      break;
    case ADCLPF_AVG:
      {
	double val = 1.0 / (double) m_sz_adclpf;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] = (float) val;
	}
      }
      break;
    default:
      break;
    }

    m_cur_adcsmpl = 0;
    if(m_verb)
      cout << " done." << endl;
  }

  m_rud_smpl[m_cur_adcsmpl] = m_rud_rmc;
  m_meng_smpl[m_cur_adcsmpl] = m_meng_rmc;
  m_seng_smpl[m_cur_adcsmpl] = m_seng_rmc;
  //m_rud_sta_smpl[m_cur_adcsmpl] = m_rud_sta;
  

  // kernel convolution
  double rud = 0., meng = 0., seng = 0., rud_sta = 0.;
  
  for(int i = 0, j = m_cur_adcsmpl; i < m_sz_adclpf; i++, j = (j + 1) % m_sz_adclpf ){
    rud += m_rud_smpl[j] * m_kern_adclpf[i];
    meng += m_meng_smpl[j] * m_kern_adclpf[i];
    seng += m_seng_smpl[j] * m_kern_adclpf[i];
    //rud_sta += m_rud_sta_smpl[j] * m_kern_adclpf[i];
  }
  
  m_rud_rmc = rud;
  m_meng_rmc = meng;
  m_seng_rmc = seng;
  // m_rud_sta = rud_sta;

  m_cur_adcsmpl = (m_cur_adcsmpl > 0 ? m_cur_adcsmpl - 1 : m_sz_adclpf - 1);
}

void f_aws1_ctrl::proc_acs_udp()
{
  if(m_acs_sock == -1){
    m_acs_sock_addr.sin_family = AF_INET;
    m_acs_sock_addr.sin_port = htons(m_acs_port);
    set_sockaddr_addr(m_acs_sock_addr);
    if(::bind(m_acs_sock, (sockaddr*) &m_acs_sock_addr, sizeof(m_acs_sock_addr)) == SOCKET_ERROR){
      cerr << "Socket error during binding UDP socket in " << m_name;
      cerr << ". AWS Control Source is automatically set to ACS_FSET." << endl;
      m_aws1_ctrl_src = ACS_FSET;
      m_acs_sock = -1;
    }
  }else{
    sockaddr_in addr;
    socklen_t sz_addr = sizeof(addr);
    int res;
    fd_set fr, fw, fe;
    timeval tv;
    s_aws1_ctrl_pkt acspkt;
    
    // recieving packet
    FD_ZERO(&fr);
    FD_ZERO(&fe);
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    
    res = select((int) m_acs_sock + 1, &fr, NULL, &fe, &tv);
    if(res > 0){
      if(FD_ISSET(m_acs_sock, &fr)){
	int len = recvfrom(m_acs_sock, (char*) &acspkt, sizeof(acspkt), 0, (sockaddr*)&addr, &sz_addr);
	if(len == SOCKET_ERROR){
	  cerr << "Socket error during recieving packet in " << m_name;
	  cerr << " . Now closing socket." << endl;
	  closesocket(m_acs_sock);
	  m_acs_sock = -1;
	  m_aws1_ctrl_src = ACS_FSET;
	}
	if(len == sizeof(acspkt)){
	  m_rud_aws = acspkt.rud;
	  m_meng_aws = acspkt.meng;
	  m_seng_aws = acspkt.seng; 
	}
      }else if(FD_ISSET(m_acs_sock, &fe)){
	cerr << "Socket error during recieving packet in " << m_name;
	cerr << " . Now closing socket." << endl;
	closesocket(m_acs_sock);
	m_acs_sock = -1;
	m_aws1_ctrl_src = ACS_FSET;
      }
    }else if(res == -1){
      int en = errno;
      cerr << "Error no " << en << " " << strerror(en) << endl;
    }else{
      cerr << "Unkonwo error in " << m_name << "." << endl;
    }
    
    // sending packet
    acspkt.tcur = m_cur_time;
    acspkt.rud = m_rud;
    acspkt.meng = m_meng;
    acspkt.seng = m_seng;
    acspkt.rud_rmc = m_rud_rmc;
    acspkt.meng_rmc = m_meng_rmc;
    acspkt.seng_rmc = m_seng_rmc;
    acspkt.rud_sta = m_rud_sta;
    acspkt.rud_sta_out = m_rud_sta_out;
    int len = sendto(m_acs_sock, (char*) &acspkt, sizeof(acspkt), 0, (sockaddr*)&addr, sz_addr);
  }
}

void f_aws1_ctrl::proc_acs_chan()
{
  s_aws1_ctrl_pkt pkt;
  int len;
  if(m_pacs_in == NULL){
    cerr << m_name  << " does not have control input channel." << endl;
  }else{
    len = m_pacs_in->read((char*) &pkt, sizeof(pkt));
    if(len == sizeof(pkt)){
      m_rud_aws = pkt.rud;
      m_meng_aws = pkt.meng;
      m_seng_aws = pkt.seng;
    }
  }

  pkt.tcur = m_cur_time;
  pkt.rud = m_rud;
  pkt.meng = m_meng;
  pkt.seng = m_seng;
  pkt.rud_rmc = m_rud_rmc;
  pkt.meng_rmc = m_meng_rmc;
  pkt.seng_rmc = m_seng_rmc;
  pkt.rud_sta = m_rud_sta;
  pkt.rud_sta_out = m_rud_sta_out;

  if(m_pacs_out == NULL){
    cerr << m_name << " does not have control output channel." << endl;
  }else{
    len = m_pacs_out->write((char*) &pkt, sizeof(pkt));
  }
}
