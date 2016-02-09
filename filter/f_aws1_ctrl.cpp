// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

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
  m_udp_ctrl(false), m_ch_ctrl(false),
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
  register_fpar("ctrl", &m_acp.ctrl, "Yes if aws controls AWS1 (default no)");

  register_fpar("acs", (int*) &m_acp.ctrl_src, ACS_NONE, str_aws1_ctrl_src,  "AWS control source.");
  register_fpar("udp", &m_udp_ctrl, "Yes if udp control is used.");
  register_fpar("ch", &m_ch_ctrl, "Yes if ch control is used.");
  register_fpar("acsport", &m_acs_port, "Port number for AWS1 UDP control.");
  // LPF related parameters
  register_fpar("adclpf", &m_adclpf, "LPF is applied for the ADC inputs.");
  register_fpar("sz_adclpf", &m_sz_adclpf, "Window size of the ADC-LPF.");
  register_fpar("type_adclpf", (int*) &m_type_adclpf, ADCLPF_NONE, m_str_adclpf_type, "Type of ADCLPF.");
  register_fpar("sigma_adclpf", &m_sigma_adclpf, "Standard deviation of the gaussian kernel of the ADC-LPF (This can only be used in the case of the filter type is gauss)");

  // aws's control parameters
  register_fpar("awsrud", &m_acp.rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_acp.meng_aws, "Control value of AWS1's main engine.");
  register_fpar("awsseng", &m_acp.seng_aws, "Control value of AWS1's sub engine.");

  // remote controller's control parameters (Read Only)
  register_fpar("rmcrud", &m_acp.rud_rmc, "Control value of AWS1's rudder controller.");
  register_fpar("rmcmeng", &m_acp.meng_rmc, "Control value of AWS1's main engine controller.");
  register_fpar("rmcseng", &m_acp.seng_rmc, "Control value of AWS1's sub engine controller.");
  register_fpar("rud_sta", &m_acp.rud_sta, "Rudder Status of AWS1's.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_acp.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_acp.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_acp.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_acp.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_acp.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_acp.meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_acp.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_acp.meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_acp.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_acp.meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.
  register_fpar("seng_max_rmc", &m_acp.seng_max_rmc, "Maximum control value of AWS1's sub engine controller.");
  register_fpar("seng_nuf_rmc", &m_acp.seng_nuf_rmc, "Nutral to Forward control value of AWS1's sub engine controller.");
  register_fpar("seng_nut_rmc", &m_acp.seng_nut_rmc, "Nutral control value of AWS1's sub engine controller.");
  register_fpar("seng_nub_rmc", &m_acp.seng_nub_rmc, "Nutral to Backward control value of AWS1's sub engine controller.");
  register_fpar("seng_min_rmc", &m_acp.seng_min_rmc, "Minimum control value of AWS1's sub engine controller.");

  // Each control points of the sub engine output
  register_fpar("seng_max", &m_acp.seng_max, "Maximum control value for AWS1's sub engine.");
  register_fpar("seng_nuf", &m_acp.seng_nuf, "Nutral to Forward control value for AWS1's sub engine.");
  register_fpar("seng_nut", &m_acp.seng_nut, "Nutral control value for AWS1's sub engine.");
  register_fpar("seng_nub", &m_acp.seng_nub, "Nutral to Backward control value for AWS1's sub engine.");
  register_fpar("seng_min", &m_acp.seng_min, "Minimum control value for AWS1's sub engine.");

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_acp.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_acp.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_acp.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_acp.rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_acp.rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_acp.rud_min, "Minimum control value for AWS1's rudder.");

  // Rudder indicator's controll points.
  register_fpar("rud_sta_max", &m_acp.rud_sta_max, "Maximum value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_nut", &m_acp.rud_sta_nut, "Nutral value of AWS1's rudder angle indicator.");
  register_fpar("rud_sta_min", &m_acp.rud_sta_min, "Minimum value of AWS1's rudder angle indicator.");

  // Control points as the rudder indicator output.
  register_fpar("rud_sta_out_max", &m_acp.rud_sta_out_max, "Maximum output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_nut", &m_acp.rud_sta_out_nut, "Nutral output value of AWS1's rudder angle to rudder pump.");
  register_fpar("rud_sta_out_min", &m_acp.rud_sta_out_min, "Minimum output value of AWS1's rudder angle to rudder pump.");

  register_fpar("meng", &m_acp.meng, "Output value for main engine.");
  register_fpar("seng", &m_acp.meng, "Output value for sub engine.");
  register_fpar("rud", &m_acp.rud, "Output value for rudder.");
  register_fpar("rud_sta_out", &m_acp.rud_sta_out, "Output value for rudder status.");
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
  m_acp.rud_rmc = ((unsigned char*) &val)[0];
  m_acp.meng_rmc = ((unsigned char*) &val)[1];
  m_acp.seng_rmc = ((unsigned char*) &val)[2];
  m_acp.rud_sta = ((unsigned char*) &val)[3];
 
  if(!m_acp.ctrl){
    m_acp.rud_aws = map_oval(m_acp.rud_rmc, 
			 m_acp.rud_max_rmc, m_acp.rud_nut_rmc, m_acp.rud_min_rmc,
			 0xff, 0x7f, 0x00);
    m_acp.meng_aws = map_oval(m_acp.meng_rmc,
			  m_acp.meng_max_rmc, m_acp.meng_nuf_rmc, m_acp.meng_nut_rmc, 
			  m_acp.meng_nub_rmc, m_acp.meng_min_rmc,
			  0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00);
    m_acp.seng_aws = map_oval(m_acp.seng_rmc,
			  m_acp.seng_max_rmc, m_acp.seng_nuf_rmc, m_acp.seng_nut_rmc, 
			  m_acp.seng_nub_rmc, m_acp.seng_min_rmc,
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

  if(m_acp.ctrl){
    m_acp.rud = map_oval(m_acp.rud_aws, 
		     0xff, 0x7f, 0x00, 
		     m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min);
    m_acp.meng = map_oval(m_acp.meng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, 
		      m_acp.meng_nub, m_acp.meng_min);  
    m_acp.seng = map_oval(m_acp.seng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, 
		      m_acp.seng_nub, m_acp.seng_min);
  }else{
    m_acp.rud = map_oval(m_acp.rud_rmc, 
		     m_acp.rud_max_rmc, m_acp.rud_nut_rmc, m_acp.rud_min_rmc,
		     m_acp.rud_max, m_acp.rud_nut, m_acp.rud_min);
    m_acp.meng = map_oval(m_acp.meng_rmc, 
		      m_acp.meng_max_rmc, m_acp.meng_nuf_rmc, m_acp.meng_nut_rmc, 
		      m_acp.meng_nub_rmc, m_acp.meng_min_rmc,
		      m_acp.meng_max, m_acp.meng_nuf, m_acp.meng_nut, m_acp.meng_nub, 
		      m_acp.meng_min);  
    m_acp.seng = map_oval(m_acp.seng_rmc, 
		      m_acp.seng_max_rmc, m_acp.seng_nuf_rmc, m_acp.seng_nut_rmc, 
		      m_acp.seng_nub_rmc, m_acp.seng_min_rmc,
		      m_acp.seng_max, m_acp.seng_nuf, m_acp.seng_nut, m_acp.seng_nub, 
		      m_acp.seng_min);
  }
  
  ((unsigned char *) &val)[0] = m_acp.rud;
  ((unsigned char *) &val)[1] = m_acp.meng;
  ((unsigned char *) &val)[2] = m_acp.seng;
  
  m_acp.rud_sta_out = map_oval(m_acp.rud_sta, 
			   m_acp.rud_sta_max, m_acp.rud_sta_nut, m_acp.rud_sta_min,
			   m_acp.rud_sta_out_max, m_acp.rud_sta_out_nut, m_acp.rud_sta_out_min);
  ((unsigned char *) &val)[3] = m_acp.rud_sta_out;

  ioctl(m_fd, ZGPIO_IOCSET2, &val);
}

bool f_aws1_ctrl::proc()
{

  s_aws1_ctrl_pars acpkt_udp, acpkt_chan;

  if(m_udp_ctrl)
    rcv_acs_udp(acpkt_udp);
  if(m_ch_ctrl)
    rcv_acs_chan(acpkt_chan);

  if(m_acp.ctrl){
    switch(m_acp.ctrl_src){
    case ACS_UDP:
      set_ctrl(acpkt_udp);
      break;
    case ACS_CHAN:
      set_ctrl(acpkt_chan);
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
    cout << "    rmc rud " << (int) m_acp.rud_rmc << " meng " << (int) m_acp.meng_rmc << " seng " << (int) m_acp.seng_rmc << endl;
    cout << "    aws rud " << (int) m_acp.rud_aws << " meng " << (int) m_acp.meng_aws << " seng " << (int) m_acp.seng_aws << endl;
    cout << "    out rud " << (int) m_acp.rud << " meng " << (int) m_acp.meng << " seng " << (int) m_acp.seng << endl;
    cout << "    rud stat in " << (int) m_acp.rud_sta << " out " << (int) m_acp.rud_sta_out << endl;

  }
  if(m_flog.is_open()){
    m_flog << m_cur_time << " ";
    m_flog << (int) m_acp.rud_rmc << " " << (int) m_acp.meng_rmc << " " << (int) m_acp.seng_rmc << " ";
    m_flog << (int) m_acp.rud_aws << " " << (int) m_acp.meng_aws << " " << (int) m_acp.seng_aws << " ";
    m_flog << (int) m_acp.rud << " " << (int) m_acp.meng << " " << (int) m_acp.seng << " ";
    m_flog << (int) m_acp.rud_sta << " " << (int) m_acp.rud_sta_out << endl;
  }

  s_aws1_ctrl_pars acpkt;
  set_acpkt(acpkt);

  if(m_udp_ctrl){
    snd_acs_udp(acpkt);
  }

  if(m_ch_ctrl){
    snd_acs_chan(acpkt);
  }
  return true;
}

void f_aws1_ctrl::lpf()
{
  if(m_sz_adclpf != m_kern_adclpf.size()){ // initialize filter
    if(m_verb)
    // allocating memory
    m_kern_adclpf.resize(m_sz_adclpf);
    m_rud_smpl.resize(m_sz_adclpf, (int) m_acp.rud_rmc);
    m_meng_smpl.resize(m_sz_adclpf, (int) m_acp.meng_rmc);
    m_seng_smpl.resize(m_sz_adclpf, (int) m_acp.seng_rmc);
    //m_rud_sta_smpl.resize(m_sz_adclpf, (int) m_acp.rud_sta);

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

  m_rud_smpl[m_cur_adcsmpl] = m_acp.rud_rmc;
  m_meng_smpl[m_cur_adcsmpl] = m_acp.meng_rmc;
  m_seng_smpl[m_cur_adcsmpl] = m_acp.seng_rmc;
  //m_rud_sta_smpl[m_cur_adcsmpl] = m_acp.rud_sta;
  

  // kernel convolution
  double rud = 0., meng = 0., seng = 0., rud_sta = 0.;
  
  for(int i = 0, j = m_cur_adcsmpl; i < m_sz_adclpf; i++, j = (j + 1) % m_sz_adclpf ){
    rud += m_rud_smpl[j] * m_kern_adclpf[i];
    meng += m_meng_smpl[j] * m_kern_adclpf[i];
    seng += m_seng_smpl[j] * m_kern_adclpf[i];
    //rud_sta += m_rud_sta_smpl[j] * m_kern_adclpf[i];
  }
  
  m_acp.rud_rmc = rud;
  m_acp.meng_rmc = meng;
  m_acp.seng_rmc = seng;
  // m_acp.rud_sta = rud_sta;

  m_cur_adcsmpl = (m_cur_adcsmpl > 0 ? m_cur_adcsmpl - 1 : m_sz_adclpf - 1);
}

void f_aws1_ctrl::rcv_acs_udp(s_aws1_ctrl_pars & acpkt)
{
  acpkt.suc = false;
  if(m_acs_sock == -1){
    m_acs_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_acs_sock == -1){
      cerr << "Failed to create socket in " << m_name << "." << endl;
      m_acp.ctrl_src = ACS_FSET;
      return;
    }
    m_acs_sock_addr.sin_family = AF_INET;
    m_acs_sock_addr.sin_port = htons(m_acs_port);
    set_sockaddr_addr(m_acs_sock_addr);
    if(::bind(m_acs_sock, (sockaddr*) &m_acs_sock_addr, sizeof(m_acs_sock_addr)) == SOCKET_ERROR){
      cerr << "Socket error during binding UDP socket in " << m_name;
      cerr << ". AWS Control Source is automatically set to ACS_FSET." << endl;
      m_acp.ctrl_src = ACS_FSET;
      m_acs_sock = -1;
    }
  }else{
    m_sz_acs_ret_addr = sizeof(m_acs_ret_addr);
    int res;
    fd_set fr, fe;
    timeval tv;
    
    // recieving packet
    FD_ZERO(&fr);
    FD_ZERO(&fe);
    tv.tv_sec = 0;
    tv.tv_usec = 10000;

    res = select((int) m_acs_sock + 1, &fr, NULL, &fe, &tv);
    if(res > 0){
      if(FD_ISSET(m_acs_sock, &fr)){
	int len = recvfrom(m_acs_sock, (char*) &acpkt, sizeof(acpkt), 0, (sockaddr*)&m_acs_ret_addr, &m_sz_acs_ret_addr);
	if(len == SOCKET_ERROR){
	  cerr << "Socket error during recieving packet in " << m_name;
	  cerr << " . Now closing socket." << endl;
	  closesocket(m_acs_sock);
	  m_acs_sock = -1;
	  m_acp.ctrl_src = ACS_FSET;
	}
      }else if(FD_ISSET(m_acs_sock, &fe)){
	cerr << "Socket error during recieving packet in " << m_name;
	cerr << " . Now closing socket." << endl;
	closesocket(m_acs_sock);
	m_acs_sock = -1;
	m_acp.ctrl_src = ACS_FSET;
      }
    }else if(res == -1){
      int en = errno;
      cerr << "Error no " << en << " " << strerror(en) << endl;
    }else{
      cerr << "Unknown error in " << m_name << "." << endl;
    }
  }
}

void f_aws1_ctrl::snd_acs_udp(s_aws1_ctrl_pars & acpkt)
{
  if(m_acs_sock == -1)
    return;

  int len = sendto(m_acs_sock, (char*) &acpkt, sizeof(acpkt), 0, (sockaddr*)&m_acs_ret_addr, m_sz_acs_ret_addr);
}

void f_aws1_ctrl::rcv_acs_chan(s_aws1_ctrl_pars & acpkt)
{
  acpkt.suc = false;
  int len;
  if(m_pacs_in == NULL){
    cerr << m_name  << " does not have control input channel." << endl;
  }else{
    len = m_pacs_in->read((char*) &acpkt, sizeof(acpkt));
  }
}

void f_aws1_ctrl::snd_acs_chan(s_aws1_ctrl_pars & acpkt)
{
  if(m_pacs_out == NULL){
    cerr << m_name << " does not have control output channel." << endl;
  }else{
    int len = m_pacs_out->write((char*) &acpkt, sizeof(acpkt));
  }
}

void f_aws1_ctrl::set_acpkt(s_aws1_ctrl_pars & acpkt)
{
  acpkt.tcur = m_cur_time;
  acpkt.ctrl = m_acp.ctrl;
  acpkt.ctrl_src = m_acp.ctrl_src;
  
  acpkt.rud = m_acp.rud;
  acpkt.meng = m_acp.meng;
  acpkt.seng = m_acp.seng;
  acpkt.rud_aws = m_acp.rud_aws;
  acpkt.meng_aws = m_acp.meng_aws;
  acpkt.seng_aws = m_acp.seng_aws;
  acpkt.rud_rmc = m_acp.rud_rmc;
  acpkt.meng_rmc = m_acp.meng_rmc;
  acpkt.seng_rmc = m_acp.seng_rmc;
  acpkt.rud_sta = m_acp.rud_sta;
  acpkt.rud_sta_out = m_acp.rud_sta_out;  
}

void f_aws1_ctrl::set_ctrl(s_aws1_ctrl_pars & acpkt)
{
  if(!acpkt.suc)
    return;

  m_acp.ctrl = acpkt.ctrl;
  m_acp.ctrl_src = acpkt.ctrl_src;

  m_acp.rud_aws = acpkt.rud_aws;
  m_acp.meng_aws = acpkt.meng_aws;
  m_acp.seng_aws = acpkt.seng_aws;
}
