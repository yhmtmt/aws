#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_nmea.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_nmea.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_nmea.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;
#include "f_nmea.h"

///////////////////////////////////////////////////////////// f_nmea
bool f_nmea::open_com()
{
#ifdef _WIN32 // for windows
	m_hcom = open_serial(m_port, m_cbr);
#else // for Linux 
	m_hcom = open_serial(m_fname, m_cbr);
#endif	
	if(m_hcom == NULL_SERIAL){
		return false;
	}
	return true;

}

bool f_nmea::open_udp(){
	m_sock = socket(AF_INET, SOCK_DGRAM, 0);
		m_sock_addr.sin_family = AF_INET;
	m_sock_addr.sin_port = htons(m_port);

	set_sockaddr_addr(m_sock_addr, m_host);

	if(::bind(m_sock, (sockaddr*)&m_sock_addr, sizeof(m_sock_addr)) == SOCKET_ERROR){
		return false;
	}

	if(strlen(m_dst_host)){	
		m_sock_out = socket(AF_INET, SOCK_DGRAM, 0);
		m_sock_addr_out.sin_family = AF_INET;
		m_sock_addr_out.sin_port = htons(m_port);
		set_sockaddr_addr(m_sock_addr_out, m_dst_host);
		if(::bind(m_sock_out, (sockaddr*) & m_sock_addr_out, sizeof(m_sock_addr_out)) == SOCKET_ERROR){
			return false;
		}
	}
	return true;
}

bool f_nmea::seek(long long seek_time)
{
	if(m_nmea_src != FILE)
		return true;

	if(!m_file.is_open()){
		return false;
	}

	int prev = (int) m_file.tellg();
	tmex tm;
	while(!m_file.eof()){
		m_file.getline(m_buf, sizeof(m_buf));

		if(!decTmStr(m_buf, tm)){
			cerr << "f_nmea" << m_name << " cannot decode time in " << m_buf << endl;
			continue;
		}else{
			long long rec_time = mkgmtimeex_tz(tm, m_time_zone_minute) * MSEC;
			if(rec_time >= seek_time){
				m_file.seekg(prev);
				return true;
			}
		}
		prev = (int) m_file.tellg();
	}

	return false;
}

bool f_nmea::open_file(){
	m_file.open(m_fname);
	if(!m_file.is_open())
		return false;
	return true;
}


bool f_nmea::rcv_file()
{
	// file record is <time> <nmea>. <time> is 19 digit. <nmea> is 83 chars.

	if(!m_file.is_open()){
		return false;
	}

	long long rec_time = 0;
	do{
		if(m_file.eof())
			return false;

		int anchor = (int) m_file.tellg();

		m_file.getline(m_buf, sizeof(m_buf));
		tmex tm;
		if(!decTmStr(m_buf, tm)){
			cerr << "f_nmea " << m_name << " cannot decode time in " << m_buf << endl;
			continue;
		}else{
			rec_time = mkgmtimeex_tz(tm, m_time_zone_minute) * MSEC;
		}

		if(rec_time > m_cur_time){
			m_file.seekg(anchor);
			break;
		}

		//cout << &m_buf[31] << endl;
		if(m_chout){
			int len = (int) strlen(&m_buf[31]);
			if(len >= 84){
				cerr << "Irregal long sentence detected. :" << m_buf << endl;
				cerr << "the length is " << len << endl;
				continue;
			}
			if(is_filtered(&m_buf[31])){
				if(!m_chout->push(&m_buf[31]))
					cerr << "Buffer overflow in ch_nmea " << m_chout->get_name() << endl;
			}
		}

		if(m_blog){
			m_flog << get_time_str() << &m_buf[31] << endl;
		}

	}while(rec_time <= m_cur_time);
	return true;
}

bool f_nmea::rcv_com()
{
	int nrcv;
	while(nrcv = read_serial(m_hcom, &m_buf[m_buf_head], SIZE_NMEA_BUF - m_buf_head)){
		m_buf_tail += nrcv;
		extract_nmea_from_buffer();
	}
	return true;
}

bool f_nmea::rcv_udp()
{
	int nrcv;
	while(nrcv = recv(m_sock, m_buf + m_buf_head, SIZE_NMEA_BUF - m_buf_head, 0)){
		m_buf_tail += nrcv;
		extract_nmea_from_buffer();
	}
	return true;
}

void f_nmea::extract_nmea_from_buffer()
{
  while(m_buf_tail){
    // if the nmea sentence is empty, seek for the head mark ! or $.
    if(m_nmea_tail == 0){
      for(;m_buf[m_buf_head] != '!' 
	    && m_buf[m_buf_head] != '$' 
	    && m_buf_head < m_buf_tail; m_buf_head++);
      
      if(m_buf_head == m_buf_tail){
	m_buf_head = 0;
	m_buf_tail = 0;
	break;
      }
    }
	
    // Copy the nmea string from buffer 
    for(;m_buf_head < m_buf_tail && m_nmea_tail < 84; 
	m_buf_head++, m_nmea_tail++){
      m_nmea[m_nmea_tail] = m_buf[m_buf_head];
      if(m_buf[m_buf_head] == 0x0D){// detecting CR
	m_nmea[m_nmea_tail] = '\0';
	m_nmea_tail = -1; // uses -1 as flag of nmea extraction
	break;
      }else if(m_nmea_tail == 84){
	m_nmea[83] = '\0';
	cerr << m_name << "::extrac_nmea_from_buffer" << 
	  " No termination character detected in 83 characters." << endl;
	cerr << "    -> string: " << m_nmea << endl;
	m_nmea_tail = 0;
      }
    }
    
    // if the  buffer ends before the full nmea sentence is transfered
    // we need to fill buffer with newly received data.
    if(m_buf_head == m_buf_tail){
      m_buf_head = 0;
      m_buf_tail = 0;
      break;
    }
    
    // If the complete nmea found, send it to the channel and log file.
    // Then the string remained in the buffer is shiftted to the head.
    if(m_nmea_tail == -1){
      if(is_filtered(m_nmea)){
	if(!m_chout->push(m_nmea)){
	  cerr << "Buffer overflow in ch_nmea " << m_chout->get_name() << endl;
	}
      }
      
      if(m_blog){
	if(m_flog.is_open())
	  m_flog << get_time_str() << m_nmea << endl;
	else{
	  sprintf(m_fname_log, "%s_%lld.nmea", m_name, m_cur_time);
	  m_flog.open(m_fname_log);
	}
      }
      
      if(m_verb){
	cout << m_name << " > " << m_nmea << endl;
      }
      
      // remained data moves to the buffer head
      int itail = 0;
      for(; m_buf_head < m_buf_tail; m_buf_head++, itail++){
	m_buf[itail] = m_buf[m_buf_head];
      }
      m_buf_head = 0; 
      m_buf_tail = itail;
      m_nmea_tail = 0;
    }
  }
}

bool f_nmea::proc(){
	int total = 0;
	int nrcv = 0;
	while(send_nmea());

	switch(m_nmea_src){
	case FILE:
		if(!rcv_file())
			return false;
		break;
	case COM:
		if(!rcv_com())
			return false;
		break;
	case UDP:
		if(!rcv_udp())
			return false;
		break;
	}
	return true;
}

int f_nmea::send_nmea()
{
	if(!m_chin){
		return 0;
	}

	int len = 0;
	if(m_chin->pop(m_buf_send)){
		int _len = (int) strlen(m_buf_send);
		if(m_buf_send[_len-2] != 0x0D && m_buf_send[_len-1] != 0x0A){
		  m_buf_send[_len] = 0x0D;
		  m_buf_send[_len+1] = 0x0A;
		  m_buf_send[_len+2] = 0x00;
		}

		if(m_blog){
			m_flog << get_time_str() << m_buf_send << endl;
		}
		if(m_verb){
		  cout << m_name << " > " << m_buf_send << endl;
		}
		
		switch(m_nmea_src){
		case FILE:
			cout << m_buf_send << endl;
			len = (int) strlen(m_buf_send);
			break;
		case COM:
			len = write_serial(m_hcom, m_buf_send, (int) strlen(m_buf_send));
			break;
		case UDP:
			len = send(m_sock_out, m_buf_send, (int) sizeof(m_buf_send), 0);
			break;
		}
	}
	return len;
}

bool f_nmea::init_run()
{

	for(int ich = 0;ich < f_base::m_chout.size(); ich++){
		ch_nmea * pch = dynamic_cast<ch_nmea*>(f_base::m_chout[ich]);
		if(pch != NULL){
			m_chout = pch;
		}
	}

	for(int ich = 0; ich < f_base::m_chin.size(); ich++){
		ch_nmea * pch = dynamic_cast<ch_nmea*>(f_base::m_chin[ich]);
		if(pch != NULL)
			m_chin = pch;
	}

	dec_type_str();
	switch(m_nmea_src){
	case NONE:
		return false;
	case FILE:
		if(!open_file())
			return false;
		break;
	case COM:
		if(!open_com())
			return false;
		break;
	case UDP:
		if(!open_udp())
			return false;
		break;
	}
	if(m_blog){
		time_t t = time(NULL);
		sprintf(m_fname_log, "%s_%lld.nmea", m_name, (long long int)t);
		m_flog.open(m_fname_log);
		if(!m_flog.is_open()){
			destroy_run();
			return false;
		}
	}

	m_buf_head = 0;
	m_buf_tail = 0;
	m_nmea_tail = 0;
	return true;
}

void f_nmea::destroy_run(){
	switch(m_nmea_src){
	case FILE:
		m_file.close();
		break;
	case COM:
#ifdef _WIN32
		CloseHandle(m_hcom);
		m_hcom = NULL;
#else
		::close(m_hcom);
		m_hcom = 0;
#endif
		break;
	case UDP:
#ifdef _WIN32
		closesocket(m_sock);
#else
		::close(m_sock);
#endif
	}
	if(m_flog.is_open())
		m_flog.close();
}

///////////////////////////////////////////////////////////// f_nmea_proc
const char * f_nmea_proc::str_aibm_type[AIBM_HEX + 1] = {
	"TEXT", "C6", "C8", "BIN", "HEX"
};

bool f_nmea_proc::init_run()
{
	for(int ich = 0; ich < f_base::m_chin.size(); ich++){
		m_chin = dynamic_cast<ch_nmea*>(f_base::m_chin[ich]);
		if(m_chin)
			break;
	}

	for(int ich = 0; ich < f_base::m_chout.size(); ich++){
		m_chout = dynamic_cast<ch_nmea*>(f_base::m_chout[ich]);
		if(m_chout)
			break;
	}

	return true;
}

void f_nmea_proc::destroy_run()
{
	m_chin = NULL;
}

bool f_nmea_proc::proc()
{
	if(!m_chin)
		return false;

	// sending phase
	if(strlen(m_str_aibm)){
		bool ret;
		switch(m_aibm_type){
		case AIBM_TEXT:
			ret = m_aibm.set_msg_text(m_str_aibm);
			break;
		case AIBM_C6:
			ret = m_aibm.set_msg_c6(m_str_aibm);
			break;
		case AIBM_C8:
			ret = m_aibm.set_msg_c8(m_str_aibm);
			break;
		case AIBM_BIN:
			ret = m_aibm.set_msg_bin(m_str_aibm);
			break;
		case AIBM_HEX:
			ret = m_aibm.set_msg_hex(m_str_aibm);
			break;
		}
		vector<string> nmeas;
		if(ret){
			ret = m_aibm.gen_nmea(m_toker, nmeas);
		}

		if(ret){
			for(int i = 0; i < nmeas.size(); i++){
				m_chout->push(nmeas[i].c_str());
			}
			m_str_aibm[0] = '\0';
		}
	}

	c_ship::list_lock();
	while(m_chin->pop(m_buf)){
		const c_nmea_dat * pnd = m_nmeadec.decode(m_buf);
		if(pnd == NULL)
			continue;

		// VDM1, 18, 19, 5, 24, TTM, GPS own ship should be registered
		switch(pnd->get_type()){
			case ENDT_VDM1:
				c_ship::register_ship_by_vdm1((c_vdm_msg1*)pnd, m_cur_time);
				break;
			case ENDT_VDM18:
				c_ship::register_ship_by_vdm18((c_vdm_msg18*)pnd, m_cur_time);
				break;
			case ENDT_VDM19:
				c_ship::register_ship_by_vdm19((c_vdm_msg19*)pnd, m_cur_time);
				break;
			case ENDT_VDM5:
				c_ship::register_ship_by_vdm5((c_vdm_msg5*)pnd);
				break;
			case ENDT_VDM8:
				c_ship::register_ship_by_vdm8((c_vdm_msg8*)pnd, m_cur_time, m_bm_ver);
				break;
			case ENDT_VDM24:
				c_ship::register_ship_by_vdm24((c_vdm_msg24*)pnd);
				break;
			case ENDT_TTM:
				c_ship::register_ship_by_ttm((c_ttm*) pnd, m_cur_time);
			case ENDT_GGA:
				break;
			case ENDT_RMC:
				{
					c_rmc * prmc = (c_rmc*) pnd;
					if(!prmc->m_v)
						break;

					c_ship::set_own_rmc(prmc, m_cur_time);
					if(m_btime_rmc){

						tmex tm;
						tm.tm_year = prmc->m_yr + (m_tm.tm_year / 100) * 100;
						tm.tm_mon = prmc->m_mn - 1;
						tm.tm_mday = prmc->m_dy;
						tm.tm_hour = prmc->m_h;
						tm.tm_min = prmc->m_m;
						tm.tm_sec = (int) prmc->m_s;
						tm.tm_msec = (int)((prmc->m_s - tm.tm_sec) * 100);
						tm.tm_isdst = -1;
						f_base::set_time(tm);
					}
				}
				break;
			case ENDT_ABK:
				pnd->show(cout);
				break;
		}

		delete pnd;
	}
	c_ship::delete_timeout_ship(m_cur_time);
	c_ship::list_unlock();

	return true;
}
