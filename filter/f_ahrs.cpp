// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_ahrs.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ahrs.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ahrs.cpp.  If not, see <http://www.gnu.org/licenses/>. 

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

#include "f_ahrs.h"

const char * f_ahrs::m_str_razor_cmd[ERC_UNDEF] = {
	"o0", "o1", 
	"ob", "ot", "oc", "on", 
	"osct", "osrt", "osbt", 
	"oscb", "osrb", "osbb",
	"f", "saw"
};

f_ahrs::f_ahrs(const char * name): f_base(name), m_state(NULL),
				   m_cmd(ERC_OT), m_omode(ERC_OT), 
	m_ocont(true), m_sync(false), m_rbuf_tail(0), m_rbuf_head(0), m_tbuf_tail(0),
	m_verb(false), m_readlen(1024)
{
	m_dname[0] = '\0';

	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");

	register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
	register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
	register_fpar("rlen", &m_readlen, "Maximum read length of the serial port");
	register_fpar("br", &m_br, "Baud rate.");
	register_fpar("cmd", (int*)&m_cmd, (int)ERC_UNDEF, m_str_razor_cmd, "Command for Razor AHRS.");
	register_fpar("verb", &m_verb, "Verbose mode for Debug");
}

f_ahrs::~f_ahrs()
{
}

bool f_ahrs::init_run()
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

void f_ahrs::destroy_run()
{
	close_serial(m_hserial);
}

bool f_ahrs::proc()
{
	// command handling
	int cmdlen = set_buf_cmd(m_cmd);
	switch(m_cmd){
	case ERC_O0:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_ocont = false;
		break;
	case ERC_O1:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_cmd = ERC_S;
		m_ocont = true;
		break;
	case ERC_OB:
	case ERC_OT:
	case ERC_OSCT:
	case ERC_OSRT:
	case ERC_OSBT:
	case ERC_OSCB:
	case ERC_OSRB:
	case ERC_OSBB:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_omode = m_cmd;
		m_cmd = ERC_S;
		break;
	case ERC_S:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_sync = true;
		break;
	case ERC_OC:
	case ERC_ON:
	case ERC_F:
	default:
		break;
	}

	// sensor output handling	
	if(m_sync){
		m_cmd = ERC_UNDEF;
		// seeking for #SYNCHaw<cr><lf>
		int len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);
		if(len > 0)
		  m_rbuf_tail += len;
		int i, j = 0;
		int tocnt = 0;
		while(1){
			const char * sync_str = "#SYNCHaw\r\n";
			for(i = m_rbuf_head; i < m_rbuf_tail; i++){
				if(m_rbuf[i] == sync_str[j])
					j++;
				else 
					j = 0;
				if(j == 10){
					m_sync = false;
					m_rbuf_head = i + 1;
					break;
				}
			}

			if(i == m_rbuf_tail){
				m_rbuf_head = m_rbuf_tail = 0;
			}

			if(!m_sync)
				break;

			len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);

			if(len > 0)
			  m_rbuf_tail += len;

			tocnt++;
			if(tocnt == 1000000){
			  cout << m_name << "Synchronization failed." << endl;
				m_cmd = ERC_S;
				break;
			}
		}
		
		for(i = m_rbuf_head, j = 0; i < m_rbuf_tail; i++, j++)
			m_rbuf[j] = m_rbuf[i];
		m_rbuf_tail = m_rbuf_tail - m_rbuf_head;
		m_rbuf_head = 0;
	}else{	
	  int len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);

	  if(len > 0)
	    m_rbuf_tail += len;
	}

	if(m_cmd == ERC_S)
		return true;

	switch(m_omode){
	case ERC_OB: 
		// total 12byte
		// <Y 4byte> <P 4byte> <R 4byte> 
		set_bin_ypr();
		break;
	case ERC_OT:
		// YPR=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSCT:
		// A-C=xxxx,xxxx,xxxx<cr><lf> 
		// M-C=xxxx,xxxx,xxxx<cr><lf>
		// G-C=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSRT:
		// A-R=xxxx,xxxx,xxxx<cr><lf> 
		// M-R=xxxx,xxxx,xxxx<cr><lf>
		// G-R=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSBT:
		// A-R=xxxx,xxxx,xxxx<cr><lf> 
		// M-R=xxxx,xxxx,xxxx<cr><lf>
		// G-R=xxxx,xxxx,xxxx<cr><lf>
		// A-C=xxxx,xxxx,xxxx<cr><lf> 
		// M-C=xxxx,xxxx,xxxx<cr><lf>
		// G-C=xxxx,xxxx,xxxx<cr><lf>
		while(seek_txt_buf()){
			dec_tok_val();
		}
		break;
	case ERC_OSCB:
		// total 36byte
		// <A-C x 4byte> <A-C y 4byte> <A-C z 4byte>
		// <M-C x 4byte> <M-C y 4byte> <M-C z 4byte>
		// <G-C x 4byte> <G-C y 4byte> <G-C z 4byte>
		set_bin_9(&m_cal);
		break;
	case ERC_OSRB:
		// total 36byte
		// <A-R x 4byte> <A-R y 4byte> <A-R z 4byte>
		// <M-R x 4byte> <M-R y 4byte> <M-R z 4byte>
		// <G-R x 4byte> <G-R y 4byte> <G-R z 4byte>
		set_bin_9(&m_raw);
		break;
	case ERC_OSBB:
		// total 72byte
		// <A-R x 4byte> <A-R y 4byte> <A-R z 4byte>
		// <M-R x 4byte> <M-R y 4byte> <M-R z 4byte>
		// <G-R x 4byte> <G-R y 4byte> <G-R z 4byte>
		// <A-C x 4byte> <A-C y 4byte> <A-C z 4byte>
		// <M-C x 4byte> <M-C y 4byte> <M-C z 4byte>
		// <G-C x 4byte> <G-C y 4byte> <G-C z 4byte>
		set_bin_9x2(&m_raw, &m_cal);
		break;
	}

	if(m_verb){
		bool braw = false;
		bool bcal = false;
		switch(m_omode){
		case ERC_OSRT:
		case ERC_OSRB:
		case ERC_OSBB:
		case ERC_OSBT:
			braw = true;
		}

		switch(m_omode){
		case ERC_OSCB:
		case ERC_OSCT:
		case ERC_OSBB:
		case ERC_OSBT:
			bcal = true;
		}

		if(braw){
			cout << "A-R=" << m_raw.ax << " " << m_raw.ay << " " << m_raw.az << endl;
			cout << "M-R=" << m_raw.mx << " " << m_raw.my << " " << m_raw.mz << endl;
			cout << "G-R=" << m_raw.gx << " " << m_raw.gy << " " << m_raw.gz << endl;
		}

		if(bcal){
			cout << "A-C=" << m_cal.ax << " " << m_cal.ay << " " << m_cal.az << endl;
			cout << "M-C=" << m_cal.mx << " " << m_cal.my << " " << m_cal.mz << endl;
			cout << "G-C=" << m_cal.gx << " " << m_cal.gy << " " << m_cal.gz << endl;
		}

		if(!braw && !bcal){
			cout << "YPR=" << m_ypr.y << " " << m_ypr.p << " " << m_ypr.r << endl;
		}
	}

	if(m_state){
	  m_state->set_attitude(get_time(), m_ypr.r, m_ypr.p, m_ypr.y);
	}

	return true;
}

int f_ahrs::seek_txt_buf()
{
	char * ptbuf = m_tbuf + m_tbuf_tail;
	char * prbuf = m_rbuf + m_rbuf_head;

	int rbuflen = m_rbuf_tail - m_rbuf_head; // length of the string remained in the rbuf
	int len = 0; // length of the string read from rbuf.

	if(m_tbuf_tail == 0){
		// seeking for the token head
		for(;*prbuf != '#' && len < rbuflen; prbuf++, len++);

		if(len == rbuflen){
			m_rbuf_tail = m_rbuf_head = 0;
			return 0;
		}

		*ptbuf = *prbuf; ptbuf++; prbuf++; len++;

		if(len == rbuflen){
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += 1;
			return 0;
		}

		m_tbuf_tail = 1;
		m_rbuf_head += len;
		rbuflen -= len;
		len = 0;
	}

	int tbuflen = m_readlen - m_tbuf_tail;
	int buflen = min(rbuflen, tbuflen);

	// Seeking for CRLF, and extract the token.
	while(1){
		// seeking for CR
		for(;*prbuf != 0x0d && len < buflen; *ptbuf = *prbuf, ptbuf++, prbuf++, len++);

		if(len == rbuflen){ // read buffer is ran out
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += len;
			return 0;
		}else if(len == tbuflen){ // token buffer is over
			m_tbuf_tail = 0;
		}

		*ptbuf = *prbuf; ptbuf++; prbuf++; len++;

		if(len == rbuflen){ // read buffer is ran out
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += len;
			return 0;
		}else if(len == tbuflen){ // token buffer is over
			m_tbuf_tail = 0;
		}

		if(*prbuf == 0x0a){ // LF  is found
			*ptbuf = *prbuf; ptbuf++; prbuf++; len++;
			*ptbuf = '\0';
			m_rbuf_head += len;
			len += m_tbuf_tail;
			m_tbuf_tail = 0;
			return len;
		}
	}

	return 0;
}
