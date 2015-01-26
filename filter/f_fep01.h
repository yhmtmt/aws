#ifndef _F_FEP01_H_
#define _F_FEP01_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_fep01.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_fep01.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_fep01.h.  If not, see <http://www.gnu.org/licenses/>. 
#include "../util/aws_serial.h"
#include "f_base.h"

class f_fep01: public f_base
{
protected:
	static const char * m_cmd_str[32];

	char m_dname[1024];
	unsigned short m_port;
	unsigned int m_br;
	AWS_SERIAL m_hcom;

	unsigned char m_addr;		// own address (0 to 255)
	unsigned char m_addr_group; // group address (0 to 255)
	unsigned char m_addr_dst;	// destination address (0 to 255), header less mode only.
	unsigned char m_header_less;// header less packet mode (0: no -> 0xF0 or 1: yes -> 0xFF)
	unsigned char m_scramble_0; // scramble code 0 (0 to 255)
	unsigned char m_scramble_1; // scramble code 1 (0 to 255)
	unsigned char m_num_freqs;	// number of frequencies used. (0 to 3)
	unsigned char m_freq0;		// 0th frequency. for L-band 024 to 060, for H-band 062 to 077
	unsigned char m_freq1;		// 1th frequency. for L-band 024 to 060, for H-band 062 to 077
	unsigned char m_freq2;		// 2th frequency. for L-band 024 to 060, for H-band 062 to 077
	unsigned char m_ant;		// antenna (0:A or 1:B)
	unsigned char m_div;		// diversity (0:use or 1:no use)
	unsigned char m_num_reps;	// number of re-transmission (0 to 255)
	unsigned char m_th_roam;	// roaming threashold (70 to 100) [-dbm]
	unsigned char m_rep_power;	// report power (0:no or 1:yes)
	unsigned char m_rep_err;	// report n0 response (0:yes or 1: no)
	unsigned char m_rep_suc;	// report success response (0:P0, P1 or 1:P0)
	unsigned char m_rep;		// report command response (0: yes or 1: no)
	unsigned char m_tint_cmd;	// interval to recognize command in header less mode (0 to 255) [msec]
	unsigned char m_fband;		// frequency band (0:L or 1:H)
	unsigned char m_tbclr;		// interval to clear reciver buffer (1 to 255) [msec]
	unsigned char m_wait_freq;  // time for waiting requency response (0 to 7) corresponding to (120 to 50) [msec]
	unsigned char m_chk_group;  // check group addres (0: no or 1: yes)
	unsigned char m_chk_addr;	// check destination address (0: no or 1: yes)
	unsigned char m_int_bcn;	// beacon interval (0 to 7) correspondint to (800 to 100) [msec]
	unsigned char m_bcn_freq;	// beacon frequency (0: var or 1: fix)
	unsigned char m_bcn;		// beacon (0: no or 1: yes)
	unsigned char m_ser_dlen;	// serial data length (0: 8bit or 1: 7bit)
	unsigned char m_ser_par;	// serial com parity (0: no or 1: yes)
	unsigned char m_ser_stp;	// serial com stop (0: 1bit or 1: 2bit)
	unsigned char m_tlp_wait_ex;// low power waiting time extension (0 to 15) corresponding to (100 to 1500) [msec]
	unsigned char m_lp_wait;	// low power waiting mode (0: no or 1: yes)
	unsigned char m_flw;		// RTS/CTS flow control (0: no or 1: yes)
	unsigned char m_tlp_wait;	// low power waiting time (0 to 255) [100msec]
	unsigned char m_crlf;		// adding cr,lf to recieved data in header less mode (0: no or 1: yes) 
	unsigned char m_delim;		// sending delimiter for header less mode (0: time out or 1: cr,lf)
	unsigned char m_tlp_slp;	// sleep time for low power waiting mode (0 to 255) [100msec]
	unsigned char m_to_hlss;	// time interval to transmit in header less mode (1 to 255) [10msec]
	unsigned char m_addr_rep0;	// address for repeater 0 in header less mode (0 to 255)
	unsigned char m_addr_rep1;  // address for repeater 1 in header less mode (0 to 255)

public:
	f_fep01(const char * name):f_base(name), m_port(0), m_br(9600), m_hcom(NULL_SERIAL),
		m_addr(0x00), m_addr_group(0xF0), m_addr_dst(0x00), m_header_less(0),
		m_scramble_0(0xFF), m_scramble_1(0xFF), m_num_freqs(0x03), m_freq0(0x18), m_freq1(0x2A), m_freq2(0x3C),
		m_ant(0), m_div(1), m_num_reps(0x0A), m_th_roam(0x50), m_rep_power(0), m_rep_err(0), m_rep_suc(0), m_rep(0),
		m_tint_cmd(0x00), m_fband(0), m_tbclr(0x64), m_wait_freq(4), m_chk_group(1), m_int_bcn(0), m_bcn_freq(0),
		m_bcn(0), m_tlp_wait_ex(0), m_lp_wait(0), m_flw(0), m_tlp_wait(0x0F), m_crlf(0), m_delim(1), m_tlp_slp(0x0F),
		m_to_hlss(0x01), m_addr_rep0(0xFF), m_addr_rep1(0xFF)
	{
		m_dname[0] = '\0';
		register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
		register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
		register_fpar("br", &m_br, "Baud rate.");
	}

	virtual bool init_run()
	{
#ifdef _WIN32
		m_hcom = open_serial(m_port, m_br);
#else
		m_hcom = open_serial(m_dname, m_br);
#endif
		if(m_hcom == NULL_SERIAL)
			return false;
		return true;
	}

	virtual void destroy_run()
	{
		if(!close_serial(m_hcom)){
			cerr << "Failed to close serial port." << endl;
		}

		m_hcom = NULL_SERIAL;
	}

	virtual bool proc()
	{
		return true;
	}
};

#endif