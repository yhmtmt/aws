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
#include "../util/util.h"
#include "../util/aws_serial.h"
#include "f_base.h"


// Note:  HEADER LESS MODE IS PROHIBITED
// I do not support header less mode. Because the recieved packets cannot be distinguished from some
// device responses. For example, if the header less mode and power reporting mode are enabled simultaneously,
// We cannot distinguish the 9 byte packet "test<cr><lf>080" from 6 byte packet"test<cr><lf>".

class f_fep01: public f_base
{
protected:
	enum e_cmd{
		NUL, ARG,  BAN, BCL, DAS,
		DBM, DVS, FCN, FRQ, IDR,
		IDW, INI, PAS, POF, PON, 
		PTE, PTN, PTS, ROF, RON,
		REG, RID, RST, TBN, TBR,
		TB2, TID, TS2, TXT, TXR,
		TX2, VER
	};
	static const char * m_cmd_str[32];

	enum e_msg_rcv{
		RBN, RBR, RB2, RXT, RXR, RX2, RNUL
	};

	enum e_cmd_stat{
		NRES=0x00, P0 = 0x01, P1 = 0x02, N0 = 0x04, N1 = 0x08, N2 = 0x10, N3 = 0x20, EOC = 0x40
	};

	static const char * m_rec_str[6];

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
	unsigned char m_fwait;  // time for waiting requency response (0 to 7) corresponding to (120 to 50) [msec]
	unsigned char m_chk_group;  // check group addres (0: no or 1: yes)
	unsigned char m_chk_addr;	// check destination address (0: no or 1: yes)
	unsigned char m_int_bcn;	// beacon interval (0 to 7) correspondint to (800 to 100) [msec]
	unsigned char m_fbcn;		// beacon frequency (0: var or 1: fix)
	unsigned char m_bcn;		// beacon (0: no or 1: yes)
	unsigned char m_ser_dlen;	// serial data length (0: 8bit or 1: 7bit)
	unsigned char m_ser_par;	// serial com parity (0: no or 1: yes)
	unsigned char m_ser_br;		// serial baud rate (0 to 3) corresponding to {9600, 19200, 384000, 115200}
	unsigned char m_ser_stp;	// serial com stop (0: 1bit or 1: 2bit)
	unsigned char m_tlp_wait_ex;// low power waiting time extension (1 to 15) corresponding to (100 to 1500) [msec]
	unsigned char m_lp_wait;	// low power waiting mode (0: no or 1: yes)
	unsigned char m_flw;		// RTS/CTS flow control (0: no or 1: yes)
	unsigned char m_tlp_wait;	// low power waiting time (0 to 255) [100msec]
	unsigned char m_crlf;		// adding cr,lf to recieved data in header less mode (0: no or 1: yes) 
	unsigned char m_delim;		// sending delimiter for header less mode (0: time out or 1: cr,lf)
	unsigned char m_tlp_slp;	// sleep time for low power waiting mode (0 to 255) [100msec]
	unsigned char m_to_hlss;	// time interval to transmit in header less mode (1 to 255) [10msec]
	unsigned char m_addr_rep0;	// address for repeater 0 in header less mode (0 to 254), 255 means no use.
	unsigned char m_addr_rep1;  // address for repeater 1 in header less mode (0 to 254), 255 means no use.

	unsigned char m_reg[29];	// register
	unsigned char m_dbm;		// dbm command result
	unsigned int m_rid;			// recieved id
	unsigned char m_ver;		// main version
	unsigned short m_sub_ver;	// sub version 
	bool read_reg();			// load register values to our m_reg
	bool write_reg();			// write m_reg values to m_reg
	bool pack_reg();			// pack filter parameters to the regs

	int m_rbuf_len;
	char m_rbuf[512];
	int m_wbuf_len;
	char m_wbuf[512];

	// parameters used by parser
	int m_pbuf_tail;
	char m_pbuf[512]; // buffer used by parser
	bool parse_rbuf();
	void init_parser();
	bool parse_response_value();
	bool parse_response();
	bool parse_message_type();
	bool parse_message_header();
	bool parse_message();

	struct s_cmd{
		e_cmd type;
		char msg[129];
		union{
			int iarg1;
			char carg1[2];
		};
		union{
			int iarg2;
			char carg2[2];
		};

		int iarg3, iarg4;

		s_cmd():type(NUL), iarg1(-1), iarg2(-1), iarg3(-1), iarg4(-1){
			msg[0] = '\0';
		}
	};
	bool m_bpush_cmd;
	s_cmd m_cmd;
	list<s_cmd> m_cmd_queue;
	bool set_cmd();

	e_cmd m_cur_cmd;
	unsigned char m_cmd_arg1, m_cmd_arg2;
	unsigned char m_parse_cr; // CR=0x0D
	unsigned char m_parse_lf; // LF=0x0A
	int m_parse_count;
	bool m_ts2_mode;
	bool m_msg_bin;
	ofstream m_flog_ts2;
	unsigned int m_cmd_stat;
	e_msg_rcv m_cur_rcv;
	bool m_rcv_header;
	bool m_rcv_done;
	unsigned char m_rcv_src, m_rcv_rep0, m_rcv_rep1, m_rcv_len;
	char m_rcv_msg[256];

	struct c_parse_exception
	{
		e_cmd cmd;
		int stat;
		int line;
		c_parse_exception(e_cmd acmd, int astat, int aline): cmd(acmd), stat(astat), line(aline)
		{
		}
	};
public:
	f_fep01(const char * name);

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
		// push a command to the command queue from a fset command
		if(m_bpush_cmd){
			m_cmd_queue.push_back(m_cmd);
			m_cmd = s_cmd();
		}

		// pop a command from queue and set to write buffer
		if(m_cmd_queue.size() != 0){
			set_cmd();
		}

		// handling recieved message. this code is temporal and simply dumping to stdout. 
		if(m_rcv_done){
			m_rcv_msg[m_rcv_len] = '\0';
			cout << "Recieve: " << m_rcv_msg << endl;
			m_rcv_done = true;
		}

		// handling processed command 
		if(m_cur_cmd != NUL && m_cmd_stat & EOC){
			cout << "Cmd " << m_cmd_str[m_cur_cmd] << " done." << endl;
			m_cur_cmd = NUL;
			m_cmd_stat = NRES;
		}

		// parsing read buffer. The read buffer is possible to contain both command response and recieved messages.
		m_rbuf_len = read_serial(m_hcom, m_rbuf, 512);
		if(m_rbuf_len < 0){
			cerr << "Read operation failed." << endl;
		}else{
			if(!parse_rbuf()){
				cerr << "Failed to parse read buffer." << endl;
			}
		}

		// writing a command if it is in the write buffer
		m_wbuf_len = (int) strlen(m_wbuf);

		int wlen = write_serial(m_hcom, m_wbuf, m_wbuf_len);
		if(m_wbuf_len != wlen){
			cerr << "Write operation failed." << endl;
		}

		m_wbuf[0] = '\0';

		return true;
	}
};

inline int str3DigitDecimal(const char * str)
{
	return (((int)str[0] - '0') * 100 + ((int)str[1] - '0') * 10  + ((int)str[2] - '0'));
}

inline int str2DigitDecimal(const char * str)
{
	return ((int)str[0] - '0') * 10  + ((int)str[1] - '0');
}

inline int str2DigitHex(const char * str)
{
	return h2i(str[0]) * 16 + h2i(str[1]);
}

#endif