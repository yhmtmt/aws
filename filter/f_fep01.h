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
#include "../channel/ch_vector.h"
#include "f_base.h"

// Note:  HEADER LESS MODE IS PROHIBITED
// I do not support header less mode. Because the recieved packets cannot be distinguished from some
// device responses. For example, if the header less mode and power reporting mode are enabled simultaneously,
// We cannot distinguish the 9 byte packet "test<cr><lf>080" from 6 byte packet"test<cr><lf>".

class f_fep01: public f_base
{
protected:
	ch_ring<char> * m_pin, * m_pout;

	// state transition
	// ST_INIT load all register (queue commands,last comand return, call unpack_reg -> ST_OP)
	// ST_RST set all register (pack_reg, queue commands, last command return, re-open device -> ST_OP 
	// ST_OP ordinal operation (recieve data stream from channel)
	// ST_TEST Invoke TS2 
	enum e_state{
		ST_INIT, ST_RST, ST_OP, ST_DBG, ST_TEST
	};
	e_state m_st;
	bool m_ts2_mode;
	static const char * m_st_str[ST_TEST+1];

	enum e_sub_state{
		SST_CMD, SST_PROC
	};
	e_sub_state m_sst;
	static const char * m_sst_str[SST_PROC+1];

	// load register values
	bool handle_init();

	// reset register values
	bool handle_rst();

	// ordinal operation (transmmit data comes from input channel and output recieved data to output channel)
	bool handle_op();

	// manual command mode (debug mode)
	bool handle_dbg();

	// invoke TS2 mode. 
	bool handle_test();

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

	enum e_com_mode{ // communication mode
		CM_P2P
	};

	e_com_mode m_cm;
	static const char * m_cm_str[CM_P2P+1];
	unsigned char m_addr_p2p;

	char m_dname[1024];
	unsigned short m_port;
	unsigned int m_br;
	int m_len_pkt;

	AWS_SERIAL m_hcom;

	// Unpacked register parameters
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
	unsigned char m_ser_sig;	// serial parity sign
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

	// Register parameter
	unsigned char m_reg[29];	// register
	unsigned char m_dbm;		// dbm command result
	unsigned int m_rid;			// recieved id
	unsigned int m_tid;			// own id
	unsigned char m_ver;		// main version
	unsigned short m_sub_ver;	// sub version 

	// Register handling methods
	void dump_reg();			// dump reg values to stdout
	bool pack_reg();			// pack filter parameters to the regs
	void unpack_reg();			// unpack regs to filter parameters

	int m_rbuf_len;
	char m_rbuf[512];
	int m_wbuf_len;
	char m_wbuf[512];

	// for command issue

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
	int m_max_queue;
	list<s_cmd> m_cmd_queue;
	bool set_cmd();

	long long m_tcmd;       // time command issued
	long long m_tcmd_wait;  // interval for data transmission command
	long long m_tcmd_out;   // command time out
	int m_num_max_retry;    // maximum number of retry 
	int m_num_retry;		// number of retry

	e_cmd m_cur_cmd;
	unsigned char m_cmd_arg1, m_cmd_arg2;
	unsigned int m_cmd_stat; // Command response P0,P1, N0, N1, N3, End of Command(EOC)
	ofstream m_flog_ts2;

	// parameters used by parser
	int m_pbuf_tail;
	char m_pbuf[512]; // buffer used by parser
	unsigned char m_parse_cr; // indicates CR=0x0D recieved. 
	unsigned char m_parse_lf; // indicates LF=0x0A recieved. 
	int m_parse_count; // for parsing binary message, CRLF cannot be delimiter. 
					   //Thus we use the data count recoreded in the message header to parse the message.

	bool parse_rbuf();
	void init_parser();
	bool parse_response_value();
	bool parse_response();
	bool parse_message_type();
	bool parse_message_header();
	bool parse_message();
	struct c_parse_exception
	{
		e_cmd cmd;
		int stat;
		int line;
		c_parse_exception(e_cmd acmd, int astat, int aline): cmd(acmd), stat(astat), line(aline)
		{
		}
	};


	// for Message parser
	e_msg_rcv m_cur_rcv;	 // Current type of recieved message
	bool m_msg_bin;			 // asserted if the recieved message header is binary
	bool m_rcv_header;		 // Flag shows message header indicating message type was recieved.
	unsigned char m_rcv_src; // Message source address
	unsigned char m_rcv_rep0; // Message source repeater address 1
	unsigned char m_rcv_rep1; // Message source repeater addres 2
	unsigned char m_rcv_len; // Recieved data length in the packet parsing.
	unsigned char m_rcv_pow;			// recieved power. used only if m_rep_power is enabled.

	char m_rcv_msg[1024];	 // recieved message.This stores all the messages until they are transmitted to the output channel.
	int m_proced_len;		 // Message length transmitted to the output channel in m_rcv_msg
	int m_msg_len;			 // Message length stored in m_rcv_msg
	int m_len_tx;						// transmission length
	long long m_total_tx, m_total_rx;	// total transmission and reception

	// for logging
	char m_fname_txlog[1024], m_fname_rxlog[1024];
	ofstream m_ftxlog, m_frxlog;
	void log_tx(unsigned char len, const char * msg)
	{
		// data record
		// <time> <len> <msg>
		if(m_ftxlog.is_open()){
			m_ftxlog.write((const char*) & m_cur_time, sizeof(m_cur_time));
			m_ftxlog.write((const char*) & len, sizeof(len));
			m_ftxlog.write((const char*) msg, len);
		}
	}

	void log_rx()
	{
		// data record
		// <time> <len> <power> <msg>
		if(m_frxlog.is_open()){
			m_frxlog.write((const char*) & m_cur_time, sizeof(m_cur_time));
			m_frxlog.write((const char*) & m_rcv_len, sizeof(m_rcv_len));
			m_frxlog.write((const char*) & m_rcv_pow, sizeof(m_rcv_pow));
			m_frxlog.write((const char*) m_rcv_msg + m_msg_len, m_rcv_len);
		}
	}

public:
	f_fep01(const char * name);

	virtual bool init_run()
	{
		if(m_fname_txlog[0]){
			m_ftxlog.open(m_fname_txlog);
			if(!m_ftxlog.is_open())
				return false;
		}

		if(m_fname_rxlog[0]){
			m_frxlog.open(m_fname_rxlog);
			if(!m_frxlog.is_open())
				return false;
		}

#ifdef _WIN32
		m_hcom = open_serial(m_port, m_br);
#else
		m_hcom = open_serial(m_dname, m_br);
#endif
		if(m_hcom == NULL_SERIAL)
			return false;

		init_parser();

		m_st = ST_INIT;
		m_sst = SST_CMD;

		if(m_chin.size() == 1){
			m_pin = dynamic_cast<ch_ring<char>*>(m_chin[0]);
		}

		if(m_chout.size() == 1){
			m_pout = dynamic_cast<ch_ring<char>*>(m_chout[0]);
		}

		m_total_tx = m_total_rx = 0;
		return true;
	}

	virtual void destroy_run()
	{
		if(!close_serial(m_hcom)){
			cerr << "Failed to close serial port." << endl;
		}

		m_hcom = NULL_SERIAL;
		if(m_ftxlog.is_open())
			m_ftxlog.close();

		if(m_frxlog.is_open())
			m_frxlog.close();
	}

	virtual bool proc()
	{
		//cout << "Proc/Cycle " << m_count_proc << "/" << m_count_clock << endl;
		switch(m_st){
		case ST_INIT:
			// push initialization command
			// final command finished successfully -> ST_OP
			// if some commands failed return false
			if(!handle_init()){
				return false;
			}
			break;
		case ST_RST:
			// push reseting commands
			// final command finished successfully -> ST_OP
			if(!handle_rst()){
				return false;
			}
			break;
		case ST_OP:
			// push data transfer command with message from input channel
			// and output recieved data to output channel
			if(!handle_op()){
				return false;
			}
			break;
		case ST_DBG:
			// manually push a command to the queue
			if(!handle_dbg()){
				return false;
			}
			break;
		case ST_TEST:
			// maybe it invokes the TS2 mode
			if(!handle_test()){
				return false;
			}
			break;
		}

		// pop a command from queue and set to write buffer
		if(m_cur_cmd == NUL && m_cmd_queue.size() != 0){
			set_cmd();

			int wlen = write_serial(m_hcom, m_wbuf, m_wbuf_len);
			if(wlen){
				m_tcmd = m_cur_time;
				m_num_retry = 0;
				cout << "Write:";
				cout.write(m_wbuf, wlen);
			}
			if(m_wbuf_len != wlen){
				cerr << "Write operation failed." << endl;
			}
		}

		// handling recieved message. this code is temporal and simply dumping to stdout. 
		if(m_rcv_len && m_st != ST_OP){
			cout << "Recieve: ";
			cout.write(m_rcv_msg, m_rcv_len);
		}

		// handling processed command 
		if(m_cur_cmd != NUL){
			if(m_cmd_stat & EOC){
				cout << "Cmd " << m_cmd_str[m_cur_cmd] << " done." << endl;
				if(m_cmd_stat & N0 || m_cmd_stat & N1 || m_cmd_stat & N2 || m_cmd_stat & N3){
					// if the communication failed repete until counting m_num_max_retry
					if(m_num_retry < m_num_max_retry){
						int wlen = write_serial(m_hcom, m_wbuf, m_wbuf_len);
						if(wlen){
							m_tcmd = m_cur_time;
							m_num_retry++;
							cout << "Retry " << m_num_retry << ":";
							cout.write(m_wbuf, wlen);
						}
					}else{
						cout << "Failed to send " << m_wbuf << endl;
						m_cur_cmd = NUL;
						m_cmd_stat = NRES;
					}
				}else{
					m_cur_cmd = NUL;
					m_cmd_stat = NRES;
					cout << "Total Tx:" << m_total_tx << " Rx:" << m_total_rx << endl;
				}
			}else if(m_rep){
				/* m_rep == 1 means data transmission commands do not return response */
				/* Simply finish data transmissoin command with time interval.        */
				switch(m_cur_cmd){
				case TBN:
				case TBR:
				case TB2:
				case TXT:
				case TXR:
				case TX2:
					if(m_tcmd + m_tcmd_wait > m_cur_time)
						break;
					m_cmd_stat = NRES;
					m_cur_cmd = NUL;
					cout << "Total Tx:" << m_total_tx << " Rx:" << m_total_rx << endl;
					break;
				default:
					break;
				}
			}else if(m_tcmd + m_tcmd_out <= m_cur_time){ // command timeout
				// retry if the retry count is less than the maximum
				if(m_num_retry < m_num_max_retry){
					int wlen = write_serial(m_hcom, m_wbuf, m_wbuf_len);
					if(wlen){
						m_tcmd = m_cur_time;
						m_num_retry++;
						cout << "Retry " << m_num_retry << ":";
						cout.write(m_wbuf, wlen);
					}
				}else{
					cout << "Failed to send " << m_wbuf << endl;
					m_cur_cmd = NUL;
					m_cmd_stat = NRES;
				}
			}
		}
		// parsing read buffer. The read buffer is possible to contain both command response and recieved messages.
		m_rbuf_len = read_serial(m_hcom, m_rbuf, 512);
		if(m_rbuf_len){
			cout << "Read:";
			cout.write(m_rbuf, m_rbuf_len);
		}
		if(m_rbuf_len < 0){
			cerr << "Read operation failed." << endl;
		}else{
			if(!parse_rbuf()){
				cerr << "Failed to parse read buffer." << endl;
			}
		}
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