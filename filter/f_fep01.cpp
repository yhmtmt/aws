#include "stdafx.h"
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_fep01.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_fep01.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_fep01.h.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_fep01.h"

const char * f_fep01::m_cmd_str[32] = {
	"NUL", "ARG", "BAN", "BCL", "DAS",
	"DBM", "DVS", "FCN", "FRQ", "IDR",
	"IDW", "INI", "PAS", "POF", "PON", 
	"PTE", "PTN", "PTS", "ROF", "RON",
	"REG", "RID", "RST", "TBN", "TBR",
	"TB2", "TID", "TS2", "TXT", "TXR",
	"TX2", "VER"
};

const char * f_fep01::m_rec_str[6] = {
	"RBN" /* binary */, "RBR" /* binary 1 stage repeat */, "RB2" /* binary 2 stage repeat */,
	"RXT" /* text */, "RXR" /* text 1 stage repeat */, "RX2" /* text 2 stage repeat */
};

const char * f_fep01::m_st_str[ST_TEST+1] = {
	"init", "rst", "op", "dbg", "test"
};

const char * f_fep01::m_sst_str[SST_PROC+1] = {
	"cmd", "proc"
};

const char * f_fep01::m_cm_str[CM_P2P+1] = {
	"p2p"
};

f_fep01::f_fep01(const char * name):f_base(name), m_pin(NULL), m_pout(NULL),
	m_port(0), m_br(9600), m_len_pkt(128), m_max_queue(10), m_cm(CM_P2P), m_addr_p2p(0), m_hcom(NULL_SERIAL),
	m_addr(0x00), m_addr_group(0xF0), 
	m_addr_dst(0x00), m_header_less(0), m_scramble_0(0xFF), m_scramble_1(0xFF), m_num_freqs(0x03), m_freq0(0x18), 
	m_freq1(0x2A), m_freq2(0x3C), m_ant(0), m_div(1), m_num_reps(0x0A), m_th_roam(0x50), m_rep_power(0), 
	m_rep_err(0), m_rep_suc(0), m_rep(0), m_tint_cmd(0x00), m_fband(0), m_tbclr(0x64), m_fwait(4), m_chk_group(1), 
	m_int_bcn(0), m_fbcn(0), m_bcn(0), m_tlp_wait_ex(0), m_lp_wait(0), m_flw(0), m_tlp_wait(0x0F), m_crlf(0), 
	m_delim(1), m_tlp_slp(0x0F), m_to_hlss(0x01), m_addr_rep0(0xFF), m_addr_rep1(0xFF), m_rbuf_len(0),
	m_wbuf_len(0), 
	m_pbuf_tail(0), m_parse_cr(0), m_parse_lf(0), m_parse_count(0), m_cur_cmd(NUL), m_tcmd(0),
	m_tcmd_wait(100 * MSEC), 
	m_tcmd_out(5*SEC), m_num_retry(3), m_num_max_retry(3), m_cmd_stat(0), m_cur_rcv(RNUL), m_rcv_len(0), m_rcv_msg_head(0), m_rcv_pow(0),
	m_rcv_msg_tail(0),
	m_total_tx(0), m_total_rx(0)
{
	m_dname[0] = '\0';
	m_fname_txlog[0] = '\0';
	m_fname_rxlog[0] = '\0';

	register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
	register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
	register_fpar("br", &m_br, "Baud rate.");
	register_fpar("lpkt", &m_len_pkt, "Packet length.");

	register_fpar("cm", (int*)&m_cm, CM_P2P+1, m_cm_str, "Communication Mode.");
	register_fpar("addr_p2p", &m_addr_p2p, "Destination address for P2P communication.");
	register_fpar("tcw", &m_tcmd_wait, "Data transmission commands interval. This works only for no-response mode. (unit 100ns)");
	register_fpar("tco", &m_tcmd_out, "Data Transmission timeout. (unit 100ns)");
	register_fpar("nretry", &m_num_max_retry, "Maximum retry count.");
	register_fpar("addr", &m_addr, "own address (0 to 239)");
	register_fpar("addr_group", &m_addr_group, "group address (240 to 254)");
	register_fpar("addr_dst", &m_addr_dst, "destination address (0 to 255), header less mode only.");
	register_fpar("header_less", &m_header_less, "header less packet mode (0: no -> 0xF0 or 1: yes -> 0xFF)");
	register_fpar("scramble0", &m_scramble_0, "scramble code 0 (0 to 255)");
	register_fpar("scramble1", &m_scramble_1, "scramble code 1 (0 to 255)");
	register_fpar("nfreqs", &m_num_freqs, "number of frequencies used. (0 to 3)");
	register_fpar("freq0", &m_freq0, "0th frequency. for L-band 024 to 060, for H-band 062 to 077");
	register_fpar("freq1", &m_freq1, "1th frequency. for L-band 024 to 060, for H-band 062 to 077");
	register_fpar("freq2", &m_freq2, "2th frequency. for L-band 024 to 060, for H-band 062 to 077");
	register_fpar("ant", &m_ant, "antenna (0:A or 1:B)");
	register_fpar("div", &m_div, "diversity (0:use or 1:no use)");
	register_fpar("nreps", &m_num_reps, "number of re-transmission (0 to 255)");
	register_fpar("th_roam", &m_th_roam, "roaming threashold (70 to 100) [-dbm]");
	register_fpar("rep_power", &m_rep_power, "report power (0:no or 1:yes)");
	register_fpar("rep_err", &m_rep_err, "report n0 response (0:yes or 1: no)");
	register_fpar("rep_suc", &m_rep_suc, "report success response (0:P0, P1 or 1:P0)");
	register_fpar("rep", &m_rep, "report command response (0: yes or 1: no)");
	register_fpar("tint_cmd", &m_tint_cmd, "interval to recognize command in header less mode (0 to 255) [msec]");
	register_fpar("fband", &m_fband, "frequency band (0:L or 1:H)");
	register_fpar("tbclr", &m_tbclr, "interval to clear reciver buffer (1 to 255) [msec]");
	register_fpar("fwait", &m_fwait, "time for waiting requency response (0 to 7) corresponding to (120 to 50) [msec]");
	register_fpar("chk_group", &m_chk_group, "check group addres (0: no or 1: yes)");
	register_fpar("chk_addr", &m_chk_addr, "check destination address (0: no or 1: yes)");
	register_fpar("int_bcn", &m_int_bcn, "beacon interval (0 to 7) correspondint to (800 to 100) [msec]");
	register_fpar("bcn_freq", &m_fbcn, "beacon frequency (0: var or 1: fix)");
	register_fpar("bcn", &m_bcn, "beacon (0: no or 1: yes)");
	register_fpar("ser_dlen", &m_ser_dlen, "serial data length (0: 8bit or 1: 7bit)");
	register_fpar("ser_par", &m_ser_par, "serial com parity (0: no or 1: yes)");
	register_fpar("ser_sig", &m_ser_sig, "serial parity sign (0: even or 1: odd)");
	register_fpar("ser_br", &m_ser_br, "serial baud rate (0 to 3) corresponding to {9600, 19200, 384000, 115200}");
	register_fpar("ser_stp", &m_ser_stp, "serial com stop (0: 1bit or 1: 2bit)");
	register_fpar("tlp_wait_ex", &m_tlp_wait_ex, "low power waiting time extension (0 to 15) corresponding to (100 to 1500) [msec]");
	register_fpar("lp_wait", &m_lp_wait, "low power waiting mode (0: no or 1: yes)");
	register_fpar("flw", &m_flw, "RTS/CTS flow control (0: no or 1: yes)");
	register_fpar("tlp_wait", &m_tlp_wait, "low power waiting time (0 to 255) [100msec]");
	register_fpar("crlf", &m_crlf, "adding cr,lf to recieved data in header less mode (0: no or 1: yes) ");
	register_fpar("delim", &m_delim, "sending delimiter for header less mode (0: time out or 1: cr,lf)");
	register_fpar("tlp_slp", &m_tlp_slp, "sleep time for low power waiting mode (0 to 255) [100msec]");
	register_fpar("to_hlss", &m_to_hlss, "time interval to transmit in header less mode (1 to 255) [10msec]");
	register_fpar("addr_rep0", &m_addr_rep0, "address for repeater 0 in header less mode (0 to 255), 255 means no use.");
	register_fpar("addr_rep1", &m_addr_rep1, "address for repeater 1 in header less mode (0 to 255), 255 means no use.");
	register_fpar("rbuf", m_rbuf, 1024, "Read buffer.");
	register_fpar("wbuf", m_wbuf, 1024, "Write buffer.");

	// for debug mode
	register_fpar("push_cmd", &m_bpush_cmd, "Push command flag.");
	register_fpar("cmd", (int*)&m_cmd.type, VER+1, m_cmd_str, "Command to be queued.");
	register_fpar("iarg1", &m_cmd.iarg1, "Command argument 1 (an integer value)");
	register_fpar("iarg2", &m_cmd.iarg2, "Command argument 2 (an integer value)");
	register_fpar("carg1", m_cmd.carg1, 2, "Command argument 1 (a char value)");
	register_fpar("carg2", m_cmd.carg2, 2, "Command argument 2 (a char value)");
	register_fpar("msg", m_cmd.msg, 129, "Message to be sent");

	// level 2 state
	register_fpar("st", (int*)&m_st, ST_TEST+ 1, m_st_str, "Level 2 state.");
	register_fpar("sst", (int*)&m_sst, SST_PROC, m_sst_str, "Level 2 sub state.");

	m_rbuf[0] = m_wbuf[0] = '\0';
}

bool f_fep01::handle_init()
{
	// push reg command 
	switch(m_sst){
	case SST_CMD:
		m_cmd.type = ARG;
		m_cmd_queue.push_back(m_cmd);
		m_sst = SST_PROC;
		break;
	case SST_PROC:
		if(m_cur_cmd == ARG && m_cmd_stat & EOC){
			unpack_reg();
			m_st = ST_OP;
			m_sst = SST_CMD;
		}
		break;
	}
	return true;
}

bool f_fep01::handle_op()
{
	if(m_cmd_queue.size() < m_max_queue){
		int len = m_pin->read(m_cmd.msg, m_len_pkt);
		if(len){
			// push one packet to the queue from input channel
			m_cmd.msg[len] = '\0';
			switch(m_cm){
			case CM_P2P:
				m_cmd.type = TBN;
				m_cmd.iarg1 = (int) m_addr_p2p;
				m_cmd.iarg2 = len;
			}

			m_cmd_queue.push_back(m_cmd);
		}
	}

	// if there exists recieved data, push to the output channel
	if(m_rcv_msg_head < m_rcv_msg_tail){
		// push one recieved data to output channel
		m_rcv_msg_head += m_pout->write(m_rcv_msg + m_rcv_msg_head, (int) (m_rcv_msg_tail - m_rcv_msg_head));
		if(m_rcv_msg_head == m_rcv_msg_tail){
			m_rcv_msg_head = 0;
			m_rcv_msg_tail = 0;
			m_cur_rcv = RNUL;
		}
	}

	return true;
}

bool f_fep01::handle_dbg()
{
	if(m_bpush_cmd){
		m_cmd_queue.push_back(m_cmd);
		m_cmd = s_cmd();
		m_bpush_cmd = false;
	}

	if(m_cmd_stat & EOC){
		cout << m_cmd_str[m_cur_cmd] << " ";
		if(P0 & m_cmd_stat) cout << "P0";
		if(P1 & m_cmd_stat) cout << "P1";
		if(N0 & m_cmd_stat) cout << "N0";
		if(N1 & m_cmd_stat) cout << "N1";
		if(N2 & m_cmd_stat) cout << "N2";
		if(N3 & m_cmd_stat) cout << "N3";
		cout << endl;

		// dump command specific messages to the console
		switch(m_cur_cmd){
		case ARG:// dump register
			dump_reg();
			break;
		case BAN:
			cout << "fband:" << (m_fband ? "H" : "L") << endl;
			break;
		case BCL:
			break;
		case DAS:
			cout << "addr:" << m_addr << endl;
			break;
		case DBM:
			cout << "dbm:-" << m_dbm << endl;
			break;
		case DVS:
			cout << "ant:" << (m_ant ? "B" : "A") << endl;
			cout << "div:" << (m_div ? "yes" : "no") << endl;
			break;
		case FCN:
			cout << "nfreqs:" << m_num_freqs << endl;
			break;
		case FRQ:
			switch(m_cmd_arg1){
			case 1:
				cout << "frq0:" << m_freq0 << endl;
				break;
			case 2:
				cout << "frq1:" << m_freq1 << endl;
				break;
			case 3:
				cout << "frq2:" << m_freq2 << endl;
				break;
			}
			break;
		case IDR:
			printf("scid0: x%02x\n", m_scramble_0);
			printf("scid1: x%02x\n", m_scramble_1);
			break;
		case IDW:
		case INI:
			break;
		case PAS:
			cout << "addr_rep0:" << m_addr_rep0 << endl;
			cout << "addr_rep1:" << m_addr_rep1 << endl;
			break;
		case POF:
		case PON:
			break;
		case PTE:
			cout << "tlp_wait_ex:" << m_tlp_wait_ex << endl;
			break;
		case PTN:
			cout << "tlp_wait:" << m_tlp_wait << endl;
			break;
		case PTS:
			cout << "tlp_slp:" << m_tlp_slp << endl;
			break;
		case ROF:
		case RON:
			break;
		case REG:
			cout << "REG" << m_cmd_arg1 << ":" << m_reg[m_cmd_arg1] << endl;
			break;
		case RID:
			cout << "rid:" << m_rid << endl;
			break;
		case RST:
		case TBN:
		case TBR:
		case TB2:
			break;
		case TID:
			cout << "tid:" << m_tid << endl;
			break;
		case TS2:
		case TXT:
		case TXR:
		case TX2:
			break;
		case VER:
			cout << "ver:" << m_ver << "." << m_sub_ver << endl;
			break;
		}
	}
	return true;
}

bool f_fep01::handle_test()
{
	m_cmd.type = TS2;
	m_cmd_queue.push_back(m_cmd);
	return true;
}

bool f_fep01::handle_rst()
{
	switch(m_sst){
	case SST_CMD:
		pack_reg();
		m_cmd = s_cmd();
		m_cmd.type = REG;
		for(int i = 0; i < 29; i++){
			if(i == 20) // Ignore change in serial setting. 
				continue;
			m_cmd.iarg1 = i;
			m_cmd.iarg2 = m_reg[i];
			m_cmd_queue.push_back(m_cmd);
		}
		m_cmd = s_cmd();
		m_cmd.type = RST;
		m_cmd_queue.push_back(m_cmd);
		break;
	case SST_PROC:
		if(m_cur_cmd == RST && m_cmd_stat & EOC){
			m_st = ST_OP;
			m_sst = SST_CMD;
		}
	}
	return true;
}

void f_fep01::dump_reg()
{
	for(int ireg = 0; ireg < 29; ireg++){
		printf("REG%02d: x%02x\n", ireg, m_reg[ireg]);
	}
}

void f_fep01::unpack_reg()
{
	m_addr = m_reg[0];
	m_addr_group = m_reg[1];
	m_addr_dst = m_reg[2];
	m_header_less = (m_reg[3] ==0xF0 ? 0 : 1);
	m_scramble_0 = m_reg[4];
	m_scramble_1 = m_reg[5];
	m_num_freqs = m_reg[6];
	m_freq0 = m_reg[7];
	m_freq1 = m_reg[8];
	m_freq2 = m_reg[9];
	m_ant = (m_reg[10] & 0x02) >> 1;
	m_div = (m_reg[10] & 0x01);
	m_num_reps = m_reg[11];
	m_th_roam = m_reg[12];
	m_rep_power = (m_reg[13] & 0x80) >> 7;
	m_rep_err = (m_reg[13] & 0x4) >> 2;
	m_rep_suc = (m_reg[13] & 0x2) >> 1;
	m_rep = (m_reg[13] & 0x1);
	m_tint_cmd =m_reg[15];
	m_fband = m_reg[16];
	m_tbclr = m_reg[17];
	m_fwait = (m_reg[18] & 0xE0) >> 5;
	m_chk_group = (m_reg[18] & 0x02) >> 1;
	m_chk_addr = (m_reg[18] & 0x01);
	m_int_bcn = (m_reg[19] & 0xE0) >> 5;
	m_fbcn = (m_reg[19] & 0x03) >> 2;
	m_bcn = (m_reg[19] & 0x02) >> 1;
	m_ser_dlen = (m_reg[20] & 0x80) >> 7;
	m_ser_par = (m_reg[20] & 0x40) >> 6;
	m_ser_sig = (m_reg[20] & 0x20) >> 5;
	m_ser_br = (m_reg[20] & 0x03);
	switch(m_ser_br){
	case 0:
		m_br = 9600;
		break;
	case 1:	
		m_br = 19200;
		break;
	case 2:
		m_br = 38400;
		break;
	case 3:
		m_br = 115200;
		break;
	}
	m_ser_stp = (m_reg[20] & 0x08) >> 3;
	m_tlp_wait_ex = (m_reg[21] & 0xF0) >> 4;
	m_lp_wait = (m_reg[21] & 0x04) >> 2;
	m_flw = (m_reg[21] & 0x02) >> 1;
	m_tlp_wait = m_reg[22];
	m_crlf = (m_reg[23] & 0x10) >> 4;
	m_delim = (m_reg[24] & 0x40) >> 6;
	m_tlp_slp = m_reg[25];
	m_to_hlss = m_reg[26];
	m_addr_rep0 = m_reg[27];
	m_addr_rep1 = m_reg[28];
}

bool f_fep01::pack_reg()
{
	if(m_addr > 239){
		return false;
	}
	m_reg[0] = m_addr;

	if(m_addr < 240){
		return false;
	}
	m_reg[1] = m_addr_group;

	m_reg[2] = m_addr_dst;
	m_reg[3] = (m_header_less ? 0xFF : 0xF0);
	m_reg[4] = m_scramble_0;
	m_reg[5] = m_scramble_1;
	if(m_num_freqs < 0 || m_num_freqs >3){
		return false;
	}
	m_reg[6] = m_num_freqs;

	if(m_freq0 < 0x18 || (m_freq0 > 0x3C && m_freq0 < 0x3E) || m_freq0 > 0x45)
		return false;
	m_reg[7] = m_freq0;

	if(m_freq1 < 0x18 || (m_freq1 > 0x3C && m_freq1 < 0x3E) || m_freq1 > 0x45)
		return false;
	m_reg[8] = m_freq1;

	if(m_freq2 < 0x18 || (m_freq2 > 0x3C && m_freq2 < 0x3E) || m_freq2 > 0x45)
		return false;
	m_reg[9] = m_freq2;
	m_reg[10] = (m_ant << 1) | (m_div);
	m_reg[11] = m_num_reps;
	if(m_th_roam < 70 && m_th_roam > 100)
		return false;
	m_reg[12] = m_th_roam;
	m_reg[13] = (m_rep_power << 7) | (m_rep_err << 2) | (m_rep_suc << 1) | m_rep;
	m_reg[15] = m_tint_cmd;
	m_reg[16] = m_fband;

	if(m_tbclr == 0)
		return false;
	m_reg[17] = m_tbclr;

	m_reg[18] = (m_fwait << 5) | (m_chk_group << 1) | m_chk_addr;
	m_reg[19] = (m_int_bcn << 5) | (m_fbcn << 2) | (m_bcn << 1);
	m_reg[20] = (m_ser_dlen << 7) | (m_ser_par << 6) | (m_ser_sig << 5) | (m_ser_stp << 3) | (m_ser_br);
	m_reg[21] = (m_tlp_wait_ex << 4) | (m_lp_wait << 2) | (m_flw << 1);
	m_reg[22] = m_tlp_wait;
	m_reg[23] = m_crlf << 4;
	m_reg[24] = m_delim << 6;
	m_reg[25] = m_tlp_slp;
	if(m_to_hlss == 0)
		return false;

	m_reg[26] = m_to_hlss;

	if(m_addr_rep0 > 239)
		return false;
	m_reg[27] = m_addr_rep0;

	if(m_addr_rep1 > 239)
		return false;
	m_reg[28] = m_addr_rep1;

	return true;
}

bool f_fep01::parse_rbuf()
{
	char * rbuf = m_rbuf;
	char * pbuf;
	for(int i = 0; i < m_rbuf_len && m_pbuf_tail < 512; i++, rbuf++){
		pbuf = m_pbuf + m_pbuf_tail;
		*pbuf = *rbuf;
		m_pbuf_tail++;
		if(*pbuf == 0x0D){
			m_parse_cr = 1;
		}else if(*pbuf == 0x0A){
			m_parse_lf = 1;
		}

		try{
			// parsing recieved message
			if(m_pbuf_tail == 3){ // depending on the first three characters, recieved message parser is activated.
				if(parse_message_type())
					continue; // entering message recieve mode
			}else if(m_cur_rcv != RNUL){
				if(!m_rcv_header){
					m_parse_count--; // counting header parts
					if(m_parse_count == 0){
						if(parse_message_header())
							continue;
					}
				}else{
					if(m_msg_bin){
						m_parse_count--; // counting message parts, power and crlf
						/*if binary we cant use cr lf as the delimiter. we count the packet length */
						if(m_parse_count == 0){ 
							if(parse_message()){
								continue;
							}else{
								throw c_parse_message_exception(m_cur_rcv, m_cmd_stat, __LINE__);
							}
						}
					}else{
						/* if not binary, cr lf is the delimiter */
						if (m_parse_cr && m_parse_lf) {
							if(parse_message()){
								continue;
							}else{
								throw c_parse_message_exception(m_cur_rcv, m_cmd_stat, __LINE__);
							}
						}
					}
				}
				continue;
			}

			// parsing command response
			if(m_parse_cr && m_parse_lf){
				if(m_cur_cmd != NUL && !(m_cmd_stat & EOC)){
					// try to process as a command ressponse
					if(m_pbuf_tail == 4){ // P0<cr><lf> etc.
						if(parse_response()){
							cout << "Response: " << m_pbuf << endl;
							pbuf = m_pbuf;
							m_pbuf_tail = 0;
							m_parse_cr = m_parse_lf = 0;
							continue;
						}
					}
					// try to process as a command's return value
					if(parse_response_value()){
						m_pbuf[m_pbuf_tail] = '\0';
						cout << "Response: " << m_pbuf << endl;
						pbuf = m_pbuf;
						m_pbuf_tail = 0;
						m_parse_cr = m_parse_lf = 0;
						continue;
					}
				}
				throw c_parse_response_exception(NUL, m_cmd_stat, __LINE__);
			}
		}catch(const c_parse_response_exception & e){
			m_pbuf[m_pbuf_tail] = '\0';
			cerr << "Errorin parsing read buffer. Status: " << m_cmd_str[e.cmd] << " line = " << e.line << "." << endl;
			cerr << "Read Buffer: ";
			cerr.write(m_pbuf, m_pbuf_tail);
			cerr << "State P0:P1:N0:N1:N3:" << (P0 & e.stat ? 1 : 0) << ":" << (P1 & e.stat ? 1 : 0) << 
				":" << (N0 & e.stat ? 1 : 0) << ":" << (N1 & e.stat ? 1 : 0) << ":" << (N3 & e.stat ? 1 : 0) << endl;
			init_parser();
			return false;
		}catch(const c_parse_message_exception & e){
			cerr << "Error in parsing read buffer. Status: " << m_rec_str[e.type] << " line = " << e.line << "." << endl;
			cerr << "Message Buffer Usage: " << m_rcv_msg_head << " to " << m_rcv_msg_tail << endl;
			cerr << "Message Length Recieved: " << m_rcv_msg << endl;
			cerr << "Read Buffer: ";
			cerr.write(m_pbuf, m_pbuf_tail);
			cerr << "Message Buffer:";
			cerr.write(m_rcv_msg + m_rcv_msg_head, m_rcv_msg_tail - m_rcv_msg_head);
			init_parser();
		}
	}

	if(m_pbuf_tail == 512){
		// discard all parse state
		cerr << "Buffer over flow." << endl;
		init_parser();
		return false;
	}
	return true;
}

void f_fep01::init_parser()
{
	m_pbuf_tail = 0;
	m_parse_cr = m_parse_lf = 0;
	m_parse_count = 0;
	m_cur_cmd = NUL;
	m_cmd_arg1 = m_cmd_arg2 = 0;
	m_cmd_stat = 0;
	m_cur_rcv = RNUL;
	m_rcv_header = false;
	m_rcv_len = 0;
	m_rcv_msg[0] = '\0';
	m_rcv_msg_tail = 0;
	m_ts2_mode = false;
}

bool f_fep01::parse_response_value()
{
	// value? process and clear m_pbuf_tail zero
	bool is_proc = true;
	switch(m_cur_cmd){
	case ARG: // REGxx:yyH
		if(m_pbuf_tail == 11 && m_pbuf[0] == 'R' && m_pbuf[1] == 'E' 
			&& m_pbuf[2] == 'G' && m_pbuf[5] == ':' && m_pbuf[8] == 'H'){
				unsigned char ireg = str2DigitDecimal(m_pbuf + 3);
				unsigned char reg = str2DigitHex(m_pbuf + 6);
				m_reg[ireg] = reg;
				if(ireg == 28){ // end of command 
					m_cmd_stat |= EOC;
				}
		}else{
			is_proc = false;
		}
		break;
	case BAN: // L or H
		if(m_pbuf_tail != 3){
			is_proc = false;
			break;
		}
		switch(m_pbuf[0]){
		case 'L':
			m_fband = 0;
			m_cmd_stat |= EOC;
			break;
		case 'H':
			m_fband = 1;
			m_cmd_stat |= EOC;
			break;
		default:
			is_proc = false;
		}
		break;
	case DAS:
		if(m_pbuf_tail != 5){
			is_proc = false;
		}else{
			int das = str3DigitDecimal(m_pbuf);
			if(das > 0 && das < 256){
				m_addr = das;
				m_cmd_stat |= EOC;
			}else
				is_proc = false;
		}
		break;
	case DBM:
		if(m_pbuf_tail != 9 || m_pbuf[0] != '-' ||
			m_pbuf[4] != 'd' || m_pbuf[5] != 'B' || m_pbuf[6] != 'm'){ 
			is_proc = false;
		}else{ // form of "-xxxdBm"
			int dbm = str3DigitDecimal(m_pbuf + 1);
			if(dbm > 0 && dbm < 256){
				m_dbm = dbm;
				m_cmd_stat |= EOC;
			}else
				is_proc = false;
		}
		break;
	case DVS:
		if(m_pbuf_tail != 3){
			is_proc = false;
			break;
		}
		switch(m_pbuf[0]){
		case 'A':
			m_ant = 0;
			m_div = 0;
			m_cmd_stat |= EOC;
			break;
		case 'B':
			m_ant = 1;
			m_div = 0;
			m_cmd_stat |= EOC;
			break;
		case 'D':
			m_div = 1;
			m_cmd_stat |= EOC;
			break;
		default:
			is_proc = false;
		}
		break;
	case FCN:
		if(m_pbuf_tail != 3){
			is_proc = false;
		}else{
			int nfreqs = m_pbuf[0] - '0';
			if(nfreqs > 0 && nfreqs < 4){
				m_num_freqs = (unsigned char) nfreqs;
				m_cmd_stat |= EOC;
			}else
				is_proc = false;
		}
		break;
	case FRQ:
		if(m_pbuf_tail != 4){
			is_proc = false;
		}else{
			int freq = str2DigitDecimal(m_pbuf);
			if(freq < 24 || (freq > 60 && freq < 62) || freq > 77){
				is_proc = false;
				break;
			}

			switch(m_cmd_arg1){
			case 1:
				m_freq0 = (unsigned char) freq;
				m_cmd_stat |= EOC;
				break;
			case 2:
				m_freq1 = (unsigned char) freq;
				m_cmd_stat |= EOC;
				break;
			case 3:
				m_freq1 = (unsigned char) freq;
				m_cmd_stat |= EOC;
			default:
				is_proc = false;
			}
		}
		break;
	case IDR:
		if(m_pbuf_tail == 7 && m_pbuf[4] == 'H'){
			int scid0, scid1;
			scid0 = str2DigitHex(m_pbuf);
			scid1 = str2DigitHex(m_pbuf+2);
			if(scid0 < 0 || scid0 > 255){
				is_proc = false;
				break;
			}
			if(scid1 < 0 || scid1 > 255){
				is_proc = false;
				break;
			}
			m_scramble_0 = scid0;
			m_scramble_1 = scid1;
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
		}
		break;
	case IDW:
		break;
	case INI:
		break;
	case PAS:
		if(m_pbuf_tail == 9 && m_pbuf[3] == ':'){
			int rep0 = str3DigitDecimal(m_pbuf);
			int rep1 = str3DigitDecimal(m_pbuf+4);
			if(rep0 < 0 || rep0 > 255){
				is_proc = false;
				break;
			}
			if(rep1 < 0 || rep1 > 255){
				is_proc = false;
				break;
			}
			m_addr_rep0 = (unsigned char) rep0;
			m_addr_rep1 = (unsigned char) rep1;
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
		}
		break;
	case POF:
		break;
	case PON:
		break;
	case PTE:
		if(m_pbuf_tail != 5){
			is_proc = false;
		}else{
			int pte = str3DigitDecimal(m_pbuf);
			if(pte < 0 || pte > 15){
				is_proc = false;
				break;
			}
			m_tlp_wait_ex = (unsigned char) pte;
			m_cmd_stat |= EOC;
		}
	case PTN:
		if(m_pbuf_tail != 5){
			is_proc = false;
		}else{
			int ptn = str3DigitDecimal(m_pbuf);
			if(ptn < 0 || ptn > 255){
				is_proc = false;
				break;
			}
			m_tlp_wait = (unsigned char) ptn;
			m_cmd_stat |= EOC;
		}
		break;
	case PTS:
		if(m_pbuf_tail != 5){
			is_proc = false;
		}else{
			int pts = str3DigitDecimal(m_pbuf);
			if(pts < 0 || pts > 255){
				is_proc = false;
				break;
			}
			m_tlp_slp = (unsigned char) pts;
			m_cmd_stat |= EOC;
		}
		break;
	case ROF:
		break;
	case RON:
		break;
	case REG:
		if(m_pbuf_tail == 5 && m_pbuf[2] == 'H'){
			int reg = str2DigitHex(m_pbuf);
			if(m_cmd_arg1 < 0 || m_cmd_arg1 > 28){
				is_proc = false;
				break;
			}
			m_reg[m_cmd_arg1] = (unsigned char) reg;
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
		}
		break;
	case RID:
		if(m_pbuf_tail == 14){
			m_pbuf[m_pbuf_tail - 2] = '\0';
			m_rid = (unsigned int) atoi(m_pbuf);
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
		}
		break;
	case RST:
		break;
	case TBN:
	case TBR:
	case TB2:
		break;
	case TID:
		if(m_pbuf_tail == 14){
			m_pbuf[m_pbuf_tail - 2] = '\0';
			m_tid = (unsigned int) atoi(m_pbuf);
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
			break;
		}
		break;
	case TS2:
		if(m_pbuf_tail == 22){
			m_pbuf[m_pbuf_tail] = '\0';
			if(m_cmd_stat & P0 && m_flog_ts2.is_open())
				m_flog_ts2 << m_time_str << " " << m_pbuf << endl;
		}else{
			is_proc = false;
		}
		break;
	case TXT:
	case TXR:
	case TX2:
		break;
	case VER:
		if(m_pbuf_tail == 7 && m_pbuf[1] == '.'){
			m_ver = m_pbuf[0] - '0';
			m_sub_ver = str3DigitDecimal(m_pbuf + 2);
			m_cmd_stat |= EOC;
		}else{
			is_proc = false;
		}
		break;
	default:
		is_proc = false;
	}	
	return is_proc;
}

bool f_fep01::parse_response()
{
	bool is_proc = true;
	// P/N response? process and clear m_pbuf_tail zero
	switch(m_pbuf[0]){
	case 'P':
		switch(m_pbuf[1]){
		case '0':
			m_cmd_stat |= P0;
			break;
		case '1':
			m_cmd_stat |= P1;
			break;
		default:
			is_proc = false;
		}
		break;
	case 'N':
		switch(m_pbuf[1]){
		case '0':
			m_cmd_stat |= N0;
			break;
		case '1':
			m_cmd_stat |= N1;
			break;
		case '3':
			m_cmd_stat |= N3;
			break;
		default:
			is_proc = false;
		}
		break;
	default:
		is_proc = false;
	}

	if(is_proc){
		switch(m_cur_cmd){
		case ARG: // N0
			if(m_cmd_stat & N0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case BAN: // P0 or N0 
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case BCL: // P0 or N0 
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case DAS: // P0 or N0 
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case DBM: // N0
			if(m_cmd_stat & N0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case DVS:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case FCN:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case FRQ:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case IDR:
			break;
		case IDW:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case INI:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case PAS:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case POF:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case PON:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case PTE:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case PTN:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case PTS:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case ROF:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case RON:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case REG:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case RID:
			if(m_cmd_stat & N0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case RST:
			if(m_cmd_stat & N0 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
				if(m_ts2_mode && (m_cmd_stat & P0)){
					m_flog_ts2.close();
				}
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case TBN:
		case TBR:
		case TB2:
			if(m_cmd_stat & N0 | m_cmd_stat & N1 | m_cmd_stat & N3 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
				if(m_cmd_stat & P0)
					m_total_tx += m_len_tx;
			}else if(m_cmd_stat & P1){
				// sending 
				break;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case TID:
			if(m_cmd_stat & N0){
				m_cmd_stat |= EOC;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case TS2:
			if(m_cmd_stat & N0){
				m_cmd_stat |= EOC;
			}else if(m_cmd_stat & P0){
				// if TS2 succeeded, log file is opened.
				snprintf(m_pbuf, "%s_ts2.log", m_name);
				m_flog_ts2.open(m_pbuf, ios_base::app);
				m_ts2_mode = true;
				m_total_tx += m_len_tx;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case TXT:
		case TXR:
		case TX2:
			if(m_cmd_stat & N0 | m_cmd_stat & N1 | m_cmd_stat & N3 | m_cmd_stat & P0){
				m_cmd_stat |= EOC;
			}else if(m_cmd_stat & P1){
				// sending 
				break;
			}else{
				throw c_parse_response_exception(m_cur_cmd, m_cmd_stat, __LINE__);
			}
			break;
		case VER:
		default:
			is_proc = false;
		}
	}
	return is_proc;
}

bool f_fep01::parse_message_type()
{
	if(m_pbuf[0] != 'R'){ 
		// This is not the received data.
		return false;
	}

	e_msg_rcv ercv = RNUL;
	for(int i = 0; i < 6; i++){
		if(m_pbuf[1] == m_rec_str[i][1] && m_pbuf[2] == m_rec_str[i][2]){
			ercv = (e_msg_rcv) i;
		}
	}
	m_cur_rcv = ercv;

	if(ercv == RNUL)
		return false;
	switch(m_cur_rcv){
	case RBN:
		m_msg_bin = true;
		m_parse_count = 6;
		break;
	case RBR:
		m_msg_bin = true;
		m_parse_count = 9;
		break;
	case RB2:
		m_msg_bin = true;
		m_parse_count = 12;
		break;
	case RXT:
		m_msg_bin = false;
		m_parse_count = 3;
		break;
	case RXR:
		m_msg_bin = false;
		m_parse_count = 6;
		break;
	case RX2:
		m_msg_bin = false;
		m_parse_count = 9;
		break;
	default:
		// this is not the recieved message
		return false;
	}

	return true;
}

bool f_fep01::parse_message_header()
{
	switch(m_cur_rcv){
	case RBN: // src, length 
		m_rcv_src = str3DigitDecimal(&m_pbuf[3]);
		m_rcv_len = str3DigitDecimal(&m_pbuf[6]);
		m_parse_count = m_rcv_len + (m_rep_power ? 3 : 0) /* power report */ + 2 /* cr lf */;
		break;
	case RBR: // rep, src, length
		m_rcv_rep0 = str3DigitDecimal(&m_pbuf[3]);
		m_rcv_src = str3DigitDecimal(&m_pbuf[6]);
		m_rcv_len = str3DigitDecimal(&m_pbuf[9]);
		m_parse_count = m_rcv_len + (m_rep_power ? 3 : 0) /* power report */ + 2 /* cr lf */;
		break;
	case RB2: // rep0, rep1, src, length
		m_rcv_rep0 = str3DigitDecimal(&m_pbuf[3]);
		m_rcv_rep1 = str3DigitDecimal(&m_pbuf[6]);
		m_rcv_src = str3DigitDecimal(&m_pbuf[9]);
		m_rcv_len = str3DigitDecimal(&m_pbuf[12]);
		m_parse_count = m_rcv_len + (m_rep_power ? 3 : 0) /* power report */ + 2 /* cr lf */;
		break;
	case RXT: // src
		m_rcv_src = str3DigitDecimal(&m_pbuf[3]);
		break;
	case RXR: // rep0, src
		m_rcv_rep0 = str3DigitDecimal(&m_pbuf[3]);
		m_rcv_src = str3DigitDecimal(&m_pbuf[6]);
		break;
	case RX2: // rep0, rep1, src
		m_rcv_rep0 = str3DigitDecimal(&m_pbuf[3]);
		m_rcv_rep1 = str3DigitDecimal(&m_pbuf[6]);
		m_rcv_src = str3DigitDecimal(&m_pbuf[9]);
		break;
	default:
		// this is not the recieved message
		return false;
	}
	m_rcv_header = true;
	return true;
}

bool f_fep01::parse_message()
{
	char * ptr, * ptr_end;
	ptr_end = &m_pbuf[m_pbuf_tail];

	switch(m_cur_rcv){
	case RBN: // src, length 
		ptr = &m_pbuf[9];
		ptr_end = &m_pbuf[9] + m_rcv_len;
		break;
	case RBR: // src, rep, length
		ptr = &m_pbuf[12];
		ptr_end = &m_pbuf[12] + m_rcv_len;
		break;
	case RB2: // src, rep0, rep1, length
		ptr = &m_pbuf[15];
		ptr_end = &m_pbuf[15] + m_rcv_len;
		break;
	case RXT: // src
		ptr = &m_pbuf[6];
		ptr_end = m_pbuf + m_pbuf_tail - 2 - (m_rep_power ? 3 : 0);
		break;
	case RXR: // src, rep0
		ptr = &m_pbuf[9];
		ptr_end = m_pbuf + m_pbuf_tail - 2 - (m_rep_power ? 3 : 0);
		break;
	case RX2: // src, rep0, rep1
		ptr = &m_pbuf[12];
		ptr_end = m_pbuf + m_pbuf_tail - 2 - (m_rep_power ? 3 : 0);
		break;
	default:
		// this is not the recieved message
		return false;
	}

	char * pmsg;
	for(pmsg = m_rcv_msg + m_rcv_msg_tail, m_rcv_len = 0; ptr != ptr_end; pmsg++, ptr++, m_rcv_len++){
		if(m_rcv_msg_tail + m_rcv_len >= 1023){
			cerr << "Overflow in buffer for recieved message" << endl;
			return false;
		}
		*pmsg = *ptr;
	}
	*pmsg = '\0';

	if(m_rep_power){
		m_rcv_pow = str3DigitDecimal(ptr);
	}

	log_rx();
	cout << "Recieve: ";
	cout.write(m_rcv_msg + m_rcv_msg_tail, m_rcv_len);
	if(m_rep_power){
		cout << "Power:" << m_rcv_pow << endl;
	}

	m_rcv_msg_tail += m_rcv_len;
	m_msg_bin = false;
	m_rcv_header = false;
	m_cur_rcv = RNUL;
	m_parse_cr = m_parse_lf = 0;
	return true;
}


bool f_fep01::set_cmd()
{
	list<s_cmd>::iterator itr = m_cmd_queue.begin();

	m_cur_cmd = itr->type;
	m_cmd_arg1 = itr->iarg1;
	m_cmd_arg2 = itr->iarg2;
	m_cmd_stat = 0;

	switch(itr->type){
	case ARG:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case BAN:
		if(itr->carg1[0] == 'L' || itr->carg1[0] == 'H'){
			snprintf(m_wbuf, 512, "@%s%s\r\n", m_cmd_str[itr->type], itr->carg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case BCL:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case DAS:
		if(itr->iarg1 != -1){
			snprintf(m_wbuf, 512, "@%s%03d\r\n", m_cmd_str[itr->type], (int) itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case DBM:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case DVS:
		if(itr->carg1[0] == 'A' || itr->carg1[0] == 'B' || itr->carg1[0] == 'D'){
			snprintf(m_wbuf, 512, "@%s%s\r\n", m_cmd_str[itr->type], itr->carg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case FCN:
		if(itr->iarg1 < 4 && itr->iarg1 > 0){
			snprintf(m_wbuf, 512, "@%s%d\r\n", m_cmd_str[itr->type], itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case FRQ:
		if(itr->iarg1 < 4 && itr->iarg1 > 0){
			if((itr->iarg2 < 61 && itr->iarg2 > 23)  || 
			(itr->iarg2 < 78 && itr->iarg2 > 61)){
				snprintf(m_wbuf, 512, "@%s%d:%d\r\n", m_cmd_str[itr->type], itr->iarg1, itr->iarg2);
			}else{
				snprintf(m_wbuf, 512, "@%s%d\r\n", m_cmd_str[itr->type], itr->iarg1);
			}
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case IDR:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case IDW:
		if(itr->iarg1 <= 0x0000FFFF && itr->iarg1 >= 0){
			snprintf(m_wbuf, 512, "@%s%4d\r\n", m_cmd_str[itr->type], itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case INI:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case PAS:
		if(itr->iarg1 < 256 && itr->iarg1 >= 0 && itr->iarg2 < 256 && itr->iarg2 >= 0){
			snprintf(m_wbuf, 512, "@%s%03d:%03d\r\n", m_cmd_str[itr->type], itr->iarg1, itr->iarg2);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case POF:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case PON:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case PTE:
		if(itr->iarg1 >= 0 && itr->iarg1 < 16){
			snprintf(m_wbuf, 512, "@%s%03d\r\n", m_cmd_str[itr->type], itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case PTN:
		if(itr->iarg1 > 0 && itr->iarg1 < 256){
			snprintf(m_wbuf, 512, "@%s%03d\r\n", m_cmd_str[itr->type], itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case PTS:
		if(itr->iarg1 > 0 && itr->iarg1 < 256){
			snprintf(m_wbuf, 512, "@%s%03d\r\n", m_cmd_str[itr->type], itr->iarg1);
		}else{
			snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case ROF:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		break;
	case RON:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		break;
		m_wbuf_len = (int) strlen(m_wbuf);
	case REG:
		if(itr->iarg1 < 29 && itr->iarg1 >= 0){
			if(itr->iarg2 < 256 && itr->iarg2 >= 0){
				snprintf(m_wbuf, 512, "@%s%02d:%03d\r\n", m_cmd_str[itr->type], itr->iarg1, itr->iarg2);
			}else{
				snprintf(m_wbuf, 512, "@%s%02d\r\n", m_cmd_str[itr->type], itr->iarg1);
			}
		}
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case RID:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case RST:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	case TBN:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255 && itr->iarg2 > 0 && itr->iarg2){
			snprintf(m_wbuf, 512, "@%s%03d%03d", m_cmd_str[itr->type], itr->iarg1, itr->iarg2);
			m_wbuf_len = (int) strlen(m_wbuf);
			memcpy(m_wbuf + strlen(m_wbuf), itr->msg, itr->iarg2);
			m_wbuf_len += itr->iarg2;
			m_wbuf[m_wbuf_len] = '\r';
			m_wbuf[m_wbuf_len+1] = '\n';
			m_wbuf_len += 2;
			m_len_tx = itr->iarg2;
			log_tx((unsigned char) itr->iarg2, (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
		break;
	case TBR:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255 
			&& itr->iarg2 >= 0 && itr->iarg2 < 255 
			&& itr->iarg3 > 0 && itr->iarg3 <129){
			m_wbuf_len = (int) strlen(m_wbuf);
			snprintf(m_wbuf, 512, "@%s%03d%03d%03d", m_cmd_str[itr->type], itr->iarg1, itr->iarg2, itr->iarg3);
			memcpy(m_wbuf + strlen(m_wbuf), itr->msg, itr->iarg3);
			m_wbuf_len += itr->iarg3;
			m_wbuf[m_wbuf_len] = '\r';
			m_wbuf[m_wbuf_len+1] = '\n';
			m_wbuf_len += 2;
			m_len_tx = itr->iarg3;
			log_tx((unsigned char) itr->iarg3, (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
		break;
	case TB2:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255 
			&& itr->iarg2 >= 0 && itr->iarg2 < 255 
			&& itr->iarg3 >= 0 && itr->iarg3 < 255 
			&& itr->iarg4 > 0 && itr->iarg4 <129){
			m_wbuf_len = (int) strlen(m_wbuf);
			snprintf(m_wbuf, 512, "@%s%03d%03d%03d%03d", m_cmd_str[itr->type], itr->iarg1, itr->iarg2, itr->iarg3, itr->iarg4);
			memcpy(m_wbuf + strlen(m_wbuf), itr->msg, itr->iarg4);
			m_wbuf_len += itr->iarg4;
			m_wbuf[m_wbuf_len] = '\r';
			m_wbuf[m_wbuf_len+1] = '\n';
			m_wbuf_len += 2;
			m_len_tx = itr->iarg4;
			log_tx((unsigned char) itr->iarg4, (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
		break;
	case TID:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		break;
	case TS2:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		break;
	case TXT:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255){
			snprintf(m_wbuf, 512, "@%s%03d%s\r\n", m_cmd_str[itr->type], itr->iarg1, itr->msg);
			m_wbuf_len = (int) strlen(m_wbuf);
			m_len_tx = (int) strlen(itr->msg);
			log_tx((unsigned char) strlen(itr->msg), (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
		break;
	case TXR:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255 
			&& itr->iarg2 >= 0 && itr->iarg2 < 255){
			snprintf(m_wbuf, 512, "@%s%03d%03d%s\r\n", m_cmd_str[itr->type], itr->iarg1, itr->iarg2, itr->msg);
			m_wbuf_len = (int) strlen(m_wbuf);
			m_len_tx = (int) strlen(itr->msg);
			log_tx((unsigned char) strlen(itr->msg), (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
	case TX2:
		if(itr->iarg1 >= 0 && itr->iarg1 < 255 
			&& itr->iarg2 >= 0 && itr->iarg2 < 255 
			&& itr->iarg3 >= 0 && itr->iarg3 < 255){
			snprintf(m_wbuf, 512, "@%s%03d%03d%03d%s\r\n", m_cmd_str[itr->type], itr->iarg1, itr->iarg2
				,itr->iarg3, itr->msg);
			m_wbuf_len = (int) strlen(m_wbuf);
			m_len_tx = (int) strlen(itr->msg);
			log_tx((unsigned char) strlen(itr->msg), (const char*) itr->msg);
			if(m_rep != 0){
				m_total_tx += m_len_tx;
			}
		}
		break;
	case VER:
		snprintf(m_wbuf, 512, "@%s\r\n", m_cmd_str[itr->type]);
		m_wbuf_len = (int) strlen(m_wbuf);
		break;
	}

	m_cmd_queue.pop_front();
	return true;
}

