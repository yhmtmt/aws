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

f_fep01::f_fep01(const char * name):f_base(name), 
	m_port(0), m_br(9600), m_hcom(NULL_SERIAL), m_addr(0x00), m_addr_group(0xF0), m_addr_dst(0x00), m_header_less(0),
	m_scramble_0(0xFF), m_scramble_1(0xFF), m_num_freqs(0x03), m_freq0(0x18), m_freq1(0x2A), m_freq2(0x3C),
	m_ant(0), m_div(1), m_num_reps(0x0A), m_th_roam(0x50), m_rep_power(0), m_rep_err(0), m_rep_suc(0), m_rep(0),
	m_tint_cmd(0x00), m_fband(0), m_tbclr(0x64), m_fwait(4), m_chk_group(1), m_int_bcn(0), m_fbcn(0),
	m_bcn(0), m_tlp_wait_ex(0), m_lp_wait(0), m_flw(0), m_tlp_wait(0x0F), m_crlf(0), m_delim(1), m_tlp_slp(0x0F),
	m_to_hlss(0x01), m_addr_rep0(0xFF), m_addr_rep1(0xFF),
	m_rbuf_len(0), m_wbuf_len(0), m_pbuf_tail(0), m_parse_cr(0), m_parse_lf(0), m_parse_count(0), 
	m_parse_pow(false), m_cur_cmd(NUL), m_cmd_stat(0)
{
	m_dname[0] = '\0';
	register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
	register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
	register_fpar("br", &m_br, "Baud rate.");

	register_fpar("addr", &m_addr, "own address (0 to 255)");
	register_fpar("addr_group", &m_addr_group, "group address (0 to 255)");
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
	m_rbuf[0] = m_wbuf[0] = '\0';
}

bool f_fep01::read_reg()
{
	return true;
}

bool f_fep01::write_reg()
{
	return true;
}

bool f_fep01::pack_reg()
{
	return true;
}

bool f_fep01::parse_rbuf()
{
	char * rbuf = m_rbuf;
	char * pbuf = m_pbuf + m_pbuf_tail;
	for(int i = 0; i < m_rbuf_len && m_pbuf_tail < 512; i++, rbuf++){
		*pbuf = *rbuf;

		if(*pbuf == 0x0D){
			m_parse_cr = 1;
		}else if(*pbuf == 0x0A){
			m_parse_lf = 1;
		}

		if(m_parse_cr && m_parse_lf){
			// parse m_pbuf
			bool is_proc = false;
			if(m_cur_cmd != NUL && m_cmd_stat & EOC){
				// try to process as a command ressponse
				if(m_pbuf_tail == 4){
					is_proc = true;
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
					case 'N':
						switch(m_pbuf[2]){
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
					default:
						is_proc = false;
					}
					if(is_proc){
						switch(m_cur_cmd){
						case ARG: // N0
							if(m_cmd_stat & N0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case BAN: // P0 or N0 
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case BCL: // P0 or N0 
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case DAS: // P0 or N0 
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case DBM: // N0
							if(m_cmd_stat & N0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case DVS:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case FCN:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case FRQ:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case IDR:
							break;
						case IDW:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case INI:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case PAS:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case POF:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case PON:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case PTE:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case PTN:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case PTS:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case ROF:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case RON:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case REG:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case RID:
							if(m_cmd_stat & N0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case RST:
							if(m_cmd_stat & N0 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
								if(m_ts2_mode && (m_cmd_stat & P0)){
									m_flog_ts2.close();
								}
							}else{
								init_parser();
								return false;
							}
							break;
						case TBN:
						case TBR:
						case TB2:
							if(m_cmd_stat & N0 | m_cmd_stat & N1 | m_cmd_stat & N3 | m_cmd_stat & P0){
								m_cmd_stat |= EOC;
							}else if(m_cmd_stat & P1){
								// sending 
								break;
							}else{
								init_parser();
								return false;
							}
							break;
						case TID:
							if(m_cmd_stat & N0){
								m_cmd_stat |= EOC;
							}else{
								init_parser();
								return false;
							}
							break;
						case TS2:
							if(m_cmd_stat & N0){
								m_cmd_stat |= EOC;
							}else if(m_cmd_stat & P0){
							
								snprintf(m_pbuf, "%s_ts2.log", m_name);
								m_flog_ts2.open(m_pbuf, ios_base::app);
								m_ts2_mode = true;
							}else{
								init_parser();
								return false;
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
								init_parser();
								return false;
							}
							break;
						case VER:
						default:
							is_proc = false;
						}
					}
				}

				if(!is_proc){
					// value? process and clear m_pbuf_tail zero
					is_proc = true;
					switch(m_cur_cmd){
					case ARG: // REGxx:yyH
						if(m_pbuf[0] == 'R' && m_pbuf[1] == 'E' 
							&& m_pbuf[2] == 'G' && m_pbuf[4] == ':'
							&& m_pbuf[7] == 'H'){
							unsigned char ireg = (unsigned char)((m_pbuf[3] - '0') * 10 + m_pbuf[4] - '0');
							unsigned char reg = (unsigned char)(h2i(m_pbuf[5]) * 16 + h2i(m_pbuf[6]));
							m_reg[ireg] = reg;
							if(ireg == 28){ // end of command 
								m_cmd_stat |= EOC;
							}
						}else{
							is_proc = false;
						}
						break;
					case BAN: // L or H
						switch(m_pbuf[0]){
						case 'L':
							m_fband = 0;
							break;
						case 'H':
							m_fband = 1;
							break;
						default:
							is_proc = false;
						}
						break;
					case DAS:
						{
							int das = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							if(das > 0 && das < 256)
								m_addr = das;
							else
								is_proc = false;
						}
						break;
					case DBM:
						{
							int dbm = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							if(dbm > 0 && dbm < 256)
								m_dbm = dbm;
							else
								is_proc = false;
						}
						break;
					case DVS:
						switch(m_pbuf[0]){
						case 'A':
							m_ant = 0;
							m_div = 0;
							break;
						case 'B':
							m_ant = 1;
							m_div = 0;
							break;
						case 'D':
							m_div = 1;
							break;
						default:
							is_proc = false;
						}
						break;
					case FCN:
						{ 
							int nfreqs = m_pbuf[0] - '0';
							if(nfreqs > 0 && nfreqs < 4)
								m_num_freqs = (unsigned char) nfreqs;
							else
								is_proc = false;
						}
						break;
					case FRQ:
						{
							int freq = (int) m_pbuf[0] * 10 + (int) m_pbuf[1];
							if(freq < 24 || (freq > 60 && freq < 62) || freq > 77){
								is_proc = false;
								break;
							}

							switch(m_cmd_arg1){
							case 1:
								m_freq0 = (unsigned char) freq;
								break;
							case 2:
								m_freq1 = (unsigned char) freq;
								break;
							case 3:
								m_freq1 = (unsigned char) freq;
							default:
								is_proc = false;
							}
						}
						break;
					case IDR:
						{
							int scid0, scid1;
							scid0 = h2i(m_pbuf[0]) * 16 + h2i(m_pbuf[1]);
							scid1 = h2i(m_pbuf[2]) * 16 + h2i(m_pbuf[3]);
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
						}
						break;
					case IDW:
						break;
					case INI:
						break;
					case PAS:
						if(m_pbuf[3] == ':'){
							int rep0 = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							int rep1 = (((int)m_pbuf[4] - '0') * 100 + ((int)m_pbuf[5] - '0') * 10  + ((int)m_pbuf[6] - '0'));
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
						}else{
							is_proc = false;
							break;
						}
						break;
					case POF:
						break;
					case PON:
						break;
					case PTE:
						{
							int pte = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							if(pte < 0 || pte > 15){
								is_proc = false;
								break;
							}
							m_tlp_wait_ex = (unsigned char) pte;
						}
					case PTN:
						{
							int ptn = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							if(ptn < 0 || ptn > 255){
								is_proc = false;
								break;
							}
							m_tlp_wait = (unsigned char) ptn;
						}
						break;
					case PTS:
						{
							int pts = (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
							if(pts < 0 || pts > 255){
								is_proc = false;
								break;
							}
							m_tlp_slp = (unsigned char) pts;
						}
						break;
					case ROF:
						break;
					case RON:
						break;
					case REG:
						if(m_pbuf[2] == 'H'){
							int reg = (int) h2i(m_pbuf[0]) * 16 + (int) h2i(m_pbuf[1]);
							if(m_cmd_arg1 < 0 || m_cmd_arg1 > 28){
								is_proc = false;
								break;
							}
							m_reg[m_cmd_arg1] = (unsigned char) reg;
						}else{
							is_proc = false;
							break;
						}
						break;
					case RID:
						if(m_pbuf_tail == 14){
							m_pbuf[m_pbuf_tail - 2] = '\0';
							m_rid = (unsigned int) atoi(m_pbuf);
						}else{
							is_proc = false;
							break;
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
							m_rid = (unsigned int) atoi(m_pbuf);
						}else{
							is_proc = false;
							break;
						}
						break;
					case TS2:
						m_pbuf[m_pbuf_tail] = '\0';
						m_flog_ts2 << m_time_str << " " << m_pbuf << endl;
						break;
					case TXT:
					case TXR:
					case TX2:
						break;
					case VER:
						if(m_pbuf[1] == '.'){
							m_ver = m_pbuf[0] - '0';
							m_sub_ver = (unsigned short) (((int)m_pbuf[0] - '0') * 100 + ((int)m_pbuf[1] - '0') * 10  + ((int)m_pbuf[2] - '0'));
						}
						break;
					default:
						is_proc = false;
					}
				}
			}

			if(!is_proc){ // m_pbuf is not processed as command response
				// try to process as a recieved message
				// if not header less mode and matched m_rec_str? process the message as recieved data
				// if header less mode, and if power info enabled, set parse_count as 3 and m_parse_pow as true
			}

			// clear cr lf flags
			m_parse_cr = m_parse_lf = 0;
		}else if(m_parse_pow){
			if(*pbuf < '0' || *pbuf > '9'){
				// discard all parser state 
				init_parser();
				return false;
			}
			m_parse_count--;
			if(m_parse_count == 0){
				// parse header less message with power info
			}
		}else{
			m_pbuf_tail++;
		}
	}

	if(m_pbuf_tail == 512){
		// discard all parse state
		init_parser();
		return false;
	}
	return true;
}

void f_fep01::init_parser()
{
	m_pbuf_tail = 0;
	m_parse_cr = m_parse_lf = 0;
	m_parse_pow = false;
	m_parse_count = 0;
	m_cur_cmd = NUL;
	m_cmd_arg1 = m_cmd_arg2 = 0;
	m_cmd_stat = 0;
	m_ts2_mode = false;
}