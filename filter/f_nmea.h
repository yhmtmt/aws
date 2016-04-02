// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_nmea.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_nmea.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_nmea.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_NMEA_H_
#define _F_NMEA_H_

#define SIZE_NMEA_BUF 166 // for two NMEA
#include "../util/aws_serial.h"
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"
#include "../util/c_ship.h"
#include "../channel/ch_base.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_nmea.h"

#include "f_base.h"

class f_nmea: public f_base
{
protected:
	ch_nmea * m_chout;
	ch_nmea * m_chin;

	bool m_blog;
	bool m_verb;
	char m_fname_log[1024];
	ofstream m_flog;
	char m_buf[SIZE_NMEA_BUF];
	char m_buf_send[SIZE_NMEA_BUF];
	char m_filter[6];

	int m_buf_head;
	int m_buf_tail;

	char m_nmea[84];
	int m_nmea_tail;
	char m_src_type_str[8];
	enum e_nmea_src{NONE, FILE, COM, UDP} m_nmea_src;
	union{
		char m_fname[1024];
		char m_host[1024];
	};

	// for serial port source
	unsigned short m_port;
	unsigned int m_cbr;

	AWS_SERIAL m_hcom;

	// for network source
	SOCKET m_sock;
	sockaddr_in m_sock_addr;
	char m_dst_host[256];
	SOCKET m_sock_out;
	sockaddr_in m_sock_addr_out;

	// for file source
	ifstream m_file;
	void dec_type_str(){
		if(strcmp("COM", m_src_type_str)==0){
			m_nmea_src = COM;
		}
		else if(strcmp("UDP", m_src_type_str) == 0){
			m_nmea_src = UDP;
		}
		else if(strcmp("FILE", m_src_type_str) == 0){
			m_nmea_src = FILE;
		}else{
			m_nmea_src = NONE;
		}
	}

	bool open_file();
	bool open_com();
	bool open_udp();

	bool rcv_file();
	bool rcv_com();
	bool rcv_udp();

	//extract_nmea_from_buffer() is the helper for rcv_com() and rcv_udp()
	void extract_nmea_from_buffer();

	int send_nmea();
	bool is_filtered(char * nmea){
		nmea++; // skip ! or $

		// comparing filter with five characters XXYYY where XX is the sender and YYY is the sentence code.
		for(int i = 0; i < 5; i++, nmea++){
			if(m_filter[i] == '*')
				continue;

			if(m_filter[i] != *nmea)
				return false;
		}
		return true;
	}

public:
 f_nmea(const char * name): f_base(name), m_chin(NULL), m_chout(NULL), m_verb(false), m_blog(false),
		m_hcom(NULL_SERIAL), m_nmea_src(NONE){
		m_fname[0] = '\0';

		m_filter[0] = m_filter[1] = m_filter[2] = m_filter[3] = m_filter[4] = '*';
		m_filter[5] = '\0';

		register_fpar("fnmea", m_fname, 1024, "File path of NMEA source file.");
		register_fpar("src_host", m_host, 1024, "IP address of NMEA source (if not specified ADDR_ANY is used).");
		register_fpar("dst_host", m_dst_host, 256, "IP address of NMEA destination (if not specified UDP output is not turned on."); 
		register_fpar("src", m_src_type_str, 128, "Source type. FILE or UDP or COM is allowed.");
		register_fpar("com", &m_port, "Number of NMEA source COM port.");
		register_fpar("bps", &m_cbr, "Baud rate of NMEA source COM port.");
		register_fpar("port", &m_port, "Port number of NMEA source UDP.");
		register_fpar("verb", &m_verb, "For debug.");
		register_fpar("log", &m_blog, "Log enable (y or n)");
		register_fpar("filter", m_filter, 6, "Sentence filter. 5 characters are to be specified. * can be used as wild card.");
	}

	~f_nmea(){
	}
	
	virtual bool seek(long long seek_time);
	virtual bool init_run();
	virtual bool proc();
	virtual void destroy_run();
};


// f_nmea_proc decodes nmea 0183 sentences 
// This class accesses global ship list. 
class f_nmea_proc: public f_base
{
protected:
	ch_nmea * m_chin;
	ch_nmea * m_chout;
	c_nmea_dec m_nmeadec;
	int m_bm_ver;
	char m_buf[84];
	bool m_btime_rmc;

	///////////// for AIS Binary message
	char m_toker[3];
	s_binary_message m_aibm; // AIS binary message
	char m_str_aibm[1024]; // string to be sent as AIS binary message
	enum e_str_aibm_type{
		AIBM_TEXT, AIBM_C6, AIBM_C8, AIBM_BIN, AIBM_HEX
	} m_aibm_type;
	static const char * str_aibm_type[AIBM_HEX + 1];

public:
	f_nmea_proc(const char * name): f_base(name), m_chin(NULL), m_chout(NULL), m_bm_ver(1), m_btime_rmc(false)
	{
		register_fpar("bm_ver", &m_bm_ver, "Version of the binary message");
		register_fpar("trmc", &m_btime_rmc, "Time is collected by RMC sentence.");

		// for AIS binary message
		register_fpar("aibm_str_type", (int*)&m_aibm_type, AIBM_HEX + 1, str_aibm_type, "Type of AIS binary message {C6, C8, BIN, HEX}.");
		m_str_aibm[0] = '\0';
		register_fpar("aibm_str", m_str_aibm, 1023, "String to be sent as AIS Binary message.");
		register_fpar("aibm_ch", &m_aibm.ch, "AIS Channel selection. 0: Auto  1: A, 2: B, 3: A and B.");
		register_fpar("aibm_sq", &m_aibm.sq, "Sequential ID. for ABM 0 to 3, for BBM 0 to 9.");
		register_fpar("aibm_mmsi", &m_aibm.mmsi, "Destination MMSI for sending ABM.");
		register_fpar("aibm_type", &m_aibm.type, "Binary message type. 6 and 12 is ABM, 8 and 14 is BBM");
		m_toker[0] = 'A';
		m_toker[1] = 'W';
		register_fpar("toker", m_toker, 3, "Toker string. (2 characters)");
	}

	~f_nmea_proc(){
	}	
	virtual bool init_run();
	virtual bool proc();
	virtual void destroy_run();
};

#endif
