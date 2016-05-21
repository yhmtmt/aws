// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_shioji.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_shioji.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_shioji.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef _F_SHIOJI_H_
#define _F_SHIOJI_H_

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"

#include "../channel/ch_base.h"
#include "../channel/ch_navdat.h"

#include "f_base.h"

class f_shioji_ctrl: public f_base
{
protected:
	s_ship_ctrl m_ctrl;
public:
	f_shioji_ctrl(const char * fname):f_base(fname)
	{
	}
	virtual ~f_shioji_ctrl()
	{
	}

	virtual bool check()
	{
		return m_chout[0] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd){
		int num_args = cmd.num_args;
		char ** args = cmd.args;

		if(num_args < 2)
			return false;

		int itok = 2;

		if(strcmp(args[itok], "rud") == 0){
			if(num_args == 4){
				m_ctrl.Rud_com = atof(args[itok+1]);
				return true;
			}
			return false;
		}else if(strcmp(args[itok], "cpp") == 0){
			if(num_args == 4){
				m_ctrl.CPP_com = atof(args[itok+1]);
				return true;
			}
			return false;
		}else if(strcmp(args[itok], "bth") == 0){
			if(num_args == 4){
				m_ctrl.B_th_com = atof(args[itok+1]);
				return true;
			}
			return false;
		}else if(strcmp(args[itok], "sth") == 0){
			if(num_args == 4){
				double S_th = atof(args[itok+1]);
				m_ctrl.Ss_th_com = S_th;
				return true;
			}
			return false;
		}
		return f_base::cmd_proc(cmd);
	}

	virtual bool proc(){
		ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chout[0]);
		if(pctrl == NULL)
			return false;

		pctrl->set(m_ctrl);
		return true;
	}
};


class f_shioji_ctrl_rcv: public f_base
{
protected:
	s_ship_ctrl m_ctrl;
	char m_buf[sizeof(s_ship_ctrl)];
	SOCKET m_sock;
	sockaddr_in m_sock_addr;
public:
	f_shioji_ctrl_rcv(const char * fname):f_base(fname){
		m_sock = socket(AF_INET, SOCK_DGRAM, 0);
		m_sock_addr.sin_family = AF_INET;
		m_sock_addr.sin_port = htons((unsigned short) 11100);
#ifdef _WIN32
		m_sock_addr.sin_addr.S_un.S_addr = INADDR_ANY;
#else
		m_sock_addr.sin_addr.s_addr = INADDR_ANY;
#endif
		if(::bind(m_sock, (sockaddr*)&m_sock_addr, sizeof(m_sock_addr)) == SOCKET_ERROR)
			return;
	}
	virtual ~f_shioji_ctrl_rcv(){
#ifdef _WIN32
			closesocket(m_sock);
#else
			::close(m_sock);
#endif
	}
	virtual size_t get_num_in_chans(){return 0;};
	virtual size_t get_num_out_chans(){return 0;};
	virtual bool proc(){
		size_t len = 0;
		size_t tot_len = 0;
		while(tot_len < sizeof(s_ship_ctrl)){
			len = recv(m_sock, &m_buf[len], (int)(sizeof(s_ship_ctrl) - len),
				0);
			if(len == 0)
				return false;
			if(len == -1)
				tot_len = 0;
			else 
				tot_len += len;
		}
		memcpy((void*)&m_ctrl, (void*) m_buf, sizeof(s_ship_ctrl));
		cout << "Control_rcv: " << m_ctrl.Rud_com << " " 
			<< m_ctrl.CPP_com << " " << m_ctrl.Ss_th_com << " " <<
			m_ctrl.Sp_th_com << " " << m_ctrl.B_th_com << " " << endl;

		return true;
	}
};


///////////////////////////////////////////////////////////////f_shioji
// f_shioji is the class for interfacing the system on the shiojimaru.
// this class can receive the shiojimaru's state data via network and redirect
// it to any addres and port. Or it can send control packet to the shiojimaru.
// Additionally, the class can replay the log file of the shiojimaru state data
// saved as the csv format.
class f_shioji: public f_base
{
private:
	// for offline mode
	bool m_bfile;
	ifstream m_file;

	// for online mode
	SOCKET m_sock;
	sockaddr_in m_sock_addr;
	sockaddr_in m_svr_sock_addr;
#ifdef _WIN32
	int m_size_svr_sock_addr;
#else
	unsigned int m_size_svr_sock_addr;
#endif
	// buffer
	char m_buf[2048];
	char * m_rec[48];

	long long m_sys_time;
	bool getrec();
	void dectime(char & d, char & h, char & m, float & s);
	void decrec();

	// for redirection
	bool m_bredir; // flag indicates redirection is enabled or not.
	SOCKET m_rsock; // redirection socket
	sockaddr_in m_raddr; // destination address and port
	char m_buf_redir[384]; // redirection buffer
	void pack_buf_redir(); // pack the data into the redirection buffer 
	void redir(); // send data in redirection buffer

	// for control 
	bool m_bctrl; // flag indicates control is enabled or not.
	SOCKET m_ctrl_sock; // socket for control (udp)
	sockaddr_in m_ctrl_addr; // shioji's control address and port
	long long m_ctrl_int; // control interval
	long long m_ctrl_prev_time; // previous time control signal sent.
	s_ship_ctrl m_ctrl; // object for control values
	long long m_sth_int; // interval required when the stern thruster rotate to the opposit direction.

	void ctrl(); // send control packet to shioji
	void sat_ctrl(s_ship_ctrl & ctrl); //helper for ctrl. limits the control values to their upper and lower bounds.

	bool m_bdump;
public:
	f_shioji(const char * name);
	virtual ~f_shioji();

	virtual size_t get_num_in_chans(){return 1;};
	virtual size_t get_num_out_chans(){return 1;};
	virtual bool check()
	{
		return m_chout[0] != NULL;
	}

	virtual bool seek(long long seek_time);
	virtual bool run(long long start_time, long long end_time);

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();

	// open shioji control socket
	bool open_ctrl(const char * url, const char * port);
	// close shioji control socket
	void close_ctrl();

	// open redirection udp sockets. packet from shioji is redirected as it is
	bool open_redir(const char * url, const char * port);
	// close redirection udp sockets
	void close_redir();

	// open shioji state listening socket
	bool open(const char * url, const char * port);
	// open shioji state log (only for offline mode)
	bool open(const char * fname);
	// close all files and sockets including control and redirection sockets
	void close();
};

#endif
