#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// command.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// command.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with command.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <errno.h>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "CmdAppBase/CmdAppBase.h"
#include "util/aws_sock.h"
#include "util/aws_thread.h"
#include "util/c_clock.h"
#include "channel/ch_base.h"
#include "filter/f_base.h"

#include "command.h"
#include "c_aws.h"

void cmd_proc_loop(const char * prompt, c_aws & aws, istream & in);
bool proc_script(const char * fname, c_aws & aws);

void * cmd_proc(void * paws)
{
	c_aws & aws = *((c_aws*) paws);

	cmd_proc_loop("aws", aws, cin);

	return NULL;
}

void cmd_proc_loop(const char * prompt, c_aws & aws, istream & in)
{
	// invoke capturing threads for each cam
	char cmd_line[CMD_LEN];

	while(!g_kill && !in.eof()){ // command processing loop
		cout << prompt << ">";
		in.getline(cmd_line, CMD_LEN, '\n');

		int len = (int) strlen(cmd_line);
		if(len >= CMD_LEN){
			cerr << "Irregal command. Command length\
					should be less than " << CMD_LEN << endl;
			continue;
		}

		// removing comment. (comment is headed by "//" or "#"
		for(int i = 0; i < len; i++){
			if((cmd_line[i] == '/' && cmd_line[i+1] == '/') || 
				cmd_line[i] == '#'){
					cmd_line[i] = '\0';
					break;
			}
		}

		cout << cmd_line << endl;
		bool stat = false;
		if(aws.push_command(cmd_line, cmd_line, stat)){
			if(!stat)
				cout << "Error: ";
			else 
				cout << "Success: ";
			cout << cmd_line << endl;
			continue;
		}

		if(proc_script(cmd_line, aws))
			continue;

		cerr << "Unknown command" << endl;
	}
}

bool split_cmd_tok(char * cmd, vector<char *> & cmd_tok)
{
	int i = 0;
	bool seek_head = true;
	cmd_tok.clear();

	while(cmd[i] != '\0'){
		if(i == CMD_LEN) // no termination character found
			return false;
		if(cmd[i] == ' ' || cmd[i] == '\t'){ // space and tab is a delimiter
			cmd[i] = '\0';
			seek_head = true;
		}else if(seek_head){
			cmd_tok.push_back(&cmd[i]);
			seek_head = false;
		}

		i++;
	}

	return true;
}

bool proc_script(const char * fname, c_aws & aws)
{
	ifstream ifile(fname);
	if(ifile.is_open()){
		cmd_proc_loop(fname, aws, ifile);
		return true;
	}
	return false;
}


//////////////////////////////////////////////////////// class c_rcmd member
c_rcmd::c_rcmd(c_aws * paws, unsigned short port):m_paws(paws){
	m_to.tv_sec = 5;
	m_to.tv_usec = 0;
	m_exit = true;
	m_svr_sock = socket(AF_INET, SOCK_STREAM, 0);
	if(m_svr_sock == SOCKET_ERROR){
		cerr << "socket failed with SOCKET_ERROR" << endl;
		m_svr_sock = NULL;
		return;
	}

	m_svr_addr.sin_family = AF_INET;
	m_svr_addr.sin_port = htons(port);
	m_svr_addr.sin_addr.s_addr = INADDR_ANY;

	int ret = ::bind(m_svr_sock, (sockaddr*)&m_svr_addr, sizeof(m_svr_addr));
	if(ret != 0){
		cerr << "bind failed with SOCKET_ERROR." << endl;
		closesocket(m_svr_sock);
		m_svr_sock = NULL;
		return;
	}

	ret = listen(m_svr_sock, 5);
	if(ret == SOCKET_ERROR){
		cerr << "listen failed with SOCKET_ERROR." << endl;
		closesocket(m_svr_sock);
		m_svr_sock = NULL;
		return;
	}

	m_exit = false;

	pthread_create(&m_th_rcmd, NULL, thrcmd, (void*)this);
}

c_rcmd::~c_rcmd(){
	if(!m_exit){
		m_exit = true;
		pthread_join(m_th_rcmd, NULL);
	}
	if(m_svr_sock != SOCKET_ERROR && m_svr_sock != NULL)
		closesocket(m_svr_sock);
}

// wait_connection waits connection to socket s. The function don't return
// until session opened or timeout.
bool c_rcmd::wait_connection(SOCKET & s){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdread);
	FD_ZERO(&m_fderr);
	FD_SET(m_svr_sock, &m_fdread);
	FD_SET(m_svr_sock, &m_fderr);
	int n = select((int)(m_svr_sock)+1, &m_fdread, NULL, &m_fderr, &m_to);

	if(n > 0){
		if(FD_ISSET(m_svr_sock, &m_fdread)){
			int len = sizeof(m_svr_addr);
#ifdef _WIN32
			s = accept(m_svr_sock, (sockaddr*)&m_svr_addr, &len);
#else
			s = accept(m_svr_sock, (sockaddr*)&m_svr_addr, (socklen_t*) &len);
#endif
			return true;
		}else if(FD_ISSET(m_svr_sock, &m_fderr)){
			cerr << "Socket error." << endl;
			m_exit = true;
			return false;
		}
	}
	return false;
}

// wait_receive receives data to buf from socket s. Data length is returned
// to the argument "total". This function should be called after session opened.
bool c_rcmd::wait_receive(SOCKET & s, char * buf, int & total){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdread);
	FD_ZERO(&m_fderr);
	FD_SET(s, &m_fdread);
	FD_SET(s, &m_fderr);

	int n = select((int)s+1, &m_fdread, NULL, &m_fderr, &m_to);
	if(n > 0){
		if(FD_ISSET(s, &m_fdread)){
			total += recv(s, buf + total, CMD_LEN - total, 0);
			return total == CMD_LEN;
		}else if(FD_ISSET(s, & m_fderr)){
			cerr << "Socket error" << endl;
			return false;
		}
	}	
	cout << "wait_receive exit" << endl;

	return false;
}

// push_command send command string (cmd_str) to the aws object.
// this function blocks until the command processed in the aws object.
// return string is stored in ret_str. if the command failed, ret_stat is
// set as false.
bool c_rcmd::push_command(const char * cmd_str, char * ret_str, bool ret_stat){
	bool stat = false;
	if(!m_paws->push_command(cmd_str, ret_str, ret_stat)){
		cerr << "Unknown command " << cmd_str << endl;
		return false;
	}
	return true;
}

// wait_send send is used to send return string to the sender of the command.
// This function returns after sending data or timeout
bool c_rcmd::wait_send(SOCKET & s, char * buf){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdwrite);
	FD_ZERO(&m_fderr);
	FD_SET(s, &m_fdwrite);
	FD_SET(s, &m_fderr);
	int n = select((int)s+1, NULL, &m_fdwrite, &m_fderr, &m_to);
	if(n > 0){
		if(FD_ISSET(s, &m_fdwrite)){
			send(s, buf, CMD_LEN, 0);
			return true;
		}else if(FD_ISSET(s, &m_fderr)){
			cerr << "Socket error" << endl;
			return false;
		}
	}
	return false;
}

// command processing thread repeates 1. waiting for connection from command 
// sender, 2. receiving the command, 3. processing the command in aws, 4. 
// returning the resulting return value. This thread is invoked in the constructor,
// and terminated in the destructor.
void * c_rcmd::thrcmd(void * ptr)
{
	c_rcmd * prcmd = (c_rcmd*) ptr;

	while(!prcmd->is_exit()){
		SOCKET s;
		if(!prcmd->wait_connection(s)){
			continue;
		}

		while(1){
			int total = 0;
			while(!prcmd->wait_receive(s, prcmd->m_buf_recv, total));
			if(strcmp("eoc", prcmd->m_buf_recv) == 0){
				break;
			}

			// push command 
			bool stat = false;
			if(total == CMD_LEN){
				if(!prcmd->m_paws->push_command(prcmd->m_buf_recv, 
					prcmd->m_buf_send, stat)){
						cerr << "Unknown command " << prcmd->m_buf_recv << endl;
				}
			}

			// return
			while(!prcmd->wait_send(s, prcmd->m_buf_send));
		}
		closesocket(s);
	}

	return NULL;
}

bool c_rcmd::is_exit(){
	return m_paws->is_exit() || m_exit;
}