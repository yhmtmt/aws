// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_time.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_time.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_time.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include "stdafx.h"

#include <errno.h>
#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_time.h"


f_time::f_time(const char * name): f_base(name), m_verb(false), mode(RCV), m_adjust_intvl(10),
	m_tnext_adj(0), m_max_rcv_wait_count(1000)
{
	m_host_dst[0] = '\0';
	register_fpar("verb", &m_verb, "Verbose for debug.");
	register_fpar("port", &m_port, "UDP port.");
	register_fpar("port_svr", &m_port_dst, "Server UDP port.");
	register_fpar("host_svr", m_host_dst, 1024, "Server address.");
	register_fpar("Tadj", &m_adjust_intvl, "Time interval adjustment occurs in second.");
	register_fpar("MaxWaitCount", &m_max_rcv_wait_count, "Wait count for recieving reply packet.");
}

bool f_time::init_run()
{
	m_sock = socket(AF_INET, SOCK_DGRAM, 0);	
	if(set_sock_nb(m_sock) != 0)
		return false;

	m_sock_addr_snd.sin_family =AF_INET;
	m_sock_addr_snd.sin_port = htons(m_port_dst);
	set_sockaddr_addr(m_sock_addr_snd, m_host_dst);

	m_sock_addr_rep.sin_family = AF_INET;
	m_sock_addr_snd.sin_port = htons(m_port_dst);
	set_sockaddr_addr(m_sock_addr_rep, m_host_dst);

	m_sock_addr_rcv.sin_family =AF_INET;
	m_sock_addr_rcv.sin_port = htons(m_port);
	set_sockaddr_addr(m_sock_addr_rcv);
	if(::bind(m_sock, (sockaddr*)&m_sock_addr_rcv, sizeof(m_sock_addr_rcv)) == SOCKET_ERROR){
		cerr << "Socket error " << strerror(errno) << endl;
		return false;
	}

	return true;
}

void f_time::destroy_run()
{
	closesocket(m_sock);
	m_sock = -1;
}

bool f_time::proc()
{
	if(m_verb)
		cout << m_time_str << endl;


	switch(mode){
	case TRN: // -> RCV or WAI
		if(!sttrn())
			return false;
		break;
	case WAI: // -> RCV or TRN or FIX
		if(!stwai())
			return false;
		break;
	case RCV: // -> RCV or TRN
		if(!strcv())
			return false;
		break;
	case REP: // -> RCV or REP
		if(!strep())
			return false;
		break;
	case FIX: // -> RCV
		if(!stfix())
			return false;
		break;
	}

	return true;
}

// State TRN transmits time synchronization request packet. If the filter is not configured as client
// , means that destination host address is not specified, the request packet never transmitted, and 
// the state never move to WAI.
// State Transition
// TRN -> WAI occurs if destination address is defined, after sending initial time synchronization request 
// TRN -> RCV occurs if destination address is not defined. In the case, the state never be back to TRN.
bool f_time::sttrn()
{
	if(m_host_dst[0] == '\0'){
#ifdef DEBUG_F_TIME
		cout << " Destination host is not specified. Move to RCV mode." << endl;
#endif
		mode = RCV;
	}else{
		// transmmit packet to the master server
		memset((void*) &m_trpkt, 0, sizeof(s_tpkt));
		m_trpkt.id = ((unsigned int) rand() << 16) | (unsigned int) rand();
		m_trpkt.tc1 = m_cur_time;
#ifdef DEBUG_F_TIME
		cout << "Sending tsync request with id:" << m_trpkt.id
			<< " Tc1:" << m_trpkt.tc1 << endl;
#endif
		socklen_t sz = sizeof(m_sock_addr_snd);
		sendto(m_sock, (const char*) &m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_snd, sz);
		m_rcv_wait_count = 0;
		mode = WAI;
	}
	return true;
}

// State WAI waits a time synchronization packet as a reply to the packet previously sent in TRN state.
// If recieving the reply packet is distarbed by other time synchronization request, the time synchronization
// process is aborted, and again the state move to TRN. Simultaneously, for the time synchronization request,
// "del" command is sent to the client. "del" command include time the client should wait for sending 
// synchronization request again. If "del" command is recieved in this state, the state is forced to be RCV,
// until the wait time goes by. The successful reception of the reply packet change the state as FIX.
// 
// STATE Transition
// WAI->TRN occurs if other time synchronization request is recieved faster.
// WAI->RCV occurs if time synchronization request is denied.
// WAI->FIX occurs if the reply packet is recieved first.
bool f_time::stwai()

{
	s_tpkt rcvpkt;
	socklen_t sz = sizeof(m_sock_addr_rep);
	long long del = m_adjust_intvl * SEC;
	int count = 0; // error counter

	// In the loop all the packets recieved at this moment are consumed.
	while(1){
		timeval to;
		to.tv_sec = 0;
		to.tv_usec = 10000; // waits 10msec
		fd_set fdrd, fder;
		FD_ZERO(&fdrd);
		FD_ZERO(&fder);
		FD_SET(m_sock, &fdrd);
		FD_SET(m_sock, &fder);
		int n = select((int)(m_sock) + 1, &fdrd, NULL, &fder, &to);
		if(n > 0){
			if(FD_ISSET(m_sock, &fdrd)){
				sz = sizeof(m_sock_addr_rep);
				recvfrom(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, &sz);
				if(rcvpkt.id == m_trpkt.id){
					if(rcvpkt.del != 0){
#ifdef DEBUG_F_TIME
						cout << "Request denied. id: " << rcvpkt.id << " Twait: " << rcvpkt.del << endl;
#endif
						m_tnext_adj = m_cur_time + rcvpkt.del;
						mode = RCV;
#ifdef DEBUG_F_TIME
						cout << "Next request is set as Tnext: " <<  m_tnext_adj << endl;
						cout << "Move to RCV mode" << endl;
#endif
						break;
					}else{
#ifdef DEBUG_F_TIME
						cout << "Recieved tsync packet from server id: " << rcvpkt.id 
							<< " tc1: " << rcvpkt.tc1 << " ts1: " << rcvpkt.ts1
							<< " ts2: " << rcvpkt.ts2 << " tc2: " << m_cur_time << endl;
#endif
						m_trpkt.ts1 = rcvpkt.ts1;
						m_trpkt.ts2 = rcvpkt.ts2;
						m_trpkt.tc2 = m_cur_time;
						if(count == 0){
							mode = FIX;
						}else{
							mode = TRN;
						}
						break;
					}
				}else{
					rcvpkt.del = del;
#ifdef DEBUG_F_TIME
					cout << "Different tsync packet recieved sent id: " << m_trpkt.id << " rcvd id: " << rcvpkt.id << endl;
					cout << "Denying request with wait time " << del << endl;
#endif
					sendto(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, sz);
					del += m_adjust_intvl * SEC;
					count++;
				}
			}else if(FD_ISSET(m_sock, &fder)){
				cerr << "Socket error." << endl;
				return false;
			}else{
				cerr << "Unknown error." << endl;
				return false;
			}
		}else{ // time out
			if(m_rcv_wait_count < 1000 && count == 0){
				mode = WAI;
				m_rcv_wait_count++;
			}else{
				mode = TRN;
			}
			break;
		}
	}

#ifdef DEBUG_F_TIME
	switch(mode){
	case RCV:
		cout << "Move to RCV mode." << endl;
		break;
	case WAI:
		cout << "Move to WAI mode." << endl;
		break;
	case FIX:
		cout << "Move to FIX mode." << endl;
	case TRN:
		cout << "Move to TRN mode." << endl;
		break;
	}
#endif

	return clearpkts();
}

// State RCV recieves a time synchronization request. Only the first client can get the reply. Otherwise, 
// last call of clearpkts() send "del" command.
// 
// State Transition
// RCV -> REP occurs if a time syncrhonization packet is recieved.
// RCV -> TRN occurs when (1) no time synchronization request arrived, and (2) the wait time passed, and 
//      (3) the filte can be a client (the request's destination is specified).
// RCV -> RCV, otherwise, this occurs.
bool f_time::strcv()		
{
	timeval to;
	to.tv_sec = 0;
	to.tv_usec = 0;
	fd_set fdrd, fder;
	FD_ZERO(&fdrd);
	FD_ZERO(&fder);
	FD_SET(m_sock, &fdrd);
	FD_SET(m_sock, &fder);
	int n = select((int)(m_sock) + 1, &fdrd, NULL, &fder, &to);
	if(n > 0){
		if(FD_ISSET(m_sock, &fdrd)){
			m_sz_rep = sizeof(m_sock_addr_rep);
			recvfrom(m_sock, (char*)&m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, &m_sz_rep);
			m_trpkt.ts1 = m_cur_time;
#ifdef DEBUG_F_TIME
			cout << "Tsync packet is recieved from client id: " << m_trpkt.id << " tc1: " << m_trpkt.tc1 << " ts1: " << m_cur_time << endl;
#endif
			mode = REP;
		}else if(FD_ISSET(m_sock, &fder)){
			cerr << "Socket error." << endl;
			return false;
		}else{
			cerr << "Unknown error." << endl;
			return false;
		}
	}else{
#ifdef DEBUG_F_TIME
		cout << "RCV mode in Curtime:" << m_cur_time << " NextTRN time: " << m_tnext_adj << " Svr: " << m_host_dst << endl; 
#endif
		if(m_tnext_adj < m_cur_time){
			if(m_host_dst[0] == '\0')
				mode = RCV;
			else
				mode = TRN;
		}			
	}

#ifdef DEBUG_F_TIME
	switch(mode){
	case REP:
		cout << "Move to REP mode." << endl;
		break;
	case RCV:
		cout << "Move to RCV mode." << endl;
		break;
	case TRN:
		cout << "Move to TRN mode." << endl;
		break;
	}
#endif
	return clearpkts();
}

// State rep send a reply packet corresponding to the packet recieved in the RCV state.
// The packets arrived at this state are all cleared by the call of clearpkts(). Of course, "del" command 
// is sent.
//
// State Transition
// REP -> RCV always occurs.
bool f_time::strep()
{
#ifdef DEBUG_F_TIME
		cout << "Replying tsync request id: " << m_trpkt.id << " tc1: " << 
			m_trpkt.tc1 << " ts1: " << m_trpkt.ts1 << " ts2: " << m_trpkt.ts2 << endl;
#endif

	m_trpkt.ts2 = m_cur_time;
	m_trpkt.tz_min = f_base::get_tz();
	sendto(m_sock, (char*)&m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, m_sz_rep);
	mode = RCV;
	return clearpkts();
}

// State fix correct the time according to the time synchronization packet recieved at WAI state.
// Then the state moves to RCV.
// 
// State Transition
// FIX -> RCV always occurs.
bool f_time::stfix()
{
	long long delta = m_trpkt.calc_delta();
	f_base::set_tz(m_trpkt.tz_min);
	//	cout << "ts1 - tc1" << m_trpkt.ts1 - m_trpkt.tc1 << endl;
	//	cout << "tc2 - ts2" << m_trpkt.tc2 - m_trpkt.ts2 << endl;
	//	cout << "(ts1-tc1)-(tc2-ts2)" << (m_trpkt.ts1 - m_trpkt.tc1) -
	//	  (m_trpkt.tc2 - m_trpkt.ts2 ) << endl;

	m_clk.set_time_delta(delta);
	m_tnext_adj = m_cur_time + (long long) m_adjust_intvl * SEC;
	mode = RCV;
	if(m_verb){
		cout << "Fix the time id: " << m_trpkt.id << " delta: " << delta << endl;
		cout << "Next request is set as Tnext: " <<  m_tnext_adj << endl;
		cout << "Move to RCV mode." << endl;
	}
	return clearpkts();
}


// Consume remained packet from clients. for every client del command is sent with different wait time.
bool f_time::clearpkts()
{
	s_tpkt rcvpkt;
	sockaddr_in addr_del;
	socklen_t len_del;
	long long del = m_adjust_intvl * SEC;
	while(1){
		timeval to;
		to.tv_sec = 0;
		to.tv_usec = 0;
		fd_set fdrd, fder;
		FD_ZERO(&fdrd);
		FD_ZERO(&fder);
		FD_SET(m_sock, &fdrd);
		FD_SET(m_sock, &fder);
		int n = select((int)(m_sock) + 1, &fdrd, NULL, &fder, &to);
		if(n > 0){
			if(FD_ISSET(m_sock, &fdrd)){
				len_del = sizeof(addr_del);
				n = recvfrom(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&addr_del, &len_del);
				if(n == SOCKET_ERROR){
#ifdef _WIN32
					cerr << "Failed to recieve packet recvfrom() " << strerror(errno) << endl;	
					n = WSAGetLastError();
#else
					cerr << "Failed to recieve packet recvfrom() " << strerror(errno) << endl;
#endif
					return false;
				}
#ifdef DEBUG_F_TIME
				cout << "Sending del packet id " << rcvpkt.id << " Twait: " << del << endl;
#endif
				rcvpkt.del = del;
				sendto(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&addr_del, len_del);
			}else if(FD_ISSET(m_sock, &fder)){
				cerr << "Socket error." << endl;
				return false;
			}else{
				cerr << "Unknown error." << endl;
				return false;
			}
		}else{
			break;
		}
		del += m_adjust_intvl * SEC;
	}
	return true;
}
