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


#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_time.h"


f_time::f_time(const char * name): f_base(name), mode(RCV), m_adjust_intvl(10), m_tnext_adj(0)
{
	m_host_dst[0] = '\0';
	register_fpar("port", &m_port, "UDP port.");
	register_fpar("port_svr", &m_port_dst, "Server UDP port.");
	register_fpar("host_svr", m_host_dst, 1024, "Server address.");
	register_fpar("Tadj", &m_adjust_intvl, "Time interval adjustment occurs in second.");
}

bool f_time::init_run()
{
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
		cerr << "Socket error" << endl;
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
	switch(mode){
	case TRN:
		if(m_host_dst[0] == '\0'){
			mode = RCV;
		}else{
			// transmmit packet to the master server
			memset((void*) &m_trpkt, 0, sizeof(s_tpkt));
			m_trpkt.id = ((unsigned int) rand() << 16) | (unsigned int) rand();
			m_trpkt.tc1 = m_cur_time;
			socklen_t sz = sizeof(m_sock_addr_snd);
			sendto(m_sock, (const char*) &m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_snd, sz);
			mode = WAI;
		}
		break;
	case WAI:
		{
			s_tpkt rcvpkt;
			socklen_t sz = sizeof(m_sock_addr_rep);
			long long del = m_adjust_intvl * SEC;
			int count = 0;
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
						recvfrom(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, &sz);
						if(rcvpkt.id != m_trpkt.id){
							rcvpkt.del = del;
							sendto(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, sz);
							count++;
							continue;
						}else{
							if(rcvpkt.del != 0){
								m_tnext_adj = m_cur_time + rcvpkt.del;
								mode = RCV;
							}else{
								m_trpkt.ts1 = rcvpkt.ts1;
								m_trpkt.ts2 = rcvpkt.ts2;
								m_trpkt.tc2 = m_cur_time;
							}
							break;
						}
					}else if(FD_ISSET(m_sock, &fder)){
						cerr << "Socket error." << endl;
						return false;
					}else{
						cerr << "Unknown error." << endl;
						return false;
					}
					del += m_adjust_intvl * SEC;
				}
				break;
			}

			if(count > 0){
				// if the count value is larger than 0, the packet transmission is disturbed by other packet. 
				// Again, we try transmission.
				mode = TRN;
			}else{
				// if count == 0, the packet is correctly returned. now the filter try to correct time offset.
				mode = FIX;
			}
		}
		break;
	case RCV:
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
					recvfrom(m_sock, (char*)&m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, &m_sz_rep);
					m_trpkt.ts1 = m_cur_time;
					mode = REP;
				}else if(FD_ISSET(m_sock, &fder)){
					cerr << "Socket error." << endl;
					return false;
				}else{
					cerr << "Unknown error." << endl;
					return false;
				}
			}else{
				if(m_tnext_adj < m_cur_time){
					if(m_host_dst[0] == '\0')
						mode = RCV;
					else
						mode = TRN;
				}			
			}

			if(mode == REP){
				// consume remained packet from clients
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
							recvfrom(m_sock, (char*)&rcvpkt, sizeof(s_tpkt), 0, (sockaddr*)&addr_del, &len_del);
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
			}
		}
		break;
	case REP:
		m_trpkt.ts2 = m_cur_time;
		sendto(m_sock, (char*)&m_trpkt, sizeof(s_tpkt), 0, (sockaddr*)&m_sock_addr_rep, m_sz_rep);
		mode = RCV;
		break;
	case FIX:
		{
			long long delta = m_trpkt.calc_delta();
			m_clk.set_time_delta(delta);
			m_tnext_adj = (long long) m_adjust_intvl * SEC;
			mode = RCV;
		}
		break;
	}

	return true;
}