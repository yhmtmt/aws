// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_time.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_time.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_time.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_TIME_H_
#define _F_TIME_H_
#include "f_base.h"

//#define DEBUG_F_TIME

// Filter for time synchronization.
// This uses simple time exchanging scheme using UDP. 
// The filter works both for server and client. 
// The filter works only as server if the server address is not specified.
// There are five state.
// TRN : Transmitting a packet to the server with client's time stamp. Then, move to WAI. (Working as the client)
// RCV : Wait a packet from client with time stamp. If recieved a packet, the server's time is recorded 
//       and state is set to REP. If not, move to TRN or RCV. (Working as the time server)
// WAI : Wait the packet reply for TRN. If succeeded, move to FIX.   
// REP : Replying server's timestamp to the client. 
// FIX : Calculating time offset and correct it. Then move to RCV.
// The filter basically wait packet from client in RCV state. If sufficient time interval is passed, the state is changed to TRN. 
// After the sequence of TRN is finished, weather successfully or not, the state returns to RCV.
class f_time: public f_base
{
protected:
	enum e_mode{
		TRN, RCV, WAI, REP, FIX
	} mode;

	struct s_tpkt{
		unsigned int id;
		long long tc1, ts1, ts2, tc2, del;
		int tz_min; // time zone in minute
		// we assume client time has offset to, therefore 
		//    ts1 - tc1 = d + R(Ts) + to
		//    tc2 - ts2 = d + R(Tc) - to
		// where d is the communication delay, R(Ts) and R(Tc) are the cycle time dependent delay components of server and client. 
		// Ts and Tc is the cycle times of the server and client. R(T) can be the value between 0 and T. Then, 
		// [(ts1 - tc1) - (tc2 - ts2)]/2 = to 
		// We add -to to current time.
		long long calc_delta(){
			return ((ts1 - tc1) - (tc2 - ts2)) / 2;  
		};
	  void pack(char * buf);
	  void unpack(const char * buf);
	} m_trpkt;

	char m_trbuf[sizeof(s_tpkt)];
	bool m_verb;
	char m_host_dst[1024];
	unsigned short m_port, m_port_dst;
	int m_len_pkt;
	SOCKET m_sock;
	socklen_t m_sz_rep;
	sockaddr_in m_sock_addr_snd, m_sock_addr_rep, m_sock_addr_rcv;
	int m_adjust_intvl;
	long long m_tnext_adj;
	int m_rcv_wait_count;
	int m_max_rcv_wait_count;
	bool sttrn();
	bool strcv();
	bool stwai();
	bool strep();
	bool stfix();
	bool clearpkts();
public:
	f_time(const char * name);

	virtual ~f_time()
	{
	}
	
	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
