#ifndef _F_EVENT_H_
#define _F_EVENT_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_event.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_event.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_event.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
enum e_evt_type
{
	EVT_TIME, EVT_PERIOD, EVT_POS
};

class f_event: public f_base
{
protected:
	static const char * m_evt_str[EVT_POS + 1];
	e_evt_type m_evt_type;
	char m_tstr[1024];
	char m_host[1024];
	unsigned short m_port;
	int m_num_itrs;
	bool m_bregister;

	struct s_event
	{
		e_evt_type m_evt_type;
		
		long long tabs;
		long long period;

		SOCKET m_sock;
		sockaddr_in m_saddr;
		int m_num_itrs;
	};

	list<s_event> m_event;

public:
	f_event(const char * name):f_base(name), m_evt_type(EVT_TIME), m_port(7743), m_num_itrs(1),
		m_bregister(false)
	{
		m_host[0] = '\0';
		register_fpar("type", (int*)&m_evt_type, (int)EVT_POS + 1, m_evt_str, "Event type.");
		register_fpar("host", m_host, 1024, "Host address for event notification.");
		register_fpar("port", &m_port, "Number of port for event notification.");
		register_fpar("tstr", m_tstr, 1024, "Time string for specifying time and period event.");
		register_fpar("breg", &m_bregister, "Register the event.");
		register_fpar("itrs", &m_num_itrs, "Number of event notifications.");
	}

	virtual bool init_run()
	{
		return true;
	}

	virtual void destroy_run()
	{
	}

	virtual bool proc()
	{
		if(m_bregister){
			if(m_host[0] != '\0'){
				// register the event
				s_event evt;
				evt.m_num_itrs = m_num_itrs;
				evt.m_sock = socket(AF_INET, SOCK_DGRAM, 0);
				evt.m_saddr.sin_family = AF_INET;
				evt.m_saddr.sin_port = htons(m_port);
				set_sockaddr_addr(evt.m_saddr, m_host);
				tmex tm;
				switch(m_evt_type){
				case EVT_TIME:
					if(!decTmStr(m_tstr, tm))
						break;
					evt.tabs = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
					m_host[0] = '\0';
					m_event.push_back(evt);
					break;
				case EVT_PERIOD:
					evt.tabs = m_cur_time;
					evt.period = (long long) (atof(m_tstr) * SEC);
					evt.tabs += evt.period;
					m_host[0] = '\0';
					m_event.push_back(evt);
					break;
				case EVT_POS:
					break;
				}
			}
			m_bregister = false;
		}

		list<s_event>::iterator itr = m_event.begin();
		while(itr != m_event.end()){
			if(itr->tabs <= m_cur_time){
				if(itr->m_num_itrs){
					// event occur
					fd_set dw;
					fd_set de;
					FD_ZERO(&dw);
					FD_ZERO(&de);
					FD_SET(itr->m_sock, &dw);
					FD_SET(itr->m_sock, &de);
					timeval tout;
					tout.tv_sec = 10;
					tout.tv_usec = 0;
					int n = select((int)itr->m_sock + 1, NULL, &dw, &de, &tout);
					if(n > 0){
						if(FD_ISSET(itr->m_sock, &dw)){
							int len = (int) strlen(m_time_str) + 1;
							int res = sendto(itr->m_sock, m_time_str, len,
								0, (sockaddr*)&(itr->m_saddr), sizeof(itr->m_saddr));

							if(len == res){
								itr->m_num_itrs--;
								if(itr->m_evt_type == EVT_PERIOD){
									itr->tabs += itr->period;
								}
							}else{
								cerr << "Failed to send event." << endl;
								itr->m_num_itrs = 0;
							}
						}else if(FD_ISSET(itr->m_sock, &de)){
							cerr << "Socket error in sending event." << endl;
							itr->m_num_itrs = 0;
						}
					}else{
						cerr << "Event notification timeout." << endl;
					}
				}else{
					// delete event
					closesocket(itr->m_sock);
					itr = m_event.erase(itr);
					continue;
				}
			}
			itr++;
		}
		return true;
	}
};
#endif 