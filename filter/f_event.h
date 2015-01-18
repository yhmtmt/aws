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
				tmex tm;
				switch(m_evt_type){
				case EVT_TIME:
					if(!decTmStr(m_tstr, tm))
						break;
					evt.tabs = mkgmtimeex(tm);
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

		list<s_event>::iterator itr;
		while(itr != m_event.end()){
			if(itr->tabs < m_cur_time){
				if(itr->m_num_itrs){
					// event occur
					int len = (int) strlen(m_time_str) + 1;
					int res = send(itr->m_sock, m_time_str, len, 0);
					if(len == res){
						itr->m_num_itrs--;
						if(itr->m_evt_type == EVT_PERIOD){
							itr->tabs += itr->period;
						}
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