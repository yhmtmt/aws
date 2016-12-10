#ifndef _F_AWS3_COM_H_
#define _F_AWS3_COM_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws3_com.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws3_com.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws3_com.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../channel/ch_base.h"
#include "f_base.h"

#include <mavlink.h>

class f_aws3_com: f_base
{

protected:

	unsigned short m_port;
	SOCKET m_sock;
	sockaddr_in m_sock_addr_rcv, m_sock_addr_snd;
	socklen_t m_sz;

	uint8_t m_buf[2048];
	
	bool m_brst;
	bool m_bcon;
public:
	f_aws3_com(const char * name);
	virtual ~f_aws3_com();
	
	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

#endif