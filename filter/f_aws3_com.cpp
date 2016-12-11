#include "stdafx.h"
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws3_com.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws3_com.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws3_com.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <list>
#include <thread>
#include <mutex>

using namespace std;

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include "f_aws3_com.h"


f_aws3_com::f_aws3_com(const char * name) :f_base(name), m_port(14550)
{
	register_fpar("port", &m_port, "UDP port recieving mavlink packets.");
}

f_aws3_com::~f_aws3_com()
{
}

bool f_aws3_com::init_run()
{
  m_sock = socket(AF_INET, SOCK_DGRAM, 0);
  
	m_sock_addr_rcv.sin_family = AF_INET;
	m_sock_addr_rcv.sin_port = htons(m_port);
	set_sockaddr_addr(m_sock_addr_rcv);
	if (::bind(m_sock, (sockaddr*)&m_sock_addr_rcv, sizeof(m_sock_addr_rcv)) == SOCKET_ERROR){
		cerr << "Socket error" << endl;
		return false;
	}

	return true;
}

void f_aws3_com::destroy_run()
{
	closesocket(m_sock);
}

bool f_aws3_com::proc()
{
	fd_set fr, fw, fe;
	timeval tv;
	int res;
	if (m_bcon){
		FD_ZERO(&fw);
		FD_ZERO(&fe);
		FD_SET(m_sock, &fw);
		FD_SET(m_sock, &fe);
		tv.tv_sec = 0;
		tv.tv_usec = 1000;

		/*Send Heartbeat */
		/*
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		*/


		/* Send Status */
		/*
		mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
		*/


		/* Send Local Position */
		/*
		mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(),

		position[0], position[1], position[2],

		position[3], position[4], position[5]);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		*/


		/* Send attitude */
		/*
		mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);

		len = mavlink_msg_to_send_buffer(buf, &msg);

		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		*/
	}

	FD_ZERO(&fr);
	FD_ZERO(&fe);
	FD_SET(m_sock, &fr);
	FD_SET(m_sock, &fe);
	tv.tv_sec = 0;
	tv.tv_usec = 1000;

	memset(m_buf, 0, 2048);
	res = select((int)m_sock + 1, &fr, NULL, &fe, &tv);
	if (FD_ISSET(m_sock, &fr)){
		res = recvfrom(m_sock, (char*)m_buf, 1024, 0, (struct sockaddr *)&m_sock_addr_snd, &m_sz);
		if (res > 0)
		{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;

			printf("Bytes Received: %d\nDatagram: ", (int)res);
			uint8_t temp;

			for (int i = 0; i < res; ++i)
			{
				temp = m_buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, m_buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
			printf("\n");
			m_bcon = true;
		}
		else{
			cerr << "Error.Reopeninng socket." << endl;
			closesocket(m_sock);
			return init_run();
		}
	}
	else if (FD_ISSET(m_sock, &fe)){
		cerr << "Failed to recieve packet." << endl;
	}

	if (m_brst){
		cout << "Reopening socket." << endl;
		closesocket(m_sock);
		return init_run();
		m_brst = m_bcon = false;
	}
	return true;
}
