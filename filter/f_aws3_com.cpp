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
		/*
		FD_ZERO(&fw);
		FD_ZERO(&fe);
		FD_SET(m_sock, &fw);
		FD_SET(m_sock, &fe);
		tv.tv_sec = 0;
		tv.tv_usec = 1000;
		*/
		mavlink_message_t msg;
		mavlink_status_t status;
		uint16_t len;
		/*Send Heartbeat */
		mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

		len = mavlink_msg_to_send_buffer(m_buf, &msg);

		res = sendto(m_sock, (char*)m_buf, len, 0, (struct sockaddr*)&m_sock_addr_snd, sizeof(struct sockaddr_in));

		/* Send controller output */
		uint16_t mask = 0x0001, btns = 0x0000;
		for (int i = 0; i < 16; i++, mask <<= 1)
			btns |= (m_jbtns[i] ? mask : 0);

		// saturation -1000 to 1000
		m_jx = max(min(m_jx, 1000), -1000);
		m_jy = max(min(m_jy, 1000), -1000);
		m_jz = max(min(m_jz, 1000), -1000);
		m_jr = max(min(m_jr, 1000), -1000);

		mavlink_msg_manual_control_pack(1, 1, &msg, 1, 
			(int16_t)m_jx, (int16_t)m_jy, (int16_t)m_jz, (int16_t)m_jr, btns);

		len = mavlink_msg_to_send_buffer(m_buf, &msg);

		res = sendto(m_sock, (char*)m_buf, len, 0, (struct sockaddr*)&m_sock_addr_snd, sizeof(struct sockaddr_in));

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
					switch (msg.msgid){
					case MAVLINK_MSG_ID_HEARTBEAT:
						mavlink_msg_heartbeat_decode(&msg, &m_heartbeat);
						break;
					case MAVLINK_MSG_ID_RAW_IMU:
						mavlink_msg_raw_imu_decode(&msg, &m_raw_imu);
						break;
					case MAVLINK_MSG_ID_SCALED_IMU2:
						mavlink_msg_scaled_imu2_decode(&msg, &m_scaled_imu2);
						break;
					case MAVLINK_MSG_ID_SCALED_PRESSURE:
						mavlink_msg_scaled_pressure_decode(&msg, &m_scaled_pressure);
						break;
					case MAVLINK_MSG_ID_SCALED_PRESSURE2:
						mavlink_msg_scaled_pressure2_decode(&msg, &m_scaled_pressure2);
						break;
					case MAVLINK_MSG_ID_SYS_STATUS:
						mavlink_msg_sys_status_decode(&msg, &m_sys_status);
						break;
					case MAVLINK_MSG_ID_POWER_STATUS:
						mavlink_msg_power_status_decode(&msg, &m_power_status);
						break;
					case MAVLINK_MSG_ID_MISSION_CURRENT:
						mavlink_msg_mission_current_decode(&msg, &m_mission_current);
						break;
					case MAVLINK_MSG_ID_SYSTEM_TIME:
						mavlink_msg_system_time_decode(&msg, &m_system_time);
						break;
					case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
						mavlink_msg_nav_controller_output_decode(&msg, &m_nav_controller_output);
						break;
					case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
						mavlink_msg_global_position_int_decode(&msg, &m_global_position_int);
						break;
					case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
						mavlink_msg_servo_output_raw_decode(&msg, &m_servo_output_raw);
						break;
					case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
						mavlink_msg_rc_channels_raw_decode(&msg, &m_rc_channels_raw);
						break;
					case MAVLINK_MSG_ID_ATTITUDE:
						mavlink_msg_attitude_decode(&msg, &m_attitude);
						break;
					//case MAVLINK_MSG_ID_RALLY_LAND_FETCH_POINT: //Message ID 178 is not appeared in ardupilot mega...
					case MAVLINK_MSG_ID_VFR_HUD:
						mavlink_msg_vfr_hud_decode(&msg, &m_vfr_hud);
						break;
					case MAVLINK_MSG_ID_HWSTATUS:
						mavlink_msg_hwstatus_decode(&msg, &m_hwstatus);
						break;
					case MAVLINK_MSG_ID_MOUNT_STATUS:
						mavlink_msg_mount_status_decode(&msg, &m_mount_status);
						break;
					case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
						mavlink_msg_ekf_status_report_decode(&msg, &m_ekf_status_report);
						break;
					case MAVLINK_MSG_ID_VIBRATION:
						mavlink_msg_vibration_decode(&msg, &m_vibration);
						break;
					case MAVLINK_MSG_ID_SENSOR_OFFSETS:
						mavlink_msg_sensor_offsets_decode(&msg, &m_sensor_offsets);
						break;
					case MAVLINK_MSG_ID_RANGEFINDER:
						mavlink_msg_rangefinder_decode(&msg, &m_rangefinder);
						break;
					case MAVLINK_MSG_ID_RPM:
						mavlink_msg_rpm_decode(&msg, &m_rpm);
						break;
					case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
						mavlink_msg_camera_feedback_decode(&msg, &m_camera_feedback);
						break;
					case MAVLINK_MSG_ID_LIMITS_STATUS:
						mavlink_msg_limits_status_decode(&msg, &m_limits_status);
						break;
					case MAVLINK_MSG_ID_SIMSTATE:
						mavlink_msg_simstate_decode(&msg, &m_simstate);
						break;
					case MAVLINK_MSG_ID_MEMINFO:
						mavlink_msg_meminfo_decode(&msg, &m_meminfo);
						break;
					case MAVLINK_MSG_ID_BATTERY2:
						mavlink_msg_battery2_decode(&msg, &m_battery2);
						break;
					case MAVLINK_MSG_ID_GIMBAL_REPORT:
						mavlink_msg_gimbal_report_decode(&msg, &m_gimbal_report);
						break;
					case MAVLINK_MSG_ID_PID_TUNING:
						mavlink_msg_pid_tuning_decode(&msg, &m_pid_tuning);
						break;
					case MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
						mavlink_msg_mag_cal_progress_decode(&msg, &m_mag_cal_progress);
						break;
					case MAVLINK_MSG_ID_MAG_CAL_REPORT:
						mavlink_msg_mag_cal_report_decode(&msg, &m_mag_cal_report);
						break;
					case MAVLINK_MSG_ID_AHRS:
						mavlink_msg_ahrs_decode(&msg, &m_ahrs);
						break;
					case MAVLINK_MSG_ID_AHRS2:
						mavlink_msg_ahrs2_decode(&msg, &m_ahrs2);
						break;
					case MAVLINK_MSG_ID_AHRS3:
						mavlink_msg_ahrs3_decode(&msg, &m_ahrs3);
						break;
					}
					
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
