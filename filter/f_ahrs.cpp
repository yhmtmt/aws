// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_ahrs.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ahrs.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ahrs.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/aws_serial.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_ahrs.h"

const char * f_ahrs::m_str_razor_cmd[ERC_UNDEF] = {
	"o0", "o1", 
	"ob", "ot", "oc", "on", 
	"osct", "osrt", "osbt", 
	"oscb", "osrb", "osbb",
	"oft", "ofb",
	"f", "saw"
};

f_ahrs::f_ahrs(const char * name): f_base(name), m_state(NULL),
				   m_cmd(ERC_OT), m_omode(ERC_OT), 
	m_ocont(true), m_sync(false), m_rbuf_tail(0), m_rbuf_head(0), m_tbuf_tail(0),
	m_verb(false), m_readlen(1024),
	m_b9dof(true), m_binit_9dof(false), m_magg(256.), m_Kai(0.00002), m_Kap(0.02), m_Kmi(0.00002), m_Kmp(1.2),
  m_max9(1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000), m_min9(-1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000, -1000),
  m_max_ypr(180, 180, 180), m_min_ypr(-180, -180, -180)
{
	m_dname[0] = '\0';

	register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");

	register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
	register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
	register_fpar("rlen", &m_readlen, "Maximum read length of the serial port");
	register_fpar("br", &m_br, "Baud rate.");
	register_fpar("cmd", (int*)&m_cmd, (int)ERC_UNDEF, m_str_razor_cmd, "Command for Razor AHRS.");
	register_fpar("verb", &m_verb, "Verbose mode for Debug");

}

f_ahrs::~f_ahrs()
{
}

bool f_ahrs::init_run()
{
#ifdef _WIN32
	m_hserial = open_serial(m_port, m_br);
#else
	m_hserial = open_serial(m_dname, m_br);
#endif
	if(m_hserial == NULL_SERIAL)
		return false;

	return true;
}

void f_ahrs::destroy_run()
{
	close_serial(m_hserial);
	m_hserial = NULL_SERIAL;
}

bool f_ahrs::proc()
{
	long long t = get_time();
	// command handling
	int cmdlen = set_buf_cmd(m_cmd);
	switch(m_cmd){
	case ERC_O0:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_ocont = false;
		break;
	case ERC_O1:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_cmd = ERC_S;
		m_ocont = true;
		break;
	case ERC_OB:
	case ERC_OT:
	case ERC_OSCT:
	case ERC_OSRT:
	case ERC_OSBT:
	case ERC_OSCB:
	case ERC_OSRB:
	case ERC_OSBB:
	case ERC_OFB:
	case ERC_OFT:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_omode = m_cmd;
		m_cmd = ERC_S;
		break;
	case ERC_S:
		write_serial(m_hserial, m_wbuf, cmdlen);
		m_sync = true;
		break;
	case ERC_OC:
	case ERC_ON:
	case ERC_F:
	default:
		break;
	}

	// sensor output handling	
	if(m_sync){
		m_cmd = ERC_UNDEF;
		// seeking for #SYNCHaw<cr><lf>
		int len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);
		if(len > 0)
		  m_rbuf_tail += len;
		int i, j = 0;
		int tocnt = 0;
		while(1){
			const char * sync_str = "#SYNCHaw\r\n";
			for(i = m_rbuf_head; i < m_rbuf_tail; i++){
				if(m_rbuf[i] == sync_str[j])
					j++;
				else 
					j = 0;
				if(j == 10){
					m_sync = false;
					m_rbuf_head = i + 1;
					break;
				}
			}

			if(i == m_rbuf_tail){
				m_rbuf_head = m_rbuf_tail = 0;
			}

			if(!m_sync)
				break;

			len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);

			if(len > 0)
			  m_rbuf_tail += len;

			tocnt++;
			if(tocnt == 1000000){
			  cout << m_name << "Synchronization failed." << endl;
				m_cmd = ERC_S;
				break;
			}
		}
		
		for(i = m_rbuf_head, j = 0; i < m_rbuf_tail; i++, j++)
			m_rbuf[j] = m_rbuf[i];
		m_rbuf_tail = m_rbuf_tail - m_rbuf_head;
		m_rbuf_head = 0;
	}else{	
	  int len = read_serial(m_hserial, m_rbuf + m_rbuf_tail, m_readlen - m_rbuf_tail);

	  if(len > 0)
	    m_rbuf_tail += len;
	}

	if(m_cmd == ERC_S)
		return true;

	switch(m_omode){
	case ERC_OB: 
		// total 12byte
		// <Y 4byte> <P 4byte> <R 4byte> 
		set_bin_ypr();
		break;
	case ERC_OT:
		// YPR=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSCT:
		// A-C=xxxx,xxxx,xxxx<cr><lf> 
		// M-C=xxxx,xxxx,xxxx<cr><lf>
		// G-C=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSRT:
		// A-R=xxxx,xxxx,xxxx<cr><lf> 
		// M-R=xxxx,xxxx,xxxx<cr><lf>
		// G-R=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OSBT:
		// A-R=xxxx,xxxx,xxxx<cr><lf> 
		// M-R=xxxx,xxxx,xxxx<cr><lf>
		// G-R=xxxx,xxxx,xxxx<cr><lf>
		// A-C=xxxx,xxxx,xxxx<cr><lf> 
		// M-C=xxxx,xxxx,xxxx<cr><lf>
		// G-C=xxxx,xxxx,xxxx<cr><lf>
	case ERC_OFT:
		// YPR=xxxx,xxxx,xxxx<cr><lf>
		// A-C=xxxx,xxxx,xxxx<cr><lf> 
		// M-C=xxxx,xxxx,xxxx<cr><lf>
		// G-C=xxxx,xxxx,xxxx<cr><lf>

		while(seek_txt_buf()){
			dec_tok_val();
		}
		break;
	case ERC_OSCB:
		// total 36byte
		// <A-C x 4byte> <A-C y 4byte> <A-C z 4byte>
		// <M-C x 4byte> <M-C y 4byte> <M-C z 4byte>
		// <G-C x 4byte> <G-C y 4byte> <G-C z 4byte>
		set_bin_9(&m_cal);
		break;
	case ERC_OSRB:
		// total 36byte
		// <A-R x 4byte> <A-R y 4byte> <A-R z 4byte>
		// <M-R x 4byte> <M-R y 4byte> <M-R z 4byte>
		// <G-R x 4byte> <G-R y 4byte> <G-R z 4byte>
		set_bin_9(&m_raw);
		break;
	case ERC_OSBB:
		// total 72byte
		// <A-R x 4byte> <A-R y 4byte> <A-R z 4byte>
		// <M-R x 4byte> <M-R y 4byte> <M-R z 4byte>
		// <G-R x 4byte> <G-R y 4byte> <G-R z 4byte>
		// <A-C x 4byte> <A-C y 4byte> <A-C z 4byte>
		// <M-C x 4byte> <M-C y 4byte> <M-C z 4byte>
		// <G-C x 4byte> <G-C y 4byte> <G-C z 4byte>
		set_bin_9x2(&m_raw, &m_cal);
		break;
	case ERC_OFB:
		set_bin_ypr9(&m_cal);
		break;
	}

	bool bypr = false;
	bool braw = false;
	bool bcal = false;
	switch (m_omode){
	case ERC_OSRT:
	case ERC_OSRB:
	case ERC_OSBB:
	case ERC_OSBT:
		braw = true;
	}

	switch (m_omode){
	case ERC_OSCB:
	case ERC_OSCT:
	case ERC_OSBB:
	case ERC_OSBT:
	case ERC_OFT:
	case ERC_OFB:
		bcal = true;
	}

	switch (m_omode){
	case ERC_OT:
	case ERC_OB:
	case ERC_OFB:
	case ERC_OFT:
		bypr = true;
	}

	if(m_verb){
		if(braw){
			cout << "A-R=" << m_raw.ax << " " << m_raw.ay << " " << m_raw.az << " ";
			cout << "M-R=" << m_raw.mx << " " << m_raw.my << " " << m_raw.mz << " ";
			cout << "G-R=" << m_raw.gx << " " << m_raw.gy << " " << m_raw.gz << " ";
		}

		if(bcal){
			cout << "A-C=" << m_cal.ax << " " << m_cal.ay << " " << m_cal.az << " ";
			cout << "M-C=" << m_cal.mx << " " << m_cal.my << " " << m_cal.mz << " ";
			cout << "G-C=" << m_cal.gx << " " << m_cal.gy << " " << m_cal.gz << " ";
		}

		if(bypr){
			cout << "YPR=" << m_ypr.y << " " << m_ypr.p << " " << m_ypr.r;
		}
		cout << endl;
	}

	if(m_state){	
		if (bcal){
			m_state->set_9dof(t, m_cal.mx, m_cal.my, m_cal.mz, m_cal.ax, m_cal.ay, m_cal.az, m_cal.gx, m_cal.gy, m_cal.gz);
			if (!bypr){
				m_cal.gx *= (GYRO_GAIN * PI / 180.);
				m_cal.gy *= (GYRO_GAIN * PI / 180.);
				m_cal.gz *= (GYRO_GAIN * PI / 180.);
				if (!m_binit_9dof){
					if (init_9dof(t, m_cal.mx, m_cal.my, m_cal.mz, m_cal.ax, m_cal.ay, m_cal.az, m_cal.gx, m_cal.gy, m_cal.gz))
						m_binit_9dof = true;
				}
				else{
					update_9dof(t, m_cal.mx, m_cal.my, m_cal.mz, m_cal.ax, m_cal.ay, m_cal.az, m_cal.gx, m_cal.gy, m_cal.gz);
				}
				float rd = m_roll * (180. / PI), pd = m_pitch * (180. / PI), yd = m_yaw  * (180. / PI);
				m_state->set_attitude(t, rd, pd, yd);
			}
		}

		if (!bcal && braw){
			m_state->set_9dof(t, m_raw.mx, m_raw.my, m_raw.mz, m_raw.ax, m_raw.ay, m_raw.az, m_raw.gx, m_raw.gy, m_raw.gz);
		}

		if (bypr)
			m_state->set_attitude(t, m_ypr.r, m_ypr.p, m_ypr.y);	  
	}

  if (!check_vals(bypr, braw, bcal)){
    m_binit_9dof = false;
    m_cmd = ERC_S;
    cerr << "AHRS may not be synchronised. Trying sync." << endl;
  }

	return true;
}

int f_ahrs::seek_txt_buf()
{
	char * ptbuf = m_tbuf + m_tbuf_tail;
	char * prbuf = m_rbuf + m_rbuf_head;

	int rbuflen = m_rbuf_tail - m_rbuf_head; // length of the string remained in the rbuf
	int len = 0; // length of the string read from rbuf.

	if(m_tbuf_tail == 0){
		// seeking for the token head
		for(;*prbuf != '#' && len < rbuflen; prbuf++, len++);

		if(len == rbuflen){
			m_rbuf_tail = m_rbuf_head = 0;
			return 0;
		}

		*ptbuf = *prbuf; ptbuf++; prbuf++; len++;

		if(len == rbuflen){
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += 1;
			return 0;
		}

		m_tbuf_tail = 1;
		m_rbuf_head += len;
		rbuflen -= len;
		len = 0;
	}

	int tbuflen = m_readlen - m_tbuf_tail;
	int buflen = min(rbuflen, tbuflen);

	// Seeking for CRLF, and extract the token.
	while(1){
		// seeking for CR
		for(;*prbuf != 0x0d && len < buflen; *ptbuf = *prbuf, ptbuf++, prbuf++, len++);

		if(len == rbuflen){ // read buffer is ran out
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += len;
			return 0;
		}else if(len == tbuflen){ // token buffer is over
			m_tbuf_tail = 0;
		}

		*ptbuf = *prbuf; ptbuf++; prbuf++; len++;

		if(len == rbuflen){ // read buffer is ran out
			m_rbuf_tail = m_rbuf_head = 0;
			m_tbuf_tail += len;
			return 0;
		}else if(len == tbuflen){ // token buffer is over
			m_tbuf_tail = 0;
		}

		if(*prbuf == 0x0a){ // LF  is found
			*ptbuf = *prbuf; ptbuf++; prbuf++; len++;
			*ptbuf = '\0';
			m_rbuf_head += len;
			len += m_tbuf_tail;
			m_tbuf_tail = 0;
			return len;
		}
	}

	return 0;
}


bool f_ahrs::init_9dof(const long long t,
	const float mx, const float my, const float mz, const float ax, const float ay,
	const float az, const float gx, const float gy, const float gz)
{
	m_t9dof_prev = t;

	m_pitch = -atan2(ax, sqrt(ay * ay + az * az));
	m_cpitch = cos(m_pitch);
	m_spitch = sin(m_pitch);

	//	float x1, y1, z1, x2, y2, z2;
	//	calc_xproduct(ax, ay, az, 1, 0, 0, x1, y1, z1); // x1 <- 0 y1 <- az z1 <- -ay
	//	calc_xproduct(1, 0, 0, x1, y1, z1, x2, y2, z2); // x2 <- 0 y2 <- ay z2 <- az
	m_roll = atan2(ay, az);
	m_croll = cos(m_roll);
	m_sroll = sin(m_roll);

	m_yaw = calc_yaw(mx, my, mz);
	m_cyaw = cos(m_yaw);
	m_syaw = sin(m_yaw);

	// Init rotation matrix
	m_DCM = Mat::zeros(3, 3, CV_32FC1);
	float * pDCM = m_DCM.ptr<float>();
	pDCM[0] = m_cyaw * m_cpitch;
	pDCM[1] = m_syaw * m_cpitch;
	pDCM[2] = -m_spitch;

	pDCM[3] = m_cyaw * m_spitch * m_sroll - m_syaw * m_croll;
	pDCM[4] = m_syaw * m_spitch * m_sroll + m_cyaw * m_croll;
	pDCM[5] = m_cpitch * m_sroll;

	pDCM[6] = m_cyaw * m_spitch * m_croll + m_syaw * m_sroll;
	pDCM[7] = m_syaw * m_spitch * m_croll - m_cyaw * m_sroll;
	pDCM[8] = m_cpitch * m_croll;
	float rd = m_roll * (180. / PI);
	float pd = m_pitch * (180. / PI);
	float yd = m_yaw * (180. / PI);

	m_waxi = m_wayi = m_wazi = 0.0;
	m_waxp = m_wayp = m_wazp = 0.0;
	m_wmxi = m_wmyi = m_wmzi = 0.0;
	m_wmxp = m_wmyp = m_wmzp = 0.0;

	return true;
}

bool f_ahrs::update_9dof(const long long t,
	const float mx, const float my, const float mz,
	const float ax, const float ay, const float az,
	const float gx, const float gy, const float gz)
{
	float dt = (float)((t - m_t9dof_prev) * (1.0 / (double)SEC));
	m_t9dof_prev = t;
	m_yaw = calc_yaw(mx, my, mz);
	m_cyaw = cos(m_yaw);
	m_syaw = sin(m_yaw);

	// Updating DCM 
	m_wx = m_waxi + m_waxp + m_wmxi + m_wmxp - gx;
	m_wy = m_wayi + m_wayp + m_wmyi + m_wmyp - gy;
	m_wz = m_wazi + m_wazp + m_wmzi + m_wmzp - gz;

	Mat W = Mat::zeros(3, 3, CV_32FC1);
	float * pW = W.ptr<float>();
	pW[1] = (float)(-dt * m_wz);
	pW[2] = (float)(dt * m_wy);
	pW[3] = (float)(dt * m_wz);
	pW[5] = (float)(-dt * m_wx);
	pW[6] = (float)(-dt * m_wy);
	pW[7] = (float)(dt * m_wx);
	/*
	pW[1] = (float)(dt * m_wz);
	pW[2] = (float)(-dt * m_wy);
	pW[3] = (float)(-dt * m_wz);
	pW[5] = (float)(dt * m_wx);
	pW[6] = (float)(dt * m_wy);
	pW[7] = (float)(-dt * m_wx);
	*/
	m_DCM += W * m_DCM;

	// Normalizing DCM [I, J, K]
	float * pDCM = m_DCM.ptr<float>();

	// calculate -0.5I^tJ
	float err = (float)(-0.5 * (pDCM[0] * pDCM[1] + pDCM[3] * pDCM[4] + pDCM[6] * pDCM[7]));
	float I[3], J[3], K[3];
	I[0] = pDCM[0] + err * pDCM[1];
	I[1] = pDCM[3] + err * pDCM[4];
	I[2] = pDCM[6] + err * pDCM[7];
	J[0] = pDCM[1] + err * pDCM[0];
	J[1] = pDCM[4] + err * pDCM[3];
	J[2] = pDCM[7] + err * pDCM[6];
	calc_xproduct(I[0], I[1], I[2], J[0], J[1], J[2], K[0], K[1], K[2]);

	float scale;
	scale = 0.5 * (3. - (I[0] * I[0] + I[1] * I[1] + I[2] * I[2]));
	pDCM[0] = scale * I[0];
	pDCM[3] = scale * I[1];
	pDCM[6] = scale * I[2];

	scale = 0.5 * (3. - (J[0] * J[0] + J[1] * J[1] + J[2] * J[2]));
	pDCM[1] = scale * J[0];
	pDCM[4] = scale * J[1];
	pDCM[7] = scale * J[2];

	scale = 0.5 * (3. - (K[0] * K[0] + K[1] * K[1] + K[2] * K[2]));
	pDCM[2] = scale * K[0];
	pDCM[5] = scale * K[1];
	pDCM[8] = scale * K[2];

	// acceleration based correction
	// Calculate the magnitude of the accelerometer vector
	float wax, way, waz;
	float maga = sqrt(ax * ax + ay * ay + az * az) * (1.0 / m_magg);
	float sa = min(1.0, max(0.0, 1.0 - 2.0 * abs(1.0 - maga)));
//	calc_xproduct(ax, ay, az, pDCM[2], pDCM[5], pDCM[8], wax, way, waz);
	calc_xproduct(pDCM[2], pDCM[5], pDCM[8], ax, ay, az, wax, way, waz);
	m_waxp = sa * m_Kap * wax;
	m_wayp = sa * m_Kap * way;
	m_wazp = sa * m_Kap * waz;
	m_waxi += sa * m_Kai * wax;
	m_wayi += sa * m_Kai * way;
	m_wazi += sa * m_Kai * waz;

	// magnet based correction
	float wmx, wmy, wmz;
	float sm = -(pDCM[0] * m_syaw) + (pDCM[1] * m_cyaw);
	wmx = sm * pDCM[2];
	wmy = sm * pDCM[5];
	wmz = sm * pDCM[8];

	m_wmxp = m_Kmp * wmx;
	m_wmyp = m_Kmp * wmy;
	m_wmzp = m_Kmp * wmz;
	m_wmxi += m_Kmi * wmx;
	m_wmyi += m_Kmi * wmy;
	m_wmzi += m_Kmi * wmz;

	// recalculate roll, pitch, yaw
	m_roll = atan2(pDCM[5], pDCM[8]);
	m_pitch = asin(-pDCM[2]);
	m_yaw = atan2(pDCM[1], pDCM[0]);
	m_sroll = sin(m_roll);
	m_croll = cos(m_roll);
	m_spitch = sin(m_pitch);
	m_cpitch = cos(m_pitch);
	return true;
}
