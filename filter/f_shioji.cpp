#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_shioji.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_shioji.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_shioji.cpp.  If not, see <http://www.gnu.org/licenses/>. 


#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "../util/aws_sock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/thread_util.h"

#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_navdat.h"

#include "f_base.h"
#include "f_shioji.h"

f_shioji::f_shioji(const char * name):f_base(name), m_bfile(false), m_bredir(false), m_sth_int(50000000), m_ctrl_int(10000000)
{
}

f_shioji::~f_shioji()
{
}

bool f_shioji::seek(long long seek_time)
{
	seek_time += m_offset_time;
	if(!m_bfile)
		return true;

	m_file.seekg(0);
	m_file.clear(ios_base::goodbit);
	m_sys_time = 0;
	ch_navdat * pshipout = dynamic_cast<ch_navdat*>(m_chout[0]);
	if(pshipout == NULL)
		return false;

	pshipout->m_sec = -1.;

	char d, h, m;
	float s;
	long long inc;
	getrec();
	dectime(d, h, m, s);
	m_sys_time = 0;
	while(getrec()){
		pshipout->m_sec = s;
		pshipout->m_min = m;
		pshipout->m_hour = h;
		pshipout->m_day = d;

		dectime(d, h, m, s);

		inc  = (long long) ((s - pshipout->m_sec) * SEC);
		inc += (long long) ((long long)(m - pshipout->m_min) * 60LL * SEC);
		inc += (long long) ((long long)(h - pshipout->m_hour) * 3600LL * SEC);
		if(inc < 0)
			inc += 24LL * 3600LL * SEC;

		if(m_sys_time + inc > seek_time)
			break;
		m_sys_time += inc;
	}

	return true;
}

void f_shioji::ctrl(){
	ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chin[0]);
	if(!pctrl)
		return;
	pctrl->get(m_ctrl);
	sat_ctrl(m_ctrl);
	sendto(m_ctrl_sock, (const char *) &m_ctrl, sizeof(s_ship_ctrl), 0, 
		(sockaddr *) &m_ctrl_addr, sizeof(m_ctrl_addr));
}

void f_shioji::sat_ctrl(s_ship_ctrl & ctrl){
	ch_navdat * pshipout = dynamic_cast<ch_navdat*>(m_chout[0]);

	if(pshipout->m_emspd > 3){
		ctrl.CPP_com = max(0., ctrl.CPP_com);
		ctrl.B_th_com = 0;
		ctrl.Ss_th_com = 0;
		ctrl.Sp_th_com = 0;
	}else{
		ctrl.B_th_com = min(ctrl.B_th_com, 15.);
		ctrl.B_th_com = max(ctrl.B_th_com, -15.);
		ctrl.Ss_th_com = min(ctrl.Ss_th_com, 1000.);
		ctrl.Ss_th_com = max(ctrl.Ss_th_com, -1000.);
	}

	ctrl.CPP_com = min(15., ctrl.CPP_com);
	ctrl.CPP_com = max(-5., ctrl.CPP_com);

	if(pshipout->m_emspd < 5){
		ctrl.Rud_com = min(ctrl.Rud_com, 35.);
		ctrl.Rud_com = max(ctrl.Rud_com, -35.);
	}else if(pshipout->m_emspd < 7){
		ctrl.Rud_com = min(ctrl.Rud_com, 20.);
		ctrl.Rud_com = max(ctrl.Rud_com, -20.);
	}else if(pshipout->m_emspd < 10){
		ctrl.Rud_com = min(ctrl.Rud_com, 15.);
		ctrl.Rud_com = max(ctrl.Rud_com, -15.);
	}else{
		ctrl.Rud_com = min(ctrl.Rud_com, 10.);
		ctrl.Rud_com = max(ctrl.Rud_com, -10.);
	}

	if((pshipout->m_sth < 0  && ctrl.Ss_th_com > 0) ||
		(pshipout->m_sth > 0 && ctrl.Ss_th_com < 0)){
			ctrl.Ss_th_com = 0.;
	}
}

bool f_shioji::open_ctrl(const char * url, const char * port){
	m_ctrl_addr.sin_family = AF_INET;
	m_ctrl_addr.sin_addr.s_addr = inet_addr(url);
	m_ctrl_addr.sin_port = htons((unsigned short) atoi(port));
	m_ctrl_sock = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
	m_bctrl = true;
	return true;
}

void f_shioji::close_ctrl()
{
	if(m_bctrl){
		m_bctrl = false;
		closesocket(m_ctrl_sock);
	}
}

bool f_shioji::open_redir(const char * url, const char * port)
{
	m_raddr.sin_family = AF_INET;
	m_raddr.sin_addr.s_addr = inet_addr(url);
	m_raddr.sin_port = htons((unsigned short) atoi(port));
	m_rsock = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
	m_bredir = true;
	return true;
}

void f_shioji::close_redir()
{
	if(m_bredir){
		m_bredir = false;
		closesocket(m_rsock);
	}
}


bool f_shioji::open(const char * url, const char * port)
{
	m_sock = socket(AF_INET, SOCK_DGRAM, 0);
	m_sock_addr.sin_family = AF_INET;
	m_sock_addr.sin_port = htons((unsigned short) atoi(port));
	m_sock_addr.sin_addr.s_addr = INADDR_ANY;
	if(::bind(m_sock, (sockaddr*)&m_sock_addr, sizeof(m_sock_addr)) == SOCKET_ERROR)
		return false;
	m_bfile = false;
	m_svr_sock_addr.sin_family = AF_INET;
	m_svr_sock_addr.sin_port = htons((unsigned short) atoi(port));
	m_svr_sock_addr.sin_addr.s_addr = inet_addr(url);
	m_size_svr_sock_addr = sizeof(m_svr_sock_addr);
	return true;
}

bool f_shioji::open(const char * fname)
{
	m_bfile = true;
	m_file.open(fname);
	return m_file.is_open();
}

void f_shioji::close()
{
	if(m_bfile)
		m_file.close();
	else{
		closesocket(m_sock);
	}
	close_ctrl();
	close_redir();	
}

bool f_shioji::getrec()
{
	if(m_bfile){
		if(m_file.eof())
			return false;

		m_file.getline(m_buf, sizeof(m_buf));
		char * ptr = m_buf;
		int i = 0;
		while(i < 43){
			m_rec[i] = ptr;
			for(;*ptr!=',' && *ptr != '\n' && *ptr != '\0';ptr++);
			*ptr = '\0';
			ptr++;
			i++;
		}
	}else{
		size_t len = 0;
		size_t tot_len = 0;
		while(tot_len < 384) // shioji lan sends 384bytes / packet
		{
#ifdef _WIN32
			len = recvfrom(m_sock, &m_buf[len], 
				(int) (sizeof(m_buf)-len), 0, 
				(sockaddr*)&m_svr_sock_addr, &m_size_svr_sock_addr);
#else
			len = recvfrom(m_sock, &m_buf[len], 
				(int) (sizeof(m_buf)-len), 0, 
				(sockaddr*)&m_svr_sock_addr, (socklen_t*) &m_size_svr_sock_addr);
#endif


			if(len == 0)
				cout << "Connection was closed." << endl;

			if(len == -1)
				tot_len = 0;
			else
				tot_len += len;
		}
	}

	decrec();

	return true;
}

void f_shioji::dectime(char & d, char & h, char & m, float & s)
{
	char * t = m_rec[2];
	d = (t[1]-'0') * 10 + t[2]-'0';
	h = (t[4]-'0') * 10 + t[5]-'0';
	m = (t[7]-'0') * 10 + t[8]-'0';
	s = (float) atof(&t[10]);
}

void f_shioji::decrec()
{
	ch_navdat * pshipout = dynamic_cast<ch_navdat*>(m_chout[0]);
	pshipout->lock();
	if(m_bfile){
		pshipout->m_emspd = atof(m_rec[3]);
		pshipout->m_windir = atof(m_rec[4]) * (PI / 180);
		pshipout->m_winspd = atof(m_rec[5]);
		pshipout->m_cpp = atof(m_rec[11]) * (PI / 180);
		pshipout->m_rud = atof(m_rec[12]) * (PI / 180);
		pshipout->m_bth = atof(m_rec[13]) * (PI / 180);
		pshipout->m_sth = atof(m_rec[14]);
		pshipout->m_hdg = atof(m_rec[19]) * (PI / 180);
		pshipout->m_lon = atof(m_rec[20]) * (PI / 180);
		pshipout->m_lat = atof(m_rec[21]) * (PI / 180);
		pshipout->m_alt = atof(m_rec[22]);
		pshipout->m_gpdir = atof(m_rec[23]) * (PI / 180);
		pshipout->m_gpspd = atof(m_rec[24]);
		pshipout->m_roll = atof(m_rec[25]) * (PI / 180);
		pshipout->m_pitch = atof(m_rec[26]) * (PI / 180);
		pshipout->m_yaw = atof(m_rec[27]) * (PI / 180);
		pshipout->m_asurge = atof(m_rec[28]);
		pshipout->m_asway = atof(m_rec[29]);
		pshipout->m_aheave = atof(m_rec[30]);
		pshipout->m_arroll = atof(m_rec[31]) * (PI / 180);
		pshipout->m_arpitch = atof(m_rec[32]) * (PI / 180);
		pshipout->m_aryaw = atof(m_rec[33]) * (PI / 180);
		pshipout->set_done();
	}else{
		pshipout->m_day = (m_buf[49]-'0')*10 + m_buf[50]-'0';
		pshipout->m_hour = (m_buf[52]-'0')*10 + m_buf[53]-'0';
		pshipout->m_min = (m_buf[55]-'0')*10 + m_buf[56]-'0';
		pshipout->m_sec = (float)(
			(m_buf[58]-'0')*10 
			+ m_buf[59]-'0' 
			+ (double)(m_buf[61]-'0')*0.1 
			+ (double)(m_buf[62]-'0')*0.01
			+ (double)(m_buf[63]-'0')*0.001);

		pshipout->m_emspd = *((double*) &m_buf[64]);
		pshipout->m_windir = *((double*) &m_buf[72]) * (PI / 180);
		pshipout->m_winspd = *((double*) &m_buf[80]);
		pshipout->m_cpp = *((double*) & m_buf[128]) * (PI / 180);
		pshipout->m_rud = *((double*) & m_buf[136]) * (PI / 180);
		pshipout->m_bth = *((double*) & m_buf[144]) * (PI / 180);
		pshipout->m_sth = *((double*) & m_buf[152]);
		pshipout->m_hdg = *((double*) &m_buf[192]) * (PI / 180);
		pshipout->m_lat = *((double*) &m_buf[200]) * (PI / 180);
		pshipout->m_lon = *((double*) &m_buf[208]) * (PI / 180);
		pshipout->m_alt = *((double*) &m_buf[216]);
		pshipout->m_gpdir = *((double*) &m_buf[224]) * (PI / 180);
		pshipout->m_gpspd = *((double*) &m_buf[232]);
		pshipout->m_roll = *((double*) &m_buf[240]) * (PI / 180);
		pshipout->m_pitch = *((double*) &m_buf[248]) * (PI / 180);
		pshipout->m_yaw = *((double*) &m_buf[256]) * (PI / 180);
		pshipout->m_asurge = *((double*) &m_buf[264]);
		pshipout->m_asway = *((double*) &m_buf[272]);
		pshipout->m_aheave = *((double*) &m_buf[280]);
		pshipout->m_arroll = *((double*) &m_buf[288]) * (PI / 180);
		pshipout->m_arpitch = *((double*) &m_buf[296]) * (PI / 180);
		pshipout->m_aryaw = *((double*) &m_buf[304]) * (PI / 180);
		pshipout->set_done();
	}

	if(m_bredir){
		redir();
	}

	pshipout->unlock();
}

void f_shioji::pack_buf_redir()
{
	ch_navdat * pshipout = dynamic_cast<ch_navdat*>(m_chout[0]);
	memset(m_buf_redir, 0, 384);
	m_buf_redir[49] = pshipout->m_day / 10 + '0';
	m_buf_redir[50] = pshipout->m_day % 10 + '0';
	m_buf_redir[52] = pshipout->m_hour / 10 + '0';
	m_buf_redir[53] = pshipout->m_hour % 10 + '0';
	m_buf_redir[55] = pshipout->m_min / 10 + '0';
	m_buf_redir[56] = pshipout->m_min % 10 + '0';
	{
		int t10, t1, t01, t001, t0001;
		t10 = (int) (pshipout->m_sec * 1000);
		t0001 = t10 % 10;
		t10 /= 10;
		t001 = t10 % 10;
		t10 /= 10;
		t01 = t10 % 10;
		t10 /= 10;
		t1 = t10 %10;
		t10 /= 10;
		m_buf_redir[58] = '0' + (char) t10;
		m_buf_redir[59] = '0' + (char) t1;
		m_buf_redir[61] = '0' + (char) t01;
		m_buf_redir[62] = '0' + (char) t001;
		m_buf_redir[63] = '0' + (char) t0001;
	}

	*((double*) (&m_buf_redir[64])) = pshipout->m_emspd;
	*((double*) (&m_buf_redir[72])) = pshipout->m_windir;
	*((double*) (&m_buf_redir[80])) = pshipout->m_winspd;
	*((double*) (&m_buf_redir[128])) = pshipout->m_cpp * (180 / PI);
	*((double*) (&m_buf_redir[136])) = pshipout->m_rud * (180 / PI);
	*((double*) (&m_buf_redir[144])) = pshipout->m_bth * (180 / PI);
	*((double*) (&m_buf_redir[152])) = pshipout->m_sth;
	*((double*) (&m_buf_redir[192])) = pshipout->m_hdg * (180 / PI);
	*((double*) (&m_buf_redir[200])) = pshipout->m_lat * (180 / PI);
	*((double*) (&m_buf_redir[208])) = pshipout->m_lon * (180 / PI);
	*((double*) (&m_buf_redir[216])) = pshipout->m_alt;
	*((double*) (&m_buf_redir[224])) = pshipout->m_gpdir;
	*((double*) (&m_buf_redir[232])) = pshipout->m_gpspd;
	*((double*) (&m_buf_redir[240])) = pshipout->m_roll * (180 / PI);
	*((double*) (&m_buf_redir[248])) = pshipout->m_pitch * (180 / PI);
	*((double*) (&m_buf_redir[256])) = pshipout->m_yaw * (180 / PI);
	*((double*) (&m_buf_redir[264])) = pshipout->m_asurge;
	*((double*) (&m_buf_redir[272])) = pshipout->m_asway;
	*((double*) (&m_buf_redir[280])) = pshipout->m_aheave;
	*((double*) (&m_buf_redir[288])) = pshipout->m_arroll * (180 / PI);
	*((double*) (&m_buf_redir[296])) = pshipout->m_arpitch * (180 / PI);
	*((double*) (&m_buf_redir[304])) = pshipout->m_aryaw * (180 / PI);
}

void f_shioji::redir()
{
	pack_buf_redir();
	sendto(m_rsock, m_buf_redir, 384, 0, 
		(sockaddr *) &m_raddr, sizeof(m_raddr));
}

bool f_shioji::run(long long start_time, long long end_time)
{
	if(!seek(start_time))
		return false;
	return f_base::run(start_time, end_time);
}

bool f_shioji::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "copen") == 0){
		if(num_args == 5)
			return open(args[itok+1], args[itok+2]);
		return false;
	}else if(strcmp(args[itok], "fopen") == 0){
		if(num_args == 4)
			return open(args[itok+1]);
		return false;
	}else if(strcmp(args[itok], "redir") == 0){
		if(num_args == 5)
			return open_redir(args[itok+1], args[itok+2]);
		else
			close_redir();
		return true;
	}else if(strcmp(args[itok], "ctrl_int") == 0){
		if(num_args == 4){
			m_ctrl_int = atoi(args[itok+1]) * 10000000;
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "ctrl") == 0){
		if(num_args == 5)
			return open_ctrl(args[itok+1], args[itok+2]);
		else
			close_redir();
		return true;
	}else if(strcmp(args[itok], "rud") == 0){
		if(num_args == 4){
			ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chin[0]);
			if(!pctrl)
				return false;		
			pctrl->get(m_ctrl);
			m_ctrl.Rud_com = atof(args[itok+1]);
			pctrl->set(m_ctrl);
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "cpp") == 0){
		if(num_args == 4){
			ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chin[0]);
			if(!pctrl)
				return false;		
			pctrl->get(m_ctrl);
			m_ctrl.CPP_com = atof(args[itok+1]);
			pctrl->set(m_ctrl);
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "bth") == 0){
		if(num_args == 4){
			ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chin[0]);
			if(!pctrl)
				return false;		
			pctrl->get(m_ctrl);
			m_ctrl.B_th_com = atof(args[itok+1]);
			pctrl->set(m_ctrl);
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "sth") == 0){
		if(num_args == 4){
			ch_ship_ctrl * pctrl = dynamic_cast<ch_ship_ctrl*>(m_chin[0]);
			if(!pctrl)
				return false;		
			pctrl->get(m_ctrl);
			double S_th = atof(args[itok+1]);
			if(S_th < 0){
				m_ctrl.Sp_th_com = -S_th;
				m_ctrl.Ss_th_com = 0;
			}else{
				m_ctrl.Sp_th_com = 0;
				m_ctrl.Ss_th_com = S_th;
			}
			pctrl->set(m_ctrl);
			return true;
		}
		return false;
	}

	return f_base::cmd_proc(cmd);
}

bool f_shioji::proc()
{
	ch_navdat * pshipout = dynamic_cast<ch_navdat*>(m_chout[0]);
	if(pshipout == NULL)
		return false;
	
	long long cur_time = m_cur_time + m_offset_time;

	// control 
	if(m_bctrl && m_cur_time > m_ctrl_int + m_ctrl_prev_time){
		ctrl();
		m_ctrl_prev_time += m_ctrl_int;
	}

	if(m_bdump){
		cout << "Control: " << m_ctrl.Rud_com << " " 
			<< m_ctrl.CPP_com << " " << m_ctrl.Ss_th_com << " " << m_ctrl.Sp_th_com
			<< " " << m_ctrl.B_th_com << " " << endl;
		pshipout->dump(cout);
	}

	if(m_bfile){
		if(m_file.eof())
			return false;

		char d, h, m;
		float s;
		long long inc;
		bool update = false;
		while(1){
			dectime(d, h, m, s);
			inc  = (long long) ((s - pshipout->m_sec) * SEC);
			inc += (long long) ((long long)(m - pshipout->m_min) * 60LL * SEC);
			inc += (long long) ((long long)(h - pshipout->m_hour) * 3600LL * SEC);
			if(inc < 0)
				inc += 24LL * 3600LL * SEC;

			if(m_sys_time + inc > cur_time)
				break;
			pshipout->m_sec = s;
			pshipout->m_min = m;
			pshipout->m_hour = h;
			pshipout->m_day = d;

			update = true;
			m_sys_time += inc;
			getrec();
		}

		if(!update) 
			return true;
	}else{
		getrec();
	}

	return true;
}