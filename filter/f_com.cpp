// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_com.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_com.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_com.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/thread_util.h"
#include "../util/c_clock.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"

#include "f_base.h"
#include "f_com.h"

const char * f_trn_img::get_err_msg(int code)
{
	const char * msg = f_base::get_err_msg(code);
	if(msg)
		return msg;

	switch(code){
	case FERR_TRN_IMG_CHAN:
		return "f_trn_img should have an ch_image  as the input." ;
	case FERR_TRN_IMG_SOCK_SVR:
		return "Error in server socket.";
	case FERR_TRN_IMG_SOCK_SVR_CON:
		return "Error in accepting client connection.";
	}
	return NULL;
}

bool f_trn_img::init_run()
{
	if(m_chin.size() == 1){ 
		m_pimgin = dynamic_cast<ch_image*>(m_chin[0]);
	}

	if(m_pimgin == NULL){
		f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_CHAN);
		return false;
	}

	m_sock_svr = socket(AF_INET, SOCK_STREAM, 0);
	m_sock_svr_addr.sin_family = AF_INET;
	m_sock_svr_addr.sin_port = htons(m_svr_port);
	set_sockaddr_addr(m_sock_svr_addr);

	if(::bind(m_sock_svr, (sockaddr*) &m_sock_svr_addr,
		sizeof(m_sock_svr_addr)) == SOCKET_ERROR){
			f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_SOCK_SVR);
			return false;
	}

	if(::listen(m_sock_svr, 1) == SOCKET_ERROR){
		f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_SOCK_SVR);
		return false;
	}

	return true;
}

bool f_trn_img::wait_connection()
{
	fd_set fr, fe;
	timeval tv;
	int itr = 0;
	while(1){
		FD_ZERO(&fr);
		FD_ZERO(&fe);
		FD_SET(m_sock_svr, &fr);
		FD_SET(m_sock_svr, &fe);
		tv.tv_sec = 3;
		tv.tv_usec = 0;

		if(select((int) m_sock_svr + 1, &fr, NULL, &fe, &tv)){
			if(FD_ISSET(m_sock_svr, &fr)){
				m_sz_sock_client_addr = sizeof(m_sock_client_addr);
				m_sock_client = accept(m_sock_svr, 
					(sockaddr*) &m_sock_client_addr, &m_sz_sock_client_addr);
				if(m_sock_client == SOCKET_ERROR){
					f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_SOCK_SVR_CON);
					return false;
				}
				cerr << "Server connection established." << endl;
				m_bconnected = true;
				break;
			}else{
				f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_SOCK_SVR_CON);
				return false;
			}
		}else{
			cerr << "Waiting connection." << endl;
			itr++;
		}

		if(itr == 10){
			return false;
		}
	}
	return true;
}

void f_trn_img::destroy_run()
{
	closesocket(m_sock_client);
	closesocket(m_sock_svr);
}

bool f_trn_img::proc()
{
	if(is_pause()){
		return true;
	}

	if(!m_bconnected){
		if(!wait_connection()){
			m_bconnected = false;
			return true;
		}
	}

	long long timg;
	
	Mat img = m_pimgin->get_img(timg);

	if(img.empty()){
		return true;
	}

	Size sz = img.size();

	Mat data;
	if(m_scale < 1.0){
		sz.width = (int)(sz.width * m_scale + 0.5);
		sz.height = (int)(sz.height * m_scale + 0.5);
		resize(img, data, sz);
	}else{
		data = img.clone();
	}

	int c = data.channels();
	vector<uchar> buf;
	if(m_fmt == 1)/* jpeg image */{
		vector<int> param = vector<int>(2);
		param[0] = CV_IMWRITE_JPEG_QUALITY;
		param[1] = min(100, max(0, m_qjpeg));
		imencode(".jpg", data, buf, param);
		data = Mat(buf);
	}else if(m_fmt == 2){
		vector<int> param = vector<int>(2);
		param[0] = CV_IMWRITE_PNG_COMPRESSION;
		param[1] = min(9, max(0, m_qpng));
		imencode(".png", data, buf, param);
		data = Mat(buf);
	}
	Mat imgtest = imdecode(data, CV_LOAD_IMAGE_COLOR);
	sz = data.size();
	int size_elem = (int) data.elemSize();
	int channel = (int) data.channels();
	int depth = (int) data.elemSize1();
	unsigned int len_data = size_elem * sz.width * sz.height;

	int res;
	fd_set fw, fe;
	timeval tv;
	s_img_pkt0 h0 = s_img_pkt0(len_data, data.type(), m_fmt, m_cfmt, 
		(unsigned int) sz.width, (unsigned int) sz.height);
	int lenh = sizeof(h0);
	FD_ZERO(&fw);
	FD_ZERO(&fe);
	FD_SET(m_sock_client, &fw);
	FD_SET(m_sock_client, &fe);
	tv.tv_sec = 3;
	tv.tv_usec = 0;

	res = select((int) m_sock_client + 1, NULL, &fw, &fe, &tv);
	if(res > 0){
		if(FD_ISSET(m_sock_client, &fw)){
			int lenh_sent = send(m_sock_client, 
				(const char *) &h0, lenh, MSG_MORE);
			if(lenh_sent != lenh){
				cerr << "Incomplete transmission of header. " << endl;
				cerr << lenh_sent << "/" << lenh << endl;
				disconnect();
				return true;
			}
		}else{
			cerr << "Socket error after returning select." << endl;
			disconnect();
			return true;
		}
	}else if(res < 0){
		int en = errno;
		cerr << "Error No " << en << " " << strerror(en) << endl;
	}else{
		cerr << "Sending stream header timeout." << endl;
		return true;
	}

	int len_data_sent = 0;
	FD_ZERO(&fw);
	FD_ZERO(&fe);
	FD_SET(m_sock_client, &fw);
	FD_SET(m_sock_client, &fe);
	tv.tv_sec = 3;
	tv.tv_usec = 0;

	res = select((int) m_sock_client + 1, NULL, &fw, &fe, &tv);
	if(res > 0){
		if(FD_ISSET(m_sock_client, &fw)){
			int len = send(m_sock_client, 
				(const char *) data.data, 
				len_data, 0);
			if(len == SOCKET_ERROR || len == 0){
				cerr << "Socket error in sending data stream." <<endl;
				disconnect();
				return true;
			}
			if(len != len_data){
				cerr << "Incomplete transmittion of data." << endl;
				cerr << len << "/" << len_data << endl;
			}
		}else{
			cerr << "Socket error after returning select." << endl;
			disconnect();
			return true;
		}

	}else if(res < 0){	
		int en = errno;
		cerr << "Error No " << en << " " << strerror(en) << endl;
	}else{
		cerr << "Sending stream data timeout." << endl;
		return true;
	}

	return true;
}


bool f_rcv_img::init_run()
{
	if(m_chout.size() == 1){
		m_pimgout = dynamic_cast<ch_image*>(m_chout[0]);
	}

	if(m_pimgout == NULL)
		return false;

	m_sock = socket(AF_INET, SOCK_STREAM, 0);
	m_sock_addr.sin_family = AF_INET;
	m_sock_addr.sin_port = htons(m_svr_port);
	set_sockaddr_addr(m_sock_addr, m_svr_addr);
	return true;
}

bool f_rcv_img::try_connection()
{
	int itr = 0;
	while(1){
		if(::connect(m_sock, 
			(sockaddr*) &m_sock_addr, 
			sizeof(m_sock_addr)) == SOCKET_ERROR){
				cerr << "Socket error in connect." << endl;
				return false;
		}
		cout << "Client connection established." << endl;
		m_bconnected = true;
		break;
	}
	return true;
}


void f_rcv_img::destroy_run()
{
	closesocket(m_sock);
}


bool f_rcv_img::proc()
{
	if(is_pause()){
		return true;
	}

	if(!m_bconnected){
		if(!try_connection()){
			m_bconnected = false;
			return true;
		}
	}

	int res;
	fd_set fr;
	fd_set fe;
	timeval tv;

	FD_ZERO(&fr);
	FD_ZERO(&fe);
	FD_SET(m_sock, &fr);
	FD_SET(m_sock, &fe);

	tv.tv_usec = 0;
	tv.tv_sec = 3;

	s_img_pkt0 h0;
	int len_rcv = sizeof(h0);
	int len_rcvd = 0;
	while(len_rcvd != len_rcv){
		res = select((int) m_sock + 1, &fr, NULL, &fe, &tv);
		if(res > 0){
			if(FD_ISSET(m_sock, &fr)){
				int len = recv(m_sock, ((char*) &h0) + len_rcvd, len_rcv - len_rcvd, MSG_MORE);
				if(len == SOCKET_ERROR || len == 0){
					cerr << "Socket error during receiving header" << endl;
					disconnect();
					return true;
				}
				len_rcvd += len;
			}else if(FD_ISSET(m_sock, &fe)){
				cerr << "Socket error" << endl;
				disconnect();
				return true;
			}
		}else if(res == -1){
			int en = errno;
			cerr << "Error no " << en << " " << strerror(en) << endl;
			return true;
		}else{
			cerr << "Receiving stream header timeout." << endl;
			continue;
		}
	}

	if(!h0.check()){
		cerr << "Failed to recieve stream header." << endl;
		disconnect();
		return true;
	}

	Mat data = Mat(h0.h, h0.w, h0.type);
	len_rcv =  h0.len;
	len_rcvd = 0;
	while(len_rcvd < len_rcv){
		FD_ZERO(&fe);
		FD_SET(m_sock, &fr);
		FD_SET(m_sock, &fe);

		tv.tv_usec = 0;
		tv.tv_sec = 3;

		res = select((int) m_sock + 1, &fr, NULL, &fe, &tv);
		if(res > 0){
			if(FD_ISSET(m_sock, &fr)){
				int len = recv(m_sock, (char*) data.data + len_rcvd, len_rcv - len_rcvd, 0);
				if(len == SOCKET_ERROR || len == 0){
					cerr << "Failed to recieve stream data." << endl;
					disconnect();
					return true;
				}
				len_rcvd += len;
			}else if(FD_ISSET(m_sock, &fe)){
				cerr << "Socket error." << endl;
				disconnect();
				return true;
			}
		}else if(res < 0){
			int en = errno;
			cerr << "Error No " << en << " " << strerror(en) << endl;
			continue;
		}else{
			cerr << "Receiving stream data timeout." << endl;
			continue;
		}
	}

	Mat img;
	switch(h0.fmt){
	case 1: //jpg
	case 2: //png
		img = imdecode(data, CV_LOAD_IMAGE_COLOR);
		break;
	case 0:
	default:
		img = data;
	}

	m_pimgout->set_img(img, m_cur_time);
	return true;
}
