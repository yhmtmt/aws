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
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_com.h"


///////////////////////////////////////////////// f_dummy_data
const char * f_dummy_data::m_str_edt[EDT_TIME_FILE + 1] = 
{
	"rand", "time", "bseq", "aseq", "trand", "file", "tfile"
};

bool f_dummy_data::init_run()
{
	if(m_dtype == EDT_FILE){
		if(m_fname_data[0] != '\0'){
			m_fdata.open(m_fname_data);
			if(!m_fdata.is_open()){
				return false;
			}
		}
	}else if(m_dtype == EDT_RAND || m_dtype == EDT_TIME_RAND){
		srand(0);
	}

	if(m_chout.size() != 1)
		return false;
	m_pout = dynamic_cast<ch_ring<char>*>(m_chout[0]);

	m_len_pkt = max(m_len_pkt, (unsigned int)sizeof(m_cur_time));
	m_buf = new unsigned char[m_len_pkt];
	if(m_buf == NULL){
		return false;
	}
	return true;
}

void f_dummy_data::destroy_run()
{
	delete[] m_buf;
	m_buf = NULL;
}

bool f_dummy_data::proc(){
	if(m_pout != NULL){
		if(m_tail_buf == 0){// no data
			switch(m_dtype){
			case EDT_RAND:
				for(unsigned int i = 0; i < m_len_pkt; m_tail_buf++, i++, m_total++)
					m_buf[i] = (unsigned char) (rand() & 0x00FF);
				break;
			case EDT_TIME:
				memset(m_buf, 0, m_len_pkt);
				memcpy(m_buf, (void*)&m_cur_time, sizeof(m_cur_time));
				m_tail_buf = sizeof(m_cur_time);
				m_total += m_tail_buf;
				break;
			case EDT_BYTE_SEQ:
				for(unsigned int i = 0; i < m_len_pkt;m_tail_buf++, i++, m_total++){
					m_buf[i] = (unsigned char) (m_total & 0xFF);
				}
				break;
			case EDT_ASCII_SEQ:
				for(unsigned int i = 0; i < m_len_pkt;m_tail_buf++, i++, m_total++){
					m_buf[i] = (char)(m_total % ('Z' - 'A' + 1)) + 'A';
				}
				break;
			case EDT_TIME_RAND:
				memcpy(m_buf, (void*)&m_cur_time, sizeof(m_cur_time));
				m_tail_buf = sizeof(m_cur_time);
				for(unsigned int i = sizeof(m_cur_time); i < m_len_pkt; m_tail_buf++, i++){
					m_buf[i] = (unsigned char) (rand() & 0x00FF);
				}
				m_total += m_tail_buf;
				break;
			case EDT_FILE:
				m_fdata.read((char*)m_buf, m_len_pkt);
				m_tail_buf = (int) m_fdata.gcount();
				m_total += m_tail_buf;
				break;
			case EDT_TIME_FILE:
				memcpy(m_buf, (void*)&m_cur_time, sizeof(m_cur_time));
				m_fdata.read((char*)(m_buf + sizeof(m_cur_time)), m_len_pkt);
				m_tail_buf = (int) m_fdata.gcount() + sizeof(m_cur_time);
				m_total += m_tail_buf;
				break;
			}
		}

		if(m_head_buf < m_tail_buf){
			m_head_buf += m_pout->write((char*)(m_buf + m_head_buf), m_tail_buf - m_head_buf);
		}
		
		if(m_head_buf == m_tail_buf){
			m_head_buf = m_tail_buf = 0;
		}
	}
	return true;
}

///////////////////////////////////////////////// f_serial

bool f_serial::init_run()
{

	if(m_chin.size() == 1)
		m_pin = dynamic_cast<ch_ring<char>*>(m_chin[0]);

	if(m_chout.size() == 1)
		m_pout = dynamic_cast<ch_ring<char>*>(m_chout[0]);

#ifdef _WIN32
	m_hserial = open_serial(m_port, m_br);
#else
	m_hserial = open_serial(m_dname, m_br);
#endif
	if(m_hserial == NULL_SERIAL)
		return false;

	// allocate the buffer if the memory is not allocated
	m_wbuf = new char [m_frm_len];
	if(m_wbuf == NULL)
		return false;
	m_rbuf = new char [m_frm_len];
	if(m_rbuf == NULL){
		delete[] m_wbuf;
		m_wbuf = NULL;
		return false;
	}

	return true;
}

void f_serial::destroy_run()
{
	close_serial(m_hserial);

	delete[] m_rbuf;
	m_rbuf = NULL;
	delete[] m_wbuf;
	m_wbuf = NULL;
}

bool f_serial::proc()
{
	if(m_pout){
		if(m_tail_rbuf == 0){
			m_tail_rbuf = read_serial(m_hserial, m_rbuf, m_frm_len);
		}

		if(m_head_rbuf < m_tail_rbuf){
			m_head_rbuf += m_pout->write(m_rbuf + m_head_rbuf, m_tail_rbuf - m_head_rbuf);
		}

		if(m_head_rbuf == m_tail_rbuf){
			m_head_rbuf = 0;
			m_tail_rbuf = 0;
		}
	}

	if(m_pin){
		if(m_tail_wbuf == 0){
			m_tail_wbuf = m_pin->read(m_wbuf, m_frm_len);
		}

		if(m_head_wbuf < m_tail_wbuf){
			m_head_wbuf += write_serial(m_hserial, m_wbuf + m_head_wbuf, m_tail_wbuf - m_head_wbuf);
		}

		if(m_head_wbuf == m_tail_wbuf){
			m_head_wbuf = 0;
			m_tail_wbuf = 0;
		}
	}
	return true;
}


///////////////////////////////////////////////// f_ch_share
bool f_ch_share::init_run()
{
  m_sock = socket(AF_INET, SOCK_DGRAM, 0);	
  if(set_sock_nb(m_sock) != 0)
    return false;
  
  if(m_host_dst[0]){
    m_svr = false;
    m_client_fixed = true;
    m_sock_addr_snd.sin_family =AF_INET;
    m_sock_addr_snd.sin_port = htons(m_port_dst);
    m_sz_sock_addr_snd = sizeof(m_sock_addr_snd);
    set_sockaddr_addr(m_sock_addr_snd, m_host_dst);
  }else{
    m_svr = true;
    m_client_fixed = false;
    m_sock_addr_rcv.sin_family =AF_INET;
    m_sock_addr_rcv.sin_port = htons(m_port);
    set_sockaddr_addr(m_sock_addr_rcv);
    if(::bind(m_sock, (sockaddr*)&m_sock_addr_rcv, sizeof(m_sock_addr_rcv)) == SOCKET_ERROR){
      cerr << "Socket error" << endl;
      return false;
    }
  }

  if(m_fname_out[0]){
    m_fout.open(m_fname_out);	
    if(!m_fout.is_open())
      return false;
  }
  
  if(m_fname_in[0]){
    m_fin.open(m_fname_in);
    if(!m_fin.is_open())
      return false;
  }

  m_len_pkt_rcv = 0;
  for (int och = 0; och < m_chout.size(); och++){
    m_len_pkt_rcv += (int) m_chout[och]->get_dsize();
  }
  
  m_len_pkt_snd = 0;
  for (int ich = 0; ich < m_chin.size(); ich++){
    m_len_pkt_snd += (int) m_chin[ich]->get_dsize();
  }
 
  m_rbuf = new char [m_len_pkt_rcv];
  if(m_rbuf == NULL)
    return false;

  m_wbuf = new char [m_len_pkt_snd];
  if(m_wbuf == NULL){
    delete[] m_rbuf;
    m_rbuf = NULL;
    return false;
  }
  return true;
}

void f_ch_share::destroy_run()
{
  delete[] m_rbuf;
  m_rbuf = NULL;
  delete[] m_wbuf;
  m_wbuf = NULL;

  if(m_fout.is_open()){
    m_fout.close();
  }

  if(m_fin.is_open()){
    m_fin.close();
  }

  closesocket(m_sock);
  m_sock = -1;
}

bool f_ch_share::proc()
{
  int res;
  fd_set fr, fw, fe;
  timeval tv;

  // sending phase
  if(m_svr && m_client_fixed || !m_svr){
    m_wbuf_head = m_wbuf_tail = 0;
    for(int ich = 0; ich < m_chin.size(); ich++)
      m_wbuf_tail += (int)(m_chin[ich]->read_buf(m_wbuf + m_wbuf_tail));
    
    while(m_wbuf_tail != m_wbuf_head){
      FD_ZERO(&fe);
      FD_ZERO(&fw);
      FD_SET(m_sock, &fe);
      FD_SET(m_sock, &fw);
      tv.tv_sec = 0;
      tv.tv_usec = 10000;
      
      res = select((int) m_sock + 1, NULL, &fw, &fe, &tv);
      
      if(FD_ISSET(m_sock, &fw)){
	m_wbuf_head += sendto(m_sock, 
			      (char*) m_wbuf + m_wbuf_head, 
			      m_wbuf_tail - m_wbuf_head, 0, 
			      (sockaddr*) &m_sock_addr_snd, m_sz_sock_addr_snd);  
      }else if(FD_ISSET(m_sock, & fe)){
	cerr << "Socket error during sending packet in " << m_name;
	cerr << ". Now closing socket." << endl;
	destroy_run();
	
	return init_run();
      }else{
	// time out;
	break;
      }
    }
    if(m_verb){
      cout << "Inputs" << endl;
      for(int ich = 0; ich < m_chin.size(); ich++)
	m_chin[ich]->print(cout);
    }
  }

  // recieving phase
  m_rbuf_head = m_rbuf_tail = 0;
  while(m_rbuf_tail != m_len_pkt_rcv){
      FD_ZERO(&fr);
      FD_ZERO(&fe);
      FD_SET(m_sock, &fr);
      FD_SET(m_sock, &fe);
      tv.tv_sec = 0;
      tv.tv_usec = 1000; 
      
      res = select((int) m_sock + 1, &fr, NULL, &fe, &tv);
      if(FD_ISSET(m_sock, &fr)){
	int res = 0;
	//res = recv(m_sock, m_rbuf, m_len_pkt_rcv - m_rbuf_tail, 0);
	m_sz_sock_addr_snd = sizeof(m_sock_addr_snd);
	res = recvfrom(m_sock, 
		       (char*) m_rbuf + m_rbuf_tail, 
		       m_len_pkt_rcv - m_rbuf_tail,
		       0,
		       (sockaddr*) & m_sock_addr_snd, 
		       &m_sz_sock_addr_snd);
	
	if(res == -1)
	  break;
	else
	  m_rbuf_tail += res;

	if(m_rbuf_tail == m_len_pkt_rcv){
	  m_client_fixed = true;
	  for(int och = 0; och < m_chout.size(); och++){
	    m_rbuf_head += (int)(m_chout[och]->write_buf(m_rbuf + m_rbuf_head));
	  }
	  if(m_verb){
	    cout << "Outputs:" << m_rbuf_tail << "/" << res << endl;
	    for(int och = 0; och < m_chout.size(); och++)
	      m_chout[och]->print(cout);
	  }
	  m_rbuf_head = m_rbuf_tail = 0;
	}
      }else if(FD_ISSET(m_sock, &fe)){
	cerr << "Socket error during recieving packet in " << m_name;
	cerr << ". Now closing socket." << endl;
	destroy_run();
	return init_run();
      }else{
	// time out;
	return true;
      }
  }


  return true;
}

///////////////////////////////////////////////// f_udp

bool f_udp::init_run()
{
	m_sock = socket(AF_INET, SOCK_DGRAM, 0);	
	if(set_sock_nb(m_sock) != 0)
		return false;

	if(m_chin.size() == 1 && (m_pin = dynamic_cast<ch_ring<char>*>(m_chin[0])) != NULL){
		m_sock_addr_snd.sin_family =AF_INET;
		m_sock_addr_snd.sin_port = htons(m_port_dst);
		set_sockaddr_addr(m_sock_addr_snd, m_host_dst);
	}

	if(m_chout.size() == 1 && (m_pout = dynamic_cast<ch_ring<char>*>(m_chout[0])) != NULL){
		m_sock_addr_rcv.sin_family =AF_INET;
		m_sock_addr_rcv.sin_port = htons(m_port);
		set_sockaddr_addr(m_sock_addr_rcv);
		if(::bind(m_sock, (sockaddr*)&m_sock_addr_rcv, sizeof(m_sock_addr_rcv)) == SOCKET_ERROR){
			cerr << "Socket error" << endl;
			return false;
		}
	}

	if(m_fname_out[0]){
		m_fout.open(m_fname_out);	
		if(!m_fout.is_open())
			return false;
	}

	if(m_fname_in[0]){
		m_fin.open(m_fname_in);
		if(!m_fin.is_open())
			return false;
	}

	m_rbuf = new char [m_len_pkt];
	if(m_rbuf == NULL)
		return false;

	m_wbuf = new char [m_len_pkt];
	if(m_wbuf == NULL){
		delete[] m_rbuf;
		m_rbuf = NULL;
		return false;
	}

	return true;
}

void f_udp::destroy_run()
{
	closesocket(m_sock);
	m_sock = -1;
	delete[] m_rbuf;
	m_rbuf = NULL;
	delete[] m_wbuf;
	m_wbuf = NULL;
	if(m_fout.is_open())
		m_fout.close();
	if(m_fin.is_open())
		m_fin.close();
}

bool f_udp::proc(){
	bool rcv_end = false;
	bool snd_end = false;
	while(!rcv_end || !snd_end){
		if(m_pout){
			if(m_tail_rbuf == 0){
				socklen_t sz = sizeof(m_sock_addr_rcv);
				m_tail_rbuf = recvfrom(m_sock, m_rbuf, m_len_pkt, 0, (sockaddr*) &m_sock_addr_snd, &sz);

				if(m_tail_rbuf == 0) // no recieved packet
					rcv_end = true;
				else
					snd_end = false;

				if(m_tail_rbuf == SOCKET_ERROR){ // packet does not arrive
					int er = get_socket_error();
					if(!ewouldblock(er) && !econnreset(er)){
						cout << "Error in socket " << m_name << endl;
						dump_socket_error();
						return false;
					}
					m_tail_rbuf = 0;
					rcv_end = true;
				}
			}

			if(m_head_rbuf < m_tail_rbuf){
				int len = m_pout->write(m_rbuf + m_head_rbuf, m_tail_rbuf - m_head_rbuf);
				if(len == 0) // output channel is full
					rcv_end = true;
				m_head_rbuf += len;
			}

			if(m_head_rbuf == m_tail_rbuf){
				if(m_fin.is_open())
					m_fin.write(m_rbuf, m_tail_rbuf);

				/*
				if(m_tail_rbuf > 0)
					cout << m_name << " recieved " << m_tail_rbuf << "bytes" << endl;
				*/
				m_head_rbuf = 0;
				m_tail_rbuf = 0;
			}

		}

		if(m_pin){
			if(m_tail_wbuf == 0){
				m_tail_wbuf = m_pin->read(m_wbuf, m_len_pkt);
				if(m_tail_wbuf == 0) // no data in the channel
					snd_end = true;
				else
					snd_end = false;
			}

			if(m_head_wbuf < m_tail_wbuf){
				int sz = sizeof(m_sock_addr_snd);
				int len = sendto(m_sock, m_wbuf + m_head_wbuf, 
					m_tail_wbuf - m_head_wbuf, 0, (sockaddr*)&m_sock_addr_snd, sz);
				if(len == 0) // socket is not ready to send packet
					snd_end = true;
				m_head_wbuf += len;
			}

			if(m_head_wbuf == m_tail_wbuf){
				if(m_fout.is_open())
					m_fout.write(m_wbuf, m_tail_wbuf);
				/*
				if(m_tail_wbuf > 0)
					cout << m_name << " sent " << m_tail_wbuf << "bytes" << endl;
				*/
				m_head_wbuf = 0;
				m_tail_wbuf = 0;
			}
		}
	}

	return true;
}

///////////////////////////////////////////////// f_trn_img
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
	int res;
	fd_set fr, fe;
	timeval tv;
	int itr = 0;
	FD_ZERO(&fr);
	FD_ZERO(&fe);
	FD_SET(m_sock_svr, &fr);
	FD_SET(m_sock_svr, &fe);
	tv.tv_sec = 3;
	tv.tv_usec = 0;

	res = select((int) m_sock_svr + 1, &fr, NULL, &fe, &tv);
	if(res > 0){
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
		}else{
			f_base::send_err(this, __FILE__, __LINE__, FERR_TRN_IMG_SOCK_SVR_CON);
			return false;
		}
	}else if(res < 0){
		int en = errno;
		cerr << "Error No " << en << " " << strerror(en) << endl;
		return false;
	}else{
		return false;	
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


////////////////////////////////////////////////////// f_rcv_img

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

//////////////////////////////////////////////////////////////// f_write_ch_log
bool f_write_ch_log::open_log(const int ich)
{	char fname[1024];
	snprintf(fname, 1024, "%s/%s.jr", m_path, m_chin[ich]->get_name());
	ofstream fjr(fname, ios_base::app);
	if(!fjr.is_open())
		return false;
	int l = (int) strlen(m_path) + 1;
	snprintf(fname, 1024, "%s/%s_%lld.log", m_path, m_chin[ich]->get_name(), m_cur_time);
	m_logs[ich] = fopen(fname, "wb");
	if(!m_logs[ich]){
		cerr << "Failed to open " << fname << "." << endl;
	}
	fjr << "#S " << m_cur_time << endl;
	fjr << (&fname[l]) << endl;
	fjr.close();
	return true;
}

bool f_write_ch_log::close_log(const int ich)
{
	if(!m_logs[ich])
		return false;

	char fname[1024];
	snprintf(fname, 1024, "%s/%s.jr", m_path, m_chin[ich]->get_name());
	ofstream fjr(fname, ios_base::app);
	if(!fjr.is_open())
		return false;
	fjr << "#E " << m_cur_time << endl;
	fjr.close();

	fclose(m_logs[ich]);
	m_logs[ich] = NULL;
	m_szs[ich] = 0;

	return true;
}

bool f_write_ch_log::init_run()
{
	m_logs.resize(m_chin.size(), NULL);
	m_szs.resize(m_chin.size(), 0);

	int ich;
	for(ich = 0; ich < m_chin.size(); ich++){
		if(!open_log(ich)){
			return false;
		}
	}
	
	return true;
}

void f_write_ch_log::destroy_run()
{
	for(int ich = 0; ich < m_chin.size(); ich++){
		close_log(ich);
	}
}

bool f_write_ch_log::proc()
{
  for(int ich = 0; ich < m_chin.size(); ich++){
	  if(m_szs[ich] > m_max_size){
		  close_log(ich);
		  open_log(ich);
	  }
    	m_szs[ich] += m_chin[ich]->write(m_logs[ich]);
  }
	return true;
}

//////////////////////////////////////////////////////////////// f_read_ch_log
bool f_read_ch_log::open_log(const int och)
{
	bool seek = false;
	char fname[1024];
	snprintf(fname, 1024, "%s/%s.jr", m_path, m_chout[och]->get_name());
	ifstream fjr(fname);

	while(!seek){
		char buf[1024];
		if(!fjr.is_open()){
			return false;
		}
		if(fjr.eof()){
			return false;
		}
		fjr.getline(buf, 1024);
		if(buf[0] != '#' || buf[1] != 'S')
			return false;
		long long ts = atol(&buf[3]);

		fjr.getline(buf, 1024);
		snprintf(fname, 1024, "%s/%s", m_path, buf);
		m_logs[och] = fopen(fname, "rb");
		cout << "Opening " << buf << " for " << m_chout[och]->get_name() << endl;
		if(!m_logs[och]){			
			cerr << "Failed to open file " << buf << "." << endl;
			return false;
		}

		fjr.getline(buf, 1024);
		if(buf[0] != '#' || buf[1] != 'E'){
			cerr << "Unknown record \"" << buf << "\".#E expected. " << endl;
			return false;
		}

		long long te = atol(&buf[3]);
		if(ts <= m_cur_time || te > m_cur_time){
			seek = true;
		}
	}
	return true;
}

bool f_read_ch_log::init_run()
{
	m_logs.resize(m_chout.size());

	int och;
	for(och = 0; och < m_chout.size(); och++){
		if(!open_log(och))
			return false;
	}
	return true;
}

void f_read_ch_log::destroy_run()
{
	if(m_logs.size())
	{
		for(int och = 0; och < m_chout.size(); och++){
			if(m_logs[och]){
				fclose(m_logs[och]);
			}
		}
		m_logs.clear();
	}
}

bool f_read_ch_log::proc()
{
  for(int och = 0; och < m_chout.size(); och++){
    if(m_chout[och]->read(m_logs[och], m_cur_time) == 0){// eof
		fclose(m_logs[och]);
		m_logs[och] = NULL;
		if(!open_log(och)){
			cout << m_chout[och]->get_name() << "'s log finished at " << m_cur_time << endl;
		}
	}
  }
  
  return true;
}
