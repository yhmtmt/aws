#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_window.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_window.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_window.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <WindowsX.h>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"

#include "../util/aws_coord.h"
#include "../util/c_ship.h"
#include "../util/c_clock.h"
//#include "../util/c_nmeadec.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_scalar.h"
#include "../channel/ch_campar.h"
#include "../channel/ch_navdat.h"
#include "../channel/ch_nmea.h"

#include "f_sprot_window.h"

#ifdef _WIN32

//////////////////////////////////////////////////////////////// f_sprot_window

f_sprot_window::f_sprot_window(const char * name): f_ds_window(name), m_bmout(NULL), 
	m_tint_ttm2ais(5 * SEC), m_bmch(0), m_tprev_ttm2ais(0), m_range(10000), m_circle(1852), m_seq_id(0),
	m_tint_trail(10 * SEC), m_tprev_trail(0), m_bm_ver(1), m_max_trail(100)
{
	m_toker[0] = 'E';
	m_toker[1] = 'I';
	m_ftrail_log[0] = '\0';
	register_fpar("toker_bm", m_toker, 3, "Toker for binary message transfer.");
	register_fpar("tlog", m_ftrail_log, 128, "File name to save trail log.");
	register_fpar("tint_bm", &m_tint_ttm2ais, "Time interval of binary message transfer in 100nsec.");
	register_fpar("bm_ch", &m_bmch, "Channel for binary message (0: no preference, 1: channel A, 2: channel B, 3: Both.");
	register_fpar("range", &m_range, "Display range in meter.");
	register_fpar("circle", &m_circle, "Circle range in meter.");
	register_fpar("tint_trail", &m_tint_trail, "Time interval of saving trail.");
	register_fpar("max_trail", &m_max_trail, "Number of positions saved as trail.");
	register_fpar("bm_ver", &m_bm_ver, "Version of the binary message");
}

f_sprot_window::~f_sprot_window()
{
}

const char * f_sprot_window::get_err_msg(int code)
{
	const char * msg = f_base::get_err_msg(code);
	if(msg)
		return msg;

	switch(code){
	case FERR_SPROT_WINDOW_SNDBBM:
		return "Failed to send binary message.";
	case FERR_SPROT_WINDOW_OSPOS:
		return "Position of own ship has not been fixed yet.";
	case FERR_SPROT_WINDOW_FTRAIL_LOG_OPEN:
		return "Failed to open trail log file.";
	}
	return NULL;
}

bool f_sprot_window::send_bm(unsigned int mmsi_dst, int & seq_id,
	int id, int chan, 
	const unsigned char *  buf, 
	int bits)
{
	if(bits > 952){
		cout << "Irregal message length in send_bbm." << endl;
		return false;
	}

	if(chan >= 4){
		cout << "Irregal channel specification in send_bbm." << endl;
		return false;
	}

	if(id != 8 && id != 14 && id != 6 && id != 12){
		cout << "Irregal message id in send_bbm." << endl;
		return false;
	}

	int num_sends;

	m_nmea[0] = '!'; m_nmea[1] = m_toker[0]; m_nmea[2] = m_toker[1];

	int bit_lim;
	// Message type "ABM," or "BBM," filled in the m_nmea buffer
	if(id == 6 || id == 12){ //ABM
		m_nmea[3] = 'A';
		bit_lim = 288;
		seq_id %= 4;
	}else{ // BBM
		m_nmea[3] = 'B'; 
		bit_lim = 348;
		seq_id %= 10;
	}
	m_nmea[4] = 'B'; m_nmea[5] = 'M'; m_nmea[6] = ',';

	// calculating number of sentences needed 
	if(bits <= bit_lim){
		num_sends = 1;
	}else{
		num_sends = 1 + (bits - bit_lim) / 360 + (((bits - bit_lim) % 360) == 0 ? 0 : 1);
		num_sends = max(num_sends, 9);
	}

	m_nmea[7] = num_sends + '0'; m_nmea[8] = ',';  // Total number of sentences
	m_nmea[11] = seq_id + '0'; m_nmea[12] = ','; // sequential message identifier
	int ibit = 0;
	int ibuf = 0;
	int im = 0;
	for(int isend = 0; isend < num_sends; isend++){
		// first 58x6=348bit
		// subsequent 60x6=360bit
		int i;
		m_nmea[9] = (1 + isend) + '0'; m_nmea[10] = ','; // sentense number (upto num_sends)
		if(isend == 0){
			if(id == 6 || id == 12){
				sprintf(&m_nmea[13], "%09d", mmsi_dst);
				m_nmea[22] = ',';
				m_nmea[23] = chan + '0'; m_nmea[24] = ',';
				if(id == 6){
					m_nmea[25] = '6';
					i = 26;
				}else{
					m_nmea[25] = '1'; m_nmea[26] = '2';
					i = 27;
				}
			}else{
				m_nmea[13] = chan + '0'; m_nmea[14] = ',';
				if(id == 8){
					m_nmea[15] = '8';
					i = 16;
				}else{
					m_nmea[15] = '1'; m_nmea[16] = '4';
					i = 17;
				}
			}
				m_nmea[i] = ',';
			i++;
		}else{
			m_nmea[13] = ','; m_nmea[14] = ',';
			if(id == 6 || id == 12){
				m_nmea[15] = ',';
				i = 16;
			}else
				i = 15;
		}
			
		// copy message
		while(1){
			unsigned char uc = 0;

			switch(im){
			case 0:
				uc = (buf[ibuf] >> 2) & 0x3F;
				ibit += 6;
				im = 1;
				break;
			case 1:
				uc = (buf[ibuf] << 4);
				ibit += 2;
				if(ibit < bits){
					ibuf++;
					uc |= ((buf[ibuf] & 0xF0) >> 4); 
					ibit += 4;
				}
				im = 2;
				break;
			case 2:
				uc = (buf[ibuf] << 2);
				ibit += 4;
				if(ibit < bits){
					ibuf++;
					uc |= (buf[ibuf] & 0xC0) >> 6;
					ibit += 2;
				}
				im = 3;
				break;
			case 3:
				uc = buf[ibuf];
				ibit += 6;
				ibuf++;
				im = 0;
				break;
			}

			m_nmea[i] = armor(uc & 0x3F);
			i++;
			if(ibit >= bits || ibit >= bit_lim)
				break;
		}

		m_nmea[i] = ',';
		i++;
		if(isend == num_sends - 1){
			int pad = (bits % 6);
			if(pad)
				pad = 6 - pad;

			m_nmea[i] = pad + '0';	
		}else{
			m_nmea[i] = '0';
		}
		i++;
		m_nmea[i] = '*';
		unsigned char chksum = calc_nmea_chksum(m_nmea);
		char c;
		i++;
		c = (chksum >> 4) & 0x0F;
		m_nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');
		i++;
		c = chksum & 0x0F;
		m_nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');

		i++;
		m_nmea[i] = 13; // cr
		i++;
		m_nmea[i] = 10; // lf
		i++;
		m_nmea[i] = '\0';

		cout << "sending: " << m_nmea << endl;
		if(m_bmout){
			m_bmout->push(m_nmea);
		}

		bit_lim += 360;
	}
	seq_id = seq_id + 1;

	return true;
}

bool f_sprot_window::init_run()
{
	if(!f_ds_window::init_run())
		return false;

	if(m_chout.size())
		m_bmout = dynamic_cast<ch_nmea*>(m_chout[0]);
	if(m_ftrail_log[0]){
		m_trail_log.open(m_ftrail_log);
		if(!m_trail_log.is_open()){
			f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_FTRAIL_LOG_OPEN);
			return false;
		}
	}
	return true;
}

void f_sprot_window::destroy_run()
{
	f_ds_window::destroy_run();

	if(m_trail_log.is_open()){
		m_trail_log.close();
	}
}

bool f_sprot_window::alloc_d3dres()
{
	if(!f_ds_window::alloc_d3dres()){
		return false;
	}
	if(!m_d3d_ship2d.init(m_pd3dev,
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 16))
		return false;
	return true;
}

void f_sprot_window::release_d3dres()
{
	m_d3d_ship2d.release();
	f_ds_window::release_d3dres();
}

bool f_sprot_window::proc()
{
	Mat Rwrld;
	Point3d Xorg;

	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);
	

//	m_maincam.SetAsRenderTarget(m_pd3dev);
	int cx = m_ViewPort.Width >> 1;
	int cy = m_ViewPort.Height >> 1;
	double rat = (double) cy / m_range;

	c_ship::list_lock();
	c_ship & ship_own = c_ship::get_own_ship();
	if(!ship_own.calc_Xwrld_own(Xorg, Rwrld)){
		//f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_OSPOS);
		c_ship::list_unlock();
		return true;
	}


	const list<c_ship*> & ships = c_ship::get_ship_list();
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		(*itr)->calc_Xwrld(Xorg, Rwrld, m_cur_time);
	}
	c_ship::list_unlock();

//	m_d3d_ship2d.render(m_pd3dev, (float)cx,
//		(float)cy, 1.0, (float) -ship_own.get_cog(), D3DCOLOR_RGBA(0, 255, 0, 0));
	
	c_ship * pship_bm =  NULL;
	long long bmt_min = m_cur_time;

	m_pd3dev->BeginScene();
	render_circle(cx, cy, rat);
	c_ship::list_lock();
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		if(!(*itr)->is_inrange(m_range))
			continue;

		(*itr)->render2d(m_pd3dev, m_pline, m_d3d_ship2d, Xorg, Rwrld, 
			rat, (float)cx, (float) cy);
	}
	c_ship::list_unlock();
		
	ship_own.render2d(m_pd3dev, m_pline, m_d3d_ship2d, Xorg, Rwrld, 
		rat, (float) cx, (float)cy);


//	m_maincam.ResetRenderTarget(m_pd3dev);
//	m_maincam.show(m_pd3dev, 0, (float) m_ViewPort.Height);
	m_d3d_txt.set_prjmtx((float) m_ViewPort.Width, (float) m_ViewPort.Height);

	m_d3d_txt.render(m_pd3dev, get_time_str(),
		10, 20, 1.,0., EDTC_LB, D3DCOLOR_ARGB(255, 0, 255, 0));

	m_pd3dev->EndScene();

	if(m_grab_name)
		grab();

	if(m_pd3dev->Present(NULL, NULL, NULL, NULL) == D3DERR_DEVICELOST){
		cerr << "device lost" << endl;
		m_blost = true;
	}

	/////////////////////////// saving trail
	if(m_tprev_trail + m_tint_trail <= m_cur_time){
		if(m_trail_log.is_open()){
			ship_own.log_trail(m_trail_log);
		}
		
	//	ship_own.save_trail(m_max_trail);
		
		for(auto itr = ships.begin(); itr != ships.end(); itr++){
			if(!(*itr)->is_inrange(m_range))
				continue;
			
			if(m_trail_log.is_open()){
				(*itr)->log_trail(m_trail_log);
			}
			
	//		(*itr)->save_trail(m_max_trail);
		}
		m_tprev_trail = m_cur_time;
	}

	///////////////////////// sending ttm
	for(auto itr = ships.begin(); itr != ships.end(); itr++){
		if((*itr)->get_data_type() == ESDT_ARPA){
			long long bmt = (*itr)->get_last_bmt();
			if(bmt < bmt_min){
				bmt_min = bmt;
				pship_bm = (*itr);
			}
		}
	}

	if(m_bmout && pship_bm && (m_tprev_ttm2ais + m_tint_ttm2ais < m_cur_time)){
		// packing position data into bm
		s_binary_message bm;
		bm.type = 8;
		bm.ch = m_bmch;
		unsigned char sec;
		unsigned char id;
		unsigned short cog, sog, dist, bear;
		int lon, lat;
		id = pship_bm->get_ttmid();
		sog = (unsigned short) ((pship_bm->get_sog()) * 10 + 0.5);
		cog = (unsigned short) ((pship_bm->get_cog() * 180. / PI) * 10. + 0.5);
		dist = (unsigned short) (pship_bm->get_dist() * (10. / (double) MILE));
		bear = (unsigned short) ((pship_bm->get_bear() * 180. /PI) * 10. + 0.5);
		lon = (int)(pship_bm->get_lon() * (180. / PI) * 600000 + 0.5);
		lat = (int)(pship_bm->get_lat() * (180. / PI) * 600000 + 0.5);
		sec = m_tm.tm_sec;

		cout << "Encode: ";
		cout << "id " << (int) id 
			<< " sog " << sog
			<< " cog " << cog
			<< " dist " << dist
			<< " bear " << bear
			<< " lon " << lon 
			<< " lat " << lat  
			<< " sec " << (unsigned int) sec << endl;

		switch(m_bm_ver){
		case 5:
			bm.set_msg_pvc5(id, sog, cog, dist, bear, sec);
			break;
		case 4:
			bm.set_msg_pvc4(id, sog, cog, lon, lat, sec);
			break;
		case 3:
			bm.set_msg_pvc3(id, sog, cog, dist, bear);
			break;
		case 2:
			bm.set_msg_pvc2(id, sog, cog, lon, lat);
			break;
		case 1:
		default:
			bm.set_msg_pvc(id, sog, cog, lon, lat);
			break;
		}

		// sending bm to channel
		vector<string> nmeas;
		if(!bm.gen_nmea(m_toker, nmeas)){
		//if(!send_bm(bm.mmsi, m_seq_id, bm.id, bm.ch, bm.msg, bm.len)){
			f_base::send_err(this, __FILE__, __LINE__, FERR_SPROT_WINDOW_SNDBBM);
		}else{
			m_tprev_ttm2ais = m_cur_time;
			pship_bm->set_bm_time(m_cur_time);
		}

		if(m_bmout){
			for(int ibm = 0; ibm < nmeas.size(); ibm++){
				m_bmout->push(nmeas[ibm].c_str());	
			}
		}
	}

	return true;
}

#endif