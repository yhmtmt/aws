#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_ship.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_ship.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_ship.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#define _USE_MATH_DEFINES
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/thread_util.h"

#include "c_nmeadec.h"
#include "c_ship.h"

/////////////////////////////////////////////////////////////// c_ship member
/////////////////////////////////////// static members
c_ship * c_ship::m_htbl[1024];
double c_ship::m_time_limit = 3000000000;
size_t c_ship::m_htbl_size = 1024;
char c_ship::m_buf[128];
list<c_ship*> c_ship::m_ship_list;
vector<c_ship*> c_ship::m_ship_list_recent;
int c_ship::m_head_slrpv = -1;
int c_ship::m_max_slrpv = 32;
c_ship * c_ship::m_ship_pool = NULL;
pthread_mutex_t c_ship::m_list_mtx;

unsigned int c_ship::m_mmsi_own = -1;
c_ship c_ship::m_ship_own;

void c_ship::init()
{
#ifdef _WIN32
	m_ship_own.m_clr = D3DCOLOR_RGBA(0, 255, 0, 255);
#endif
	pthread_mutex_init(&m_list_mtx, NULL);
	memset(m_htbl, 0, sizeof(c_ship*) * m_htbl_size);
	m_ship_list_recent.resize(m_max_slrpv, NULL);
}

void c_ship::destroy()
{
	if(m_ship_list.size() != 0){
		for(list<c_ship*>::iterator itr = m_ship_list.begin();
			itr != m_ship_list.end(); itr++){
			if((*itr)->m_mmsi != UINT_MAX)
				c_ship::pop_htbl((*itr)->get_mmsi());			
			free((*itr));
		}
	}

	m_ship_list.clear();
	m_ship_list_recent.clear();
	if(m_ship_pool){
		delete m_ship_pool;
		m_ship_pool = NULL;
	}
	pthread_mutex_destroy(&m_list_mtx);
}

void c_ship::list_lock()
{
	pthread_mutex_lock(&m_list_mtx);
}

void c_ship::list_unlock()
{
	pthread_mutex_unlock(&m_list_mtx);
}

void c_ship::set_mmsi_own(unsigned int mmsi_own)
{
	m_mmsi_own = mmsi_own;
	m_ship_own.m_mmsi = mmsi_own;
}

c_ship & c_ship::get_own_ship()
{
	return c_ship::m_ship_own;
}

c_ship * c_ship::alloc()
{
	if(m_ship_pool == NULL)
		return new c_ship;

	c_ship * pship = m_ship_pool;
	m_ship_pool = pship->m_pnext;
	(*pship) = c_ship();
	return pship;
}

void c_ship::free(c_ship * pship)
{
	pship->m_pnext = m_ship_pool;
	m_ship_pool = pship;
}

c_ship * c_ship::get_htbl(unsigned int mmsi)
{
	if(m_mmsi == mmsi)
		return this;

	if(m_pnext == NULL)
		return NULL;

	return m_pnext->get_htbl(mmsi);
}

void c_ship::push_htbl(c_ship * pship)
{
	if(m_pnext == NULL){
		m_pnext = pship;
		return;
	}

	m_pnext->push_htbl(pship);
}

c_ship * c_ship::pop_htbl(unsigned int mmsi){
	int ihtbl = get_hash_value(mmsi);
	c_ship * ptmp = m_htbl[ihtbl];

	if(ptmp == NULL)
		return NULL;

	if(ptmp->m_mmsi == mmsi){
		m_htbl[ihtbl] = ptmp->m_pnext;
		ptmp->m_pnext = NULL;
	}else{
		ptmp = ptmp->pop_htbl_link(mmsi);
	}

	return ptmp;
}

c_ship * c_ship::pop_htbl_link(unsigned int mmsi)
{
	if(m_pnext == NULL)
		return NULL;

	if(m_pnext->m_mmsi == mmsi){
		c_ship * ptmp = m_pnext;
		m_pnext = m_pnext->m_pnext;
		ptmp->m_pnext = NULL;
		return ptmp;
	}

	return	m_pnext->pop_htbl_link(mmsi);
}

void c_ship::delete_timeout_ship(long long curtime)
{
	c_ship * pship;
	list<c_ship*>::iterator itr = m_ship_list.begin(); 
	while(itr != m_ship_list.end()){
		pship = (*itr);
		if(pship->is_timeout(curtime)){
			pop_recent(pship);
			pop_htbl(pship->m_mmsi);
			itr = m_ship_list.erase(itr);
			delete pship;
			continue;
		}

		itr++;
	}
}

const c_ship * c_ship::get_ship_by_mmsi(unsigned int mmsi)
{
	int ihtbl = get_hash_value(mmsi);
	if(m_htbl[ihtbl] == NULL)
		return NULL;
	return m_htbl[ihtbl]->get_htbl(mmsi);
}

const c_ship * c_ship::get_ship_by_ttmid(unsigned int ttmid){
	unsigned int mmsi = ttmid | 0x80000000;
	int ihtbl = get_hash_value(mmsi);
	if(m_htbl[ihtbl] == NULL)
		return NULL;
	return m_htbl[ihtbl]->get_htbl(mmsi);
}

const list<c_ship*> & c_ship::get_ship_list()
{
	return m_ship_list;
}

void c_ship::push_ship(c_ship * pship)
{
	m_ship_list.push_back(pship);
	if(pship->m_mmsi == UINT_MAX)
		return;

	int ihtbl = get_hash_value(pship->m_mmsi);
	if(m_htbl[ihtbl] == NULL){
		m_htbl[ihtbl] = pship;
		return;
	}

	m_htbl[ihtbl]->push_htbl(pship);
}

void c_ship::push_recent(c_ship * pship)
{
	m_head_slrpv = (m_head_slrpv + 1) % m_max_slrpv; 
	m_ship_list_recent[m_head_slrpv] = pship;
}

void c_ship::pop_recent(c_ship * pship)
{
	for(int i = 0; i < m_ship_list_recent.size(); i++){
		if(m_ship_list_recent[i] == pship)
			m_ship_list_recent[i] = NULL;
	}
}

const c_ship * c_ship::get_recent(int i)
{
	int irecent = m_head_slrpv - i;
	if(irecent < 0)
		irecent += m_max_slrpv;
	if(irecent < 0)
		return NULL;

	return m_ship_list_recent[irecent];
}

void c_ship::delete_ship(c_ship * pship)
{
	m_ship_list.remove(pship);

	pop_recent(pship);	

	if(pship->m_mmsi != UINT_MAX){
		pop_htbl(pship->m_mmsi);
	}

	free(pship);
}

void c_ship::delete_ship_by_mmsi(unsigned int mmsi)
{
	c_ship * pship = pop_htbl(mmsi);

	if(pship == NULL)
		return;

	m_ship_list.remove(pship);

	pop_recent(pship);

	free(pship);
}

void c_ship::set_own_rmc(c_rmc * prmc, long long cur_time)
{
	float lat = (float)((prmc->m_lat_dir == EGP_N ? 1. : -1.) * prmc->m_lat_deg * PI / 180.);
	float lon = (float)((prmc->m_lon_dir == EGP_E ? 1. : -1.) * prmc->m_lon_deg * PI / 180.);

	m_ship_own.m_Xbih.lat = lat;
	m_ship_own.m_Xbih.lon = lon;
	m_ship_own.m_Xbih.alt = 0.;
	m_ship_own.m_cog = prmc->m_crs * (PI / 180.);
	m_ship_own.m_vel = prmc->m_vel;
	m_ship_own.m_Vwrld.x = prmc->m_vel * sin(m_ship_own.m_cog) * KNOT;
	m_ship_own.m_Vwrld.y = prmc->m_vel * cos(m_ship_own.m_cog) * KNOT;
	m_ship_own.set_time(cur_time);
	m_ship_own.m_update = true;
}

const c_ship * c_ship::register_ship_by_vdm1(c_vdm_msg1 * pvdm1, long long cur_time)
{
	if(pvdm1 == NULL)
		return NULL;

	if(pvdm1->m_vdo){
		return NULL;
	}

	unsigned int mmsi = pvdm1->m_mmsi;
	if(mmsi == m_mmsi_own)
		return NULL;

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi(mmsi));
	if(ps == NULL){ // register new ship
		ps = alloc();
		ps->m_ais_class_A = true;
		ps->m_mmsi = pvdm1->m_mmsi;

		push_ship(ps);
	}
	ps->m_data_type = ESDT_VDM;

	ps->set_pos(pvdm1->m_lat * (PI/180.), pvdm1->m_lon * (PI/180.),
		0, pvdm1->m_speed, pvdm1->m_course * (PI/180.),
		pvdm1->m_heading, pvdm1->m_turn);	

	// push recent list
	push_recent(ps);

	ps->set_time(cur_time);
	return ps;
}

const c_ship * c_ship::register_ship_by_vdm5(c_vdm_msg5 * pvdm5)
{
	if(pvdm5 == NULL)
		return NULL;

	if(pvdm5->m_vdo){
		return NULL;
	}

	unsigned int mmsi = pvdm5->m_mmsi;
	if(mmsi == m_mmsi_own)
		return NULL;

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi(mmsi));
	if(ps == NULL){ // register new ship
		ps = alloc();
		ps->m_ais_class_A = true;
		ps->m_mmsi = pvdm5->m_mmsi;

		push_ship(ps);
	}
	ps->m_data_type = ESDT_VDM;

	ps->set_name((const char*) pvdm5->m_shipname);
	ps->m_type = pvdm5->m_shiptype;
	ps->set_dimension(pvdm5->m_to_bow, pvdm5->m_to_stern,
		pvdm5->m_to_port, pvdm5->m_to_starboard);
	ps->set_callsign((const char*) pvdm5->m_callsign);
	return ps;
}

const c_ship * c_ship::register_ship_by_vdm8(c_vdm_msg8 * pvdm8, long long cur_time, int bm_ver)
{
	if(pvdm8 == NULL){
		return NULL;
	}
	unsigned char id;
	unsigned short cog, sog, dist, bear;
	unsigned char sec;
	const c_ship * pbs = get_ship_by_mmsi(pvdm8->m_mmsi);

	int lon, lat;
	
	// decoding binary message
	cout << "Decode: ";
	switch(bm_ver){
	case 5:
		{
			if(!pvdm8->m_msg.get_msg_pvc5(id, sog, cog, dist, bear, sec))
				return NULL;
			cout << "id " << (int) id 
				<< " sog " << sog
				<< " cog " << cog
				<< " dist " << dist
				<< " bear " << bear 
				<< " sec " << (unsigned int) sec << endl;

			pbs = get_ship_by_mmsi(pvdm8->m_mmsi);
			if(pbs == NULL)
				return NULL;
			break;
		}
	case 4:
		if(!pvdm8->m_msg.get_msg_pvc4(id, sog, cog, lon, lat, sec))
			return NULL;
		cout << "id " << (int) id 
			<< " sog " << (unsigned short) sog
			<< " cog " << (unsigned short) cog
			<< " lon " << lon 
			<< " lat " << lat 
			<< " sec " << (unsigned int) sec << endl;
	
		break;
	case 3: // relative 
		{
			if(!pvdm8->m_msg.get_msg_pvc3(id, sog, cog, dist, bear))
				return NULL;
			cout << "id " << (int) id 
				<< " sog " << sog
				<< " cog " << cog
				<< " dist " << dist
				<< " bear " << bear << endl;

			pbs = get_ship_by_mmsi(pvdm8->m_mmsi);
			if(pbs == NULL)
				return NULL;
			break;
		}
	case 2: // avoiding packing into DAC,FI,AckReq,TxtSeq field
		if(!pvdm8->m_msg.get_msg_pvc2(id, sog, cog, lon, lat))
			return NULL;
		cout << "id " << (int) id 
			<< " sog " << (unsigned short) sog
			<< " cog " << (unsigned short) cog
			<< " lon " << lon 
			<< " lat " << lat << endl;
	
		break;
	case 1: // legacy
	default:
		if(!pvdm8->m_msg.get_msg_pvc(id, sog, cog, lon, lat))
			return NULL;
		cout << "id " << (int) id 
			<< " sog " << (unsigned short) sog
			<< " cog " << (unsigned short) cog
			<< " lon " << lon 
			<< " lat " << lat << endl;
		break;
	}

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi((unsigned int) id | 0xC0000000));

	if(ps == NULL){
		ps = alloc();
		ps->set_bmid(id);
		push_ship(ps);
	}
	
	switch(bm_ver){
	case 5:
		{
			Mat Rwrld;
			Point3d Xorg;
			getwrldrot(pbs->get_bihpos(), Rwrld);
			bihtoecef(pbs->get_bihpos(), Xorg);
			ps->m_vel = (double) sog * 0.1;
			ps->m_cog = (double) cog * 0.1 * PI / 180;
			ps->m_dist =(double) dist * 0.1 * MILE;	
			ps->m_bear = (double) bear * 0.1 * PI / 180;
			double c = cos(ps->m_bear);
			double s = sin(ps->m_bear);	
			ps->m_Xwrld.x = ps->m_dist * s;
			ps->m_Xwrld.y = ps->m_dist * c;
			ps->m_Xwrld.z = 0;
			wrldtobih(Xorg, Rwrld, ps->m_Xwrld, ps->m_Xbih); 
			ps->m_Vwrld.x = ps->m_vel * sin(ps->m_cog) * KNOT;
			ps->m_Vwrld.y = ps->m_vel * cos(ps->m_cog) * KNOT;
			ps->m_update = true;
			ps->set_time(cur_time);
			ps->m_data_type = ESDT_BM;
			break;
		}
	case 3: // relative 
		{
			Mat Rwrld;
			Point3d Xorg;
			getwrldrot(pbs->get_bihpos(), Rwrld);
			bihtoecef(pbs->get_bihpos(), Xorg);
			ps->m_vel = (double) sog * 0.1;
			ps->m_cog = (double) cog * 0.1 * PI / 180;
			ps->m_dist =(double) dist * 0.1 * MILE;	
			ps->m_bear = (double) bear * 0.1 * PI / 180;
			double c = cos(ps->m_bear);
			double s = sin(ps->m_bear);	
			ps->m_Xwrld.x = ps->m_dist * s;
			ps->m_Xwrld.y = ps->m_dist * c;
			ps->m_Xwrld.z = 0;
			wrldtobih(Xorg, Rwrld, ps->m_Xwrld, ps->m_Xbih); 
			ps->m_Vwrld.x = ps->m_vel * sin(ps->m_cog) * KNOT;
			ps->m_Vwrld.y = ps->m_vel * cos(ps->m_cog) * KNOT;
			ps->m_update = true;
			ps->set_time(cur_time);
			ps->m_data_type = ESDT_BM;
			break;
		}
	case 4:
	case 2: // avoiding packing into DAC,FI,AckReq,TxtSeq field
	case 1: // legacy
	default:
		ps->m_data_type = ESDT_BM;
		ps->set_pos((double) lat * (1.0 / 600000) * (PI / 180.), 
			(double) lon * (1.0 / 600000) * (PI / 180.), 0,
			(double) sog * 0.10, ((double) cog) * 0.10 * PI / 180., 0., 0.);
		ps->set_time(cur_time);
		break;
	}

	return ps;
}

const c_ship * c_ship::register_ship_by_vdm18(c_vdm_msg18 * pvdm18, long long cur_time)
{
	if(pvdm18 == NULL)
		return NULL;

	if(pvdm18->m_vdo){
		return NULL;
	}

	if(pvdm18->m_mmsi == m_mmsi_own)
		return NULL;

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi(pvdm18->m_mmsi));
	if(ps == NULL){ // register new ship
		ps = alloc();
		ps->m_ais_class_A = false;
		ps->m_mmsi = pvdm18->m_mmsi;
		push_ship(ps);
	}
	ps->m_data_type = ESDT_VDM;

	ps->set_pos(pvdm18->m_lat * (PI/180.), pvdm18->m_lon * (PI/180.),
		0, pvdm18->m_speed, pvdm18->m_course * (PI/180.),
		pvdm18->m_heading, 0.0);	

	// push recent list
	push_recent(ps);
	ps->set_time(cur_time);

	return ps;
}

const c_ship * c_ship::register_ship_by_vdm19(c_vdm_msg19 * pvdm19, long long cur_time)
{
	if(pvdm19 == NULL)
		return NULL;

	if(pvdm19->m_vdo){
		return NULL;
	}

	if(pvdm19->m_mmsi == m_mmsi_own)
		return NULL;

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi(pvdm19->m_mmsi));
	if(ps == NULL){ // register new ship
		ps = alloc();
		ps->m_ais_class_A = false;
		ps->m_mmsi = pvdm19->m_mmsi;

		push_ship(ps);
	}
	ps->m_data_type = ESDT_VDM;

	ps->set_pos(pvdm19->m_lat * (PI/180.), pvdm19->m_lon * (PI/180.),
		0, pvdm19->m_speed, pvdm19->m_course * (PI/180.),
		pvdm19->m_heading, 0.0);	
	ps->set_name((const char*) pvdm19->m_shipname);
	ps->m_type = pvdm19->m_shiptype;
	ps->set_dimension(pvdm19->m_to_bow, pvdm19->m_to_stern,
		pvdm19->m_to_port, pvdm19->m_to_starboard);

	// push recent list
	push_recent(ps);
	ps->set_time(cur_time);

	return ps;
}

const c_ship * c_ship::register_ship_by_vdm24(c_vdm_msg24 * pvdm24)
{
	if(pvdm24 == NULL)
		return NULL;

	if(pvdm24->m_vdo){
		return NULL;
	}

	unsigned int mmsi = pvdm24->m_mmsi;

	if(pvdm24->m_mmsi == m_mmsi_own)
		return NULL;

	c_ship * ps = const_cast<c_ship*>(get_ship_by_mmsi(mmsi));
	if(ps == NULL){ // register new ship
		ps = alloc();
		ps->m_ais_class_A = true;
		ps->m_mmsi = pvdm24->m_mmsi;

		push_ship(ps);
	}

	if(pvdm24->m_part_no == 0){
		ps->set_name((const char*) pvdm24->m_shipname);
	}else{
		ps->m_type = pvdm24->m_shiptype;
		ps->set_dimension(pvdm24->m_to_bow, pvdm24->m_to_stern,
			pvdm24->m_to_port, pvdm24->m_to_starboard);
		ps->set_callsign((const char*) pvdm24->m_callsign);
		ps->m_ms_mmsi = pvdm24->m_ms_mmsi;
	}

	ps->m_data_type = ESDT_VDM;

	return ps;
}

const c_ship * c_ship::register_ship_by_ttm(c_ttm * pttm, long long cur_time)
{
	if(pttm == NULL){
		return NULL;
	}

	c_ship * ps = const_cast<c_ship*>(get_ship_by_ttmid(pttm->m_id));
	if(pttm->m_state == 'L' && ps){
		delete_ship(ps);
	}

	if(ps == NULL){
		ps = alloc();
		ps->set_ttmid(pttm->m_id);
		push_ship(ps);
	}
	ps->m_data_type = ESDT_ARPA;

	if(pttm->m_data[0] != '\0'){
		ps->set_name((const char*) pttm->m_data);
	}

	ps->m_cog = pttm->m_crs * PI / 180;
	ps->m_dist = pttm->m_dist * (pttm->m_dist_unit == 'K' ? 1000. : (pttm->m_dist_unit == 'N' ? MILE : 1609.344));
	ps->m_vel = pttm->m_spd * (pttm->m_dist_unit == 'K' ? (double) MILE / 1000. : (pttm->m_dist_unit == 'N' ? 1.0 : (double) MILE / 1609.344));
	ps->m_bear = pttm->m_bear * PI / 180.;
	ps->m_Vwrld.x = ps->m_vel * sin(ps->m_cog) * KNOT;
	ps->m_Vwrld.y = ps->m_vel * cos(ps->m_cog) * KNOT;
	ps->m_update = true;
	ps->set_time(cur_time);
	return ps;
}

bool c_ship::calc_Xwrld_own(Point3d & org, Mat & rot)
{
	if(!m_update)
		return false;

	getwrldrot(m_Xbih, rot);
	bihtoecef(m_Xbih, org);
	m_Xwrld.x = 0;
	m_Xwrld.y = 0;
	m_Xwrld.z = 0;
	return true;	
}

bool c_ship::calc_Xwrld(Point3d & Xorg, Mat & Rwrld, long long tabs)
{
	long long dt = tabs - m_tabs;

	if(dt > m_time_limit)
		return false;

	if(m_data_type == ESDT_VDM){
		bihtowrld(Xorg, Rwrld, m_Xbih, m_Xwrld);
	}else if(m_data_type == ESDT_BM){						
		bihtowrld(Xorg, Rwrld, m_Xbih, m_Xwrld);
	}else if(m_data_type == ESDT_ARPA){
		double c = cos(m_bear);
		double s = sin(m_bear);
		m_Xwrld.x = m_dist * s;
		m_Xwrld.y = m_dist * c;
		m_Xwrld.z = 0;
		wrldtobih(Xorg, Rwrld, m_Xwrld, m_Xbih);
	}


	m_dist = sqrt(m_Xwrld.x * m_Xwrld.x + m_Xwrld.y * m_Xwrld.y);
	m_inv_dist = 1.0 / m_dist;
		/*
		if(!m_update){
		m_Xwrld += m_Vwrld * (double) dt * 1e-7;
		}
		*/

	return true;
}


const char * c_ship::get_ship_type()
{
	return get_ship_type_name(m_type);
}


////////////////////////////////////////////////// Direct 3D code
#ifdef _WIN32
LPDIRECT3DVERTEXBUFFER9 c_ship::m_pvb_std = NULL;
LPDIRECT3DINDEXBUFFER9 c_ship::m_pidb_std = NULL;
D3DMATERIAL9 c_ship::m_mtrl_std;
D3DLIGHT9 c_ship::m_light_std;

bool c_ship::load_std_model(LPDIRECT3DDEVICE9 pd3dev)
{
	const int num_vtxs = 22;
	const int num_tris = 10;

	// standard ship model includes 7 vertices and 10 triangles
	// these vertices is defined in right handed coordinate system.
	// (ECEF coordinate is also the right handed.) 
	// because the direct 3d coordinate system is left handed,
	// we should invert sign of the x values at the time placing them onto
	// world coordinate.
	CUSTOMVERTEX4 vtx2[num_vtxs] = {
		{0.f, 80.f, 11.54f, 0.f, 0.f, -1.f},	// 0 top side
		{-20.f, 60.f, 11.54f, 0.f, 0.f, -1.f},	// 1
		{20.f, 60.f, 11.54f, 0.f, 0.f, -1.f},	// 2
		{-20.f, -20.f, 11.54f, 0.f, 0.f, -1.f},	// 3
		{20.f, -20.f, 11.54f, 0.f, 0.f, -1.f},  // 4
		{-20.f, -20.f, 11.54f, 0.f, 1.f, 0.f},// 5 back side
		{20.f, -20.f, 11.54f, 0.f, 1.f, 0.f}, // 6
		{0.f, -20.f, -23.09f, 0.f, 1.f, 0.f}, // 7
		{-20.f, 60.f, 11.54f, 0.865956528f, 0.f, 0.500119277f},	//8 left side
		{-20.f, -20.f, 11.54f, 0.865956528f, 0.f, 0.500119277f},	//9
		{0.f, -20.f, -23.09f, 0.865956528f, 0.f, 0.500119277f},	//10
		{0.f, 60.f, -23.09f, 0.865956528f, 0.f, 0.500119277f},	//11
		{20.f, 60.f, 11.54f, -0.865956528f, 0.f, 0.500119277f},	//12 right side
		{20.f, -20.f, 11.54f, -0.865956528f, 0.f, 0.500119277f},	//13
		{0.f, -20.f, -23.09f, -0.865956528f, 0.f, 0.500119277f},	//14
		{0.f, 60.f, -23.09f, -0.865956528f, 0.f, 0.500119277f},	//15
		{0.f, 80.f, 11.54f, 0.654623918f, 0.654623918f, 0.378067524f},	//16 top left side
		{-20.f, 60.f, 11.54f, 0.654623918f, 0.654623918f, 0.378067524f},//17
		{0.f, 60.f, -23.09f, 0.654623918f, 0.654623918f, 0.378067524f},	//18
		{0.f, 80.f, 11.54f, -0.654623918f, 0.654623918f, 0.378067524f}, //19 top right side
		{20.f, 60.f, 11.54f, -0.654623918f, 0.654623918f, 0.378067524f},//20
		{0.f, 60.f, -23.09f, -0.654623918f, 0.654623918f, 0.378067524f}	//21
	};

	WORD idx2[num_tris * 3] = {
		0, 2, 1,
		1, 2, 3,
		3, 2, 4,
		5, 6, 7,
		8, 9, 10,
		10, 11, 8,
		15, 14, 13,
		15, 13, 12,
		16, 17, 18,
		19, 21, 20
	};
/*
	CUSTOMVERTEX3 vtx[num_vtxs];
	vtx[0].x = 0.;
	vtx[0].y = 80.0;
	vtx[0].z = 11.54;
	vtx[0].color = 0xffffffff;

	vtx[1].x = -20.0;
	vtx[1].y = 60.0;
	vtx[1].z = 11.54;
	vtx[1].color = 0xffffffff;

	vtx[2].x = 20.0;
	vtx[2].y = 60.0;
	vtx[2].z = 11.54;
	vtx[2].color = 0xffffffff;

	vtx[3].x = -20.0;
	vtx[3].y = -20.0;
	vtx[3].z = 11.54;
	vtx[3].color = 0xffffffff;

	vtx[4].x = 20.0;
	vtx[4].y = -20.0;
	vtx[4].z = 11.54;
	vtx[4].color = 0xffffffff;

	vtx[5].x = 0.0;
	vtx[5].y = -20.0;
	vtx[5].z = -23.09;
	vtx[5].color = 0xffffffff;

	vtx[6].x = 0.0;
	vtx[6].y = 60.0;
	vtx[6].z = -23.09;
	vtx[6].color = 0xffffffff;

	// index is specified in clock wise manner
	WORD idx[num_tris * 3];
	idx[0] = 0; idx[1] = 2; idx[2] = 1; // first triangle
	idx[3] = 1; idx[4] = 2; idx[5] = 3; 
	idx[6] = 3; idx[7] = 2; idx[8] = 4;
	idx[9] = 3; idx[10] = 4; idx[11] = 5;
	idx[12] = 3; idx[13] = 5; idx[14] = 1;
	idx[15] = 1; idx[16] = 5; idx[17] = 6;
	idx[18] = 5; idx[19] = 4; idx[20] = 2;
	idx[21] = 5; idx[22] = 2; idx[23] = 6;
	idx[24] = 0; idx[25] = 6; idx[26] = 2;
	idx[27] = 0; idx[28] = 1; idx[29] = 6; // 10th triangle
	*/
	////////// create vertex buffer
	int bytes = num_vtxs * sizeof(CUSTOMVERTEX4);
	HRESULT hr = pd3dev->CreateVertexBuffer(bytes, 0
		,D3DFVF_CUSTOMVERTEX4, D3DPOOL_DEFAULT, &m_pvb_std, 0);

	if(FAILED(hr))
		return false;

	VOID * ptr;
	hr = m_pvb_std->Lock(0, bytes, (void**) &ptr, 0);
	if(FAILED(hr))
		return false;

	memcpy(ptr, vtx2, bytes);
	m_pvb_std->Unlock();

	////////// create index buffer
	bytes = num_tris * sizeof(WORD) * 3;
	hr = pd3dev->CreateIndexBuffer(bytes, D3DUSAGE_WRITEONLY, 
		D3DFMT_INDEX16, D3DPOOL_DEFAULT, &m_pidb_std, NULL);

	if(FAILED(hr)){
		release_std_model();
		return false;
	}

	hr = m_pidb_std->Lock(0, bytes, (void**) &ptr, 0);
	if(FAILED(hr)){
		release_std_model();
		return false;
	}

	memcpy(ptr, idx2, bytes);
	m_pidb_std->Unlock();

	/////////// initialized material
	ZeroMemory(&m_mtrl_std, sizeof(m_mtrl_std));
	// completely white object
	m_mtrl_std.Ambient.r = 0.25f;
	m_mtrl_std.Ambient.g = 0.f;
	m_mtrl_std.Ambient.b = 0.f;
	m_mtrl_std.Ambient.a = 1.f;
	m_mtrl_std.Diffuse.r = 1.f;
	m_mtrl_std.Diffuse.g = 0.f;
	m_mtrl_std.Diffuse.b = 0.f;
	m_mtrl_std.Diffuse.a = 1.f;

	ZeroMemory(&m_light_std, sizeof(m_light_std));
	m_light_std.Diffuse.r = 0.5f;
	m_light_std.Diffuse.g = 0.5f;
	m_light_std.Diffuse.b = 0.5f;
	m_light_std.Diffuse.a = 0.5f;
//	m_light_std.Range = 500000;
	m_light_std.Type = D3DLIGHT_DIRECTIONAL;
	m_light_std.Direction.x = 0.f;
	m_light_std.Direction.y = 0.f;
	m_light_std.Direction.z = 1.f;

	return true;
}

bool c_ship::set_render_state(LPDIRECT3DDEVICE9 pd3dev)
{
	if(m_pvb == NULL){
		pd3dev->SetMaterial(&m_mtrl_std);
	}

	pd3dev->SetRenderState(D3DRS_AMBIENT, D3DXCOLOR(0.5, 0., 0., 0.5));
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);
	pd3dev->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	pd3dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
	pd3dev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	pd3dev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
	//	pd3dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	pd3dev->SetFVF(D3DFVF_CUSTOMVERTEX4);
	return true;
}

bool c_ship::render2d(LPDIRECT3DDEVICE9 pd3dev, ID3DXLine * pline,  
	c_d3d_ship2d & ship2d, Point3d & Xorg, Mat & Rwrld, 
	double rat, float cx, float cy, bool trail)
{
	if(m_data_type == ESDT_VDM){
		m_clr = D3DCOLOR_RGBA(255, 255, 0, 255);
	}else if(m_data_type == ESDT_BM){						
		m_clr = D3DCOLOR_RGBA(255, 0, 255, 255);
	}else if(m_data_type == ESDT_ARPA){
		m_clr = D3DCOLOR_RGBA(0, 0, 255, 255);
	}

	D3DXVECTOR2 pt[2];	
	pt[0].x = (float)(m_Xwrld.x * rat + cx);
	pt[0].y = (float)(-m_Xwrld.y * rat + cy);

	// velocity vector
	pt[1].x = (float)((m_Xwrld.x + m_Vwrld.x * 180) * rat + cx);
	pt[1].y = (float)(- (m_Xwrld.y + m_Vwrld.y * 180) * rat + cy);

	pline->Draw(pt, 2, D3DCOLOR_RGBA(255, 255, 255, 255));

	pline->Begin();
	if(trail){
		Point3d x;
		
		int c = 255;
		int c_step = 255 / ((int) m_trail.size() + 1); 
		if(m_trail_tail != m_trail_head){
			int i = m_trail_tail - 1;
			if(i < 0){
				i = (int) m_trail.size() - 1;
			}

			for(int i = m_trail_tail - 1, j = 0;;j++){
				bihtowrld(Xorg, Rwrld, m_trail[i], x);
				if(j & 0x1){
					pt[0].x = (float)(x.x * rat + cx);
					pt[0].y = (float)(-x.y * rat + cy);
				}else{
					pt[1].x = (float)(x.x * rat + cx);
					pt[1].y = (float)(-x.y * rat + cy);
				}
				pline->Draw(pt, 2, D3DCOLOR_RGBA(c, c, c, c));
				c -= c_step;

				if(i == m_trail_head)
					break;

				i--;
				if(i < 0)
					i = (int) m_trail.size() - 1;
			}
		}
	}

	pline->End();

	return ship2d.render(pd3dev, (float) (m_Xwrld.x * rat + cx), 
		(float) (-m_Xwrld.y * rat + cy), 1.0, (float) -m_cog, m_clr);
}

bool c_ship::render3d(LPDIRECT3DDEVICE9 pd3dev)
{
	if(m_pvb == NULL){	
		pd3dev->SetStreamSource(0, m_pvb_std, 0, sizeof(CUSTOMVERTEX4));
		pd3dev->SetIndices(m_pidb_std);
	}

	D3DXMATRIX Mwrld;
	// calculate world transformation
	if(m_to_bow != 0){
		double sy = (double) (m_to_bow + m_to_stern) / 100.;
		double sx = (double) (m_to_port + m_to_starboard) / 40.;
		D3DXMatrixScaling(&Mwrld, (float) sx, (float) sy, (float) sy);
	}else
		D3DXMatrixIdentity(&Mwrld);

	D3DXMATRIX Mtemp;
	D3DXMatrixRotationZ(&Mtemp, (float) -m_cog);
//	D3DXMatrixRotationX(&Mtemp, -0.25 * PI);
	Mwrld *= Mtemp;
	D3DXMatrixTranslation(&Mtemp, (float)m_Xwrld.x, (float)m_Xwrld.y, (float)m_Xwrld.z);
	//D3DXMatrixTranslation(&Mtemp, 0, -120, 0);
	Mwrld *= Mtemp;

	pd3dev->SetTransform(D3DTS_WORLD, &Mwrld);
	pd3dev->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, 0, 0, 22, 0, 10);

	return true;
}

void c_ship::release_std_model()
{
	m_pvb_std->Release();
	m_pvb_std = NULL;

	m_pidb_std->Release();
	m_pidb_std = NULL;
}

bool c_ship::load_original_model(LPDIRECT3DDEVICE9 pd3dev)
{
	return true;
}

void c_ship::release_original_model()
{
}

bool c_ship::render(LPDIRECT3DDEVICE9 pd3dev, ID3DXLine * pLine,
	c_d3d_dynamic_text & txt, Mat & Mtrn)
{
	{
		Point3d vvec = 10 * m_Vwrld;
		vvec += m_Xwrld;
		trans(Mtrn, m_Xwrld, m_Xview);
		trans(Mtrn, vvec, m_Vview);
	}

	float x = (float) m_Xview.x;
	float y = (float) m_Xview.y;
	D3DXVECTOR2 v[2];
	v[0] = D3DXVECTOR2(x - 15, y - 15);
	v[1] = D3DXVECTOR2(x + 15, y - 15);
	
	pLine->Begin();
	HRESULT hr = pLine->Draw(v, 2, m_clr);
	v[1] = D3DXVECTOR2(x - 15, y + 15);
	hr = pLine->Draw(v, 2, m_clr);
	v[0] = D3DXVECTOR2(x + 15, y + 15);
	hr = pLine->Draw(v, 2, m_clr);
	v[1] = D3DXVECTOR2(x + 15, y - 15);
	hr = pLine->Draw(v, 2, m_clr);

	v[0] = D3DXVECTOR2(x, y);
	v[1] = D3DXVECTOR2((float) m_Vview.x, (float) m_Vview.y);
	hr = pLine->Draw(v, 2, m_clr);
	pLine->End();

	float sx, sy;
	sprintf(m_buf, "%2.1f", m_dist);
	txt.get_text_size(sx, sy, m_buf);
	txt.render(pd3dev, m_buf, (float) (x-15), (float) (y+15),1., 
		(float)(0.25*PI),EDTC_RC,m_clr);
	sprintf(m_buf, "%s", m_name);
	txt.render(pd3dev, m_buf, (float) (x-15), (float) (y-15), 1., 
		(float) (0.5*PI), EDTC_LT, m_clr);
	sprintf(m_buf, "%s", get_ship_type_name(m_type));
	txt.render(pd3dev, m_buf, (float) (x+15), (float) (y-15), 1., 
		(float) (0.5*PI), EDTC_LB, m_clr);
	return true;
}
#endif


