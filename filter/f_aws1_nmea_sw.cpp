#include "stdafx.h"

// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_aws1_nmea_sw.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_nmea_sw.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_nmea_sw.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include "../util/aws_stdlib.h"

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_nmeadec.h"

#include "../channel/ch_base.h"
#include "../channel/ch_nmea.h"
#include "f_aws1_nmea_sw.h"

f_aws1_nmea_sw::f_aws1_nmea_sw(const char * name): f_base(name), m_aws_nmea_i(NULL), m_ap_nmea_i(NULL), m_gff_nmea_i(NULL),
	m_ais_nmea_i(NULL), m_aws_nmea_o(NULL), m_ap_nmea_o(NULL), m_gff_nmea_o(NULL), m_ais_nmea_o(NULL),
	m_aws_ctrl(false), m_aws_oint(1), m_ap_oint(1), m_gff_oint(1), m_ais_oint(1),
	m_aws_ocnt(0), m_ap_ocnt(0), m_gff_ocnt(0), m_ais_ocnt(0)
{
	register_fpar("aws_ctrl", &m_aws_ctrl, "If yes, APB message is switched from fish finder to aws (default false)");
	register_fpar("awsint", &m_aws_oint, "Output interval of aws output channel (default 1)");
	register_fpar("apint", &m_ap_oint, "Output interval of autopilot output channel (default 1)");
	register_fpar("gffint", &m_gff_oint, "Output interval of GPS fish finder output channel (default 1)");
	register_fpar("aisint", &m_ais_oint, "Output interval of AIS output channel (default1)");
}

f_aws1_nmea_sw::~f_aws1_nmea_sw()
{
}

bool f_aws1_nmea_sw::init_run()
{
	if(m_chin.size() != 4){
		cerr << "Error in f_aws_1_nmea_sw::init_run. Expected number of input channels is 4." << endl;
		return false;
	}

	if(m_chout.size() != 4){
		cerr << "Error in f_aws_1_nmea_sw::init_run. Expected number of output channels is 4." << endl;
		return false;
	}

	m_aws_nmea_i = dynamic_cast<ch_nmea*>(m_chin[0]);
	m_ap_nmea_i = dynamic_cast<ch_nmea*>(m_chin[1]);
	m_gff_nmea_i = dynamic_cast<ch_nmea*>(m_chin[2]);
	m_ais_nmea_i = dynamic_cast<ch_nmea*>(m_chin[3]);
	if(m_aws_nmea_i == NULL || m_ap_nmea_i == NULL || m_gff_nmea_i == NULL || m_ais_nmea_i == NULL){
		cerr << "Error in f_aws_1_nmea_sw::init_run. The input channel should be ch_nmea." << endl;
		return false;
	}

	m_aws_nmea_o = dynamic_cast<ch_nmea*>(m_chout[0]);
	m_ap_nmea_o = dynamic_cast<ch_nmea*>(m_chout[1]);
	m_gff_nmea_o = dynamic_cast<ch_nmea*>(m_chout[2]);
	m_ais_nmea_o = dynamic_cast<ch_nmea*>(m_chout[3]);
	if(m_aws_nmea_o == NULL || m_ap_nmea_o == NULL || m_gff_nmea_o == NULL || m_ais_nmea_o == NULL){
		cerr << "Error in f_aws_1_nmea_sw::init_run. The input channel should be ch_nmea." << endl;
		return false;
	}		


	return true;
}

void f_aws1_nmea_sw::destroy_run()
{
	m_aws_nmea_i = m_ap_nmea_i = m_gff_nmea_i = m_ais_nmea_i = NULL;
	m_aws_nmea_o = m_ap_nmea_o = m_gff_nmea_o = m_ais_nmea_o = NULL;
}

bool f_aws1_nmea_sw::proc()
{
	bool m_aws_out, m_ap_out, m_gff_out, m_ais_out;
	m_aws_out = m_ap_out = m_gff_out = m_ais_out = false;

	while(m_aws_nmea_i->pop(m_nmea)){
		if(m_aws_ctrl && m_nmea[3] == 'A' && m_nmea[4] == 'P' && m_nmea[5] == 'B')
			if(m_aws_ocnt == 0){
				m_ap_nmea_o->push(m_nmea);			
				m_ap_out = true;
			}	
	}

	while(m_ap_nmea_i->pop(m_nmea)){
		if(m_aws_ocnt == 0){
			m_aws_nmea_o->push(m_nmea);
			m_aws_out = true;
		}
	}


	while(m_gff_nmea_i->pop(m_nmea)){
		m_aws_nmea_o->push(m_nmea);
		if(m_nmea[3] != 'R' && m_nmea[4] == 'M' && m_nmea[5] == 'C'){
			if(m_gff_ocnt == 0){
				m_gff_nmea_o->push(m_nmea);
				m_gff_out = true;
			}
			if(m_ais_ocnt == 0){
				m_ais_nmea_o->push(m_nmea);
				m_ais_out = true;
			}
		}
		if(m_nmea[3] != 'G' && m_nmea[4] == 'G' && m_nmea[5] == 'A'){
			if(m_gff_ocnt == 0){
				m_gff_nmea_o->push(m_nmea);
				m_gff_out = true;
			}
			if(m_ais_ocnt == 0){
				m_ais_nmea_o->push(m_nmea);
				m_ais_out = true;
			}
		}
		if(m_nmea[3] != 'V' && m_nmea[4] == 'T' && m_nmea[5] == 'G'){
			if(m_gff_ocnt == 0){
				m_gff_nmea_o->push(m_nmea);
				m_gff_out = true;
			}
			if(m_ais_ocnt == 0){
				m_ais_nmea_o->push(m_nmea);
				m_ais_out = true;
			}
		}
		if(!m_aws_ctrl && m_nmea[3] != 'A' && m_nmea[4] == 'P' && m_nmea[5] == 'B'){
			if(m_ap_ocnt == 0){
				m_ap_nmea_o->push(m_nmea);
				m_ap_out = true;
			}
		}
	}

	while(m_ais_nmea_i->pop(m_nmea)){
		m_aws_nmea_o->push(m_nmea);
		if(m_nmea[3] != 'V' && m_nmea[4] == 'D' && m_nmea[5] == 'M'){
			if(m_gff_ocnt == 0){
				m_gff_nmea_o->push(m_nmea);
				m_gff_out = true;
			}
		}		
	}

	if(m_aws_out)
		m_aws_ocnt = m_aws_oint;
	else if(m_aws_ocnt > 0)
		m_aws_ocnt--;

	if(m_ap_out)
		m_ap_ocnt = m_ap_oint;
	else if(m_ap_ocnt > 0)
		m_ap_ocnt--;

	if(m_gff_out)
		m_gff_ocnt = m_gff_oint;
	else if(m_gff_ocnt > 0)
		m_gff_ocnt--;

	if(m_ais_out)
		m_ais_ocnt = m_ais_oint;
	else if(m_ais_ocnt > 0)
		m_ais_ocnt--;

	return true;
}
