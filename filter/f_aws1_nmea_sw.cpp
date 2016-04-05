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
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include <opencv2/opencv.hpp>

using namespace cv;

#include "f_aws1_nmea_sw.h"

f_aws1_nmea_sw::f_aws1_nmea_sw(const char * name): f_base(name), 
	m_state(NULL),
	m_aws_nmea_i(NULL), m_ap_nmea_i(NULL), m_gff_nmea_i(NULL),
	m_ais_nmea_i(NULL), m_gps_nmea_i(NULL),
	m_aws_nmea_o(NULL), m_ap_nmea_o(NULL), m_gff_nmea_o(NULL), 
	m_ais_nmea_o(NULL), m_gps_nmea_o(NULL),
	m_aws_ctrl(false), m_verb(false),
	m_aws_oint(1), m_ap_oint(1), m_gff_oint(1), m_ais_oint(1), m_gps_oint(1),
	m_aws_ocnt(0), m_ap_ocnt(0), m_gff_ocnt(0), m_ais_ocnt(0), m_gps_ocnt(0)
{
	register_fpar("state", (ch_base**)&m_state, typeid(m_state).name(), "State output channel.");
	register_fpar("aws_nmea_i", (ch_base**)&m_aws_nmea_i, typeid(ch_nmea).name(), "Input Channel of aws_nmea.");
	register_fpar("aws_nmea_o", (ch_base**)&m_aws_nmea_o, typeid(ch_nmea).name(), "Output Channel of aws_nmea.");
	register_fpar("ap_nmea_i", (ch_base**)&m_ap_nmea_i, typeid(ch_nmea).name(), "Input Channel of ap_nmea.");
	register_fpar("ap_nmea_o", (ch_base**)&m_ap_nmea_o, typeid(ch_nmea).name(), "Output Channel of ap_nmea.");
	register_fpar("gff_nmea_i", (ch_base**)&m_gff_nmea_i, typeid(ch_nmea).name(), "Input Channel of gff_nmea.");
	register_fpar("gff_nmea_o", (ch_base**)&m_gff_nmea_o, typeid(ch_nmea).name(), "Output Channel of gff_nmea.");
	register_fpar("ais_nmea_i", (ch_base**)&m_ais_nmea_i, typeid(ch_nmea).name(), "Input Channel of ais_nmea.");
	register_fpar("ais_nmea_o", (ch_base**)&m_ais_nmea_o, typeid(ch_nmea).name(), "Output Channel of ais_nmea.");
	register_fpar("gps_nmea_i", (ch_base**)&m_gps_nmea_i, typeid(ch_nmea).name(), "Input Channel of gps_nmea.");
	register_fpar("gps_nmea_o", (ch_base**)&m_gps_nmea_o, typeid(ch_nmea).name(), "Output Channel of gps_nmea.");

	register_fpar("aws_ctrl", &m_aws_ctrl, "If yes, APB message is switched from fish finder to aws (default false)");
	register_fpar("awsint", &m_aws_oint, "Output interval of aws output channel (default 1)");
	register_fpar("apint", &m_ap_oint, "Output interval of autopilot output channel (default 1)");
	register_fpar("gffint", &m_gff_oint, "Output interval of GPS fish finder output channel (default 1)");
	register_fpar("aisint", &m_ais_oint, "Output interval of AIS output channel (default 1)");
	register_fpar("gpsint", &m_gps_oint, "Output interval of GPS output channel (default 1)");
	register_fpar("verb", &m_verb, "For debug.");
}

f_aws1_nmea_sw::~f_aws1_nmea_sw()
{
}

bool f_aws1_nmea_sw::init_run()
{
  return true;
}

void f_aws1_nmea_sw::destroy_run()
{
}

void f_aws1_nmea_sw::aws_to_out()
{
	while(m_aws_nmea_i->pop(m_nmea)){
		e_nd_type type = get_nd_type(m_nmea);

		if(m_verb){
			cout << "AWS > " << m_nmea << endl;
		}

		if(m_ap_nmea_o && m_aws_ctrl){
			switch(type){
			case ENDT_APB:
			case ENDT_AAM:
			case ENDT_BOD:
			case ENDT_BWC:
			case ENDT_XTE:
			case ENDT_RMB:
			case ENDT_APA:
				if(m_aws_ocnt == 0){
					if(m_verb){
						cout << "AP < " << m_nmea << endl;
					}
					m_ap_nmea_o->push(m_nmea);			
					m_ap_out = true;				
				}	
				break;
			default:
				break;
			}
		}
	}
}

void f_aws1_nmea_sw::ap_to_out()
{
	while(m_ap_nmea_i->pop(m_nmea)){
		if(m_verb)
			cout << "AP > " << m_nmea << endl;
		if(m_aws_ocnt == 0){
			if(m_verb)
				cout << "AWS < " << m_nmea << endl;
			if(m_aws_nmea_o)
				m_aws_nmea_o->push(m_nmea);
			m_aws_out = true;
		}
	}
}

void f_aws1_nmea_sw::gff_to_out()
{
	while(m_gff_nmea_i->pop(m_nmea)){
		e_nd_type type = get_nd_type(m_nmea);

		// toker forced to be "GF" to avoid mixing with the GPS's nmea.
		m_nmea[1] = 'G';
		m_nmea[2] = 'F';
		if(m_verb)
			cout << "GFF > " << m_nmea << endl;

		if(m_aws_nmea_o)
			m_aws_nmea_o->push(m_nmea);

		if(m_ap_nmea_o && !m_aws_ctrl){
			switch(type){
			case ENDT_APB:
			case ENDT_AAM:
			case ENDT_VTG:
			case ENDT_XTE:
				if(m_ap_ocnt == 0){
					if(m_verb)
						cout << "AP < " << m_nmea << endl;

					m_ap_nmea_o->push(m_nmea);
					m_ap_out = true;
				}
				break;				
			default:
				break;
			}
		}

		if(m_state && type == ENDT_DBT){
			const c_dbt * pdbt = dynamic_cast<const c_dbt*>(m_nmea_dec.decode(m_nmea));
			if(pdbt){
				m_state->set_depth(pdbt->dm);
			}
		}
	}
}

void f_aws1_nmea_sw::ais_to_out()
{
	while(m_ais_nmea_i->pop(m_nmea)){
		e_nd_type type = get_nd_type(m_nmea);

		m_nmea[1] = 'A';
		m_nmea[2] = 'I';

		if(m_verb)
			cout << "AIS > " << m_nmea << endl;
		
		if(m_aws_nmea_o)
			m_aws_nmea_o->push(m_nmea);

		if(m_gff_nmea_o &&  type == ENDT_VDM){
			if(m_gff_ocnt == 0){
				if(m_verb)
					cout << "GFF < " << m_nmea << endl;

				m_gff_nmea_o->push(m_nmea);
				m_gff_out = true;
			}
		}		
	}
}

void f_aws1_nmea_sw::gps_to_out()
{
	while(m_gps_nmea_i->pop(m_nmea)){
		e_nd_type type = get_nd_type(m_nmea);

		if(m_verb)
			cout << "GPS > " << m_nmea << endl;

		if(m_aws_nmea_o)
			m_aws_nmea_o->push(m_nmea);

		switch(type){
		case ENDT_RMC:
		case ENDT_GGA:
		case ENDT_GLL:
			if(m_ais_nmea_o && m_ais_ocnt == 0){
				if(m_verb)
					cout << "AIS < " << m_nmea << endl;
				if(m_ais_nmea_o)
					m_ais_nmea_o->push(m_nmea);
				m_ais_out = true;
			}
			break;
		case ENDT_VTG:
			if(m_ais_nmea_o && m_ais_ocnt == 0){
				if(m_verb)
					cout << "AIS < " << m_nmea << endl;

				if(m_ais_nmea_o)
					m_ais_nmea_o->push(m_nmea);
				m_ais_out = true;
			}

			if(m_ap_nmea_o && m_ap_ocnt == 0){
				if(m_verb)
					cout << "AP < " << m_nmea << endl;
				if(m_ap_nmea_o)
					m_ap_nmea_o->push(m_nmea);
				m_ap_out = true;
			}
			break;
		}

		if(m_state){
			switch(type){
			case ENDT_GGA: // lat, lon, alt
				{
					const c_gga * pgga = dynamic_cast<const c_gga*>(m_nmea_dec.decode(m_nmea));
					if(pgga){
						m_state->set_position(
							(float) (pgga->m_lat_dir == EGP_E ? pgga->m_lat_deg : -pgga->m_lat_deg),
							(float) (pgga->m_lon_dir == EGP_N ? pgga->m_lon_deg : -pgga->m_lon_deg),
							pgga->m_alt, pgga->m_geos);
					}
				}
				break;
			case ENDT_VTG: // cog, sog
				{
					const c_vtg * pvtg = dynamic_cast<const c_vtg*>(m_nmea_dec.decode(m_nmea));
					if(pvtg){
						m_state->set_velocity(pvtg->crs_t, pvtg->v_n);
					}
				}
			}
		}
	}
}

bool f_aws1_nmea_sw::proc()
{
  m_aws_out = m_ap_out = m_gff_out = m_ais_out = false;
  
  if(m_aws_nmea_i)
	aws_to_out();

  if(m_ap_nmea_i)
	ap_to_out();

  if(m_gff_nmea_i)
	gff_to_out();

  if(m_ais_nmea_i)
	ais_to_out();

  if(m_gps_nmea_i)
	gps_to_out();

  if(m_aws_ocnt > 0)
    m_aws_ocnt--;
  
  if(m_aws_out == true)
    m_aws_ocnt = m_aws_oint;
  
  if(m_ap_ocnt > 0)
    m_ap_ocnt--;
  
  if(m_ap_out == true)
    m_ap_ocnt = m_ap_oint;
  
  if(m_gff_ocnt > 0)
    m_gff_ocnt--;
  
  if(m_gff_out == true)
    m_gff_ocnt = m_gff_oint;
  
  if(m_ais_ocnt > 0)
    m_ais_ocnt--;
  
  if(m_ais_out == true)
    m_ais_ocnt = m_ais_oint;
  
  if(m_gps_ocnt > 0)
	  m_gps_ocnt--;

  if(m_gps_out == true)
	  m_gps_ocnt = m_gps_oint;
  return true;
}
 
