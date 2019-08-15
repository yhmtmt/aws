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

#include "../util/aws_serial.h"
#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>

using namespace cv;

#include "f_aws1_nmea_sw.h"

f_aws1_nmea_sw::f_aws1_nmea_sw(const char * name): f_base(name), 
	m_state(NULL), m_ais_obj(NULL),
	m_aws_nmea_i(NULL), m_wx220_nmea_i(NULL), m_gff_nmea_i(NULL),
	m_ais_nmea_i(NULL), m_v104_nmea_i(NULL),
	m_aws_nmea_o(NULL), m_wx220_nmea_o(NULL), m_gff_nmea_o(NULL), 
	m_ais_nmea_o(NULL), m_v104_nmea_o(NULL),
	m_aws_ctrl(false), m_verb(false),
	m_aws_oint(1), m_wx220_oint(1), m_gff_oint(1), m_ais_oint(1), m_v104_oint(1),
	m_aws_ocnt(0), m_wx220_ocnt(0), m_gff_ocnt(0), m_ais_ocnt(0), m_v104_ocnt(0)
{
	register_fpar("state", (ch_base**)&m_state, typeid(ch_state).name(), "State output channel.");
	register_fpar("ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
	register_fpar("aws_nmea_i", (ch_base**)&m_aws_nmea_i, typeid(ch_nmea).name(), "Input Channel of aws_nmea.");
	register_fpar("aws_nmea_o", (ch_base**)&m_aws_nmea_o, typeid(ch_nmea).name(), "Output Channel of aws_nmea.");
	register_fpar("wx220_nmea_i", (ch_base**)&m_wx220_nmea_i, typeid(ch_nmea).name(), "Input Channel of wx220_nmea.");
	register_fpar("wx220_nmea_o", (ch_base**)&m_wx220_nmea_o, typeid(ch_nmea).name(), "Output Channel of wx220_nmea.");
	register_fpar("gff_nmea_i", (ch_base**)&m_gff_nmea_i, typeid(ch_nmea).name(), "Input Channel of gff_nmea.");
	register_fpar("gff_nmea_o", (ch_base**)&m_gff_nmea_o, typeid(ch_nmea).name(), "Output Channel of gff_nmea.");
	register_fpar("ais_nmea_i", (ch_base**)&m_ais_nmea_i, typeid(ch_nmea).name(), "Input Channel of ais_nmea.");
	register_fpar("ais_nmea_o", (ch_base**)&m_ais_nmea_o, typeid(ch_nmea).name(), "Output Channel of ais_nmea.");
	register_fpar("v104_nmea_i", (ch_base**)&m_v104_nmea_i, typeid(ch_nmea).name(), "Input Channel of v104_nmea.");
	register_fpar("v104_nmea_o", (ch_base**)&m_v104_nmea_o, typeid(ch_nmea).name(), "Output Channel of v104_nmea.");

	register_fpar("aws_ctrl", &m_aws_ctrl, "If yes, APB message is switched from fish finder to aws (default false)");
	register_fpar("awsint", &m_aws_oint, "Output interval of aws output channel (default 1)");
	register_fpar("wx220int", &m_wx220_oint, "Output interval of WX220 output channel (default 1)");
	register_fpar("gffint", &m_gff_oint, "Output interval of GPS fish finder output channel (default 1)");
	register_fpar("aisint", &m_ais_oint, "Output interval of AIS output channel (default 1)");
	register_fpar("v104int", &m_v104_oint, "Output interval of V104 output channel (default 1)");
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
    
    if(m_wx220_nmea_o && m_aws_ctrl){
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
	  m_wx220_nmea_o->push(m_nmea);			
	  m_wx220_out = true;				
	}	
	break;
      default:
	break;
      }
    }
  }
}

void f_aws1_nmea_sw::wx220_to_out()
{
  while(m_wx220_nmea_i->pop(m_nmea)){
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
    
    if(m_state && type == ENDT_DBT){
      const c_dbt * pdbt = dynamic_cast<const c_dbt*>(m_nmea_dec.decode(m_nmea));
      if(pdbt){
	m_state->set_depth(get_time(), pdbt->dm);
      }
    }
    
    if(m_verb)
      cout << "GFF > " << m_nmea << endl;
    
    if(m_aws_nmea_o)
      m_aws_nmea_o->push(m_nmea);
    
    if(m_wx220_nmea_o && !m_aws_ctrl){
      switch(type){
      case ENDT_APB:
      case ENDT_AAM:
      case ENDT_VTG:
      case ENDT_XTE:
	if(m_wx220_ocnt == 0){
	  if(m_verb)
	    cout << "AP < " << m_nmea << endl;
	  
	  m_wx220_nmea_o->push(m_nmea);
	  m_wx220_out = true;
	}
	break;				
      default:
	break;
      }
    }   
  }
}

void f_aws1_nmea_sw::ais_to_out()
{
  while(m_ais_nmea_i->pop(m_nmea)){
    e_nd_type type = get_nd_type(m_nmea);
    
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
    
    if(m_ais_obj){
      const c_nmea_dat * pnd = m_nmea_dec.decode(m_nmea);
      if(pnd){
	//pnd->show(cout);
	switch(pnd->get_type()){			  
	case ENDT_VDM1:
	  {
	    const c_vdm_msg1 * pvdm1 = dynamic_cast<const c_vdm_msg1*>(pnd);
	    m_ais_obj->push(get_time(), pvdm1->m_mmsi, pvdm1->m_lat, pvdm1->m_lon,
			    pvdm1->m_course, pvdm1->m_speed, pvdm1->m_heading);
	  }
	  break;
	case ENDT_VDM18:
	  {
	    const c_vdm_msg18 * pvdm18 = dynamic_cast<const c_vdm_msg18*>(pnd);
	    m_ais_obj->push(get_time(), pvdm18->m_mmsi, pvdm18->m_lat, pvdm18->m_lon, 
			    pvdm18->m_course, pvdm18->m_speed, pvdm18->m_heading);
	  }
	  break;
	case ENDT_VDM19:
	  {
	    const c_vdm_msg19 * pvdm19 = dynamic_cast<const c_vdm_msg19*>(pnd);
	    m_ais_obj->push(get_time(), pvdm19->m_mmsi, pvdm19->m_lat, pvdm19->m_lon, 
			    pvdm19->m_course, pvdm19->m_speed, pvdm19->m_heading);
	  }
	  break;
	}
	
	// clean up ais objects older than 5 minutes
	m_ais_obj->remove_old(get_time() - (long long) 300 * (long long) SEC);
      }
    }
  }
}

void f_aws1_nmea_sw::v104_to_out()
{
  while(m_v104_nmea_i->pop(m_nmea)){
    e_nd_type type = get_nd_type(m_nmea);
    
    if(m_state){
      switch(type){
      case ENDT_GGA: // lat, lon, alt
	{
	  const c_gga * pgga = dynamic_cast<const c_gga*>(m_nmea_dec.decode(m_nmea));
	  if(pgga){
	    m_state->set_position(get_time(), 
				  (float) (pgga->m_lat_dir == EGP_N ? pgga->m_lat_deg : -pgga->m_lat_deg),
				  (float) (pgga->m_lon_dir == EGP_E ? pgga->m_lon_deg : -pgga->m_lon_deg),
				  pgga->m_alt, pgga->m_geos);
	  }
	}
	break;
      case ENDT_VTG: // cog, sog
	{
	  const c_vtg * pvtg = dynamic_cast<const c_vtg*>(m_nmea_dec.decode(m_nmea));
	  if(pvtg){
	    m_state->set_velocity(get_time(), pvtg->crs_t, pvtg->v_n);
	  }
	}
      case ENDT_RMC: // time
	{
	  const c_rmc * prmc = dynamic_cast<const c_rmc*>(m_nmea_dec.decode(m_nmea));	  
	  if(prmc){
	    tmex tm;
	    tm.tm_year = prmc->m_yr + (m_tm.tm_year / 100) * 100;
	    tm.tm_mon = prmc->m_mn - 1;
	    tm.tm_mday = prmc->m_dy;
	    tm.tm_hour = prmc->m_h;
	    tm.tm_min = prmc->m_m;
	    tm.tm_sec = (int) prmc->m_s;
	    tm.tm_msec = (int)((prmc->m_s - tm.tm_sec) * 100);
	    tm.tm_isdst = -1;
	    f_base::set_time(tm);	  
	  }
	}
	break;
      case ENDT_ZDA: // time 
	{
	  const c_zda * pzda = dynamic_cast<const c_zda*>(m_nmea_dec.decode(m_nmea));
	  if(pzda){
	    tmex tm;
	    tm.tm_year = pzda->m_yr + (m_tm.tm_year / 100) * 100;
	    tm.tm_mon = pzda->m_mn - 1;
	    tm.tm_mday = pzda->m_dy;
	    tm.tm_hour = pzda->m_h;
	    tm.tm_min = pzda->m_m;
	    tm.tm_sec = (int) pzda->m_s;
	    tm.tm_msec = (int)((pzda->m_s - tm.tm_sec) * 100);
	    tm.tm_isdst = -1;
	    f_base::set_time(tm);	  
	    f_base::set_tz(pzda->m_lzh * 60 + pzda->m_lzm);
	  }
	}
	break;
      }
    }
    
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
      
      if(m_wx220_nmea_o && m_wx220_ocnt == 0){
	if(m_verb)
	  cout << "AP < " << m_nmea << endl;
	if(m_wx220_nmea_o)
	  m_wx220_nmea_o->push(m_nmea);
	m_wx220_out = true;
      }
      break;
    }    
  }
}

bool f_aws1_nmea_sw::proc()
{
  m_aws_out = m_wx220_out = m_gff_out = m_ais_out = false;
  
  if(m_aws_nmea_i)
	aws_to_out();

  if(m_wx220_nmea_i)
	wx220_to_out();

  if(m_gff_nmea_i)
	gff_to_out();

  if(m_ais_nmea_i)
	ais_to_out();

  if(m_v104_nmea_i)
	v104_to_out();

  if(m_aws_ocnt > 0)
    m_aws_ocnt--;
  
  if(m_aws_out == true)
    m_aws_ocnt = m_aws_oint;
  
  if(m_wx220_ocnt > 0)
    m_wx220_ocnt--;
  
  if(m_wx220_out == true)
    m_wx220_ocnt = m_wx220_oint;
  
  if(m_gff_ocnt > 0)
    m_gff_ocnt--;
  
  if(m_gff_out == true)
    m_gff_ocnt = m_gff_oint;
  
  if(m_ais_ocnt > 0)
    m_ais_ocnt--;
  
  if(m_ais_out == true)
    m_ais_ocnt = m_ais_oint;
  
  if(m_v104_ocnt > 0)
	  m_v104_ocnt--;

  if(m_v104_out == true)
	  m_v104_ocnt = m_v104_oint;
  return true;
}
 
