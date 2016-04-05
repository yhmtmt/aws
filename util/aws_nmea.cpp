#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_nmea.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_nmea.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_nmea.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <cstdio>
#include <stdlib.h>
#include <wchar.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include "aws_sock.h"
#include "aws_thread.h"

using namespace std;

#include "aws_nmea.h"

const char * str_nd_type[ENDT_UNDEF] = {
  "GGA", "GSA", "GSV", "RMC", "VTG", "ZDA", "GLL",
  "TTM", 
  "DBT", "MTW",
  "VDM", "VDO", "ABK",
  "APB", "AAM", "BOD", "BWC", "XTE", "RMB", "APA"
};

e_nd_type get_nd_type(const char * str)
{
	for(int i = 0; i < ENDT_UNDEF; i++){
		const char * st = str_nd_type[i];
		if(st[0] == str[3] && st[1] == str[4] && st[2] == str[5])
			return (e_nd_type) i;
	}
	return ENDT_UNDEF;
}


unsigned char calc_nmea_chksum(const char * str)
{
	unsigned char cs = 0x00;
	int i;
	for(i = 1; str[i] != '*' && str[i] != '\0'; i++)
		cs ^= str[i];
	return cs;
}

bool eval_nmea_chksum(const char * str)
{
	
	unsigned char cs = 0x00;
	int i;
	for(i = 1; str[i] != '*' && str[i] != '\0' && i < 83; i++)
		cs ^= str[i];

	if(i == 83)
		return false;

	unsigned char csa = (unsigned char) htoi(&str[i+1]);
	return cs == csa;
}

//////////////////////////////////////////////// string handler

unsigned int htoi(const char * str)
{
	unsigned int r = 0;
	for(int i = 0; str[i] != '\0'; i++){
		r *= 16;
		r += (str[i] > 'a' ? str[i] - 'a' + 10: 
		(str[i] > 'A' ? str[i] - 'A'  + 10: str[i] - '0') );
	}

	return r;
}

bool parstrcmp(const char * str1, const char * str2)
{
	for(int i = 0; str1[i] != '\0' && str2[i] != '\0'; i++)
		if(str1[i] != str2[i])
			return false;
	return true;
}

int parstrcpy(char * str, const char * src, int num)
{
	int i;
	for(i = 0; i < num; i++)
		str[i] = src[i];
	str[i] = '\0';
	return i;
}

int parstrcpy(char * str, const char * src, char delim, int max_buf)
{
	int i;
	
	for(i = 0; src[i] != delim && src[i] != '\0' && i < max_buf; i++)
		str[i] = src[i];

	str[i] = '\0';
	return (src[i] == delim ? i : -1);
}



///////////////////////////////////////////// navdat decoder
const c_nmea_dat * c_nmea_dec::decode(const char * str)
{
	if(!eval_nmea_chksum(str)){
	  cerr << "Check sum is not valid. " << str << endl;
		return NULL;
	}

	e_nd_type nt = get_nd_type(str);
	if(nt == ENDT_UNDEF)
		return NULL;

	c_nmea_dat * pnd = NULL;
	switch(nt){
	case ENDT_GGA:
	  pnd = &gga;
		break;
	case ENDT_GSA:
		pnd = &gsa;
		break;
	case ENDT_GSV:
		pnd = &gsv;
		break;
	case ENDT_RMC:
		pnd = &rmc;
		break;
	case ENDT_VTG:
		pnd = &vtg;
		break;
	case ENDT_ZDA:
		pnd = &zda;
		break;
	case ENDT_TTM:
		pnd = &ttm;
		break;
	case ENDT_DBT:
		pnd = &dbt;
		break;
	case ENDT_MTW:
		pnd = &mtw;
		break;
	case ENDT_VDM:	
		pnd = vdmdec.dec(str);
		break;
	case ENDT_VDO:
		pnd = vdodec.dec(str);
		break;
	case ENDT_ABK:
		pnd = &abk;
		break;
	case ENDT_UNDEF:
		pnd = NULL;
	};
	
	if(pnd && pnd->dec(str)){	
		pnd->m_toker[0] = str[1];
		pnd->m_toker[1] = str[2];
		pnd->m_cs = true;
	}else{
		return NULL;
	}
	return pnd;
}

//////////////////////////////////////////////// ttm decoder
bool c_ttm::dec(const char * str)
{
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	char tok[32];
	while(ipar < 16){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;

		switch(ipar){
		case 0: // $**TTM
			break;
		case 1:
			m_id = atoi(buf);
			break;
		case 2:
			m_dist = (float) atof(buf);
			break;
		case 3:
			m_bear = (float) atof(buf);
			break;
		case 4:
			m_is_bear_true = (buf[0] == 'T' ? true : false);
			break;
		case 5:
			m_spd = (float) atof(buf);
			break;
		case 6:
			m_crs = (float) atof(buf);
			break;
		case 7:
			m_is_crs_true = (buf[0] == 'T' ? true : false);
			break;
		case 8:
			m_dcpa = (float) atof(buf);
			break;
		case 9:
			m_tcpa = (float) atof(buf);
			break;
		case 10:
			m_dist_unit = buf[0];
			break;
		case 11:
			m_data[0] = '\0';
			if(strlen(buf) >= 20){
				break;
			}
			strcpy(m_data, buf); 
			break;
		case 12:
			m_state = buf[0];
			break;
		case 13:
			m_is_ref = (buf[0] == 'R' ? true : false);
			break;
		case 14:
			parstrcpy(tok, buf, 2);
			m_utc_h = (char) atoi(tok);
			parstrcpy(tok, buf+2, 2);
			m_utc_m = (char) atoi(tok);
			parstrcpy(tok, buf+4, 2);
			m_utc_s = (char) atoi(tok);
			parstrcpy(tok, buf+7, 2);
			m_utc_ms = (char) atoi(tok);
			break;
		case 15:
			m_is_auto = (buf[0] == 'A' ? true : false);
			break;
		}
		ipar++;
	}

	return true;
}

//////////////////////////////////////////////// dbt decoder
bool c_dbt::dec(const char * str)
{
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	while(ipar < 7){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;

		switch(ipar){
		case 0: // $**DBT
			break;
		case 1:
			dfe = (float) atof(buf);
			break;
		case 2:
			if(buf[0] != 'f')
				goto dbterr;
			break;
		case 3:
			dm = (float) atof(buf);
			break;
		case 4:
			if(buf[0] != 'M')
				goto dbterr;
			break;
		case 5:
			dfa = (float) atof(buf);
			break;
		case 6:
			if(buf[0] != 'F')
				goto dbterr;
			break;
		}
		ipar++;
	}

	return true;
dbterr:
	return false;
}

//////////////////////////////////////////////// mtw decoder
bool c_mtw::dec(const char * str)
{
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	while(ipar < 7){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;

		switch(ipar){
		case 0: // $**MTW
			break;
		case 1:
			t = (float) atof(buf);
			break;
		case 2:
			if(buf[0] != 'C')
				goto mtwerr;
			break;
		}
		ipar++;
	}

	return true;
mtwerr:
	return false;
}


