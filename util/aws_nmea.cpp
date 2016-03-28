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
	"GGA", "GSA", "GSV", "RMC", "VTG", "ZDA", 
	"TTM", 
	"DBT", "MTW",
	"VDM", "VDO", "ABK"
};

c_nmea_dat * (*nmea_dec[ENDT_UNDEF])(const char * str) = 
{
	c_gga::dec_gga, c_gsa::dec_gsa, c_gsv::dec_gsv, c_rmc::dec_rmc, c_vtg::dec_vtg, c_zda::dec_zda,
	c_ttm::dec_ttm, 
	c_dbt::dec_dbt, c_mtw::dec_mtw,
	c_vdm::dec_vdm, c_vdm::dec_vdo, c_abk::dec_abk
};

e_nd_type get_nd_type(const char * str)
{
	for(int i = 0; i < ENDT_UNDEF; i++){
		const char * st = str_nd_type[i];
		if(st[0] == str[3] && str[1] == str[4] && str[2] == str[5])
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

bool eval_nmea_chksum(const char * str){
	
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
c_nmea_dat * c_nmea_dat::dec_nmea_dat(const char * str)
{
	if(!eval_nmea_chksum(str)){
		return NULL;
	}

	e_nd_type nt = get_nd_type(str);
	if(nt == ENDT_UNDEF)
		return NULL;

	c_nmea_dat * pnd = nmea_dec[nt](str);

	if(pnd){
		pnd->m_toker[0] = str[1];
		pnd->m_toker[1] = str[2];
		pnd->m_cs = true;
	}

	return pnd;
}

//////////////////////////////////////////////// ttm decoder
c_nmea_dat * c_ttm::dec_ttm(const char * str)
{
	c_ttm * pnd = new c_ttm;
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
			pnd->m_id = atoi(buf);
			break;
		case 2:
			pnd->m_dist = (float) atof(buf);
			break;
		case 3:
			pnd->m_bear = (float) atof(buf);
			break;
		case 4:
			pnd->m_is_bear_true = (buf[0] == 'T' ? true : false);
			break;
		case 5:
			pnd->m_spd = (float) atof(buf);
			break;
		case 6:
			pnd->m_crs = (float) atof(buf);
			break;
		case 7:
			pnd->m_is_crs_true = (buf[0] == 'T' ? true : false);
			break;
		case 8:
			pnd->m_dcpa = (float) atof(buf);
			break;
		case 9:
			pnd->m_tcpa = (float) atof(buf);
			break;
		case 10:
			pnd->m_dist_unit = buf[0];
			break;
		case 11:
			pnd->m_data[0] = '\0';
			if(strlen(buf) >= 20){
				break;
			}
			strcpy(pnd->m_data, buf); 
			break;
		case 12:
			pnd->m_state = buf[0];
			break;
		case 13:
			pnd->m_is_ref = (buf[0] == 'R' ? true : false);
			break;
		case 14:
			parstrcpy(tok, buf, 2);
			pnd->m_utc_h = (char) atoi(tok);
			parstrcpy(tok, buf+2, 2);
			pnd->m_utc_m = (char) atoi(tok);
			parstrcpy(tok, buf+4, 2);
			pnd->m_utc_s = (char) atoi(tok);
			parstrcpy(tok, buf+7, 2);
			pnd->m_utc_ms = (char) atoi(tok);
			break;
		case 15:
			pnd->m_is_auto = (buf[0] == 'A' ? true : false);
			break;
		}
		ipar++;
	}

	return pnd;
}

//////////////////////////////////////////////// dbt decoder
c_nmea_dat * c_dbt::dec_dbt(const char * str)
{
	c_dbt * pnd = new c_dbt;
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
			pnd->dfe = (float) atof(buf);
			break;
		case 2:
			if(buf[0] != 'f')
				goto dbterr;
			break;
		case 3:
			pnd->dfa = (float) atof(buf);
			break;
		case 4:
			if(buf[0] != 'M')
				goto dbterr;
			break;
		case 5:
			pnd->dfa = (float) atof(buf);
			break;
		case 6:
			if(buf[0] != 'F')
				goto dbterr;
			break;
		}
		ipar++;
	}

	return pnd;
dbterr:
	delete pnd;
	return NULL;
}

//////////////////////////////////////////////// mtw decoder
c_nmea_dat * c_mtw::dec_mtw(const char * str)
{
	c_mtw * pnd = new c_mtw;
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
			pnd->t = (float) atof(buf);
			break;
		case 2:
			if(buf[0] != 'C')
				goto mtwerr;
			break;
		}
		ipar++;
	}

	return pnd;
mtwerr:
	delete pnd;
	return NULL;
}

