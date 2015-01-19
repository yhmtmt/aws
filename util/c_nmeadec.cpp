#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_nmeadec.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_nmeadec.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_nmeadec.cpp.  If not, see <http://www.gnu.org/licenses/>. 
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

#include "c_nmeadec.h"

unsigned int htoi(const char * str);
unsigned char decchar(unsigned char c6);
unsigned char encchar(unsigned char c8);
bool parstrcmp(const char * str1, const char * str2);
int parstrcpy(char * str, const char * src, int num);
int parstrcpy(char * str, const char * src, char delim, int max_buf = 32);

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

//////////////////////////////////////////////////////////////// s_binary_message
bool s_binary_message::set_msg_hex(char * buf)
{
	int imsg = 0;
	len = 0;
	unsigned char u;
	bool odd = false;
	while(*buf){
		if(*buf <= '9' && *buf >= '0') 
			u = *buf - '0';
		else if(*buf <= 'F' && *buf >= 'A')
			u = *buf - 'A' + 10;
		else{
			len = 0;
			return false;
		}

		if(!odd){
			msg[imsg] = u << 4;
		}else{
			msg[imsg] |= u;
			imsg++;
		}
		len += 4;
		odd = !odd;
		buf++;
	}
	return true;
}

bool s_binary_message::set_msg_bin(char * buf)
{
	int imsg = 0;
	len = 0;
	int mask = 0x80;
	msg[imsg] = 0;
	while(*buf){
		if(*buf == '0'){
		}else if(*buf == '1'){
			msg[imsg] |= mask;
		}else{
			len = 0;
			return false;
		}

		mask >>= 1;
		len++;
		buf++;
		if(mask == 0){
			mask = 0x80;
			imsg++;
			msg[imsg] = 0;
		}
	}
	return true;
}

bool s_binary_message::set_msg_c8(char * buf)
{
	int imsg = 0;
	len = 0;
	while(*buf){
		msg[imsg] = *buf;
		len += 8;
	}

	return true;
}

bool s_binary_message::set_msg_c6(char * buf)
{
	int imsg = 0; 
	unsigned char u;
	len = 0;
	while(*buf){
		// 0 rem 6 res
		u = encchar(*buf);
		msg[imsg] = u << 2;
		len += 6;
		buf++;
		if(!*buf)
			break;

		// 2 rem 4 res
		u = encchar(*buf);
		msg[imsg] |= (u & 0x30) >> 4;
		imsg++;
		msg[imsg] = (u & 0x0F) << 4;
		len += 6;
		buf++;
		if(!*buf)
			break;

		// 4 rem 2 res
		u = encchar(*buf);
		msg[imsg] |= (u & 0x3C) >> 2;
		imsg++;
		msg[imsg] = (u & 0x03) << 6;
		len += 6;
		buf++;
		if(!*buf)
			break;

		// 6 rem 0 res
		u = encchar(*buf);
		msg[imsg] |= u;
		imsg++;
		len += 6;
		buf++;
	}
	return true;
}

bool s_binary_message::set_msg_pvc2(
					unsigned char id, 
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/)
{
	// |0-9| DAC = 0
	// |10-15| FI = 0
	msg[0] = 0;
	msg[1] = 0;

	// |16| AckReq = 0
	// |17-27| TextSeq = 0
	msg[2] = 0;
	msg[3] = 0;

	// id -> 8bit
	msg[3] |= (id & 0x0F);
	msg[4] = (id & 0xF0);

	// sog -> 10bit
	msg[4] |= (sog & 0x000F);
	msg[5] = (sog & 0x03F0) >> 2;

	// cog -> 12bit
	msg[5] |= (cog & 0x0003);
	msg[6] = (cog & 0x03FC) >> 2;
	msg[7] = (cog & 0x0C00) >> 4;

	// lon -> 28bit
	msg[7] |= (lon & 0x0000003F);
	msg[8] = (lon & 0x00003FC0) >> 6;
	msg[9] = (lon & 0x003FC000) >> 14;
	msg[10] = (lon & 0x0FC00000) >> 20;

	// lat -> 27bit
	msg[10] |= (lat & 0x00000003);
	msg[11] = (lat & 0x000003FC) >> 2;
	msg[12] = (lat & 0x0003FC00) >> 10;
	msg[13] = (lat & 0x03FC0000) >> 18;
	msg[14] = (lat & 0x04000000) >> 19;
	
	len = 113;
	return true;
}

bool s_binary_message::get_msg_pvc2(
					unsigned char & id,
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					int & lon /* 28bit 1/10000 minutes / unit = 1/600000 deg /unit*/ , 
					int & lat /* 27bit 1/10000 minutes / unit = 1/600000 deg /unit*/)
{
	id = (msg[4] & 0xF0) | (msg[3] & 0x0F);
	cout << "id received " << (int) id << endl;
	sog = (unsigned short) (
		((msg[5] & 0xFC) << 2) | ((msg[4] & 0x0F))
		);
	cog = (unsigned short) (
		((msg[7] & 0xC0) << 4) | ((msg[6]) << 2) | ((msg[5] & 0x03))
		);

	if(cog < 0 || cog > 3600)
		return false;

	lon = (int) (
		((msg[10] & 0xFC) << 20) | ((msg[9]) << 14) | ((msg[8]) << 6) | ((msg[7] & 0x3F))
		);
	lat = (int) (
		(((msg[14] & 0x80) << 19) 
		| ((msg[13]) << 18) 
		| ((msg[12]) << 10) 
		| ((msg[11]) << 2) 
		| (msg[10] & 0x03))
		);
	return true;
}

bool s_binary_message::set_msg_pvc3(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					unsigned short dist /* 10bit 0.1 mile / unit */,
					unsigned short bear /* 12bit 0.1 deg / unit */)
{
	// |0-9| DAC = 0
	// |10-15| FI = 0
	msg[0] = 0;
	msg[1] = 0;

	// |16| AckReq = 0
	// |17-27| TextSeq = 0
	msg[2] = 0;
	msg[3] = 0;

	// id -> 8bit
	msg[3] |= (id & 0x0F);
	msg[4] = (id & 0xF0);

	// sog -> 10bit
	msg[4] |= (sog & 0x000F);
	msg[5] = (sog & 0x03F0) >> 2;

	// cog -> 12bit
	msg[5] |= (cog & 0x0003);
	msg[6] = (cog & 0x03FC) >> 2;
	msg[7] = (cog & 0x0C00) >> 4;

	// dist -> 10bit
	msg[7] |= (dist & 0x0000003F);
	msg[8] = (dist & 0x000003C0) >> 2;

	// bear -> 12bit
	msg[8] |= (bear & 0x0000000F);
	msg[9] = (bear & 0x00000FF0) >> 4;

	len = 80;
	return true;
}

bool s_binary_message::get_msg_pvc3(
					unsigned char & id, /* 8bit */
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					unsigned short & dist /* 10bit 0.1 mile / unit */,
					unsigned short & bear /* 12bit 0.1 deg / unit */)
{
	id = (msg[4] & 0xF0) | (msg[3] & 0x0F);
	cout << "id received " << (int) id << endl;
	sog = (unsigned short) (
		((msg[5] & 0xFC) << 2) | ((msg[4] & 0x0F))
		);
	cog = (unsigned short) (
		((msg[7] & 0xC0) << 4) | ((msg[6]) << 2) | ((msg[5] & 0x03))
		);

	if(cog < 0 || cog > 3600)
		return false;

	dist = (int) (
		((msg[8] & 0xF0) << 2) | ((msg[7] & 0x3F))
		);
	bear = (int) (
		((msg[9]) << 4) | (msg[8] & 0x0F)
		);
	return true;
}

bool s_binary_message::set_msg_pvc4(
					unsigned char id, 
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/,
					unsigned char sec /* 6bit 1sec / unit */)
{
	// |0-9| DAC = 0
	// |10-15| FI = 0
	msg[0] = 0;
	msg[1] = 0;

	// |16| AckReq = 0
	// |17-27| TextSeq = 0
	msg[2] = 0;
	msg[3] = 0;

	// id -> 8bit
	msg[3] |= (id & 0x0F);
	msg[4] = (id & 0xF0);

	// sog -> 10bit
	msg[4] |= (sog & 0x000F);
	msg[5] = (sog & 0x03F0) >> 2;

	// cog -> 12bit
	msg[5] |= (cog & 0x0003);
	msg[6] = (cog & 0x03FC) >> 2;
	msg[7] = (cog & 0x0C00) >> 4;

	// lon -> 28bit
	msg[7] |= (lon & 0x0000003F);
	msg[8] = (lon & 0x00003FC0) >> 6;
	msg[9] = (lon & 0x003FC000) >> 14;
	msg[10] = (lon & 0x0FC00000) >> 20;

	// lat -> 27bit
	msg[10] |= (lat & 0x00000003);
	msg[11] = (lat & 0x000003FC) >> 2;
	msg[12] = (lat & 0x0003FC00) >> 10;
	msg[13] = (lat & 0x03FC0000) >> 18;
	msg[14] = (lat & 0x04000000) >> 19;
	
	// sec -> 6bit
	msg[14] |= (sec & 0x3F) << 1;

	len = 119;
	return true;
}

bool s_binary_message::get_msg_pvc4(
					unsigned char & id,
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					int & lon /* 28bit 1/10000 minutes / unit = 1/600000 deg /unit*/ , 
					int & lat /* 27bit 1/10000 minutes / unit = 1/600000 deg /unit*/,
					unsigned char & sec /* 6bit 1sec / unit */)
{
	id = (msg[4] & 0xF0) | (msg[3] & 0x0F);
	cout << "id received " << (int) id << endl;
	sog = (unsigned short) (
		((msg[5] & 0xFC) << 2) | ((msg[4] & 0x0F))
		);
	cog = (unsigned short) (
		((msg[7] & 0xC0) << 4) | ((msg[6]) << 2) | ((msg[5] & 0x03))
		);

	if(cog < 0 || cog > 3600)
		return false;

	lon = (int) (
		((msg[10] & 0xFC) << 20) | ((msg[9]) << 14) | ((msg[8]) << 6) | ((msg[7] & 0x3F))
		);
	lat = (int) (
		(((msg[14] & 0x80) << 19) 
		| ((msg[13]) << 18) 
		| ((msg[12]) << 10) 
		| ((msg[11]) << 2) 
		| (msg[10] & 0x03))
		);

	sec = (msg[14] & 0x7E) >> 1;

	return true;
}



bool s_binary_message::set_msg_pvc5(
					unsigned char id, /* 8bit */
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					unsigned short dist /* 10bit 0.1 mile / unit */,
					unsigned short bear /* 12bit 0.1 deg / unit */,
					unsigned char sec /* 6bit 1 sec / unit */)
{
	// |0-9| DAC = 0
	// |10-15| FI = 0
	msg[0] = 0;
	msg[1] = 0;

	// |16| AckReq = 0
	// |17-27| TextSeq = 0
	msg[2] = 0;
	msg[3] = 0;

	// id -> 8bit
	msg[3] |= (id & 0x0F);
	msg[4] = (id & 0xF0);

	// sog -> 10bit
	msg[4] |= (sog & 0x000F);
	msg[5] = (sog & 0x03F0) >> 2;

	// cog -> 12bit
	msg[5] |= (cog & 0x0003);
	msg[6] = (cog & 0x03FC) >> 2;
	msg[7] = (cog & 0x0C00) >> 4;

	// dist -> 10bit
	msg[7] |= (dist & 0x0000003F);
	msg[8] = (dist & 0x000003C0) >> 2;

	// bear -> 12bit
	msg[8] |= (bear & 0x0000000F);
	msg[9] = (bear & 0x00000FF0) >> 4;

	// sec -> 6bit
	msg[10] = (sec & 0x3F) << 2;

	len = 86;
	return true;
}

bool s_binary_message::get_msg_pvc5(
					unsigned char & id, /* 8bit */
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					unsigned short & dist /* 10bit 0.1 mile / unit */,
					unsigned short & bear /* 12bit 0.1 deg / unit */,
					unsigned char & sec /* 6bit 1 sec / unit */)
{
	id = (msg[4] & 0xF0) | (msg[3] & 0x0F);
	cout << "id received " << (int) id << endl;
	sog = (unsigned short) (
		((msg[5] & 0xFC) << 2) | ((msg[4] & 0x0F))
		);
	cog = (unsigned short) (
		((msg[7] & 0xC0) << 4) | ((msg[6]) << 2) | ((msg[5] & 0x03))
		);

	if(cog < 0 || cog > 3600)
		return false;

	dist = (int) (
		((msg[8] & 0xF0) << 2) | ((msg[7] & 0x3F))
		);
	bear = (int) (
		((msg[9]) << 4) | (msg[8] & 0x0F)
		);

	sec = (unsigned char) (msg[10]) >> 2;
	return true;
}

bool s_binary_message::set_msg_pvc(
					unsigned char id, 
					unsigned short sog /* 10bit 0.1 knot / unit */,
					unsigned short cog /* 12bit 0.1 deg / unit */,
					int lon /* 28bit 10000 minutes  = 1/600000 deg /unit*/ , 
					int lat /* 27bit 10000 minutes  = 1/600000 deg /unit*/)
{
	msg[0] = id; // 8bit

	msg[1] = sog & 0x00FF; // 8bit

	msg[2] = (sog & 0x0300) >> 8; // 2bit
	msg[2] |= (cog & 0x003F) << 2; // 6bit

	msg[3] = (cog & 0x0FC0) >> 6; // 6bit
	msg[3] |= (lon & 0x00000003) << 6; // 2bit

	msg[4] = (lon & 0x000003FC) >> 2; // 8bit

	msg[5] = (lon & 0x0003FC00) >> 10; // 8bit

	msg[6] = (lon & 0x03FC0000) >> 18; // 8bit

	msg[7] = (lon & 0x0C000000) >> 26; // 2bit
	msg[7] |= (lat & 0x0000003F) << 2; // 6bit

	msg[8] = (lat & 0x00003FC0) >> 6; //8bit

	msg[9] = (lat & 0x003FC000) >> 14; //8bit

	msg[10] = (lat & 0x07C00000) >> 22; // 5bit

	// total 85bit but 88
	len = 88;
	return true;
}

bool s_binary_message::get_msg_pvc(
					unsigned char & id,
					unsigned short & sog /* 10bit 0.1 knot / unit */,
					unsigned short & cog /* 12bit 0.1 deg / unit */,
					int & lon /* 28bit 1/10000 minutes / unit = 1/600000 deg /unit*/ , 
					int & lat /* 27bit 1/10000 minutes / unit = 1/600000 deg /unit*/)
{
	id = msg[0];
	cout << "id received " << (int) id << endl;
	sog = (unsigned short) (msg[2] & 0x03);
	sog <<= 8;
	sog |= (unsigned short) msg[1];

	cog = (unsigned short) (msg[3] & 0x3F);
	cog <<= 6;
	cog |= (unsigned short) (msg[2] & 0xFC) >> 2;
	if(cog < 0 || cog > 3600)
		return false;

	lon = (int) (msg[7] & 0x03);
	lon <<= 30;
	lon >>= 4;
	lon |= (int) msg[6] << 18;
	lon |= (int) msg[5] << 10;
	lon |= (int) msg[4] << 2;
	lon |= (int) (msg[3] & 0xC0) >> 6;

	lat = (int) (msg[10] & 0x1F);
	lat <<= 27;
	lat >>= 5;
	lat |= (int) msg[9] << 14;
	lat |= (int) msg[8] << 6;
	lat |= (int) (msg[7] & 0xFC) >> 2;
	return true;
}


bool s_binary_message::gen_nmea(const char * toker, vector<string> & nmeas)
{
	if(len > 952){
		cout << "Irregal message length in send_bbm." << endl;
		return false;
	}

	if(ch >= 4){
		cout << "Irregal channel specification in send_bbm." << endl;
		return false;
	}

	if(type != 8 && type != 14 && type != 6 && type != 12){
		cout << "Irregal message id in send_bbm." << endl;
		return false;
	}

	nmeas.clear();

	int num_sends;

	nmea[0] = '!'; nmea[1] = toker[0]; nmea[2] = toker[1];

	int bit_lim;
	// Message type "ABM," or "BBM," filled in the m_nmea buffer
	if(type == 6 || type == 12){ //ABM
		nmea[3] = 'A';
		bit_lim = 288;
		sq %= 4;
	}else{ // BBM
		nmea[3] = 'B'; 
		bit_lim = 348;
		sq %= 10;
	}
	nmea[4] = 'B'; nmea[5] = 'M'; nmea[6] = ',';

	// calculating number of sentences needed 
	if(len <= bit_lim){
		num_sends = 1;
	}else{
		num_sends = 1 + (len - bit_lim) / 360 + (((len - bit_lim) % 360) == 0 ? 0 : 1);
		num_sends = max(num_sends, 9);
	}

	nmea[7] = num_sends + '0'; nmea[8] = ',';  // Total number of sentences
	nmea[11] = sq + '0'; nmea[12] = ','; // sequential message identifier
	int ibit = 0;
	int ibuf = 0;
	int im = 0;
	for(int isend = 0; isend < num_sends; isend++){
		// first 58x6=348bit
		// subsequent 60x6=360bit
		int i;
		nmea[9] = (1 + isend) + '0'; nmea[10] = ','; // sentense number (upto num_sends)
		if(isend == 0){
			if(type == 6 || type == 12){
				sprintf(&nmea[13], "%09d", mmsi);
				nmea[22] = ',';
				nmea[23] = ch + '0'; nmea[24] = ',';
				if(type == 6){
					nmea[25] = '6';
					i = 26;
				}else{
					nmea[25] = '1'; nmea[26] = '2';
					i = 27;
				}
			}else{
				nmea[13] = ch + '0'; nmea[14] = ',';
				if(type == 8){
					nmea[15] = '8';
					i = 16;
				}else{
					nmea[15] = '1'; nmea[16] = '4';
					i = 17;
				}
			}
			nmea[i] = ',';
			i++;
		}else{
			nmea[13] = ','; nmea[14] = ',';
			if(type == 6 || type == 12){
				nmea[15] = ',';
				i = 16;
			}else
				i = 15;
		}

		// copy message
		while(1){
			unsigned char uc = 0;

			switch(im){
			case 0:
				uc = (msg[ibuf] >> 2) & 0x3F;
				ibit += 6;
				im = 1;
				break;
			case 1:
				uc = (msg[ibuf] << 4);
				ibit += 2;
				if(ibit < len){
					ibuf++;
					uc |= ((msg[ibuf] & 0xF0) >> 4); 
					ibit += 4;
				}
				im = 2;
				break;
			case 2:
				uc = (msg[ibuf] << 2);
				ibit += 4;
				if(ibit < len){
					ibuf++;
					uc |= (msg[ibuf] & 0xC0) >> 6;
					ibit += 2;
				}
				im = 3;
				break;
			case 3:
				uc = msg[ibuf];
				ibit += 6;
				ibuf++;
				im = 0;
				break;
			}

			nmea[i] = armor(uc & 0x3F);
			i++;
			if(ibit >= len || ibit >= bit_lim)
				break;
		}

		nmea[i] = ',';
		i++;
		if(isend == num_sends - 1){
			int pad = (len % 6);
			if(pad)
				pad = 6 - pad;

			nmea[i] = pad + '0';	
		}else{
			nmea[i] = '0';
		}
		i++;
		nmea[i] = '*';
		unsigned char chksum = calc_nmea_chksum(nmea);
		char c;
		i++;
		c = (chksum >> 4) & 0x0F;
		nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');
		i++;
		c = chksum & 0x0F;
		nmea[i] = (c < 10 ? c + '0' : c - 10 + 'A');

		i++;
		nmea[i] = 13; // cr
		i++;
		nmea[i] = 10; // lf
		i++;
		nmea[i] = '\0';

		nmeas.push_back(string(nmea));

		bit_lim += 360;
	}
	sq = sq + 1;

	return true;
}

///////////////////////////////////////////// navdat decoder
c_nmea_dat * c_nmea_dat::dec_nmea_dat(const char * str)
{
	if(!eval_nmea_chksum(str)){
		return NULL;
	}

	c_nmea_dat * pnd = NULL;
	if(parstrcmp(&str[3], "GGA")){
		pnd = c_gga::dec_gga(str);
	}else if(parstrcmp(&str[3], "RMC")){
		pnd = c_rmc::dec_rmc(str);
	}else if(parstrcmp(&str[3], "ZDA")){
		pnd = c_zda::dec_zda(str);
	}else if(parstrcmp(&str[3], "VDM")){
		pnd = c_vdm::dec_vdm(str);
	}else if(parstrcmp(&str[3], "VDO")){
		c_vdm * pvmd = c_vdm::dec_vdm(str);
		if(pvmd)
			pvmd->m_vdo = true;
		pnd = pvmd;
	}else if(parstrcmp(&str[3], "TTM")){
		pnd = c_ttm::dec_ttm(str);
	}else if(parstrcmp(&str[3], "ABK"))
		pnd = c_abk::dec_abk(str);

	if(pnd){
		pnd->m_toker[0] = str[1];
		pnd->m_toker[1] = str[2];
		pnd->m_cs = true;
	}

	return pnd;
}

//////////////////////////////////////////// gga decoder

c_nmea_dat * c_gga::dec_gga(const char * str)
{
	c_gga * pnd = new c_gga;
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	char tok[32];
	while(ipar < 15){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;
		if(len == 0){ 
			ipar++;
			continue;
		}

		switch(ipar){
		case 0: // $GPGGA 
			break;
		case 1: // TIME hhmmss
			parstrcpy(tok, buf, 2);
			pnd->m_h = (short) atoi(tok);
			parstrcpy(tok, buf+2, 2);
			pnd->m_m = (short) atoi(tok);
			parstrcpy(tok, buf+4, '\0');
			pnd->m_s = (float) atof(tok);
			break;
		case 2: // LAT
			parstrcpy(tok, buf, 2);
			pnd->m_lat_deg = atof(tok);
			parstrcpy(tok, buf+2, '\0');
			pnd->m_lat_deg += atof(tok) / 60;
			break;
		case 3: // N or S
			if(buf[0] == 'N')
				pnd->m_lat_dir = EGP_N;
			else
				pnd->m_lat_dir = EGP_S;
			break;				
		case 4: // LON
			parstrcpy(tok, buf, 3);
			pnd->m_lon_deg = atof(tok);
			parstrcpy(tok, buf + 3, '\0');
			pnd->m_lon_deg += atof(tok) / 60;
			break;
		case 5: // E or W
			if(buf[0] == 'E')
				pnd->m_lon_dir = EGP_E;
			else
				pnd->m_lon_dir = EGP_W;
			break;
		case 6: // Fix Stats
			switch(buf[0]){
			case '0':
				pnd->m_fix = EGPF_LOST;
				break;
			case '1':
				pnd->m_fix = EGPF_GPSF;
				break;
			case '2':
				pnd->m_fix = EGPF_DGPSF;
				break;
			}
		case 7: // NUM_SATS
			pnd->m_num_sats = atoi(buf);
			break;
		case 8: // HDOP
			pnd->m_hdop = (float) atof(buf);
			break;
		case 9: // Altitude
			pnd->m_alt = (float) atof(buf);
			break;
		case 10: // M
			break;
		case 11:// Geoidal separation
			pnd->m_geos = (float) atof(buf);
			break;
		case 12: // M
			break;
		case 13: // dgps age
			pnd->m_dgps_age = (float) atof(buf);
			break;
		case 14: // dgps station id
			if(parstrcpy(tok, buf, '*'))
				pnd->m_dgps_station = atoi(buf);
			break;
		}

		ipar++;
	}

	return pnd;
}


/////////////////////////////////////////// rmc decoder
c_nmea_dat * c_rmc::dec_rmc(const char * str)
{
	c_rmc * pnd = new c_rmc;
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	char tok[32];

	while(ipar < 12){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;
		if(len == 0){ 
			ipar++;
			continue;
		}

		switch(ipar){
		case 0: // $GPRMC
			break;
		case 1: // TIME hhmmss
			parstrcpy(tok, buf, 2);
			pnd->m_h = (short) atoi(tok);
			parstrcpy(tok, buf+2, 2);
			pnd->m_m = (short) atoi(tok);
			parstrcpy(tok, buf+4, '\0');
			pnd->m_s = (float) atof(tok);
			break;
		case 2: // Validity flag
			if(buf[0] == 'A')
				pnd->m_v = true;
			else
				pnd->m_v = false;
			break;
		case 3: // Lat
			parstrcpy(tok, buf, 2);
			pnd->m_lat_deg = (float) atof(tok);
			parstrcpy(tok, buf+2, '\0');
			pnd->m_lat_deg += atof(tok) / 60;
			break;
		case 4: // N or S
			if(buf[0] == 'N')
				pnd->m_lat_dir = EGP_N;
			else
				pnd->m_lat_dir = EGP_S;
			break;				
		case 5: // LON
			parstrcpy(tok, buf, 3);
			pnd->m_lon_deg = (float) atof(tok);
			parstrcpy(tok, buf + 3, '\0');
			pnd->m_lon_deg += atof(tok) / 60;
			break;
		case 6: // E or W
			if(buf[0] == 'E')
				pnd->m_lon_dir = EGP_E;
			else
				pnd->m_lon_dir = EGP_W;
			break;
		case 7: // Speed
			pnd->m_vel = atof(buf);
			break;
		case 8: // Course
			pnd->m_crs = atof(buf);
			break;
		case 9: // Date
			parstrcpy(tok, buf, 2);
			pnd->m_dy = atoi(tok);
			parstrcpy(tok, buf+2, 2);
			pnd->m_mn = atoi(tok);
			parstrcpy(tok, buf+4, 2);
			pnd->m_yr = atoi(tok);
			break;
		case 10: // Course Variation
			pnd->m_crs_var = atof(buf);
			break;
		case 11: // Direction of Variation
			if(buf[0] == 'E')
				pnd->m_crs_var_dir = EGP_E;
			else
				pnd->m_crs_var_dir = EGP_W;
			break;
		}
		ipar++;
	}

	return pnd;
}

////////////////////////////////////////////////zda decoder
c_nmea_dat * c_zda::dec_zda(const char * str)
{
	c_zda * pnd = new c_zda;
	int i = 0;
	int ipar = 0;
	int len;
	char buf[32];
	char tok[32];

	while(ipar < 7){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;
		if(len == 0){ 
			ipar++;
			continue;
		}

		switch(ipar){
		case 0: // $GPRMC
			break;
		case 1: // TIME hhmmss
			parstrcpy(tok, buf, 2);
			pnd->m_h = (short) atoi(tok);
			parstrcpy(tok, buf+2, 2);
			pnd->m_m = (short) atoi(tok);
			parstrcpy(tok, buf+4, '\0');
			pnd->m_s = (float) atof(tok);
			break;
		case 2: // day
			pnd->m_dy = atoi(buf);
			break;
		case 3: // month
			pnd->m_mn = atoi(buf);
			break;
		case 4: // year
			pnd->m_yr = atoi(buf);
			break;				
		case 5: // local zone hour offset
			pnd->m_lzh = atoi(buf);
			break;
		case 6: // local zone minute offset
			pnd->m_lzm = atoi(buf);
			break;
		}
		ipar++;
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

//////////////////////////////////////////////// vdm decoder
list<s_vdm_pl*> s_vdm_pl::m_tmp;
s_vdm_pl * s_vdm_pl::m_pool = NULL;

void s_vdm_pl::clear()
{
	for(list<s_vdm_pl*>::iterator itr = m_tmp.begin(); itr != m_tmp.end(); itr++)
		delete *itr;
	m_tmp.clear();
}

char armor(char c)
{
	if(c < 40)
		c += 48;
	else
		c += 56;
	return c;
}

void s_vdm_pl::dearmor(const char * str)
{
	int i;

	for(i = 0; str[i] != '\0' && m_pl_size < 256; i++, m_pl_size++){
		m_payload[m_pl_size] = str[i] - 48;
		if(m_payload[m_pl_size] > 40)
			m_payload[m_pl_size] -= 8;
	}
}

c_vdm * c_vdm::dec_vdm(const char * str)
{
	c_vdm * pnd;
	s_vdm_pl * ppl = NULL;
	int i = 0;
	int ipar = 0;
	int len;
	char buf[64];
	int fcounts = 0, fnumber = 0, seqmsgid = 0;
	while(ipar < 7){
		len = parstrcpy(buf, &str[i], ',', 64);
		i += len + 1;

		switch(ipar){
		case 0: // !--VDM
			break;
		case 1: // number of fragments
			fcounts = atoi(buf);
			break;
		case 2: // fragment number
			fnumber = atoi(buf);
			break;
		case 3: // sequential message number
			if(len != 0)
				seqmsgid = htoi(buf);
			if(fnumber == 1){
				ppl = s_vdm_pl::alloc();
				ppl->m_fcounts = fcounts;
				ppl->m_seqmsgid = seqmsgid;
			}else{
				// finding same seqmsgid
				list<s_vdm_pl*>::iterator itr = s_vdm_pl::m_tmp.begin();
				for(;itr != s_vdm_pl::m_tmp.end(); itr++)
					if((*itr)->m_seqmsgid == seqmsgid)
						break;
				if(itr == s_vdm_pl::m_tmp.end()){ // not found
					cerr << "Cannot find preceding fragment." << endl;
					return NULL;
				}
				ppl = *itr;
				s_vdm_pl::m_tmp.erase(itr);
			}
			ppl->m_fnumber = fnumber;
			break;
		case 4: // channnel A or B
			ppl->m_is_chan_A = buf[0] == 'A';
			break;
		case 5: // dearmor payload
			ppl->dearmor(buf);
			break;
		case 6: // number of padded zeros
			ppl->m_num_padded_zeros = buf[0] - '0';
		}

		ipar++;
	}

	if(!ppl->is_complete()){
		s_vdm_pl::m_tmp.push_back(ppl);
		if(s_vdm_pl::m_tmp.size() > 10){
			s_vdm_pl::free(*s_vdm_pl::m_tmp.begin());
			s_vdm_pl::m_tmp.pop_front();
#ifdef _DEBUG
			cerr << "Payload fragment exceeded 10." << endl;
#endif
		}
		return NULL;
	}

	pnd = ppl->dec_payload();
	s_vdm_pl::free(ppl);

	return pnd;
}

c_vdm * s_vdm_pl::dec_payload()
{
	m_type = (short) m_payload[0];

	c_vdm * pnd = NULL;
	switch(m_type){
	case 1: // Position report class A 
	case 2: // Position Report Class A (Assigned Schedule)
	case 3: // Position Report Class A (Response to interrogation)
		pnd = new c_vdm_msg1;
		break;
	case 4:
	case 11:
		pnd = new c_vdm_msg4;
		break;
	case 5:
		pnd = new c_vdm_msg5;
		break;
	case 8:
		pnd = new c_vdm_msg8;
		break;
	case 18:
		pnd = new c_vdm_msg18;
		break;
	case 19:
		pnd = new c_vdm_msg19;
		break;
	case 24:
		pnd = new c_vdm_msg24;
		break;
	default:
		break;
	}
	if(pnd != NULL)
		pnd->dec_payload(this);
	return pnd;
}

////////////////////////////////////// c_vdm_msg1

void c_vdm_msg1::dec_payload(s_vdm_pl * ppl)
{
	unsigned int tmpu;
	char tmpc;
	int tmpi;
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl);

	m_status = (dat[6] & 0x0F);

	tmpc = ((dat[7] & 0x3F) << 2) | 
		((dat[8] & 0x30) >> 4);
	m_turn = (float) (tmpc * (1.0 / 4.733));
	m_turn *= (tmpc < 0 ? -m_turn : m_turn);

	tmpu = ((dat[8] & 0x0F) << 6) |
		(dat[9] & 0x3F);
	m_speed = (float)((float) tmpu * (1.0/10.0));

	m_accuracy = ((dat[10] & 0x20) >> 7);

	tmpi = ((dat[10] & 0x1F) << 27) |
		((dat[11] & 0x3F) << 21) |
		((dat[12] & 0x3F) << 15) |
		((dat[13] & 0x3F) << 9) |
		((dat[14] & 0x3E) << 3);
	tmpi >>= 4;
	tmpi = (tmpi & 0x08000000 ? tmpi | 0xF0000000 : tmpi);
	m_lon = (float) (tmpi * (1.0/600000.0));

	tmpi = ((dat[14] & 0x01) << 31) |
		((dat[15] & 0x3F) << 25) | 
		((dat[16] & 0x3F) << 19) |
		((dat[17] & 0x3F) << 13) |
		((dat[18] & 0x3F) << 7) |
		((dat[19] & 0x30) << 1);
	tmpi >>= 5;
	tmpi = (tmpi & 0x04000000 ? tmpi | 0xF8000000 : tmpi);
	m_lat = (float) (tmpi * (1.0/600000.0));

	tmpu = ((dat[19] & 0x0F) << 8) |
		((dat[20] & 0x3F) << 2) |
		((dat[21] & 0x30) >> 4);
	
	m_course = (float) (tmpu * (1.0/10.0));
	m_heading = ((dat[21] & 0x0F) << 5) |
		((dat[22] & 0x3E) >> 1);

	m_second =  ((dat[22] & 0x01) << 1) |
		((dat[23] & 0x3E) >> 1);

	m_maneuver = ((dat[23] & 0x01) << 1) |
		((dat[24] & 0x20) >> 5);

	// dat[24] & 0x0C is not used

	m_raim = (dat[24] & 0x02) >> 1;

	m_radio = ((dat[24] & 0x01) << 18) |
		((dat[25] & 0x3F) << 12) |
		((dat[26] & 0x3F) << 6) |
		(dat[27] & 0x3F);

}


ostream & c_vdm_msg1::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 1 << ">";
	out<< "MMSI:" << m_mmsi;
	out<< " STAT:";
	switch(m_status){
	case 0:
		out<< "Under way using engine";
		break;
	case 1:
		out<< "At anchor";
		break;
	case 2:
		out<< "Not under command";
		break;
	case 3:
		out<< "Restricted manoeuverability";
		break;
	case 4:
		out<< "Constrained by her draught";
		break;
	case 5:
		out<< "Moored";
		break;
	case 6:
		out<< "Aground";
		break;
	case 7:
		out<< "Engaged in Fishing";
		break;
	case 8:
		out<< "Under way sailing";
		break;
	default:
		out<< "Unknown state";
	}

	out<< " TRN:" << m_turn << "(deg/min)";
	out<< " SPD:" << m_speed << "(knot)";
	out<< " DGPS:" << (m_accuracy?"yes":"no");
	out<< " LON:" << m_lon << "(deg)";
	out<< " LAT:" << m_lat << "(deg)";
	out<< " CRS:" << m_course << "(deg)";
	out<< " HDG:" << m_heading << "(deg)";
	out<< " UTC:" << (int) m_second << "(sec)";
	out<< " MAN:" << (m_maneuver? (m_maneuver == 1 ? "no special":"special"):"na");
	out<< " RAIM:" << (m_raim?"yes":"no");
	out << endl;

	return out;
}


////////////////////////////////////// c_vdm_msg4

void c_vdm_msg4::dec_payload(s_vdm_pl * ppl)
{
	int tmpi;
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl);

	m_year = ((dat[6] & 0x0F) << 10) |
		((dat[7] & 0x3F) << 4) | 
		((dat[8] & 0x3C) >> 2);

	m_month = ((dat[8] & 0x03) << 2) |
		((dat[9] & 0x30) >> 4);

	m_day = ((dat[9] & 0x0F) << 1) |
		((dat[10] & 0x20) >> 5);

	m_hour = (dat[10] & 0x1F);

	m_minute = dat[11];
	m_second = dat[12];
	m_accuracy = ((dat[13] & 0x20) ? true : false);
	
	tmpi = ((dat[13] & 0x1F) << 27) |
		((dat[14] & 0x3F) << 21) |
		((dat[15] & 0x3F) << 15) |
		((dat[16] & 0x3F) << 9) |
		((dat[17] & 0x3E) << 3);
	tmpi >>= 4;
	tmpi = (tmpi & 0x08000000 ? tmpi | 0xF0000000 : tmpi);
	m_lon = (float) (tmpi * (1.0/600000.0));

	tmpi = ((dat[17] & 0x01) << 31) |
		((dat[18] & 0x3F) << 25) | 
		((dat[19] & 0x3F) << 19) |
		((dat[20] & 0x3F) << 13) |
		((dat[21] & 0x3F) << 7) |
		((dat[22] & 0x30) << 1);
	tmpi >>= 5;
	tmpi = (tmpi & 0x04000000 ? tmpi | 0xF8000000 : tmpi);
	m_lat = (float) (tmpi * (1.0/600000.0));

	m_epfd = dat[22] & 0x0F;

	m_raim = (dat[24] & 0x02) >> 1;
	m_radio = ((dat[24] & 0x01) << 18) |
		((dat[25] & 0x3F) << 12)|
		((dat[26] & 0x3F) << 6) |
		(dat[27] & 0x3F);
}


ostream & c_vdm_msg4::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 4 << ">";
	out<< "MMSI:" << m_mmsi;
	out<< " DGPS:" << (m_accuracy?"yes":"no");
	out<< " LON:" << m_lon << "(deg)";
	out<< " LAT:" << m_lat << "(deg)";
	out<< " UTC:" << (int) m_day << "," << (int) m_month 
		<< "," << m_year << "," << (int) m_hour 
		<< ":" << (int) m_minute <<":" << (int) m_second << ":";
	out << " EPFD:";
	switch(m_epfd){
	case 0:
		out << "Undefined";
		break;
	case 1:
		out << "GPS";
		break;
	case 2:
		out << "GLONASS";
		break;
	case 3:
		out << "Combined GPS/GLONASS";
		break;
	case 4:
		out << "Loran-C";
		break;
	case 5:
		out << "Chayka";
		break;
	case 6:
		out << "Integrated navigation system";
		break;
	case 7:
		out << "Surveyed";
		break;
	case 8:
		out << "Galileo";
	}

	out<< " RAIM:" << (m_raim?"yes":"no");
	out << endl;

	return out;
}


////////////////////////////////////// c_vdm_msg5

void c_vdm_msg5::dec_payload(s_vdm_pl * ppl)
{
	unsigned char tmpu;
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl);

	m_ais_version = ((dat[6] & 0x0C) >> 2);

	m_imo = ((dat[6] & 0x03) << 28) |
		((dat[7] & 0x3F) << 22) | 
		((dat[8] & 0x3F) << 16) |
		((dat[9] & 0x3F) << 10) |
		((dat[10] & 0x3F) << 4) |
		((dat[11] & 0x3C) >> 2);

	//consume 7 bytes
	for(int i = 0, byte=11; i < 7; i++, byte++){
		m_callsign[i] = decchar(((dat[byte] & 0x03) << 4) |
			((dat[byte+1] & 0x3C) >> 2)); 
		if(m_callsign[i] == '@')
			m_callsign[i] = '\0';
	}
	m_callsign[7] = 0;

	// consume 20 bytes
	for(int i = 0, byte=18; i < 20; i++, byte++){
		m_shipname[i] = decchar(((dat[byte] & 0x03) << 4) |
			((dat[byte+1] & 0x3C) >> 2)); 
		if(m_shipname[i] == '@')
			m_shipname[i] = '\0';
	}
	m_shipname[20] = '\0';

	// 38
	m_shiptype = ((dat[38] & 0x03) << 6) |
		(dat[39] & 0x3F);

	m_to_bow = (dat[40] << 3) |
		((dat[41] & 0x38) >> 3);

	m_to_stern = ((dat[41] & 0x07) << 6) |
		dat[42];
	m_to_port = dat[43];
	m_to_starboard = dat[44];

	m_epfd = (dat[45] & 0x3C) >> 2;

	m_month = ((dat[45] & 0x03) << 2) |
		((dat[46] & 0x30) >> 2);

	m_day = ((dat[46] & 0x0F) << 1) |
		((dat[47] & 0x20) >> 5);

	m_hour = (dat[47] & 0x1F); 
	m_minute = dat[48];

	tmpu = (dat[49] << 2) |
		((dat[50] & 0x30) >> 4);
	m_draught = (float) (tmpu * (1.0/10));

	for(int i = 0, byte = 50; i < 20; i++, byte++){
		m_destination[i] = decchar(((dat[byte] & 0x0F) << 2) |
			((dat[byte] & 0x30) >> 4));
	}
	m_destination[20] = 0;

	m_dte =((dat[70] & 0x08) ? false:true);
}

ostream & c_vdm_msg5::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 5 << ">";
	out<< "MMSI:" << m_mmsi;

	out << " EPFD:";
	switch(m_epfd){
	case 0:
		out << "Undefined";
		break;
	case 1:
		out << "GPS";
		break;
	case 2:
		out << "GLONASS";
		break;
	case 3:
		out << "Combined GPS/GLONASS";
		break;
	case 4:
		out << "Loran-C";
		break;
	case 5:
		out << "Chayka";
		break;
	case 6:
		out << "Integrated navigation system";
		break;
	case 7:
		out << "Surveyed";
		break;
	case 8:
		out << "Galileo";
	}

	out << " VER:" << (int) m_ais_version;
	out << " IMO:" << m_imo;
	out << " CLSGN:" << m_callsign;
	out << " VSLNM:" << m_shipname;
	out << " TYPE:";

	out << get_ship_type_name(m_shiptype);

	out << " DIM(bow, stern, port, starboard):" << m_to_bow << ","  
		<< m_to_stern << "," << (int) m_to_port << "," << (int) m_to_starboard;
	out<< " ETA:" << (int) m_day << "," << (int) m_month 
		<< "," << (int) m_hour << ":" << (int) m_minute;
	out << " DRT:" << m_draught;
	out << " DTE:" << (m_dte ? "rdy" : "not rdy");
	out << endl;

	return out;
}
////////////////////////////////////// c_vdm_msg6
void c_vdm_msg6::dec_payload(s_vdm_pl * ppl)
{
	char * dat = ppl->m_payload;
	c_vdm::dec_payload(ppl); // 0-37 bit consumed
	// 40-69 destination MMSI
	m_mmsi_dst = ((dat[6] & 0x03) << 28) |
		((dat[7] & 0x3F) << 22) |
		((dat[8] & 0x3F) << 16) |
		((dat[9] & 0x3F) << 10) |
		((dat[10] & 0x3F) << 4) |
		((dat[11] & 0x3C) >> 2);

	m_dac = ((dat[12] & 0x3F) << 4) |
		((dat[13] & 0x3C) >> 2);
	m_fid = ((dat[13] & 0x03) << 4) |
		((dat[14] & 0x3C) >> 2);

	// 40 to end
	for(int i = 0, byte=6;;){
		m_msg.msg[i] =  ((dat[byte] & 0x03) << 6); byte++;
		if(byte >= ppl->m_pl_size)
			break;
		m_msg.msg[i] |= (dat[byte] & 0x3F); byte++;

		if(byte >= ppl->m_pl_size)
			break;
		i++; 
		m_msg.msg[i] = ((dat[byte] & 0x3F) << 2); byte++;
		if(byte >= ppl->m_pl_size)
			break;

		m_msg.msg[i] |= ((dat[byte] & 0x30) >> 4);

		i++;
		m_msg.msg[i] = ((dat[byte] & 0x0F) << 4); byte++;
		if(byte >= ppl->m_pl_size)
			break;

		m_msg.msg[i] |= ((dat[byte] & 0x3C) >> 2);
		i++;
	}
}

ostream & c_vdm_msg6::show(ostream & out)
{
	return out;
}

////////////////////////////////////// c_vdm_msg8
void c_vdm_msg8::dec_payload(s_vdm_pl * ppl)
{
	char * dat = ppl->m_payload;

	c_vdm::dec_payload(ppl); // 0-37 bit consumed

	m_dac = ((dat[6] & 0x03) << 8) |
		((dat[7] & 0x3F) << 2) |
		((dat[8] & 0x30) >> 4);
	m_fid = ((dat[8] & 0x0F) << 2) |
		((dat[9] & 0x30) >> 4);

	m_msg_size = ppl->m_pl_size * 6 /* 6bit per char */
		- 40 - ppl->m_num_padded_zeros;

	// 40 to end
	for(int i = 0, byte=6;;){
		m_msg.msg[i] =  ((dat[byte] & 0x03) << 6); byte++;
		if(byte >= ppl->m_pl_size)
			break;
		m_msg.msg[i] |= (dat[byte] & 0x3F); byte++;

		if(byte >= ppl->m_pl_size)
			break;
		i++; 
		m_msg.msg[i] = ((dat[byte] & 0x3F) << 2); byte++;
		if(byte >= ppl->m_pl_size)
			break;

		m_msg.msg[i] |= ((dat[byte] & 0x30) >> 4);

		i++;
		m_msg.msg[i] = ((dat[byte] & 0x0F) << 4); byte++;
		if(byte >= ppl->m_pl_size)
			break;

		m_msg.msg[i] |= ((dat[byte] & 0x3C) >> 2);
		i++;
	}
}

ostream & c_vdm_msg8::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 8 << ">";
	out<< "MMSI:" << m_mmsi;

	out << " DAC: " << m_dac;
	out << " FID: " << m_fid; 
	
	out << " Message length:" << m_msg_size << endl;
	out << "MSG(as 6bit ascii):";
	int ibyte = 0;
	int num_bytes = m_msg_size / 8 + (m_msg_size % 8 ? 1 : 0);
	while(1){
		unsigned char c;
		if(ibyte >= num_bytes)
			break;
		c = (unsigned char) (0xFC & m_msg.msg[ibyte]) >> 2;
		out.put(decchar(c));

		c = (unsigned char) (0x03 & m_msg.msg[ibyte]) << 4;
		ibyte++;
		if(ibyte >= num_bytes)
			break;
		c |= (unsigned char) (0xF0 & m_msg.msg[ibyte]) >> 4;
		out.put(decchar(c));

		c = (unsigned char) (0x0F & m_msg.msg[ibyte]) << 2;
		ibyte++;
		if(ibyte >= num_bytes)
			break;
		c |= (unsigned char) (0xC0 & m_msg.msg[ibyte]) >> 6;
		out.put(decchar(c));

		c = (unsigned char) (0x3F & m_msg.msg[ibyte]);
		out.put(decchar(c));
		ibyte++;
	}
	out << endl;

	out << "MSG(as 8bit ascii):" ;
	m_msg.msg[num_bytes] = '\0';
	out << m_msg.msg << endl;

	out << "MSG(in Hex):";
	for(int i = 0; i < num_bytes; i++){
		//upper byte
		int h = (m_msg.msg[i] >> 4) & 0x0F;
		if(h < 10){
			out.put(h + '0');
		}else{
			out.put(h - 10 + 'A');
		}
		// lower byte
		h = m_msg.msg[i] & 0x0F;
		if(h < 10){
			out.put(h + '0');
		}else{
			out.put(h - 10 + 'A');
		}

		out << " ";
	}
	out << endl;

	out << "MSG(in Bin):";
	for(int i = 0; i < num_bytes; i++){
		for(unsigned char mask = 0x80; mask != 0; mask >>= 1){
			out << (mask & m_msg.msg[i] ? 1:0);
		}
	}
	out << endl;
	return out;
}

////////////////////////////////////// c_vdm_msg18

void c_vdm_msg18::dec_payload(s_vdm_pl * ppl)
{
	unsigned int tmpu;
	int tmpi;
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl); // 0-37 bit consumed

	// 38-45 bit reserved

	// 46-55 speed over ground x10
	tmpu = ((dat[7] & 0x03) << 8) |
		(dat[8] << 2) |
		((dat[9] & 0x30) >> 4);

	m_speed = (float)((float) tmpu * (1.0/10.0));

	// 56 gps accuracy
	m_accuracy = ((dat[9] & 0x08) >> 3);

	// 57-84 lon
	tmpi = ((dat[9] & 0x07) << 25) |
		((dat[10] & 0x3F) << 19) |
		((dat[11] & 0x3F) << 13) |
		((dat[12] & 0x3F) << 7) |
		((dat[13] & 0x3F) << 1) |
		((dat[14] & 0x20) >> 5);
	tmpi = (tmpi & 0x08000000 ? tmpi | 0xF0000000 : tmpi);
	m_lon = (float) (tmpi * (1.0/600000.0));

	// 85-111 lat
	tmpi = ((dat[14] & 0x1F) << 22) |
		((dat[15] & 0x3F) << 16) |
		((dat[16] & 0x3F) << 10) |
		((dat[17] & 0x3F) << 4) |
		((dat[18] & 0x3C) >> 2);
	tmpi = (tmpi & 0x04000000 ? tmpi | 0xF8000000 : tmpi);
	m_lat = (float) (tmpi * (1.0/600000.0));

	// 112-123
	tmpu = ((dat[18] & 0x03) << 10) |
		((dat[19] & 0x3F) << 4) |
		((dat[20] & 0x3C) >> 2);
	m_course = (float)((float) tmpu * (1.0/10.0));

	// 124-132
	tmpu = ((dat[20] & 0x03) << 7) |
		((dat[21] & 0x3F) << 1) |
		((dat[22] & 0x20) >> 5);
	m_heading = tmpu;

	m_second = ((dat[22] & 0x1F) << 1) |
		((dat[23] & 0x20) >> 5);

	//139-140 reserved

	//141
	m_cs = (dat[23] & 0x04 ? true : false);

	//142
	m_disp = (dat[23] & 0x02 ? true : false);

	//143
	m_dsc = (dat[23] & 0x01 ? true : false);

	// 144
	m_band = (dat[24] & 0x20 ? true : false);

	// 145
	m_msg22 = (dat[24] & 0x10 ? true : false);

	//146
	m_assigned = (dat[24] & 0x08 ? true : false);

	// 147
	m_raim = (dat[24] & 0x04 ? true : false);
}

ostream & c_vdm_msg18::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 18 << ">";
	out<< "MMSI:" << m_mmsi;
	out<< " SPD:" << m_speed << "(knot)";
	out<< " DGPS:" << (m_accuracy?"yes":"no");
	out<< " LON:" << m_lon << "(deg)";
	out<< " LAT:" << m_lat << "(deg)";
	out<< " CRS:" << m_course << "(deg)";
	out<< " HDG:" << m_heading << "(deg)";
	out<< " UTC:" << (int) m_second << "(sec)";
	out << " CS:" << (m_cs ? "yes":"no");
	out << " DISP:" << (m_disp ? "yes":"no");
	out << " DSC:" << (m_dsc ? "yes":"no");
	out << " BAND:" << (m_band ? "yes":"no");
	out << " MSG22:" << (m_msg22 ? "yes":"no");
	out << " ASSIGNED:" << (m_assigned ? "yes":"no");
	out<< " RAIM:" << (m_raim ?"yes":"no");
	out << endl;

	return out;
}

////////////////////////////////////// c_vdm_msg19

void c_vdm_msg19::dec_payload(s_vdm_pl * ppl)
{
	unsigned int tmpu;
	int tmpi;
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl); // 0-37 bit consumed

	// 38-45 bit reserved

	// 46-55 speed over ground x10
	tmpu = ((dat[7] & 0x03) << 8) |
		(dat[8] << 2) |
		((dat[9] & 0x30) >> 4);

	m_speed = (float)((float) tmpu * (1.0/10.0));

	// 56 gps accuracy
	m_accuracy = ((dat[9] & 0x08) >> 3);

	// 57-84 lon
	tmpi = ((dat[9] & 0x07) << 25) |
		((dat[10] & 0x3F) << 19) |
		((dat[11] & 0x3F) << 13) |
		((dat[12] & 0x3F) << 7) |
		((dat[13] & 0x3F) << 1) |
		((dat[14] & 0x20) >> 5);
	tmpi = (tmpi & 0x08000000 ? tmpi | 0xF0000000 : tmpi);

	m_lon = (float) (tmpi * (1.0/600000.0));

	// 85-111 lat
	tmpi = ((dat[14] & 0x1F) << 22) |
		((dat[15] & 0x3F) << 16) |
		((dat[16] & 0x3F) << 10) |
		((dat[17] & 0x3F) << 4) |
		((dat[18] & 0x3C) >> 2);
	tmpi = (tmpi & 0x04000000 ? tmpi | 0xF8000000 : tmpi);
	m_lat = (float) (tmpi * (1.0/600000.0));

	// 112-123
	tmpu = ((dat[18] & 0x03) << 10) |
		((dat[19] & 0x3F) << 4) |
		((dat[20] & 0x3C) >> 2);
	m_course = (float)((float) tmpu * (1.0/10.0));

	// 124-132
	tmpu = ((dat[20] & 0x03) << 7) |
		((dat[21] & 0x3F) << 1) |
		((dat[22] & 0x20) >> 5);
	m_heading = tmpu;

	m_second = ((dat[22] & 0x1F) << 1) |
		((dat[23] & 0x20) >> 5);

	//139-142 reserved

	//143-262
	for(int i = 0, byte = 23; i < 20; i++, byte++){
		m_shipname[i] = decchar(((dat[byte] & 0x01) << 5) |
			((dat[byte+1] & 0x3E) >> 1));
		if(m_shipname[i] == '@')
			m_shipname[i] = '\0';
	}
	m_shipname[20] = '\0';

	// 263-270
	m_shiptype = ((dat[43] & 0x01) << 7) |
		((dat[44] & 0x3F) << 1) |
		((dat[45] & 0x20) >> 5);

	// 271-279
	m_to_bow = ((dat[45] & 0x1F) << 4) |
		((dat[46] & 0x3C) >> 2);

	// 280-288
	m_to_stern = ((dat[46] & 0x03) << 7) |
		((dat[47] & 0x3F) << 1) |
		((dat[48] & 0x20) >> 5);
	
	// 289-294
	m_to_port = ((dat[48] & 0x1F) << 1) | 
		((dat[49] & 0x20) >> 5);

	// 295-300
	m_to_starboard = ((dat[49] & 0x1F) << 1) |
		((dat[50] & 0x20) >> 5);

	// 301-304
	m_epfd = (dat[50] & 0x1E);

	// 305
	m_raim = (dat[50] & 0x01 ? true : false);

	// 306 
	m_dte = (dat[51] & 0x20 ? true : false);

	// 307
	m_assigned = (dat[51] & 0x10 ? true : false);
}


ostream & c_vdm_msg19::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 19 << ">";
	out<< "MMSI:" << m_mmsi;
	out<< " SPD:" << m_speed << "(knot)";
	out<< " DGPS:" << (m_accuracy?"yes":"no");
	out<< " LON:" << m_lon << "(deg)";
	out<< " LAT:" << m_lat << "(deg)";
	out<< " CRS:" << m_course << "(deg)";
	out<< " HDG:" << m_heading << "(deg)";
	out<< " UTC:" << (int) m_second << "(sec)";
	out<< " RAIM:" << (m_raim ? "yes":"no");
	out << " VSLNM:" << m_shipname;
	out << " TYPE:";
	out << get_ship_type_name(m_shiptype);
	out << " DIM(bow, stern, port, starboard):" << m_to_bow << ","  
		<< m_to_stern << "," << (int) m_to_port << "," << (int) m_to_starboard;
	out << endl;

	return out;
}

////////////////////////////////////// c_vdm_msg24
void c_vdm_msg24::dec_payload(s_vdm_pl * ppl)
{
	char * dat = ppl->m_payload;

	// decodes mmsi and repeat flag
	c_vdm::dec_payload(ppl); // 0-37 bit consumed

	// 38-39
	m_part_no = ((dat[6] & 0x0C) >> 2);

	if(m_part_no == 0){
		for(int i = 0, byte = 6; i < 20; i++, byte++){
			m_shipname[i] = decchar(((dat[byte] & 0x03) << 4) |
				((dat[byte+1] & 0x3C) >> 2));
			if(m_shipname[i] == '@')
				m_shipname[i] = '\0';
		}
		m_shipname[20] = '\0';
	}else{
		m_shiptype = ((dat[6] & 0x02) << 6) |
			(dat[7] & 0x3F);

		for(int i = 0, byte = 8; i < 7; i++, byte++){
			m_vendorid[i] = decchar(dat[byte]);
			if(m_vendorid[i] == '@')
				m_vendorid[i] = '\0';
		}
		m_vendorid[7] = '\0';

		for(int i = 0, byte = 15; i < 7; i++, byte++){
			m_callsign[i] = decchar(dat[byte]);
			if(m_callsign[i] == '@')
				m_callsign[i] = '\0';
		}
		m_callsign[7] = '\0';

		m_to_bow = ((dat[22] & 0x3F) << 3) | 
			((dat[23] & 0x38) >> 3);

		m_to_stern = ((dat[23] & 0x07) << 6) |
			dat[24];

		m_to_port = dat[25];

		m_to_starboard = dat[26];
		m_ms_mmsi = ((dat[27] & 0x3F) << 24) | 
			((dat[28] & 0x3F) << 18) | 
			((dat[29] & 0x3F) << 12) | 
			((dat[30] & 0x3F) << 6) |
			dat[31];
	}
}

ostream & c_vdm_msg24::show(ostream & out)
{
	out << "AIS " << (m_is_chan_A ? "A":"B") 
		<< " MSG" << 24 << ">";
	out<< "MMSI:" << m_mmsi;
	if(m_part_no == 0){
		out << " PART:A";
		out << " VSLNM:" << m_shipname;
		out << endl;
	}else{
		out << " PART:B";
		out << " TYPE:";
		out << get_ship_type_name(m_shiptype);
		out << " DIM(bow, stern, port, starboard):" << m_to_bow << ","  
			<< m_to_stern << "," << (int) m_to_port << "," << (int) m_to_starboard;
		out << " MSMMSI:" << m_ms_mmsi;
		out << endl;
	}

	return out;
}

//////////////////////////////////////////// abk decoder
c_nmea_dat * c_abk::dec_abk(const char * str)
{
	c_abk * pnd = new c_abk;
	int i = 0;
	int ipar = 0;
	int len ;
	char buf[32];

	while(ipar < 15){
		len = parstrcpy(buf, &str[i], ',');
		i += len + 1;
		if(len == 0){
			ipar++;
			continue ;
		}

		switch(ipar){
		case 0: // ABK
			break;
		case 1: // channel
			if(buf[0] == 'A')
				pnd->m_rch = c_abk::CHA;
			else if(buf[0] == 'B')
				pnd->m_rch = c_abk::CHB;
			break;
		case 2: // mmsi
			pnd->m_mmsi = atoi(buf);
			break;
		case 3: // id
			pnd->m_msg_id  = atoi(buf);
			break;
		case 4:
			pnd->m_seq = atoi(buf);
			break;
		case 5: // stat
			pnd->m_stat = atoi(buf);
			break;
		}
		ipar++;
	}
	return pnd;
}

ostream & c_abk::show(ostream & out)
{
	out << "ABK " << " CH:" << (m_rch == CHA ? "A": (m_rch == CHB ? "B" : "NA")) 
		<< " MSG" << m_msg_id << " MMSI:" << m_mmsi << " SEQ:" << m_seq << " STAT: ";
	switch(m_stat){
	case 0:
		out << " ABM successfully received by destination." << endl;
		break;
	case 1:
		out << " ABM successfully broadcasted, but no acknowledgement." << endl;
		break;
	case 2:
		out << " Message could not be broadcasted." << endl;
		break;
	case 3:
		out << " BBM successfully completed." << endl;
		break;
	case 4:
		out << " Late reception of acknoledgement." << endl;
	}
	return out;
}

//////////////////////////////////////////////// string handler

unsigned char decchar(unsigned char c6)
{
	if(c6 < ' ')
		return c6 + '@';
	
	return c6;
}

unsigned char encchar(unsigned char c8)
{
	if(c8 < '`' && c8 >= '@'){
		return c8 - '@';
	}else if(c8 >= ' '){
		return c8;
	}
	return 0;
}

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


const char * get_ship_type_name(unsigned char uc)
{
	switch(uc){
	case 0:
		return "null";
	case 20:
	case 21:
	case 22:
	case 23:
	case 24:
	case 25:
	case 26:
	case 27:
	case 28:
	case 29:
		return "WIG";
	case 30:
		return "Fishing";
	case 31:
		return "Towing";
	case 32:
		return "Towing (L>200m or W > 25m)";
	case 33:
		return "Dredging or underwater ops";
	case 34:
		return "Diving ops";
	case 35:
		return "Military ops";
	case 36:
		return "Sailing";
	case 37:
		return "Pleasure Craft";
	case 40:
		return "HSC";
	case 41:
		return "HSC A";
	case 42:
		return "HSC B";
	case 43:
		return "HSC C";
	case 44:
		return "HSC D";
	case 50:
		return "Pilot Vessel";	
	case 51:
		return "SAR Vessel";
	case 52:
		return "Tug";
	case 53:
		return "Port Tender";
	case 54:
		return "Anti-pollution equipment";
	case 55:
		return "Law Enforcement";
	case 56:
	case 57:
		return "Spare -Local Vessel";
	case 58:
		return "Medical Transport";
	case 59:
		return "Noncombatant ship according to RR Resolution No.18";
	case 60:
		return "Passenger";
	case 61:
		return "Passenger A";
	case 62:
		return "Passenger B";
	case 63:
		return "Passenger C";
	case 64:
		return "Passenger D";
	case 70:
		return "Cargo";
	case 71:
		return "Cargo A";
	case 72:
		return "Cargo B";
	case 73:
		return "Cargo C";
	case 74:
		return "Cargo D";
	case 80:
		return "Tanker";
	case 81:
		return "Tanker A";
	case 82:
		return "Tanker B";
	case 83:
		return "Tanker C";
	case 84:
		return "Tanker D";
	default:
		return "null";
	}

	return NULL;
}
