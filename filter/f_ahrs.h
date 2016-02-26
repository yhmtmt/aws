// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_ahrs.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ahrs.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ahrs.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AHRS_H_
#define _F_AHRS_H_

#include "../util/aws_serial.h"
#include "f_base.h"

class f_ahrs: public f_base
{
protected:
	AWS_SERIAL m_hserial;
	char m_wbuf[1024];
	char m_rbuf[1024];
	int m_rbuf_head;
	int m_rbuf_tail;
	char m_tbuf[1024];
	int m_tbuf_tail;

	char m_dname[1024];
	unsigned short m_port;
	unsigned int m_br;
	enum e_razor_cmd{
		ERC_O0, ERC_O1, 
		ERC_OB, ERC_OT, ERC_OC, ERC_ON, 
		ERC_OSCT, ERC_OSRT, ERC_OSBT, 
		ERC_OSCB, ERC_OSRB, ERC_OSBB, 
		ERC_F, ERC_S, ERC_UNDEF
	} m_cmd, m_omode;
	bool m_ocont;
	bool m_sync;
	
	static const char * m_str_razor_cmd[ERC_UNDEF];

	struct s_ahrs_ypr{
		float y, p, r;
	} m_ypr;
	struct s_ahrs9{
		float ax, ay, az;
		float mx, my, mz;
		float gx, gy, gz;
	} m_raw, m_cal;

	int set_buf_cmd(e_razor_cmd cmd)
	{
		if(m_cmd == ERC_UNDEF){
			return 0;
		}
		int len = 3;
		const char * str = m_str_razor_cmd[cmd];
		char * p = m_wbuf;
		*p = '#';p++;

		for(;*str!='\0'; str++, p++, len++){
			*p = *str;
		}
		*p = 0x0d;p++;
		*p = 0x0a;p++;
		*p = '\0';
		return len;
	}
	
	int seek_txt_buf();

	int set_bin_ypr()
	{
		int sz = sizeof(s_ahrs_ypr);
		int rblen = m_rbuf_tail - m_rbuf_head;
		if(rblen < sz)
			return 0;

		int res = rblen % sz;
		const char * ptr = m_rbuf + rblen - res - sz + m_rbuf_head;
		memcpy((void*) &m_ypr, (void*) ptr, (size_t) sz);
		ptr += sz;
		memcpy((void*) m_rbuf, (void*) ptr, (size_t) res);
		m_rbuf_tail = res;
		return sz;
	}

	int set_bin_9(s_ahrs9 * p9)
	{
		int sz = sizeof(s_ahrs9);
		int rblen = m_rbuf_tail - m_rbuf_head;
		if(rblen < sz)
			return 0;

		int res = rblen % sz;
		const char * ptr = m_rbuf + rblen - res - sz + m_rbuf_head;
		memcpy((void*) p9, (void*) ptr, (size_t) sz);
		ptr += sz;
		memcpy((void*) m_rbuf, (void*) ptr, (size_t) res);
		m_rbuf_tail = res;
		return sz;
	}

	int set_bin_9x2(s_ahrs9 * p91, s_ahrs9 *p92)
	{
		int sz = sizeof(s_ahrs9) * 2;
		int rblen = m_rbuf_tail - m_rbuf_head;
		if(rblen < sz)
			return 0;

		int res = rblen % sz;
		const char * ptr = m_rbuf + rblen - res - sz + m_rbuf_head;
		memcpy((void*) p91, (void*) ptr, sizeof(s_ahrs9));
		ptr += sizeof(s_ahrs9);
		memcpy((void*) p92, (void*) ptr, sizeof(s_ahrs9));
		ptr += sizeof(s_ahrs9);
		memcpy((void*) m_rbuf, (void*) ptr, (size_t) res);
		m_rbuf_tail = res;
		return sz;
	}

	void dec_tok_val()
	{
		float *px, *py, *pz;

		if(m_tbuf[1] == 'Y'){ // YPR
			px = &m_ypr.y; py = &m_ypr.p; pz = &m_ypr.r;
		}else{
			s_ahrs9 * p9;
			if(m_tbuf[3] == 'R'){ // raw data
				p9 = &m_raw;
			}else{ // calibrated data
				p9 = &m_cal;
			}
			switch(m_tbuf[1]){
			case 'A':
				px = &p9->ax; py = &p9->ay; pz = &p9->az;
				break;
			case 'M':
				px = &p9->mx; py = &p9->my; pz = &p9->mz;
				break;
			case 'G':
				px = &p9->gx; py = &p9->gy; pz = &p9->gz;
				break;
			}
		}

		char * strx, * stry, * strz,  * ptr;
		ptr = m_tbuf + 5;

		strx = ptr;
		for(;*ptr != ','; ptr++);
		*ptr = '\0'; ptr++;

		stry = ptr;
		for(;*ptr != ','; ptr++);
		*ptr = '\0'; ptr++;

		strz = ptr;
		for(;*ptr != 0x0d; ptr++);
		*ptr = '\0'; ptr++;

		*px = (float) atof(strx);
		*py = (float) atof(stry);
		*pz = (float) atof(strz);
	}
public:
	f_ahrs(const char * name);
	virtual ~f_ahrs();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
