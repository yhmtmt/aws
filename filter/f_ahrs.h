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

#include "f_base.h"
#include "../channel/ch_state.h"


#define AHRS_BUF 1024
#define GYRO_GAIN 0.06957

class f_ahrs: public f_base
{
protected:
  ch_state * m_state;

  AWS_SERIAL m_hserial;
  char m_wbuf[AHRS_BUF];
  char m_rbuf[AHRS_BUF];
  int m_rbuf_head;
  int m_rbuf_tail;
  char m_tbuf[AHRS_BUF];
  int m_tbuf_tail;
  int m_readlen;
  char m_dname[1024];
  unsigned short m_port;
  unsigned int m_br;
  enum e_razor_cmd{
    ERC_O0, ERC_O1, 
    ERC_OB, ERC_OT, ERC_OC, ERC_ON, 
    ERC_OSCT, ERC_OSRT, ERC_OSBT, 
    ERC_OSCB, ERC_OSRB, ERC_OSBB,
	ERC_OFT, ERC_OFB, 
    ERC_F, ERC_S, ERC_UNDEF
  } m_cmd, m_omode;
  bool m_ocont;
  bool m_sync;
  bool m_verb;
  
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
		for(int i = 0; i < res; i++, ptr++)
			m_rbuf[i] = *ptr;
		m_rbuf_head = 0;
		m_rbuf_tail = res;
		return sz;
	}

	int set_bin_ypr9(s_ahrs9 * p9)
	{
		int sz = sizeof(s_ahrs9)+sizeof(s_ahrs_ypr);
		int rblen = m_rbuf_tail - m_rbuf_head;
		if (rblen < sz)
			return 0;

		int res = rblen % sz;
		const char * ptr = m_rbuf + m_rbuf_tail - res - sz;
		memcpy((void*)&m_ypr, (void*)ptr, sizeof(s_ahrs_ypr));
		ptr += sizeof(s_ahrs_ypr);
		memcpy((void*)p9, (void*)ptr, sizeof(s_ahrs9));
		ptr += sizeof(s_ahrs9);
		for (int i = 0; i < res; i++, ptr++)
			m_rbuf[i] = *ptr;
		m_rbuf_head = 0;
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
		const char * ptr = m_rbuf + m_rbuf_tail - res - sz;
		memcpy((void*) p9, (void*) ptr, (size_t) sz);
		ptr += sz;
		for(int i = 0; i < res; i++, ptr++)
			m_rbuf[i] = *ptr;
		m_rbuf_head = 0;
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
		const char * ptr = m_rbuf + m_rbuf_tail - res - sz;
		memcpy((void*) p91, (void*) ptr, sizeof(s_ahrs9));
		ptr += sizeof(s_ahrs9);
		memcpy((void*) p92, (void*) ptr, sizeof(s_ahrs9));
		ptr += sizeof(s_ahrs9);
		for(int i = 0; i < res; i++, ptr++)
			m_rbuf[i] = *ptr;
		m_rbuf_head = 0;
		m_rbuf_tail = res;
		return sz;
	}

	void dec_tok_val()
	{
		float *px, *py, *pz;
		px = py = pz = NULL;
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

	bool m_b9dof;
	bool m_binit_9dof;
	long long m_t9dof_prev;
	float m_roll, m_pitch, m_yaw;
	float m_croll, m_sroll, m_cpitch, m_spitch, m_cyaw, m_syaw;
	float m_mag_x, m_mag_y;
	const float m_magg;
	const float m_Kai, m_Kap, m_Kmi, m_Kmp;
	float m_wx, m_wy, m_wz,
		m_waxi, m_wayi, m_wazi,
		m_waxp, m_wayp, m_wazp,
		m_wmxi, m_wmyi, m_wmzi,
		m_wmxp, m_wmyp, m_wmzp;

	Mat m_DCM;

	bool init_9dof(const long long t, const float mx, const float my, const float mz, const float ax,
		const float ay, const float az, const float gx, const float gy, const float gz);

	bool update_9dof(const long long t, const float mx, const float my, const float mz, const float ax,
		const float ay, const float az, const float gx, const float gy, const float gz);
	const float calc_yaw(const float mx, const float my, const float mz)
	{
		m_mag_x = mx * m_cpitch + my * m_sroll * m_spitch + mz * m_croll * m_spitch;
		m_mag_y = my * m_croll - mz * m_sroll;

		return atan2(-m_mag_y, m_mag_x);
	}

	void calc_xproduct(
		const float x1, const float y1, const float z1,
		const float x2, const float y2, const float z2,
		float & xo, float & yo, float & zo)
	{
		xo = y1 * z2 - z1 * y2;
		yo = z1 * x2 - x1 * z2;
		zo = x1 * y2 - y1 * x2;
	}

public:
	f_ahrs(const char * name);
	virtual ~f_ahrs();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
