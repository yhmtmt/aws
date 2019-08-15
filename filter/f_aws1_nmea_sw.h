
// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_aws1_nmea_sw.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_nmea_sw.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_nmea_sw.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AWS1_NMEA_SW_H_
#define _F_AWS1_NMEA_SW_H_

#include "../util/aws_stdlib.h"
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/aws_nmea.h"
#include "../util/aws_coord.h"

#include "../channel/ch_nmea.h"
#include "../channel/ch_state.h"
#include "../channel/ch_obj.h"

#include "f_nmea.h"

inline bool is_nmea_type(const char * strt, const char * nmea){
  return nmea[3] == strt[0] && nmea[4] == strt[1] && nmea[5] == strt[2];
}

class f_aws1_nmea_sw: public f_base
{
protected:
	c_nmea_dec m_nmea_dec;
	ch_state * m_state;
	ch_ais_obj * m_ais_obj;

	ch_nmea * m_aws_nmea_i;
	ch_nmea * m_wx220_nmea_i;
	ch_nmea * m_gff_nmea_i;
	ch_nmea * m_ais_nmea_i;
	ch_nmea * m_v104_nmea_i;

	ch_nmea * m_aws_nmea_o;
	ch_nmea * m_wx220_nmea_o;
	ch_nmea * m_gff_nmea_o;
	ch_nmea * m_ais_nmea_o;
	ch_nmea * m_v104_nmea_o;

	bool m_aws_ctrl;
	bool m_verb;
	int m_aws_oint, m_wx220_oint, m_gff_oint, m_ais_oint, m_v104_oint;
	int m_aws_ocnt, m_wx220_ocnt, m_gff_ocnt, m_ais_ocnt, m_v104_ocnt;

	bool m_aws_out, m_wx220_out, m_gff_out, m_ais_out, m_v104_out;

	char m_nmea[84];

	void aws_to_out();
	void wx220_to_out();
	void gff_to_out();
	void ais_to_out();
	void v104_to_out();
public:
	f_aws1_nmea_sw(const char * name);

	virtual ~f_aws1_nmea_sw();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};


#endif
