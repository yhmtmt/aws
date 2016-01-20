
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
#include "f_nmea.h"

class f_aws1_nmea_sw: public f_base
{
protected:
	ch_nmea * m_aws_nmea_i;
	ch_nmea * m_ap_nmea_i;
	ch_nmea * m_gff_nmea_i;
	ch_nmea * m_ais_nmea_i;

	ch_nmea * m_aws_nmea_o;
	ch_nmea * m_ap_nmea_o;
	ch_nmea * m_gff_nmea_o;
	ch_nmea * m_ais_nmea_o;
	
	bool m_aws_ctrl;
	int m_aws_oint, m_ap_oint, m_gff_oint, m_ais_oint;
	int m_aws_ocnt, m_ap_ocnt, m_gff_ocnt, m_ais_ocnt;

	char m_nmea[83];
public:
	f_aws1_nmea_sw(const char * name);

	virtual ~f_aws1_nmea_sw();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
