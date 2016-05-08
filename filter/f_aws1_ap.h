#ifndef _F_AWS1_AP_H_
#define _F_AWS1_AP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ap.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ap.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ap.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_state.h"
#include "../channel/ch_wp.h"

// automatically controls along with the waypoints
// connects to ch_wp
class f_aws1_ap: public f_base
{
protected:
	ch_state * m_state;
	ch_aws1_ctrl_inst * m_ctrl_inst;
	ch_aws1_ctrl_stat * m_ctrl_stat;
	ch_wp * m_wp;
	s_aws1_ctrl_inst m_inst;

	bool m_verb;

	float m_meng, m_seng, m_rud;
	float m_smax;
	float m_meng_max, m_meng_min;
	float m_seng_max, m_seng_min;

	float m_cdiff, m_sdiff; // differences to the target values of course and speed.
	float m_dcdiff, m_dsdiff; // difference of cdiff and sdiff
	float m_icdiff, m_isdiff; // integral of cdiff and sdiff
	float m_pc, m_ic, m_dc;
	float m_ps, m_is, m_ds;
public:
	f_aws1_ap(const char * name);
	virtual ~f_aws1_ap();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif