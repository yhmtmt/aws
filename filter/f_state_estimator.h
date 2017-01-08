#ifndef _F_STATE_ESTIMATOR_H_
#define _F_STATE_ESTIMATOR_H_

// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_state_estimator.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_state_estimator.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_state_estimator.h.  If not, see <http://www.gnu.org/licenses/>. 


#include "../util/aws_sock.h"
#include "../util/aws_vlib.h"


#include "../channel/ch_state.h"
#include "f_base.h"

class f_state_estimator : public f_base
{
protected:
	ch_state * m_ch_state;
	ch_estate * m_ch_estate;
	long long m_tpos_prev;
	float m_lat_prev, m_lon_prev, m_alt_prev;
	float m_xecef_prev, m_yecef_prev, m_zecef_prev;
	float m_lat_opt, m_lon_opt, m_alt_opt;
	float m_xecef_opt, m_yecef_opt, m_zecef_opt;

	long long m_tvel_prev;
	float m_cog_prev, m_sog_prev;
	float m_u_prev, m_v_prev;
	float m_u_opt, m_v_opt;

	Mat m_Renu_prev, m_Renu_opt;
	Mat m_Qx, m_Qv, m_Rx, m_Rv, m_Px, m_Pv;

public:
	f_state_estimator(const char * name);
	~f_state_estimator();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
