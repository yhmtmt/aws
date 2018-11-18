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
#include "../channel/ch_obj.h"
#include "../channel/ch_wp.h"

// automatically controls along with the waypoints
// connects to ch_wp
class f_aws1_ap: public f_base
{
protected:
	ch_state * m_state;
	ch_eng_state * m_engstate;
	ch_aws1_ctrl_inst * m_ctrl_inst;
	ch_aws1_ctrl_stat * m_ctrl_stat;
	ch_aws1_ap_inst * m_ap_inst;
	ch_wp * m_wp;
	ch_obst * m_obst;
	ch_ais_obj * m_ais_obj;

	s_aws1_ctrl_inst m_inst;
	float m_Lo, m_Wo; // assumed size for my own ship
	float m_Lais, m_Wais; // assumed size for ais target
	float m_Rav; // range for avoidance(multiple of ship size)
	float m_Tav; // time for avoidance
	float m_Cav_max; // maximum course change in degree

	bool m_verb;

	// control situation estimate
	char fname_situation_estimate[1024];
	float dyaw, dcog, byaw;
	float rudmidlr, rudmidrl;
	float dir_local_flow, spd_local_flow;
	float tbl_stable_rpm[256];
	float tbl_stable_spd[60];

	// for wp mode
	float m_cdiff, m_sdiff; // differences to the target values of course and speed.
	float m_dcdiff, m_dsdiff; // difference of cdiff and sdiff
	float m_icdiff, m_isdiff; // integral of cdiff and sdiff
	float m_pc, m_ic, m_dc;
	float m_ps, m_is, m_ds;
	float m_meng, m_seng, m_rud;
	float m_smax, m_smin;
	float m_meng_max, m_meng_min;
	float m_seng_max, m_seng_min;

	// for stay mode
	float m_ydiff;
	float m_dydiff;
	float m_iydiff;
	float m_meng_max_stay, m_meng_min_stay;
	float m_rud_max_stay, m_rud_min_stay;
	float m_ssmax; // maximum speed
	float m_dssmax; //distance maximum speed allowed
	float m_pc_s, m_ic_s, m_dc_s;
	float m_ps_s, m_is_s, m_ds_s;
	const float calc_course_change_for_ais_ship(const float yaw);
	void ctrl_to_location(const float sog, const float d,
			      const float cdiff, float smax, float smin);
	void flw_tgt(const float sog, const float cog, const float yaw, bool bav = false);
	void wp(const float sog, const float cog, const float yaw, bool bav = false);
	void stay(const float sog, const float cog, const float yaw);
	void cursor(const float sog, const float cog, const float yaw, bool bav = false);

	void calc_stat();
public:
	f_aws1_ap(const char * name);
	virtual ~f_aws1_ap();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
