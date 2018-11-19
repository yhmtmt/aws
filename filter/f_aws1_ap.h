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
	float dyaw, dcog, byaw, drev; // derivative of yaw, cog, rev and bias of yaw
	bool is_rud_ltor;
	float yaw_prev, cog_prev, rev_prev;
	long long tyaw_prev, tcog_prev, trev_prev;
	float devyaw, devcog, devrev;      // deviation of stable yaw, cog, rev

	long long twindow_stability_check;  // time window for stability check
	int twindow_stability_check_sec;   // second version of twindow_stability_check
	long long tbegin_stable;           // the time yaw/cog/rev stabilized
	float yaw_stbl, cog_stbl, rev_stbl;
	bool is_yaw_cog_rev_stable(const float cog,
				   const float yaw, const float rev);
	
	float rudmidlr, rudmidrl;
	float dir_local_flow, spd_local_flow;
	float tbl_stable_rpm[60];	
	float alpha_tbl_stable_rpm;
	float alpha_rud_mid;
	
	// for wp mode
	float m_cdiff, m_sdiff, m_revdiff; 
	float m_dcdiff, m_dsdiff, m_drevdiff; 
	float m_icdiff, m_isdiff, m_irevdiff;
	float m_prev, m_irev, m_drev; // PID for rev control
	float m_pc, m_ic, m_dc; // PID for course control
	float m_ps, m_is, m_ds; // PID for speed control
	
	float m_meng, m_seng, m_rud;
	unsigned short  dmeng, dseng, drud;
	unsigned short meng_prev, seng_prev, rud_prev;
	
	// control limitter 
	float m_smax, m_smin;
	float m_rev_max, m_rev_min;
	float m_meng_max, m_meng_min;
	float m_seng_max, m_seng_min;

	const float calc_course_change_for_ais_ship(const float yaw);
	void ctrl_to_sog_cog(const float sog,
			      const float cdiff, const float smax, const float smin);
	void ctrl_to_cog(const float cdiff);
	void ctrl_to_sog(const float sog, const float smax, const float smin);
	void ctrl_to_rev(const float rev, const float rev_tgt,
			 const float rev_max, const float rev_min);
	void stb_man(const float cog, const float rev);
	void flw_tgt(const float sog, const float cog, const float yaw, bool bav = false);
	void wp(const float sog, const float cog, const float yaw, bool bav = false);
	void stay(const float sog, const float cog, const float yaw);
	void cursor(const float sog, const float cog, const float yaw, bool bav = false);

	void calc_stat(const long long tcog, const float cog,
		       const long long tyaw, const float yaw,
		       const long long trev, const float rev,
		       const s_aws1_ctrl_stat & stat);
public:
	f_aws1_ap(const char * name);
	virtual ~f_aws1_ap();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
