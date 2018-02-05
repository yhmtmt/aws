#ifndef F_AWS1_SIM_H
#define F_AWS1_SIM_H
// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_aws1_sim.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_sim.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_sim.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "f_base.h"

#include "../channel/ch_state.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_aws1_sys.h"

//////////////////////////////////////////////////////// f_aws1_sim
#define RUD_PER_CYCLE 0.45f

class f_aws1_sim : public f_base
{
protected:
	ch_state * m_state;
	ch_eng_state * m_engstate;
	ch_aws1_ctrl_stat * m_ch_ctrl_stat;
	ch_aws1_ctrl_inst * m_ch_ctrl_ui, *m_ch_ctrl_ap1, *m_ch_ctrl_ap2;

	long long m_tprev;
	bool m_ahrs, m_gps;
	float r, p, y; // roll(deg), pitch(deg), yaw(deg)
	float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	float cog, sog; // Course over ground(deg), Speed over ground (kts)
	float depth; // water depth
	s_aws1_ctrl_stat m_stat;

	float m_rud_sta_sim;
	float m_trud_swing; // in second
	float m_spd_rud_swing; // rud_sta value per 100n second
	void simulate_rudder(long long tcur, long long tprev);

	float m_thrtl_eng; // engine throttle
	float m_rpm_eng;   // engine rpm
	void simulate_engine(long long tcur, long long trepv);

	float m_mass;	
	void simulate_dynamics(long long tcur, long long tprev);
	
	void select_control_input();
	void set_control_output();

	void set_state();

 public:
	f_aws1_sim(const char * name);

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
