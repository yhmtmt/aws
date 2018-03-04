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
	// input channels
	ch_state * m_state;
	ch_eng_state * m_engstate;
	ch_aws1_ctrl_stat * m_ch_ctrl_stat;
	ch_aws1_ctrl_inst * m_ch_ctrl_ui, *m_ch_ctrl_ap1, *m_ch_ctrl_ap2;
	
	s_aws1_ctrl_stat m_ctrl_stat;

	// output channels
	ch_state * m_state_sim;
	ch_eng_state * m_engstate_sim;
	ch_aws1_ctrl_stat * m_ch_ctrl_stat_sim;

	struct s_state_vector {
		long long t;
		double lat, lon, xe, ye, ze, roll, pitch, yaw, cog, sog;		
		Mat Rwrld;
		float eng, rud, rev, fuel; 
		s_state_vector(const long long & _t, const double & _lat, const double & _lon, const double & _roll,
			const double & _pitch, const double & _yaw, const double & _cog, const double & _sog,
			const float & _eng, const float & _rud, const float & _rev, const float & _fuel) :
			t(_t), lat(_lat), lon(_lon), roll(_roll), pitch(_pitch), yaw(_yaw), cog(_cog), sog(_sog),
			eng(_eng), rud(_rud), rev(_rev), fuel(_fuel)
		{
			update_coordinates();
		}

		s_state_vector() :lat(135.f), lon(35.f), roll(0.f), pitch(0.f), yaw(0.f), cog(0.f), sog(0.f),
			eng(127.0f), rud(127.0f), rev(700.f), fuel(0.1f)
		{
			update_coordinates();
		}

		void update_coordinates() {
			bihtoecef(lat, lon, 0, xe, ye, ze);
			getwrldrot(lat, lon, Rwrld);
		}
	};

	float m_int_smpl_sec;
	unsigned int m_int_smpl; // sampling interval (m_int_smpl_sec * 10e7)
	unsigned int m_iv_head;
	unsigned int m_wismpl; // inputs past m_wismpl * m_int_smpl secs are hold 
	vector<s_state_vector> m_input_vectors; // time sequence of  input vectors
	unsigned int m_wosmpl; // outputs next m_wismpl * m_int_smpl secs are calculated
	vector<s_state_vector> m_output_vectors; // time sequence of output vectors

	s_state_vector m_sv_init, m_sv_cur;
	void init_input_sample();
	void update_input_sample();
	void init_output_sample();
	void update_output_sample(const long long & tcur);

	// time of previous sampling 
	long long m_tprev;

	float m_rud_sta_sim;
	float m_trud_swing; // in second
	float m_spd_rud_swing; // rud_sta value per 100n second
	void simulate_rudder(long long tcur, long long tprev);

	float m_thrtl_eng; // engine throttle
	float m_rpm_eng;   // engine rpm
	void simulate_engine(long long tcur, long long trepv);

	float m_mass;	
	void simulate_dynamics(long long tcur, long long tprev);
	
	void set_control_input();
	void set_control_output();
	void set_input_state_vector(const long long & tcur);
	void set_output_state_vector();

	void simulate(const long long tsim, const int iosv);

	bool m_bcsv_out;
	char m_fcsv_out[1024];
	ofstream m_fcsv;
	void save_csv(const long long tcur);
 public:
	f_aws1_sim(const char * name);

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
