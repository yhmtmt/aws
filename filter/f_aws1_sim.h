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

class c_model_3dof
{
private:
	double xg, yg, iz;
	double m; // mass matrix
	double ma[9]; // added mass matrix
	double dl[9]; // linear drag matrix
	double dq[9]; // quadratic drag matrix

	Mat M;
	Mat Minv;
	Mat Dl;
	Mat Dq;
	Mat V;
	Mat C;

	double mxx;
	double myy;
	double mxg;
	double ny;

	double v[3]; // state vector (u, v, r)
public:
	c_model_3dof():xg(0), yg(0)
	{
		for (int i = 0; i < 9; i++)
			ma[i] = dl[i] = dq[i] = 0;
	}

	~c_model_3dof()
	{
	}

	double & Iz()
	{
		return iz;
	}

	double & xgrav()
	{
		return xg;
	}

	double & ygrav()
	{
		return yg;
	}

	double & mass()
	{
		return m;
	}

	double & mass_a(int i, int j)
	{
		return ma[i * 3 + j];
	}

	double & drag_l(int i, int j)
	{
		return dl[i * 3 + j];
	}

	double & drag_q(int i, int j)
	{
		return dq[i * 3 + j];
	}

	void init_matrix()
	{
		M = Mat::zeros(3, 3, CV_64FC1);
		double * data = M.ptr<double>();
		
		for (int i = 0; i < 9; i++)
			data[i] = ma[i];
		data[0] += m;
		data[4] += m; 
		data[8] += iz;

		mxx = data[0];
		myy = data[4];
		mxg = m * xg;
		ny = (ma[5] + ma[7]) * 0.5;

		Dl = Mat::zeros(3, 3, CV_64FC1);
		data = Dl.ptr<double>();
		for (int i = 0; i < 9; i++) {
			data[i] = dl[i];		
		}

		Dq = Mat::zeros(3, 3, CV_64FC1);
		C = Mat(3, 3, CV_64FC1);

		V = Mat(3, 1, CV_64FC1, v);	
	}

	void set_state(const double * _v)
	{
		v[0] = _v[0];
		v[1] = _v[1];
		v[2] = _v[2];
	}

	void get_sate(double * _v)
	{
		_v[0] = v[0];
		_v[1] = v[1];
		_v[2] = v[2];
	}

	bool update_state(double * f, const double dt)
	{
		if (Dq.empty() || M.empty() || Dl.empty()) {
			cerr << "Matrix should be initialized!" << endl;
			return false;
		}
		// initializing quadratic drag
		double * data = Dq.ptr<double>();
		for (int i = 0; i < 9; i++) {
			data[i] = v[i % 3] * dq[i];
		}
		data = C.ptr<double>();

		double * mdata = M.ptr<double>();
		double mxxu = mxx * v[0];
		double myyv = myy * v[1];
		double mxgr = mxg * v[2];
		double nyr = ny * v[2];
		data[2] = -mxgr - myyv + nyr;
		data[5] = mxxu;
		data[6] = -data[2];
		data[7] = -data[5];

		Mat T(3, 1, CV_64FC1, f);
		Mat Vnext;
		Vnext = V + M.inv() * (T + (C + Dl + Dq) * V) * dt;

		data = Vnext.ptr<double>();
		v[0] = data[0];
		v[1] = data[1];
		v[2] = data[2];

		return true;
	}
};

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
		float thro_pos, gear_pos, rud_pos;
		s_state_vector(const long long & _t, const double & _lat, const double & _lon, const double & _roll,
			const double & _pitch, const double & _yaw, const double & _cog, const double & _sog,
			const float & _eng, const float & _rud, const float & _rev, const float & _fuel) :
			t(_t), lat(_lat), lon(_lon), roll(_roll), pitch(_pitch), yaw(_yaw), cog(_cog), sog(_sog),
			eng(_eng), rud(_rud), rev(_rev), fuel(_fuel)
		{
			update_coordinates();
		}

		s_state_vector() :lat(135.f), lon(35.f), roll(0.f), pitch(0.f), yaw(0.f), cog(0.f), sog(0.f),
			eng(127.0f), rud(127.0f), rev(700.f), fuel(0.1f), thro_pos(0.f), gear_pos(0), rud_pos(0)
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
	float m_tgear_swing; // in second
	float m_tthro_swing; // in second
	float m_spd_rud_swing; // rud_sta value per 100n second
	float m_spd_gear_swing; 
	float m_spd_thro_swing;
	float m_tau_sog; // time constant for final value of sog by rev

	float m_mass;	
	
	void set_control_input();
	void set_control_output();
	void set_input_state_vector(const long long & tcur);
	void set_output_state_vector();

	void simulate(const long long tcur, const int iosv);
	void simulate_rudder(const float rud, const float rud_pos, float & rud_pos_next);
	void simulate_engine(const float eng, const float eng_pos, const float gear_pos, float & eng_pos_next, float & gear_pos_next);
	const float final_rev(const float thr_pos)
	{
		if (thr_pos < 0.417)
			return 700;
		if (thr_pos < 0.709)
			return (5000 - 700) * (thr_pos - 0.417) / (0.709 - 0.417) + 700;
		if (thr_pos < 0.854)
			return (5600 - 5000) * (thr_pos - 0.709) / (0.854 - 0.709) + 5000;
		return 5600;
	}

	float simulate_sog(const float gear_pos, const float rev,  const float sog) {
		// speed over ground is modeled as first order differential equation.
		// sog_final = sog + tau (d sog/d t) ---(1)
		// (1) is transformed as
		// sog_final = sog_next  + tau (sog_next - sog_prev) / delta_t
		// here delta_t is m_int_smpl_sec, then we can solve the equation for sog
		// sog_next = [sog_prev + (delta_t / tau) sog_final]/[1 + (delta_t / tau)]
		float sog_final = final_sog(gear_pos, rev);
		float tdt = (float)((float)m_int_smpl_sec / m_tau_sog );
		return (float)(sog  + tdt * sog_final) / (1.0f + tdt);
	}

	float final_sog(const float gear_pos, const float rev)
	{
		if (gear_pos <= -1.f) {
			return 2.5f;
		}
		else if (gear_pos >= 1.f) {
			if (rev < 700)
				return 2.5f;
			if (rev < 3500)
				return (float)((11.f - 2.5f)*(rev - (float)700) / (double)(3500 - 700) + 2.5f);
			if(rev < 4800)
				return (float)((19.5f - 11.f)*(rev - (float)3500) / (double)(4800 - 3500) + 11.f);
			if (rev < 5500)
				return (float)((22.5f - 19.5f)*(rev - (float)4800) / (double)(5500 - 4800) + 19.5f);
			return 22.5f;
		}

		return 0.0f;
	}

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
