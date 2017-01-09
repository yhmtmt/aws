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
	float m_cog_opt, m_sog_opt;

	Mat m_Renu_prev, m_Renu_opt;
	Mat m_Qx, m_Qv, m_Rx, m_Rv, m_Px, m_Pv, m_Px_ecef;

	Mat calc_cov_ecef(Mat & Penu){
		Mat Pecef = Mat::zeros(3, 3, CV_32FC1);
		Mat R;
		m_Renu_opt.convertTo(R, CV_32FC1);
		Pecef.at<float>(0, 0) = Penu.at<float>(0, 0);
		Pecef.at<float>(0, 1) = Penu.at<float>(0, 1);
		Pecef.at<float>(1, 0) = Penu.at<float>(1, 0);
		Pecef.at<float>(1, 1) = Penu.at<float>(1, 1);
	    Pecef = R * Pecef;
		Pecef *= R.t();
		return Pecef;
	}

	// Related to measuring auto-covariance
	bool m_bacv;
	int m_lag_x, m_lag_v;
	Mat m_ACVx, m_ACVv;
	double m_m_ex, m_m_ey, m_m_eu, m_m_ev;
	vector<float> m_ex, m_ey, m_eu, m_ev;
	int m_cur_x, m_cur_v;
	int m_cnt_x, m_cnt_v;

	bool m_blog;
	ofstream m_flog_x, m_flog_v;

public:
	f_state_estimator(const char * name);
	~f_state_estimator();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
