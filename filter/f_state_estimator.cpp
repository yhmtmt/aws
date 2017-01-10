#include "stdafx.h"
// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_state_estimator.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_state_estimator.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_state_estimator.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <thread>
#include <mutex>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;


#include "f_state_estimator.h"

f_state_estimator::f_state_estimator(const char * name) : f_base(name), m_ch_state(NULL), m_ch_estate(NULL),
m_tpos_prev(0), m_tvel_prev(0), m_bacv(false), m_lag_x(10), m_lag_v(10), m_blog(false)

{
	m_Qx = Mat::zeros(2, 2, CV_32FC1);
	float * pQx = m_Qx.ptr<float>();
	m_Qv = Mat::zeros(2, 2, CV_32FC1);
	float * pQv = m_Qv.ptr<float>();
	m_Rx = Mat::zeros(2, 2, CV_32FC1);
	float * pRx = m_Rx.ptr<float>();
	m_Rv = Mat::zeros(2, 2, CV_32FC1);
	float * pRv = m_Rv.ptr<float>();
	
	register_fpar("ch_state", (ch_base**)(&m_ch_state), typeid(ch_state).name(), "State channel");
	register_fpar("ch_estate", (ch_base**)(&m_ch_estate), typeid(ch_estate).name(), "Estimated state channel");
	register_fpar("qxx", pQx, "Qx(0, 0)");
	register_fpar("qxy", pQx + 1, "Qx(0, 1)");
	register_fpar("qyx", pQx + 2, "Qx(1, 0) should be equal to Qx(0, 1)");
	register_fpar("qyy", pQx + 3, "Qx(1, 1)");
	register_fpar("quu", pQv, "Qv(0, 0)");
	register_fpar("quv", pQv + 1, "Qv(0, 1)");
	register_fpar("qvu", pQv + 2, "Qv(1, 0) should be equal to Qv(0, 1)");
	register_fpar("qvv", pQv + 3, "Qv(1, 1)");
	register_fpar("rxx", pQx, "Rx(0, 0)");
	register_fpar("rxy", pQx + 1, "Rx(0, 1)");
	register_fpar("ryx", pQx + 2, "Rx(1, 0) should be equal to Rx(0, 1)");
	register_fpar("ryy", pQx + 3, "Rx(1, 1)");
	register_fpar("ruu", pQv, "Rv(0, 0)");
	register_fpar("ruv", pQv + 1, "Rv(0, 1)");
	register_fpar("rvu", pQv + 2, "Rv(1, 0) should be equal to Rv(0, 1)");
	register_fpar("rvv", pQv + 3, "Rv(1, 1)");

	register_fpar("acv", &m_bacv, "Calculating auto-covariance.");
	register_fpar("lag_x", &m_lag_x, "Maximum lag calculating auto-covariance for position.");
	register_fpar("lag_v", &m_lag_v, "Maximum lag calculating auto-covariance for velocity.");

	register_fpar("log", &m_blog, "Logging mode");

}

f_state_estimator::~f_state_estimator()
{
}

bool f_state_estimator::init_run()
{
	if (!m_ch_state){
		cerr << "State channel is not connected." << endl;
		return false;
	}

	if (!m_ch_estate){
		cerr << "Estimated state channel is not connected." << endl;
		return false;
	}

	m_tpos_prev = m_tvel_prev = 0;

	m_Px = Mat::zeros(2, 2, CV_32FC1);
	m_Pv = Mat::zeros(2, 2, CV_32FC1);

	if (m_bacv){
		m_ACVx = Mat::zeros(m_lag_x*2, m_lag_x*2, CV_64FC1);
		m_ACVv = Mat::zeros(m_lag_v*2, m_lag_v*2, CV_64FC1);
		m_ex.resize(m_lag_x);
		m_ey.resize(m_lag_x);
		m_eu.resize(m_lag_v);
		m_ev.resize(m_lag_v);
		m_cur_x = m_cnt_x = 0;
		m_cur_v = m_cnt_v = 0;
		m_m_ex = m_m_ey = m_m_eu = m_m_ev;
	}

	if (m_blog){
		char fname[1024];
		snprintf(fname, 1024, "%s_log_x.csv", m_name);
		m_flog_x.open(fname);
		if (!m_flog_x.is_open()){
			cerr << "Failed to open " << fname << endl;
			return false;
		}

		snprintf(fname, 1024, "%s_log_v.csv", m_name);
		m_flog_v.open(fname);
		if (!m_flog_v.is_open()){
			cerr << "Failed to open " << fname << endl;
			return false;
		}
		m_flog_x.precision(12);
		m_flog_v.precision(12);
		m_flog_x.setf(ios::scientific);
		m_flog_v.setf(ios::scientific);
		m_flog_x << "Time, Observation, lat, lon alt, elat, elon, ealt, x, y, z, ex, ey, ez, sxx, syy, szz, sxy, sxz, syz" << endl;

		m_flog_v << "Time Observation, u, v, eu, ev, cog, sog, ecog, esog, suu, svv, suv" << endl;
	}

	return true;
}

void f_state_estimator::destroy_run()
{
	if (m_bacv){
		char fname[1024];
		snprintf(fname, 1024, "%s_acv.csv", m_name);
		ofstream ofile(fname);

		if (ofile.is_open()){
			double inv_cnt_x = 1.0 / m_cnt_x;
			double inv_cnt_v = 1.0 / m_cnt_v;
			double soff = 0.;
			m_m_ex *= inv_cnt_x;
			m_m_ey *= inv_cnt_x;
			m_m_eu *= inv_cnt_v;
			m_m_ev *= inv_cnt_v;
			ofile << "Number of position samples, " << m_cnt_x << endl;
			ofile << "Average error (x, y)," << m_m_ex << "," << m_m_ey << endl;
			ofile << "Number of velosity samples, " << m_cnt_v << endl;
			ofile << "Average error (u, v)," << m_m_eu << "," << m_m_ev << endl;

			ofile << "Autocovariance for position" << endl;
			for (int i = 0; i < m_ACVx.rows; i++){
				double * p = m_ACVx.ptr<double>(i);
				for (int j = 0; j < m_ACVx.cols; j++){
					ofile << *p * inv_cnt_x << ",";
					if (i != j)
						soff += abs(*p);
					p++;			
				}
				ofile << endl;
			}
			soff *= inv_cnt_x;
			ofile << "Sum of off-diagonal part," << soff << endl;

			ofile << "Autocovariance for velosity" << endl;
			for (int i = 0; i < m_ACVv.rows; i++){
				double * p = m_ACVv.ptr<double>(i);
				for (int j = 0; j < m_ACVv.cols; j++){
					ofile << *p * inv_cnt_v << ",";
					if (i != j)
						soff += abs(*p);
					p++;
				}
			}
			soff *= inv_cnt_v;
			ofile << "Sum of off-diagonal part," << soff << endl;
		}
	}

	if (m_blog){
		m_flog_x.close();
		m_flog_v.close();
	}
}

bool f_state_estimator::proc()
{

	long long t = m_cur_time;
	// retrieve state from state channel
	long long tbih;
	float gps_lat, gps_lon, gps_alt, gps_galt;
	m_ch_state->get_position(tbih, gps_lat, gps_lon, gps_alt, gps_galt);
	long long tecef;
	float gps_xecef, gps_yecef, gps_zecef;
	m_ch_state->get_position_ecef(tecef, gps_xecef, gps_yecef, gps_zecef);
	long long tenu;
	Mat Renu = m_ch_state->get_enu_rotation(tenu);

	long long tvel;
	float cog, sog;
	m_ch_state->get_velocity(tvel, cog, sog);

	float dtx = (float)((m_cur_time - tbih) * (1.0 / (double)SEC));
	float dtv = (float)((m_cur_time - tvel) * (1.0 / (double)SEC));

	if (m_tpos_prev == 0 && tbih != 0){
		m_lat_prev = m_lat_opt = gps_lat;
		m_lon_prev = m_lon_opt = gps_lon;
		m_alt_prev = m_alt_opt = gps_alt;
		m_xecef_prev = m_xecef_opt = gps_xecef;
		m_yecef_prev = m_yecef_opt = gps_yecef;
		m_zecef_prev = m_zecef_opt = gps_zecef;
		m_Renu_prev = m_Renu_opt = Renu;
		m_ch_estate->set_pos(t, m_lat_opt, m_lon_opt, 0);
		m_ch_estate->set_pos_ecef(t, m_xecef_opt, m_yecef_opt, m_zecef_opt, m_Px);
		m_ch_estate->set_enu_rot(t, m_Renu_opt);
		m_tpos_prev = tbih;
		return true;
	}

	if (m_tvel_prev == 0 && tvel != 0){
		float cog_rad = (float)(cog * (PI / 180.));
		float cog_cos = (float)cos(cog_rad);
		float cog_sin = (float)sin(cog_rad);
		float sog_mps = (float)(sog * (1852. / 3600.));
		float u = (float)(sog_mps * cog_sin), v = (float)(sog_mps * cog_cos);
		m_cog_prev = m_cog_opt = cog;
		m_sog_prev = m_sog_opt = sog;
		m_u_prev = m_u_opt = u;
		m_v_prev = m_v_opt = v;
		m_ch_estate->set_velp(t, cog, sog);
		m_ch_estate->set_vel(t, u, v, m_Pv);
		m_tvel_prev = tvel;
		return true;
	}

	if (m_tvel_prev == 0 || m_tpos_prev == 0){
		return true;
	}

	// write estimated state to estate channel
	// estimation (x, y, u, v) is observed position in enu coordinate, (xp, yp, up, vp) is estimated position in enu coordinate
	//| xp |  =|  1  0  t  0  ||x|  = Fx(t) |x|
 	//| yp |   |  0  1  0  t  ||y|          |y|
	//                         |u|          |u| 
	//                         |v|          |v|
	//
	// constant velocity assumption
	//| up |  =|  1  0  ||u| = Fv(t) |u|
	//| vp |   |  0  1  ||v|         |v|

	// residual (xe, ye, ue, ve) is difference between  (xp, yp, up, vp) and  (x, y, u, v)
	// |ex| = |x| - |xe|
	// |ey|   |y|   |ye|
	//
	// |eu| = |u| - |ue|
	// |ev|   |v|   |ve|

	// State variable (xp, yp, up, vp) contains error q = (qx, qy, qu, qv), a 4-d normal random variables.
	// In unit time, covariance matrix Q is 
	// Qx = | qx  0 |
	//      |  0 qy |
	// Qv = | qu  0 |
	//      |  0 qv |

	// Predicted covariance matrices
	// Px(dtx) = dtx Qx + Fx(dtx) |Px(0)       0 | Fx(dtx)^t
	//                            |    0  Pv(dtv)|
	// Pv(dtv) = dtv Qv + Fv(dtv) Pv(0) Fv(dtv)^t
	// 
	//where Px(0) are Pv(0) covariances in previous estimations.
	// dtx and dtv are the elapsed times after last observations.

	// Ovservation errors 
	// Rx = | rx  0 |
	//      |  0 ry |
	// Rv = | ru  0 |
	//      |  0 rv |
	
	// Kalman Gain K is
	// Kx = Px(dtx)(Rx + P(dtx))^-1
	// Kv = Pv(dtv)(Rv + P(dtv))^-1

	// Estimation is done when observations take place.
	// | xe | = Kx | ex | + | x |
	// | ye |      | ey |   | y |
	//
	// | ue | = Kv | eu | +  | u |
	// | ve |      | ev |    | v |

	Mat Pdtv;
	Pdtv = dtv * m_Qv + m_Pv;
	if (tbih == tecef && tbih == tenu){
		Mat Pdtx;
		Pdtx = dtx * m_Qx + m_Px + dtx * dtx * Pdtv;

		if (tbih > m_tpos_prev){
			// updating previous measurement
			float xp = m_u_prev * dtx, yp = m_v_prev * dtx;
			Mat Kx = Pdtx * (m_Rx + Pdtx).inv();
			float x, y, z; // observed x, y, z
			//eceftowrld(m_Renu_opt, m_xecef_opt, m_yecef_opt, m_zecef_opt, gps_xecef, gps_yecef, gps_zecef, x, y, z);
			x = y = z = 0.;
			float ex = (float)(x - xp), ey = (float)(y - yp);

			float *pK = Kx.ptr<float>(0);
			float xe, ye;

			xe = (float)(pK[0] * ex + pK[1] * ey + xp);
			ye = (float)(pK[2] * ex + pK[3] * ey + yp);
			float x_opt, y_opt, z_opt;
			wrldtoecef(m_Renu_opt, m_xecef_opt, m_yecef_opt, m_zecef_opt, xe, ye, 0.f,x_opt, y_opt, z_opt);
			m_xecef_opt = x_opt;
			m_yecef_opt = y_opt;
			m_zecef_opt = z_opt;
			eceftobih(m_xecef_opt, m_yecef_opt, m_zecef_opt, m_lat_opt, m_lon_opt, m_alt_opt);
			m_Px = (Mat::eye(2, 2, CV_32FC1) - Kx) * Pdtx;

			m_Px_ecef = calc_cov_ecef(m_Px);

			getwrldrot(m_lat_opt, m_lon_opt, m_Renu_opt);

			m_lat_opt *= (float)(180. / PI);
			m_lon_opt *= (float)(180. / PI);
			m_ch_estate->set_pos(tbih, m_lat_opt, m_lon_opt, 0);
			m_ch_estate->set_pos_ecef(tbih, m_xecef_opt, m_yecef_opt, m_zecef_opt, m_Px_ecef);
			m_ch_estate->set_enu_rot(tbih, m_Renu_opt);

			m_lat_prev = gps_lat;
			m_lon_prev = gps_lon;
			m_alt_prev = gps_alt;
			m_xecef_prev = gps_xecef;
			m_yecef_prev = gps_yecef;
			m_zecef_prev = gps_zecef;
			m_Renu_prev = Renu;
			m_tpos_prev = tbih;

			if (m_bacv){
				m_ex[m_cur_x] = ex;
				m_m_ex += ex;
				m_ey[m_cur_x] = ey;
				m_m_ey += ey;
				m_cur_x = (m_cur_x + 1) % m_lag_x;
				m_cnt_x++;
				if (m_cnt_x > 10){
					for (int i = 0, l = m_cur_x; i < m_lag_x; i++){
						for (int j = i, m = l; j < m_lag_x; j++){
							double * pij = m_ACVx.ptr<double>(i * 2, j * 2);
							pij[0] += m_ex[l] * m_ex[m];
							pij[1] += m_ex[l] * m_ey[m];
							pij += m_ACVx.cols;
							pij[0] += m_ey[l] * m_ex[m];
							pij[1] += m_ey[l] * m_ev[m];

							m--;
							if (m < 0)
								m += m_lag_x;
						}
						l--;
						if (l < 0)
							l += m_lag_x;
					}
				}
			}

			if (m_blog){
				float * p = m_Px.ptr<float>();
				m_flog_x << t << ",1," 
					<< gps_lat << "," << gps_lon << "," << gps_alt << "," 
					<< m_lat_opt << "," << m_lon_opt << ",0," 
					<< gps_xecef << "," << gps_yecef << "," << gps_zecef << ","
					<< m_xecef_opt << "," << m_yecef_opt << "," << m_zecef_opt
					<< p[0] << "," << p[4] << "," << p[8] << "," << p[1] << "," << p[2] << "," << p[5] << endl;
			}
		}
		else{
			// update prediction
			
			float xp = m_u_prev * dtx, yp = m_v_prev * dtx;
			float xpecef, ypecef, zpecef, plat, plon, palt;
			wrldtoecef(m_Renu_opt, m_xecef_opt, m_yecef_opt, m_zecef_opt, xp, yp, 0.f, xpecef, ypecef, zpecef);
			eceftobih(xpecef, ypecef, zpecef, plat, plon, palt);
			Mat P = calc_cov_ecef(Pdtx);
			Mat R;
			getwrldrot(plat, plon, R);
			plat *= (float)(180. / PI);
			plon *= (float)(180. / PI);
			m_ch_estate->set_pos(t, plat, plon, 0.f);
			m_ch_estate->set_pos_ecef(t, xpecef, ypecef, zpecef, P);
			m_ch_estate->set_enu_rot(t, R);

			if (m_blog){
				float * p = P.ptr<float>();
				m_flog_x << t << ",0," 
					<< ",,,"
					<< plat << "," << plon << ",0," 
					<< ",,,"
					<< xpecef << "," << ypecef << "," << zpecef
					<< p[0] << "," << p[4] << "," << p[8] <<  "," << p[1] << "," << p[2] << "," << p[5] << endl;
			}
			
		}
	}

	if (tvel > m_tvel_prev){
		Mat Kv = Pdtv * (m_Rv + Pdtv).inv();
		float cog_rad = (float)(cog * (PI / 180.));
		float cog_cos = (float)cos(cog_rad);
		float cog_sin = (float)sin(cog_rad);
		float sog_mps = (float)(sog * (1852. / 3600.));
		float u = (float)(sog_mps * cog_sin), v = (float)(sog_mps * cog_cos);
		m_cog_prev = cog;
		m_sog_prev = sog;
		m_u_prev = u;
		m_v_prev = v;
		m_Pv = (Mat::eye(2, 2, CV_32FC1) - Kv) * Pdtv;
		float eu = (float)(u - m_u_opt), ev = (float)(v - m_v_opt);
		float * pK = Kv.ptr<float>(0);
		m_u_opt = (float)(pK[0] * eu + pK[1] * ev + u);
		m_v_opt = (float)(pK[2] * eu + pK[3] * ev + v);
		m_cog_opt = (float)atan2(u, v) * (180. / PI);
		m_sog_opt = (float)sqrt(m_u_opt * m_u_opt + m_v_opt * m_v_opt) * (3600. / 1852.);
		m_tvel_prev = t;
		m_ch_estate->set_velp(tvel, m_cog_opt, m_sog_opt);
		m_ch_estate->set_vel(tvel, m_u_opt, m_v_opt, m_Pv);

		if (m_bacv){
			m_eu[m_cur_v] = eu;
			m_m_eu += eu;
			m_ev[m_cur_v] = ev;
			m_m_ev += ev;
			m_cur_v = (m_cur_v + 1) % m_lag_v;
			m_cnt_v++;
			if (m_cnt_v > 10){
				for (int i = 0, l = m_cur_v; i < m_lag_v; i++){
					for (int j = i, m = l; j < m_lag_v; j++){
						double * pij = m_ACVv.ptr<double>(i * 2, j * 2);
						pij[0] += m_eu[l] * m_eu[m];
						pij[1] += m_eu[l] * m_ev[m];
						pij += m_ACVv.cols;
						pij[0] += m_ev[l] * m_eu[m];
						pij[1] += m_ev[l] * m_ev[m];

						m--;
						if (m < 0)
							m += m_lag_v;
					}
					l--;
					if (l < 0)
						l += m_lag_v;
				}
			}
		}

		if (m_blog){
			float * p = m_Pv.ptr<float>();
			m_flog_v << t << ",1," 
				<< u << "," << v 
				<< m_u_opt << "," << m_v_opt << "," 
				<< cog << "," << sog << ","
				<< m_cog_opt << "," << m_sog_opt
				<< p[0] << "," << p[3] << "," << p[1] << endl;
		}
	}

	return true;
}


