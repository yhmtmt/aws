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

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/glu.h>


#include "../util/aws_glib.h"

#include "f_state_estimator.h"

#define GYRO_GAIN 0.06957

f_state_estimator::f_state_estimator(const char * name) : f_base(name), m_ch_state(NULL), m_ch_estate(NULL),
m_tpos_prev(0), m_tvel_prev(0), m_bacv(false), m_lag_x(10), m_lag_v(10), m_blog(false), m_bverb(false)
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
	register_fpar("rxx", pRx, "Rx(0, 0)");
	register_fpar("rxy", pRx + 1, "Rx(0, 1)");
	register_fpar("ryx", pRx + 2, "Rx(1, 0) should be equal to Rx(0, 1)");
	register_fpar("ryy", pRx + 3, "Rx(1, 1)");
	register_fpar("ruu", pRv, "Rv(0, 0)");
	register_fpar("ruv", pRv + 1, "Rv(0, 1)");
	register_fpar("rvu", pRv + 2, "Rv(1, 0) should be equal to Rv(0, 1)");
	register_fpar("rvv", pRv + 3, "Rv(1, 1)");

	register_fpar("acv", &m_bacv, "Calculating auto-covariance.");
	register_fpar("lag_x", &m_lag_x, "Maximum lag calculating auto-covariance for position.");
	register_fpar("lag_v", &m_lag_v, "Maximum lag calculating auto-covariance for velocity.");

	register_fpar("log", &m_blog, "Logging mode");

	register_fpar("verb", &m_bverb, "Debug mode");
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
	m_Px_ecef = Mat::zeros(3, 3, CV_32FC1);
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
		m_m_ex = m_m_ey = m_m_eu = m_m_ev = 0.;
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
		m_flog_x << "Time, Observation Time, lat, lon alt, elat, elon, ealt, x, y, z, ex, ey, ez, sxx, syy, szz, sxy, sxz, syz" << endl;

		m_flog_v << "Time Observation Time, u, v, eu, ev, cog, sog, ecog, esog, suu, svv, suv" << endl;
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
			ofile << "Avg of off-diagonal part," << soff << endl;

			ofile << "Autocovariance for velosity" << endl;
			soff = 0;
			for (int i = 0; i < m_ACVv.rows; i++){
				double * p = m_ACVv.ptr<double>(i);
				for (int j = 0; j < m_ACVv.cols; j++){
					ofile << *p * inv_cnt_v << ",";
					if (i != j)
						soff += abs(*p);
					p++;
				}
				ofile << endl;
			}
			soff *= inv_cnt_v;
			ofile << "Avg of off-diagonal part," << soff << endl;
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

	long long tpos;
	float gps_lat, gps_lon, gps_alt, gps_galt;
	float gps_xecef, gps_yecef, gps_zecef;
	Mat Renu;
	m_ch_state->get_position(tpos, gps_lat, gps_lon, gps_alt, gps_galt, gps_xecef, gps_yecef, gps_zecef, Renu);

	long long tvel;
	float cog, sog;
	m_ch_state->get_velocity(tvel, cog, sog);

	if (m_tpos_prev == 0 && tpos != 0){
		m_lat_prev = m_lat_opt = gps_lat;
		m_lon_prev = m_lon_opt = gps_lon;
		m_alt_prev = m_alt_opt = gps_alt;
		m_xecef_prev = m_xecef_opt = gps_xecef;
		m_yecef_prev = m_yecef_opt = gps_yecef;
		m_zecef_prev = m_zecef_opt = gps_zecef;
		m_Renu_prev = m_Renu_opt = Renu;
		m_ch_estate->set_pos_opt(t, m_lat_opt, m_lon_opt, m_alt_opt, 
			m_xecef_opt, m_yecef_opt, m_zecef_opt, m_Px_ecef, m_Px, m_Renu_opt);
		m_tpos_prev = tpos;
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
		m_ch_estate->set_vel_opt(t, u, v, cog, sog, m_Pv);
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


	if (tpos > m_tpos_prev){
		float dtx = (float)((tpos - m_tpos_prev) * (1.0 / (double)SEC));
		float dtv = (float)((tpos - m_tvel_prev) * (1.0 / (double)SEC));

		Mat Pdtv;
		Pdtv = dtv * m_Qv + m_Pv;
		Mat Pdtx;
		Pdtx = dtx * m_Qx + m_Px + dtx * dtx * Pdtv;

		// updating previous measurement
		float xp = m_u_opt * dtx, yp = m_v_opt * dtx;
		Mat Kx = Pdtx * (m_Rx + Pdtx).inv();
		float x, y, z; // observed x, y, z
		eceftowrld(m_Renu_opt, m_xecef_opt, m_yecef_opt, m_zecef_opt, gps_xecef, gps_yecef, gps_zecef, x, y, z);

		float ex = (float)(x - xp);
		float ey = (float)(y - yp);
		float * pK = Kx.ptr<float>(0);
		float xe = (float)(pK[0] * ex + pK[1] * ey + xp);
		float ye = (float)(pK[2] * ex + pK[3] * ey + yp);

		float x_opt, y_opt, z_opt;
		wrldtoecef(m_Renu_opt, m_xecef_opt, m_yecef_opt, m_zecef_opt, xe, ye, 0., x_opt, y_opt, z_opt);
		m_xecef_opt = x_opt;
		m_yecef_opt = y_opt;
		m_zecef_opt = z_opt;

		eceftobih(m_xecef_opt, m_yecef_opt, m_zecef_opt, m_lat_opt, m_lon_opt, m_alt_opt);

		m_Px = (Mat::eye(2, 2, CV_32FC1) - Kx) * Pdtx;
		m_Px_ecef = calc_cov_ecef(m_Px);

		getwrldrot(m_lat_opt, m_lon_opt, m_Renu_opt);

		m_lat_opt *= (float)(180. / PI);
		m_lon_opt *= (float)(180. / PI);

		m_ch_estate->set_pos_opt(tpos, m_lat_opt, m_lon_opt, m_alt_opt,
			m_xecef_opt, m_yecef_opt, m_zecef_opt, m_Px_ecef, m_Px, m_Renu_opt);

		m_lat_prev = gps_lat;
		m_lon_prev = gps_lon;
		m_alt_prev = gps_alt;
		m_xecef_prev = gps_xecef;
		m_yecef_prev = gps_yecef;
		m_zecef_prev = gps_zecef;
		m_Renu_prev = Renu;
		m_tpos_prev = tpos;

		if (m_bverb){
			cout << "tsys:" << m_cur_time << " t:" << tpos << " (xp,yp)=(" << xp << "," << yp << ")"
				<< " (xo, yo)=(" << x << "," << y << ")" 
				<< " (xe,ye)=(" << xe << "," << ye << ")" << endl;
		}

		if (m_bacv){
			calc_pos_acv(ex, ey);
		}

		if (m_blog){
			log_pos(t, tpos, gps_lat, gps_lon, gps_alt, gps_xecef, gps_yecef, gps_zecef);
		}
	}


	if (tvel > m_tvel_prev){
		float dtx = (float)((tvel - m_tpos_prev) * (1.0 / (double)SEC));
		float dtv = (float)((tvel - m_tvel_prev) * (1.0 / (double)SEC));

		Mat Pdtv;
		Pdtv = dtv * m_Qv + m_Pv;

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

		m_ch_estate->set_vel_opt(tvel, m_u_opt, m_v_opt, m_cog_opt, m_sog_opt, m_Pv);

		if (m_bverb){
			cout << "tsys:" << m_cur_time << " t:" << tvel << " (u,v)=(" << u << "," << v << ") (uopt,vopt)=(" 
				<< m_u_opt << "," << m_v_opt << ")" << endl;
		}

		if (m_bacv){
			calc_vel_acv(eu, ev);
		}

		if (m_blog){
			log_vel(t, tvel, u, v, cog, sog);
		}
	}

	return true;
}

void f_state_estimator::log_pos(const long long t, const long long tpos, 
	const float gps_lat, const float gps_lon, const float gps_alt,
	const float gps_xecef, const float gps_yecef, const float gps_zecef)
{
	float * p = m_Px_ecef.ptr<float>();
	m_flog_x << t << "," << tpos << ","
		<< gps_lat << "," << gps_lon << "," << gps_alt << ","
		<< m_lat_opt << "," << m_lon_opt << ",0,"
		<< gps_xecef << "," << gps_yecef << "," << gps_zecef << ","
		<< m_xecef_opt << "," << m_yecef_opt << "," << m_zecef_opt << ","
		<< p[0] << "," << p[4] << "," << p[8] << "," << p[1] << "," << p[2] << "," << p[5] << endl;
}

void f_state_estimator::log_vel(const long long t, const long long tvel, const float u, const float v, const float cog, const float sog)
{
	float * p = m_Pv.ptr<float>();
	m_flog_v << t << "," << tvel << ","
		<< u << "," << v
		<< m_u_opt << "," << m_v_opt << ","
		<< cog << "," << sog << ","
		<< m_cog_opt << "," << m_sog_opt
		<< p[0] << "," << p[3] << "," << p[1] << endl;
}


void f_state_estimator::calc_pos_acv(const float ex, const float ey)
{
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

void f_state_estimator::calc_vel_acv(const float eu, const float ev)
{
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
////////////////////////////////////////////////////////////////////// f_est_viewer

f_est_viewer::f_est_viewer(const char * name) : f_glfw_window(name), m_ch_state(NULL), m_ch_estate(NULL), m_range(3000), m_tf(180)
{
	register_fpar("ch_state", (ch_base**)(&m_ch_state), typeid(ch_state).name(), "State channel");
	register_fpar("ch_estate", (ch_base**)(&m_ch_estate), typeid(ch_estate).name(), "Estimated state channel");
	register_fpar("range", &m_range, "Range in the view (radius)");
}

f_est_viewer::~f_est_viewer()
{
}

bool f_est_viewer::init_run()
{
	if (!m_ch_state){
		cerr << "State channel is not connected." << endl;
		return false;
	}

	if (!m_ch_estate){
		cerr << "Estimated state channel is not connected." << endl;
		return false;
	}

	if (!f_glfw_window::init_run())
		return false;

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	m_xscale = (float)(0.5 * m_sz_win.width);
	m_yscale = (float)(0.5 * m_sz_win.height);
	m_ixscale = (float)(2.0 / (double)m_sz_win.width);
	m_iyscale = (float)(2.0 / (double)m_sz_win.height);

	for (int i = 0; i < 12; i++){
		float c, s;
		double th = i * PI * (2. / 12.);
		c = (float)(cos(th));
		s = (float)(sin(th));
		m_ncirc[i].x = c;
		m_ncirc[i].y = s;
	}
	return true;
}

void f_est_viewer::destroy_run()
{
}

bool f_est_viewer::proc()
{
	if (glfwWindowShouldClose(pwin()))
		return false;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	glDisable(GL_DEPTH_TEST);
	long long t = m_cur_time;
	long long tf = t + (long long)(m_tf * SEC);

	// coordinate system is centered at the position of last estimation 
	vector<s_pos_opt> pos_opt;
	m_ch_estate->get_pos_opt(pos_opt);
	if (pos_opt.size() == 0){
		return true;
	}

	// pixels per meter
	double ppm = (double)(m_sz_win.height >> 1) / m_range;
	double dfacx = ppm * m_ixscale, dfacy = ppm * m_iyscale;
	s_pos_opt & pcur = pos_opt[0];
	Mat Renuf, Renu = pcur.Renu;
	Renu.convertTo(Renuf, CV_32F);
	Point2f circ[12];
	Point2f offset_prev(0, 0);
	float xorg = pcur.xecef, yorg = pcur.yecef, zorg = pcur.zecef;
	for (int i = 0; i < pos_opt.size(); i++){
		s_pos_opt & p = pos_opt[i];
		Mat Penu = Renuf * p.Pecef * Renuf.t();
		float x, y, z;
		eceftowrld(Renu, xorg, yorg, zorg, p.xecef, p.yecef, p.zecef, x, y, z);

		// calculating center of the prediction
		Point2f offset((float)(x * dfacx), (float)(y * dfacy));
		float sx2 = Penu.at<float>(0, 0), sy2 = Penu.at<float>(1, 1), sz2 = Penu.at<float>(2, 2);
		float sx = (float)(sqrt(sx2) * dfacx), sy = (float)(sqrt(sy2) * dfacy);
		for (int j = 0; j < 12; j++){
			circ[j].x = m_ncirc[j].x * sx;
			circ[j].y = m_ncirc[j].y * sy;
		}

		drawGlPolygon2Df(circ, 12, offset, 0, 1, 0, 1);
		drawGlLine2Df(offset_prev.x, offset_prev.y, offset.x, offset.y, 0, 1, 0, 1, 1);
		offset_prev = offset;
	}

	glfwSwapBuffers(pwin());
	glfwPollEvents();
	return true;
}