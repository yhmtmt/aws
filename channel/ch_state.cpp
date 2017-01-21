#include "stdafx.h"
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_state.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_state.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_state.cpp.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <cstring>
#include <map>
using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "ch_state.h"

bool ch_estate::get_pos(const long long t, const Mat & Qv, const Mat & Qx,
	float & lat, float & lon, float & alt,
	float & xecef, float & yecef, float & zecef,
	Mat & Pecef, Mat & Renu, bool both)

{
	if (num_pos_opt == 0)
		return false;

	float u, v;
	Mat Pv;
	if (!get_vel(t, Qv, u, v, Pv))
		return false;

	lock();
	// seek nearest position record
	int ipos_new = (cur_pos_opt + pos_opt.size() - 1) % pos_opt.size();
	int ipos_old = (ipos_new + num_pos_opt - 1) % pos_opt.size();
	long long tnew = pos_opt[ipos_new].t;
	long long told = pos_opt[ipos_old].t;
	if (tnew < t){
		ipos_old = ipos_new;
		ipos_new = -1;
	}
	else if (told > t){
		ipos_new = ipos_old;
		ipos_old = -1;
	}
	else{
		int n = num_pos_opt;
		while (n != 0)
		{
			int nmid = n >> 1;
			int ipos_mid = (ipos_old + nmid) % pos_opt.size();
			if (pos_opt[ipos_mid].t < t){
				ipos_old = ipos_mid;
				n -= nmid;
			}
			else{
				ipos_new = ipos_mid;
				n = nmid;
			}
		}
	}

	float dtxf, dtxb;
	Mat Pdtxf, Pdtxb, Peceff, Pecefb;
	float xf, yf, xb, yb;
	float xef, yef, zef, xeb, yeb, zeb;
	if (ipos_old >= 0){
		s_pos_opt & pold = pos_opt[ipos_old];
		dtxf = (float)((t - pold.t)  * (1.0 / (double)SEC));
		Pdtxf = pold.Penu + dtxf * Qx + dtxf * dtxf * Pv;

		xf = (float)(u * dtxf);
		yf = (float)(v * dtxf);
		wrldtoecef(pold.Renu, pold.xecef, pold.yecef, pold.zecef, xf, yf, 0., xef, yef, zef);
		Peceff = calc_Pecef(pold.Renu, Pdtxf);
	}

	if ((both && ipos_new >= 0) || ipos_old < 0){
		s_pos_opt & pnew = pos_opt[ipos_new];
		dtxb = (float)((pnew.t - t) * (1.0 / (double)(SEC)));
		Pdtxb = pnew.Penu + dtxb * Qx + dtxb * dtxb * Pv;

		xb = (float)(-u * dtxb);
		yb = (float)(-v * dtxb);
		wrldtoecef(pnew.Renu, pnew.xecef, pnew.yecef, pnew.zecef, xb, yb, 0., xeb, yeb, zeb);
		Pecefb = calc_Pecef(pnew.Renu, Pdtxb);
	}

	if (both && ipos_new >= 0 && ipos_old >= 0){
		Mat Pfi = Peceff.inv();
		Mat Pbi = Pecefb.inv();
		Pecef = (Pfi + Pbi).inv();
		float * pPfi = Pfi.ptr<float>();
		float * pPbi = Pbi.ptr<float>();
		float _xf, _yf, _zf, _xb, _yb, _zb;
		_xf = pPfi[0] * xef + pPfi[1] * yef + pPfi[2] * zef;
		_yf = pPfi[3] * xef + pPfi[4] * yef + pPfi[5] * zef;
		_zf = pPfi[6] * xef + pPfi[7] * yef + pPfi[8] * zef;
		_xb = pPbi[0] * xeb + pPbi[1] * yeb + pPbi[2] * zeb;
		_yb = pPbi[3] * xeb + pPbi[4] * yeb + pPbi[5] * zeb;
		_zb = pPbi[6] * xeb + pPbi[7] * yeb + pPbi[8] * zeb;
		_xf += _xb;
		_yf += _yb;
		_zf += _zb;

		float * pPecef = Pecef.ptr<float>();
		xecef = pPecef[0] * _xf + pPecef[1] * _yf + pPecef[2] * _zf;
		yecef = pPecef[3] * _xf + pPecef[4] * _yf + pPecef[5] * _zf;
		zecef = pPecef[6] * _xf + pPecef[7] * _yf + pPecef[8] * _zf;
	}
	else{
		if (ipos_old >= 0){
			Pecef = Peceff;
			xecef = xef;
			yecef = yef;
			zecef = zef;
		}
		else{
			Pecef = Pecefb;
			xecef = xeb;
			yecef = yeb;
			zecef = zeb;
		}
	}
	eceftobih(xecef, yecef, zecef, lat, lon, alt);
	getwrldrot(lat, lon, Renu);
	lat *= (float)(180. / PI);
	lon *= (float)(180. / PI);

	unlock();

	return true;
}
bool ch_estate::get_vel(const long long t, const Mat & Qv, float & u, float & v,
	Mat & Pv, bool both)
{
	lock();
	int ivel_new = (cur_vel_opt + vel_opt.size() - 1) % vel_opt.size();
	int ivel_old = (ivel_new + num_vel_opt - 1) % vel_opt.size();
	long long tnew = vel_opt[ivel_new].t;
	long long told = vel_opt[ivel_old].t;
	if (tnew < t){
		ivel_old = ivel_new;
		ivel_new = -1;
	}
	else if (told > t){
		ivel_new = ivel_old;
		ivel_old = -1;
	}
	else{
		int n = num_vel_opt;
		while (n != 0)
		{
			int nmid = n >> 1;
			int ivel_mid = (ivel_old + nmid) % vel_opt.size();
			if (vel_opt[ivel_mid].t < t){
				ivel_old = ivel_mid;
				n -= nmid;
			}
			else{
				ivel_new = ivel_mid;
				n = nmid;
			}
		}
	}

	Mat Pvf, Pvb;
	float uf, vf, ub, vb;
	if (ivel_old >= 0){
		s_vel_opt & v = vel_opt[ivel_old];
		float dt = (float)((t - v.t) * (1. / (double)SEC));
		Pvf = v.Pv + dt * Qv;
		uf = v.u;
		vf = v.v;
	}

	if ((both && ivel_new >= 0) || ivel_old < 0){
		s_vel_opt & v = vel_opt[ivel_new];
		float dt = (float)((v.t - t) * (1. / (double)SEC));
		Pvb = v.Pv + dt * Qv;
		ub = v.u;
		uf = v.v;
	}

	if (both && ivel_new >= 0 && ivel_old >= 0){
		Mat Pfi = Pvf.inv();
		Mat Pbi = Pvb.inv();
		Pv = (Pfi + Pbi).inv();
		float * pPfi = Pfi.ptr<float>();
		float * pPbi = Pbi.ptr<float>();
		float _uf = (float)(pPfi[0] * uf + pPfi[1] * vf);
		float _vf = (float)(pPfi[2] * uf + pPfi[3] * vf);
		float _ub = (float)(pPbi[0] * ub + pPbi[1] * vb);
		float _vb = (float)(pPbi[2] * ub + pPbi[3] * vb);
		_uf += _ub;
		_vf += _vb;
		float * pPv = Pv.ptr<float>();
		u = (float)(pPv[0] * _uf + pPv[1] * _vf);
		v = (float)(pPv[2] * _uf + pPv[3] * _vf);
	}
	else{
		if (ivel_old >= 0){
			u = ub;
			v = vb;
			Pv = Pvb;
		}
		else{
			u = uf;
			v = vf;
			Pv = Pvf;
		}
	}
	unlock();
	return true;
}
