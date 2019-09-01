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
		vb = v.v;
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

bool ch_estate::get_att(const long long t, float & roll, float & pitch, float & yaw)
{
	lock();
	int iatt_new = (cur_att_opt + att_opt.size() - 1) % att_opt.size();
	int iatt_old = (iatt_new + num_att_opt - 1) % att_opt.size();
	long long tnew = att_opt[iatt_new].t;
	long long told = att_opt[iatt_old].t;
	if (tnew < t){
		iatt_old = iatt_new;
		iatt_new = -1;
	}
	else if (told > t){
		iatt_new = iatt_old;
		iatt_old = -1;
	}
	else{
		int n = num_att_opt;
		while (n != 0)
		{
			int nmid = n >> 1;
			int iatt_mid = (iatt_old + nmid) % att_opt.size();
			if (att_opt[iatt_mid].t < t){
				iatt_old = iatt_mid;
				n -= nmid;
			}
			else{
				iatt_new = iatt_mid;
				n = nmid;
			}
		}
	}

	if (iatt_old >= 0 && iatt_new > 0 && iatt_old != iatt_new){
		s_att_opt & att_new = att_opt[iatt_new];
		s_att_opt & att_old = att_opt[iatt_old];
		float a = (float)((float)(att_new.t - t) / (float)(att_new.t - att_old.t));
		float ia = 1.0 - a;

		roll = ia * att_new.roll - a * att_old.roll;
		pitch = ia * att_new.pitch - a * att_old.pitch;
		yaw = ia * att_new.yaw - a * att_old.yaw;

	}
	else if (iatt_old >= 0){
		s_att_opt & att_old = att_opt[iatt_old];
		roll = att_old.roll;
		pitch = att_old.pitch;
		yaw = att_old.yaw;
	}
	else{
		s_att_opt & att_new = att_opt[iatt_new];
		roll = att_new.roll;
		pitch = att_new.pitch;
		yaw = att_new.yaw;
	}
	unlock();
	return true;
}


///////////////////////////////////////////////////////////////////////// ch_state
size_t ch_state::write_buf(const char * buf)
{
	lock();
	const long long *lptr = (const long long*)buf;
	tpos = lptr[0];
	talt = lptr[1];
	tatt = lptr[2];
	tvel = lptr[3];
	tdp = lptr[4];
	twx = lptr[5];
	
	const float * ptr = (const float*)(lptr + 6);
	roll = ptr[0];
	pitch = ptr[1];
	yaw = ptr[2];
	lat = ptr[3];
	lon = ptr[4];
	alt = ptr[5];
	x = ptr[6];
	y = ptr[7];
	z = ptr[8];
	cog = ptr[9];
	sog = ptr[10];
	{
	  float th = (float)(cog * (PI / 180.));
	  nvx = (float)sin(th);
	  nvy = (float)cos(th);
	  float mps = (float)(sog * KNOT);
	  vx = (float)(mps * nvx);
	  vy = (float)(mps * nvy);
	}
	depth = ptr[11];
	bar = ptr[12];
	temp_air = ptr[13];
	hmdr = ptr[14];
	dew = ptr[15];
	dir_wnd_t = ptr[16];
	wspd_mps = ptr[17];
	
	const double * dptr = (const double*)(ptr + 18);
	memcpy((void*)R.data, (void*)dptr, sizeof(double)* 9);

	unlock();
	return get_dsize();
}

size_t ch_state::read_buf(char * buf)
{
	lock();
	long long * lptr = (long long *)buf;
	lptr[0] = tpos;
	lptr[1] = talt;
	lptr[2] = tatt;
	lptr[3] = tvel;
	lptr[4] = tdp;
	lptr[5] = twx;
	
	float * ptr = (float*)(lptr + 6);
	ptr[0] = roll;
	ptr[1] = pitch;
	ptr[2] = yaw;
	ptr[3] = lat;
	ptr[4] = lon;
	ptr[5] = alt;
	ptr[6] = x;
	ptr[7] = y;
	ptr[8] = z;
	ptr[9] = cog;
	ptr[10] = sog;
	ptr[11] = depth;
	ptr[12] =  bar;
	ptr[13] = temp_air;
	ptr[14] = hmdr;
	ptr[15] = dew;
	ptr[16] = dir_wnd_t;
	ptr[17] = wspd_mps;
	
	double * dptr = (double*)(ptr + 18);
	memcpy((void*)dptr, (void*)R.data, sizeof(double)* 9);
	unlock();
	return get_dsize();
}

int ch_state::write(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;
  
  int sz = 0;
  if (tdp <= tdpf && tvel <= tvelf && tatt <= tattf && tpos <= tposf
      && talt <= taltf){
    return sz;
  }
  
  sz = sizeof(long long)* 6;
	
  lock();
  fwrite((void*)&tcur, sizeof(long long), 1, pf);
  fwrite((void*)&tpos, sizeof(long long), 1, pf);

  fwrite((void*)&lat, sizeof(float), 1, pf);
  fwrite((void*)&lon, sizeof(float), 1, pf);
  tposf = tpos;
  
  fwrite((void*)&talt, sizeof(long long), 1, pf);	
  fwrite((void*)&alt, sizeof(float), 1, pf);

  sz += sizeof(float)* 3;

  fwrite((void*)&tatt, sizeof(long long), 1, pf);
  fwrite((void*)&roll, sizeof(float), 1, pf);
  fwrite((void*)&pitch, sizeof(float), 1, pf);
  fwrite((void*)&yaw, sizeof(float), 1, pf);
  tattf = tatt;
  sz += sizeof(float)* 3;

  fwrite((void*)&tvel, sizeof(long long), 1, pf);
  fwrite((void*)&cog, sizeof(float), 1, pf);
  fwrite((void*)&sog, sizeof(float), 1, pf);
  tvelf = tvel;
  sz += sizeof(float)* 2;
	
  fwrite((void*)&tdp, sizeof(long long), 1, pf);
  fwrite((void*)&depth, sizeof(float), 1, pf);
  tdpf = tdp;
  sz += sizeof(float);

  fwrite((void*)&twx, sizeof(long long), 1, pf);
  fwrite((void*)&bar, sizeof(float), 1, pf);
  fwrite((void*)&temp_air, sizeof(float), 1, pf);
  fwrite((void*)&hmdr, sizeof(float), 1, pf);
  fwrite((void*)&dew, sizeof(float), 1, pf);
  fwrite((void*)&dir_wnd_t, sizeof(float), 1, pf);
  fwrite((void*)&wspd_mps, sizeof(float), 1, pf);
  twxf = twx;
  sz += sizeof(float) * 6;
  
  m_tfile = tcur;
  unlock();
  return sz;
}

int ch_state::read(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;
  
  int sz = 0;
  if (tposf < tcur && tposf != tpos){
    set_position(tposf, latf, lonf);
  }
  
  if(taltf < tcur && taltf != talt){
    set_alt(taltf, altf);
  }
  
  if (tattf < tcur && tattf != tatt){
    set_attitude(tattf, rollf, pitchf, yawf);
  }
  
  if (tvelf < tcur && tvelf != tvel){
    set_velocity(tvelf, cogf, sogf);
  }
  
  if (tdpf < tcur && tdpf != tdp){
    set_depth(tdpf, depthf);
  }
  
  if (t9doff < tcur && t9doff != t9dof){
    set_9dof(t9doff, mxf, myf, mzf, axf, ayf, azf, gxf, gyf, gzf);
  }
  
  while (!feof(pf)){
    if (m_tfile > tcur){
      break;
    }
    
    lock();
    size_t res = 0;
    res += fread((void*)&m_tfile, sizeof(long long), 1, pf);
    
    res += fread((void*)&tposf, sizeof(long long), 1, pf);
    res += fread((void*)&latf, sizeof(float), 1, pf);
    res += fread((void*)&lonf, sizeof(float), 1, pf);

    res += fread((void*)&taltf, sizeof(long long), 1, pf);
    res += fread((void*)&altf, sizeof(float), 1, pf);
    
    res += fread((void*)&tattf, sizeof(long long), 1, pf);
    res += fread((void*)&rollf, sizeof(float), 1, pf);
    res += fread((void*)&pitchf, sizeof(float), 1, pf);
    res += fread((void*)&yawf, sizeof(float), 1, pf);
    
    res += fread((void*)&tvelf, sizeof(long long), 1, pf);
    res += fread((void*)&cogf, sizeof(float), 1, pf);
    res += fread((void*)&sogf, sizeof(float), 1, pf);
		
    res += fread((void*)&tdpf, sizeof(long long), 1, pf);
    res += fread((void*)&depthf, sizeof(float), 1, pf);

    res += fread((void*)&twxf, sizeof(long long), 1, pf);
    res += fread((void*)&barf, sizeof(float), 1, pf);
    res += fread((void*)&temp_airf, sizeof(float), 1, pf);
    res += fread((void*)&hmdrf, sizeof(float), 1, pf);
    res += fread((void*)&dewf, sizeof(float), 1, pf);
    res += fread((void*)&dir_wnd_tf, sizeof(float), 1, pf);
    res += fread((void*)&wspd_mpsf, sizeof(float), 1, pf);
    sz = res;
    unlock();
  }
  return sz;
}

bool ch_state::log2txt(FILE * pbf, FILE * ptf)
{
  fprintf(ptf, "t, tpos, talt, tatt, tvel, tdp, twx, lat, lon, alt, yaw, pitch, roll, cog, sog, depth, bar, temp_air, hmdr, dew, dir_wnd_t, wspd_mps\n");
  while (!feof(pbf)){
    long long t, tmax = 0;
	  
    size_t res = 0;
    res += fread((void*)&m_tfile, sizeof(long long), 1, pbf);
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&lat, sizeof(float), 1, pbf);
    res += fread((void*)&lon, sizeof(float), 1, pbf);
    tposf = tpos = t;
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&alt, sizeof(float), 1, pbf);
    taltf = talt = t;		
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&roll, sizeof(float), 1, pbf);
    res += fread((void*)&pitch, sizeof(float), 1, pbf);
    res += fread((void*)&yaw, sizeof(float), 1, pbf);
    tattf = tatt = t;
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&cog, sizeof(float), 1, pbf);
    res += fread((void*)&sog, sizeof(float), 1, pbf);
    
    tvelf = tvel = t;
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&depth, sizeof(float), 1, pbf);
    tdpf = tdp = t;
    
    res += fread((void*)&t, sizeof(long long), 1, pbf);
    res += fread((void*)&bar, sizeof(float), 1, pbf);
    res += fread((void*)&temp_air, sizeof(float), 1, pbf);
    res += fread((void*)&hmdr, sizeof(float), 1, pbf);
    res += fread((void*)&dew, sizeof(float), 1, pbf);
    res += fread((void*)&dir_wnd_t, sizeof(float), 1, pbf);
    res += fread((void*)&wspd_mps, sizeof(float), 1, pbf);        
    twxf = twx = t;
    
    fprintf(ptf, "%lld, %lld, %lld, %lld, %lld, %lld, %lld, %+013.8f, %+013.8f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f,%+06.1f,%+06.1f,%+06.1f,%+06.1f \n",
	    m_tfile, tpos, talt, tatt, tvel, tdp, twx, lat, lon, alt, yaw, pitch, roll, cog, sog, depth, bar, temp_air, hmdr, dew, dir_wnd_t, wspd_mps);
  }
  return true;
}

/////////////////////////////////////////////////////////////// ch_env

size_t ch_env::write_buf(const char * buf)
{
  lock();
  const long long * lptr = (const long long*)buf;
  t = lptr[0];

  const float * fptr = (const float*)(lptr + 1);
  baro = fptr[0];
  temp = fptr[1];
  humd = fptr[2];
  ilum = fptr[3];

  unlock();
  return get_dsize();
}

size_t ch_env::read_buf(char * buf)
{
  lock();
  long long * lptr = (long long*)buf;
  lptr[0] = t;
  
  float * fptr = (float*)(lptr + 1);
  fptr[0] = baro;
  fptr[1] = temp;
  fptr[2] = humd;
  fptr[3] = ilum;

  unlock();
  return get_dsize();
}

int ch_env::write(FILE * pf, long long tcur)
{
  if(!pf)
    return 0;

  if(t <= tf){
    return 0;
  }
  
  int sz = (int) get_dsize();
  
  lock();
  fwrite((void*)&tcur, sizeof(long long), 1, pf); // the time write operation executed
  fwrite((void*)&t, sizeof(long long), 1, pf); // the time sensor data acquired
  tf = t;
  fwrite((void*)&baro, sizeof(float), 1, pf);
  fwrite((void*)&temp, sizeof(float), 1, pf);
  fwrite((void*)&humd, sizeof(float), 1, pf);
  fwrite((void*)&ilum, sizeof(float), 1, pf);
  unlock();

  return sz + sizeof(long long);
}

int ch_env::read(FILE * pf, long long tcur)
{
  if(!pf)
    return 0;

  if(tf <= tcur && tf != t){
    set(tf, barof, tempf, humdf, ilumf);
    return (int)(get_dsize() + sizeof(long long));
  }

  // reading next data record
  int res;
  while (!feof(pf)){
    if(tf > tcur){
      break;
    }
    lock();
    res = fread((void*)&tf, sizeof(long long), 1, pf); // the time the data written is ignored (this line is only required to proceed file pointer.)
    res = fread((void*)&tf, sizeof(long long), 1, pf); 
    res = fread((void*)&barof, sizeof(float), 1, pf);
    res = fread((void*)&tempf, sizeof(float), 1, pf);
    res = fread((void*)&humdf, sizeof(float), 1, pf);
    res = fread((void*)&ilumf, sizeof(float), 1, pf);
    
    unlock();
  }
  
  return 0;
}

bool ch_env::log2txt(FILE * pbf, FILE * ptf)
{
  fprintf(ptf, "trec, tsens, baro, temp, humd, ilum¥n");
  int res;
  while(!feof(pbf)){
    res = fread((void*)&tf, sizeof(long long), 1, pbf); // the time the data written is ignored (this line is only required to proceed file pointer.)
    res = fread((void*)&t, sizeof(long long), 1, pbf); 
    res = fread((void*)&baro, sizeof(float), 1, pbf);
    res = fread((void*)&temp, sizeof(float), 1, pbf);
    res = fread((void*)&humd, sizeof(float), 1, pbf);
    res = fread((void*)&ilum, sizeof(float), 1, pbf);
    fprintf(ptf, "%lld, %lld, %06.1f, %03.1f, %03.1f, %05.1f\n", tf, t, baro, temp, humd, ilum);
  }
  return true;
}

void ch_env::print(ostream & out)
{
  cout << "t=" << t << " B=" << baro << " T=" << temp << " H=" << humd << " I=" << ilum << endl; 
}

//////////////////////////////////////////////////////////////////////////////////////// ch_volt

size_t ch_volt::write_buf(const char * buf)
{
  lock();
  const long long * lptr = (const long long*)buf;
  t = lptr[0];

  const float * fptr = (const float*)(lptr + 1);
  memcpy(val, fptr, sizeof(float)* 8);

  unlock();
  return get_dsize();
}

size_t ch_volt::read_buf(char * buf)
{
  lock();
  long long * lptr = (long long*)buf;
  lptr[0] = t;

  float * fptr = (float*)(lptr + 1);
  memcpy(fptr, val, sizeof(float)* 8);

  unlock();
  return get_dsize();
}

int ch_volt::write(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;

  if (t <= tf){
    return 0;
  }

  int sz = (int)get_dsize();

  lock();
  fwrite((void*)&tcur, sizeof(long long), 1, pf); // the time write operation executed
  fwrite((void*)&t, sizeof(long long), 1, pf); // the time sensor data acquired
  tf = t;
  fwrite((void*)val, sizeof(float), 8, pf);
  unlock();

  return sz + sizeof(long long);
}

int ch_volt::read(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;

  if (tf <= tcur && tf != t){
    set(tf, valf);
    return (int)(get_dsize() + sizeof(long long));
  }

  // reading next data record
  while (!feof(pf)){
    if (tf > tcur){
      break;
    }
    lock();
    int res;
    res = fread((void*)&tf, sizeof(long long), 1, pf); // the time the data written is ignored (this line is only required to proceed file pointer.)
    res = fread((void*)&tf, sizeof(long long), 1, pf);
    res = fread((void*)valf, sizeof(float), 8, pf);

    unlock();
  }

  return 0;
}

bool ch_volt::log2txt(FILE * pbf, FILE * ptf)
{
  fprintf(ptf, "trec, tsens,");
  for (int iprobe = 0; iprobe < 8; iprobe++){
    fprintf(ptf, ",v[%d]", iprobe);
  }
  fprintf(ptf, "\n");

  while (!feof(pbf)){
    int res;
    res = fread((void*)&tf, sizeof(long long), 1, pbf); // the time the data written is ignored (this line is only required to proceed file pointer.)
    res = fread((void*)&t, sizeof(long long), 1, pbf);
    res = fread((void*)val, sizeof(float), 8, pbf);
    fprintf(ptf, "%lld, %lld", tf, t);
    for (int iprobe = 0; iprobe < 8; iprobe++){
      fprintf(ptf, ",%03.2f", val[iprobe]);
    }
    fprintf(ptf, "\n");
  }
  return true;
}

void ch_volt::print(ostream & out)
{
  printf("trec=%lld, tsens=%lld", tf, t);
  for (int iprobe = 0; iprobe < 8; iprobe++){
    printf(",v[%d]=%03.2f", iprobe, val[iprobe]);
  }
  printf("/n");

}


//////////////////////////////////////////////////////////////////////////////////////// ch_eng_state

const char * strStatEng1[EmergencyStop+1] = {
    "Check Engine", "Over Temperature", "Low Oil Pressure", "Low Oil Level", "Low Fuel Pressure",
    "Low System Voltage", "Low Coolant Level", "Water Flow", "Water In Fuel", "Charge Indicator",
    "Preheat Indicator", "High Boost Pressure", "Rev Limit Exceeded", "EGR System",
    "Throttle Position Sensor", "Emergency Stop"
};;

const char * strStatEng2[EngineShuttingDown+1] = {
  "Warning Level1", "Warning Level2", "Power Reduction", "Maintenance Needed",
  "Engine Comm Error", "Subor Secondary Throttle", "Neutral Start Protect",
  "Engine Shutting Down"
};

const char * strStatGear[Reverse+1] = {
  "Forward", "Neutral", "Reverse"
};

size_t ch_eng_state::write_buf(const char * buf)
{
  lock();
  const long long * lptr = (const long long*)buf;
  t = lptr[0];
  trapid = lptr[1];
  tdyn = lptr[2];
  ttran = lptr[3];
  ttrip = lptr[4];
  
  const float * fptr = (const float*)(lptr + 5);
  rpm = fptr[0];
  toil = fptr[1];
  temp = fptr[2];
  valt = fptr[3];
  frate = fptr[4];
  tgoil = fptr[5];
  flavg = fptr[6];
  fleco = fptr[7];
  flinst = fptr[8];

  const unsigned char * ucptr = (const unsigned char*)(fptr + 9);
  trim = ucptr[0];
  ld = ucptr[1];
  tq = ucptr[2];
  
  const int * iptr = (const int *)(ucptr + 3);
  poil = iptr[0];
  pclnt = iptr[1];
  pfl = iptr[2];
  pgoil = iptr[3];
  flused = iptr[4];

  const unsigned int * uiptr = (const unsigned int*)(iptr + 5);
  teng = uiptr[0];

  const StatEng1 * se1ptr = (const StatEng1 *)(uiptr + 1);
  stat1 = *se1ptr;

  const StatEng2 * se2ptr = (const StatEng2 *)(se1ptr + 1);
  stat2 = *se2ptr;

  const StatGear * sgptr = (const StatGear *)(se2ptr +1);
  gear = *sgptr;

  unlock();
  return get_dsize();
}

size_t ch_eng_state::write_buf_back(const char * buf)
{
  lock();
  const long long * lptr = (const long long*)buf;
  tf = lptr[0];
  trapidf = lptr[1];
  tdynf = lptr[2];
  ttranf = lptr[3];
  ttripf = lptr[4];
  
  const float * fptr = (const float*)(lptr + 5);
  rpmf = fptr[0];
  toilf = fptr[1];
  tempf = fptr[2];
  valtf = fptr[3];
  fratef = fptr[4];
  tgoilf = fptr[5];
  flavgf = fptr[6];
  flecof = fptr[7];
  flinstf = fptr[8];

  const unsigned char * ucptr = (const unsigned char*)(fptr + 9);
  trimf = ucptr[0];
  ldf = ucptr[1];
  tqf = ucptr[2];
  
  const int * iptr = (const int *)(ucptr + 3);
  poilf = iptr[0];
  pclntf = iptr[1];
  pflf = iptr[2];
  pgoilf = iptr[3];
  flusedf = iptr[4];

  const unsigned int * uiptr = (const unsigned int*)(iptr + 5);
  tengf = uiptr[0];

  const StatEng1 * se1ptr = (const StatEng1 *)(uiptr + 1);
  stat1f = *se1ptr;

  const StatEng2 * se2ptr = (const StatEng2 *)(se1ptr + 1);
  stat2f = *se2ptr;

  const StatGear * sgptr = (const StatGear *)(se2ptr +1);
  gearf = *sgptr;

  unlock();
  return get_dsize();
}

size_t ch_eng_state::read_buf(char * buf)
{
  lock();
  long long * lptr = (long long*)buf;
  lptr[0] = t;
  lptr[1] =  trapid;
  lptr[2] = tdyn;
  lptr[3] = ttran;
  lptr[4] = ttrip;
  
  float * fptr = (float*)(lptr + 5);
  fptr[0] = rpm;
  fptr[1] = toil;
  fptr[2] = temp;
  fptr[3] = valt;
  fptr[4] = frate;
  fptr[5] = tgoil;
  fptr[6] = flavg;
  fptr[7] = fleco;
  fptr[8] = flinst;

  unsigned char * ucptr = (unsigned char*)(fptr + 9);
  ucptr[0] = trim;
  ucptr[1] = ld;
  ucptr[2] = tq;
  
  int * iptr = (int *)(ucptr + 3);
  iptr[0] = poil;
  iptr[1] = pclnt;
  iptr[2] = pfl;
  iptr[3] = pgoil;
  iptr[4] = flused;

  unsigned int * uiptr = (unsigned int*)(iptr + 5);
  uiptr[0] = teng;

  StatEng1 * se1ptr = (StatEng1 *)(uiptr + 1);
  *se1ptr = stat1;

  StatEng2 * se2ptr = (StatEng2 *)(se1ptr + 1);
  *se2ptr = stat2;

  StatGear * sgptr = (StatGear *)(se2ptr +1);
  *sgptr = gear;
  
  unlock();
  return get_dsize();
}

int ch_eng_state::write(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;

  if (t <= tf){
    return 0;
  }

  int sz = (int)get_dsize();

  tf = tcur;
  read_buf(buf);
  (*(long long*) buf) = tcur;
  fwrite(buf, sizeof(char), sz, pf);
  
  return sz + sizeof(long long);
}

int ch_eng_state::read(FILE * pf, long long tcur)
{
  if (!pf)
    return 0;

  if(trapidf <= tcur){
    set_rapid(trapidf, rpmf, trimf);
  }

  if(tdynf <= tcur){
    set_dynamic(tdynf, poilf, toilf, tempf,
		valtf, fratef, tengf, pclntf, pflf, stat1f, stat2f, ldf, tqf);
  }

  if(ttranf <= tcur){
    set_tran(ttranf, gearf, pgoilf, tgoilf);
  }

  if(ttripf <= tcur){
    set_trip(ttripf, flusedf, flavgf, flecof, flinstf);
  }

  // reading next data record
  size_t sz = get_dsize();
  
  while (!feof(pf)){
    if (tf > tcur){
      break;
    }

    int res;
    res = fread(buf, sizeof(char), sz, pf);
    write_buf_back(buf);
  }

  return 0;
}

bool ch_eng_state::log2txt(FILE * pbf, FILE * ptf)
{
  fprintf(ptf, "trec, trapid, tdyn, ttran, ttrip, rpm, trim, poil, toil, temp, valt, frate, teng, pclnt, pfl, stat1, stat2, ld, tq, gear, pgoil, tgoil, flused, flavg, fleco, flinst\n");
  size_t sz = get_dsize();
  while (!feof(pbf)){

    int res;
    res = fread(buf, sizeof(char), sz, pbf);
    write_buf(buf);

    fprintf(ptf, "%lld, %lld, %lld, %lld, %lld,", t, trapid, tdyn, ttran, ttrip);
    const char * str1, * str2, * str3;
    str1 = str2 = str3 = "null";
    
    if((int)stat1 <= (int)EmergencyStop && (int)stat1 >= 0){
      str1 = strStatEng1[stat1];
    }
    if((int)stat2 <= (int)EngineShuttingDown && (int)stat2 >= 0){
      str2 = strStatEng2[stat2];
    }
    if((int)gear <= (int)Reverse && (int) gear >= 0){
      str3 = strStatGear[gear];
    }
    
    fprintf(ptf, "%f, %d, %d, %03.2f, %02.1f, %02.2f, %03.2f, %u, %d, %d, %s, %s, %u, %u,",
	    rpm, (int)trim,
	    poil, toil, temp, valt, frate, teng, pclnt, pfl, str1,
	    str2, (unsigned int)ld, (unsigned int)tq);

    fprintf(ptf, "%s, %d, %03.2f,", str3, pgoil, tgoil);
    fprintf(ptf, "%d, %03.2f, %03.2f, %03.2f", flused, flavg, fleco, flinst);
    
    fprintf(ptf, "\n");
  }
  return true;
}

void ch_eng_state::print(ostream & out)
{
  out << "engstate rpm=" << rpm << endl;
}
