#ifndef _CH_STATE_H_
#define _CH_STATE_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_state.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_state.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_state.h.  If not, see <http://www.gnu.org/licenses/>.

#include "../util/aws_coord.h"

#include "ch_base.h"

struct s_pos_opt{
	long long t;
	float lat, lon, alt;
	float xecef, yecef, zecef;
	Mat Renu, Penu, Pecef;
	s_pos_opt():t(0), lat(0), lon(0), alt(0), xecef(0), yecef(0), zecef(0), Renu(), Penu(), Pecef(){}
	s_pos_opt(const s_pos_opt & p) :t(p.t), lat(p.lat), lon(p.lon), alt(p.alt), 
		xecef(p.xecef), yecef(p.yecef), zecef(p.zecef)
	{
		Renu = p.Renu.clone();
		Penu = p.Penu.clone();
		Pecef = p.Pecef.clone();
	}
	s_pos_opt(s_pos_opt & p) :t(p.t), lat(p.lat), lon(p.lon), alt(p.alt),
		xecef(p.xecef), yecef(p.yecef), zecef(p.zecef)
	{
		Renu = p.Renu.clone();
		Penu = p.Penu.clone();
		Pecef = p.Pecef.clone();
	}
};

struct s_vel_opt{
	long long t;
	float u, v, cog, sog;
	Mat Pv;
	s_vel_opt():t(0), u(0), v(0), cog(0), sog(0), Pv(){}
	s_vel_opt(const s_vel_opt & _v) :t(_v.t), u(_v.u), v(_v.v), cog(_v.cog), sog(_v.sog)
	{
		Pv = _v.Pv.clone();
	}
	s_vel_opt(s_vel_opt & _v) :t(_v.t), u(_v.u), v(_v.v), cog(_v.cog), sog(_v.sog)
	{
		Pv = _v.Pv.clone();
	}
};

// estimated state channel contains estimmated sensor data
class ch_estate : public ch_base
{
protected:
	int cur_pos_opt;
	int num_pos_opt;
	vector<s_pos_opt> pos_opt;


	int cur_vel_opt;
	int num_vel_opt;
	vector<s_vel_opt> vel_opt;

public:
	ch_estate(const char * name) :ch_base(name)
	{
		cur_pos_opt = num_pos_opt = 0;
		cur_vel_opt = num_vel_opt = 0;
		pos_opt.resize(100);
		vel_opt.resize(100);
	}
	virtual ~ch_estate()
	{
	}

	void set_pos_opt(const long long _t, const float _lat, const float _lon, const float _alt,
		const float _x, const float _y, const float _z, const Mat & Pecef, const Mat & Penu, const Mat & Renu)
	{
		lock();
		s_pos_opt & p = pos_opt[cur_pos_opt];
		p.t = _t;
		p.lat = _lat;
		p.lon = _lon;
		p.alt = _alt;
		p.xecef = _x;
		p.yecef = _y;
		p.zecef = _z;
		p.Pecef = Pecef.clone();
		p.Penu = Penu.clone();
		p.Renu = Renu.clone();
		cur_pos_opt++;
		if (cur_pos_opt == pos_opt.size())
			cur_pos_opt = 0;
		if (num_pos_opt < pos_opt.size())
			num_pos_opt++;

		unlock();
	}
	
	void get_pos_opt(vector<s_pos_opt> & _pos_opt)
	{
		lock();
		_pos_opt.resize(num_pos_opt);
	
		for (int i = 0, ipos = (cur_pos_opt + pos_opt.size() - 1) % pos_opt.size();
			i < num_pos_opt; i++, ipos = (ipos + pos_opt.size() - 1) % pos_opt.size()){
			_pos_opt[i].t = pos_opt[ipos].t;
			_pos_opt[i].alt = pos_opt[ipos].alt;
			_pos_opt[i].lat = pos_opt[ipos].lat;
			_pos_opt[i].lon = pos_opt[ipos].lon;
			_pos_opt[i].xecef = pos_opt[ipos].xecef;
			_pos_opt[i].yecef = pos_opt[ipos].yecef;
			_pos_opt[i].zecef = pos_opt[ipos].zecef;
			_pos_opt[i].Pecef = pos_opt[ipos].Pecef.clone();
			_pos_opt[i].Penu = pos_opt[ipos].Penu.clone();
			_pos_opt[i].Renu = pos_opt[ipos].Renu.clone();
		}
		unlock();
	}

	bool get_pos(const long long t, const Mat & Qv, const Mat & Qx,
		float & lat, float & lon, float & alt,
		float & xecef, float & yecef, float & zecef,
		Mat & Pecef, Mat & Renu, bool both = false);

	Mat calc_Pecef(const Mat & Renu, const Mat & Penu){
		Mat Pecef = Mat::zeros(3, 3, CV_32FC1);
		Mat Renuf;
		Renu.convertTo(Renuf, CV_32FC1);
		Pecef.at<float>(0, 0) = Penu.at<float>(0, 0);
		Pecef.at<float>(0, 1) = Penu.at<float>(0, 1);
		Pecef.at<float>(1, 0) = Penu.at<float>(1, 0);
		Pecef.at<float>(1, 1) = Penu.at<float>(1, 1);
		Pecef = Renuf.t() * Pecef * Renuf;
		return Pecef;
	}



	void set_vel_opt(const long long _t, const float _u, const float _v, 
		const float _cog, const float _sog, const Mat & Pv)
	{
		lock();
		s_vel_opt & v = vel_opt[cur_vel_opt];
		v.t = _t;
		v.u = _u;
		v.v = _v;
		v.cog = _cog;
		v.sog = _sog;
		v.Pv = Pv.clone();
		cur_vel_opt++;
		if (cur_vel_opt == vel_opt.size())
			cur_vel_opt = 0;

		if (num_vel_opt < vel_opt.size())
			num_vel_opt++;

		unlock();
	}

	bool get_vel(const long long t, const Mat & Qv, float & u, float & v, Mat & Pv, bool both = false);


};

// state channel contains row sensor data.
class ch_state: public ch_base
{
 protected:
	 long long tatt, tpos, tvel, tdp;
	 long long tattf, tposf, tvelf, tdpf;
	 float roll, pitch, yaw; // roll(deg), pitch(deg), yaw(deg)
	 float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	 float x, y, z; // ecef coordinate
	 Mat R, Rret; // Rotation matrix for ENU transformation
	 float cog, sog; // Course over ground(deg), Speed over ground (kts)
	 float depth; // water depth
	 long long m_tfile;
	 float rollf, pitchf, yawf; // roll(deg), pitch(deg), yaw(deg)
	 float lonf, latf, altf, galtf; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
	 float xf, yf, zf; // ecef coordinate
	 float cogf, sogf; // Course over ground(deg), Speed over ground (kts)
	 float depthf; // water depth

	 // 9dof sensor calibrated
	 long long t9dofc;
	 float mxc, myc, mzc;
	 float axc, ayc, azc;
	 float gxc, gyc, gzc;

	 // 9dof sensor raw
	 long long t9dofr;
	 float mxr, myr, mzr;
	 float axr, ayr, azr;
	 float gxr, gyr, gzr;

	 // 9dof sensor calibrated
	 long long t9dofcf;
	 float mxcf, mycf, mzcf;
	 float axcf, aycf, azcf;
	 float gxcf, gycf, gzcf;

	 // 9dof sensor raw
	 long long t9dofrf;
	 float mxrf, myrf, mzrf;
	 float axrf, ayrf, azrf;
	 float gxrf, gyrf, gzrf;

 public:
 ch_state(const char * name): ch_base(name), 
	 m_tfile(0),
	 tatt(0), tpos(0), tvel(0), tdp(0), tattf(0), tposf(0), tvelf(0), tdpf(0), 
	 roll(0), pitch(0), yaw(0),
    lon(0), lat(0), alt(0), galt(0), x(0), y(0), z(0), cog(0), sog(0), depth(0),
	t9dofc(0), mxc(0), myc(0), mzc(0), axc(0), ayc(0), azc(0), gxc(0), gyc(0), gzc(0),
	t9dofr(0), mxr(0), myr(0), mzr(0), axr(0), ayr(0), azr(0), gxr(0), gyr(0), gzr(0)
    {
		R = Mat::eye(3, 3, CV_64FC1);
    }

  void set_attitude(const long long _tatt, const float _r, const float _p, const float _y)
  {
    lock();
	tatt = _tatt;
    roll = _r; 
    pitch = _p;
    yaw = _y;
    unlock();
  }

  void set_9dof_calib(const long long _t, const float _mxc, const float _myc, const float _mzc,
	  const float _axc, const float _ayc, const float _azc, 
	  const float _gxc, const float _gyc, const float _gzc)
  {
	  lock();
	  t9dofc = _t;
	  mxc = _mxc;
	  myc = _myc;
	  mzc = _mzc;
	  axc = _axc;
	  ayc = _ayc;
	  azc = _azc;
	  gxc = _gxc;
	  gyc = _gyc;
	  gzc = _gzc;
	  unlock();
  }

  void get_9dof_calib(long long & _t, float & _mxc, float & _myc, float & _mzc,
	  float & _axc, float & _ayc, float & _azc,
	  float & _gxc, float & _gyc, float & _gzc)
  {
	  lock();
	  _t = t9dofc;
	  _mxc = mxc;
	  _myc = myc;
	  _mzc = mzc;
	  _axc = axc;
	  _ayc = ayc;
	  _azc = azc;
	  _gxc = gxc;
	  _gyc = gyc;
	  _gzc = gzc;

	  unlock();
  }

  void set_9dof_raw(const long long _t, const float _mxr, const float _myr, const float _mzr,
	  const float _axr, const float _ayr, const float _azr,
	  const float _gxr, const float _gyr, const float _gzr)
  {
	  lock();
	  t9dofr = _t;
	  mxr = _mxr;
	  myr = _myr;
	  mzr = _mzr;
	  axr = _axr;
	  ayr = _ayr;
	  azr = _azr;
	  gxr = _gxr;
	  gyr = _gyr;
	  gzr = _gzr;
	  unlock();
  }

  void get_9dof_raw(long long & _t, float & _mxr, float & _myr, float & _mzr,
	  float & _axr, float & _ayr, float & _azr,
	  float & _gxr, float & _gyr, float & _gzr)
  {
	  lock();
	  _t = t9dofr;
	  _mxr = mxr;
	  _myr = myr;
	  _mzr = mzr;
	  _axr = axr;
	  _ayr = ayr;
	  _azr = azr;
	  _gxr = gxr;
	  _gyr = gyr;
	  _gzr = gzr;

	  unlock();
  }

  void set_position(const long long _tpos, const float _lat, const float _lon, const float _alt, const float _galt)
  {
    lock();
	tpos = _tpos;
    lat = _lat;
    lon = _lon;
    alt = _alt;
	galt = _galt;
	float lat_rad = (float)(lat * (PI / 180.)), lon_rad = (float)(lon * (PI / 180.));
	getwrldrot(lat_rad, lon_rad, R);
	bihtoecef(lat_rad, lon_rad, alt, x, y, z);
    unlock();
  }

  void set_velocity(const long long & _tvel, const float _cog, const float _sog)
  {
	  lock();
	  tvel = _tvel;
	  cog = _cog;
	  sog = _sog;
	  unlock();
  }

  void set_depth(const long long & _tdp, const float _depth){
	  lock();
	  tdp = _tdp;
	  depth = _depth;
	  unlock();
  }

  void get_attitude(long long & _tatt, float & _r, float & _p, float & _y)
  {
    lock();
	_tatt = tatt;
    _r = roll;
    _p = pitch;
    _y = yaw;
    unlock();
  }

  void get_position(long long & _tpos, float & _lat, float & _lon, float & _alt, float & _galt,
	  float & _x, float & _y, float & _z, Mat & Renu)
  {
	  lock();
	  _tpos = tpos;
	  _lat = lat;
	  _lon = lon;
	  _alt = alt;
	  _galt = galt;
	  _x = x;
	  _y = y;
	  _z = z;
	  R.copyTo(Renu);
	  unlock();
  }

  void get_position(long long & _tpos, float & _lat, float & _lon, float & _alt, float & _galt)
  {
    lock();
	_tpos = tpos;
    _lat = lat;
    _lon = lon;
    _alt = alt;
	_galt = galt;
    unlock();
  }

  void get_position_ecef(long long & _tpos, float & _x, float & _y, float & _z)
  {
	  lock();
	  _tpos = tpos;
	  _x = x;
	  _y = y;
	  _z = z;
	  unlock();
  }

  const Mat & get_enu_rotation(long long & _tpos)
  {
	  lock();
	  _tpos = tpos;
	  R.copyTo(Rret);
	  unlock();
	  return Rret;
  }

  void get_velocity(long long & _tvel, float & _cog, float & _sog)
  {
	  lock();
	  _tvel = tvel;
	  _cog = cog;
	  _sog = sog;
	  unlock();
  }

  void get_depth(long long & _tdp, float & _depth)
  {
	  lock();
	  _tdp = tdp;
	  _depth = depth;
	  unlock();
  }

  virtual size_t get_dsize()
  {
    return sizeof(long long) * 4 + sizeof(float) * 13 + sizeof(double) * 9
		+ sizeof(long long) * 2 + sizeof(float) * 18;
  }
  
  virtual size_t write_buf(const char *buf)
  {
    lock();
	const long long *lptr = (const long long*) buf;
	tpos = lptr[0];
	tatt = lptr[1];;
	tvel = lptr[2];
	tdp = lptr[3];
	t9dofc = lptr[4];
	t9dofr = lptr[5];

    const float * ptr = (const float*) (lptr + 6);
    roll = ptr[0];
    pitch = ptr[1];
    yaw = ptr[2];
    lat = ptr[3];
    lon = ptr[4];
    alt = ptr[5];
    galt = ptr[6];
	x = ptr[7];
	y = ptr[8];
	z = ptr[9];
    cog = ptr[10];
    sog = ptr[11];
    depth = ptr[12];
	mxc = ptr[13];
	myc = ptr[14];
	mzc = ptr[15];
	axc = ptr[16];
	ayc = ptr[17];
	azc = ptr[18];
	gxc = ptr[19];
	gyc = ptr[20];
	gzc = ptr[21];
	mxr = ptr[22];
	myr = ptr[23];
	mzr = ptr[24];
	axr = ptr[25];
	ayr = ptr[26];
	azr = ptr[27];
	gxr = ptr[28];
	gyr = ptr[29];
	gzr = ptr[30];


	const double * dptr = (const double*)(ptr + 31);
	memcpy((void*)R.data, (void*) dptr, sizeof(double) * 9);

    unlock();
    return get_dsize();
  }

  virtual size_t read_buf(char * buf)
  {
    lock();
	long long * lptr = (long long *) buf;
	lptr[0] = tpos;
	lptr[1] = tatt;
	lptr[2] = tvel;
	lptr[3] = tdp;
	lptr[4] = t9dofc;
	lptr[5] = t9dofr;

    float * ptr =  (float*) (lptr + 4);
    ptr[0] = roll;
    ptr[1] = pitch;
    ptr[2] = yaw;
    ptr[3] = lat;
    ptr[4] = lon;
    ptr[5] = alt;
    ptr[6] = galt;
	ptr[7] = x;
	ptr[8] = y;
	ptr[9] = z;
    ptr[10] = cog;
    ptr[11] = sog;
    ptr[12] = depth;
	ptr[13] = mxc;
	ptr[14] = myc;
	ptr[15] = mzc;
	ptr[16] = axc;
	ptr[17] = ayc;
	ptr[18] = azc;
	ptr[19] = gxc;
	ptr[20] = gyc;
	ptr[21] = gzc;
	ptr[22] = mxr;
	ptr[23] = myr;
	ptr[24] = mzr;
	ptr[25] = axr;
	ptr[26] = ayr;
	ptr[27] = azr;
	ptr[28] = gxr;
	ptr[29] = gyr;
	ptr[30] = gzr;

	double * dptr = (double*) (ptr + 31);
	memcpy((void*)dptr, (void*) R.data, sizeof(double) * 9);
    unlock();
    return get_dsize();
  }
  
  virtual void print(ostream & out)
  {
    out << "channel " << m_name  <<  endl;
	out << "cal mxmymz axayaz gxgygz" << mxc << "," << myc << "," << mzc << " " << axc << "," << ayc << "," << azc << " " << gxc << "," << gyc << "," << gzc << endl;
	out << "raw mxmymz axayaz gxgygz" << mxr << "," << myr << "," << mzr << " " << axr << "," << ayr << "," << azr << " " << gxr << "," << gyr << "," << gzr << endl;
  }

  virtual int write(FILE * pf, long long tcur)
  {
	  if(!pf)
		  return 0;

	  int sz = 0;
	  if(tdp <= tdpf && tvel <= tvelf && tatt <= tattf && tpos <= tposf){
		  return sz;
	  }

	  sz = sizeof(long long) * 6;

	  lock();
	  fwrite((void*) &tcur, sizeof(long long), 1, pf);

	  fwrite((void*) &tpos, sizeof(long long), 1, pf);

	  fwrite((void*) &lat, sizeof(float), 1, pf);
	  fwrite((void*) &lon, sizeof(float), 1, pf);
	  fwrite((void*) &alt, sizeof(float), 1, pf);
	  fwrite((void*) &galt, sizeof(float), 1, pf);
	  tposf = tpos;
	  sz += sizeof(float) * 4;

	  fwrite((void*) &tatt, sizeof(long long), 1, pf);
	  fwrite((void*) &roll, sizeof(float), 1, pf);
	  fwrite((void*) &pitch, sizeof(float), 1, pf);
	  fwrite((void*) &yaw, sizeof(float), 1, pf);
	  tattf = tatt;
	  sz += sizeof(float) * 3;
	  
	  fwrite((void*) &tvel, sizeof(long long), 1, pf);
	  fwrite((void*) &cog, sizeof(float), 1, pf);
	  fwrite((void*) &sog, sizeof(float), 1, pf);
	  tvelf = tvel;
	  sz += sizeof(float) * 2;
	  
	  fwrite((void*) &tdp, sizeof(long long), 1, pf);
	  fwrite((void*) &depth, sizeof(float), 1, pf);
	  tdpf = tdp;
	  sz += sizeof(float);

	  fwrite((void*)&t9dofc, sizeof(long long), 1, pf);
	  fwrite((void*)&mxc, sizeof(float), 1, pf);
	  fwrite((void*)&myc, sizeof(float), 1, pf);
	  fwrite((void*)&mzc, sizeof(float), 1, pf);
	  fwrite((void*)&axc, sizeof(float), 1, pf);
	  fwrite((void*)&ayc, sizeof(float), 1, pf);
	  fwrite((void*)&azc, sizeof(float), 1, pf);
	  fwrite((void*)&gxc, sizeof(float), 1, pf);
	  fwrite((void*)&gyc, sizeof(float), 1, pf);
	  fwrite((void*)&gzc, sizeof(float), 1, pf);
	  fwrite((void*)&t9dofr, sizeof(long long), 1, pf);
	  fwrite((void*)&mxr, sizeof(float), 1, pf);
	  fwrite((void*)&myr, sizeof(float), 1, pf);
	  fwrite((void*)&mzr, sizeof(float), 1, pf);
	  fwrite((void*)&axr, sizeof(float), 1, pf);
	  fwrite((void*)&ayr, sizeof(float), 1, pf);
	  fwrite((void*)&azr, sizeof(float), 1, pf);
	  fwrite((void*)&gxr, sizeof(float), 1, pf);
	  fwrite((void*)&gyr, sizeof(float), 1, pf);
	  fwrite((void*)&gzr, sizeof(float), 1, pf);
	  sz += sizeof(float)* 18;

	  m_tfile = tcur;
	  unlock();
	  return sz;
  }

  virtual int read(FILE * pf, long long tcur)
  {
	  if(!pf)
		  return 0;

	  int sz = 0;
	  if(tposf < tcur && tposf != tpos){
		  lock();
		  lat = latf;
		  lon = lonf;
		  alt = altf;
		  galt = galtf;
		  float lat_rad = (float)(lat * (PI / 180.)), lon_rad = (float)(lon * (PI / 180.));
		  getwrldrot(lat_rad, lon_rad, R);
		  bihtoecef(lat_rad, lon_rad, alt, x, y, z);
		  unlock();
		  tpos = tposf;
	  }

	  if(tattf < tcur && tattf != tatt){
		  lock();
		  roll = rollf;
		  pitch = pitchf;
		  yaw = yawf;
		  tatt = tattf;
		  unlock();
	  }

	  if(tvelf < tcur && tvelf != tvel){
		  lock();
		  cog = cogf;
		  sog = sogf;
		  tvel = tvelf;
		  unlock();
	  }

	  if(tdpf < tcur && tdpf != tdp){
		  lock();
		  depth = depthf;
		  tdp = tdpf;
		  unlock();
	  }

	  if (t9dofcf < tcur && t9dofcf != t9dofc){
		  lock();
		  t9dofc = t9dofcf;
		  mxc = mxcf;
		  myc = mycf;
		  mzc = mzcf;
		  axc = axcf;
		  ayc = aycf;
		  azc = azcf;
		  gxc = gxcf;
		  gyc = gycf;
		  gzc = gzcf;
		  unlock();
	  }

	  if (t9dofrf < tcur && t9dofrf != t9dofr){
		  lock();
		  t9dofr = t9dofrf;
		  mxr = mxrf;
		  myr = myrf;
		  mzr = mzrf;
		  axr = axrf;
		  ayr = ayrf;
		  azr = azrf;
		  gxr = gxrf;
		  gyr = gyrf;
		  gzr = gzrf;
		  unlock();
	  }

	  while(!feof(pf)){
		  if(m_tfile > tcur){
			  break;
		  }

		  lock();
		  size_t res = 0;
		  res += fread((void*) &m_tfile, sizeof(long long), 1, pf);

		  res += fread((void*) &tposf, sizeof(long long), 1, pf);
		  res += fread((void*) &latf, sizeof(float), 1, pf);
		  res += fread((void*) &lonf, sizeof(float), 1, pf);
		  res += fread((void*) &altf, sizeof(float), 1, pf);
		  res += fread((void*) &galtf, sizeof(float), 1, pf);			

		  res += fread((void*) &tattf, sizeof(long long), 1, pf);
		  res += fread((void*) &rollf, sizeof(float), 1, pf);
		  res += fread((void*) &pitchf, sizeof(float), 1, pf);
		  res += fread((void*) &yawf, sizeof(float), 1, pf);
	  
		  res += fread((void*) &tvelf, sizeof(long long), 1, pf);
		  res += fread((void*) &cogf, sizeof(float), 1, pf);
		  res += fread((void*) &sogf, sizeof(float), 1, pf);

		  res += fread((void*) &tdpf, sizeof(float), 1, pf);
		  res += fread((void*) &depthf, sizeof(float), 1, pf);

		  res += fread((void*)&t9dofcf, sizeof(long long), 1, pf);
		  res += fread((void*)&mxcf, sizeof(float), 1, pf);
		  res += fread((void*)&mycf, sizeof(float), 1, pf);
		  res += fread((void*)&mzcf, sizeof(float), 1, pf);
		  res += fread((void*)&axcf, sizeof(float), 1, pf);
		  res += fread((void*)&aycf, sizeof(float), 1, pf);
		  res += fread((void*)&azcf, sizeof(float), 1, pf);
		  res += fread((void*)&gxcf, sizeof(float), 1, pf);
		  res += fread((void*)&gycf, sizeof(float), 1, pf);
		  res += fread((void*)&gzcf, sizeof(float), 1, pf);
		  res += fread((void*)&t9dofrf, sizeof(long long), 1, pf);
		  res += fread((void*)&mxrf, sizeof(float), 1, pf);
		  res += fread((void*)&myrf, sizeof(float), 1, pf);
		  res += fread((void*)&mzrf, sizeof(float), 1, pf);
		  res += fread((void*)&axrf, sizeof(float), 1, pf);
		  res += fread((void*)&ayrf, sizeof(float), 1, pf);
		  res += fread((void*)&azrf, sizeof(float), 1, pf);
		  res += fread((void*)&gxrf, sizeof(float), 1, pf);
		  res += fread((void*)&gyrf, sizeof(float), 1, pf);
		  res += fread((void*)&gzrf, sizeof(float), 1, pf);

		  sz = res;
		  unlock();
	  }
	  return sz;
  }

  virtual bool log2txt(FILE * pbf, FILE * ptf)
  {
	  fprintf(ptf, "t, tpos, tatt, tvel, tdp, lat, lon, alt, galt, yaw, pitch, roll, cog, sog, depth\n");
	  while(!feof(pbf)){
		  long long t, tmax = 0;

		  size_t res = 0;
		  res += fread((void*) &m_tfile, sizeof(long long), 1, pbf);
	
		  res += fread((void*) &t, sizeof(long long), 1, pbf);
		  res += fread((void*) &lat, sizeof(float), 1, pbf);
		  res += fread((void*) &lon, sizeof(float), 1, pbf);
		  res += fread((void*) &alt, sizeof(float), 1, pbf);
		  res += fread((void*) &galt, sizeof(float), 1, pbf);			
		  tposf = tpos = t;
	
		  res += fread((void*) &t, sizeof(long long), 1, pbf);
		  res += fread((void*) &roll, sizeof(float), 1, pbf);
		  res += fread((void*) &pitch, sizeof(float), 1, pbf);
		  res += fread((void*) &yaw, sizeof(float), 1, pbf);
		  tattf = tatt = t;
	  
		  res += fread((void*) &t, sizeof(long long), 1, pbf);
		  res += fread((void*) &cog, sizeof(float), 1, pbf);
		  res += fread((void*) &sog, sizeof(float), 1, pbf);

		  tvelf = tvel = t;
	
		  res += fread((void*) &t, sizeof(float), 1, pbf);
		  res += fread((void*) &depth, sizeof(float), 1, pbf);
		  tdpf = tdp = t;

		  res += fread((void*)&t9dofcf, sizeof(long long), 1, pbf);
		  res += fread((void*)&mxcf, sizeof(float), 1, pbf);
		  res += fread((void*)&mycf, sizeof(float), 1, pbf);
		  res += fread((void*)&mzcf, sizeof(float), 1, pbf);
		  res += fread((void*)&axcf, sizeof(float), 1, pbf);
		  res += fread((void*)&aycf, sizeof(float), 1, pbf);
		  res += fread((void*)&azcf, sizeof(float), 1, pbf);
		  res += fread((void*)&gxcf, sizeof(float), 1, pbf);
		  res += fread((void*)&gycf, sizeof(float), 1, pbf);
		  res += fread((void*)&gzcf, sizeof(float), 1, pbf);
		  res += fread((void*)&t9dofrf, sizeof(long long), 1, pbf);
		  res += fread((void*)&mxrf, sizeof(float), 1, pbf);
		  res += fread((void*)&myrf, sizeof(float), 1, pbf);
		  res += fread((void*)&mzrf, sizeof(float), 1, pbf);
		  res += fread((void*)&axrf, sizeof(float), 1, pbf);
		  res += fread((void*)&ayrf, sizeof(float), 1, pbf);
		  res += fread((void*)&azrf, sizeof(float), 1, pbf);
		  res += fread((void*)&gxrf, sizeof(float), 1, pbf);
		  res += fread((void*)&gyrf, sizeof(float), 1, pbf);
		  res += fread((void*)&gzrf, sizeof(float), 1, pbf);
		  t9dofc = t9dofcf;
		  t9dofr = t9dofrf;

		  fprintf(ptf, "%lld, %lld, %lld, %lld, %lld, %lld, %lld, %+013.8f, %+013.8f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f, %+06.1f,",
			  m_tfile, tpos, tatt, tvel, tdp, t9dofc, t9dofr, lat, lon, alt, galt, yaw, pitch, roll, cog, sog, depth);
		  fprintf(ptf, "%+04.4f, %+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,",
			  mxcf, mycf, mzcf, axcf, aycf, azcf, gxcf, gycf, gzcf);
		  fprintf(ptf, "%+04.4f, %+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,%+04.4f,",
			  mxrf, myrf, mzrf, axrf, ayrf, azrf, gxrf, gyrf, gzrf);

	  }
	  return true;
  }
};

#endif
