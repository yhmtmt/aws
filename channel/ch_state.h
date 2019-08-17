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
};

struct s_att_opt{
	long long t;
	float roll, pitch, yaw;
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


	int cur_att_opt;
	int num_att_opt;
	vector<s_att_opt> att_opt;
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
	
		for (int i = 0, ipos = (int)((cur_pos_opt + pos_opt.size() - 1) % pos_opt.size());
			i < num_pos_opt; i++, ipos = (int)((ipos + pos_opt.size() - 1) % pos_opt.size())){
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

	void set_att_opt(const long long t, const float & roll, const float & pitch, const float & yaw)
	{
		lock();
		s_att_opt & at = att_opt[cur_att_opt];
		at.t = t;
		at.roll = roll;
		at.pitch = pitch;
		at.yaw = yaw;
		cur_att_opt++;
		if (cur_att_opt == att_opt.size())
			cur_att_opt = 0;

		if (num_att_opt < att_opt.size())
			num_att_opt++;

		unlock();
	}

	bool get_att(const long long t, float & roll, float & pitch, float & yaw);
};

// state channel contains row sensor data.
class ch_state: public ch_base
{
 protected:
  long long tatt, tpos, talt, tvel, tdp;
  long long tattf, tposf, taltf, tvelf, tdpf;
  float roll, pitch, yaw; // roll(deg), pitch(deg), yaw(deg)
  float lon, lat, alt, galt; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
  float x, y, z; // ecef coordinate
  Mat R, Rret; // Rotation matrix for ENU transformation
  float cog, sog; // Course over ground(deg), Speed over ground (kts)
  float vx, vy;	 
  float nvx, nvy;
  float depth; // water depth
  long long m_tfile;
  float rollf, pitchf, yawf; // roll(deg), pitch(deg), yaw(deg)
  float lonf, latf, altf, galtf; // longitude(deg), latitude(deg), altitude(m), geoid altitude(m)
  float xf, yf, zf; // ecef coordinate
  float cogf, sogf; // Course over ground(deg), Speed over ground (kts)
  float depthf; // water depth
  
  // 9dof sensor calibrated
  long long t9dof;
  float mx, my, mz;
  float ax, ay, az;
  float gx, gy, gz;
  
  // 9dof sensor calibrated
  long long t9doff;
  float mxf, myf, mzf;
  float axf, ayf, azf;
  float gxf, gyf, gzf;
  
 public:
 ch_state(const char * name): ch_base(name), 
    m_tfile(0), tatt(0), tpos(0), tvel(0), tdp(0),
    tattf(0), tposf(0), tvelf(0), tdpf(0), 
    roll(0), pitch(0), yaw(0), lon(0), lat(0), alt(0), galt(0),
    x(0), y(0), z(0), cog(0), sog(0), depth(0),
    t9dof(0), mx(0), my(0), mz(0), ax(0), ay(0), az(0),
    gx(0), gy(0), gz(0)
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

  void set_9dof(const long long _t,
		const float _mx, const float _my, const float _mz,
		const float _ax, const float _ay, const float _az,
		const float _gx, const float _gy, const float _gz)
  {
    lock();
    t9dof = _t;
    mx = _mx;
    my = _my;
    mz = _mz;
    ax = _ax;
    ay = _ay;
    az = _az;
    gx = _gx;
    gy = _gy;
    gz = _gz;
    unlock();
  }
  
  void get_9dof(long long & _t, float & _mx, float & _my, float & _mz,
	  float & _ax, float & _ay, float & _az,
	  float & _gx, float & _gy, float & _gz)
  {
    lock();
    _t = t9dof;
    _mx = mx;
    _my = my;
    _mz = mz;
    _ax = ax;
    _ay = ay;
    _az = az;
    _gx = gx;
    _gy = gy;
    _gz = gz;
    
    unlock();
  }


  void set_position(const long long _tpos, const float _lat, const float _lon)
  {
    lock();
    tpos = _tpos;
    lat = _lat;
    lon = _lon;
    float lat_rad = (float)(lat * (PI / 180.)), lon_rad = (float)(lon * (PI / 180.));
    getwrldrot(lat_rad, lon_rad, R);
    bihtoecef(lat_rad, lon_rad, alt, x, y, z);
    unlock();
  }

  void set_alt(const long long _talt, const float _alt)
  {
    lock();
    talt = _talt;
    alt = _alt;
    unlock();
  }

  void get_alt(long long & _talt, float & _alt)
  {
    lock();
    _talt = talt;
    _alt = alt;
    unlock();
  }
  
  void set_velocity(const long long & _tvel, const float _cog, const float _sog)
  {
	  lock();
	  tvel = _tvel;
	  cog = _cog;
	  sog = _sog;
	  float th = (float)(cog * (PI / 180.));
	  nvx = (float)sin(th);
	  nvy = (float)cos(th);
	  float mps = (float)(sog * KNOT);
	  vx = (float)(mps * nvx);
	  vy = (float)(mps * nvy);
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

  void get_position(long long & _tpos, float & _lat, float & _lon, 
	  float & _x, float & _y, float & _z, Mat & Renu)
  {
    lock();
    _tpos = tpos;
    _lat = lat;
    _lon = lon;
    _x = x;
    _y = y;
    _z = z;
    R.copyTo(Renu);
    unlock();
  }

  void get_position(long long & _tpos, float & _lat, float & _lon)
  {
    lock();
    _tpos = tpos;
    _lat = lat;
    _lon = lon;
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

  void get_velocity_vector(long long & _tvel, float & _vx, float & _vy)
  {
    lock();
    _tvel = tvel;
    _vx = vx;
    _vy = vy;
    unlock();
  }

  void get_norm_velocity_vector(long long & _tvel, float & _nvx, float & _nvy)
  {
    lock();
    _tvel = tvel;
    _nvx = nvx;
    _nvy = nvy;
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
    return sizeof(long long) * 5 + sizeof(float) * 12 + sizeof(double) * 9;
  }
  
  virtual size_t write_buf(const char *buf);
  virtual size_t read_buf(char * buf);
  
  virtual void print(ostream & out)
  {
    out << "channel " << m_name << ":";
    out << "LATLON:" << lat << "," << lon;
    out << " RPY:" << roll << "," << pitch << "," << yaw;
    out << "cogsog:" << cog << "," << sog;
    out << endl;
  }
  
  virtual int write(FILE * pf, long long tcur);
  
  virtual int read(FILE * pf, long long tcur);
  
  virtual bool log2txt(FILE * pbf, FILE * ptf);
};


enum StatEng1{
    CheckEngine, OverTemperature, LowOilPressure, LowOilLevel, LowFuelPressure,
    LowSystemVoltage, LowCoolantLevel, WaterFlow, WaterInFuel, ChargeIndicator,
    PreheatIndicator, HighBoostPressure, RevLimitExceeded, EGRSystem,
    ThrottlePositionSensor, EmergencyStop
};

extern const char * strStatEng1[EmergencyStop+1];

enum StatEng2{
  WarningLevel1, WarningLevel2, PowerReduction, MaintenanceNeeded,
  EngineCommError, SuborSecondaryThrottle, NeutralStartProtect,
  EngineShuttingDown
};

extern const char * strStatEng2[EngineShuttingDown+1];

enum StatGear{
  Forward, Neutral, Reverse
};

extern const char * strStatGear[Reverse+1];

class ch_eng_state: public ch_base
{
 private:
  long long t, tf;

  long long trapid, trapidf;
  float rpm, rpmf; // x0.25
  unsigned char trim, trimf; // x1

  long long tdyn, tdynf;
  int poil, poilf; // oil pressure
  float toil, toilf; // oil temperature
  float temp, tempf; // Engine? temperature
  float valt, valtf; // alternator potential 
  float frate, fratef; // fuel rate (L/h)
  unsigned int teng, tengf; // total engine hour
  int pclnt, pclntf; // coolant pressure
  int pfl, pflf; // fuel pressure
  StatEng1 stat1, stat1f; // Status 1
  StatEng2 stat2, stat2f; // Status 2
  unsigned char ld, ldf; // load
  unsigned char tq, tqf; // torque

  long long ttran, ttranf;
  StatGear gear, gearf;
  int pgoil, pgoilf;
  float tgoil, tgoilf;

  long long ttrip, ttripf;
  int flused, flusedf; // fuel used
  float flavg, flavgf; // fuel rate average
  float fleco, flecof; // fuel rate eco
  float flinst, flinstf; // Instantaneous fuel rate
  
  char *buf;
   public:
 ch_eng_state(const char * name):ch_base(name), buf(NULL), tf(0)
  {
   buf = new char[get_dsize()];
  }
  ~ch_eng_state()
    {
    delete[] buf;
    }

  void set_rapid(const long long _t, const float _rpm, const unsigned char _trim)
  {
    lock();
    t = trapid = _t;
    rpm = _rpm;
    trim = _trim;
    unlock();
  }

  void get_rapid(long long & _t, float & _rpm, unsigned char & _trim)
  {
    lock();
    _t = trapid;
    _rpm = rpm;
    _trim = trim;
    unlock();
  }

  void set_dynamic(const long long _t, const int _poil, const float _toil,
		   const float _temp,  const float _valt, const float _frate,
		   const unsigned int _teng, const int _pclnt, const int _pfl,
		   const StatEng1 _stat1, const StatEng2 _stat2,
		   const unsigned char _ld, const unsigned char _tq)
  {
    lock();
    t = tdyn = _t;
    poil = _poil;
    toil = _toil;
    temp = _temp;
    valt = _valt;
    frate = _frate;
    teng = _teng;
    pclnt = _pclnt;
    pfl = _pfl;
    stat1 = _stat1;
    stat2 = _stat2;
    ld = _ld;
    tq = _tq;
    unlock();
  }

  void get_dynamic(long long & _t,  int & _poil,  float & _toil,
		   float & _temp, float & _valt,  float & _frate,
		    unsigned int & _teng,  int & _pclnt,  int & _pfl,
		    StatEng1 & _stat1,  StatEng2 & _stat2,
		    unsigned char & _ld, unsigned char & _tq)
  {
    lock();
    _t = tdyn;
    _poil = poil;
    _toil = toil;
    _temp = temp;
    _valt = valt;
    _frate = frate;
    _teng = teng;
    _pclnt = pclnt;
    _pfl = pfl;
    _stat1 = stat1;
    _stat2 = stat2;
    _ld = ld;
    _tq = tq;
    unlock();    
  }

  void set_tran(const long long _t, const StatGear _gear, const int _pgoil,
		const float _tgoil)
  {
    lock();
    t = ttran = _t;
    gear = _gear;
    pgoil = _pgoil;
    tgoil = _tgoil;
    unlock();
  }

  void get_tran(long long & _t, StatGear & _gear, int & _pgoil, float & _tgoil)
  {
    lock();
    _t = ttran;
    _gear = gear;
    _pgoil = pgoil;
    _tgoil = tgoil;
    unlock();
  }

  void set_trip(const long long _t, const int _flused, const float _flavg,
		const float _fleco, const float _flinst)
  {
    lock();
    t = ttrip = _t;
    flused = _flused;
    flavg = _flavg;
    fleco = _fleco;
    flinst = _flinst;
    unlock();
  }

  void get_trip(long long & _t, int & _flused, float & _flavg, float & _fleco,
		float & _flinst)
  {
    lock();
    _t = ttrip;
    _flused = flused;
    _flavg = flavg;
    _fleco = fleco;
    _flinst = flinst;
    unlock();
  }

    virtual size_t get_dsize(){
      return sizeof(long long)*5 + sizeof(float)*9 + sizeof(unsigned char)*3
	+ sizeof(int)*5 + sizeof(unsigned int) + sizeof(StatEng1)
	+ sizeof(StatEng2) + sizeof(StatGear);
    }

    virtual size_t write_buf_back(const char * buf);
    virtual size_t write_buf(const char * buf);
    virtual size_t read_buf(char * buf);
    virtual int write(FILE * pf, long long tcur);
    virtual int read(FILE * Pf, long long tcur);
    virtual void print(ostream & out);
    virtual bool log2txt(FILE * pbf, FILE * ptf);
};

class ch_env: public ch_base 
{
 private:
  long long t, tf;
  float baro, barof; // barometer
  float temp, tempf; // temperature
  float humd, humdf; // humidity
  float ilum, ilumf; // illuminance
  
 public:
 ch_env(const char * name):ch_base(name), t(0), tf(0), baro(0.f), temp(0.f), humd(0.f), ilum(0.f){}
  virtual ~ch_env(){}

  void set(const long long _t, const float _baro, const float _temp, 
	   const float _humd, const float _ilum){
    lock();
    t = _t;
    baro = _baro;
    temp = _temp;
    humd = _humd;
    ilum = _ilum;
    unlock();
  };

  void get(long long & _t, float & _baro, float & _temp, float & _humd, float &_ilum)
  {
    lock();
    _t = t;
    _baro = baro;
    _temp = temp;
    _humd = humd;
    _ilum = ilum;
    unlock();
  }

  virtual size_t get_dsize(){
    return sizeof(long long) + sizeof(float) * 4;
  }
  virtual size_t write_buf(const char * buf);
  virtual size_t read_buf(char * buf);
  virtual int write(FILE * pf, long long tcur);
  virtual int read(FILE * Pf, long long tcur);
  virtual void print(ostream & out);
  virtual bool log2txt(FILE * pbf, FILE * ptf);
};


// 8 channel voltage sensor outputs
class ch_volt : public ch_base
{
private:
  long long t, tf;
  float val[8];
  float valf[8];
public:
  ch_volt(const char * name) : ch_base(name)
  {
    for (int i = 0; i < 8; i++)
      val[i] = valf[i] = 0;
  }
  ~ch_volt()
  {
  }

  void set(const long long _t, const float * _val)
  {
    lock();
    t = _t;
    for (int iprobe = 0; iprobe < 8; iprobe++){
      val[iprobe] = _val[iprobe];
    }
    unlock();
  }

  const float get(const int iprobe)
  {
    lock();
    const float _val = val[iprobe];
    unlock();
    return _val;
  }

  virtual size_t get_dsize(){
    return sizeof(long long)+sizeof(float)* 8;
  }

  virtual size_t write_buf(const char * buf);
  virtual size_t read_buf(char * buf);
  virtual int write(FILE * pf, long long tcur);
  virtual int read(FILE * Pf, long long tcur);
  virtual void print(ostream & out);
  virtual bool log2txt(FILE * pbf, FILE * ptf);

};
#endif
