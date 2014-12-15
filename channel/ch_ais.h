// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_ais.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_ais.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_ais.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util//c_nmeadec.h"

class ch_ais: public ch_base
{
protected:
	int m_max_msg1;
	vector<c_vdm_msg1*> m_msg1; // message 1 to 3
	int m_max_msg5;
	vector<c_vdm_msg5*> m_msg5; // message 5
	int m_max_msg8;
	vector<c_vdm_msg8*> m_msg8;
	int m_max_msg18;
	vector<c_vdm_msg18*> m_msg18;
	int m_max_msg19;
	vector<c_vdm_msg19*> m_msg19;
	int m_max_msg24;
	vector<c_vdm_msg24*> m_msg24;

	int m_max_abk;
	vector<c_abk*> m_abk;
public:
	ch_ais(const char * name):ch_base(name), m_max_msg1(128), m_max_msg5(32),
		m_max_msg18(32), m_max_msg19(32), m_max_msg24(32), m_abk(2)
	{};

	virtual ~ch_ais(){};

	void push_msg1(c_vdm_msg1 * pmsg1)
	{
		lock();
		if(m_msg1.size() == (size_t) m_max_msg1){
			delete *m_msg1.begin();
			m_msg1.erase(m_msg1.begin());
		}

		m_msg1.push_back(pmsg1);
		unlock();
	}

	c_vdm_msg1 * pop_msg1()
	{
		if(m_msg1.size() == 0)
			return NULL;

		lock();
		c_vdm_msg1 * pmsg1;
		pmsg1 = m_msg1.back();
		m_msg1.pop_back();
		unlock();
		return pmsg1;
	}
	void push_msg5(c_vdm_msg5 * pmsg5)
	{
		lock();
		if(m_msg5.size() == (size_t) m_max_msg5){
			delete *m_msg5.begin();
			m_msg5.erase(m_msg5.begin());
		}

		m_msg5.push_back(pmsg5);
		unlock();
	}

	c_vdm_msg5 * pop_msg5()
	{
		if(m_msg5.size() == 0)
			return NULL;
		lock();
		c_vdm_msg5 * pmsg5;
		pmsg5 = m_msg5.back();
		m_msg5.pop_back();
		unlock();
		return pmsg5;
	}

	void push_msg8(c_vdm_msg8 * pmsg8)
	{
		lock();
		if(m_msg8.size() == (size_t) m_max_msg8){
			delete *m_msg8.begin();
			m_msg8.erase(m_msg8.begin());
		}

		m_msg8.push_back(pmsg8);
		unlock();
	}

	c_vdm_msg8 * pop_msg8()
	{
		if(m_msg8.size() == 0)
			return NULL;
		lock();
		c_vdm_msg8 * pmsg8;
		pmsg8 = m_msg8.back();
		m_msg8.pop_back();
		unlock();
		return pmsg8;
	}


	void push_msg18(c_vdm_msg18 * pmsg18)
	{
		lock();
		if(m_msg18.size() == (size_t) m_max_msg18){
			delete *m_msg18.begin();
			m_msg18.erase(m_msg18.begin());
		}

		m_msg18.push_back(pmsg18);
		unlock();
	}

	c_vdm_msg18 * pop_msg18()
	{
		if(m_msg18.size() == 0)
			return NULL;

		lock();
		c_vdm_msg18 * pmsg18;
		pmsg18 = m_msg18.back();
		m_msg18.pop_back();
		unlock();
		return pmsg18;
	}

	void push_msg19(c_vdm_msg19 * pmsg19)
	{
		lock();
		if(m_msg19.size() == (size_t) m_max_msg19){
			delete *m_msg19.begin();
			m_msg19.erase(m_msg19.begin());
		}

		m_msg19.push_back(pmsg19);
		unlock();
	}

	c_vdm_msg19 * pop_msg19()
	{
		if(m_msg19.size() == 0)
			return NULL;

		lock();
		c_vdm_msg19 * pmsg19;
		pmsg19 = m_msg19.back();
		m_msg19.pop_back();
		unlock();
		return pmsg19;
	}

	void push_msg24(c_vdm_msg24 * pmsg24)
	{
		lock();
		if(m_msg24.size() == (size_t) m_max_msg24){
			delete *m_msg24.begin();
			m_msg24.erase(m_msg24.begin());
		}

		m_msg24.push_back(pmsg24);
		unlock();
	}

	c_vdm_msg24 * pop_msg24()
	{
		if(m_msg24.size() == 0)
			return NULL;

		lock();
		c_vdm_msg24 * pmsg24;
		pmsg24 = m_msg24.back();
		m_msg24.pop_back();
		unlock();
		return pmsg24;
	}

	void push_abk(c_abk * pabk)
	{
		lock();
		if(m_abk.size() == (size_t) m_max_abk){
			delete *m_abk.begin();
			m_abk.erase(m_abk.begin());
		}

		m_abk.push_back(pabk);
		unlock();
	}

	c_abk * pop_abk()
	{
		if(m_abk.size() == 0)
			return NULL;

		lock();
		c_abk * pabk;
		pabk = m_abk.back();
		m_abk.pop_back();
		unlock();
		return pabk;
	}

	virtual void tran()
	{
	}
};



class ch_pvt: public ch_base
{
protected:
	// position
	bool m_bp;
	double m_lon_deg, m_lat_deg, m_alt;
	e_gp_dir m_lon_dir, m_lat_dir;

	// velosity, course
	bool m_bv;
	double m_vel, m_crs, m_crs_var;

	// time
	bool m_bt;
	short m_h, m_m;
	float m_s;

public:
	ch_pvt(const char * name):ch_base(name),m_bp(false), 
		m_bv(false), m_bt(false), m_h(0), m_m(0), m_s(0.) 
	{};

	virtual ~ch_pvt()
	{};

	void set_pos(double & lon, e_gp_dir lon_dir, 
		double & lat, e_gp_dir lat_dir, double alt, bool flag){
		lock();
		m_bp = flag;
		m_lon_deg = lon;
		m_lon_dir = lon_dir;
		m_lat_deg = lat;
		m_lat_dir = lat_dir;
		m_alt = alt;
		unlock();
	}

	void set_pos(double & lon, e_gp_dir lon_dir, 
		double & lat, e_gp_dir lat_dir, bool flag){
		lock();
		m_bp = flag;
		m_lon_deg = lon;
		m_lon_dir = lon_dir;
		m_lat_deg = lat;
		m_lat_dir = lat_dir;
		unlock();
	}

	void set_vel(double & vel, double & crs, 
		double & crs_var, bool flag){
		lock();
		m_bv = flag;
		m_vel = vel; 
		m_crs = crs;
		m_crs_var = crs_var;
		unlock();
	}

	void set_time(short h, short m, float s, bool flag)
	{
		lock();
		m_h = h;
		m_m = m;
		m_s = s;
		m_bt = flag;
		unlock();
	}

	bool get_pos(double & lon, e_gp_dir & lon_dir, 
		double & lat, e_gp_dir & lat_dir, double & alt, bool & bp){
		lock();
		lon = m_lon_deg;
		lat = m_lat_deg;
		lon_dir = m_lon_dir;
		lat_dir = m_lat_dir;
		alt = m_alt;
		bp = m_bp;
		m_bp = false;
		unlock();
		return m_bp;
	}

	bool get_vel(double & vel, double & crs, 
		double & crs_var, bool & bv){
		lock();
		vel = m_vel;
		crs = m_crs;
		crs_var = m_crs_var;
		bv = m_bv;
		m_bv = false;
		unlock();
		return m_bv;
	}

	bool get_time(short & h, short & m, float & s, bool & bt)
	{
		lock();
		h = m_h;
		m = m_m;
		s = m_s;
		bt = m_bt;
		m_bt = false;
		unlock();
		return m_bt;
	}

	virtual void tran()
	{
	}
};