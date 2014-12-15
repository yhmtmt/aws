// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_scalar.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_scalar.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_scalar.h.  If not, see <http://www.gnu.org/licenses/>. 

class ch_ptz: public ch_base
{
protected:
	unsigned short m_pn;
	short m_tt;
	unsigned short m_zm;
	bool m_update;
public:
	ch_ptz(const char * name): ch_base(name), m_pn(0), m_tt(0), m_zm(0), m_update(false)
	{};
	virtual ~ch_ptz()
	{}

	void set(unsigned short p, short t, unsigned short z){
		lock();
		m_pn = p;
		m_tt = t;
		m_zm = z;
		m_update = true;
		unlock();
	}

	bool get(unsigned short & p, short & t, unsigned short & z){
		if(!m_update)
			return false;
		lock();
		p = m_pn;
		t = m_tt;
		z = m_zm;
		m_update = false;
		unlock();
		return true;
	}

	virtual void tran()
	{
	}
};

class ch_ptzctrl: public ch_base
{
protected:
	unsigned short m_pn;
	unsigned short m_tt;
	unsigned short m_zm;
	bool m_bp, m_bt, m_bz;
public:
	ch_ptzctrl(const char * name): ch_base(name), m_pn(0), m_tt(0), m_zm(0),
		m_bp(false), m_bt(false), m_bz(false)
	{
	}

	virtual ~ch_ptzctrl()
	{
	}

	void inst_pan(unsigned short p)
	{
		lock();
		m_bp = true; m_pn = p;
		unlock();
	}

	void inst_tilt(unsigned short t)
	{
		lock();
		m_bt = true; m_tt = t;
		unlock();
	}

	void inst_zoom(unsigned short z)
	{
		lock();
		m_bz = true; m_zm = z;
		unlock();
	}

	bool is_pan(unsigned short & p){
		p = m_pn;
		if(!m_bp)
			return false;
		
		m_bp = false;
		return true;
	}

	bool is_tilt(unsigned short & t){
		t = m_tt;
		if(!m_bt)
			return false;
		m_bt = false;
		return true;
	}

	bool is_zoom(unsigned short & z){
		z = m_zm;
		if(!m_bz)
			return false;
		m_bz = false;
		return true;
	}

	virtual void tran()
	{
	}
};
