#ifndef _CH_SCALAR_H_
#define _CH_SCALAR_H_
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
#include "ch_base.h"

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
};

class ch_sample: public ch_base
{
 protected:
  long long t, tf;
  int val, valf;
 public:
 ch_sample(const char * name):ch_base(name),val(0)
    {
    }

  void set(const long long _t, const int _val)
  {
    lock();
    t = _t;
    val = _val;
    unlock();
  }

  void get(long long & _t, int & _val)
  {
    lock();
    _t = t;
    _val = val;
    unlock();
  }
  virtual size_t get_dsize(){
    return sizeof(t) + sizeof(val);
  }

  virtual size_t write_buf(const char * buf){
    t = *((long long*) buf);
    val = *((int*) (((long long*) buf) + 1));
    return sizeof(t) + sizeof(val);
  }

   virtual size_t write_buf_f(const char * buf){
    tf = *((long long*) buf);
    valf = *((int*) (((long long*) buf) + 1));
    return sizeof(t) + sizeof(val);
  }
 
  virtual size_t read_buf(char * buf){
    *((long long*)buf) = t;
    *((int*)(((long long*)buf) + 1)) = val;
    return sizeof(t) + sizeof(val);
  }

  virtual void print(ostream & out){
    out << "channel " << m_name << " " << val << endl;
  }

  virtual int write(FILE * pf, long long tcur)
  {
    if (!pf)
      return 0;

    char buf[get_dsize()];

    lock();
    read_buf(buf);
    unlock();
    
    fwrite((void*)buf, sizeof(buf), 1, pf);
    
    return get_dsize();
  }
  
  virtual int read(FILE * pf, long long tcur)
  {
    if(!pf)
      return 0;

    if(tf <= tcur)
      set(tf, valf);
    else
      return 0;
    char buf[get_dsize()];

    int res = fread((void*)buf, sizeof(buf), 1, pf);

    if(res != get_dsize())
      return 0;

    lock();
    write_buf_f(buf);
    unlock();
  }

  virtual bool log2txt(FILE * pbf, FILE * ptf)
  {
    if(!pbf || !ptf)
      {
	cerr << "In ch_sample::log2txt(), File is not opened." << endl;
      }

    fprintf(ptf, "T, val\n"); 
    char buf[get_dsize()];
    while(!feof(pbf)){
      int res = fread((void*)buf, sizeof(buf), 1, pbf);
      if(!res){
	break;
      }

      write_buf(buf);
      fprintf(ptf, "%lld, %d\n", t, val);
    }
    return true;
  }
  
};
#endif
