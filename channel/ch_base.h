#ifndef _CH_BASE_H_
#define _CH_BASE_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_base.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_base.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_base.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../command.h"
#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"

class f_base;
class ch_base;

typedef ch_base * (*CreateChannel)(const char * name);
template <class T> ch_base * createChannel(const char * name)
{
	return reinterpret_cast<ch_base*>(new T(name));
}

typedef map<const char *, CreateChannel, cmp> CHMap;

class ch_base
{
protected:
	static CHMap m_chmap;
	// calling register_factor<T>(tname) to register all filters to be used.
	static void register_factory();

	// registering filter classes to the factory
	template <class T> static void register_factory(const char * tname)
	{
		m_chmap.insert(CHMap::value_type(tname, createChannel<T>));
	}

public:
	// factory function
	static ch_base * create(const char * tname, const char * fname);

	// initialize mutex and signal objects. this is called by the c_aws constructor
	static void init();

	// uninitialize mutex and signal objects. called by the c_aws destoractor
	static void uninit();
protected:
	char * m_name;
	pthread_mutex_t m_mtx;
public:
	ch_base(const char * name):m_name(NULL){
		m_name = new char[strlen(name) + 1];
		strcpy(m_name, name);
		pthread_mutex_init(&m_mtx, NULL);
	};

	virtual ~ch_base(){
		delete [] m_name;
		pthread_mutex_destroy(&m_mtx);
	}
	
	void lock()
	{
		pthread_mutex_lock(&m_mtx);
	}
	
	void unlock()
	{
		pthread_mutex_unlock(&m_mtx);
	}
	
	const char * get_name(){ return m_name;};

	void get_info(s_cmd & rcmd, int ich)
	{
		snprintf(rcmd.get_ret_str(), RET_LEN, "%s %d", m_name, ich);
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t)
	{
		return true;
	}

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t)
	{
		return true;
	}
};

#endif