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

class f_base;

class ch_base
{
protected:
	char * m_name;
	pthread_mutex_t m_mtx;
public:
	static ch_base * create(const char * type_name, const char * chan_name);

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

	virtual void tran() = 0;
};
