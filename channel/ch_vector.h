#ifndef _CH_VECTOR_H_
#define _CH_VECTOR_H_
#include "ch_base.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_vector.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_vector.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_vector.h.  If not, see <http://www.gnu.org/licenses/>. 

template<class T> class ch_vector: public ch_base
{
protected:
	vector<T*> m_objs;
	
public:
	ch_vector(const char * name):ch_base(name){};
	virtual ~ch_vector(){};
	void push(T * pobj){
		lock();
		m_objs.push_back(pobj);
		unlock();
	}

	T * pop(){
		T * pobj = NULL;
		if(m_objs.size()){
			lock();
			pobj = m_objs.back();
			m_objs.pop_back();
			unlock();
		}

		return pobj;
	}

	virtual void tran(){
		return;
	}
};

template<class T, int size = 1024> class ch_ring: public ch_base
{
protected:
	int m_size;
	int m_head, m_tail, m_num;
	T * m_buf;
public:
	ch_ring(const char * name):ch_base(name), m_buf(NULL), m_size(size), m_head(0), m_tail(0), m_num(0){
		m_buf = new T[m_size];
	}

	virtual ~ch_ring()
	{
	}

	int write(T * buf, int len){
		int i;
		lock();
		for(i = 0; m_num < m_size && i < len; m_num++, i++, m_tail = (m_tail + 1) % m_size){
			m_buf[m_tail] = buf[i];
		}
		unlock();
		return i;
	}

	int read(T * buf, int len){
		int i;
		lock();
		for(i = 0; m_num != 0 && i < len; m_num--, i++, m_head = (m_head + 1) % m_size){
			buf[i] = m_buf[m_head];
		}
		unlock();
		return i;
	}
};

#endif