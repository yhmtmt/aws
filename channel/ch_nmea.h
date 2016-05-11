// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_nmea.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_nmea.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_nmea.h.  If not, see <http://www.gnu.org/licenses/>. 


#ifndef _CH_NMEA_H_
#define _CH_NMEA_H_
#include "ch_base.h"

class ch_nmea: public ch_base
{
protected:
	int m_max_buf;
	int m_head;
	int m_tail;
	char ** m_buf;
	unsigned int m_new_nmeas;

	bool alloc(int size){
		m_buf = new char * [size];
		if(!m_buf)
			return false;

		char * p = new char [84 * size];
		if(!p){
			delete[] m_buf;
			m_buf = NULL;
			return false;
		}

		for(int i = 0; i < size; i++, p+=84){
			m_buf[i] = p;
		}

		m_new_nmeas = 0;
		return true;
	}

	void release(){
		if(m_buf){
			if(m_buf[0])
				delete[] m_buf[0];
			delete[] m_buf;
		}
		m_buf = NULL;
	}

public:
	ch_nmea(const char * name): ch_base(name), m_max_buf(128), 
		m_head(0), m_tail(0), m_new_nmeas(0)
	{
		alloc(m_max_buf);
	}

	virtual ~ch_nmea()
	{
		release();
	}

	bool pop(char * buf)
	{
		lock();
		if(m_head == m_tail){
			unlock();
			return false;

		}
		char * p = m_buf[m_head];
		for(;*p != '\0'; p++, buf++){
			*buf = *p;
		}

		*buf = *p;
		m_head = (m_head + 1) % m_max_buf;

		unlock();
		return true;
	}

	bool push(const char * buf)
	{
		lock();
		char * p = m_buf[m_tail];
		int len = 0;
		for( ;*buf != '\0' && len < 84; buf++, p++, len++){
			*p = *buf;
		}

		if(len == 84){
			m_buf[m_tail][83] = '\0';
			cerr << "Error in " << m_name << "::push(const char*). No null character in the string given " << endl;
			cerr << "    -> string: " << m_buf[m_tail] << endl;
			unlock();
			return false;
		}

		*p = *buf;
		int next_tail = (m_tail + 1) % m_max_buf;
		if(m_head == next_tail){
			m_new_nmeas--;
			m_head = (m_head + 1) % m_max_buf;
		}
		m_tail = next_tail;

		m_new_nmeas++;
		unlock();
		return true;
	}
};

#endif
