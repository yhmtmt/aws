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

		char * p = new char [83 * size];
		if(!p){
			delete[] m_buf;
			m_buf = NULL;
			return false;
		}

		for(int i = 0; i < size; i++, p+=83){
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
		m_head++;
		m_head %= m_max_buf;
		unlock();
		return true;
	}

	bool push(const char * buf)
	{
		lock();
		int next_tail = (m_tail + 1) % m_max_buf;
		if(m_head == next_tail){
			m_head++;
			m_head %= m_max_buf;
		}
		char * p = m_buf[m_tail];
		for( ;*buf != '\0'; buf++, p++){
			*p = *buf;
		}
		*p = *buf;
		m_tail = next_tail;
		m_new_nmeas++;
		unlock();
		return true;
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t)
	{
		lock();
		unsigned long long ul = (unsigned long long) (m_new_nmeas);
		fout.write((const char *) &ul, sizeof(unsigned long long));
		int head = m_tail - m_new_nmeas;
		if(head < 0) 
			head += m_max_buf;

		for(int i = 0; i < (int) m_new_nmeas; i++){
			fout.write((const char *)m_buf[head], sizeof(char) * 83);
			head = (head + 1) % m_max_buf;
		}
		m_new_nmeas = 0;
		unlock();
		return true;
	}

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t)
	{
		lock();
		unsigned long long ul;
		fin.read((char *) &ul, sizeof(unsigned long long));
		for(int i = 0; i < (int) ul; i++){
			int next_tail = (m_tail + 1) % m_max_buf;
			fin.read((char*) m_buf[m_tail], sizeof(char) * 83);
			if(m_head == next_tail){
				m_head++;
				m_head %= m_max_buf;
			}else{
				m_tail = next_tail;
			}
		}
		
		unlock();
		return true;
	}

};

class ch_ship: public ch_base
{
private:

};

#endif
