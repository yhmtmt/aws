// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_sample.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_sample.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_sample.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_SAMPLE_H_
#define _F_SAMPLE_H_
#include "f_base.h"
#include "../channel/ch_scalar.h"

/////////////////////////////////////////////////////////// sample implementation of filter class

class f_sample: public f_base // 1) inherit from f_base
{
private:
  ch_sample * m_ch_sample;

	// 2) define variables used in this filter (you can define channel aliases)
	double m_f64par;
	long long m_s64par;
	unsigned long long m_u64par;
public:
	// 3) constructor should have a c-string as object name. then the object name should be passed to the f_base constructor.
 f_sample(const char * fname): f_base(fname), m_f64par(0.), m_s64par(0), m_u64par(0)
	{
		// 3-1)register variables to be accessed from consoles by calling register_fpar
	  register_fpar("ch_sample", (ch_base**)&m_ch_sample, typeid(ch_sample).name(), "Channel sample.");
		register_fpar("f64par", &m_f64par, "Double precision floating point number.");
		register_fpar("s64par", &m_s64par, "64 bit signed integer.");
		register_fpar("u64par", &m_u64par, "64 bit unsigned integer.");
	}

	virtual bool init_run(){
		// 4) override this function if you need to initialized filter class just before invoking fthread.
		//		open file or communication channels, set up and check channels and their aliases.
		return true;
	}

	virtual void detroy_run(){
		// override this function if you need to do something in stopping filter thread
	}

	virtual bool proc(){
		// 5) implement your filter body. this function is called in the loop of fthread.
		cout << get_time_str() << endl;
		if(is_pause()){
			cout << "Filter is puasing." << endl;
		}

		cout << " f64par:" << m_f64par << " s64par:" 
			<< m_s64par << " u64par:" << m_u64par << endl;
		int val;
		m_ch_sample->get(val);
		cout << "Channel val=" << val << endl;
		m_ch_sample->set((int) m_f64par);
		
		return true;
	}
};
// 6) you should jump toward f_base.cpp and add the line of creation code to f_base::create the factory function.
// 7) If you are the linux builder, the filter object should be added to the Makefile.
#endif
