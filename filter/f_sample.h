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
  ch_sample * m_ch_sample_in, *m_ch_sample_out;

  // 2) define variables used in this filter (you can define channel aliases)
  double m_f64par;
  long long m_s64par;
  unsigned long long m_u64par;
  int val;
  bool increment;
 public:
  // 3) constructor should have an object name as a c-string. Then the object name should be passed to the f_base constructor.
 f_sample(const char * fname): f_base(fname),
    m_ch_sample_in(NULL), m_ch_sample_out(NULL), 
    m_f64par(0.), m_s64par(0), m_u64par(0), val(0), increment(false)
    {
      // 3-1) register variables to be accessed from consoles by calling register_fpar. These parameters are set via fset command.
      // 3-1-1) Channels can be registered as parameters. (Channels are the data object which can be shared with other filter objects.)
      register_fpar("ch_sample_in", (ch_base**)&m_ch_sample_in, typeid(ch_sample).name(), "Channel sample.");
      register_fpar("ch_sample_out", (ch_base**)&m_ch_sample_out, typeid(ch_sample).name(), "Channel sample.");
      
      // 3-1-2) Typical types are also supported.
      register_fpar("f64par", &m_f64par, "Double precision floating point number.");
      register_fpar("s64par", &m_s64par, "64 bit signed integer.");
      register_fpar("u64par", &m_u64par, "64 bit unsigned integer.");
      register_fpar("increment", &increment, "Enable incrementing val");
    }
  
  virtual bool init_run(){
    // 4) override this function if you need to initialized filter class just before invoking fthread.
    //		open file or communication channels, set up and check channels and their aliases.
    return true;
  }
  
  virtual void destroy_run(){
    // override this function if you need to do something in stopping filter thread
  }
  
  virtual bool proc(){
    // 5) implement your filter body. this function is called in the loop of fthread.
    cout << m_name << ":" << get_time_str() << endl;
    if(is_pause()){
      cout << "Filter is puasing." << endl;
    }
    
    cout << " f64par:" << m_f64par << " s64par:" 
	 << m_s64par << " u64par:" << m_u64par << endl;

    // Typical channels has setter/getter with mutual exclusion. 
    if(m_ch_sample_in){
      long long t;
      m_ch_sample_in->get(t, val);
      cout << "t=" << t << " val=" << val << endl;
    }
    if (increment)
      val++;
    
    if(m_ch_sample_out){
      m_ch_sample_out->set(get_time(), val);
    }
    
    return true;
  }
};
// 6) you should jump toward f_base.cpp and add the line of creation code to f_base::create the factory function.
// 7) If you are the linux builder, the filter object should be added to the Makefile.
#endif
