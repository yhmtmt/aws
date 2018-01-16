// Copyright(c) 2017 Yohei Matsumoto,  All right reserved. 

// f_test_vsrc.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_test_vsrc.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_test_vsrc.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef F_TEST_VSRC_H
#define F_TEST_VSRC_H

#include "f_base.h"
#include "../channel/ch_image.h"

class f_test_vsrc: public f_base
{
 protected:
  ch_image_ref * ch_img;
  int width, height;
  long long frm;
  e_imfmt fmt;
  double rdrop;
  bool verb;
 public:
  f_test_vsrc(const char * name);  
  ~f_test_vsrc();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
  
};
#endif
