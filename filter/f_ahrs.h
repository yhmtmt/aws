// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_ahrs.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ahrs.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ahrs.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_AHRS_H_
#define _F_AHRS_H_

#include "../util/aws_serial.h"
#include "f_base.h"

class f_ahrs: public f_base
{
protected:
	AWS_SERIAL m_hserial;
	char m_dname[1024];
	unsigned short m_port;
	unsigned int m_br;
	
public:
	f_ahrs(const char * name);
	virtual ~f_ahrs();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
