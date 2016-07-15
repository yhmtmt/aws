// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_estate.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_estate.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_estate.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include "../channel/ch_state.h"
#ifndef _F_ESTATE_H_
#define _F_ESTATE_H_

class f_estate : public f_base
{
protected:
	ch_state * m_ch_state;
	ch_estate * m_ch_estate;
public:
	f_estate(const char * name);
	virtual ~f_estate();
	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif
