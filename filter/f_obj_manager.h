// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_obj_manager.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_obj_manager.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_obj_manager.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_OBJ_MANAGER_H_
#define _F_OBJ_MANAGER_H_

#include "f_base.h"
#include "../channel/ch_state.h"
#include "../channel/ch_obj.h"

// Description:
// The filter manages the objects, and shares objects between hosts efficiently.
// 
class f_obj_manager: public f_base
{
protected:
	ch_state * m_state;
	ch_ais_obj * m_ais_obj;
	ch_obj * m_obj;

	long long m_dtold;
	float m_range;

public:
	f_obj_manager(const char * name);
	virtual ~f_obj_manager();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif