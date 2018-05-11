// Copyright(c) 2018 Yohei Matsumoto,  All right reserved. 

// f_router.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_router.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_router.h  If not, see <http://www.gnu.org/licenses/>. 


#ifndef _F_ROUTER_H_

#include "f_base.h"
#include "../channel/ch_state.h"
#include "../channel/ch_wp.h"
#include "../channel/ch_map.h"
class f_router : public f_base
{
protected:
	ch_state * ch_state;
	ch_wp * ch_wp;
	ch_map * ch_map;
	ch_route * ch_route;
public:
	f_router(const char * name);
	virtual ~f_router();

	bool init_run();
	void destroy_run();
	bool proc();
};

#endif
