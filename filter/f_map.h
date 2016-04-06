#ifndef _F_MAP_H_
#define _F_MAP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_map.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"


// load location related map, and save newly added data.
// connects ch_map
class f_map: public f_base
{
protected:
public:
	f_map(const char * name):f_base(name)
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif