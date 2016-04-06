#ifndef _CH_MAP_H_
#define _CH_MAP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_map.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"

// contains map information - multiple layered, dynamically updatable map.
// has insert, delete, configuration methods
class ch_map: public ch_base
{
protected:
public:
	ch_map(const char * name):ch_base(name)
	{
	}
};

#endif