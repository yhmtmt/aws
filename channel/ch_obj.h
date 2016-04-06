#ifndef _CH_OBJ_H_
#define _CH_OBJ_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_obj.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_obj.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_obj.h.  If not, see <http://www.gnu.org/licenses/>.

#include "ch_base.h"

// contains recent object list, expected object list
// has insert, delete, and search method
class ch_obj: public ch_base
{
protected:
public:
	ch_obj(const char * name): ch_base(name)
	{
	}
};

#endif
