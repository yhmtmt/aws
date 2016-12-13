#ifndef _CH_AWS3_H_
#define _CH_AWS3_H_
#include "ch_base.h"

// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_aws3.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_aws3.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_aws3.h.  If not, see <http://www.gnu.org/licenses/>. 

class ch_aws3_state: ch_base
{
private:
public:
	ch_aws3_state(const char * name) : ch_base(name)
	{
	}

	virtual ~ch_aws3_state()
	{
	}
};

class ch_aws3_cmd: ch_base
{
private:
public:
	ch_aws3_cmd(const char * name) : ch_base(name)
	{
	}

	virtual ~ch_aws3_cmd()
	{
	}
};

#endif
