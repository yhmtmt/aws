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
#include "../channel/ch_state.h"
#include "../channel/ch_map.h"

// load location related map, and save newly added data.
// connects ch_map
class f_map: public f_base
{
protected:
	char m_path[1024];
	char m_list[1024];
	ch_map * m_ch_map;

	struct s_cl_bb{
		Point3f bb[4];
		bool in_range;
	};

	list<s_cl_bb>  m_cl_bbs; // corners of the bounding box of the coast line
	list<vector<Point3f>*> m_cls; // coast lines

public:
	f_map(const char * name):f_base(name), m_ch_map(NULL)
	{
		m_path[0] = '.';
		m_path[1] = '\0';
		m_list[0] = '\0';

		register_fpar("path", m_path, 1024, "Path to map data.");
		register_fpar("list", m_list, 1024, "List file of maps.");
		register_fpar("ch_map", (ch_base**) m_ch_map, typeid(ch_map).name(), "Map channel.");
	}

	virtual ~f_map()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif