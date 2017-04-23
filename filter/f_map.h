#ifndef _F_MAP_H_
#define _F_MAP_H_
// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

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
	enum e_map_cmd {
		emc_normal, emc_add, emc_release, emc_none
	} m_cmd;

	enum e_map_dtype {
		emd_jpgis, emd_none
	} m_dtype;

	static const char * m_cmd_str[emc_none];
	static const char * m_mdt_str[emd_none];

	// map's graph parameters
	int m_num_levels;
	double m_min_meter_per_pix;
	double m_min_step;
	double m_scale;
	unsigned int m_ncache;
	char m_fdata[1024];
	char m_path[1024];
	ch_map * m_ch_map;

	bool proc_normal();
	float m_range;
	Point3f m_point_prev;
	
	bool proc_add();
	bool proc_release();
public:
	f_map(const char * name);
	virtual ~f_map();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif