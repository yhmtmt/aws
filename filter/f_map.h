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
#include "../util/aws_map.h"
#include "../channel/ch_state.h"
#include "../channel/ch_map.h"

// load location related map, and save newly added data.
// connects ch_map
class f_map: public f_base
{
protected:
	AWSMap2::MapDataBase m_db;
	bool m_verb;
	char m_path[1024];

	char m_fdata[1024];
	
	enum e_dat_type{
		edt_jpjis = 0, edt_undef
	} m_dtype;
	static const char * m_str_dtype[edt_undef];

	enum e_map_cmd{
		emc_update = 0, emc_add_data, emc_set_pos, emc_render, emc_save, emc_check, emc_undef
	} m_cmd;
	static const char * m_str_cmd[emc_undef];

	ch_map * m_ch_map;
	unsigned int m_max_num_nodes;
	unsigned int m_max_total_size_layer_data;
	unsigned int m_max_size_layer_data[AWSMap2::lt_undef];

	bool update_channel();
	bool add_data();
	void render_data();
	void check_data();
	void set_pos();
	AWSMap2::LayerData * load_jpjis();

	double lat, lon, res, range;
public:
	f_map(const char * name);
	virtual ~f_map();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif