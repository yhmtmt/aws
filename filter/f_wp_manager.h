// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_wp_manager.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_wp_manager.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_wp_manager.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_WP_MANAGER_H_
#define _F_WP_MANAGER_H_

#include "f_base.h"
#include "../channel/ch_state.h"
#include "../channel/ch_wp.h"

// Description:
// The filter manages the objects, and shares objects between hosts efficiently.
// 
class f_wp_manager: public f_base
{
protected:
	enum e_cmd {
		cmd_ins, cmd_ers, cmd_save, cmd_load, cmd_next, cmd_prev, cmd_null
	} cmd;

	int id;
	double lat, lon, rarv, vel;

	static const char * str_cmd[cmd_null];

	const char * m_aws1_waypoint_file_version;
	char path[1024];
	ch_state * m_state;
	ch_wp * m_wp;

	void save(const int id);
	void load(const int id);
public:
	f_wp_manager(const char * name);
	virtual ~f_wp_manager();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif