#ifndef _F_AWS1_AP_H_
#define _F_AWS1_AP_H_
// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_aws1_ap.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_aws1_ap.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ap.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_base.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_state.h"

class f_aws1_ap: public f_base
{
protected:
	ch_state * m_state;
	ch_aws1_ctrl * m_ctrl_out;
	ch_aws1_ctrl * m_ctrl_in;
	s_aws1_ctrl_pars m_acp;
public:
	f_aws1_ap(const char * name);
	virtual ~f_aws1_ap();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

#endif