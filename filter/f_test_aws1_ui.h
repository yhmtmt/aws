// Copyright(c) 2018 Yohei Matsumoto, All right reserved. 

// f_test_aws1_ui.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_test_aws1_ui.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_aws1_ui.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_TEST_AWS1_UI_H_
#define _F_TEST_AWS1_UI_H_
#include "f_base.h"

#include "../channel/ch_image.h"
#include "../channel/ch_state.h"
#include "../channel/ch_aws1_ctrl.h"
#include "../channel/ch_aws1_sys.h"
#include "../channel/ch_map.h"
#include "../channel/ch_wp.h"
#include "../channel/ch_obj.h"

class f_test_aws1_ui: public f_base
{
 private:  
  ch_state * m_state;			// required
  ch_eng_state * m_engstate;            // optional
  ch_aws1_sys * m_ch_sys;		// is not used
  ch_aws1_ctrl_inst * m_ch_ctrl_inst;	// optional
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;   // optional
  ch_wp * m_ch_wp;	 	        // optional,
                                        //ref. update_route_cfg_box(),
                                        //     update_route(), add_waypoint()
  ch_map * m_ch_map;			// optional, ref. update_map()
  ch_obj * m_ch_obj;			// is not used
  ch_ais_obj * m_ch_ais_obj;		// optional, ref. update_ais_obj()
  ch_obst * m_ch_obst;			// is not used
  ch_aws1_ap_inst * m_ch_ap_inst;	// optional,
                                        // ref. update_ctrl_mode_box(),
                                        //      handle_ctrl_csr()
  ch_image_ref * m_ch_cam;
 public:
  f_test_aws1_ui(const char * name);
  ~f_test_aws1_ui();

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif
