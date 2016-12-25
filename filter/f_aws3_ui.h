
#ifndef _F_AWS3_UI_H_
#define _F_AWS3_UI_H_

#include "../util/aws_sock.h"
#include "../util/aws_stdlib.h"
#include "f_glfw_window.h"

#include "../channel/ch_aws3.h"
#include "../util/aws_jpad.h"

class f_aws3_ui : public f_glfw_window
{
private:
	ch_aws3_param * m_ch_param;
	ch_aws3_state * m_ch_state;
	ch_aws3_cmd * m_ch_cmd;

	bool m_verb;

	int m_js_id;
	s_jc_u3613m m_js;
public:
	f_aws3_ui(const char * name);

	virtual ~f_aws3_ui();

	virtual bool init_run();

	virtual void destroy_run();

	virtual bool proc();
};

#endif
