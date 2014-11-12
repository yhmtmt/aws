#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_aws is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_aws is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_aws.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
using namespace std;

#ifdef _WIN32
#include <direct.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "CmdAppBase/CmdAppBase.h"
#include "util/aws_sock.h"
#include "util/thread_util.h"

#include "util/c_clock.h"
#include "channel/ch_base.h"
#include "filter/f_base.h"
#include "command.h"
#include "c_aws.h"

const char * c_aws::m_str_cmd[CMD_UNKNOWN] = {
	"channel", "filter", "fcmd", "fset", "fget", 
	"go", "stop", "quit", "step","cyc","online",
	"pause", "syn", "clear", "rcmd", "trat" 
};

c_aws::c_aws(int argc, char ** argv):CmdAppBase(argc, argv),
	m_cmd_port(20000), m_working_path(NULL),
	m_bsync(true), m_bonline(true), m_exit(false),
	m_cycle_time(166667), m_time(0), m_time_zone_minute(540), m_time_rate(1)
{
	set_name_app("aws");
	set_version(0, 10);
	set_year_copy(2014);
	set_name_coder("Yohei Matsumoto");
	set_contact("matumoto (at) kaiyodai.ac.jp");

	add_arg("-port", "Port number the command processor uses.");
	add_val(&m_cmd_port, "int");

	add_arg("-wpath", "Path to the working directory.");
	add_val(&m_working_path, "string");

	add_arg("-tzm", "Time Zone in minutes.");
	add_val(&m_time_zone_minute, "int");
	f_base::init();
	// mutex for main thread and command thread
	pthread_mutex_init(&m_mtx, NULL);
	pthread_cond_init(&m_cnd_ret, NULL);
	pthread_cond_init(&m_cnd_none, NULL);
#ifdef _WIN32
	// initialize winsock2
	WSAStartup(MAKEWORD(2, 0), &m_wsad);

	// initialize COM interface
	HRESULT hr = CoInitialize(NULL);
	if(FAILED(hr))
		return;
#endif
}

c_aws::~c_aws()
{
	f_base::uninit();
	clear();

#ifdef WIN32
	CoUninitialize();
	WSACleanup();
#endif

	pthread_mutex_destroy(&m_mtx);
	pthread_cond_destroy(&m_cnd_ret);
	pthread_cond_destroy(&m_cnd_none);
}


void c_aws::clear()
{
	for(vector<f_base*>::iterator itr = m_filters.begin(); itr != m_filters.end(); itr++)
		delete (*itr);
	m_filters.clear();

	for(vector<ch_base*>::iterator itr = m_channels.begin(); itr != m_channels.end(); itr++)
		delete (*itr);
	m_channels.clear();
}

bool c_aws::push_command(const char * cmd_str, char * ret_str, 
	bool & ret_stat){
	pthread_mutex_lock(&m_mtx);
//	pthread_lock lock(m_mtx);
	/*
	while(m_cmd.stat != CS_NONE){
		pthread_cond_wait(&m_cnd_none, &m_mtx);
	}
	*/
	s_cmd & cmd = m_cmd;
	memset(m_cmd.ret, 0, RET_LEN);

	// split token
	int itok = 0;
	int total_len = 0;
	const char * ptr0 = cmd_str;
	char * ptr1 = m_cmd.mem;

	while(itok < CMD_ARGS){
		m_cmd.args[itok] = ptr1;
		while(*ptr0 == ' ' || *ptr0 == '\t') // skipping space
			ptr0++;

		int len = 0;
		int bq = 0;
		while(bq || *ptr0 != ' ' && *ptr0 != '\t' && *ptr0 != '\0'){ 
			if(*ptr0 == '['){
				bq++;
			}else if(*ptr0 == ']'){
				bq--;
			}

			if(total_len == CMD_LEN){
				ret_stat = false;
				return false;
			}

			*ptr1 = *ptr0;
			ptr0++;
			ptr1++;
			total_len++;
			len++;
		}

		if(len !=  0){
			if(total_len == CMD_LEN){
				ret_stat = false;
				return false;
			}

			*ptr1 = '\0'; // terminating the token
			ptr1++;
			total_len++;
			itok++;
		}

		if(*ptr0 == '\0') // break if the command string ends
			break;
		ptr0++;
	}

	if (itok == 0){ // no token.
		ret_stat = true;
		pthread_mutex_unlock(&m_mtx);
		return true;
	}
	cmd.num_args = itok;

	// decoding command type
	cmd.type = CMD_UNKNOWN;
	for(int icmd = 0; icmd < (int) CMD_UNKNOWN; icmd++){
		if(strcmp(m_cmd.args[0], m_str_cmd[icmd]) == 0){
			cmd.type = (e_cmd) icmd;
			break;
		}
	}

	if(cmd.type == CMD_UNKNOWN){
		cerr << "Unknown command." << endl;
		pthread_mutex_unlock(&m_mtx);
		return false;
	}

	// waiting for the command processed
	m_cmd.stat = CS_SET;
	while(m_cmd.stat != CS_RET && m_cmd.stat != CS_ERR){
		pthread_cond_wait(&m_cnd_ret, &m_mtx);
	}

	memcpy(ret_str, m_cmd.ret, RET_LEN);

	if(m_cmd.stat == CS_ERR)
		ret_stat = false;
	else
		ret_stat = true;

	m_cmd.stat = CS_NONE;

//pthread_cond_signal(&m_cnd_none);
	pthread_mutex_unlock(&m_mtx);
	return true;
}

///////////////////////// command handler
// handle_stop stops all the filters in the graph
bool c_aws::handle_stop()
{
	bool stopped = false;
	while(!stopped){
		stopped = true;
		for(vector<f_base*>::iterator fitr = m_filters.begin(); fitr != m_filters.end(); fitr++)
			stopped = stopped && (*fitr)->stop();
		f_base::clock(-1);
	}
	return true;
}

// handle_run runs all the filters in the graph.
bool c_aws::handle_run(s_cmd & cmd)
{
	f_base::set_tz(m_time_zone_minute);

	if(f_base::m_clk.is_run()){
		cout << "AWS is running." << endl;
		return false;
	}

	if(f_base::m_clk.is_pause()){ // pause to run transition happens
		// in pause mode only 
		if(cmd.num_args == 3){
			if(cmd.args[1][0] != 't' || cmd.args[1][1] != 'o'){
				sprintf(cmd.get_ret_str(), "In pause state, \"go [to <later absolute time>]\"");
				return false;
			}
			tmex tm;
			if(decTmStr(cmd.args[2], tm))
				m_end_time = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
			else
				m_end_time = (long long) atol(cmd.args[1]) * (long long) SEC;
		}else{
			m_end_time = LLONG_MAX;
		}

		if(m_end_time <= f_base::m_clk.get_time()){
			sprintf(cmd.get_ret_str(), "Time should be later than current time.");
			return false;
		}

		return f_base::m_clk.restart();
	}

	if(!check_graph()){
		sprintf(cmd.get_ret_str(), "Failed to run filter graph.");
		return false;
	}

	// if the starting time is given the time is decoded and stored into m_start_time 
	if(cmd.num_args >= 2){
		tmex tm;
		if(decTmStr(cmd.args[1], tm))
			m_start_time = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
		else
			m_start_time = (long long) atol(cmd.args[1]) * (long long) SEC;
	}else{
		m_start_time = (long long) time(NULL) * SEC; 
	}

	// ifthe end time is given, decoded and stored into m_end_time.
	if(cmd.num_args >= 3){
		tmex tm;
		if(decTmStr(cmd.args[2], tm))
			m_end_time = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
		else
			m_end_time = (long long) atol(cmd.args[2]) * (long long) SEC;
	}else{
		m_end_time = LLONG_MAX;
	}

	f_base::m_clk.start((unsigned) m_cycle_time, 
		(unsigned) m_cycle_time, m_start_time, m_bonline, m_time_rate);
	m_time = f_base::m_clk.get_time();

	f_base::init_run_all();
	f_base::clock(m_start_time);

	// check filter's status. 
	for(vector<f_base*>::iterator fitr = m_filters.begin(); fitr != m_filters.end(); fitr++){
		if(!(*fitr)->run(m_start_time, m_end_time)){
			snprintf(cmd.get_ret_str(),  RET_LEN, "Error in starting filter %s.", (*fitr)->get_name());
			return false;
		}
	}

	return true;
}

void c_aws::proc_command()
{
	// message loop for the system window
#ifdef _WIN32
	MSG uMsg;
	while(PeekMessage(&uMsg, NULL, 0, 0, PM_REMOVE)){
		TranslateMessage(&uMsg);
		DispatchMessage(&uMsg);
		if(uMsg.message == WM_QUIT){
			m_exit = true;
		}
	}
#endif
	f_base::flush_err_buf();

	if(m_cmd.stat != CS_SET) // no command
		return;
	{
		pthread_mutex_lock(&m_mtx);
		//pthread_lock lock(m_mtx);

		s_cmd & cmd = m_cmd;
		bool result = false;
		switch(cmd.type){
		case CMD_CHAN:
			if(!(result = add_channel(cmd)))
				sprintf(cmd.get_ret_str(), "Failed to create channel %s of %s.", cmd.args[2], cmd.args[1]);
			break;
		case CMD_FLTR:
			if(!(result = add_filter(cmd)))
				sprintf(cmd.get_ret_str(), "Failed to create filter %s of %s.",  cmd.args[2], cmd.args[1]);
			break;
		case CMD_FCMD:
			{
				f_base * pfilter = get_filter(cmd.args[1]);
				if(pfilter == NULL){
					sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
					result = false;
				}else{
					pfilter->lock_cmd(true);
					if(!pfilter->cmd_proc(m_cmd)){
						result = false;
					}else{
						result = true;
					}
					pfilter->unlock_cmd(true);
				}
			}
			break;
		case CMD_FSET:
			{
				f_base * pfilter = get_filter(cmd.args[1]);
				if(pfilter == NULL){
					sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
					result = false;
				}else{
					pfilter->lock_cmd(true);
					if(!pfilter->set_par(cmd)){
						result = false;
					}else{
						result = true;
					}
					pfilter->unlock_cmd(true);
				}
			}
			break;
		case CMD_FGET:
			{
				f_base * pfilter = get_filter(cmd.args[1]);
				if(pfilter == NULL){
					sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
					result = false;
				}else{
					pfilter->lock_cmd(true);
					if(!pfilter->get_par(cmd)){
						result = false;
					}else{
						result = true;
					}
					pfilter->unlock_cmd(true);
				}
			}
			break;
		case CMD_GO:
			handle_run(cmd);
			result = true;
			break;
		case CMD_STOP:
			handle_stop();
			result = true;
			break;
		case CMD_QUIT:
			m_exit = true;
			result = true;
			break;
		case CMD_STEP:
			if(!f_base::m_clk.is_pause()){
				sprintf(cmd.get_ret_str(), "Step can only  be used in pause state.");
				result = false;
			}else{
				// parsing argument 
				int cycle;
				if(cmd.num_args == 1){
					cycle = 1;
					f_base::m_clk.step(cycle);
					result = true;
				}else if(cmd.num_args == 2){
					// step to absolute time
					long long tabs;
					tmex tm;
					if(decTmStr(cmd.args[1], tm)){
						tabs = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
					}else{
						tabs = (long long) atol(cmd.args[1]) * (long long) SEC;
					}
					f_base::m_clk.step(tabs);
					result = true;
				}else if(cmd.num_args == 3){
					if(cmd.args[1][0] != 'c'){
						result = false;
					}else{
						cycle = atoi(cmd.args[2]);
						f_base::m_clk.step(cycle);
						result = true;
					}
				}
			}
			break;
		case CMD_CYC:
			if(!f_base::m_clk.is_stop()){
				cout << "stop:" << f_base::m_clk.is_stop() << endl;
				cout << "run:" << f_base::m_clk.is_run() << endl;
				cout << "pause:" << f_base::m_clk.is_pause() << endl;

				sprintf(cmd.get_ret_str(), "Cycle cannot be changed during execution");
				result = false;
			}else{
				m_cycle_time = (long long) (atof(cmd.args[1]) * 1e7);
				result = true;
			}
			break;
		case CMD_ONLINE:
			if(!f_base::m_clk.is_stop()){
				sprintf(cmd.get_ret_str(), "Online/offline mode cannot be changed during execution.");
				result = false;
			}else{
				if(strcmp(cmd.args[1], "yes") == 0)
					m_bonline = true;
				else
					m_bonline = false;
				result = true;
			}
			break;
		case CMD_PAUSE:
			if(!f_base::m_clk.is_run() || m_bonline){
				sprintf(cmd.get_ret_str(), "Pause command should be used in run state.");
				result = false;
			}else{
				if(!f_base::m_clk.pause()){
					result = false;
				}else{
					result = true;
				}
			}
			break;
		case CMD_SYN:
			if(!f_base::m_clk.is_stop()){
				sprintf(cmd.get_ret_str(), "Synchronization status cannot be changed during execution.");
				result = false;
			}else{
				if(strcmp(cmd.args[1], "yes") == 0)
					m_bsync = true;
				else
					m_bsync = false;
				result = true;
			}
			break;
		case CMD_CLEAR:
			if(!f_base::m_clk.is_run()){
				sprintf(cmd.get_ret_str(), "Graph cannot be cleared during execution.");
				result = false;
			}else{
				clear();
				result = true;
			}
			break;
		case CMD_RCMD:
			if(cmd.num_args == 2){	
				c_rcmd * pcmd = new c_rcmd(this, atoi(cmd.args[1]));
				if(!pcmd->is_exit()){
					m_rcmds.push_back(pcmd);
					result = true;
				}else{
					delete pcmd;
				}
			}
			break;
		case CMD_TRAT:
			if(!f_base::m_clk.is_stop()){
				sprintf(cmd.get_ret_str(), "Trat cannot be changed during execution");
				result = false;
			}else{
				m_time_rate = (int) atoi(cmd.args[1]);
				result = true;
			}
			break;
		}

		if(!result){
			m_cmd.stat = CS_ERR;
		}else{
			m_cmd.stat = CS_RET;
		}

		m_cmd.set_ret_stat(result);
		pthread_cond_signal(&m_cnd_ret);
		pthread_mutex_unlock(&m_mtx);
	}
}

ch_base * c_aws::get_channel(const char * name)
{
	for(vector<ch_base*>::iterator itr = m_channels.begin();
		itr != m_channels.end(); itr++){
		const char * cname = (*itr)->get_name();
		if(strcmp(cname, name) == 0)
			return *itr;
	}
	return NULL;
}

f_base * c_aws::get_filter(const char * name)
{
	for(vector<f_base*>::iterator itr = m_filters.begin();
		itr != m_filters.end(); itr++){
		const char * fname = (*itr)->get_name();
		if(strcmp(fname, name) == 0)
			return *itr;
	}
	return NULL;
}


bool c_aws::add_channel(s_cmd & cmd)
{
	char ** tok = cmd.args;
	int itok = 1;
	if(get_channel(tok[itok+1]) != NULL){
		cerr << "Cannot register channels with same name " << tok[itok+1] << "." << endl;
		return false;
	}

	ch_base * pchan = ch_base::create(tok[itok], tok[itok+1]);
	if(pchan == NULL)
		return false;
	m_channels.push_back(pchan);
	return true;
}

bool c_aws::add_filter(s_cmd & cmd){
	char ** tok = cmd.args;
	int itok = 1;

	if(get_filter(tok[itok+1]) != NULL){
		cerr << "Cannot register filters with same name " << tok[itok + 1] << "." << endl;
		return false;
	}
	f_base * pfilter = f_base::create(tok[itok], tok[itok+1]);

	if(pfilter == NULL){
		cerr << "Failed to add filter " << tok[itok] << endl;
		return false;
	}

	try{
		itok = 3;
		if(strcmp(tok[itok], "-i") != 0){
			sprintf(cmd.get_ret_str(), "\"-i\" is expected for the %d th argument in declaring filter %s", itok, tok[2]);
			throw cmd.ret;
		}
		itok++;

		for(; strcmp(tok[itok], "-o") != 0 && itok < cmd.num_args; itok++){
			ch_base * pchan = get_channel(tok[itok]);
			if(pchan == NULL){ // no channel found.
				sprintf(cmd.get_ret_str(), "Channel %s not found for connecting filter %s", tok[itok], tok[2]);
				throw cmd.ret;
			}

			pfilter->set_ichan(pchan);
		}

		itok++;

		for(; itok < cmd.num_args; itok++){
			ch_base * pchan = get_channel(tok[itok]);
			if(pchan == NULL){ // no channel found.
				sprintf(cmd.get_ret_str(), "Channel %s not found for connecting filter %s", tok[itok], tok[2]);
				throw cmd.ret;
			}
			pfilter->set_ochan(pchan);
		}
	}catch(char *){
		delete pfilter;
		pthread_mutex_unlock(&m_mtx);
		return false;
	}
	cout << "filter " << pfilter->get_name() << " added." << endl;

	m_filters.push_back(pfilter);
	return true;
}

bool c_aws::check_graph()
{
	bool healthy = true;

	for(vector<f_base*>::iterator itr = m_filters.begin(); 
		itr != m_filters.end(); itr++){
		if(!(*itr)->check()){
			healthy = false;
		}
	}

	return healthy;
}

bool c_aws::main(){

	c_rcmd * prcmd = new c_rcmd(this, m_cmd_port);
	if(!prcmd->is_exit())
		m_rcmds.push_back(prcmd);
	else{
		delete prcmd;
		cerr << "Failed to construct command thread." << endl;
		return false;
	}

	if(m_working_path){
		if(chdir(m_working_path) != 0)
			cerr << "Failed to change path to " << m_working_path << "." << endl;
	}

	m_exit = false;

	while(!m_exit){
		proc_command();
		if(!f_base::m_clk.is_stop()){

			f_base::m_clk.wait();
			m_time = f_base::m_clk.get_time();

			// sending clock signal to each filters. 
			f_base::clock(m_time);

			// checking activity of filters. 
			for(vector<f_base*>::iterator itr = m_filters.begin(); 
				itr != m_filters.end(); itr++){
				if(!(*itr)->is_active()){
					cout << (*itr)->get_name() << " stopped." << endl;
					f_base::m_clk.stop();
					break;
				}
			}

			// Time is exceeded over m_end_time, automatically pause.
			if(m_time >= m_end_time){
				if(!m_bonline){
					f_base::m_clk.pause();
				}else{
					f_base::m_clk.stop();
				}
			}

			if(f_base::m_clk.is_stop()){
				// stop all the filters
				handle_stop();
				cout << "Processing loop stopped." << endl;
			}
		}
	}

	handle_stop();

	for(int i = 0; i < m_rcmds.size() ;i++){
		delete m_rcmds[i];
	}
	m_rcmds.clear();

	return true;
}
