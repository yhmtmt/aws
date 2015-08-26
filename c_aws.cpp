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
#include <list>
#include <queue>
using namespace std;

#ifdef _WIN32
#include <direct.h>
#endif

#include <opencv2/opencv.hpp>
using namespace cv;

#include "util/aws_sock.h"
#include "util/aws_thread.h"

#include "util/c_clock.h"
#include "channel/ch_base.h"
#include "filter/f_base.h"
#include "util/c_ship.h"
#include "command.h"
#include "c_aws.h"

void cmd_proc_loop(const char * prompt, c_aws & aws, istream & in);

bool proc_script(const char * fname, c_aws & aws)
{
	ifstream ifile(fname);
	if(ifile.is_open()){
		cmd_proc_loop(fname, aws, ifile);
		return true;
	}
	return false;
}

void * cmd_proc(void * paws)
{
	c_aws & aws = *((c_aws*) paws);

	cmd_proc_loop("aws", aws, cin);

	return NULL;
}

void cmd_proc_loop(const char * prompt, c_aws & aws, istream & in)
{
	// invoke capturing threads for each cam
	char cmd_line[CMD_LEN];

	while(!g_kill && !in.eof()){ // command processing loop
		cout << prompt << ">";
		in.getline(cmd_line, CMD_LEN, '\n');

		int len = (int) strlen(cmd_line);
		if(len >= CMD_LEN){
			cerr << "Irregal command. Command length\
					should be less than " << CMD_LEN << endl;
			continue;
		}

		// removing comment. (comment is headed by "//" or "#"
		for(int i = 0; i < len; i++){
			if((cmd_line[i] == '/' && cmd_line[i+1] == '/') || 
				cmd_line[i] == '#'){
					cmd_line[i] = '\0';
					break;
			}
		}

		cout << cmd_line << endl;
		bool stat = false;
		if(aws.push_command(cmd_line, cmd_line, stat)){
			if(!stat)
				cout << "Error: ";
			else 
				cout << "Success: ";
			cout << cmd_line << endl;
			continue;
		}

		if(proc_script(cmd_line, aws))
			continue;

		cerr << "Unknown command" << endl;
	}
}



// Command explanation
// * channel <type> <name>
// Creating an channel instance of <type> with <name> 
// * filter <type> <name> -i <input list> -o <output list>
// Creating an filter instance of <type> with <name>. In <input list> and <output list>, users can specify names of the channel instances.
// * fcmd <filter name> <filter command>
// Executing filter commands defined in each filter.  
// * fset <filter name> <parameter and value list>
// Setting the values to their specified parameters. If only a parameter name is specified as <parameter and value list>, an explanation to the parameter is returned.
// * fget <filter name> <parameter list>
// Getting the values of the parameters specified.
// * finf | finf <filter name> | finf n <filter id>
// This will be hidden command. The first case, the number of filters are returned.
// The second and third case, filter name, filter id, number of parameters, number of input channels, number of output channels, are returned  
// * fpar <filter name> <parameter id>
// An explanatio of the parameter of the filter is returned.
// * chinf | chinf <channel name> | chinf n <channel id>
// This will be hidden command. The first case, the number of channels are returned. 
// The second and third cases, channel name and id are returned.
// * go | go <start time> | go <start time> <end time> | go to <end time>
// start the filter graph's threads from <start time> to <end time> if specified. Filter graph moves the state "stop" to "run".
// Start and end times are not allowed for online mode. In the online mode, the filter graph's start and end time should be controled in script.
// * stop 
// stop the filter graph's threads.
// * quit
// shutdown the aws process
// * step | step <time> | step c <cycle>
// In pause state, first usage proceeds one cycle, second one proceeds to the specified <time>, and third one proceeds specified cycles.
// * cyc <sec>
// Setting cycle time 
// * online {yes | no}
// Setting execution mode. In the offline mode, pause and step is allowed. 
// * pause
// The filter graph moves the sate "run" to "pause".
// * clear
// All filters and channels are discarded.
// * rcmd <port number>
// Invoking reciever thread of rcmd with <port number>
// * trat <int >= 1>
// Setting trat. trat enables the faster time clocking. For example, the time goes twice as fast as usual with trat of 2. 
// Not that, trat is only allowed for offline mode.


c_aws::c_aws(int argc, char ** argv):CmdAppBase(argc, argv),
	m_cmd_port(20000), m_working_path(NULL), m_bonline(true), m_exit(false),
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

	// Initializing filter globals
	f_base::init(this);

	// Initializing channel globals
	ch_base::init();

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

	c_ship::init();
}

c_aws::~c_aws()
{
	c_ship::destroy();
	f_base::uninit();
	ch_base::uninit();
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
	for(vector<f_base*>::iterator itr = m_filters.begin(); 
		itr != m_filters.end(); itr++)
		delete (*itr);
	m_filters.clear();

	for(vector<ch_base*>::iterator itr = m_channels.begin();
		itr != m_channels.end(); itr++)
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
	cmd.type = cmd_str_to_id(m_cmd.args[0]);

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
		for(vector<f_base*>::iterator fitr = m_filters.begin(); 
			fitr != m_filters.end(); fitr++)
			stopped = stopped && (*fitr)->stop();
		f_base::clock(-1);
	}
	for(vector<f_base*>::iterator fitr = m_filters.begin();
	    fitr != m_filters.end(); fitr++)
	  (*fitr)->runstat();

	return true;
}

// handle_run runs all the filters in the graph.
bool c_aws::handle_chan(s_cmd & cmd)
{
	bool result;
	if(cmd.num_args < 3)
		return false;

	if(!(result = add_channel(cmd)))
		sprintf(cmd.get_ret_str(), "Failed to create channel %s of %s.",
			cmd.args[2], cmd.args[1]);
	return result;
}

bool c_aws::handle_fltr(s_cmd & cmd)
{
	bool result;
	if(cmd.num_args < 5)
		return false;
	if(!(result = add_filter(cmd)))
		sprintf(cmd.get_ret_str(), "Failed to create filter %s of %s.", 
			cmd.args[2], cmd.args[1]);
	return result;
}

bool c_aws::handle_fcmd(s_cmd & cmd)
{
	bool result;
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
	return result;
}

bool c_aws::handle_fset(s_cmd & cmd)
{
	bool result;

	if(cmd.num_args <= 2){
		result = false;
		return result;
	}

	f_base * pfilter = get_filter(cmd.args[1]);

	if(pfilter == NULL){
		sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
		result = false;
	}else{
		if(cmd.num_args == 3){ // no value present
			result = pfilter->get_par_info_by_fset(cmd);
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

	return result;
}

bool c_aws::handle_fget(s_cmd & cmd)
{
	bool result;
	if(cmd.num_args <= 2){
		result = false;
		return result;
	}

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

	return result;
}

bool c_aws::handle_finf(s_cmd & cmd)
{
	bool result;
	// This command retrieves filter information
	f_base * pfilter = NULL;
	int ifilter;
	if(cmd.num_args == 2){ // The second argument is filter name
		pfilter = get_filter(cmd.args[1]);
		if(pfilter == NULL){
			sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
		}else{
			for(ifilter = 0; ifilter < m_filters.size(); ifilter++){
				if(pfilter == m_filters[ifilter])
					break;
			}
		}
	}else if(cmd.num_args == 3 && cmd.args[1][0] == 'n'){ 
		// if n is specified as the second argument, the filter id is used as 
		// the argument.
		ifilter = atoi(cmd.args[2]);
		if(ifilter >= m_filters.size()){
			sprintf(cmd.get_ret_str(), "Filter id=%d does not exist.", ifilter);
		}else{
			pfilter = m_filters[ifilter];
		}
	}else{ // if there is no argument, the number of filters is returned
		sprintf(cmd.get_ret_str(), "%d", m_filters.size());
		result = true;
		return result;
	}

	if(pfilter == NULL){
		result = false;
	}else{
		pfilter->get_info(cmd, ifilter);
		result = true;
	}

	return result;
}

bool c_aws::handle_fpar(s_cmd & cmd)
{
	bool result;
	f_base * pfilter = get_filter(cmd.args[1]);
	if(pfilter == NULL){
		sprintf(cmd.get_ret_str(), "Filter %s was not found.", cmd.args[1]);
		result = false;
	}else{
		if(!pfilter->get_par_info(cmd)){
			result = false;
		}else{
			result = true;
		}
	}
	return result;
}

bool c_aws::handle_chinf(s_cmd & cmd)
{
	bool result;

	// This command retrieves channel information the codes below are almost
	// same as FINF
	ch_base * pch = NULL;
	int ich;
	if(cmd.num_args == 2){
		pch = get_channel(cmd.args[1]);
		if(pch == NULL){
			sprintf(cmd.get_ret_str(), "Channel %s was not found.", cmd.args[1]);
		}else{
			for(ich = 0; ich < m_filters.size(); ich++){
				if(pch == m_channels[ich])
					break;
			}
		}
	}else if(cmd.num_args == 3 && cmd.args[1][0] == 'n'){
		ich = atoi(cmd.args[2]);
		if(ich >= m_channels.size()){
			sprintf(cmd.get_ret_str(), "Channel id=%d does not exist.", ich);
		}else{
			pch = m_channels[ich];
		}
	}else{
		sprintf(cmd.get_ret_str(), "%d", m_channels.size());
		result = true;
		return result;
	}

	if(pch == NULL){
		result = false;
	}else{
		pch->get_info(cmd, ich);
		result = true;
	}
	
	return result;
}

bool c_aws::handle_quit(s_cmd & cmd)
{
	bool result = true;
	m_exit = true;
	return result;
}

bool c_aws::handle_step(s_cmd & cmd)
{
	bool result;
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

	return result;
}

bool c_aws::handle_cyc(s_cmd & cmd)
{
	bool result;
	if(!f_base::m_clk.is_stop()){
		cout << "stop:" << f_base::m_clk.is_stop() << endl;
		cout << "run:" << f_base::m_clk.is_run() << endl;
		cout << "pause:" << f_base::m_clk.is_pause() << endl;

		sprintf(cmd.get_ret_str(), "Cycle cannot be changed during execution");
		result = false;
	}else{
		m_cycle_time = (long long) (atof(cmd.args[1]) * SEC);
		result = true;
	}
	return result;
}

bool c_aws::handle_online(s_cmd & cmd)
{
	bool result;
	if(!f_base::m_clk.is_stop()){
		sprintf(cmd.get_ret_str(), 
			"Online/offline mode cannot be changed during execution.");
		result = false;
	}else{
		if(strcmp(cmd.args[1], "yes") == 0)
			m_bonline = true;
		else
			m_bonline = false;
		result = true;
	}
	return result;
}

bool c_aws::handle_pause(s_cmd & cmd)
{
	bool result;
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
	return result;
}

bool c_aws::handle_clear(s_cmd & cmd)
{
	bool result;
	if(!f_base::m_clk.is_run()){
		sprintf(cmd.get_ret_str(), "Graph cannot be cleared during execution.");
		result = false;
	}else{
		clear();
		result = true;
	}
	return result;
}

bool c_aws::handle_rcmd(s_cmd & cmd)
{
	bool result;
	if(cmd.num_args == 2){	
		c_rcmd * pcmd = new c_rcmd(this, atoi(cmd.args[1]));
		if(!pcmd->is_exit()){
			m_rcmds.push_back(pcmd);
			result = true;
		}else{
			delete pcmd;
		}
	}
	return result;
}

bool c_aws::handle_trat(s_cmd & cmd)
{
	bool result;
	if(!f_base::m_clk.is_stop()){
		sprintf(cmd.get_ret_str(), "Trat cannot be changed during execution");
		result = false;
	}else{
		m_time_rate = (int) atoi(cmd.args[1]);
		result = true;
	}
	return result;
}

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
				sprintf(cmd.get_ret_str(), 
					"In pause state, \"go [to <later absolute time>]\"");
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
		snprintf(cmd.get_ret_str(), RET_LEN, "ERROR: Failed to run filter graph.");
		return false;
	}

	// if the starting time is given the time is decoded and stored into m_start_time 
	if(cmd.num_args >= 2){
		if(m_bonline){
			snprintf(cmd.get_ret_str(), RET_LEN, 
				"ERROR: Online mode does not support time specification.");
			return false;
		}

		tmex tm;
		if(decTmStr(cmd.args[1], tm))
			m_start_time = mkgmtimeex_tz(tm, f_base::get_tz()) * MSEC;
		else
			m_start_time = (long long) atol(cmd.args[1]) * (long long) SEC;
	}else{
		// if no time specified, current time is used as the start time.
		m_start_time = (long long) time(NULL) * SEC; 
	}

	// if the end time is given, decoded and stored into m_end_time.
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
		(unsigned) m_cycle_time, m_start_time, 
		m_bonline, m_time_rate);
	m_time = f_base::m_clk.get_time();

	f_base::init_run_all();
	f_base::clock(m_start_time);

	// check filter's status. 
	for(vector<f_base*>::iterator fitr = m_filters.begin();
		fitr != m_filters.end(); fitr++){
		if(!(*fitr)->run(m_start_time, m_end_time)){
			snprintf(cmd.get_ret_str(),  RET_LEN, 
				"Error in starting filter %s.", (*fitr)->get_name());
			return false;
		}
	}

	return true;
}

bool c_aws::handle_frm(s_cmd & cmd)
{
	if(cmd.num_args != 2){
		return false;
	}

	for(vector<f_base*>::iterator itr = m_filters.begin();
		itr != m_filters.end(); itr++){
		if(strcmp(cmd.args[1], (*itr)->get_name()) == 0){
			m_filters.erase(itr);
			break;
		}
	}

	return true;
}

bool c_aws::handle_chrm(s_cmd & cmd)
{
	if(cmd.num_args != 2){
		return false;
	}

	for(vector<ch_base*>::iterator itr = m_channels.begin();
		itr != m_channels.end(); itr++){
		if(strcmp(cmd.args[1], (*itr)->get_name()) == 0){
			m_channels.erase(itr);
			break;
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
			result = handle_chan(cmd);
			break;
		case CMD_FLTR:
			result = handle_fltr(cmd);
			break;
		case CMD_FCMD:
			result = handle_fcmd(cmd);
			break;
		case CMD_FSET:
			result = handle_fset(cmd);
			break;
		case CMD_FGET:
			result = handle_fget(cmd);
			break;
		case CMD_FINF:
			result = handle_finf(cmd);
			break;
		case CMD_FPAR:
			result = handle_fpar(cmd);
			break;
		case CMD_CHINF:
			result = handle_chinf(cmd);
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
			result = handle_step(cmd);
			break;
		case CMD_CYC:
			result = handle_cyc(cmd);
			break;
		case CMD_ONLINE:
			result = handle_online(cmd);
			break;
		case CMD_PAUSE:
			result = handle_pause(cmd);
			break;
		case CMD_CLEAR:
			result = handle_clear(cmd);
			break;
		case CMD_RCMD:
			result = handle_rcmd(cmd);
			break;
		case CMD_TRAT:
			result = handle_trat(cmd);
			break;
		case CMD_CHRM:
			result = handle_chrm(cmd);
			break;
		case CMD_FRM:
			result = handle_frm(cmd);
			break;
		case CMD_CD:
			if(cmd.num_args != 2)
				result = false;
			else{
				chdir(cmd.args[1]);
				result = true;
			}
			break;
		case CMD_TIME:
		  if(cmd.num_args == 1){
		    snprintf(cmd.get_ret_str(), RET_LEN, "%s", f_base::get_time_str());
		    result = true;
		  }else if(cmd.num_args == 2 && cmd.args[1][0] == 'n'){
		    snprintf(cmd.get_ret_str(), RET_LEN, "%lld", f_base::get_time());
		    result = true;
		  }else if(cmd.num_args == 2 && cmd.args[1][0] == 's'){
			f_base::set_sys_time();
			result = true;
		  }else
		    result = false;
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
			snprintf(cmd.get_ret_str(), RET_LEN, 
				"\"-i\" is expected for the %d th argument in declaring filter %s", 
				itok, tok[2]);
			throw cmd.ret;
		}
		itok++;

		for(; strcmp(tok[itok], "-o") != 0 && itok < cmd.num_args; itok++){
			ch_base * pchan = get_channel(tok[itok]);
			if(pchan == NULL){ // no channel found.
				snprintf(cmd.get_ret_str(), RET_LEN, 
					"Channel %s not found for connecting filter %s", 
					tok[itok], tok[2]);
				throw cmd.ret;
			}

			pfilter->set_ichan(pchan);
		}

		itok++;

		for(; itok < cmd.num_args; itok++){
			ch_base * pchan = get_channel(tok[itok]);
			if(pchan == NULL){ // no channel found.
				snprintf(cmd.get_ret_str(), RET_LEN,
					"Channel %s not found for connecting filter %s",
					tok[itok], tok[2]);
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
			// wait the time specified in cyc command.
			f_base::m_clk.wait();

			// getting current time
			m_time = f_base::m_clk.get_time();

			// sending clock signal to each filters. The time string for current time is generated simultaneously
			f_base::clock(m_time);

			// checking activity of filters. 
			for(vector<f_base*>::iterator itr = m_filters.begin(); 
				itr != m_filters.end(); itr++){
					if((*itr)->is_main_thread())
						(*itr)->fthread();
				if(!(*itr)->is_active()){
					cout << (*itr)->get_name() << " stopped." << endl;
					f_base::m_clk.stop();
					break;
				}
			}

			// Time is exceeded over m_end_time, automatically pause.
			if(!m_bonline && m_time >= m_end_time){
				f_base::m_clk.pause();
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

//////////////////////////////////////////////////////// class c_rcmd member
c_rcmd::c_rcmd(c_aws * paws, unsigned short port):m_paws(paws){
	m_to.tv_sec = 5;
	m_to.tv_usec = 0;
	m_exit = true;
	m_svr_sock = socket(AF_INET, SOCK_STREAM, 0);
	if(m_svr_sock == SOCKET_ERROR){
		cerr << "socket failed with SOCKET_ERROR" << endl;
		m_svr_sock = -1;
		return;
	}

	m_svr_addr.sin_family = AF_INET;
	m_svr_addr.sin_port = htons(port);
	m_svr_addr.sin_addr.s_addr = INADDR_ANY;

	int ret = ::bind(m_svr_sock, (sockaddr*)&m_svr_addr, sizeof(m_svr_addr));
	if(ret != 0){
		cerr << "bind failed with SOCKET_ERROR." << endl;
		closesocket(m_svr_sock);
		m_svr_sock = -1;
		return;
	}

	ret = listen(m_svr_sock, 5);
	if(ret == SOCKET_ERROR){
		cerr << "listen failed with SOCKET_ERROR." << endl;
		closesocket(m_svr_sock);
		m_svr_sock = -1;
		return;
	}

	m_exit = false;

	pthread_create(&m_th_rcmd, NULL, thrcmd, (void*)this);
}

c_rcmd::~c_rcmd(){
	if(!m_exit){
		m_exit = true;
		pthread_join(m_th_rcmd, NULL);
	}
	if(m_svr_sock != SOCKET_ERROR && m_svr_sock != -1)
		closesocket(m_svr_sock);
}

// wait_connection waits connection to socket s. The function don't return
// until session opened or timeout.
bool c_rcmd::wait_connection(SOCKET & s){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdread);
	FD_ZERO(&m_fderr);
	FD_SET(m_svr_sock, &m_fdread);
	FD_SET(m_svr_sock, &m_fderr);
	int n = select((int)(m_svr_sock)+1, &m_fdread, NULL, &m_fderr, &m_to);

	if(n > 0){
		if(FD_ISSET(m_svr_sock, &m_fdread)){
			int len = sizeof(m_svr_addr);
#ifdef _WIN32
			s = accept(m_svr_sock, (sockaddr*)&m_svr_addr, &len);
#else
			s = accept(m_svr_sock, (sockaddr*)&m_svr_addr, (socklen_t*) &len);
#endif
			return true;
		}else if(FD_ISSET(m_svr_sock, &m_fderr)){
			cerr << "Socket error." << endl;
			m_exit = true;
			return false;
		}
	}
	return false;
}

// wait_receive receives data to buf from socket s. Data length is returned
// to the argument "total". This function should be called after session opened.
bool c_rcmd::wait_receive(SOCKET & s, char * buf, int & total){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdread);
	FD_ZERO(&m_fderr);
	FD_SET(s, &m_fdread);
	FD_SET(s, &m_fderr);

	int n = select((int)s+1, &m_fdread, NULL, &m_fderr, &m_to);
	if(n > 0){
		if(FD_ISSET(s, &m_fdread)){
			total += recv(s, buf + total, CMD_LEN - total, 0);
			return total == CMD_LEN;
		}else if(FD_ISSET(s, & m_fderr)){
			cerr << "Socket error" << endl;
			return false;
		}
	}	

	return false;
}

// push_command send command string (cmd_str) to the aws object.
// this function blocks until the command processed in the aws object.
// return string is stored in ret_str. if the command failed, ret_stat is
// set as false.
bool c_rcmd::push_command(const char * cmd_str, char * ret_str, bool ret_stat){
	bool stat = false;
	if(!m_paws->push_command(cmd_str, ret_str, ret_stat)){
		cerr << "Unknown command " << cmd_str << endl;
		return false;
	}
	return true;
}

// wait_send send is used to send return string to the sender of the command.
// This function returns after sending data or timeout
bool c_rcmd::wait_send(SOCKET & s, char * buf){
	m_to.tv_sec = 10;
	m_to.tv_usec = 0;
	FD_ZERO(&m_fdwrite);
	FD_ZERO(&m_fderr);
	FD_SET(s, &m_fdwrite);
	FD_SET(s, &m_fderr);
	int n = select((int)s+1, NULL, &m_fdwrite, &m_fderr, &m_to);
	if(n > 0){
		if(FD_ISSET(s, &m_fdwrite)){
			send(s, buf, CMD_LEN, 0);
			return true;
		}else if(FD_ISSET(s, &m_fderr)){
			cerr << "Socket error" << endl;
			return false;
		}
	}
	return false;
}

// command processing thread repeates 1. waiting for connection from command 
// sender, 2. receiving the command, 3. processing the command in aws, 4. 
// returning the resulting return value. This thread is invoked in the constructor,
// and terminated in the destructor.
void * c_rcmd::thrcmd(void * ptr)
{
	c_rcmd * prcmd = (c_rcmd*) ptr;

	while(!prcmd->is_exit()){
		SOCKET s;
		if(!prcmd->wait_connection(s)){
			continue;
		}

		while(1){
			int total = 0;
			while(!prcmd->wait_receive(s, prcmd->m_buf_recv, total));
			if(strcmp("eoc", prcmd->m_buf_recv) == 0){
				break;
			}

			// push command 
			bool stat = false;
			if(total == CMD_LEN){
				if(!prcmd->m_paws->push_command(prcmd->m_buf_recv, 
					prcmd->m_buf_send, stat)){
						cerr << "Unknown command " << prcmd->m_buf_recv << endl;
				}
			}

			// return
			while(!prcmd->wait_send(s, prcmd->m_buf_send));
		}
		closesocket(s);
	}

	return NULL;
}

bool c_rcmd::is_exit(){
	return m_paws->is_exit() || m_exit;
}
