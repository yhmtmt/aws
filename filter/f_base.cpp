#include "stdafx.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_base is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_base is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_base.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/c_clock.h"
#include "f_base.h"
#include "../c_aws.h"
#include "../util/aws_stdlib.h"

/////////////////////////////////////////////////////////////// filter factory
FMap f_base::m_fmap;

f_base* f_base::create(const char * tname, const char * fname)
{
	FMap::iterator itr = m_fmap.find(tname);
	if(itr == m_fmap.end()){
	  cerr << "Cannot find " << tname << " in our filter list." << endl;
	  cerr << "We have :" << endl;
	  for(itr = m_fmap.begin(); itr != m_fmap.end(); itr++)
	    cerr << "\t" << itr->first << endl;
	  return NULL;
	}
	CreateFilter creator = itr->second;
	return (f_base*) creator(fname);
}
//////////////////////////////////////////////////// filter parameter
bool f_base::s_fpar::set(const char * valstr){	
	switch(type){
	case F64:
		*f64 = atof(valstr);break;
	case S64:
		*s64 = atoll(valstr);break;
	case U64:
		*u64 = strtoull(valstr, NULL, 0);break;
	case F32:
		*f32 = (float) atof(valstr); break;
	case S32:
		*s32 = atoi(valstr); break;
	case U32:
		*u32 = strtoul(valstr, NULL, 0);break;
	case S16:
		*s16 = (short) atoi(valstr); break;
	case U16:
		*u16 = (unsigned short) strtoul(valstr, NULL, 0); break;
	case S8:
		*s8 = (char) atoi(valstr); break;
	case U8:
		*u8 = (unsigned char) strtoul(valstr, NULL, 0); break;
	case BIN:
		if(valstr[0] == 'y')
			*bin = true;
		else if (valstr[0] == 'n')
			*bin = false;
		else
			return false;
		break;
	case CSTR:
		if(strlen(valstr)+1 > len)
			return false;
		strncpy(cstr, valstr, len);
		break;
	case ENUM:
		for(*s32 = 0; *s32 < len; (*s32)++){
			if(strcmp(str_enum[*s32], valstr) == 0){
				break;
			}
		}
		break;
	}
	return true;
}

bool f_base::s_fpar::get(char * valstr, size_t sz){
	int n;
	switch(type){
	case F64:
		n = snprintf(valstr, sz, "%e", *f64);break;
	case S64:
		n = snprintf(valstr, sz, "%lld", *s64);break;
	case U64:
		n = snprintf(valstr, sz, "%llu", *u64);break;
	case F32:
		n = snprintf(valstr, sz, "%e", *f32);break;
	case S32:
		n = snprintf(valstr, sz, "%d", *s32);break;
	case U32:
		n = snprintf(valstr, sz, "%u", *u32);break;
	case S16:
		n = snprintf(valstr, sz, "%hd", *s16);break;
	case U16:
		n = snprintf(valstr, sz, "%hu", *u16);break;
	case S8:
		n = snprintf(valstr, sz, "%hhd", *s8);break;
	case U8:
		n = snprintf(valstr, sz, "%hhu", *u8);break;
	case BIN:
		n = snprintf(valstr, sz, "%s", (*bin) ? "y" : "n");break;
	case CSTR:
		n = snprintf(valstr, sz, "%s", cstr);break;
	case ENUM:
		n = snprintf(valstr, sz, "%s", str_enum[*s32]);break;
	}
	return n < sz;	
}

void f_base::s_fpar::get_info(s_cmd & cmd){
	snprintf(cmd.get_ret_str(), CMD_LEN, "%s: %s", name, explanation);
	if(type == ENUM){
		int slen = (int) strlen(cmd.get_ret_str());
		char * ptr = cmd.get_ret_str() + slen;
		*ptr = ' ';
		snprintf(ptr, CMD_LEN - slen, " in {");
		ptr = cmd.get_ret_str();
		slen = (int) strlen(ptr);
		for(int i = 0; i < len && slen < len; i++){
			snprintf(ptr + slen, CMD_LEN - slen, "%s,", str_enum[i]);
			slen += (int) strlen(str_enum[i]) + 1;
		}
		slen = (int) strlen(cmd.get_ret_str());
		ptr[slen-1] = '}';
	}
}

void f_base::s_ferr::dump_err(ostream & out)
{
	out << tstr << " "
		<< fname << ":"
		<< line << ":";
	const char * msg = ptr->get_err_msg(code);
	if(msg)
		out << msg;
	out << endl;
}

////////////////////////////////////////////////////// f_base members
pthread_mutex_t f_base::m_mutex;
pthread_cond_t f_base::m_cond;
long long f_base::m_cur_time = 0;
long long f_base::m_count_clock = 0;
int f_base::m_time_zone_minute = 540;
char f_base::m_time_str[32];
tmex f_base::m_tm;
c_clock f_base::m_clk;
c_aws * f_base::m_paws = NULL;

ofstream f_base::m_file_err;
f_base::s_ferr f_base::m_err_buf[SIZE_FERR_BUF];
int f_base::m_err_head = 0;
int f_base::m_err_tail = 0;
pthread_mutex_t f_base::m_err_mtx;

// this is the filter thread function, but actually called from main thread.
// this function is used if the filter should be executed in the mainthread such as the case using OpenGL
void f_base::fthread()
{
	if((unsigned int) m_cycle < m_intvl){
		m_cycle++;
		return;
	}

	if(m_bactive){		
		tran_chout();
		calc_time_diff();

		if (!proc()){
			m_bactive = false;
		}

		if(m_clk.is_run()){
			m_count_proc++;
			m_max_cycle = max(m_cycle, m_max_cycle);
			m_proc_rate = (double)  m_count_proc / (double) m_count_clock;
		}
	}else{
		m_bstopped = true;
	}
}

// Filter thread function
void * f_base::fthread(void * ptr)
{
	f_base * filter = (f_base *) ptr;
	while(filter->m_bactive){
		filter->m_count_pre = filter->m_count_clock;
		
		while(filter->m_cycle < (int) filter->m_intvl){
			filter->clock_wait();
			filter->m_cycle++;
		}
		filter->lock_cmd();

		if(filter->m_bochrep){
			if(!filter->repoch())
				filter->m_bactive = false;
		} else if(filter->m_bochlog){
			filter->logoch();
		}

		filter->tran_chout();
		filter->calc_time_diff();

		if (!filter->m_bochrep){
		  if(!filter->proc()){
			filter->m_bactive = false;
		  }
		}

		if(filter->m_clk.is_run()){
			filter->m_count_proc++;
			filter->m_max_cycle = max(filter->m_cycle, filter->m_max_cycle);
			filter->m_count_post = filter->m_count_clock;
			filter->m_cycle = (int)(filter->m_count_post - filter->m_count_pre);
			filter->m_cycle -= filter->m_intvl;
			filter->m_proc_rate = (double)  filter->m_count_proc / (double) filter->m_count_clock;
		}

		filter->unlock_cmd();
	}

	filter->m_bstopped = true;
	return NULL;
}

void f_base::flush_err_buf(){
	pthread_mutex_lock(&m_err_mtx);
	for(;m_err_tail != m_err_head; 
		m_err_head = (1 + m_err_head) % SIZE_FERR_BUF){
			m_err_buf[m_err_head].dump_err(m_file_err);
			m_err_buf[m_err_head].dump_err(cerr);
	}
	pthread_mutex_unlock(&m_err_mtx);
}

void f_base::send_err(f_base * ptr, const char * fname, int line, int code)
{
	pthread_mutex_lock(&m_err_mtx);
	int next_tail = (m_err_tail + 1) % SIZE_FERR_BUF;
	if(next_tail == m_err_head){
		pthread_mutex_unlock(&m_err_mtx);
		flush_err_buf();
		pthread_mutex_lock(&m_err_mtx);
	} 

	strncpy(m_err_buf[m_err_tail].tstr, get_time_str(), 32);
	m_err_buf[m_err_tail].ptr = ptr;
	m_err_buf[m_err_tail].fname = fname;
	m_err_buf[m_err_tail].line = line;
	m_err_buf[m_err_tail].code = code;
	m_err_tail = next_tail;
	pthread_mutex_unlock(&m_err_mtx);
}

void f_base::clock(long long cur_time){
	pthread_mutex_lock(&m_mutex);
	if(m_clk.is_run())
		m_count_clock++;
	m_cur_time = cur_time;
	gmtimeex(m_cur_time / MSEC  + m_time_zone_minute * 60000, m_tm);
	snprintf(m_time_str, 32, "[%s %s %02d %02d:%02d:%02d.%03d %d] ", 
		getWeekStr(m_tm.tm_wday), 
		getMonthStr(m_tm.tm_mon),
		m_tm.tm_mday,
		m_tm.tm_hour,
		m_tm.tm_min,
		m_tm.tm_sec,
		m_tm.tm_msec,
		m_tm.tm_year + 1900);

	pthread_mutex_unlock(&m_mutex);
	pthread_cond_broadcast(&m_cond);
}

f_base::f_base(const char * name):m_offset_time(0), m_bactive(false), 
	m_intvl(1), m_bstopped(true), m_cmd(false), 
	m_bochlog(false), m_bochrep(false), m_tdrvchk(10 * SEC), m_spddrvuse(0), m_curochlogdrv(-1)

{
	pthread_mutex_init(&m_mutex_cmd, NULL);
	pthread_cond_init(&m_cnd_cmd, NULL);
	m_name = new char[strlen(name) + 1];
	strncpy(m_name, name, strlen(name) + 1);

	register_fpar("TimeShift", &m_offset_time, "Filter time offset relative to global clock. (may be offline mode only)");
	register_fpar("Interval", &m_intvl, "Execution interval in cycle. (default 0)");
	register_fpar("ProcCount", &m_count_proc, "Number of execution."); 
	register_fpar("ClockCount", &m_count_clock, "Number of clock cycles passed." );
	register_fpar("ProcRate", &m_proc_rate, "Processing rate.(Read only)");
	register_fpar("MaxCycle", &m_max_cycle, "Maximum cycles per processing.(Read Only)");

	// och logging
	m_fochlogpath[0] = '\0';
	m_fjochlog[0] = '\0';

	register_fpar("fochlogpath", m_fochlogpath, 1024, "File path to a file of a list of storages for the output channel logs."); 
	register_fpar("fjochlog", m_fjochlog, 1024, "File path to a journal of the output channel logs.");
	register_fpar("ochlog", &m_bochlog, "Enable output channel logging mode.");
	register_fpar("ochrep", &m_bochrep, "Enable output channel replay mode.");
	register_fpar("tdrvchk", & m_tdrvchk, "Time interval checking drive use specified in fochlogpath.");
}

f_base::~f_base()
{
	pthread_mutex_destroy(&m_mutex_cmd);
	m_chin.clear();
	m_chout.clear();
	delete [] m_name;
}

bool f_base::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	int itok = 2;

	if(strcmp(args[itok], "tsh") == 0){
		if(num_args != 4)
			return false;
		cout << m_name << " time offset changed from " << m_offset_time << " to ";
		m_offset_time += (long long) (atof(args[itok+1]) * SEC);
		m_offset_time = max(m_offset_time, 0LL);
		cout << m_offset_time << "." << endl;
		return true;
	}else if(strcmp(args[itok], "intvl") == 0){
		if(num_args != 4)
			return false;
		m_intvl = max(0, atoi(args[itok+1]));
		cout << "Execution interval of " << m_name << " is set to " << m_intvl << " cycles." << endl;

		return true;
	}else if(strcmp(args[itok], "ar") == 0){ // the command returns activity ratio
		snprintf(cmd.get_ret_str(),  RET_LEN, "%f", (double) m_count_proc / (double) m_count_clock);
		return true;
	}
	return false;
}

bool f_base::set_par(s_cmd & cmd){
	int iarg, ipar;
	for(iarg = 2; iarg < cmd.num_args; iarg++){
		ipar = find_par(cmd.args[iarg]);
		if(ipar < 0){
			snprintf(cmd.get_ret_str(),  RET_LEN, "Filter %s does not have parameter %s.", cmd.args[1], cmd.args[iarg]);
			return false;
		}
		iarg++;
		if(iarg >= cmd.num_args){
			snprintf(cmd.get_ret_str(),  RET_LEN, "Value is not specifeid for parameter %s of %s.", cmd.args[iarg--], cmd.args[1]);
			return false;
		}

		if(!m_pars[ipar].set(cmd.args[iarg])){
			snprintf(cmd.get_ret_str(),  RET_LEN, "Failed to set parameter %s of %s", cmd.args[iarg--], cmd.args[1]);
			return false;
		}
	}

	return true;
}

bool f_base::get_par(s_cmd & cmd){
	int iarg, ipar;
	int len = 0;
	char * valstr = cmd.get_ret_str();
	char * valsubstr = valstr;
	for(iarg = 2; iarg < cmd.num_args; iarg++){
		ipar = find_par(cmd.args[iarg]);
		if(ipar < 0){
			snprintf(cmd.get_ret_str(), RET_LEN, "Filter %s does not have parameter %s.", cmd.args[1], cmd.args[iarg]);
			return false;
		}

		if(!m_pars[ipar].get(valsubstr, RET_LEN - len - 1)){
			snprintf(cmd.get_ret_str(), RET_LEN, "Failed to get parameter %s of %s",  cmd.args[iarg], cmd.args[1]);
			return false;
		}

		len += (int) strlen(valsubstr);
		valstr[len] = ' ';
		len += 1;
		valsubstr = valstr + len;
	}
	valstr[len] = '\0';

	return true;
}

bool f_base::load_ochlogpath()
{
	if(!m_bochlog)
		return false;
	if(m_fochlogpath[0] == '\0')
		return false;

	try{
		ifstream fochlogpath(m_fochlogpath);
		if(fochlogpath.is_open()){
			// in the file,  the list of @<path> <threashold><crlf> is appeared.
			char buf[1024];
			while(!fochlogpath.eof()){
				fochlogpath.getline(buf, 1024);
				if(buf[0] != '@')
					break;
				// extract file path 
				char * ph = buf + 1, * pt = ph;
				while(*pt != ' ' || *(pt-1) == '\\'){
					pt++;
				}
				*pt = '\0';
				m_ochlogdrv.push_back(string(ph));

				// extract threashold 
				ph = pt + 1;
				float th = (float) atof(ph);
				if(th <= 0. || th >= 1.){
					m_ochlogdrv.clear();
					m_thdrvuse.clear();
					cerr << "Irregal file format " << m_fochlogpath << endl;
					cerr << "File should include a list of \"<path> <threashold>\" pairs." << endl; 
					m_bochlog = false;
					return false;
				}

				m_thdrvuse.push_back(th);
			}
		}else{
			cerr << "Failed to open file includes ochlog paths " << m_fochlogpath << endl;
			m_bochlog = false;
			return false;
		}
	}catch(...){
		cerr << "Failed to load ochlog paths from " << m_fochlogpath << endl;
		m_bochlog = false;
		return false;
	}
	return true;
}

// Opens new ochlog file on available drive in the given list.
// If the function fails, the logging mode is disabled by deasserting m_bochlog.
bool f_base::open_ochlogfile()
{
	if(m_ochlogout.is_open())
		m_ochlogout.close();

	float drvuse;

	// find and check the use of the new drive.
	m_curochlogdrv = -1;
	do{
		m_curochlogdrv++;
		if(m_curochlogdrv >= m_ochlogdrv.size()){
			cerr << "Filter " << m_name << " ran out the output channel logging drives." << endl;
			m_bochlog = false;
			return false;
		}
		drvuse = getDrvUse(m_ochlogdrv[m_curochlogdrv].c_str());
	}while(drvuse >= m_thdrvuse[m_curochlogdrv]);

	m_drvuseprev = drvuse;

	// opening journal file.
	ofstream fochjournal;
	if(m_fjochlog[0] == '\0'){
		snprintf(m_fjochlog, 1023, "%s.jochlog", m_name);
	}
	
	fochjournal.open(m_fjochlog, ios_base::app);
	if(!fochjournal.is_open()){
		cerr << "Failed to open journal file " << m_fjochlog << endl;
		m_bochlog = false;
		return false;
	}

	// opening new ochlog file.
	char buf[1024];
	snprintf(buf, 1023, "%s/%s_%lld.ochlog", 
		m_ochlogdrv[m_curochlogdrv].c_str(), m_name, m_cur_time);

	m_ochlogout.open(buf, ios_base::binary);
	if(!m_ochlogout.is_open()){
		cerr << "Failed to open new log file " << buf << endl;
		m_bochlog = false;
		fochjournal.close();
		return false;
	}

	// recording time of the log file and the file path to the journal 
	fochjournal << m_cur_time << endl << buf << endl;
	fochjournal.close();

	return true;
}

// Logging output channel data. The data is recorded in the ochlog file with binary mode. 
// Each output channel has the write method which seriarizes their data. Of course, corresponding
// read method is used in the replay mode. 
// Therefore the data at a cycle should be <time:8byte> <och #0> <och #1> .... <och #n>, and each <och #i> 
// depends on the implementation of the read/write method in each channel.
// The drive use is periodically checked according to the cycle specified in m_tdrvchk (10 sec in default).
// The speed consuming drives are recoreded, and I intended to control the checking period according to this.
// But still not impelemented.
// This function is called immediatelly after calling proc(). Note that, the tran() method has not been called yet.
void f_base::logoch()
{
	if(m_bochlog){
		// check free disk
		if(m_tnextdrvchk > m_cur_time){
			float drvuse = getDrvUse(m_ochlogdrv[m_curochlogdrv].c_str());
			//  if not free, create and open log file in the next candidate drive
			if(drvuse > m_thdrvuse[m_curochlogdrv]){
				// if the drive use is larger than the threashold, open new log file on the new drive available.
				if(!open_ochlogfile()){
					return;
				}
			}else{
				// updating speed of drive use
				m_spddrvuse = (float) (0.9 *  m_spddrvuse + 0.1 * (drvuse - m_drvuseprev));
			}

			m_tnextdrvchk += (long long) m_tdrvchk;
		}

		if(m_ochlogout.is_open()){
			m_ochlogout.write((const char*)&m_cur_time, sizeof(long long));
			for(int i = 0; i < m_chout.size(); i++)
				m_chout[i]->write(this, m_ochlogout, m_cur_time);
		}else{
			if(!open_ochlogfile()){
				return;
			}
		}
	}
}

// Replaying ochlogfile. This method is called instead of proc(). Therefore, the init_run() and init_destroy()
// is not called if the m_bochrep is asserted. 
// Journal file is used to seek the file contains the record of the current time, then the record is found 
// merely feeding the data until encountering the current time recoord.
bool f_base::repoch()
{
	// seek log file related to the time
	if(m_bochrep){
		long long ltime;
		do{
			// seeking log file.
			if(m_ochlogin.eof()){
				m_ochlogin.close();
			}

			if(m_ochlogin.is_open()){
				if(m_cur_time_rec == -1){
					m_ochlogin.read((char*) &m_cur_time_rec, sizeof(long long));
					//cout << "Recorded time " << m_cur_time_rec << " Current time " << m_cur_time << endl;
				}

				if(m_cur_time_rec < 0){
					m_ochlogin.close();
					break;
				}

				ltime = m_cur_time_rec;
				
				if(ltime <= m_cur_time){
					for(int i = 0; i < m_chout.size(); i++)
						m_chout[i]->read(this, m_ochlogin, m_cur_time);
					m_cur_time_rec = -1;
				}
			}else{
				ifstream fochjournal;
				
				if(m_fjochlog[0] == '\0')
					snprintf(m_fjochlog, 1023, "%s.jochlog", m_name);
				fochjournal.open(m_fjochlog);
				if(!fochjournal.is_open()){
					cerr << "Failed to open journal file " << m_fjochlog << endl;
					return false;
				}

				char buf[1024];
				while(1){
					if(fochjournal.eof()){
						cerr << "Ochlog in " << m_name << " is end." << endl;
						return false;
					}

					fochjournal.getline(buf, 1024);
					ltime = atoll(buf);
					if(ltime > m_tcurochlog)
						break;
				}
				fochjournal.getline(buf, 1024);
				m_ochlogin.open(buf, ios_base::binary);
				//cout << "Opening ochlog " << buf <<endl;
				m_tcurochlog = ltime;
			}
		}while(m_cur_time > ltime);
	}
	return true;
}
