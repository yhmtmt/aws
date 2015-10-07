#ifndef _F_BASE_H_
#define _F_BASE_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_base.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_base.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_base.h.  If not, see <http://www.gnu.org/licenses/>. 

// README: The basis of the filter class. All the filters should be the 
// subclass of f_base. Filter classes are to be registered to the factory 
// function f_base::create to create its object by name. Then the filter
// should override followoings.
//
// * f_base::proc is the main function which is executed synchronous to the clock. you can 
// implements any functions using data in input channels and transfer the results
// to the next filter via output channels. Or you can control the latency of the proc
// by setting m_intvl with cmd_proc.
// * f_base::cmd_proc is the processing function for filter command. by overriding
// the function, you can change the internal parameters of the filter you desgined.
// cmd_proc is called by c_aws for each cycle. because f_base members 
// are manipulated with the function, you should execute the f_base::cmd_proc
// in your original cmd_proc.
// * f_base::check is called by framework before the filter graph is started.
// if at least one filter failed the method, graph wont be started. you can use
// the function to check validity of the input and output channels, or other 
// states of your filters.
// * f_base::seek is called meaningful only if c_aws is in the offline mode. 
// the function is assumed to seek for the state at the time specified.
// you should implement this function if you assume that the filter is used 
// in the offline mode.

#include "../util/aws_stdlib.h"
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"
#include "../command.h"
#include "../channel/ch_base.h"

class f_base;

#define FILE_FERR_LOG "ferr.log"
#define SIZE_FERR_BUF 64
#define SIZE_FERR_MSG 1024

#define FERR_SPROT_WINDOW_SNDBBM 0
#define FERR_SPROT_WINDOW_OSPOS 1
#define FERR_SPROT_WINDOW_FTRAIL_LOG_OPEN 2

#define FERR_AVT_CAM 32
#define FERR_AVT_CAM_INIT (FERR_AVT_CAM + 1)
#define FERR_AVT_CAM_OPEN (FERR_AVT_CAM + 2)
#define FERR_AVT_CAM_ALLOC (FERR_AVT_CAM + 3)
#define FERR_AVT_CAM_CLOSE (FERR_AVT_CAM + 4)
#define FERR_AVT_CAM_CFETH (FERR_AVT_CAM + 5)
#define FERR_AVT_CAM_START (FERR_AVT_CAM + 6)
#define FERR_AVT_CAM_STOP (FERR_AVT_CAM + 7)
#define FERR_AVT_CAM_CH (FERR_AVT_CAM + 8)
#define FERR_AVT_CAM_DATA (FERR_AVT_CAM + 9)

#define FERR_TRN_IMG 64
#define FERR_TRN_IMG_CHAN (FERR_TRN_IMG + 1)
#define FERR_TRN_IMG_SOCK_SVR (FERR_TRN_IMG + 2)
#define FERR_TRN_IMG_SOCK_SVR_CON (FERR_TRN_IMG + 3)

typedef f_base * (*CreateFilter)(const char * name);
template <class T> f_base * createFilter(const char * name)
{
	return reinterpret_cast<f_base*>(new T(name));
}

class c_aws;

typedef map<const char *, CreateFilter, cmp> FMap;

class f_base
{
protected:
	static FMap m_fmap;
	// calling register_factor<T>(tname) to register all filters to be used.
	static void register_factory();

	// registering filter classes to the factory
	template <class T> static void register_factory(const char * tname)
	{
		m_fmap.insert(FMap::value_type(tname, createFilter<T>));
	}

public:
	// factory function
	static f_base * create(const char * tname, const char * fname);

	// initialize mutex and signal objects. this is called by the c_aws constructor
	static c_aws * m_paws;
	static void init(c_aws * paws);

	// uninitialize mutex and signal objects. called by the c_aws destoractor
	static void uninit();
		
protected:
	char * m_name; // filter name
public:
	const char * get_name()
	{
		return m_name;
	};

	///////////////////////////////////////// channel and the methods
protected:
	// filter input and output channels
	vector<ch_base *> m_chin;
	vector<ch_base *> m_chout;

	// only output log is recorded
	static vector<string> m_logdrv;
	static vector<long long> m_capdrv;
	bool m_bochlog;
	bool m_brepochlog;
	ofstream m_ochlogout;
	ifstream m_ochlogin;

	void logoch(){
		if(m_bochlog){
			// check free disk
			//  if not free, create and open log file in the next candidate drive
			//         if successfully done, link is recorded older file and 
			if(m_ochlogout.is_open()){
				for(int i = 0; i < m_chout.size(); i++)
					m_chout[i]->write(m_ochlogout, m_cur_time);
			}
		}
	}

	void replayoch(){
		// seek log file related to the time
		if(m_brepochlog){
			if(m_ochlogin.is_open()){
				for(int i = 0; i < m_chout.size(); i++)
					m_chout[i]->read(m_ochlogin, m_cur_time);
			}
		}
	}

public:
	void set_ichan(ch_base * pchan){
		m_chin.push_back(pchan);
	}

	void set_ochan(ch_base * pchan){
		m_chout.push_back(pchan);
	}

protected:
	///////////////////////////////////////// parameter table and the methods
	// filter parameter structure 
	struct s_fpar{ 
		const char * name; // name of the parameter
		const char * explanation; // explanation of the parameter
		enum e_par_type{
			F64, S64, U64, F32, S32, U32, S16, U16, S8, U8, BIN, CSTR, ENUM, UNKNOWN
		} type; 	// parameter type

		union{
			double * f64;
			long long * s64;
			unsigned long long * u64;
			float * f32;
			int * s32;
			unsigned int * u32;
			short * s16;
			unsigned short * u16;
			char * s8;
			unsigned char * u8;
			char * cstr;
			bool * bin;
		}; // parameter variable (shared field)

		int len; // length of the string (only for CSTR) or 
				// lengtho of the string list of ENUM
		const char ** str_enum; // string list for ENUM

		// A value specified in valstr as a string is decoded and
		// stored into variable of corresponding type.
		// only type CSTR checks the length of the buffer. 
		bool set(const char * valstr);

		// return paramter to valstr as a null terminated string
		// valstr should be the buffer with length sz, and
		// function returns false if the length of return string exceeds
		// sz.
		bool get(char * valstr, size_t sz);

		// get_info() returns parameter explanation.
		void get_info(s_cmd & cmd);
	};

	vector<s_fpar> m_pars; // parameter table
	
	// helper function for registering parameters onto the table

	void register_fpar_common(const char * name, const char * expl, s_fpar & par){
		par.name = name;
		par.explanation = expl;
		m_pars.push_back(par);
	}

	void register_fpar(const char * name, float * f32, const char * expl = NULL){
		s_fpar par;
		par.f32 = f32;
		par.type = s_fpar::F32;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, double * f64, const char * expl = NULL){
		s_fpar par;
		par.f64 = f64;
		par.type = s_fpar::F64;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, long long * s64, const char * expl = NULL){
		s_fpar par;
		par.s64 = s64;
		par.type = s_fpar::S64;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, unsigned long long * u64, const char * expl = NULL){
		s_fpar par;
		par.u64 = u64;
		par.type = s_fpar::U64;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, int * s32, const char * expl = NULL){
		s_fpar par;
		par.s32 = s32;
		par.type = s_fpar::S32;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, unsigned int * u32, const char * expl = NULL){
		s_fpar par;
		par.u32 = u32;
		par.type = s_fpar::U32;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, short * s16, const char * expl = NULL){
		s_fpar par;
		par.s16 = s16;
		par.type = s_fpar::S16;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, unsigned short * u16, const char * expl = NULL){
		s_fpar par;
		par.u16 = u16;
		par.type = s_fpar::U16;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, char * s8, const char * expl = NULL){
		s_fpar par;
		par.s8 = s8;
		par.type = s_fpar::S8;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, unsigned char * u8, const char * expl = NULL){
		s_fpar par;
		par.u8 = u8;
		par.type = s_fpar::U8;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, bool * bin, const char * expl = NULL){
		s_fpar par;
		par.bin = bin;
		par.type = s_fpar::BIN;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, char * cstr, int len, const char * expl = NULL){
		s_fpar par;
		par.cstr = cstr;
		par.type = s_fpar::CSTR;
		par.len = len;
		register_fpar_common(name, expl, par);
	}

	void register_fpar(const char * name, int * e, int len , const char ** strlst, const char * expl = NULL){
		s_fpar par;
		par.s32 = e;
		par.type = s_fpar::ENUM;
		par.len = len;
		par.str_enum = strlst;
		register_fpar_common(name, expl, par);
	}

	// find parameter index by its name
	int find_par(const char * parstr){
		for(int ipar = 0; ipar < m_pars.size(); ipar++){
			if(strcmp(m_pars[ipar].name, parstr) == 0){
				return ipar;
			}
		}		
		return -1;
	}

	////////////////////////////////////////////////////////////// main thread 
protected:
	// thread object.
	pthread_t m_fthread;
	// thread body. called with m_fthread
	static void * fthread(void * ptr);

	bool m_bactive; // if it is true, filter thread continues to loop
	bool m_bstopped; //true indicates filter is stopped. 
	// count number of proc() executed
	long long m_count_pre, m_count_post;
	int m_cycle;
	long long m_count_proc;
	double m_proc_rate;
	int m_max_cycle;

	// mutex and signal for clocking
	static pthread_mutex_t m_mutex; 
	static pthread_cond_t m_cond;	

	// swap back buffer of output channels by calling ch_base::tran function.
	// some channel uses back buffer and the swapping timing is immediately after the proc function end.
	void tran_chout(){
		for(vector<ch_base*>::iterator itr = m_chout.begin(); itr != m_chout.end(); itr++){
			if(*itr == NULL)
				continue;
			(*itr)->tran();
		}
	}

	virtual bool init_run()
	{
		return true;
	}

	virtual bool seek(long long seek_time)
	{
		return true;
	}

	virtual void destroy_run()
	{
	}
public:	

	virtual bool check()
	{
		return true;
	}

	// invoke main thread.
	virtual bool run(long long start_time, long long end_time)
	{
		if(!init_run()){
			return false;
		}

		m_prev_time = start_time;

		if(!seek(start_time)){
			return false;
		}

		m_bactive = true;
		m_bstopped = false;
		m_count_proc =  0;	
		m_max_cycle = 0;
		m_cycle = 0;
		m_count_pre = m_count_post = m_count_clock;
		if(!is_main_thread())
			pthread_create(&m_fthread, NULL, fthread, (void*) this);
		return true;
	}

	// stop main thread. the function is called from c_aws until the thread stops.
	virtual bool stop()
	{	
		m_bactive= false;
		if(m_bstopped || is_main_thread()){
		  destroy_run();
		  return true;
		}
		return false;
	}
	
	void runstat(){
	  cout << get_name() << " stopped." << endl;
	  cout << "Processing rate was " << m_proc_rate;
	  cout << "(" << m_count_proc << "/" << m_count_clock << ")" << endl;
	  cout << "Number of max cycles was " << m_max_cycle << endl;
	}

	// check the filter activity condition
	virtual bool is_active(){
		return m_bactive;
	}

	void calc_time_diff(){
		m_time_diff = m_cur_time - m_prev_time;
		m_prev_time = m_cur_time;
	}

	bool is_pause(){
		return m_time_diff == 0 && m_clk.is_pause();
	}

	// This is the function called in main thread when the filter should be executed in the main thread.
	// The filter to be executed in main thread should return true for the call of is_main_thread().
	// f_base::is_main_thread() returns false by default, please override it as you need.
	void fthread();

	virtual bool is_main_thread()
	{
		return false;
	}

	virtual bool proc() = 0;

	////////////////////////////////////////////////////// time and the methods
protected:
	// offset time for the filter. only for offline mode.
	long long m_offset_time;

	// cycle count the filter should be processed. cycle time is defined in c_aws.
	unsigned int m_intvl;

	// time in 10^-7 second. the variable is common for all filters
	static long long m_cur_time;

	long long m_prev_time;
	long long m_time_diff;

	static tmex m_tm;

	// time string 
	static char m_time_str[32];

	// clock counter
	static long long m_count_clock;

	// wait signal from aws main loop clocked with hardware timer.
	void clock_wait(){
		pthread_mutex_lock(&m_mutex);
		pthread_cond_wait(&m_cond, &m_mutex);
		pthread_mutex_unlock(&m_mutex);
	}

public:
	// reference clock 
	static c_clock m_clk;
	static int m_time_zone_minute;
	static int get_tz(){
		return m_time_zone_minute;
	}
	static void set_tz(int tz){
		m_time_zone_minute = tz;
	}

	void set_time(tmex & tm){m_clk.set_time(tm);};
	void set_time(long long & t){m_clk.set_time(t);};
	static void set_sys_time(){
#ifdef _WIN32
		SYSTEMTIME st;
		ZeroMemory(&st, sizeof(st));
		st.wDay = m_tm.tm_mday;
		st.wDayOfWeek = m_tm.tm_wday;
		st.wMonth = m_tm.tm_mon;
		st.wYear = m_tm.tm_year + 1900;
		st.wHour = m_tm.tm_hour;
		st.wMinute = m_tm.tm_min;
		st.wSecond = m_tm.tm_sec;
		st.wMilliseconds = m_tm.tm_msec;
		SetLocalTime(&st);
#else
		timespec ts;
		ts.tv_sec = m_cur_time / SEC;
		ts.tv_nsec = (m_cur_time - SEC * ts.tv_sec) * 100;
		clock_settime(CLOCK_REALTIME, &ts);
#endif
	}

	// clock signal issued by c_aws's main loop
	static void clock(long long cur_time);

	static void init_run_all(){
		m_count_clock = 0;
	}

	static const char * get_time_str(){
		return m_time_str;
	}

	static long long get_time(){
	  return m_cur_time;
	}
	
	void get_info(s_cmd & cmd, int ifilter){
		// currentlly returning filter name, id, number of parameters, number of input channels and output channels.
		snprintf(cmd.get_ret_str(), RET_LEN, "%s %d %d %d %d", m_name, ifilter, m_pars.size(), m_chin.size(), m_chout.size());
	}

	bool get_par_info(s_cmd & cmd){
		if(cmd.num_args == 2){ // if parameter index is not specified, the number of parameters is returned.
			snprintf(cmd.get_ret_str(), RET_LEN, "%d", m_pars.size());
			return false;
		}
		
		int ipar = atoi(cmd.args[2]);
		if(ipar >= m_pars.size()){
			snprintf(cmd.get_ret_str(), RET_LEN, "Filter %s does not have parameter id=%d", m_name, ipar);
			return false;
		}
		m_pars[ipar].get_info(cmd);
		return true;
	}

	bool get_par_info_by_fset(s_cmd & cmd){
		int ipar = find_par(cmd.args[2]);
		if(ipar < 0){		
			snprintf(cmd.get_ret_str(), RET_LEN, "Filter %s does not have parameter %s", m_name, cmd.args[2]);
			return false;
		}
		m_pars[ipar].get_info(cmd);
		return true;	
	}

	void set_offset_time(long long offset)
	{
		m_offset_time = offset;
	}

	const long long & get_offset_time()
	{
		return m_offset_time;
	}
	
	////////////////////////////////////////////// error log data and methods
protected:
	// filter error structure
	struct s_ferr{
		f_base * ptr; // filter caused error 
		char tstr[32]; // time string
		const char * fname; // file name
		int line; // line number in fname
		int code; // error code in the filter


		s_ferr(){
		}

		~s_ferr(){
		}

		void dump_err(ostream & out);
	};

	static ofstream m_file_err;
	static s_ferr m_err_buf[SIZE_FERR_BUF];
	static int m_err_head;
	static int m_err_tail;
	static pthread_mutex_t m_err_mtx;

	static void send_err(f_base * ptr, const char * fname, int line, int code);
public:
	static void flush_err_buf();
	virtual const char * get_err_msg(int code){
		return NULL;
	}

public:
	f_base(const char * name);
	virtual ~f_base();

	////////////////////////////////////////////////// command and the methods
protected:
	// mutex for command processing
	pthread_mutex_t m_mutex_cmd;
	pthread_cond_t m_cnd_cmd;
	bool m_cmd;
public:
	// lock for command processing mutex
	void lock_cmd(bool bcmd = false)
	{
		if(bcmd)
			m_cmd = true;

		pthread_mutex_lock(&m_mutex_cmd);
		while(m_cmd && !bcmd){
			pthread_cond_wait(&m_cnd_cmd, &m_mutex_cmd);
		}
	}

	// unlock for command processing mutex
	void unlock_cmd(bool bcmd = false)
	{
		pthread_mutex_unlock(&m_mutex_cmd);
		if(bcmd){
			pthread_cond_signal(&m_cnd_cmd);
			m_cmd = false;
		}
	}

	// interface for original command in each child class
	virtual bool cmd_proc(s_cmd & cmd);

	// set list of parameters
	bool set_par(s_cmd & cmd);

	// returns list of parameters as a string to cmd.ret.
	// function returns false if the return values exceed run out the buffer cmd.ret.
	bool get_par(s_cmd & cmd);
};

#include "../c_aws.h"
#endif
