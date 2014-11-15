

//////////////////////////////////////////////////////////// class c_aws
// c_aws is the main class of automatic watch system.
// main() instantiate a c_aws objects and runs c_aws::run(). 
class c_aws: public CmdAppBase
{
protected:
	static const char * m_str_cmd[CMD_UNKNOWN];
	s_cmd m_cmd;
	pthread_mutex_t m_mtx;
	pthread_cond_t m_cnd_ret;
	pthread_cond_t m_cnd_none;

	int m_cmd_port;
	char * m_working_path;

#ifdef _WIN32
	WSAData m_wsad;
#endif

	// for remote command processor
	vector<c_rcmd*> m_rcmds;

	int m_time_rate;

	// start time and end time
	long long m_start_time, m_end_time;

	// current time
	long long m_time;

	// time zone in minute
	int m_time_zone_minute;

	// cycle time
	long long m_cycle_time; // in 100ns

	vector<f_base *> m_filters;
	vector<ch_base *> m_channels;

	bool m_blk_cmd;
	void proc_command();

	// getting a pointer of a channel object by its name.
	ch_base * get_channel(const char * name);

	// getting a pointer of a filter object by its name
	f_base * get_filter(const char * name);

	void clear();

	// flag for exiting function run()
	bool m_exit;

	bool m_bonline;

	// create and add channel
	bool add_channel(s_cmd & cmd);

	// create and add filter
	bool add_filter(s_cmd & cmd);

	// check filter graph
	bool check_graph();

	bool handle_run(s_cmd & cmd);
	bool handle_stop();

public:
	c_aws(int argc, char ** argv);
	virtual ~c_aws();

	bool push_command(const char * cmd_str, char * ret_str, bool & ret_stat);

	bool is_exit(){
		return m_exit;
	}

	virtual bool main();

};

extern bool g_kill;
