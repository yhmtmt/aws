#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "util/aws_stdlib.h"

void * cmd_proc(void * pasw);
bool split_cmd_tok(char * cmd, vector<char *> & cmd_tok);

#define CMD_LEN 1024
#define RET_LEN 1023
#define CMD_ARGS 32
#define CMD_QUEUE 4

enum e_cmd {
	CMD_CHAN, CMD_FLTR, CMD_FCMD, CMD_FSET, CMD_FGET, 
	CMD_FINF, CMD_FPAR, CMD_CHINF, 
	CMD_GO, CMD_STOP,	CMD_QUIT, CMD_STEP, CMD_CYC,
	CMD_ONLINE, CMD_PAUSE, CMD_CLEAR, 
	CMD_RCMD, CMD_TRAT, CMD_UNKNOWN
};

enum e_cmd_stat{
	CS_NONE, CS_SET, CS_RET, CS_ERR
};

// command structure
struct s_cmd{
	e_cmd type;
	e_cmd_stat stat;
	int num_args;
	char * args[CMD_ARGS];
	char mem[CMD_LEN];
	char ret[RET_LEN + 1];
	void set_ret_stat(bool stat)
	{
		ret[0] = (stat ? 1 : 0);
	}

	char * get_ret_str(){
		return ret + 1;
	}
	s_cmd():type(CMD_UNKNOWN), stat(CS_NONE), num_args(0){};
	~s_cmd(){}
};

class c_aws;

// class for remote command processor 
// this class is instanciated when the rcmd command is issued.
// aws instantiate at least one object of this class.
class c_rcmd
{
private:
	c_aws * m_paws;			// pointer to the system
	SOCKET m_svr_sock;		// socket waiting for commands
	sockaddr_in m_svr_addr; // server address initiating socket
	sockaddr_in m_client;	// client address initiating socket
	pthread_t m_th_rcmd;	// command processing thread
	fd_set m_fdread;		// for use select() in recieving command
	fd_set m_fdwrite;		// for use select() in transmitting return value
	fd_set m_fderr;			// for use select() 
	timeval m_to;			// for use select()
	bool m_exit;			// thread termination flag

	char m_buf_recv[CMD_LEN]; // buffer for recieving command
	char m_buf_send[RET_LEN]; // buffer for return value 

	static void * thrcmd(void * ptr); // command processing thread function
	bool wait_connection(SOCKET & s); 
	bool wait_receive(SOCKET & s, char * buf, int & total);
	bool push_command(const char * cmd_str, char * ret_str, bool ret_stat);
	bool wait_send(SOCKET & s, char * buf);

public:
	c_rcmd(c_aws * paws, unsigned short port);
	~c_rcmd();

	bool is_exit();
};

#endif
