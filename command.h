#ifndef _COMMAND_H_
#define _COMMAND_H_

bool split_cmd_tok(char * cmd, vector<char *> & cmd_tok);

#define CMD_LEN 1024
#define RET_LEN 1023
#define CMD_ARGS 32

enum e_cmd {
	CMD_CHAN, CMD_FLTR, CMD_FCMD, CMD_FSET, CMD_FGET, 
	CMD_FINF, CMD_FPAR, CMD_CHINF, CMD_GO, CMD_STOP, 
	CMD_QUIT, CMD_STEP, CMD_CYC, CMD_PAUSE, 
	CMD_CLEAR, CMD_RCMD, CMD_TRAT, CMD_CHRM, CMD_FRM, 
	CMD_CD, CMD_TIME,
	CMD_UNKNOWN
};

e_cmd cmd_str_to_id(const char * cmd_str);

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
#endif
