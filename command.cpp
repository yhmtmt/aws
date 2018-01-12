#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// command.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// command.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with command.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <errno.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include "command.h"

const char * str_cmd[CMD_UNKNOWN] = {
	"channel", "filter", "fcmd", "fset", "fget", 
	"finf", "fpar", "chinf", "go", "stop", "quit",
	"step","cyc", "pause","clear", "rcmd", 
	"trat", "chrm", "frm", "awscd", "awstime"
};

e_cmd cmd_str_to_id(const char * cmd_str)
{
	e_cmd type = CMD_UNKNOWN;
	for(int icmd = 0; icmd < (int) CMD_UNKNOWN; icmd++){
		if(strcmp(cmd_str, str_cmd[icmd]) == 0){
			type = (e_cmd) icmd;
			break;
		}
	}
	return type;
}

bool split_cmd_tok(char * cmd, vector<char *> & cmd_tok)
{
	int i = 0;
	bool seek_head = true;
	cmd_tok.clear();

	while(cmd[i] != '\0'){
		if(i == CMD_LEN) // no termination character found
			return false;
		if(cmd[i] == ' ' || cmd[i] == '\t'){ // space and tab is a delimiter
			cmd[i] = '\0';
			seek_head = true;
		}else if(seek_head){
			cmd_tok.push_back(&cmd[i]);
			seek_head = false;
		}

		i++;
	}

	return true;
}


