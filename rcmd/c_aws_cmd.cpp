#include <iostream>
#include "c_aws_cmd.h"

////////////////////////////////////////////////// helper function
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

//////////////////////////////////////////////////// c_aws_cmd
c_aws_cmd::c_aws_cmd()
{
  cf.open(".aws");
  if(!cf.is_open()){
    cerr << "Configuration file \".aws\" does not found." << endl; 
    exit(1);
  }
  
  // find rcmd section in the .aws file.
  vector<char *> tok;
  bool brcmd = false;
  while(!cf.eof()){
    cf.getline(buf, CMD_LEN);
    split_cmd_tok(buf, tok);
    if(strcmp(tok[0], "rcmd") == 0){
      addr.sin_port = htons((unsigned short)atoi(tok[2]));
      addr.sin_family = AF_INET;
      addr.sin_addr.s_addr = inet_addr(tok[1]);
      brcmd = true;
    }
  }
  
  if(brcmd == false){
    cerr << "rcmd section was not found in .aws file." << endl;
    exit(1);
  }

  sock = socket(AF_INET, SOCK_STREAM, 0);
  
  int ret = connect(sock, (sockaddr*)&addr, sizeof(addr));
  if(ret == -1){
    cerr << "fail to connect." << endl;
    exit(1);
  }
}

c_aws_cmd::~c_aws_cmd()
{
  int ret;
  // finish the command session ("eoc" command is sent")
  buf[0] = 'e'; buf[1] = 'o'; buf[2] = 'c'; buf[3] = '\0';
  ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send end of command message." << endl;
  }
  close(sock);
}
