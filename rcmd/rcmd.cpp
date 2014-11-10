#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#define SOCKET int

#ifndef SOCKET_ERROR 
#define SOCKET_ERROR (-1)
#endif
#define CMD_LEN 1024


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

int main(int argc, char ** argv)
{
  ifstream cf(".aws");
  char buf[CMD_LEN];
  sockaddr_in addr;
  SOCKET sock;

  if(!cf.is_open()){
    cerr << "Configuration file \".aws\" does not found." << endl; 
    return 1;
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
      cout << "connecting to " << tok[1] << ":" << tok[2] << endl;
    }
  }
  
  if(brcmd == false){
    cerr << "rcmd section is not found in .aws file." << endl;
    return 1;
  }

  char * ptr = buf;
  for(int iarg = 1; iarg < argc; iarg++){
    for(int i = 0; (*ptr = argv[iarg][i]) != '\0'; i++, ptr++);
    *ptr = ' ';
  }
  *ptr = '\0';

  sock = socket(AF_INET, SOCK_STREAM, 0);
  
  cout << "trying connection." << endl;
  int ret = connect(sock, (sockaddr*)&addr, sizeof(addr));
  if(ret == -1){
    cerr << "failed to connect." << endl;
    return 1;
  }

  cout << "sending " << buf << endl;

  ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "failed to send data." << endl;
    return 1;
  }

  cout << "waiting reply." << endl;
  recv(sock, buf, CMD_LEN, 0);
  cout << buf << endl;

  close(sock);

  return 0;
}
