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
bool split_cmd_tok(char * cmd, vector<char *> & cmd_tok);
int aws_cmd(int argc, char ** argv, const char * cmd);

class c_aws_cmd
{
 protected:
  ifstream cf;
  char buf[CMD_LEN];
  sockaddr_in addr;
  SOCKET sock;
 public:
  c_aws_cmd();
  ~c_aws_cmd();

  virtual bool send_cmd(int argc, char ** argv) = 0;
  char * recv_result(){
    memset(buf, 0, CMD_LEN);
    recv(sock, buf, CMD_LEN, 0);
    if(buf[0])
      return &buf[1];
    return NULL;
  }
};
