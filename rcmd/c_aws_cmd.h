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

int aws_cmd(int argc, char ** argv, const char * cmd);

class c_aws_cmd
{
 private:
  ifstream cf;
  char buf[CMD_LEN];
  sockaddr_in addr;
  SOCKET sock;
 public:
  c_aws_cmd();
  ~c_aws_cmd();

  virtual bool send_cmd(int argc, const char * argv) = 0;
};
