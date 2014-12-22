#include <iostream>

//////////////////////////////////////////////////// c_aws_cmd
c_asw_cmd::c_aws_cmd()
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
  close(sock);  
}
