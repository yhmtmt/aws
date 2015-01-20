#include <iostream>
#include "../util/aws_sock.h"
#include "c_aws_cmd.h"

class c_awsevt: public c_aws_cmd
{
private:
  pthread_t thwait;
  const char * fname; // filter name
  bool bsuccess;
  const char * evtstr;
  unsigned short port;
  const char * host;
  sockaddr_in addr;

  const char * tstr; // time string in aws's [] format
  int num_itrs; // number of iteration
  double period; // pereiod in second
  timeval tout; // time out 

  /////////////////////////////// helper functions
  // thread event wait 
  static void * wait(void * ptr){
    SOCKET sock;
    sockaddr_in addr;
    c_awsevt * pawsevt = (c_awsevt*) ptr;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(pawsevt->port);

    //    cout << "Socket:" << sock << endl;
    //    cout << "Port:" <<  pawsevt->port << endl;
    //    cout << "Host:" << pawsevt->host << endl;

    set_sockaddr_addr(addr, NULL);
    if(::bind(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR){
      //      cerr << "Socket error" << endl;
      pawsevt->bsuccess = false;
      return NULL;
    }

    int len = 0;
    do{
      fd_set dread, derr;
      FD_ZERO(&dread);
      FD_ZERO(&derr);
      FD_SET(sock, &dread);
      FD_SET(sock, &derr);
      
      timeval tout = pawsevt->tout;
      int n = select((int)sock+1, &dread, NULL, &derr, &tout);
      
      if(n > 0){
	if(FD_ISSET(sock, &dread)){
	  len += recv(sock, pawsevt->buf + len, 32 - len, 0);
	}else if(FD_ISSET(sock, &derr)){
	  return NULL;
	}
      }else{ // timeout
	return NULL;
      }
    }while(len != 32);

    return NULL;
  }
  
public:
  virtual bool send_cmd(int argc, char ** argv);
};

bool c_awsevt::send_cmd(int argc, char ** argv)
{
  int iarg = 1;
  
  host = NULL;
  port = 0;
  evtstr = NULL;
  num_itrs = 1; // default iteration is 1
  tout.tv_sec = 5; // default tout is 5sec
  tout.tv_usec = 0;
  while(iarg < argc){
    if(strcmp(argv[iarg], "-f") == 0){
      iarg++;
      if(iarg == argc){
	return false;
      }
      fname = argv[iarg];
      iarg++;
    }else if(strcmp(argv[iarg], "-t") == 0){// absolute time in [] form
      iarg++;
      if(iarg == argc){ // argument is not specified
	return false;
      }
      tstr = argv[iarg];
      evtstr = "time";
      iarg++;
    }else if(strcmp(argv[iarg], "-p") == 0){ // period in second
      iarg++;
      if(iarg == argc){
	return false;
      }
      tstr = argv[iarg];
      evtstr = "period";
      iarg++;
    }else if(strcmp(argv[iarg], "-n") == 0){ // number of iteration
      iarg++;
      if(iarg == argc){
	return false;
      }
      num_itrs = atoi(argv[iarg]);
      iarg++;
    }else if(strcmp(argv[iarg], "-sock") == 0){ // socket host and port
      iarg++;
      if(iarg == argc){ // host address is not specified
	return false;
      }
      host = argv[iarg];
      iarg++;

      if(iarg == argc){ // port is not specified.
	return false;
      }
      port = (unsigned short) atoi(argv[iarg]);
      iarg++;
    }else if(strcmp(argv[iarg], "-to") == 0){ // time out value in second
      iarg++;
      if(iarg == argc){
	return false;
      }
      double to = atof(argv[iarg]);
      tout.tv_sec = (long) to;
      tout.tv_usec = (long)((to - (double) tout.tv_sec) * 1e6);
      iarg++;
    }
  }

  // waiting socket is not configured
  if(host == NULL || port == 0)
    return false;

  // invoke wait thread
  pthread_create(&thwait, NULL, wait, (void*) this);

  if(evtstr == NULL){ // if no event is specifed simply wait the event
    pthread_join(thwait, NULL);
    return true;
  }
  sprintf(buf, "fset %s type %s tstr %s itrs %d host %s port %d breg y", 
	  fname, evtstr, tstr, num_itrs, host, (int)port);
  cout << buf << endl;
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    pthread_join(thwait, NULL);
    return false;
  }

  const char * res = recv_result();
  if(res == NULL){
    pthread_join(thwait, NULL);
    return false;
  }

  pthread_join(thwait, NULL);
  cout << buf << endl;
  return true;
}

int main(int argc, char ** argv)
{
  c_awsevt awsevt;

  if(!awsevt.send_cmd(argc, argv))
    return 1;

  return 0;
}
