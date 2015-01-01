#include <iostream>
#include "c_aws_cmd.h"

class c_fls: public c_aws_cmd
{
private:
  /////////////////////////////// helper functions
  // issue "finf" to get the number of filters
  int get_num_filters();
  // issue "finf n $ifilter" to get filter information.
  bool get_filter_inf(int ifilter);
public:
  virtual bool send_cmd(int argc, char ** argv);
};

bool c_fls::send_cmd(int argc, char ** argv)
{
  int num_filters = get_num_filters();
  if(num_filters < 0){
    cerr << "Filter List cannot be enumerated." << endl;
    return false;
  }
  cout << "#Filter List (" << num_filters << " filters found." << endl;

  for(int ifilter = 0; ifilter < num_filters; ifilter++){
    if(!get_filter_inf(ifilter)){
      cerr << "Failed to list filters." << endl;
    }
  }
  return true;
}

int c_fls::get_num_filters()
{
  // generating command string
  sprintf(buf, "finf");

  // sending command
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    return -1;
  }

  // recieving result
  const char * res;
  if(res = recv_result()){
    int num_filters = atoi(res);
    return num_filters;
  }
  return -1;
}

bool c_fls::get_filter_inf(int ifilter)
{
  sprintf(buf, "finf n %d", ifilter);
  const char * res;
  if(res = recv_result()){
    cout << res << endl;
    return true;
  }
  return false;
}

int main(int argc, char ** argv)
{
  c_fls fls;

  if(!fls.send_cmd(argc, argv))
    return 1;

  return 0;
}
