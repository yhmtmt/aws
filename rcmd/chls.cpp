#include <iostream>
#include "c_aws_cmd.h"

class c_chls: public c_aws_cmd
{
private:
  /////////////////////////////// helper functions
  // issue "finf" to get the number of channels
  int get_num_channels();
  // issue "finf n $ichannel" to get channel information.
  bool get_channel_inf(int ichannel);
public:
  virtual bool send_cmd(int argc, char ** argv);
};

bool c_chls::send_cmd(int argc, char ** argv)
{
  int num_channels = get_num_channels();
  if(num_channels < 0){
    cerr << "Channel List cannot be enumerated." << endl;
    return false;
  }
  cout << "#Channel List (" << num_channels << ") channels found." << endl;

  for(int ichannel = 0; ichannel < num_channels; ichannel++){
    if(!get_channel_inf(ichannel)){
      cerr << "Failed to list channels." << endl;
    }
  }
  return true;
}

int c_chls::get_num_channels()
{
  // generating command string
  sprintf(buf, "chinf");

  // sending command
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    return -1;
  }

  // recieving result
  const char * res;
  if(res = recv_result()){
    int num_channels = atoi(res);
    return num_channels;
  }
  return -1;
}

bool c_chls::get_channel_inf(int ichannel)
{
  sprintf(buf, "chinf n %d", ichannel);
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    return false;
  }

  const char * res;
  if(res = recv_result()){
    cout << res << endl;
    return true;
  }
  return false;
}

int main(int argc, char ** argv)
{
  c_chls chls;

  if(!chls.send_cmd(argc, argv))
    return 1;

  return 0;
}
