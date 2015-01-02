#include <iostream>
#include "c_aws_cmd.h"

class c_fpls: public c_aws_cmd
{
private:
  /////////////////////////////// helper functions
  // issue "finf" to get the number of parameters 
  int get_num_fpars(const char * fname);
  // issue "finf n $ichannel" to get channel information.
  bool get_fpar_inf(const char * fname, int ipar);
public:
  virtual bool send_cmd(int argc, char ** argv);
};

bool c_fpls::send_cmd(int argc, char ** argv)
{
  int num_fpars = get_num_fpars(argv[1]);
  if(num_fpars < 0){
    cerr << "The number of parameters in the filter \"" 
	 << argv[1] << "\" cannot be retrieved." << endl;
    return false;
  }
  cout << "#Parameter List in filter \"" << argv[1] 
       << "\" (" << num_fpars << ") parameters found." << endl;

  for(int ifpar = 0; ifpar < num_fpars; ifpar++){
    if(!get_fpar_inf(argv[1], ifpar)){
      cerr << "Failed to get " << ifpar 
	   << "th parameter in filter \"" << argv[1] << "\"." << endl;
    }
  }
  return true;
}

int c_fpls::get_num_fpars(const char * fname)
{
  // generating command string
  sprintf(buf, "finf %s", fname);

  // sending command
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    return -1;
  }

  // recieving result
  char * res;
  vector<char*> finf;
  if(res = recv_result()){
    if(!split_cmd_tok(res, finf)){
      cerr << "Failed to retrieve filter information." << endl;
      return -1;
    }
    int num_fpars = atoi(finf[2]);
    return num_fpars;
  }
  return -1;
}

bool c_fpls::get_fpar_inf(const char * fname, int ichannel)
{
  sprintf(buf, "fpar %s %d", fname, ichannel);
  int ret = send(sock, buf, CMD_LEN, 0);
  if(ret == -1){
    cerr << "Failed to send command." << endl;
    return -1;
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
  c_fpls fpls;

  if(!fpls.send_cmd(argc, argv))
    return 1;

  return 0;
}
