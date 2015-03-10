#include <iostream>
#include <fstream>
using namespace std;
#include <string.h>

bool handle_txlog(const char * fname)
{
  ifstream file;
  file.open(fname, ios_base::binary);
  if(!file.is_open()){
    cerr << "Cannot open file " << fname << "." << endl; 
    return false;
  }
  char buf[256];
  while(!file.eof()){
    long long t;
    unsigned char len;
    unsigned char dst;
    file.read((char*)&t, sizeof(long long));
    file.read((char*)&len, sizeof(unsigned char));
    file.read((char*)&dst, sizeof(unsigned char));
    file.read((char*)buf, 128);
    buf[len] = '\0';
    cout << "time: " << t << " len: " << (int) len
	 << "msg: " << buf << endl;
  }
  return true;
}

bool handle_rxlog(const char * fname)
{
  ifstream file;
  file.open(fname, ios_base::binary);
  if(!file.is_open()){
    cerr << "Cannot open file " << fname << "." << endl; 
    return false;
  }
  char buf[256];
  while(!file.eof()){
    long long t;
    unsigned char len, src, rep0, rep1, power;
    file.read((char*)&t, sizeof(long long));
    file.read((char*)&len, sizeof(unsigned char));
    file.read((char*)&src, sizeof(unsigned char));
    file.read((char*)&rep0, sizeof(unsigned char));
    file.read((char*)&rep1, sizeof(unsigned char));
    file.read((char*)&power, sizeof(unsigned char));
    file.read((char*)buf, 128);
    buf[len] = '\0';
    cout << "time: " << t 
	 << " len: " << (int) len
	 << " src: " << (int) src
	 << " rep0: " << (int) rep0
	 << " rep1: " << (int) rep1
	 << " power: " << (int) power
	 << "msg: " << buf << endl;
  }

  return true; 
}

// Description:
// Input: fep01 log, gps log, flight log
// Output: time x power, time x error, distance x power, distance x error
//         attitude x power, attitude x error, power x error
// for error stat, binning steps should be determined
// 
// Experiment:
// Parameters: speed, distance, height, attitude, direction
// line (fix direction, height), circle(fix distance, height), 

int main(int argc, char ** argv)
{
  if(argc != 3){
    cerr << "fep01log <tx or rx> <log file>" << endl;
    return 1;
  }

  if(strcmp(argv[1], "tx") == 0){
    // tx log <time> <len> <msg>
    if(!handle_txlog(argv[2]))
      return 1;
    return 0;
  }

  if(strcmp(argv[1], "rx") == 0){
    // rx log <time> <len> <src> <rep0> <rep1> <power> <msg> 
    if(!handle_rxlog(argv[2]))
      return 1;
    return 0;
  }

  return 0;
}
