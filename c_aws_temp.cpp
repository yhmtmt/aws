#include "stdafx.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <queue>
using namespace std;

#ifdef _WIN32
#include <direct.h>
#endif

#include "util/aws_stdlib.h"
#include "util/aws_sock.h"
#include "util/aws_thread.h"

#include "util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;
#include "channel/ch_base.h"
#include "filter/f_base.h"
#include "command.h"
#include "c_aws.h"

void c_aws::print_title()
{
  if(name_app)
    cout << name_app;
  else
    cout<< "nanashi";
  
  cout << " Ver." << ver_main << "." << ver_sub;
  cout << " (built " << __DATE__ << " " << __TIME__ << ")" << endl;
  cout << "Copyright (c) " << year_copy << " " << name_coder << " All Rights Reserved" << endl;
  if(contact)
    cout << contact << endl;
}
