#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;
#include "CmdAppBase/CmdAppBase.h"
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

bool g_kill;
//#ifdef _WIN32
//int _tmain(int argc, _TCHAR* argv[])
//#else
int main(int argc, char ** argv)
//#endif
{
	c_aws aws(argc, argv);
	
	// invoke command thread
//	thread th_cmd(cmd_proc, &aws);
//	pthread_t th_cmd;
//	pthread_create(&th_cmd, NULL, cmd_proc, &aws);

	aws.run();
	g_kill = true;
//	th_cmd.join();
	return 0;
}
