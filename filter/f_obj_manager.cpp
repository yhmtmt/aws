// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_obj_manager.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_obj_manager.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_obj_manager.h.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include <opencv2/opencv.hpp>

using namespace cv;


#include "f_obj_manager.h"

f_obj_manager::f_obj_manager(const char * name): f_base(name), m_ais_obj(NULL)
{
	register_fpar("ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
}

f_obj_manager::~f_obj_manager()
{
}

bool f_obj_manager::init_run()
{
	return true;
}

void f_obj_manager::destroy_run()
{
}

bool f_obj_manager::proc()
{

	return true;
}
