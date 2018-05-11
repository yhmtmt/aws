// Copyright(c) 2018 Yohei Matsumoto,  All right reserved. 

// f_router.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_router.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_router.h  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cstdio>
#include <cstring>
#include <cmath>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;

#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;


#include "f_router.h"

f_router::f_router(const char * name) : f_base(name), ch_state(NULL), ch_wp(NULL), ch_map(NULL), ch_route(NULL)
{
	register_fpar("ch_state", (ch_base**)&ch_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_wp", (ch_base**)&ch_wp, typeid(ch_wp).name(), "Waypoint channel");
	register_fpar("ch_map", (ch_base**)&ch_map, typeid(ch_map).name(), "Map channel");
	register_fpar("ch_route", (ch_base**)&ch_route, typeid(ch_route).name(), "Route channel");
}

f_router::~f_router()
{

}

bool f_router::init_run()
{
	return true;
}


void f_router::destroy_run()
{
}

bool f_router::proc()
{
	return true;
}