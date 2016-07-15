// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_estate.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_estate.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_estate.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include "stdafx.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;


#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_coord.h"

#include "f_estate.h"

f_estate::f_estate(const char * name) :f_base(name), m_ch_state(NULL), m_ch_estate(NULL)
{
	register_fpar("ch_state", (ch_base**)&m_ch_state, typeid(ch_state).name(), "State channel");
	register_fpar("ch_estate", (ch_base**)&m_ch_estate, typeid(ch_estate).name(), "Estimated state channel");
}

f_estate::~f_estate()
{
}

bool f_estate::init_run()
{
	return true;
}

void f_estate::destroy_run()
{
}

bool f_estate::proc()
{
	return true;
}
