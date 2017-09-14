// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// ch_wp.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_wp.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_image.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <list>

using namespace std;

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_thread.h"
#include "ch_wp.h"

s_wp ch_wp::wp_null(true);
