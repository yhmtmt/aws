#include "stdafx.h"
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_fep01.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_fep01.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_fep01.h.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_fep01.h"

const char * f_fep01::m_cmd_str[32] = {
	"NUL", "ARG", "BAN", "BCL", "DAS",
	"DBM", "DVS", "FCN", "FRQ", "IDR",
	"IDW", "INI", "PAS", "POF", "PON", 
	"PTE", "PTN", "PTS", "ROF", "RON",
	"REG", "RID", "RST", "TBN", "TBR",
	"TB2", "TID", "TS2", "TXT", "TXR",
	"TX2", "VER"
};