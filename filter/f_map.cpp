// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

// f_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_map.h.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <map>

using namespace std;
#include <cmath>
#include <cstring>

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_map.h"

const char * f_map::m_cmd_str[emc_none] =
{
	"normal", "add", "release"
};

const char * f_map::m_mdt_str[emd_none] = 
{
	"jpgis"
};

f_map::f_map(const char * name) :f_base(name), m_ch_map(NULL), m_num_levels(4),
m_min_meter_per_pix(1), m_min_step(1000), m_scale(10), m_ncache(1000)
{
	m_path[0] = '.';
	m_path[1] = '\0';
	register_fpar("ch_map", (ch_base**)&m_ch_map, typeid(ch_map).name(), "Map channel.");

	register_fpar("path", m_path, 1024, "Path to map data.");
	register_fpar("nLevels", &m_num_levels, "Number of scale levels");
	register_fpar("minMeterPerPix", &m_min_meter_per_pix, "Dimension of the pixel in the lowest level.");
	register_fpar("minStep", &m_min_step, "Minimum node distance in the lowest level");
	register_fpar("scale", &m_scale, "Scale between levels.");
	register_fpar("ncache", &m_ncache, "Cache size for nodes.");

	// map command related 
	register_fpar("dtype", (int*)&m_dtype, (int)emd_none, m_mdt_str, "Data type to be added.");
	register_fpar("fdata", m_fdata, 1024, "File path to map data to be added");

	register_fpar("cmd", (int*)&m_cmd, (int)emc_none, m_cmd_str, "Map command.");

}

f_map::~f_map()
{
}

bool f_map::init_run()
{
	if (!m_ch_map)
		return false;

	m_ch_map->init(m_num_levels);
	return true;
}

void f_map::destroy_run()
{
}

bool f_map::proc()
{
	switch (m_cmd){
	case emc_normal:
		return proc_normal();
	case emc_add:
		return proc_add();
	case emc_release:
		return proc_release();
	case emc_none:
		break;
	}

	return true;
}

bool f_map::proc_normal()
{
	m_range = m_ch_map->get_range();
	Point3f point;
	m_ch_map->get_center(point.x, point.y, point.z);
	double d = norm(point-m_point_prev);
	if (d < m_range)
		return true;

	return true;
}

bool f_map::proc_add()
{
	switch (m_dtype){
	case emd_jpgis:
		break;
	}
	return true;
}

bool f_map::proc_release()
{
	return true;
}