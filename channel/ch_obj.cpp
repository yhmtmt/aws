// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// ch_obj.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_obj.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_obj.cpp.  If not, see <http://www.gnu.org/licenses/>.
#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;
#include "ch_obj.h"

/////////////////////////////////////////////////////// c_obj member (The base class of the objects in aws)
c_obj::c_obj():
	m_type(EOT_UNKNOWN), m_src(EOS_UNDEF), m_tst(EOST_UNDEF), m_dtype(EOD_UNDEF),
	m_t(0), m_lat(0), m_lon(0), m_alt(0), m_cog(0), m_sog(0), m_x(0), m_y(0), m_z(0),
	m_vx(0), m_vy(0), m_vz(0), m_xr(0), m_yr(0), m_zr(0), m_vxr(0), m_vyr(0), m_vzr(0)

{
}

c_obj::~c_obj()
{
}

////////////////////////////////////////////////////// c_vobj member
c_vobj::c_vobj():m_rc()
{
}

c_vobj::c_vobj(const long long t, const Rect & rc, const Mat & tmp, const f_base * pfsrc, bool manual)
{
	set_time(t);
	set(t, rc, tmp, pfsrc, manual);
}

c_vobj::~c_vobj()
{
}


///////////////////////////////////////////////////// c_ais_obj member
c_ais_obj::c_ais_obj():m_mmsi(0)
{
}

c_ais_obj::c_ais_obj(const c_ais_obj & obj)
{
	set(obj);
}


c_ais_obj::c_ais_obj(const long long t, const unsigned int mmsi, float lat, float lon, float cog, float sog, float hdg)
{
	set(t, mmsi, lat, lon, cog, sog, hdg);
}

c_ais_obj::~c_ais_obj()
{
}

//////////////////////////////////////////////////// 