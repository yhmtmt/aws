// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_ahrs.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ahrs.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ahrs.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <iostream>
#include <fstream>
using namespace std;

#include "f_ahrs.h"

f_ahrs::f_ahrs(const char * name): f_base(name)
{
	m_dname[0] = '\0';
	register_fpar("dev", m_dname, 1024, "Device file path of the serial port to be opened.");
	register_fpar("port", &m_port, "Port number of the serial port to be opened. (for Windows)");
	register_fpar("br", &m_br, "Baud rate.");
}

f_ahrs::~f_ahrs()
{
}

bool f_ahrs::init_run()
{
#ifdef _WIN32
	m_hserial = open_serial(m_port, m_br);
#else
	m_hserial = open_serial(m_dname, m_br);
#endif
	if(m_hserial == NULL_SERIAL)
		return false;

	return true;
}

void f_ahrs::destroy_run()
{
	close_serial(m_hserial);
}

bool f_ahrs::proc()
{
	return true;
}