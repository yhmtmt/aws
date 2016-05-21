// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_imgs.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_imgs.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_imgs.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _F_IMGS_H_
#define _F_IMGS_H_
// Alert: This module is written only for Windows. 
#ifdef _WIN32
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_cam.h"

#include <wchar.h>

#define BUFSIZE_F_IMGS 256
class f_imgs: public f_cam
{
protected:
	HANDLE m_hfile;
	WIN32_FIND_DATAW m_FileData;
	char m_path[BUFSIZE_F_IMGS];
	char * m_fname;
public:
	f_imgs(const char * name):f_cam(name)
	{
	}

	virtual ~f_imgs()
	{
		close();
	}

	bool open(const char * path, const char * filter);
	virtual void close();

	virtual bool grab(Mat & img);
	virtual bool cmd_proc(s_cmd & cmd);
};

#endif
#endif
