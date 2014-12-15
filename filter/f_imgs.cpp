// Copyright(c) 2013 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_imgs.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_imgs.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_imgs.cpp.  If not, see <http://www.gnu.org/licenses/>. 

// NOTICE: this module uses win32 api. don't make this as target 
// in unix system.
#include "stdafx.h"

#include <cstdio>

#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/thread_util.h"
#include "../util/c_clock.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_campar.h"

#include "f_base.h"
#include "f_cam.h"
#include "f_imgs.h"

bool f_imgs::open(const char * path, const char * filter){
	if(path == NULL)
		return false;
	wchar_t wpath_with_filter[BUFSIZE_F_IMGS];
	{

		int len;
		wchar_t wbuf1[BUFSIZE_F_IMGS];
		len = (int) strlen(path) + 1;
		if(len > BUFSIZE_F_IMGS){
			cerr << "Buffer overflow in f_imgs::open. Path exceeds " << BUFSIZE_F_IMGS << " chars." << endl;
			return false;
		}

		strcpy(m_path, path);
		m_path[len-1] = '\\';
		m_fname = (m_path + len);

		mbstowcs(wbuf1, path, len);
		if(filter != NULL){
			wchar_t wbuf2[BUFSIZE_F_IMGS >> 3];
			len = (int) strlen(filter) + 1;
			if(len > BUFSIZE_F_IMGS >> 3){
				cerr << "Buffer overflow in f_imgs::open. Filter exceeds" << (BUFSIZE_F_IMGS >> 3) << " chars." << endl;
				return false;
			}

			mbstowcs(wbuf2, filter, len);
			wsprintf(wpath_with_filter, L"%s\\*.%s", wbuf1, wbuf2);
		}else{
			wsprintf(wpath_with_filter, L"%s\\*.*", wbuf1);
		}
	}

	m_hfile = FindFirstFile(wpath_with_filter, &m_FileData);

	if(m_hfile == INVALID_HANDLE_VALUE){
		cerr << "Failed to open path " << path << "\\" << filter << endl;
		return false;
	}

	return true;
}

void f_imgs::close()
{
	FindClose(m_hfile);
}

bool f_imgs::grab(Mat & img){
	int len = (int) wcslen(m_FileData.cFileName) + 1;
	if(len > BUFSIZE_F_IMGS){
		cerr << "Buffer overflow in f_imgs::grab. File name ecceeds " << BUFSIZE_F_IMGS << " chars." << endl;
		return false;
	}

	wcstombs(m_fname, m_FileData.cFileName, len);
	img = imread(m_path);

	if(!FindNextFile(m_hfile, &m_FileData))
		return false;

	return true;
}

bool f_imgs::cmd_proc(s_cmd & cmd){
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;
	int itok = 2;

	if(strcmp("open", args[itok]) == 0){
		if(num_args != 5)
			return false;

		return open(args[itok+1], args[itok+2]);
	}

	return f_cam::cmd_proc(cmd);
}

