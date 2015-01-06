#ifndef _F_DS_VDEV_H_
#define _F_DS_VDEV_H_
// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ds_video.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ds_video.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ds_video.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_ds_video.h"

class f_ds_vdev: public f_ds_video
{
	bool get_src_filter(int dev);
	void dump_moniker_inf(IMoniker * pMoniker);
	LONGLONG m_duration;

	// only for Monster XX
	IBaseFilter * m_pYUVConv;
	int m_dev;
public:
	f_ds_vdev(const char * name);
	virtual ~f_ds_vdev();

	bool open(int dev);
	virtual void close();
	virtual bool init_run()
	{
		if(!f_ds_video::init_run())
			return false;
		return open(m_dev);
	}

	virtual void destroy_run()
	{
		return close();
	}

	virtual bool grab(Mat & img);
	virtual bool cmd_proc(s_cmd & cmd);
};

#endif