// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_uvc_cam.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_uvc_cam.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_uvc_cam.  If not, see <http://www.gnu.org/licenses/>. 

class f_uvc_cam: public f_cam
{
private:
	enum io_method {
		IO_METHOD_READ,
		IO_METHOD_MMAP,
		IO_METHOD_USERPTR,
	} io;

	char            dev_name[1024];
	int              fd;
	struct buffer    *buffers;
	unsigned int     n_buffers;
	int              out_buf;
	bool              force_format;

	char fname[128];

	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	bool init_read(unsigned int buffer_size);
	bool init_mmap();
	bool init_userp(unsigned int buffer_size);

	void process_image(Mat & img, const void * p, int size);
public:
	f_uvc_cam(const char * name):
	  f_cam(name), io(IO_METHOD_MMAP), 
		fd(-1), buffers(NULL), out_buf(0), force_format(false)
	{
		strcpy(dev_name, "/dev/video0");
		register_fpar("dev", dev_name, 1024, "Device name (e.g. /dev/video0)");
	}

	~f_uvc_cam()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool grab(Mat & img);
};