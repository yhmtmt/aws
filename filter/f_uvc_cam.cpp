// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_uvc_cam.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_uvc_cam.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_uvc_cam.  If not, see <http://www.gnu.org/licenses/>. 

#include "stdafx.h"

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
//#include <getopt.h>             /* getopt_long() */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <list>
#include <cmath>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/thread_util.h"
#include "../util/c_clock.h"
#include "../util/util.h"
#include "../channel.h"
#include "f_base.h"
#include "f_cam.h"

#include "f_uvc_cam.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

static void errnoout(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
}

const char * get_fmt(unsigned int fmt){
	switch(fmt){
	case V4L2_PIX_FMT_GREY: return "GREY";
	case V4L2_PIX_FMT_Y10: return "Y10";
	//case V4L2_PIX_FMT_Y12: return "Y12";
	//case V4L2_PIX_FMT_Y10BPACK: return "Y10BPACK";
	case V4L2_PIX_FMT_Y16: return "Y16";
	//case V4L2_PIX_FMT_UV8:return "UV8";
	case V4L2_PIX_FMT_YUYV: return "YUYV";
	case V4L2_PIX_FMT_UYVY: return "UYVY";
	case V4L2_PIX_FMT_YVYU: return "YVYU";
	//case V4L2_PIX_FMT_VYUV:return "VYUV";
	case V4L2_PIX_FMT_Y41P: return "Y41P";
	case V4L2_PIX_FMT_YVU420: return "YVU420";
	case V4L2_PIX_FMT_YUV420: return "YUV420";
	//case V4L2_PIX_FMT_YUV420M: return "YUV420M";
	case V4L2_PIX_FMT_YUV410: return "YUV410";
	//case V4L2_PIX_FMT_YUV422:return "YUV422";
	//case V4L2_PIX_FMT_YUV411:return "YUV411";
	case V4L2_PIX_FMT_NV12: return "NV12";
	case V4L2_PIX_FMT_NV21: return "NV21";
	//case V4L2_PIX_FMT_NV12M: return "NV12M";
	//case V4L2_PIX_FMT_NV21M: return "NV21M";
	//case V4L2_PIX_FMT_NV12MT:	return "NV12MT";
	case V4L2_PIX_FMT_NV16: return "NV16";
	case V4L2_PIX_FMT_NV61: return "NV61";
	//case V4L2_PIX_FMT_NV24: return "NV24";
	//case V4L2_PIX_FMT_NV42: return "NV42";
	//case V4L2_PIX_FMT_M420: return "M420";
	//case V4L2_PIX_FMT_RGB1:return "RGB1";
	//case V4L2_PIX_FMT_R444:return "R444";
	//case V4L2_PIX_FMT_RGBO:return "RGBO";
	//case V4L2_PIX_FMT_RGBP:return "RGBP";
	//case V4L2_PIX_FMT_RGBQ:return "RGBQ";
	//case V4L2_PIX_FMT_RGBR:return "RGBR";
	//case V4L2_PIX_FMT_RGRH:return "RGRH";
	//case V4L2_PIX_FMT_BGR3:return "BGR3";
	//case V4L2_PIX_FMT_RGB3:return "RGB3";
	//case V4L2_PIX_FMT_BGR4:return "BGR4";
	//case V4L2_PIX_FMT_RGB4:return "RGB4";
	}

	return "Unknown";
}

struct buffer {
	void   *start;
	size_t  length;
};


static int xioctl(int fh, int request, void *arg)
{
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}


bool f_uvc_cam::init_read(unsigned int buffer_size)
{
	buffers = (buffer*)calloc(1, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		return false;
	}

	buffers[0].length = buffer_size;
	buffers[0].start = malloc(buffer_size);

	if (!buffers[0].start) {
		fprintf(stderr, "Out of memory\n");
		free(buffers);
		buffers = NULL;
		return false;
	}
	return true;
}

bool f_uvc_cam::init_mmap(void)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
				"memory mapping\n", dev_name);
			return false;
		} else {
			errnoout("VIDIOC_REQBUFS");
			return false;
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n",
			dev_name);
		return false;
	}

	buffers = (buffer*)calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		return false;
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){
			errnoout("VIDIOC_QUERYBUF");
			return false;
		}

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap(NULL /* start anywhere */,
			buf.length,
			PROT_READ | PROT_WRITE /* required */,
			MAP_SHARED /* recommended */,
			fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start){
			errnoout("mmap");
			// unmapping code
			n_buffers--;
			for(; n_buffers > 0; n_buffers--){
				if(-1 == munmap(buffers[n_buffers].start, buffers[n_buffers].length)){
					errnoout("munmap");
					exit(1);
				}
			}
			free(buffers);
			buffers = NULL;
			return false;
		}
	}
	return true;
}

bool f_uvc_cam::init_userp(unsigned int buffer_size)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count  = 4;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
				"user pointer i/o\n", dev_name);
		} else {
			errnoout("VIDIOC_REQBUFS");
		}
		return false;
	}

	buffers = (buffer*)calloc(4, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		return false;
	}

	for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
		buffers[n_buffers].length = buffer_size;
		buffers[n_buffers].start = malloc(buffer_size);

		if (!buffers[n_buffers].start) {
			fprintf(stderr, "Out of memory\n");
			n_buffers--;
			for(; n_buffers > 0; n_buffers--){
				free(buffers[n_buffers].start);
			}
			free(buffers);
			buffers = NULL;
			return false;
		}
	}
	return true;
}

void f_uvc_cam::process_image(Mat & img, const void *p, int size)
{
	if (out_buf)
		fwrite(p, size, 1, stdout);

	// yuyv conversion
	uchar * p0 = img.data;
	uchar * p1 = (uchar*) p;
	int num = img.cols * img.rows * 3;
	for(int i = 0, j = 0; i < num; i+=6, j+=4){
		int cr = (int)p1[j+3] - 128;
		int cb = (int)p1[j+1] - 128;
		int tcr1 = (int)(cr * 1.402);
		int tcr2 = (int)(cr * 0.714);
		int tcb1 = (int)(cb * 1.772);
		int tcb2 = (int)(cb * 0.344);

		p0[i+2] = saturate_cast<uchar>((int)p1[j] + tcr1);
		p0[i+1] = saturate_cast<uchar>((int)p1[j] - tcb2 - tcr2);
		p0[i] = saturate_cast<uchar>((int)p1[j] + tcb1);
		p0[i+5] = saturate_cast<uchar>((int)p1[j+2] + tcr1);
		p0[i+4] = saturate_cast<uchar>((int)p1[j+2] - tcb2 - tcr2);
		p0[i+3] = saturate_cast<uchar>((int)p1[j+2] + tcb1);
	}
}


bool f_uvc_cam::init_run()
{
	/////////////////////////////////open
	struct stat st;
	
	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
			dev_name, errno, strerror(errno));
		return false;
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		return false;
	}

	fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
			dev_name, errno, strerror(errno));
		return false;
	}

	///////////////////////////////init
	unsigned int min;

	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",
				dev_name);
		} else {
			errnoout("VIDIOC_QUERYCAP");
		}
		return false;
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n",
			dev_name);
		return false;
	}

	switch (io) {
	case IO_METHOD_READ:
		if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf(stderr, "%s does not support read i/o\n",
				dev_name);
			return false;
		}
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf(stderr, "%s does not support streaming i/o\n",
				dev_name);
			return false;
		}
		break;
	}

	/* Select video input, video standard and tune here. */
	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (force_format) {
		fmt.fmt.pix.width       = 640;
		fmt.fmt.pix.height      = 480;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
			errnoout("VIDIOC_S_FMT");
			return false;
		}

		/* Note VIDIOC_S_FMT may change width and height. */
	} else {
		/* Preserve original settings as set by v4l2-ctl for example */
		if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)){
			errnoout("VIDIOC_G_FMT");
			return false;
		}
	}

	cout << "Image specification" << endl;
	cout << " Device name " << dev_name << endl;
	cout << "Width " << fmt.fmt.pix.width << endl;
	cout << "Height " << fmt.fmt.pix.height << endl;
	cout << "PIX format " << get_fmt(fmt.fmt.pix.pixelformat) << endl;
	cout << "Bytes per line " << fmt.fmt.pix.bytesperline << endl;
	cout << "Size of image " << fmt. fmt.pix.sizeimage << endl;

	if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV){
		return false;
	}

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	switch (io) {
	case IO_METHOD_READ:
		if(!init_read(fmt.fmt.pix.sizeimage))
			return false;
		break;

	case IO_METHOD_MMAP:
		if(!init_mmap())
			return false;
		break;

	case IO_METHOD_USERPTR:
		if(!init_userp(fmt.fmt.pix.sizeimage))
			return false;
		break;
	}

	/////////////////////////////// start
	unsigned int i;
	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;

			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
				errnoout("VIDIOC_QBUF");
				return false;
			}
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
			errnoout("VIDIOC_STREAMON");
			return false;
		}

		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;
			buf.index = i;
			buf.m.userptr = (unsigned long)buffers[i].start;
			buf.length = buffers[i].length;

			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
				errnoout("VIDIOC_QBUF");
				return false;
			}
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
			errnoout("VIDIOC_STREAMON");
			return false;
		}
		break;
	}

	return true;
}

void f_uvc_cam::destroy_run()
{
  if(-1 == fd) // no need to destroy
    return;

	///////////////////////////stop

	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
			errnoout("VIDIOC_STREAMOFF");
		}
		break;
	}
	////////////////////////uninit
	unsigned int i;
	if(buffers != NULL){

		switch (io) {
		case IO_METHOD_READ:
			free(buffers[0].start);
			break;

		case IO_METHOD_MMAP:
			for (i = 0; i < n_buffers; ++i)
				if (-1 == munmap(buffers[i].start, buffers[i].length)){
					errnoout("munmap");
					exit(1);
				}
				break;

		case IO_METHOD_USERPTR:
			for (i = 0; i < n_buffers; ++i)
				free(buffers[i].start);
			break;
		}

		free(buffers);
		buffers = NULL;
	}
	///////////////////////////close
	if (-1 == close(fd)){
		errnoout("close");
	}

	fd = -1;
}

bool f_uvc_cam::grab(Mat & img)
{
	if(img.cols != fmt.fmt.pix.width || img.rows != fmt.fmt.pix.height){
		img.create(fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC3);
	}

	//////////////// frame waiting
	fd_set fds;
	struct timeval tv;
	int r;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;

	r = select(fd + 1, &fds, NULL, NULL, &tv);

	if (-1 == r) {
		if (EINTR == errno)
			return true;
		errnoout("select");
	}

	if (0 == r) {
		fprintf(stderr, "select timeout\n");
		exit(EXIT_FAILURE);
	}

	/////////////////////////// grab
	struct v4l2_buffer buf;
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
		if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errnoout("read");
				return false;
			}
		}

		process_image(img, buffers[0].start, buffers[0].length);
		break;

	case IO_METHOD_MMAP:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return false;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errnoout("VIDIOC_DQBUF");
				return false;
			}
		}

		assert(buf.index < n_buffers);

		process_image(img, buffers[buf.index].start, buf.bytesused);

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
			errnoout("VIDIOC_QBUF");
			return false;
		}
		break;

	case IO_METHOD_USERPTR:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return false;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errnoout("VIDIOC_DQBUF");
				return false;
			}
		}

		for (i = 0; i < n_buffers; ++i)
			if (buf.m.userptr == (unsigned long)buffers[i].start
				&& buf.length == buffers[i].length)
				break;

		assert(i < n_buffers);

		process_image(img, (void *)buf.m.userptr, buf.bytesused);

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
			errnoout("VIDIOC_QBUF");
			return false;
		}
		break;
	}

	return true;
}
