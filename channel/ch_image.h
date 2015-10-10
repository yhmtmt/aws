#ifndef _CH_IMAGE_H_
#define _CH_IMAGE_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_image.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_image.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_image.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "ch_base.h"

class ch_image: public ch_base
{
public:
	ch_image(const char * name):ch_base(name){}
	virtual ~ch_image(){}

	virtual Mat get_img(long long & t) = 0;
	virtual Mat get_img(long long & t, long long & ifrm) = 0;
	virtual void set_img(Mat & img, long long t) = 0;
	virtual void set_img(Mat & img, long long t, long long ifrm) = 0;
};

// ch_image_cln output clone of the image for get_img
class ch_image_cln: public ch_image
{
protected:
	int m_back, m_front;
	bool m_bnew;
	Mat m_img[2];
	long long m_time[2]; // frame time (aws time)
	long long m_ifrm[2]; // frame index (if available.)
public:
	ch_image_cln(const char * name):ch_image(name), m_front(0), m_back(1), m_bnew(false)
	{
		m_time[0] = m_time[1] = 0;
		m_ifrm[0] = m_ifrm[1] = -1;
	}

	virtual ~ch_image_cln()
	{
	}

	virtual Mat get_img(long long & t){
		Mat img;
		if(m_img[m_front].empty())
			return img;

		lock();
		img = m_img[m_front].clone();
		t = m_time[m_front];
		unlock();
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		Mat img;
		if(m_img[m_front].empty())
			return img;

		lock();
		img = m_img[m_front].clone();
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		unlock();
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		unlock();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		unlock();
	}

	// swap front and back buffers.
	virtual void tran()
	{
		if(!m_bnew)
			return;
		lock();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		m_bnew = false;
		unlock();
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t)
	{
		lock();
		vector<uchar> buf;
		imencode(".png", m_img[m_back], buf);
		Mat bufm(buf);
		uchar * ptr = bufm.ptr<uchar>();
		unsigned long long ul = (unsigned long long) (bufm.cols * bufm.rows);
		fout.write((const char*) &m_ifrm[m_back], sizeof(long long));
		fout.write((const char*) &ul, sizeof(unsigned long long));
		fout.write((const char*) ptr, (streamsize) ul);
		unlock();
		return true;
	}

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t)
	{
		lock();
		unsigned long long ul;
		m_time[m_back] = t;
		fin.read((char*) &m_ifrm[m_back], sizeof(long long));
		fin.read((char*) &ul, sizeof(unsigned long long));
		Mat bufm(1, (int) ul, CV_8UC1);
		fin.read((char*) bufm.data, (streamsize) ul);

		imdecode(bufm, CV_LOAD_IMAGE_ANYDEPTH, &m_img[m_back]);
		unlock();
		return true;
	}
};

// ch_image_ref returns reference of the image for get_img. Don't change the data if you use this as inputs for multiple filters.
class ch_image_ref: public ch_image
{
protected:
	int m_back, m_front;
	bool m_bnew;
	Mat m_img[2];
	long long m_time[2];
	long long m_ifrm[2]; // frame index (if available.)
public:
	ch_image_ref(const char * name): ch_image(name), 
		m_front(0), m_back(1), m_bnew(false)
	{
		m_time[0] = m_time[1] = 0;
		m_ifrm[0] = m_ifrm[1] = -1;
	}

	virtual ~ch_image_ref(){
	}

	virtual Mat get_img(long long & t){
		lock();
		Mat img = m_img[m_front];
		t = m_time[m_front];
		unlock();
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		lock();
		Mat img = m_img[m_front];
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		unlock();
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		unlock();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		unlock();
	}
	
	bool is_buf_in_use(const unsigned char * buf){
		return m_img[m_front].data == buf || m_img[m_back].data == buf;
	}

	// swap front and back buffers.
	virtual void tran()
	{
		if(!m_bnew)
			return;
		lock();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		m_bnew = false;
		unlock();
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t)
	{
		lock();
		vector<uchar> buf;
		imencode(".png", m_img[m_back], buf);
		Mat bufm(buf);
		uchar * ptr = bufm.ptr<uchar>();
		unsigned long long ul = (unsigned long long) (bufm.cols * bufm.rows);
		fout.write((const char*) &m_ifrm[m_back], sizeof(long long));
		fout.write((const char*) &ul, sizeof(unsigned long long));
		fout.write((const char*) ptr, (streamsize) ul);
		unlock();
		return true;
	}

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t)
	{
		lock();
		unsigned long long ul;
		m_time[m_back] = t;
		fin.read((char*) &m_ifrm[m_back], sizeof(long long));
		fin.read((char*) &ul, sizeof(unsigned long long));
		Mat bufm(1, (int) ul, CV_8UC1);
		fin.read((char*) bufm.data, (streamsize) ul);

		imdecode(bufm, CV_LOAD_IMAGE_ANYDEPTH, &m_img[m_back]);
		unlock();
		return true;
	}

};

#endif
