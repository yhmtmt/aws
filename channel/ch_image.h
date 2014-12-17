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


class ch_image: public ch_base
{
public:
	ch_image(const char * name):ch_base(name){}
	virtual ~ch_image(){}

	virtual Mat get_img(long long & t) = 0;
	virtual void set_img(Mat & img, long long t) = 0;
};

class ch_image_cln: public ch_image
{
protected:
	int m_back, m_front;
	bool m_bnew;
	Mat m_img[2];
	long long m_time[2];
public:
	ch_image_cln(const char * name):ch_image(name), m_front(0), m_back(1), m_bnew(false)
	{
		m_time[0] = m_time[1] = 0;
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

	virtual void set_img(Mat & img, long long t){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		unlock();
	}

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
};

class ch_image_ref: public ch_image
{
protected:
	int m_back, m_front;
	bool m_bnew;
	Mat m_img[2];
	long long m_time[2];
public:
	ch_image_ref(const char * name): ch_image(name), 
		m_front(0), m_back(1), m_bnew(false)
	{
		m_time[0] = m_time[1] = 0;
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

	virtual void set_img(Mat & img, long long t){
		lock();
		m_bnew = true;
		m_img[m_back] = img;
		m_time[m_back] = t;
		unlock();
	}

	bool is_buf_in_use(const unsigned char * buf){
		return m_img[m_front].data == buf || m_img[m_back].data == buf;
	}

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
};
