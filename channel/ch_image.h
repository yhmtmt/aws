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

enum e_campar{
	ECP_FX = 0, ECP_FY, ECP_CX, ECP_CY, ECP_K1, ECP_K2, ECP_P1, ECP_P2, ECP_K3, ECP_K4, ECP_K5, ECP_K6
};

class ch_image: public ch_base
{
protected:
	pthread_mutex_t m_mtx_bk, m_mtx_fr;
	bool m_bparam[ECP_K6+1];
	double m_param[ECP_K6+1];
	bool m_brvec;
	union{
		double rvec[3];
		double R[9];
	};
	double tvec[3];
	
public:
	ch_image(const char * name):ch_base(name), m_brvec(true)
	{
		pthread_mutex_init(&m_mtx_bk, NULL);
		pthread_mutex_init(&m_mtx_fr, NULL);
	}
	virtual ~ch_image()
	{
		pthread_mutex_destroy(&m_mtx_bk);
		pthread_mutex_destroy(&m_mtx_fr);
	}

	void set_int_campar(const e_campar & epar, const double val)
	{
		m_bparam[epar] = true;
		m_param[epar] = val;
	}

	bool get_int_campar(const e_campar & epar, double & val)
	{
		return m_bparam[epar];
	}

	void set_ext_campar(const double * _rvec, const double * _tvec)
	{
		m_brvec = true;
		rvec[0] = _rvec[0];
		tvec[0] = _tvec[0];
		rvec[1] = _rvec[1];
		tvec[1] = _tvec[1];
		rvec[2] = _rvec[2];
		tvec[2] = _tvec[2];
	}

	bool get_ext_campar(double * _rvec, double * _tvec)
	{
		_rvec[0] = rvec[0];
		_tvec[0] = tvec[0];
		_rvec[1] = rvec[1];
		_tvec[1] = tvec[1];
		_rvec[2] = rvec[2];
		_tvec[2] = tvec[2];
		return m_brvec;
	}

	void set_ext_campar_mat(const double * _R, const double * _tvec)
	{
		memcpy((void*)R, (void*)_R, sizeof(double) * 9);
		tvec[0] = _tvec[0];
		tvec[1] = _tvec[1];
		tvec[2] = _tvec[2];
	}

	bool get_ext_campar_mat(double * _R, double * _tvec)
	{
		memcpy((void*)_R, (void*)R, sizeof(double) * 9);
		_tvec[0] = tvec[0];
		_tvec[1] = tvec[1];
		_tvec[2] = tvec[2];
		return !m_brvec;
	}

	void lock_fr(){pthread_mutex_lock(&m_mtx_fr);};
	void unlock_fr(){pthread_mutex_unlock(&m_mtx_fr);};
	void lock_bk(){pthread_mutex_lock(&m_mtx_bk);};
	void unlock_bk(){pthread_mutex_unlock(&m_mtx_bk);};

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
	
	Mat m_img[2];
	long long m_time[2]; // frame time (aws time)
	long long m_ifrm[2]; // frame index (if available.)
public:
	ch_image_cln(const char * name):ch_image(name), m_front(0), m_back(1)
	{
		m_time[0] = m_time[1] = 0;
		m_ifrm[0] = m_ifrm[1] = -1;
	}

	virtual ~ch_image_cln()
	{
	}

	virtual Mat get_img(long long & t){
		Mat img;
		lock_fr();
		if(m_img[m_front].empty()){
			unlock_fr();
			return img;
		}

		img = m_img[m_front].clone();
		t = m_time[m_front];
		unlock_fr();
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		Mat img;
		lock_fr();
		if(m_img[m_front].empty()){
			unlock_fr();
			return img;
		}

		img = m_img[m_front].clone();
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		unlock_fr();
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		lock_bk();
		m_img[m_back] = img;
		m_time[m_back] = t;
		lock_fr();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		unlock_fr();
		unlock_bk();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		lock_bk();
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		lock_fr();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		unlock_fr();
		unlock_bk();
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t);

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t);
};

// ch_image_ref returns reference of the image for get_img. Don't change the data if you use this as inputs for multiple filters.
class ch_image_ref: public ch_image
{
protected:
	int m_back, m_front;
	Mat m_img[2];
	long long m_time[2];
	long long m_ifrm[2]; // frame index (if available.)
public:
	ch_image_ref(const char * name): ch_image(name), 
		m_front(0), m_back(1)
	{
		m_time[0] = m_time[1] = 0;
		m_ifrm[0] = m_ifrm[1] = -1;
	}

	virtual ~ch_image_ref(){
	}

	virtual Mat get_img(long long & t){
		lock_fr();
		Mat img = m_img[m_front];
		t = m_time[m_front];
		unlock_fr();
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		lock_fr();
		Mat img = m_img[m_front];
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		unlock_fr();
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		lock_bk();
		m_img[m_back] = img;
		m_time[m_back] = t;
		lock_fr();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		unlock_fr();
		unlock_bk();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		lock_bk();
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		lock_fr();
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		unlock_fr();
		unlock_bk();
	}
	
	bool is_buf_in_use(const unsigned char * buf){
		return m_img[m_front].data == buf || m_img[m_back].data == buf;
	}

	// for channel logging
	virtual bool write(f_base * pf, ofstream & fout, long long t);

	// for channel replay
	virtual bool read(f_base * pf, ifstream & fin, long long t);
	
};

#endif
