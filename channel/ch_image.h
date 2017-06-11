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

enum e_campar_fish
{
	ECPF_FX = 0, ECPF_FY, ECPF_CX, ECPF_CY, ECPF_K1, ECPF_K2, ECPF_K3, ECPF_K4
};

class ch_image: public ch_base
{
protected:
	int m_back, m_front;
	
	Point2i m_offset;
	Size m_sz_sensor;

	Mat m_img[2];
	long long m_time[2]; // frame time (aws time)
	long long m_ifrm[2]; // frame index (if available.)
	long long m_tfile;	 // time fwrite called

	mutex m_mtx_bk, m_mtx_fr;
//	pthread_mutex_t m_mtx_bk, m_mtx_fr;
	bool m_bfisheye;
	bool m_bparam[ECP_K6+1];
	double m_param[ECP_K6+1];
	bool m_brvec;
	bool m_bR;
	union{
		double rvec[3];
		double R[9];
	};
	double tvec[3];
	
public:
 ch_image(const char * name):ch_base(name), m_bfisheye(false), m_brvec(true), m_bR(false), m_front(0), m_back(1), m_tfile(0), m_offset(0, 0), m_sz_sensor(0, 0)
	{
		m_time[0] = m_time[1] = 0;
		m_ifrm[0] = m_ifrm[1] = -1;

		for(int i = 0; i < ECP_K6 + 1; i++){
			m_bparam[i] = false;
		}
//		pthread_mutex_init(&m_mtx_bk, NULL);
//		pthread_mutex_init(&m_mtx_fr, NULL);
	}
	virtual ~ch_image()
	{
//		pthread_mutex_destroy(&m_mtx_bk);
//		pthread_mutex_destroy(&m_mtx_fr);
	}

	bool is_new(const long long t)
	{
		return t < m_time[m_front];
	}

	void set_offset(const int ox, const int oy){
		m_offset.x = ox;
		m_offset.y = oy;
	}

	void get_offset(int & ox, int & oy){
		ox = m_offset.x;
		oy = m_offset.y;
	}

	void set_sz_sensor(const int sw, const int sh){
	  m_sz_sensor.width = sw;
	  m_sz_sensor.height = sh;
	}

	void get_sz_sensor(int & sw, int & sh){
	  sw = m_sz_sensor.width;
	  sh = m_sz_sensor.height;
	}

	void reset_campar()
	{
		for(int i = 0; i < ECP_K6 + 1; i++){
			m_bparam[i] = false;
		}
		m_brvec = m_bR = false;
	}

	void set_fisheye(bool bfisheye)
	{
		m_bfisheye = bfisheye;
	}

	void set_int_campar(const e_campar & epar, const double val)
	{
		lock();
		m_bparam[epar] = true;
		m_param[epar] = val;
		unlock();
	}

	void set_int_campar(const e_campar_fish & epar, const double val)
	{
		lock();
		m_bparam[epar] = true;
		m_param[epar] = val;
		unlock();
	}

	bool get_int_campar(const e_campar epar, double & val)
	{
		lock();
		val = m_param[epar];
		bool r = m_bparam[epar];
		unlock();
		return r;
	}

	void set_ext_campar(const double * _rvec, const double * _tvec)
	{
		lock();
		m_brvec = true;
		rvec[0] = _rvec[0];
		tvec[0] = _tvec[0];
		rvec[1] = _rvec[1];
		tvec[1] = _tvec[1];
		rvec[2] = _rvec[2];
		tvec[2] = _tvec[2];
		unlock();
	}

	bool get_ext_campar(double * _rvec, double * _tvec)
	{
		lock();
		_rvec[0] = rvec[0];
		_tvec[0] = tvec[0];
		_rvec[1] = rvec[1];
		_tvec[1] = tvec[1];
		_rvec[2] = rvec[2];
		_tvec[2] = tvec[2];
		bool r = m_brvec;
		unlock();
		return 	r;
	}

	void set_ext_campar_mat(const double * _R, const double * _tvec)
	{
		lock();
		m_bR = true;
		memcpy((void*)R, (void*)_R, sizeof(double) * 9);
		tvec[0] = _tvec[0];
		tvec[1] = _tvec[1];
		tvec[2] = _tvec[2];
		unlock();
	}

	bool get_ext_campar_mat(double * _R, double * _tvec)
	{
		lock();
		memcpy((void*)_R, (void*)R, sizeof(double) * 9);
		_tvec[0] = tvec[0];
		_tvec[1] = tvec[1];
		_tvec[2] = tvec[2];
		bool r = m_bR;
		unlock();
		return r;
	}

	virtual Mat get_img(long long & t) = 0;
	virtual Mat get_img(long long & t, long long & ifrm) = 0;
	virtual void set_img(Mat & img, long long t) = 0;
	virtual void set_img(Mat & img, long long t, long long ifrm) = 0;

	// file writer method
	virtual int write(FILE * pf, long long tcur);
	// file reader method
	virtual int read(FILE * pf, long long tcur);

	virtual bool log2txt(FILE * pbf, FILE * ptf);
};

// ch_image_cln output clone of the image for get_img
class ch_image_cln: public ch_image
{
protected:
public:
	ch_image_cln(const char * name):ch_image(name)
	{
	}

	virtual ~ch_image_cln()
	{
	}

	virtual Mat get_img(long long & t){
		Mat img;
		unique_lock<mutex> lock(m_mtx_fr);
		if(m_img[m_front].empty()){
			return img;
		}

		img = m_img[m_front].clone();
		t = m_time[m_front];
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		Mat img;
		unique_lock<mutex> lock(m_mtx_bk);
		if(m_img[m_front].empty()){
			return img;
		}

		img = m_img[m_front].clone();
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		unique_lock<mutex> lock_bk(m_mtx_bk);
		m_img[m_back] = img;
		m_time[m_back] = t;
		unique_lock<mutex> lock_fr(m_mtx_fr);
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		lock_fr.unlock();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		unique_lock<mutex> lock_bk(m_mtx_bk);
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		unique_lock<mutex> lock_fr(m_mtx_fr);
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		lock_fr.unlock();
	}
};

// ch_image_ref returns reference of the image for get_img. Don't change the data if you use this as inputs for multiple filters.
class ch_image_ref: public ch_image
{
protected:
public:
	ch_image_ref(const char * name): ch_image(name)
	{
	}

	virtual ~ch_image_ref(){
	}

	virtual Mat get_img(long long & t){
		unique_lock<mutex> lock(m_mtx_fr);
		Mat img = m_img[m_front];
		t = m_time[m_front];
		return img;
	}

	virtual Mat get_img(long long & t, long long & ifrm){
		unique_lock<mutex> lock(m_mtx_fr);
		Mat img = m_img[m_front];
		t = m_time[m_front];
		ifrm = m_ifrm[m_front];
		return img;
	}

	virtual void set_img(Mat & img, long long t){
		unique_lock<mutex> lock_bk(m_mtx_bk);
		m_img[m_back] = img;
		m_time[m_back] = t;
		unique_lock<mutex> lock_fr(m_mtx_fr);
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		lock_fr.unlock();
	}

	virtual void set_img(Mat & img, long long t, long long ifrm){
		unique_lock<mutex> lock_bk(m_mtx_bk);
		m_img[m_back] = img;
		m_time[m_back] = t;
		m_ifrm[m_back] = ifrm;
		unique_lock<mutex> lock_fr(m_mtx_fr);
		int tmp = m_front;
		m_front = m_back;
		m_back = tmp;
		lock_fr.unlock();
	}
};

#endif
