#ifndef _F_MISC_H_
#define _F_MISC_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_misc.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_misc.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_misc.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_sock.h"
#include "../util/aws_vlib.h"
#include "../util/c_clock.h"

#include "../channel/ch_image.h"

#include "f_base.h"

class f_misc: public f_base
{
protected:
	ch_image * m_pin;
	ch_image * m_pout;
public:
	f_misc(const char * name):f_base(name), m_pin(NULL), 
		m_pout(NULL)
	{
	}

	virtual ~f_misc()
	{
	}

	virtual bool check()
	{
		return true;
	}

};

class f_debayer: public f_misc
{
protected:
	ch_image * m_pin, * m_pout;
	long long m_timg;
	enum e_bayer_type{
		BG8, GB8, RG8, GR8, GR8NN, GR8Q, BG16, GB16, RG16, GR16, UNKNOWN
	} m_type;
	static const char * m_strBayer[UNKNOWN];

	char m_type_str[16];
public:
	f_debayer(const char * name): f_misc(name), m_pin(NULL), m_pout(NULL), m_type(BG8), m_timg(-1)
	{
		register_fpar("bayer", (int*)&m_type,
			(int) UNKNOWN, m_strBayer,
			"Type of bayer pattern. ");
	}

	virtual bool init_run()
	{		
		if(!m_chin.size()){
			return false;
		}

		if(!m_chout.size()){
			return false;
		}

		m_pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(!m_pin)
			return false;

		m_pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(!m_pout)
			return false;

		return true;
	}

	virtual void destroy_run()
	{
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
	}

	virtual bool proc();
};

class f_imread: public f_misc
{
protected:
	ch_image * m_pout;
	char m_fname[1024];
	ifstream m_flist;
	bool m_verb;

public:
	f_imread(const char * name): f_misc(name), m_verb(false)
	{
		m_fname[0] = '\0';
		register_fpar("verb", &m_verb, "Verbose for debug");
		register_fpar("flst", m_fname, 1024, "Image list file.");
	};

	~f_imread()
	{
	}

	virtual bool init_run()
	{
		m_flist.open(m_fname);

		if(!m_flist.is_open()){
			cerr << m_fname << " cannot be opened." << endl;
			return false;
		}
		if(!m_chout.size()){
			cerr << "Output channel is not sat." << endl;
			return false;
		}

		m_pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(!m_pout){
			cerr << "Output channel is not sat as ch_image." << endl;
			return false;
		}

		return true;
	}

	virtual void destroy_run()
	{
		m_flist.close();
	}

	virtual bool proc();
};

class f_imwrite: public f_misc
{
protected:
	ch_image * m_pin;
	bool m_verb;
	enum eImgType{
		eitTIFF, eitJPG, eitJP2, eitPNG, eitRAW
	} m_type;

	static const char * m_strImgType[eitRAW+1];
	int m_qjpg; // 0 to 100
	int m_qpng; // 0 to 10
	char m_path[1024];
	long long m_cur_timg;
public:
	f_imwrite(const char * name): f_misc(name), m_verb(false), m_pin(NULL), m_type(eitJPG), m_cur_timg(0)
	{
		m_path[0] = '.';
		m_path[1] = '\0';
		register_fpar("verb", &m_verb, "Verbose for debug.");
		register_fpar("type", (int*)&m_type, (int)eitRAW+1, m_strImgType, "Image type");
		register_fpar("qjpg", &m_qjpg, "Jpeg quality [0-100]");
		register_fpar("qpng", &m_qpng, "PNG quality [0-10]");
		register_fpar("path", m_path, 1024, "File path");
	}

	virtual bool init_run()
	{
		if(!m_chin.size()){
			return false;
		}

		m_pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(!m_pin)
			return false;

		return true;
	}

	virtual void destroy_run()
	{
	}

	virtual bool proc();
};

class f_gry: public f_misc
{
protected:
public:
	f_gry(const char * name):f_misc(name)
	{};
	~f_gry(){};

	virtual bool proc();
};

class f_imreg: public f_misc
{
protected:
public:
	f_imreg(const char * name): f_misc(name){
	}
	~f_imreg()
	{
	}

	virtual bool proc(){
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pin == NULL)
			return false;

		ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pout == NULL)
			return false;

		long long timg;
		Mat img = pin->get_img(timg);

		if(img.empty())
			return true;

		pout->set_img(img, timg);
		return true;
	}

//	virtual bool cmd_proc(s_cmd & cmd);
};

class f_gauss: public  f_misc
{
protected:
	double m_sigma;
public:
	f_gauss(const char * name): f_misc(name), m_sigma(3.)
	{};
	~f_gauss(){};

	virtual bool proc(){
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pin == NULL)
			return false;

		ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pout == NULL)
			return false;

		ch_image * psubin = dynamic_cast<ch_image*>(m_chin[1]);
		ch_image * psubout = dynamic_cast<ch_image*>(m_chout[1]);

		long long timg;
		Mat img = pin->get_img(timg);
		long long tsub;
		Mat imgsub;
		if(psubin != NULL && psubout != NULL){
			imgsub = psubin->get_img(tsub);
			if(tsub != timg)
				return true;
		}

		Mat out;
		GaussianBlur(img, out, Size(0, 0), m_sigma);
		pout->set_img(out, timg);
		if(psubin != NULL && psubout != NULL){
			psubout->set_img(imgsub, tsub);
		}

		return true;
	}

	virtual bool cmd_proc(s_cmd & cmd);
};

class f_clip: public  f_misc
{
protected:
	Rect m_rc_clip;
public:
	f_clip(const char * name):  f_misc(name), m_rc_clip(0,0,0,0)
	{
	}

	virtual ~f_clip()
	{
	}

	virtual bool proc()
	{
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pin == NULL)
			return false;

		ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pout == NULL)
			return false;

		long long timg;
		Mat img = pin->get_img(timg);
		Mat clip = img(m_rc_clip);
		pout->set_img(clip, timg);
		return true;
	}

	virtual bool cmd_proc(s_cmd & cmd);
};

class f_edge:public f_misc
{
protected:
public:
	f_edge(const char * name): f_misc(name)
	{
	}

	virtual ~f_edge()
	{
	}

	virtual bool proc()
	{
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pin == NULL)
			return false;

		ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pout == NULL)
			return false;

		long long timg;
		Mat img = pin->get_img(timg);

		Mat out;
		Canny(img, out, 50, 200, 3);

		pout->set_img(out, timg);

		return true;
	}
};

class f_bkgsub: public f_misc
{
protected:
	Mat m_avgimg;
	double m_nfac;
	double m_alpha;
	bool m_binit;
	float m_th;
public:
	f_bkgsub(const char * name):f_misc(name), m_nfac((float)(1./255.)),
		m_th(0.1f), m_alpha(0.5f), m_binit(false)
	{
	}

	~f_bkgsub()
	{
	}

	virtual bool proc()
	{
		ch_image * pin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pin == NULL)
			return false;

		ch_image * pout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pout == NULL)
			return false;

		long long timg;
		Mat img = pin->get_img(timg);
		Mat out;

		if(m_avgimg.empty()){
			switch(img.type()){
			case CV_8UC3:
				m_avgimg.create(img.rows, img.cols, CV_32FC3);
				break;
			case CV_8UC1:
				m_avgimg.create(img.rows, img.cols, CV_32FC1);
				break;
			}
			out.create(img.rows, img.cols, img.type());
		}

		int chans = img.channels();
		// subtracting average image 
		uchar * p0 = img.ptr<uchar>(0);
		float * p1 = m_avgimg.ptr<float>(0);
		uchar * p2 = out.ptr<uchar>(0);
		for(int i = 0; i <img.cols; i++){
			for(int j = 0; j < img.rows; j++){
				for(int k = 0; k < chans; k++, p0++, p1++, p2++){
					if(!m_binit){
						*p1 = (float) (*p0 * m_nfac);
					}

					float val = (float) (*p0 * m_nfac - *p1);
					if(fabs(val) > m_th){ // not back ground
						*p2 = *p0;
					}else{ // update back ground
						*p2 = 0;
						*p1 += (float) (val * m_alpha);
					}

				}
			}
		}
	
		m_binit = true;

		return true;
	}

	virtual bool cmd_proc(s_cmd & cmd);
};

class f_houghp: public f_base
{
protected:
	ch_image * m_pedgein;
	ch_image * m_pclrin;
	ch_image * m_pclrout;
public:
	f_houghp(const char * name): f_base(name),
		m_pedgein(NULL), m_pclrin(NULL), 
		m_pclrout(NULL)
	{
	}

	virtual ~f_houghp()
	{
	}

	virtual bool check()
	{
		return m_chin[0] != NULL;
	}

	virtual bool proc()
	{
		ch_image * pedgein = dynamic_cast<ch_image*>(m_chin[0]);
		if(pedgein == NULL)
			return false;

		ch_image * pclrin = dynamic_cast<ch_image*>(m_chout[1]);
		if(pclrin == NULL)
			return false;

		ch_image * pclrout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pclrout == NULL)
			return false;

		long long tedge;
		Mat img = pedgein->get_img(tedge);
		long long timg;
		Mat clrimg = pclrin->get_img(timg);

		if(clrimg.empty())
			return true;

		if(img.empty())
			return true;

		if(tedge != timg) 
			return true;

		vector<Vec4i> lines;
		HoughLinesP(img, lines, 1, CV_PI/180., 80, 30, 10);

		for(size_t i = 0; i < lines.size(); i++){
			line(clrimg, Point(lines[i][0], lines[i][1]), 
				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
		}

		pclrout->set_img(clrimg, tedge);

		return true;
	}
};

#endif
