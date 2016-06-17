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

class f_bkg_mask: public f_misc
{
protected:
	ch_image_ref * m_ch_img_in, * m_ch_img_out, *m_ch_mask_out;
	Mat m_img, m_img_prev, m_mavg, m_mask;
	bool m_bupdate;
	int m_mth;
	double m_alpha, m_mbkgth;
	long long m_t, m_ifrm;
	char m_fmask[1024];

	void calc_mask_8uc1();
	void calc_mask_8uc3();
	void calc_mask_16uc1();
	void calc_mask_16uc3();

	void aply_mask_8uc1(Mat & img);
	void aply_mask_8uc3(Mat & img);
	void aply_mask_16uc1(Mat & img);
	void aply_mask_16uc3(Mat & img);

public:
	f_bkg_mask(const char * name);
	virtual ~f_bkg_mask();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

class f_lcc: public f_misc
{
protected:
	ch_image_ref * m_ch_img_in, * m_ch_img_out;

	long long m_t, m_ifrm;
	Mat m_img;
	bool m_flipx, m_flipy;
	bool m_bpass;

	enum e_alg {
		RAD, FULL, MM, QUAD, UNDEF
	} m_alg;
	static const char * m_str_alg[UNDEF];
	double m_alpha, m_range, m_bias;
	double m_sigma;
	Mat m_map;

	bool m_update_map; 
	char m_fmap[1024]; // map file name

	// rad algorithm
	char m_fcp[1024];
	AWSCamPar m_campar;
	vector<float> m_amap, m_vmap;
	int m_cx, m_cy, m_cx2, m_cy2;
	bool load_rad_data();
	void save_rad_data();
	void init_rad_data();
	void calc_rad_map();
	void calc_avg_and_var_16uc1_rad(Mat & img);
	void calc_avg_and_var_8uc1_rad(Mat & img);
	void calc_avg_and_var_16uc3_rad(Mat & img);
	void calc_avg_and_var_8uc3_rad(Mat & img);

	// full algorithm
	Mat m_aimg, m_vimg;
	bool load_full_data();
	void save_full_data();
	void init_full_data();
	void calc_full_map();
	void calc_avg_and_var_16uc1(Mat & img);
	void calc_avg_and_var_8uc1(Mat & img);
	void calc_avg_and_var_16uc3(Mat & img);
	void calc_avg_and_var_8uc3(Mat & img);

	// mini max algorithm
	Mat m_min, m_max;
	bool load_mm_data();
	void save_mm_data();
	void init_mm_data();
	void calc_mm_map();
	void calc_mm_map_16u();
	void calc_mm_map_8u();

	void calc_mm_16uc1(Mat & img);
	void calc_mm_8uc1(Mat & img);
	void calc_mm_16uc3(Mat & img);
	void calc_mm_8uc3(Mat & img);

	// min = avg - range dev
	// max = avg + range dev
	// scale = 255 / (max - min)
	// v' = (v - min) * 255 / (max - min)
	// v' = v scale - min scale
	
	// quadratic curve algorithm
	int m_depth;
	double m_qs, m_qb;
	// (qs r^2 + 1) color + qb r^2
	void calc_qmap_16u();
	void calc_qmap_8u();

	void filter_16uc1(Mat & in, Mat & out);
	void filter_8uc1(Mat & in, Mat & out);
	void filter_16uc3(Mat & in, Mat & out);
	void filter_8uc3(Mat & in, Mat & out);

public:
	f_lcc(const char * name);

	virtual ~f_lcc()
	{
	}

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

class f_stereo_disp: public f_misc
{
protected:
	ch_image_ref * m_ch_img1, * m_ch_img2;
	ch_image_ref * m_ch_disp;

	enum s_out{
		DISP, IMG1, IMG2
	} m_out;
	static const char * m_str_out[IMG2 + 1];

	Mat m_img1, m_img2, m_disp;
	long long m_timg1, m_timg2;
	long long m_ifrm1, m_ifrm2;

	bool m_bflipx, m_bflipy; // image flipping option 

	// Frame state flag
	bool m_bnew; // new frame
	bool m_bsync; // synchronized frame

	char m_fcpl[1024], m_fcpr[1024], m_fstp[1024];
	bool m_bpl, m_bpr, m_bstp, m_brct;

	// for rectification 
	AWSCamPar m_camparl, m_camparr;
	Mat m_Rl, m_Rr;
	Mat m_Pl, m_Pr;
	Mat m_mapl1, m_mapl2, m_mapr1, m_mapr2;
	Mat m_Rlr, m_Tlr;
	Mat m_E, m_F, m_Q;

	bool load_stereo_pars();
	bool rectify_stereo();

	// stereo block matching
	Rect m_roi;
	Ptr<StereoSGBM> m_sgbm;
	struct s_sgbm_par{
		bool m_update;
		bool m_bsg;

		int minDisparity; /* normally zero. it depends on rectification algorithm. */
		int numDisparities; /* maximum disparity - minimum disparity. it must be the divisible number by 16*/
		int blockSize; /* 3 to 11 */
		int P1, P2;
		int disp12MaxDiff;
		int preFilterCap;
		int uniquenessRatio;
		int speckleWindowSize;
		int speckleRange;
		int mode;
		s_sgbm_par():m_update(false), m_bsg(true), minDisparity(0), numDisparities(64), blockSize(3),
			disp12MaxDiff(1), preFilterCap(0), uniquenessRatio(10), speckleWindowSize(100),
			speckleRange(32), mode(StereoSGBM::MODE_SGBM)
		{
			P1 = 8 * blockSize * blockSize;
			P2 = P1 * 4;
		}
	} m_sgbm_par;

	Ptr<StereoBM> m_bm;
public:
	f_stereo_disp(const char * name);
	virtual ~f_stereo_disp();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};

class f_debayer: public f_misc
{
protected:
	ch_image * m_pin, * m_pout;
	long long m_timg;
	enum e_bayer_type{
		BG8, GB8, RG8, GR8, GR8NN, GR8Q, GR8GQ, GR8DGQ, BG16, GB16, RG16, GR16, UNKNOWN
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
