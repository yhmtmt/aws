#ifndef _F_STABILIZER_H_
#define _F_STABILIZER_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_stabilizer.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_stabilizer.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_stabilizer.h.  If not, see <http://www.gnu.org/licenses/>. 

#define STAB_STR_SIZE 512


#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/aws_vlib.h"
#include "../util/c_clock.h"
#include "../util/c_imgalign.h"
#include "../util/c_ship.h"

#include "../channel/ch_image.h"
#include "../channel/ch_vector.h"

#include "f_base.h"

class f_stabilizer: public f_base
{
protected:
	vector<Mat> m_pyrimg[2];
	int m_num_itrs;
	int m_num_pyr_level;
	bool m_bclr;
	bool m_bweight;
	bool m_brobust;
	double m_th_robust;
	e_interpol_type m_interpol_type;
	const static char * m_strIntlType[EIT_UNKNOWN];
	e_warp_type m_warp_type;	
	const static char * m_strWarpType[EWT_UNKNOWN];
	Size m_sz_hblk;
	Mat m_refimg;
	bool m_bWinit;
	Mat m_M; // Motion Matrix
	Mat m_iM; // inverse of Motion Matrix
	double m_beta;
	double m_alpha[9]; // Motion Constant
	Mat m_W; // Warp Matrix
	c_imgalign m_core;
	Rect m_roi;
	char m_str[STAB_STR_SIZE];
	bool m_bthrough;

	bool m_bmask;
	Mat m_mask;
	vector<Mat> m_Tmask;

	int m_num_conv_frms;
	int m_num_frms;

	bool m_disp_inf;
	char m_fname_log[1024];
	char m_fname_mask[1024];
public:
	f_stabilizer(const char * name):f_base(name), m_bclr(false),
		m_beta(0.0001), m_disp_inf(true), m_bweight(false), m_brobust(false),
		m_th_robust(100.0), m_sz_hblk(4, 4), m_interpol_type(EIT_BIL), m_warp_type(EWT_RGD),
		m_num_pyr_level(4), m_num_itrs(5), m_roi(0,0,0,0), m_bWinit(false),
		m_bmask(false), m_bthrough(false), m_num_conv_frms(0),
		m_num_frms(0)
	{
		m_fname_log[0] = '\0';
		m_fname_mask[0] = '\0';
		set_alpha(0.1);
		register_fpar("rx", &m_roi.x, "x position of the ROI for motion tracking");
		register_fpar("ry", &m_roi.y, "y position of the ROI for motion tracking");
		register_fpar("rw", &m_roi.width, "Width of the ROI for motion tracking");
		register_fpar("rh", &m_roi.height, "Height of the ROI for motion tracking");
		register_fpar("plv", &m_num_pyr_level, "Number of pyramid levels");
		register_fpar("itrs", &m_num_itrs, "Number of Gauss-Newton iteration");
		register_fpar("bclear", &m_bclr, "Clear flag");
		register_fpar("bweight",&m_bweight, "Weighted Gauss-Newton enable flag");
		register_fpar("brobust", &m_brobust, "Robust Gauss-Newton enable flag");
		register_fpar("throb", &m_th_robust, "Robustness threashold");
		register_fpar("wtype", (int*) &m_warp_type, (int) EWT_UNKNOWN, m_strWarpType, "Warp type");
		register_fpar("ipltype", (int*)&m_interpol_type, (int) EIT_UNKNOWN, m_strIntlType, "Interpolation type");
		register_fpar("whblk", &m_sz_hblk.width, "Size of block for Hessian recalculation.");
		register_fpar("hhblk", &m_sz_hblk.height, "Size of block for Hessian recalculation.");
		register_fpar("beta", &m_beta, "Motion stabilization parameter");
		register_fpar("alpha0", &m_alpha[0], "Motion stabilization parameter");
		register_fpar("alpha1", &m_alpha[1], "Motion stabilization parameter");
		register_fpar("alpha2", &m_alpha[2], "Motion stabilization parameter");
		register_fpar("alpha3", &m_alpha[3], "Motion stabilization parameter");
		register_fpar("alpha4", &m_alpha[4], "Motion stabilization parameter");
		register_fpar("alpha5", &m_alpha[5], "Motion stabilization parameter");
		register_fpar("alpha6", &m_alpha[6], "Motion stabilization parameter");
		register_fpar("alpha7", &m_alpha[7], "Motion stabilization parameter");
		register_fpar("alpha8", &m_alpha[8], "Motion stabilization parameter");
		register_fpar("bthrough", &m_bthrough, "Flag for pass through the filter");
		register_fpar("bdisp", &m_disp_inf, "Flag for displaying stabilizer's information.");
		register_fpar("fmask", m_fname_mask, 1024, "File path of mask."); 

	}

	virtual ~f_stabilizer()
	{
	}

	virtual bool init_run(){
		m_core.set_num_itrs(m_num_itrs);
		m_core.set_robust(m_brobust);
		m_core.set_robust_th(m_th_robust);
		m_core.set_wt(m_warp_type);
		m_core.set_interpol_type(m_interpol_type);
		m_core.set_wt(m_warp_type);
		m_core.set_tmpl_sblk_sz(m_sz_hblk);
		if(m_fname_mask[0] != '\0'){
			m_mask = imread(m_fname_mask);
			if(m_mask.type() != CV_8UC1){
				cerr << "Error. Pixel mask should be gray scale image" << endl;
				return false;
			}
		}

		return true;
	}

	virtual void destroy_run(){

	}

	void set_alpha(double alpha){
		alpha = min(1.0, alpha);
		alpha = max(0.0, alpha);
		for(int i = 0; i < 9; i++)
			m_alpha[i] = alpha;
	}

	void init(){
		if(m_core.get_wt() == EWT_HMG){
			m_M = Mat::eye(3, 3, CV_64FC1);
			m_W = Mat::eye(3, 3, CV_64FC1);
		}else{
			m_M = Mat::eye(2, 3, CV_64FC1);
			m_W = Mat::eye(2, 3, CV_64FC1);
		}
	}

	virtual bool check()
	{
		return m_chin[0] != NULL && m_chin[1] != NULL
			&& m_chout[0] != NULL && m_chout[1] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd);

	virtual bool proc();
};


class f_tracker: public f_base
{
protected:
	ch_vector<c_track_obj> * m_pobjin;
	ch_vector<c_track_obj> * m_pobjout;

	ch_image * m_pgryin;
	vector<Mat> m_pyrimg;
	int m_num_pyr_levels;

	vector<c_track_obj> m_obj;

	c_imgalign m_core;
public:
	f_tracker(const char * name);
	virtual ~f_tracker();

	virtual size_t get_num_in_chans(){return 2;}
	virtual size_t get_num_out_chans(){return 1;}

	virtual bool check()
	{
		return m_chin[0] != NULL && m_chin[1] != NULL 
			&& m_chout[0] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd);
	virtual bool proc();

};

#endif