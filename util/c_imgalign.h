#ifndef _C_IMGALIGN_H_
#define _C_IMGALIGN_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_imgalign.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_imgalign.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_imgalign.h.  If not, see <http://www.gnu.org/licenses/>. 


//#define UTIL_DBG
enum e_interpol_type{
	EIT_NN, EIT_BIL, EIT_UNKNOWN
};

enum e_warp_type{
	EWT_TRN, EWT_RGD, EWT_SIM, EWT_AFN, EWT_HMG, EWT_UNKNOWN
};

e_warp_type get_warp_type(const char * str);
const char * get_warp_type_name(e_warp_type type);
e_interpol_type get_interpol_type(const char * str);
const char* get_interpol_type_name(e_interpol_type type);

class c_imgalign{
protected:
	// statistics
	vector<int> m_num_itrs;
	vector<double> m_delta_p;
	int m_rjct_pix_cnt;
	vector<double> m_rjct_pix_ratio;
	int m_rjct_blk_cnt;
	vector<double> m_rjct_blk_ratio;
	void init_statistics(int plv)
	{
		m_num_itrs.resize(plv);
		m_delta_p.resize(plv);
		m_rjct_pix_ratio.resize(plv);
		m_rjct_blk_ratio.resize(plv);
	}

	void init_itr_statistics(){
		m_rjct_pix_cnt = 0;
		m_rjct_blk_cnt = 0;
	}

	void calc_itr_statistics(int ilv, int itr, int num_pix, int num_blks){
		m_num_itrs[ilv] = itr;
		m_delta_p[ilv] = m_delta_norm;
		m_rjct_pix_ratio[ilv] = (double) m_rjct_pix_cnt / (double) num_pix;
		m_rjct_blk_ratio[ilv] = (double) m_rjct_blk_cnt / (double) num_blks;
	}

	bool m_bconv;

	// algorithm parameters
	int m_inew, m_icur;
	Mat m_Wtmp[2];
	Mat m_invW;
	Mat m_J; // Jacobian
	vector<vector<Mat> > m_H;

	Mat m_invH; // Hessian
	Mat m_aH;
	Mat m_kxrow, m_kxcol, m_kyrow, m_kycol;
	double m_itr_exit_th;
	int m_num_max_itrs;
	double m_delta_norm;
	bool m_weight;  // error weight
	bool m_robust; // robust function
	double m_err_th; // error threshold for robust function
	// template block size for hessian calculation
	int m_tmpl_blk_sx; 
	int m_tmpl_blk_sy;

	e_warp_type m_wt;

	bool m_bmask;
	vector<Mat> m_Tmask;
	Mat m_mask;

	// affine
	void calc_H_afn(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
		int start_x, int start_y, int end_x, int end_y, Mat & H);
	void afn_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q, 
		int offset_x, int offset_y);

	// translation
	void calc_H_trn(Mat & T, Mat & Tx, Mat & Ty, Mat & Q,
		int start_x, int start_y, int end_x, int end_y, Mat & H);
	void trn_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q, 
		int offset_x, int offset_y);

	// rigid 
	void calc_H_rgd(Mat & T, Mat & Tx, Mat & Ty, Mat & Q,
		int start_x, int start_y, int end_x, int end_y, Mat & H);
	void rgd_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q, 
		int offset_x, int offset_y);

	// similarity
	void calc_H_sim(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
		int start_x, int start_y, int end_x, int end_y, Mat & H);
	void sim_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q, 
		int offset_x, int offset_y);

	// homography
	void calc_H_hmg(Mat & T, Mat & Tx, Mat & Ty, Mat & Q,
		int start_x, int start_y, int end_x, int end_y, Mat & H);
	void hmg_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q, 
		int offset_x, int offset_y);

	enum e_interpol_type m_interpol_type;

	bool get_warped_pix_val(Mat & I, double & wx, double & wy, double & val)
	{
		int x = (int) wx;
		int y = (int) wy;
		int xp = x + 1;
		int yp = y + 1;
		if(x < 0 || xp >= I.cols || y < 0 || yp >= I.rows)
			return false;
		switch(m_interpol_type){
		case EIT_NN: // nearest neighbour interpolation
			{
				val = (double) I.at<unsigned char>((int) floor(wy+0.5), (int) floor(wx+0.5));
			}
			return true;
		case EIT_BIL: // bi-linear interpolation
			{
				double rx = wx - x;
				double ry = wy - y;
				/*
				double irx = 1 - rx;
				double iry = 1 - ry;
				val = (irx * (double) I.at<unsigned char>(y, x)
					 + rx * (double) I.at<unsigned char>(y, xp)) * iry 
					 + (irx * (double) I.at<unsigned char>(yp, x) 
					 +rx * (double) I.at<unsigned char>(yp, xp)) * ry;
				*/
				
				short p00 = (short) I.at<unsigned char>(y, x);
				short p01 = (short) I.at<unsigned char>(yp, x);
				short p10 = (short) I.at<unsigned char>(y, xp);
				short p11 = (short) I.at<unsigned char>(yp, xp);
				double tmp0 = ((double) p00 + rx * (double) (p10 - p00));
				double tmp1 = ((double) p01 + rx * (double) (p11 - p01));
				val =  tmp0 + ry * (tmp1 - tmp0);
			}
			return true;
		}

		return false;
	}	

public:
	c_imgalign():m_icur(0), m_inew(1), m_weight(true), m_robust(false),
		m_tmpl_blk_sx(2), m_tmpl_blk_sy(2), m_err_th(100.0), m_itr_exit_th(0.1),
		m_num_max_itrs(5), m_wt(EWT_TRN), m_interpol_type(EIT_BIL), m_bmask(false),
		m_mask()
	{
		getDerivKernels(m_kxrow, m_kxcol, 1, 0, 3, true, CV_64F);
		getDerivKernels(m_kyrow, m_kycol, 0, 1, 3, true, CV_64F);
	};

	virtual ~c_imgalign(){};

	void set_interpol_type(e_interpol_type type){
		m_interpol_type = type;
	}

	void set_itr_exit_th(double th)
	{
		m_itr_exit_th = th;
	}

	bool is_conv()
	{
		return m_bconv;
	}

	void set_num_itrs(int num_itrs)
	{
		m_num_max_itrs = num_itrs;
	}

	void set_weight(bool flag)
	{
		m_weight = flag;
	}

	void set_robust(bool flag)
	{
		m_robust = flag;
	}

	void set_robust_th(double th)
	{
		m_err_th = th;
	}

	void set_tmpl_sblk_sz(Size & sz)
	{
		m_tmpl_blk_sx = sz.width;
		m_tmpl_blk_sy = sz.height;
	}

	void set_wt(e_warp_type type){
		m_wt = type;
	}

	e_interpol_type get_interpol_type(){
		return m_interpol_type;
	}

	e_warp_type get_wt(){
		return m_wt;
	}

	double get_delta_norm()
	{
		return m_delta_norm;
	}

	double get_itr_exit_th()
	{
		return m_itr_exit_th;
	}

	int get_num_itrs()
	{
		return m_num_max_itrs;
	}

	bool get_weight()
	{
		return m_weight;
	}

	bool get_robust()
	{
		return m_robust;
	}

	double get_robust_th()
	{
		return m_err_th;
	}

	Size get_tmpl_sblk_sz()
	{
		Size sz(m_tmpl_blk_sx, m_tmpl_blk_sy);
		return sz;
	}

	int get_lv()
	{
		return (int) m_num_itrs.size();
	}

	int get_num_itrs(int ilv)
	{
		return m_num_itrs[ilv];
	}

	double get_rjct_blk(int ilv)
	{
		return m_rjct_blk_ratio[ilv];
	}

	double get_rjct_pix(int ilv)
	{
		return m_rjct_pix_ratio[ilv];
	}

	double get_delta(int ilv)
	{
		return m_delta_p[ilv];
	}

	// Tpyr: pyramid images of a template image 
	// Ipyr: pyramid images of a whole image to be tracked
	// roi: template image ROI in the original whole image
	// Wini: initial warp
	// Return Value: Motion matrix contains Wini
	Mat & calc_warp(vector<Mat> & Tpyr, vector<Mat> & Ipyr,
		Rect & roi, Mat & Wini);
	Mat & calc_warp(vector<Mat> & Tpyr, vector<Mat> & Tmask, vector<Mat> & Ipyr,
		Rect & roi, Mat & Wini);
};

#endif