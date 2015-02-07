#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// c_imgalign.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// c_imgalign.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with c_imgalign.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_thread.h"

#include "util.h"
#include "c_imgalign.h"


static const char * wtname[] = {"trn", "rgd", "sim", "afn", "hmg", "unknown"};
static const char * ipltname[] = {"nn", "bil", "unknown"};

e_warp_type get_warp_type(const char * str)
{
	if(strcmp("trn", str) == 0)
		return EWT_TRN;
	if(strcmp("rgd", str) == 0)
		return EWT_RGD;
	if(strcmp("sim", str) == 0)
		return EWT_SIM;
	if(strcmp("afn", str) == 0)
		return EWT_AFN;
	if(strcmp("hmg", str) == 0)
		return EWT_HMG;
	return EWT_UNKNOWN;
}

const char * get_warp_type_name(e_warp_type type)
{
	return wtname[type];
}

e_interpol_type get_interpol_type(const char * str)
{
	if(strcmp("nn", str) == 0)
		return EIT_NN;
	if(strcmp("bil", str) == 0)
		return EIT_BIL;
	return  EIT_UNKNOWN;
}

const char * get_interpol_type_name(e_interpol_type type)
{
	return ipltname[type];
}

void c_imgalign::calc_H_trn(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
	int start_x, int start_y, int end_x, int end_y, Mat & H)
{
	H = Mat::zeros(2, 2, CV_64FC1);
	m_J = Mat::zeros(1, 2, CV_64FC1);

	double * ptr0, * ptr1;
	for(int x = start_x; x < end_x; x++){
		for(int y = start_y; y < end_y; y++){
			if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
				continue;
			ptr0 = m_J.ptr<double>(0);
			ptr0[0] = Tx.at<double>(y,x);
			ptr0[1] = Ty.at<double>(y,x);
			if(m_weight){
				double q = Q.at<double>(y, x);
				ptr1 = H.ptr<double>(0);
				ptr1[0] += q * ptr0[0] * ptr0[0];
				ptr1[1] += q * ptr0[0] * ptr0[1];
				ptr1 = H.ptr<double>(1);
				ptr1[1] += q * ptr0[1] * ptr0[1];
			}else{
				ptr1 = H.ptr<double>(0);
				ptr1[0] += ptr0[0] * ptr0[0];
				ptr1[1] += ptr0[0] * ptr0[1];
				ptr1 = H.ptr<double>(1);
				ptr1[1] += ptr0[1] * ptr0[1];
			}
		}
	}

	H.at<double>(1, 0) = H.at<double>(0, 1);
}

void c_imgalign::calc_H_rgd(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
	int start_x, int start_y, int end_x, int end_y, Mat & H)
{
	H = Mat::zeros(3, 3, CV_64FC1);
	m_J = Mat::zeros(1, 3, CV_64FC1);
	double c = m_Wtmp[m_icur].at<double>(0, 0), 
		s = m_Wtmp[m_icur].at<double>(1,0);
	double * ptr0, * ptr1;
	for(int x = start_x; x < end_x; x++){
		for(int y = start_y; y < end_y; y++){

			if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
				continue;

			ptr0 = m_J.ptr<double>(0);
			ptr0[0] = Tx.at<double>(y,x);
			ptr0[1] = Ty.at<double>(y,x);
			ptr0[2] = ptr0[0] * (- s * (double) y - c * (double) y) +
				ptr0[1] * (c * (double) x -s * (double) y);
			if(m_weight){
				double q = Q.at<double>(y, x);
				for(int i = 0; i < 3; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += q * ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 3; j++){
						ptr1[j] += q * ptr0[i] * ptr0[j];
					}
				}
			}else{
				for(int i = 0; i < 3; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 3; j++){
						ptr1[j] += ptr0[i] * ptr0[j];
					}
				}
			}
		}
	}

	for(int i = 0; i < 3; i++){
		for(int j = i + 1; j < 3; j++){
			H.at<double>(j, i) = H.at<double>(i, j);
		}
	}
}

void c_imgalign::calc_H_sim(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
	int start_x, int start_y, int end_x, int end_y, Mat & H)
{
	H = Mat::zeros(4, 4, CV_64FC1);
	m_J = Mat::zeros(1, 4, CV_64FC1);
	double * ptr0, * ptr1;
	for(int x = start_x; x < end_x; x++){
		for(int y = start_y; y < end_y; y++){

			if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
				continue;

			ptr0 = m_J.ptr<double>(0);
			ptr0[0] = Tx.at<double>(y,x);
			ptr0[1] = Ty.at<double>(y,x);
			ptr0[2] = ptr0[0] * (double) x + ptr0[1] * (double) y;
			ptr0[3] = - ptr0[0] * (double) y + ptr0[1] * (double) x;

			if(m_weight){
				double q = Q.at<double>(y, x);
				for(int i = 0; i < 4; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += q * ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 4; j++){
						ptr1[j] += q * ptr0[i] * ptr0[j];
					}
				}
			}else{
				for(int i = 0; i < 4; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 4; j++){
						ptr1[j] += ptr0[i] * ptr0[j];
					}
				}
			}
		}
	}

	for(int i = 0; i < 4; i++){
		for(int j = i + 1; j < 4; j++){
			H.at<double>(j, i) = H.at<double>(i, j);
		}
	}
}

void c_imgalign::calc_H_afn(Mat & T, Mat & Tx, Mat & Ty, Mat & Q, 
	int start_x, int start_y, int end_x, int end_y, Mat & H)
{
	H = Mat::zeros(6, 6, CV_64FC1);
	m_J = Mat::zeros(1, 6, CV_64FC1);

	double * ptr0, * ptr1;
	for(int x = start_x; x < end_x; x++){
		for(int y = start_y; y < end_y; y++){

			if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
				continue;

			ptr0 = m_J.ptr<double>(0);
			ptr0[4] = Tx.at<double>(y,x);
			ptr0[5] = Ty.at<double>(y,x);
			ptr0[0] = ptr0[4] * (double) x;
			ptr0[1] = ptr0[5] * (double) x;
			ptr0[2] = ptr0[4] * (double) y;
			ptr0[3] = ptr0[5] * (double) y;
			if(m_weight){
				double q = Q.at<double>(y, x);
				for(int i = 0; i < 6; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += q * ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 6; j++){
						ptr1[j] += q * ptr0[i] * ptr0[j];
					}
				}
			}else{
				for(int i = 0; i < 6; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 6; j++){
						ptr1[j] += ptr0[i] * ptr0[j];
					}
				}
			}
		}
	}

	for(int i = 0; i < 6; i++){
		for(int j = i + 1; j < 6; j++){
			H.at<double>(j, i) = H.at<double>(i, j);
		}
	}
}

void c_imgalign::calc_H_hmg(Mat & T, Mat & Tx, Mat & Ty, Mat & Q,
		int start_x, int start_y, int end_x, int end_y, Mat & H)
{
	H = Mat::zeros(8, 8, CV_64FC1);
	m_J = Mat::zeros(1, 8, CV_64FC1);
	double * ptr0, * ptr1;
	for(int x = start_x; x < end_x; x++){
		for(int y = start_y; y < end_y; y++){

			if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
				continue;

			ptr0 = m_J.ptr<double>(0);
			ptr0[2] = Tx.at<double>(y,x);
			ptr0[5] = Ty.at<double>(y,x);
			ptr0[0] = ptr0[2] * (double) x;
			ptr0[1] = ptr0[2] * (double) y;
			ptr0[3] = ptr0[5] * (double) x;
			ptr0[4] = ptr0[5] * (double) y;
			ptr0[6] = - (ptr0[2] * (x * x) + ptr0[5] * (y * x));
			ptr0[7] = - (ptr0[2] * (x * y) + ptr0[5] * (y * y));

			if(m_weight){
				double q = Q.at<double>(y, x);
				for(int i = 0; i < 8; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += q * ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 8; j++){
						ptr1[j] += q * ptr0[i] * ptr0[j];
					}
				}
			}else{
				for(int i = 0; i < 8; i++){
					ptr1 = H.ptr<double>(i);
					ptr1[i] += ptr0[i] * ptr0[i];
					for(int j = i + 1; j < 8; j++){
						ptr1[j] += ptr0[i] * ptr0[j];
					}
				}
			}
		}
	}

	for(int i = 0; i < 8; i++){
		for(int j = i + 1; j < 8; j++){
			H.at<double>(j, i) = H.at<double>(i, j);
		}
	}
}

void c_imgalign::trn_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q,
	int offset_x, int offset_y)
{
	double * ptr0, * ptr1;
	// gauss-newton iteration
	Mat tmp = Mat::zeros(1, 2, CV_64FC1);
	ptr1 = tmp.ptr<double>(0);
	if(m_robust)
		m_aH = Mat::zeros(2, 2, CV_64FC1);

	for(int start_x = 0, end_x = m_tmpl_blk_sx, i = 0; 
		end_x <= T.cols;
		start_x += m_tmpl_blk_sx, end_x += m_tmpl_blk_sx, i++){

		for(int start_y = 0, end_y = m_tmpl_blk_sy, j = 0;
			end_y <= T.rows; 
			start_y += m_tmpl_blk_sy, end_y += m_tmpl_blk_sy, j++){
			double hw = 1.0;
			for(int x = start_x; x < end_x; x++){
				for(int y = start_y; y < end_y; y++){

					if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
						continue;

					// for wx, wy,  the affine transformed x, y, nearest integer is used.
					ptr0 = m_Wtmp[m_icur].ptr<double>(0);
					double wx = (double) (x + offset_x) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(1);
					double wy = (double) (y + offset_y) + ptr0[2];

					// calc tmp = sum_x { J(x)^t[I(Wx)-T(x)]}
					double val;
					if(!get_warped_pix_val(I, wx, wy, val))
						continue;

					ptr0 = m_J.ptr<double>(0);
					ptr0[0] = Tx.at<double>(y,x);
					ptr0[1] = Ty.at<double>(y,x);
					double diff = val - (double) T.at<unsigned char>(y, x);

					if(m_weight)
						diff *= Q.at<double>(y, x);

					if(m_robust){
						if(fabs(diff) > m_err_th){
							hw = 0.0;
							m_rjct_pix_cnt++;
							continue;
						}
					}

					ptr1[0] += ptr0[0] * diff;
					ptr1[1] += ptr0[1] * diff;
				}
			}
			
			if(m_robust){
				if(hw == 0)
					m_rjct_blk_cnt++;
				m_aH += hw * m_H[i][j];
			}
		}
	}

	if(m_robust)
		m_invH = m_aH.inv(DECOMP_CHOLESKY);

	// delta_p = inv H * tmp (tmp = sum_x { J(x)^t[I(Wx)-T(x)]})
	tmp = m_invH * tmp.t();	
#ifdef UTIL_DBG
	cout << "delta_p=" << tmp << endl;
#endif
	// set iW = inv W(delta p)
	ptr0 = tmp.ptr<double>(0);
	ptr1 = m_invW.ptr<double>(0);
	ptr1[0] /*(0,0)*/ = 1.0;  
	ptr1[1]	/*(0,1)*/ = 0;
	ptr1[2] /*(0,2)*/ = -ptr0[0];
	ptr1[3] /*(1,0)*/ = 0;
	ptr1[4] /*(1,1)*/ = 1.0;
	ptr1[5] /*(1,2)*/ = -ptr0[1];

	// calculating |delta_p|
	m_delta_norm = sqrt(ptr0[0]*ptr0[0] + ptr0[1] * ptr0[1]);

#ifdef UTIL_DBG
	cout << "|delta_p| = " << m_delta_norm << endl;
#endif
	// update W = W * inv W(delta_p)
	synth_afn(m_Wtmp[m_icur], m_invW, m_Wtmp[m_inew]);
}

void c_imgalign::rgd_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q,
	int offset_x, int offset_y)
{
	double * ptr0, * ptr1;
	// gauss-newton iteration
	Mat tmp = Mat::zeros(1, 3, CV_64FC1);
	ptr1 = tmp.ptr<double>(0);
	double c = m_Wtmp[m_icur].at<double>(0, 0), 
		s = m_Wtmp[m_icur].at<double>(1,0);
	if(m_robust)
		m_aH = Mat::zeros(3, 3, CV_64FC1);

	for(int start_x = 0, end_x = m_tmpl_blk_sx, i = 0; 
		end_x <= T.cols;
		start_x += m_tmpl_blk_sx, end_x += m_tmpl_blk_sx, i++){

		for(int start_y = 0, end_y = m_tmpl_blk_sy, j = 0;
			end_y <= T.rows; 
			start_y += m_tmpl_blk_sy, end_y += m_tmpl_blk_sy, j++){
			double hw = 1.0;
			for(int x = start_x; x < end_x; x++){
				for(int y = start_y; y < end_y; y++){

					if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
						continue;

					// for wx, wy,  the affine transformed x, y, nearest integer is used.
					ptr0 = m_Wtmp[m_icur].ptr<double>(0);
					double wx = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(1);
					double wy = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];

					// calc tmp = sum_x { J(x)^t[I(Wx)-T(x)]}
					double val;
					if(!get_warped_pix_val(I, wx, wy, val))
						continue;

					ptr0 = m_J.ptr<double>(0);
					ptr0[0] = Tx.at<double>(y,x);
					ptr0[1] = Ty.at<double>(y,x);
					ptr0[2] = ptr0[0] * (- s * (double) y - c * (double) y) +
						ptr0[1] * (c * (double) x -s * (double) y);

					double diff = val - (double) T.at<unsigned char>(y, x);

					if(m_weight)
						diff *= Q.at<double>(y, x);

					if(m_robust){
						if(fabs(diff) > m_err_th){
							hw = 0.0;	
							m_rjct_pix_cnt++;
							continue;
						}
					}

					for(int i = 0; i < 3; i++)
						ptr1[i] += ptr0[i] * diff;
				}
			}

			if(m_robust){
				if(hw == 0.0)
					m_rjct_blk_cnt++;
				m_aH += hw * m_H[i][j];
			}
		}
	}

	if(m_robust)
		m_invH = m_aH.inv(DECOMP_CHOLESKY);

	// delta_p = inv H * tmp (tmp = sum_x { J(x)^t[I(Wx)-T(x)]})
	tmp = m_invH * tmp.t();	
#ifdef UTIL_DBG
	cout << "delta_p=" << tmp << endl;
#endif
	// set iW = inv W(delta p)
	ptr0 = tmp.ptr<double>(0);
	c = cos(ptr0[2]);
	s = sin(ptr0[2]);
	ptr1 = m_invW.ptr<double>(0);
	ptr1[0] /*(0,0)*/ = c;  
	ptr1[1]	/*(0,1)*/ = s;
	ptr1[2] /*(0,2)*/ = -(c * ptr0[0] +c * ptr0[1]);
	ptr1[3] /*(1,0)*/ = -s;
	ptr1[4] /*(1,1)*/ = c;
	ptr1[5] /*(1,2)*/ = (s * ptr0[0] - c * ptr0[1]);

	// calculating |delta_p|
	m_delta_norm = sqrt(ptr0[0] * ptr0[0] + ptr0[1] * ptr0[1]);

#ifdef UTIL_DBG
	cout << "|delta_p| = " << m_delta_norm << endl;
#endif
	// update W = W * inv W(delta_p)
	synth_afn(m_Wtmp[m_icur], m_invW, m_Wtmp[m_inew]);
}

void c_imgalign::sim_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q,
	int offset_x, int offset_y)
{
	double * ptr0, * ptr1;
	// gauss-newton iteration
	Mat tmp = Mat::zeros(1, 4, CV_64FC1);
	ptr1 = tmp.ptr<double>(0);
	if(m_robust)
		m_aH = Mat::zeros(4, 4, CV_64FC1);

	for(int start_x = 0, end_x = m_tmpl_blk_sx, i = 0; 
		end_x <= T.cols;
		start_x += m_tmpl_blk_sx, end_x += m_tmpl_blk_sx, i++){

		for(int start_y = 0, end_y = m_tmpl_blk_sy, j = 0;
			end_y <= T.rows; 
			start_y += m_tmpl_blk_sy, end_y += m_tmpl_blk_sy, j++){
			double hw = 1.0;
			for(int x = start_x; x < end_x; x++){
				for(int y = start_y; y < end_y; y++){

					if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
						continue;

					// for wx, wy,  the affine transformed x, y, nearest integer is used.
					ptr0 = m_Wtmp[m_icur].ptr<double>(0);
					double wx = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(1);
					double wy = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];

					// calc tmp = sum_x { J(x)^t[I(Wx)-T(x)]}
					double val;
					if(!get_warped_pix_val(I, wx, wy, val))
						continue;

					ptr0 = m_J.ptr<double>(0);
					ptr0[0] = Tx.at<double>(y,x);
					ptr0[1] = Ty.at<double>(y,x);
					ptr0[2] = ptr0[0] * (double) x + ptr0[1] * (double) y;
					ptr0[3] = - ptr0[0] * (double) y + ptr0[1] * (double) x;
					double diff = val - (double) T.at<unsigned char>(y, x);

					if(m_weight)
						diff *= Q.at<double>(y, x);

					if(m_robust){
						if(fabs(diff) > m_err_th){
							hw = 0.0;	
							m_rjct_pix_cnt++;
							continue;
						}
					}

					for(int i = 0; i < 4; i++)
						ptr1[i] += ptr0[i] * diff;
				}
			}
			
			if(m_robust){
				if(hw == 0.0)
					m_rjct_blk_cnt++;
				m_aH += hw * m_H[i][j];
			}
		}
	}

	if(m_robust)
		m_invH = m_aH.inv(DECOMP_CHOLESKY);

	// delta_p = inv H * tmp (tmp = sum_x { J(x)^t[I(Wx)-T(x)]})
	tmp = m_invH * tmp.t();	
#ifdef UTIL_DBG
	cout << "delta_p=" << tmp << endl;
#endif
	// set iW = inv W(delta p)
	ptr0 = tmp.ptr<double>(0);
	ptr1 = m_invW.ptr<double>(0);
	double a = 1.0 + ptr0[2];
	double coeff = 1.0 / (a * a + ptr0[3] * ptr0[3]);
	ptr1[0] /*(0,0)*/ = coeff * a;  
	ptr1[1]	/*(0,1)*/ = coeff * ptr0[3];
	ptr1[2] /*(0,2)*/ = -coeff * (a * ptr0[0] + ptr0[3] * ptr0[1]);
	ptr1[3] /*(1,0)*/ = -coeff * ptr0[3];
	ptr1[4] /*(1,1)*/ = coeff * a;
	ptr1[5] /*(1,2)*/ = coeff * (ptr0[3] * ptr0[0] - a * ptr0[1]);

	// calculating |delta_p|
	m_delta_norm = sqrt(ptr0[0] * ptr0[0] + ptr0[1] * ptr0[1]);

#ifdef UTIL_DBG
	cout << "|delta_p| = " << m_delta_norm << endl;
#endif
	// update W = W * inv W(delta_p)
	synth_afn(m_Wtmp[m_icur], m_invW, m_Wtmp[m_inew]);
}


void c_imgalign::afn_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q,
	int offset_x, int offset_y)
{
	double * ptr0, * ptr1;
	// gauss-newton iteration
	Mat tmp = Mat::zeros(1, 6, CV_64FC1);
	ptr1 = tmp.ptr<double>(0);
	if(m_robust)
		m_aH = Mat::zeros(6, 6, CV_64FC1);

	for(int start_x = 0, end_x = m_tmpl_blk_sx, i = 0; 
		end_x <= T.cols;
		start_x += m_tmpl_blk_sx, end_x += m_tmpl_blk_sx, i++){

		for(int start_y = 0, end_y = m_tmpl_blk_sy, j = 0;
			end_y <= T.rows; 
			start_y += m_tmpl_blk_sy, end_y += m_tmpl_blk_sy, j++){
			double hw = 1.0;
			for(int x = start_x; x < end_x; x++){
				for(int y = start_y; y < end_y; y++){

					if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
						continue;

					// for wx, wy,  the affine transformed x, y, nearest integer is used.
					ptr0 = m_Wtmp[m_icur].ptr<double>(0);
					double wx = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(1);
					double wy = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];

					// calc tmp = sum_x { J(x)^t[I(Wx)-T(x)]}
					double val;
					if(!get_warped_pix_val(I, wx, wy, val))
						continue;

					ptr0 = m_J.ptr<double>(0);
					ptr0[4] = Tx.at<double>(y,x);
					ptr0[5] = Ty.at<double>(y,x);
					ptr0[0] = ptr0[4] * (double) x;
					ptr0[1] = ptr0[5] * (double) x;
					ptr0[2] = ptr0[4] * (double) y;
					ptr0[3] = ptr0[5] * (double) y;

					double diff = val - (double) T.at<unsigned char>(y, x);
					if(m_weight)
						diff *= Q.at<double>(y, x);

					if(m_robust){
						if(fabs(diff) > m_err_th){
							hw = 0.0;	
							m_rjct_pix_cnt++;
							continue;
						}
					}

					for(int i = 0; i < 6; i++)
						ptr1[i] += ptr0[i] * diff;
				}
			}

			if(m_robust){
				if(hw == 0.0)
					m_rjct_blk_cnt++;
				m_aH += hw * m_H[i][j];
			}
		}
	}

	if(m_robust)
		m_invH = m_aH.inv(DECOMP_CHOLESKY);

	// delta_p = inv H * tmp (tmp = sum_x { J(x)^t[I(Wx)-T(x)]})
	tmp = m_invH * tmp.t();	
#ifdef UTIL_DBG
	cout << "delta_p=" << tmp << endl;
#endif
	// set iW = inv W(delta p)
	ptr0 = tmp.ptr<double>(0);
	ptr1 = m_invW.ptr<double>(0);
	double coeff = (1.0 / ((1 + ptr0[0]) * (1 + ptr0[3]) - ptr0[1] * ptr0[2]));
	ptr1[0] /*(0,0)*/ = coeff * (1 + ptr0[3]);  
	ptr1[1]	/*(0,1)*/ = -coeff * ptr0[2];
	ptr1[2] /*(0,2)*/ = coeff * (-ptr0[4] - ptr0[3] * ptr0[4] + ptr0[2] * ptr0[5]);
	ptr1[3] /*(1,0)*/ = -coeff * ptr0[1];
	ptr1[4] /*(1,1)*/ = coeff * (1 + ptr0[0]);
	ptr1[5] /*(1,2)*/ = coeff * (-ptr0[5] - ptr0[0] * ptr0[5] + ptr0[1] * ptr0[4]);

	m_delta_norm = sqrt(ptr0[4] * ptr0[4] + ptr0[5] * ptr0[5]);

#ifdef UTIL_DBG
	cout << "|delta_p| = " << m_delta_norm << endl;
#endif
	// update W = W * inv W(delta_p)
	synth_afn(m_Wtmp[m_icur], m_invW, m_Wtmp[m_inew]);
}


void c_imgalign::hmg_itr(Mat & T, Mat & Tx, Mat & Ty, Mat & I, Mat & Q,
	int offset_x, int offset_y)
{
	double * ptr0, * ptr1;
	// gauss-newton iteration
	Mat tmp = Mat::zeros(1, 8, CV_64FC1);
	ptr1 = tmp.ptr<double>(0);
	if(m_robust)
		m_aH = Mat::zeros(8, 8, CV_64FC1);

	for(int start_x = 0, end_x = m_tmpl_blk_sx, i = 0; 
		end_x <= T.cols;
		start_x += m_tmpl_blk_sx, end_x += m_tmpl_blk_sx, i++){

		for(int start_y = 0, end_y = m_tmpl_blk_sy, j = 0;
			end_y <= T.rows; 
			start_y += m_tmpl_blk_sy, end_y += m_tmpl_blk_sy, j++){
			double hw = 1.0;
			for(int x = start_x; x < end_x; x++){
				for(int y = start_y; y < end_y; y++){
					if(m_bmask && m_mask.at<unsigned char>(y, x) == 0)
						continue;

					// for wx, wy,  the affine transformed x, y, nearest integer is used.
					ptr0 = m_Wtmp[m_icur].ptr<double>(0);
					double wx = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(1);
					double wy = ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + ptr0[2];
					ptr0 = m_Wtmp[m_icur].ptr<double>(2);
					double coeff = 1.0 / (ptr0[0] * (double) (x + offset_x) + ptr0[1] * (double) (y + offset_y) + 1);
					wx *= coeff;
					wy *= coeff;

					// calc tmp = sum_x { J(x)^t[I(Wx)-T(x)]}
					double val;
					if(!get_warped_pix_val(I, wx, wy, val))
						continue;

					ptr0 = m_J.ptr<double>(0);
					ptr0[2] = Tx.at<double>(y,x);
					ptr0[5] = Ty.at<double>(y,x);
					ptr0[0] = ptr0[2] * (double) x;
					ptr0[1] = ptr0[2] * (double) y;
					ptr0[3] = ptr0[5] * (double) x;
					ptr0[4] = ptr0[5] * (double) y;
					ptr0[6] = - (ptr0[2] * (x * x) + ptr0[5] * (y * x));
					ptr0[7] = - (ptr0[2] * (x * y) + ptr0[5] * (y * y));
					double diff = val - (double) T.at<unsigned char>(y, x);

					if(m_weight)
						diff *= Q.at<double>(y, x);

					if(m_robust){
						if(fabs(diff) > m_err_th){
							hw = 0.0;	
							m_rjct_pix_cnt++;
							continue;
						}
					}

					for(int i = 0; i < 8; i++)
						ptr1[i] += ptr0[i] * diff;
				}
			}

			if(m_robust){
				if(hw == 0.0)
					m_rjct_blk_cnt++;
				m_aH += hw * m_H[i][j];
			}
		}
	}

	if(m_robust)
		m_invH = m_aH.inv(DECOMP_CHOLESKY);

	// delta_p = inv H * tmp (tmp = sum_x { J(x)^t[I(Wx)-T(x)]})
//	cout << "diff = " << tmp << endl;
	tmp = m_invH * tmp.t();	
#ifdef UTIL_DBG
	cout << "delta_p=" << tmp << endl;
#endif
	// set iW = inv W(delta p)
	ptr0 = tmp.ptr<double>(0);
	ptr1 = m_invW.ptr<double>(0);
	for(int i = 0; i < 8; i++)
		ptr1[i] = ptr0[i];
	ptr1[0] += 1.0;
	ptr1[4] += 1.0;
	ptr1[8] = 1.0;

	// calculating |delta_p|
	m_delta_norm = sqrt(ptr0[2] * ptr0[2] + ptr0[5] * ptr0[5]);

#ifdef UTIL_DBG
	cout << "|delta_p| = " << m_delta_norm << endl;
#endif
	// update W = W * inv W(delta_p)
	m_Wtmp[m_inew] = m_Wtmp[m_icur] * m_invW.inv();
	ptr1 = m_Wtmp[m_inew].ptr<double>(0);
	double coeff = 1.0 / ptr1[8];
	for(int i = 0; i < 8; i++)
		ptr1[i] *= coeff;
	ptr1[8] = 1.0;
}


Mat & c_imgalign::calc_warp(vector<Mat> & Tpyr, vector<Mat> & Ipyr, 
	Rect & roi, Mat & Wini)
{
	bool init = false;
	m_bconv = false;

	if(m_wt == EWT_HMG){
		m_invW = Mat::eye(3, 3, CV_64FC1);
		m_Wtmp[0] = Mat::eye(3, 3, CV_64FC1);
		m_Wtmp[1] = Mat::eye(3, 3, CV_64FC1);
	}else{
		m_invW = Mat::eye(2, 3, CV_64FC1);
		m_Wtmp[0] = Mat::eye(2, 3, CV_64FC1);
		m_Wtmp[1] = Mat::eye(2, 3, CV_64FC1);
	}

	int levels = min((int) Tpyr.size(), (int) Ipyr.size());

	int offset_x, offset_y;

	init_statistics(levels);

	Mat Tx, Ty, Q;
	for(int ilv = levels - 1; ilv >= 0; ilv--){
		if(m_bmask)
			m_mask = m_Tmask[ilv];

		if(!init){ // scaling initial warp parameters according to pyramid levels
			Wini.copyTo(m_Wtmp[m_icur]);

			double scale = pow(0.5, (double) (ilv));
			m_Wtmp[m_icur].at<double>(0, 2) *= scale;
			m_Wtmp[m_icur].at<double>(1, 2) *= scale;
			if(m_wt == EWT_HMG){
				scale = pow(2.0, (double) (ilv));
				m_Wtmp[m_icur].at<double>(2, 0) *= scale;
				m_Wtmp[m_icur].at<double>(2, 1) *= scale;

			}
			init = true;
		}

#ifdef UTIL_DBG
		cout << "level " << ilv << " Initial Warp =" << endl;
		cout << m_Wtmp[m_icur] << endl;
#endif
		offset_x = roi.x >> ilv;
		offset_y = roi.y >> ilv;

		// pyramid iteration
		// calc grad T = (Tx, Ty)
		Mat T = Tpyr[ilv];
		Mat I = Ipyr[ilv];
		sepFilter2D(T, Tx, CV_64F, m_kxrow, m_kxcol);
		sepFilter2D(T, Ty, CV_64F, m_kyrow, m_kycol);

		// calc weight function (|grad T|)
		if(m_weight){
			Q.create(Tx.rows, Tx.cols, CV_64F);
			MatConstIterator_<double> tx = Tx.begin<double>(), tx_end = Tx.end<double>();
			MatConstIterator_<double> ty = Ty.begin<double>();
			MatIterator_<double> q = Q.begin<double>();
			for(; tx != tx_end; tx++, ty++, q++){
				*q = 1.0 / max(sqrt(*tx * *tx + *ty * *ty), 1.0);
			}
		}

		// calc H = sum_x J(x)^t * J(x) where J(x) = grad T(x) * dW/dp|p=0 
		int sx, sy;
		if(m_robust){
			sx = m_tmpl_blk_sx;
			sy = m_tmpl_blk_sy;
		}else{
			sx = T.cols;
			sy = T.rows;
		}

		int num_tblk_x = T.cols / sx;
		int num_tblk_y = T.rows / sy;
		
		if(num_tblk_x > m_H.size())
			m_H.resize(num_tblk_x);

		for(int start_x = 0, end_x = sx, i = 0; 
			end_x <= T.cols; 
			start_x += sx, end_x += sx, i++){

			if(num_tblk_y > m_H[i].size())
				m_H[i].resize(num_tblk_y);

			for(int start_y = 0, end_y = sy, j = 0; 
				end_y <= T.rows; 
				start_y += sy, end_y += sy, j++){
				switch(m_wt){
				case EWT_TRN:
					calc_H_trn(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 2x2 hessian
					break;
				case EWT_AFN: 
					calc_H_afn(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 6x6 hessian
					break;
				case EWT_RGD:
					calc_H_rgd(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 3x3 hessian
					break;
				case EWT_SIM:
					calc_H_sim(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 4x4 hessian
					break;
				case EWT_HMG:
					calc_H_hmg(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 8x8 hessian
					break;
				default:
					calc_H_afn(T, Tx, Ty, Q,
						start_x, start_y, 
						end_x, end_y, 
						m_H[i][j]); // 6x6 hessian
					break;
				}
			}
		}

		// if robust function is not used, the inverse of Hessian
		// can be calculated preliminaly to the iterations.
		if(!m_robust)
			m_invH = m_H[0][0].inv(DECOMP_CHOLESKY);

		// calc SD
		int itr;
		m_delta_norm = DBL_MAX;

		for(itr = 0; m_delta_norm > m_itr_exit_th && itr < m_num_max_itrs ; itr++){
			init_itr_statistics();
			switch(m_wt){
			case EWT_TRN:
				trn_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			case EWT_AFN:
				afn_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			case EWT_RGD:
				rgd_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			case EWT_SIM:
				sim_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			case EWT_HMG:
				hmg_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			default:
				afn_itr(T, Tx, Ty, I, Q, offset_x, offset_y);
				break;
			}
#ifdef UTIL_DBG
			cout << "level " << ilv << " itr " << itr << " Warp " << endl;
			cout << m_Wtmp[m_inew] << endl;
#endif
			m_icur = (m_icur == 0 ? 1 : 0);
			m_inew = (m_inew == 0 ? 1 : 0);
		}

		// the case iteration did not converge
		if(itr == m_num_max_itrs && m_delta_norm > m_itr_exit_th){
			init = false;
			m_bconv = false;
		}else{
			m_bconv = true;
		}

		calc_itr_statistics(ilv, itr, T.rows * T.cols, num_tblk_x * num_tblk_y);

		// doulbe the parameters of W
		if(ilv != 0){
			m_Wtmp[m_icur].at<double>(0, 2) *= 2.0;
			m_Wtmp[m_icur].at<double>(1, 2) *= 2.0;
			if(m_wt == EWT_HMG){
				m_Wtmp[m_icur].at<double>(2, 0) *= 0.5;
				m_Wtmp[m_icur].at<double>(2, 1) *= 0.5;
			}
		}
	}

	if(!init)
		Wini.copyTo(m_Wtmp[m_icur]);

	return m_Wtmp[m_icur];
}

Mat & c_imgalign::calc_warp(vector<Mat> & Tpyr, vector<Mat> & Tmask, vector<Mat> & Ipyr,
		Rect & roi, Mat & Wini)
{
	m_bmask = true;
	m_Tmask = Tmask;
	Mat W = calc_warp(Tpyr, Ipyr, roi, Wini);
	m_bmask = false;
	return m_Wtmp[m_icur];
}