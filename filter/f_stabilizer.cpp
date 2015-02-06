#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_stabilizer.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_stabilizer.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_stabilizer.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"
#include "../util/util.h"
#include "../util/c_imgalign.h"
#include "../util/c_ship.h"

#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_vector.h"

#include "f_base.h"
#include "f_stabilizer.h"

////////////////////////////////////////////////////////////// f_stabilizer
const char * f_stabilizer::m_strIntlType[EIT_UNKNOWN] = 
{
	"nn", "bil"
};

const char * f_stabilizer::m_strWarpType[EWT_UNKNOWN] = 
{
	"trn", "rgd", "sim", "afn", "hmg"
};

// cmd_proc is discarded soon when I confirmed the fset/fget work sufficiently
bool f_stabilizer::cmd_proc(s_cmd & cmd)
{
	int num_args = cmd.num_args;
	char ** args = cmd.args;

	if(num_args < 2)
		return false;

	if(num_args == 2){
		cout << "roi <left> <top> <width> <height>" << endl;
		cout << "plv <num>, itr <num>, clr_ref, w" << endl;
		cout << "wt <trn, rgd, sim, afn>" << endl;
		cout << "ipt <nn, bil>" << endl;
		cout << "rb, rbth <threshold>, rsbsz <width> <height>" << endl;
		cout << "alpha <0 to 1.0> // moving average coefficient" << endl;
		cout << "m <file name> // mask file (single channel) " << endl;
		cout << "disp // turn on or off telop" << endl;
		cout << "through // turn on or off the filter" << endl;

		return false;
	}

	int itok = 2;

	if(strcmp(args[itok], "roi") == 0){
		if(num_args != 7)
			return false;

		m_roi.x = atoi(args[itok+1]);
		m_roi.y = atoi(args[itok+2]);
		m_roi.width = atoi(args[itok+3]);
		m_roi.height = atoi(args[itok+4]);
		return true;
	}else if(strcmp(args[itok], "plv") == 0){
		if(num_args != 4)
			return false;
		m_num_pyr_level = atoi(args[itok+1]);
		return true;
	}else if(strcmp(args[itok], "itr") == 0){
		if(num_args != 4)
			return false;
		m_core.set_num_itrs(atoi(args[itok+1]));
		return true;
	}else if(strcmp(args[itok], "log") == 0){
		if(num_args != 4)
			return false;
		ofstream file(args[itok+1], ios_base::out | ios_base::app);
		if(!file.is_open())
			return false;

		file << m_roi.x << ",";
		file << m_roi.y << ",";
		file << m_roi.width << ",";
		file << m_roi.height << ",";
		file << m_num_pyr_level << ",";
		file << m_core.get_itr_exit_th() << ",";
		file << (m_core.get_weight() ? "yes":"no") << ",";
		file << (m_core.get_robust() ? "yes":"no") << ",";
		file << m_core.get_tmpl_sblk_sz().width << ",";
		file << m_core.get_tmpl_sblk_sz().height << ",";
		file << get_warp_type_name(m_core.get_wt()) << ",";
		file << get_interpol_type_name(m_core.get_interpol_type()) << ",";
		file << m_num_conv_frms << ",";
		file << m_num_frms;
		file << endl;
		return true;
	}else if(strcmp(args[itok], "clr") == 0){
		ch_image * pgryin = dynamic_cast<ch_image*>(m_chin[0]);
		if(pgryin == NULL)
			return false;
		ch_image * pgryout = dynamic_cast<ch_image*>(m_chout[0]);
		if(pgryout == NULL)
			return false;

		long long timg;
		Mat img = pgryin->get_img(timg);
		pgryout->set_img(img, timg);
		m_num_conv_frms = m_num_frms = 0;
		m_bWinit = false;
		return true;
	}else if(strcmp(args[itok], "w") == 0){
		m_core.set_weight(!m_core.get_weight());
		return true;
	}else if(strcmp(args[itok], "m") == 0){
		m_Tmask.clear();
		m_mask.release();
		if(num_args != 4){
			m_bmask = false;
			return  false;
		}

		m_mask = imread(args[itok+1], 0);

		if(m_mask.depth() != CV_8U)
			return true;

		m_bmask = true;
		return true;
	}else if(strcmp(args[itok], "rb") == 0){
		m_core.set_robust(!m_core.get_robust());
		return true;
	}else if(strcmp(args[itok], "rbth") == 0){
		if(num_args != 4)
			return false;
		m_core.set_robust_th(atof(args[itok+1]));
		return true;
	}else if(strcmp(args[itok], "rsbsz") == 0){
		if(num_args != 5)
			return false;
		Size sz(atoi(args[itok+1]), atoi(args[itok+2]));
		m_core.set_tmpl_sblk_sz(sz);
		return true;
	}else if(strcmp(args[itok], "wt") == 0){
		if(num_args != 4)
			return false;
		m_core.set_wt(get_warp_type(args[itok+1]));
		m_bWinit = false;
		return true;
	}else if(strcmp(args[itok], "ipt") == 0){
		if(num_args != 4)
			return false;
		m_core.set_interpol_type(get_interpol_type(args[itok+1]));
		return true;
	}else if(strcmp(args[itok], "alpha") == 0){
		if(num_args == 4){
			set_alpha(atof(args[itok+1]));
			return true;
		}

		if(num_args == 5){
			int ialpha = atoi(args[itok+1]);

			if(ialpha >= 9 || ialpha < 0){
				cout << "Index of the alpha parameter should be less than 9" << endl;
				return false;
			}

			m_alpha[ialpha] = atof(args[itok+2]);
			cout << "alpha[" << ialpha << "] = " << m_alpha[ialpha] << endl;
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "beta") == 0){
		if(num_args == 4){
			m_beta = atof(args[itok + 1]);
			return true;
		}
		return false;
	}else if(strcmp(args[itok], "through") == 0){
		m_bthrough = !m_bthrough;
		return true;
	}else if(strcmp(args[itok], "disp") == 0){
		m_disp_inf = !m_disp_inf;
		return true;
	}

	return f_base::cmd_proc(cmd);
}

bool f_stabilizer::proc(){	

	ch_image * pgryin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pgryin == NULL)
		return false;
	ch_image * pclrin = dynamic_cast<ch_image*>(m_chin[1]);
	if(pclrin == NULL)
		return false;

	ch_image * pgryout = dynamic_cast<ch_image*>(m_chout[0]);
	if(pgryout == NULL)
		return false;

	ch_image * pclrout = dynamic_cast<ch_image*>(m_chout[1]);
	if(pclrout == NULL)
		return false;
	long long tgry;
	Mat gry = pgryin->get_img(tgry);
	if(gry.empty())
		return true;

	long long tclr;
	Mat clr = pclrin->get_img(tclr);
	if(clr.empty())
		return true;

	if(m_bthrough){
		pgryout->set_img(gry, tgry);
		pclrout->set_img(clr, tclr);
		return true;
	}

	if(m_bclr)
	{
		long long timg;
		Mat img = pgryin->get_img(timg);
		pgryout->set_img(img, timg);
		m_num_conv_frms = m_num_frms = 0;
		m_bWinit = false;
		m_bclr = false;
		return true;
	}

	if(tgry != tclr)
		return true;

	// getting new gray image to be stabilized
	//Mat & gry = m_pgryin->get_img();
	
	// roi saturation 
	if(m_roi.x < 0 || m_roi.x > gry.cols)
		m_roi.x = 0;
	if(m_roi.y < 0 || m_roi.y > gry.rows)
		m_roi.y = 0;
	if(m_roi.width <= 0 || (m_roi.x + m_roi.width) >= gry.cols)
		m_roi.width = gry.cols - m_roi.x;
	if(m_roi.height <= 0 || (m_roi.y + m_roi.height) >= gry.rows)
		m_roi.height = gry.rows - m_roi.y;

	// making image pyramid of a new image
	buildPyramid(gry, m_pyrimg[0], m_num_pyr_level);

	// making a template and its pyramid
	if(m_refimg.rows != gry.rows &&
		m_refimg.cols != gry.cols )
		m_refimg = gry.clone();

	if(m_bmask && m_Tmask.size() != m_num_pyr_level)
		buildPyramid(m_mask, m_Tmask, m_num_pyr_level);

	// making image pyramid of the template image sampled from ROI 
	// in the previous image stabilized with warp parameters m_W
	buildPyramid(m_refimg(m_roi), m_pyrimg[1], m_num_pyr_level);
	if(!m_bWinit){
		init();
		m_bWinit = true;
	}

	// calculating new warp. The new warp contains the previous warp. 
	// Warp function basically is  ROI of Wnew(Original Image) = ROI of Wold(Previous Image) 
	Mat Wnew; 
	
	if(m_bmask)
		Wnew = m_core.calc_warp(m_pyrimg[1] /* previous image template*/,
			m_Tmask /* Mask image. Pixels with value zero is ignored in the error calculation.*/, 
			m_pyrimg[0] /* image the warp to be calculated */,
			m_roi /* roi for template sampling */,  
			m_W /* previous warp */);
	else
		Wnew = m_core.calc_warp(m_pyrimg[1] /* previous image */,
			m_pyrimg[0] /* image the warp to be calculated */, 
			m_roi/* roi for template sampling */, 
			m_W /* previous warp */);

	// convergence check
	if(m_core.is_conv())
		m_num_conv_frms++;
	else // if not, the previous warp parameters are used.
		m_W.copyTo(Wnew);

	m_num_frms++;

	// Motion parameters are estimated as the moving average of Wnew
	Mat clrout, gryout;
	switch(m_core.get_wt()){
	case EWT_AFN:
	case EWT_SIM:
	case EWT_RGD:
		{
			double theta0 = asin(m_M.at<double>(1, 0));
			double theta1 = asin(Wnew.at<double>(1, 0));
			// we want to cancel the rotation. but still need to be canceled slightly.
			// because the alignment errors cause unintended rotation.
			// Very small (m_alpha[0] + m_beta) sets the camear's average rotation as the stable rotation
			theta0 = /*(1 - m_alpha[0]) * theta0 + */(m_alpha[0] + m_beta) * theta1;
			double s0, c0;
			s0 = sin(theta0);
			m_M.at<double>(0, 1) = -s0;
			m_M.at<double>(1, 0) = s0;
			theta1 = cos(theta0);
			m_M.at<double>(0, 0) = c0;
			m_M.at<double>(1, 1) = c0;
		}
	case EWT_TRN:
		// usually we dont want to cancel the horizontal motion. The coefficient m_alpha[2] + m_beta should nearly be 1.0
		m_M.at<double>(0, 2) = /*(1 - m_alpha[2]) * m_M.at<double>(0, 2) +*/
			(m_alpha[2] + m_beta) * Wnew.at<double>(0, 2);
		m_M.at<double>(1, 2) = /*(1 - m_alpha[5]) * m_M.at<double>(1, 2) +*/
			 (m_alpha[5] + m_beta) * Wnew.at<double>(1, 2);

		// Here m_M is the true motion it should not be canceled
		invertAffineTransform(m_M, m_iM);

		// Subtract m_M by multiplying Wnew and m_iM the inverse of m_M
		synth_afn(Wnew, m_iM, m_W);

		// Applying resulting warp function for both color and gray scale image
		warpAffine(clr, clrout, 
			m_W, Size(clr.cols, clr.rows)
			, INTER_LINEAR | WARP_INVERSE_MAP);

		warpAffine(gry, gryout,
			m_W, Size(gry.cols, gry.rows)
			, INTER_LINEAR | WARP_INVERSE_MAP);
		break;
	case EWT_HMG:
		m_iM = m_M.inv();
		m_W = Wnew * m_iM;
		warpPerspective(clr, clrout, 
			m_W, Size(clr.cols, clr.rows)
			, INTER_LINEAR | WARP_INVERSE_MAP);
		warpPerspective(gry, gryout,
			m_W, Size(gry.cols, gry.rows)
			, INTER_LINEAR | WARP_INVERSE_MAP);
		break;
	}
	
	m_refimg = gryout;
	pgryout->set_img(gryout, tgry);
	pclrout->set_img(clrout, tclr);

	// output image 
	if(!m_disp_inf)
		return true;
	cv::rectangle(clrout, m_roi, CV_RGB(255, 0, 0), 2);

	sprintf(m_str, "plv: %d, max_itr: %d, exit_th: %f, w: %s", 
		m_num_pyr_level,
		m_core.get_num_itrs(),
		m_core.get_itr_exit_th(),
		(m_core.get_weight() ? "yes":"no"));

	cv::putText(clrout, m_str, Point(10, 20), 
		FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 200, 100));

	sprintf(m_str, "wt : %s ipt: %s ", 
		get_warp_type_name(m_core.get_wt()),
		get_interpol_type_name(m_core.get_interpol_type()));
	cv::putText(clrout, m_str, Point(10, 40),
		FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 200, 100));

	Size sz = m_core.get_tmpl_sblk_sz();
	sprintf(m_str, "r : %s th: %f sbsz: (%d, %d)", 
		(m_core.get_robust() ? "yes":"no"), 
		m_core.get_robust_th(),
		sz.width, sz.height);
	cv::putText(clrout, m_str, Point(10, 60),
		FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 200, 100));

	sprintf(m_str, "Convergence ratio = %f ( %d / %d )", 
		(double) m_num_conv_frms/ (double) m_num_frms,
		m_num_conv_frms, m_num_frms);
	cv::putText(clrout, m_str, Point(10, 80),
		FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 200, 100));

	int num_lvs = m_core.get_lv();
	for(int ilv = 0; ilv < num_lvs; ilv++){
		sprintf(m_str, "lv: %d, itr: %d, delta: %f pix_rat: %f, blk_rat: %f",
			ilv, m_core.get_num_itrs(ilv),
			m_core.get_delta(ilv),
			m_core.get_rjct_pix(ilv),
			m_core.get_rjct_blk(ilv));
		cv::putText(clrout, m_str, Point(10, 100 + ilv * 20 ),
			FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 200, 100));
	}

	return true;
}


/////////////////////////////////////////////////////// f_tracker
f_tracker::f_tracker(const char * name):f_base(name),
	m_pobjin(NULL), m_pgryin(NULL),
	m_num_pyr_levels(0)
{
	m_core.set_interpol_type(EIT_BIL);
	//		m_core.set_wt(EWT_SIM);
	m_core.set_wt(EWT_TRN);

	m_core.set_num_itrs(30);
}

f_tracker::~f_tracker()
{
}

bool f_tracker::cmd_proc(s_cmd & cmd)
{
	return true;
}

bool f_tracker:: proc()
{	
	ch_image * pgryin = dynamic_cast<ch_image*>(m_chin[0]);
	if(pgryin == NULL)
		return false;

	ch_vector<c_track_obj> * pobjin = dynamic_cast<ch_vector<c_track_obj> *>(m_chin[1]);
	if(pobjin == NULL)
		return false;

	ch_vector<c_track_obj> * pobjout = dynamic_cast<ch_vector<c_track_obj> *>(m_chout[0]);
	if(pobjout == NULL)
		return false;
	long long timg;
	Mat gimg = pgryin->get_img(timg);
	if(gimg.empty())
		return true;

	buildPyramid(gimg, m_pyrimg, m_num_pyr_levels);

	c_track_obj * pobj;
	Mat Mnew;

	while((pobj = pobjin->pop()) != NULL){

		if(!pobj->is_apyr_set()){ // apearance has not been set yet
			Mat apimg  = gimg(pobj->get_rc());
			pobj->set_apimg(apimg, m_num_pyr_levels);
			pobjout->push(pobj);
			continue;
		}

		// track 
		Mnew = m_core.calc_warp(pobj->get_apyr(), 
			m_pyrimg, pobj->get_rc(), pobj->get_motion());

		// if tracking failed delete the object
		if(!m_core.is_conv()){
			pobj->set_lost();
			pobjout->push(pobj);
			cerr << "tracking failed." << endl;
			continue;
		}

		pobj->set_motion(Mnew);

		// if tracking succeeded push
		pobjout->push(pobj);
	}

	return true;
}