// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_stereo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_stereo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_stereo.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/aws_sock.h"
#include "../util/aws_vlib.h"

#include "../channel/ch_image.h"

#include "f_base.h"


class f_stereo : public f_base
{
protected:
	ch_image_ref * m_ch_img1, *m_ch_img2;
	ch_image_ref * m_ch_disp;

	enum s_out{
		DISP, IMG1, IMG2
	} m_out;
	static const char * m_str_out[IMG2 + 1];

	Mat m_img1, m_img2, m_disp;
	long long m_timg1, m_timg2;
	long long m_ifrm1, m_ifrm2;
	long long m_ifrm_diff;
	int m_fm_count;
	int m_fm_max_count;
	int m_fm_time_min_dfrm;
	int m_fm_time_min;

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
s_sgbm_par m_sgbm_par;

	Ptr<StereoBM> m_bm;
public:
	f_stereo(const char * name);
	virtual ~f_stereo();

	virtual bool init_run();
	virtual void destroy_run();
	virtual bool proc();
};
