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

class f_stabilizer: public f_base
{
protected:
	vector<Mat> m_pyrimg[2];
	int m_num_pyr_level;

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

public:
	f_stabilizer(const char * name):f_base(name), 
		m_beta(0.0001), m_disp_inf(true), 
		m_num_pyr_level(4), m_roi(0,0,0,0), m_bWinit(false),
		m_bmask(false), m_bthrough(false), m_num_conv_frms(0),
		m_num_frms(0)
	{
		set_alpha(0.1);
		m_core.set_num_itrs(5);
	}

	virtual ~f_stabilizer()
	{
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