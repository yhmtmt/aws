// Copyright(c) 2016 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_ship_detector.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_ship_detector.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_ship_detector.h.  If not, see <http://www.gnu.org/licenses/>. 

#define DEBUG_SD

#ifndef _F_SHIP_DETECTOR_H_
#define _F_SHIP_DETECTOR_H_

#include "../util/aws_sock.h"

#include "../channel/ch_image.h"
#include "../channel/ch_vector.h"

#include "f_base.h"

class f_ship_detector: public f_base
{
protected:
	ch_image * m_pin;
	ch_vector<Rect> * m_pout;

	struct s_wf{
		int x, y;
		int d;
		s_wf():x(0),y(0),d(0){};
		s_wf(int vx, int vy, int vd):x(vx), y(vy), d(vd){};
	};

	Mat m_label, m_bin;
	int m_sd;
	int m_wait_cnt;
	vector<s_wf> m_sq;
	int m_num_depth_per_chan;
	int m_num_bins_per_chan;
	double m_alpha;
	double m_th;
	int m_th_pix;
	double *** m_hist;
	int m_num_smpls;
	unsigned int *** m_hist_tmp;
	vector<Rect> m_smplroi;
	vector<Rect> m_dtctroi; // roi for detection

public:
	f_ship_detector(const char * name): f_base(name),
		m_pin(NULL), m_pout(NULL), m_hist(NULL),
		m_hist_tmp(NULL), m_num_depth_per_chan(4),
		m_alpha(0.1), m_th(1e-8), m_sd(3), m_th_pix(15), m_wait_cnt(0)
	{
		init();
	}

	virtual ~f_ship_detector()
	{
		free();
	}

	void init()
	{
		if(m_hist != NULL)
			free();
		m_num_bins_per_chan = 1 << m_num_depth_per_chan;
		m_hist = new double**[m_num_bins_per_chan];
		m_hist_tmp = new unsigned int**[m_num_bins_per_chan];
		for(int i = 0; i < m_num_bins_per_chan; i++){
			m_hist[i] = new double*[m_num_bins_per_chan];
			m_hist_tmp[i] = new unsigned int*[m_num_bins_per_chan];
			for(int j = 0; j < m_num_bins_per_chan; j++){
				m_hist[i][j] = new double[m_num_bins_per_chan];
				m_hist_tmp[i][j] = new unsigned int[m_num_bins_per_chan];
				for(int k = 0; k < m_num_bins_per_chan; k++){
					m_hist[i][j][k] = 0.0;
					m_hist_tmp[i][j][k] = 0;
				}
			}
		}

		m_wait_cnt = 0;
	}

	void free()
	{
		if(m_hist == NULL)
			return;

		for(int i = 0; i < m_num_bins_per_chan; i++){
			for(int j = 0; j < m_num_bins_per_chan; j++){
				delete[] m_hist[i][j];
				delete[] m_hist_tmp[i][j];
			}
			delete[] m_hist[i];
			delete[] m_hist_tmp[i];
		}

		m_hist = NULL;
		m_hist_tmp = NULL;
	}

	void expand(int x, int y, int d, int ilabel);

	virtual bool check()
	{
		return m_chin[0] != NULL && m_chout[0] != NULL;
	}

	virtual bool cmd_proc(s_cmd & cmd);

	virtual bool proc();
};

#endif
