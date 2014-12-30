#ifndef _CH_CAMPAR_H_
#define _CH_CAMPAR_H_
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// ch_campar.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// ch_campar.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with ch_campar.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "../util/coord.h"

class ch_campar:public ch_base
{
protected:
	Size m_Pix;
	bool m_bPix;
	Mat m_Intrinsic;
	bool m_bInt;
	Point3d m_pos;
	bool m_bpos;
	s_rotpar m_rot;
	bool m_brot;
public:
	ch_campar(const char * name): ch_base(name), m_bPix(false),
		m_bInt(false), m_bpos(false), m_brot(false)
	{
		m_Intrinsic = Mat::eye(3, 3, CV_64FC1);
	}

	void set_Pix(int h, int v){
		lock();
		m_Pix.width = h;
		m_Pix.height = v;
		m_bPix = true;
		unlock();
	}

	void set_Int(Mat & Int)
	{
		lock();
		Int.copyTo(m_Intrinsic);
		m_bInt = true;
		unlock();
	}

	void set_pos(Point3d & pos){
		lock();
		m_pos = pos;
		m_bpos = true;
		unlock();
	}

	void set_rot(s_rotpar & rot){
		lock();
		m_rot = rot;
		m_brot = true;
		unlock();
	}

	bool get_Size(int & h, int & v){
		lock();
		h = m_Pix.width;
		v = m_Pix.height;
		unlock();
		if(m_bPix){
			m_bPix = false;
			return true;
		}

		return false;
	}

	bool get_Int(Mat & Int)
	{
		lock();
		m_Intrinsic.copyTo(Int);
		unlock();
		if(m_bInt){
			m_bInt = false;
			return true;
		}

		return false;
	}

	bool get_pos(Point3d & pos){
		lock();
		pos = m_pos;
		unlock();
		if(m_bpos){
			m_bpos = false;
			return true;
		}
		return false;
	}

	bool get_rot(s_rotpar & rot){
		lock();
		rot = m_rot;
		unlock();
		if(m_brot){
			m_brot = false;
			return true;
		}
		return false;
	}

	virtual void tran()
	{
	}
};

#endif