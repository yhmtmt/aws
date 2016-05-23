// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_window.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_window.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_window.h.  If not, see <http://www.gnu.org/licenses/>. 


////////////////////////////////Alert////////////////////////////////////
// these highgui windows won't work because of the recent parallelization.
#include "../util/aws_sock.h"
#include "../util/aws_thread.h"

#include "../util/aws_coord.h"
#include "../util/c_ship.h"
//#include "../util/aws_nmea.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"
#include "../channel/ch_ais.h"
#include "../channel/ch_vector.h"
#include "../channel/ch_scalar.h"
#include "../channel/ch_campar.h"
#include "../channel/ch_navdat.h"
#include "../channel/ch_nmea.h"

#include "f_base.h"

class f_window: public f_base
{
protected:
	ch_image * m_pin;
public:
	f_window(const char * name):f_base(name)
	{
		namedWindow(m_name);
	};

	virtual ~f_window()
	{
		destroyWindow(m_name);
	}

	virtual bool check()
	{
		return m_chin[0] != NULL;
	}

	virtual bool proc(){
		long long timg;
		Mat img = m_pin->get_img(timg);
		if(img.empty())
			return true;
		imshow(m_name, img);
		return true;
	}
};

////////////////////////////////Alert////////////////////////////////////
// these highgui windows won't work because of the recent parallelization.

class f_mark_window: public f_window
{
protected:
	vector<Point2i> m_point;
	int m_scale;

	void push(int x, int y)
	{
		m_point.push_back(Point2i(x, y));
	}

	void write(const char * fname){
		ofstream file;
		file.open(fname, ios_base::trunc);

		if(!file.is_open()){
			cerr << "Failed to open file " << fname << endl;
			return;
		}

		for(int i = 0; i < m_point.size(); i++)
			file << m_point[i].x << " " << m_point[i].y << endl;
	}

	void render(){
		long long timg;
		Mat img = m_pin->get_img(timg);
		if(img.empty())
			return;

		Mat img_show;
		resize(img, img_show, Size(img.cols*m_scale, img.rows*m_scale));

		Point p1, p2;

		for(int i = 0; i < m_point.size(); i++){
			p1.x = m_point[i].x * m_scale - 5;
			p2.y = p1.y = m_point[i].y * m_scale;
			p2.x = p1.x + 10;
			line(img_show, p1, p2, CV_RGB(255, 0, 0), 1);

			p2.x = p1.x = m_point[i].x * m_scale;
			p1.y = m_point[i].y * m_scale - 5;
			p2.y = p1.y + 10;
			line(img_show, p1, p2, CV_RGB(255, 0, 0), 1);
		}
		imshow(m_name, img_show);
	}

	static void on_mouse(int event, int x, int y, int flags, void * param=NULL)
	{
		f_mark_window * pwin = (f_mark_window*) param;
		switch(event){
		case CV_EVENT_LBUTTONDOWN:
			pwin->push(x / pwin->m_scale, y / pwin->m_scale);
		}
		pwin->render();
	}

public:
	f_mark_window(const char * name);
	virtual ~f_mark_window();

	virtual bool proc(){
		render();
		return true;
	}

	virtual bool cmd_proc(s_cmd & cmd);
};

