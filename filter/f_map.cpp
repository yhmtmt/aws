// Copyright(c) 2016 Yohei Matsumoto, All right reserved. 

// f_map.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_map.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_map.h.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

using namespace std;
#include <cmath>
#include <cstring>

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_map.h"

bool f_map::init_run()
{
	if(m_list[0] == '\0'){
		cerr << "List of the map should be specified." << endl;
		return false;
	}

	ifstream flist;
	ifstream fmap;
	flist.open(m_list);

	list<Point2f> line;
	char buf[1024];
	while(!flist.eof()){
		flist.getline(buf, 1024);
		fmap.open(buf);
		if(!fmap.is_open()){
			continue;
		}

		bool bline = false;
		while(!fmap.eof()){
			fmap.getline(buf, 1024);
			if(!bline){
				if(strcmp(buf, "<gml:posList>") == 0){
					bline = true;
				}
			}else{
				if(strcmp(buf, "</gml:posList>") == 0){
					vector<Point3f> * pline = new vector<Point3f>;
					pline->resize(line.size());
					Point2f pmax(-FLT_MAX, -FLT_MAX), pmin(FLT_MAX, FLT_MAX);
					if(pline == NULL){
						goto failed;
					}
					int i = 0;
					for(list<Point2f>::iterator itr = line.begin(); itr != line.end(); itr++, i++){
						bihtoecef((*itr).x, (*itr).y, 0.f, (*pline)[i].x, (*pline)[i].y, (*pline)[i].z);
						pmax.x = max(pmax.x, (*itr).x);
						pmax.y = max(pmax.y, (*itr).y);
						pmin.x = min(pmin.x, (*itr).x);
						pmin.y = min(pmin.y, (*itr).y);
					}
					m_cls.push_back(pline);
					line.clear();
					s_cl_bb bb;
					bihtoecef(pmax.x, pmax.y, 0.f, bb.bb[0].x, bb.bb[0].y, bb.bb[0].z);
					bihtoecef(pmax.x, pmin.y, 0.f, bb.bb[1].x, bb.bb[1].y, bb.bb[1].z);
					bihtoecef(pmin.x, pmin.y, 0.f, bb.bb[2].x, bb.bb[2].y, bb.bb[2].z);
					bihtoecef(pmin.x, pmax.y, 0.f, bb.bb[3].x, bb.bb[3].y, bb.bb[3].z);
					bb.in_range = false;
					m_cl_bbs.push_back(bb);
				}else{
					Point2f pt;
					char * p;
					for(p = buf; *p != ' ' && *p != '\0'; p++);
					*p = '\0';
					p++;
					pt.x = (float) atof(buf);
					pt.y = (float) atof(p);
					line.push_back(pt);
				}				
			}
		}

		fmap.close();
	}

	return true;
failed:
	for(list<vector<Point3f>*>::iterator itr = m_cls.begin(); itr != m_cls.end(); itr++){
		delete[] *itr;
	}
	m_cls.clear();
	m_cl_bbs.clear();
	return false;
}

void f_map::destroy_run()
{
	for(list<vector<Point3f>*>::iterator itr = m_cls.begin(); itr != m_cls.end(); itr++){
		delete[] *itr;
	}
	m_cls.clear();
	m_cl_bbs.clear();
}

bool f_map::proc()
{
	float range = m_ch_map->get_range();
	Point3f cecef; 
	m_ch_map->get_center(cecef.x, cecef.y, cecef.z);

	{
		list<vector<Point3f>*>::iterator itr = m_cls.begin();
		list<s_cl_bb>::iterator itr_bb = m_cl_bbs.begin();

		float r2 = (float)(range * range);
		for(; itr_bb != m_cl_bbs.end(); itr++, itr_bb++){
			// find nearest point of the bb
			float xmin = FLT_MAX, ymin = FLT_MAX, zmin = FLT_MAX;

			for(int i = 0; i < 4; i++){
				Point3f & pt = (*itr_bb).bb[i];
				xmin = min(xmin, (float)abs((float)(pt.x - cecef.x)));
				ymin = min(ymin, (float)abs((float)(pt.y - cecef.y)));
				zmin = min(zmin, (float)abs((float)(pt.z - cecef.z)));
			}
			float d2 = (float)(xmin * xmin + ymin * ymin + zmin * zmin);
			if(d2 > r2){
				if((*itr_bb).in_range){
					m_ch_map->erase(*itr);
				}
				((*itr_bb).in_range) = false;
			}else{
				if(!(*itr_bb).in_range){
					m_ch_map->insert(*itr);
				}
				((*itr_bb).in_range) = true;
			}
		}
	}
	return true;
}
