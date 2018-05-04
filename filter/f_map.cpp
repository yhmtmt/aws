// Copyright(c) 2017 Yohei Matsumoto, All right reserved. 

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
#include <list>
#include <map>

using namespace std;
#include <cmath>
#include <cstring>

#include "../util/aws_stdlib.h"
#include "../util/aws_thread.h"
#include "../util/c_clock.h"

#include <opencv2/opencv.hpp>
using namespace cv;

#include "f_map.h"

const char * f_map::m_str_dtype[edt_undef] =
{
	"jpjis"
};

const char * f_map::m_str_cmd[emc_undef] =
{
	"update", "add_data", "set_pos", "render", "save", "check"
};

f_map::f_map(const char * name) :f_base(name), m_ch_map(NULL), m_dtype(edt_jpjis), 
m_max_total_size_layer_data(0x00FFFFFF)/*16MB*/, m_max_num_nodes(64),
lat(35.0), lon(140.0), res(1.0), range(5000), m_verb(false)
{
	m_path[0] = '.';
	m_path[1] = '\0';
	m_fdata[0] = '.';
	m_fdata[1] = '\0';
	register_fpar("ch_map", (ch_base**)&m_ch_map, typeid(ch_map).name(), "Map channel.");
	register_fpar("cmd", (int*)&m_cmd, emc_undef, m_str_cmd, "Command");
	register_fpar("dtype", (int*)&m_dtype, edt_undef, m_str_dtype, "Data type of fdata");
	register_fpar("path", m_path, 1024, "Path to map data.");
	register_fpar("fdata", m_fdata, 1024, "File path to data file.");
	register_fpar("max_num_nodes", &m_max_num_nodes, "Maximum number of nodes.");
	m_max_size_layer_data[AWSMap2::lt_coast_line] = (0x00FFFFF) /*64KB*/;

	register_fpar("max_total_size_layer_data", &m_max_total_size_layer_data, "Maximum total size of layer data.");
	register_fpar("max_size_coast_line", &m_max_size_layer_data[AWSMap2::lt_coast_line], "Max data size of coast line layer data per node.");

	register_fpar("lat", &lat, "lattitude configured by set_pos");
	register_fpar("lon", &lon, "longitude configured by set_pos");
	register_fpar("res", &res, "Resolution configured by set_pos");
	register_fpar("range", &range, "Range configured by set_pos");

	register_fpar("verb", &m_verb, "Verbose for debug");
}

f_map::~f_map()
{
}

bool f_map::init_run()
{
  if (!m_ch_map)
    return false;
  
  m_db.setPath(m_path);
  m_db.setMaxNumNodes(m_max_num_nodes);
  m_db.setMaxTotalSizeLayerData(m_max_total_size_layer_data);
  m_db.setMaxSizeLayerData(AWSMap2::lt_coast_line, m_max_size_layer_data[AWSMap2::lt_coast_line]);
  if(!m_db.init())
    return false;
  return true;
}

void f_map::destroy_run()
{
}

bool f_map::proc()
{
  bool success = false;

  m_ch_map->lock();
  if(m_ch_map->is_update())
    m_cmd = emc_update;
  m_ch_map->unlock();
  
  switch (m_cmd)
  {
  case emc_update:
    if (update_channel())
    {
      success = true;
    }
    break;
  case emc_add_data:
    if (add_data()){
      success = true;
      m_fdata[0] = '\0';
    }
    break;
  case emc_set_pos:
	  set_pos();
	  break;
  case emc_render:
	  render_data();
	  break;
  case emc_save:
	  m_db.save();
	  break;
  case emc_check:
	  check_data();
  }
  m_cmd = emc_undef;

  return true;
}

void f_map::check_data()
{
	AWSMap2::CoastLine cl;
	cl.setCenter(m_ch_map->get_center());
	cl.setRadius(m_ch_map->get_range());

	const list<AWSMap2::LayerDataPtr> cls = m_ch_map->get_layer_data(AWSMap2::lt_coast_line);
	for (auto itr = cls.begin(); itr != cls.end(); itr++) {
		const AWSMap2::CoastLine & cld = dynamic_cast<const AWSMap2::CoastLine&>(**itr);

		const_cast<AWSMap2::CoastLine*>(&cld)->merge(cl);
	}
}

bool f_map::update_channel()
{
  // lock channel
  // clear layer data in channel
  // get range from channel
  // get active layer types from channel
  // get layer data from db
  // set layer data to the channel
  // unlock channel
  m_ch_map->lock();
  m_ch_map->clear_layer_datum();

  m_db.restruct();

  list < list<AWSMap2::LayerDataPtr>> layerDatum;
  list<AWSMap2::LayerType> layerTypes;
  
  for (int layer_type = 0; layer_type < (int)AWSMap2::lt_undef; layer_type++){
    if (m_ch_map->is_layer_enabled((AWSMap2::LayerType)layer_type)){
      layerTypes.push_back((AWSMap2::LayerType)layer_type);
      layerDatum.push_back(list<AWSMap2::LayerDataPtr>());
    }
  }

  m_db.request(layerDatum, layerTypes, 
	       m_ch_map->get_center(), 
	       m_ch_map->get_range(), 
	       m_ch_map->get_resolution());
  if (m_verb) {
	  cout << "MapManager Information" << endl;
	  cout << "\tNumber of Nodes Alive: " << AWSMap2::Node::getNumNodesAlive() << endl;
	  cout << "\tMaximum Node Level: " << AWSMap2::Node::getMaxLevel() << endl;
	  cout << "\tLayer Data Size: " << AWSMap2::LayerData::getTotalSize() << endl;

  }
  auto itrData = layerDatum.begin(); 
  for (auto itrType = layerTypes.begin(); itrType != layerTypes.end();
       itrType++, itrData++){
	  if (m_verb) {
		  for (auto itr = itrData->begin(); itr != itrData->end(); itr++)
			  (*itr)->print();
	  }
    m_ch_map->set_layer_data(*itrType, *itrData);
  }
  m_ch_map->reset_update();
  m_ch_map->set_ready();
  m_ch_map->unlock();
  return true;
}

bool f_map::add_data()
{
  AWSMap2::LayerData * pld;
  switch (m_dtype)
    {
    case edt_jpjis:
      pld = load_jpjis();
      break;
    }
  
  if (pld != NULL){
    m_db.insert(pld);
  }
  
  return true;
}

AWSMap2::LayerData * f_map::load_jpjis()
{
	AWSMap2::CoastLine * pcl = new AWSMap2::CoastLine;
  char fpath[2048];
  snprintf(fpath, 2048, "%s/%s", m_path, m_fdata);
	if (!pcl->loadJPJIS(fpath)){
		delete pcl;
		return NULL;
	}

	return pcl;
}

void f_map::render_data()
{
	m_ch_map->lock();
	AWSMap2::vec3 cecef = m_ch_map->get_center();
	AWSMap2::vec2 cbih;
	double calt = 0;
	eceftobih(cecef.x, cecef.y, cecef.z, cbih.lat, cbih.lon, calt);
	Mat Rwrld;
	getwrldrot(cbih.lat, cbih.lon, Rwrld);

	float mres = m_ch_map->get_resolution();
	float mrng = m_ch_map->get_range();

	Mat img = Mat::zeros(1024, 1024, CV_8UC3);
	double scl = (double)(img.cols / 2) / (double) mrng;

	const list<AWSMap2::LayerDataPtr> cls = m_ch_map->get_layer_data(AWSMap2::lt_coast_line);
	for (auto itr = cls.begin(); itr != cls.end(); itr++) {
		
		const AWSMap2::CoastLine & cl = dynamic_cast<const AWSMap2::CoastLine&>(**itr);
		{
			cout << "Rendering ";
			cl.print();
			const AWSMap2::Node * pn = cl.getNode();
			vector<Point2i> ptn(4);
			for (int i = 0; i < 3; i++) {
				const AWSMap2::vec3 & pte = pn->getVtxECEF(i);
				double wx, wy, wz;
				eceftowrld(Rwrld, cecef.x, cecef.y, cecef.z, pte.x, pte.y, pte.z, wx, wy, wz);
				ptn[i].x = (int)(scl * wx) + 512;
				ptn[i].y = -(int)(scl * wy) + 512;
			}
			ptn[3] = ptn[0];
			for (int i = 0; i < 3; i++) {
				Point pt[2];
				int vx, vy;
				int j = 0;
				if (ptn[i].x >= 0 && ptn[i].x < 1024 && ptn[i].y >= 0 && ptn[i].x < 1024){
					pt[j].x = ptn[i].x;
					pt[j].y = ptn[i].y;
					j++;
				}
				else {
					vx = ptn[i].x - ptn[i + 1].x;
					vy = ptn[i].y - ptn[i + 1].y;
				}

				if(ptn[i + 1].x >= 0 && ptn[i + 1].x < 1024 && ptn[i + 1].y >= 0 && ptn[i + 1].y < 1024) {
					pt[j].x = ptn[i + 1].x;
					pt[j].y = ptn[i + 1].y;
					j++;
				}else{
					vx = ptn[i + 1].x - ptn[i].x;
					vy = ptn[i + 1].y - ptn[i].y;
				}


				double a = (double)(ptn[i].y - ptn[i + 1].y) / (double)(ptn[i].x - ptn[i + 1].x);
				double b = (double)ptn[i].y - a * (double)ptn[i].x;
				double y0 = b, y1 = a * 1023 + b;
				double x0 = -b / a, x1 = (1023 - b) / a;
				if (j == 1) {
					if (vx == 0) {
						pt[j].x = pt[0].x;
						if (vy > 0)
							pt[j].y = 1023;
						else
							pt[j].y = 0;
						j++;
					}
					else if (vy == 0) {
						pt[j].y = pt[0].y;
						if (vx > 0)
							pt[j].x = 1023;
						else
							pt[j].x = 0;
						j++;
					}
					else {
						if (vx > 0) {//pt[j].x > pt[0].x  -> 1023 or x0 or x1
							if (y1 >= 0 && y1 < 1024) {
								pt[j].x = 1023;
								pt[j].y = (int)y1;
								j++;
							}
							else if (x0 > pt[0].x && x0 >= 0 && x0 < 1024) {
								pt[j].x = (int)x0;
								pt[j].y = 0;
								j++;
							}
							else if (x1 > pt[0].x && x1 >= 0 && x1 < 1024) {
								pt[j].x = (int)x1;
								pt[j].y = 1023;
								j++;
							}
						}
						else {// pt[j].x < pt[0].x
							if (y0 >= 0 && y0 < 1024) {
								pt[j].x = 0;
								pt[j].y = (int)y0;
								j++;
							}
							else if (x0 < pt[0].x && x0 >= 0 && x0 < 1024) {
								pt[j].x = (int)x0;
								pt[j].y = 0;
								j++;
							}
							else if (x1 < pt[0].x && x1 >= 0 && x1 < 1024) {
								pt[j].x = (int)x1;
								pt[j].y = 1023;
								j++;
							}
						}
					}
				}
				else if (j == 0) {
					if (0 <= y0 && y0 < 1024 && j < 2) {
						pt[j].x = 0;
						pt[j].y = y0;
						j++;
					}

					if (0 <= y1 && y1 < 1024 && j < 2) {
						pt[j].x = 1023;
						pt[j].y = y1;
						j++;
					}

					if (0 <= x0 && x0 < 1024 && j < 2) {
						pt[j].x = x1;
						pt[j].y = 0;
						j++;
					}

					if (0 <= x1 && x1 < 1024 && j < 2) {
						pt[j].x = x1;
						pt[j].y = 1023;
						j++;
					}
				}
				if(j == 2){
					line(img, pt[0], pt[1], Scalar(255, 0, 0), 3);
				}
			}
		}
	
		for(int id = 0; id < cl.getNumLines(); id++){
			const vector<AWSMap2::vec2> & pts_bih = cl.getPointsBIH(id);
			const vector<AWSMap2::vec3> & pts = cl.getPointsECEF(id);
			vector<Point2i> pts_wrld(pts.size());

			auto iwpt = pts_wrld.begin();
			bool binside = false;
			for (auto ipt = pts.begin(); ipt != pts.end(); ipt++, iwpt++){
				double wx, wy, wz;
				eceftowrld(Rwrld, cecef.x, cecef.y, cecef.z, ipt->x, ipt->y, ipt->z, wx, wy, wz);
				iwpt->x = (int)(scl * wx) + 512;
				iwpt->y = -(int)(scl * wy) + 512;
				if (iwpt->x < 1024 && iwpt->x > 0 && iwpt->y < 1024 && iwpt->y > 0) {
					binside = true;
				}
			}
			/*
			if (binside) {
				cout << "line[" << id << "]:";
				cout.precision(12);
				cout << "bih (" <<pts_bih.front().x <<"," << pts_bih.front().y << ")-(" <<pts_bih.back().x << "," << pts_bih.back().y << ")";
				cout << "ecef(" <<pts.front().x << "," <<pts.front().y <<  ")-(" << pts.back().x << "," << pts.back().y << ")";
				cout << "wrld(" <<pts_wrld.front().x << "," << pts_wrld.front().y <<  ")-(" << pts_wrld.back().x << "," << pts_wrld.back().y << ")";
				cout << endl;
			}
			*/
			unsigned char r, g, b;
			r = rand() % 128 + 128;
			g = rand() % 128 + 128;
			b = rand() % 128 + 128;
			polylines(img, pts_wrld, false, Scalar(r, g, b));
		}
	}

	char frender[1024];
	snprintf(frender, 1024, "%s/%f03.5-%f04.5.png", m_path, cbih.lat, cbih.lon);
	imwrite(frender, img);
	m_ch_map->unlock();
}

void f_map::set_pos()
{
	double x, y, z;
	bihtoecef(lat * PI/180.f, lon * PI/180.f, 0, x, y, z);
	m_ch_map->set_center((float)x, (float)y, (float)z);
	m_ch_map->set_range((float)range);
	m_ch_map->set_resolution((float)res);
}
