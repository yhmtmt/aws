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
	"update", "add_data", "set_pos", "render"
};

f_map::f_map(const char * name) :f_base(name), m_ch_map(NULL), m_dtype(edt_jpjis), lat(35.0), lon(140.0), res(1.0), range(5000)
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
	register_fpar("max_total_size_layer_data", &m_max_total_size_layer_data, "Maximum total size of layer data.");
	register_fpar("max_size_coast_line", &m_max_size_layer_data[AWSMap2::lt_coast_line], "Max data size of coast line layer data per node.");

	register_fpar("lat", &lat, "lattitude configured by set_pos");
	register_fpar("lon", &lon, "longitude configured by set_pos");
	register_fpar("res", &res, "Resolution configured by set_pos");
	register_fpar("range", &range, "Range configured by set_pos");
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
  }
  m_cmd = emc_undef;

  return true;
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

  list < list<const AWSMap2::LayerData *>> layerDatum;
  list<AWSMap2::LayerType> layerTypes;
  
  for (int layer_type = 0; layer_type < (int)AWSMap2::lt_undef; layer_type++){
    if (m_ch_map->is_layer_enabled((AWSMap2::LayerType)layer_type)){
      layerTypes.push_back((AWSMap2::LayerType)layer_type);
      layerDatum.push_back(list<const AWSMap2::LayerData*>());
    }
  }

  m_db.request(layerDatum, layerTypes, 
	       m_ch_map->get_center(), 
	       m_ch_map->get_range(), 
	       m_ch_map->get_resolution());
  auto itrData = layerDatum.begin(); 
  for (auto itrType = layerTypes.begin(); itrType != layerTypes.end();
       itrType++, itrData++){
    m_ch_map->set_layer_data(*itrType, *itrData);
  }
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

	const list<const AWSMap2::LayerData*> cl = m_ch_map->get_layer_data(AWSMap2::lt_coast_line);
	for (auto itr = cl.begin(); itr != cl.end(); itr++) {
		const AWSMap2::CoastLine * pcl = dynamic_cast<const AWSMap2::CoastLine*>(*itr);
		for(int id = 0; id < pcl->getNumLines(); id++){
			const vector<AWSMap2::vec2> & pts_bih = pcl->getPointsBIH(id);
			const vector<AWSMap2::vec3> & pts = pcl->getPointsECEF(id);
			vector<Point2f> pts_wrld(pts.size());
		
			auto iwpt = pts_wrld.begin();
			for (auto ipt = pts.begin(); ipt != pts.end(); ipt++, iwpt++){
				double wx, wy, wz;
				eceftowrld(Rwrld, cecef.x, cecef.y, cecef.z, ipt->x, ipt->y, ipt->z, wx, wy, wz);
				iwpt->x = (float)scl * wx;
				iwpt->y = (float)scl * wy;
			}

			polylines(img, pts_wrld, false, Scalar(255, 255, 255));
		}
	}

	char frender[1024];
	snprintf(frender, 1024, "%s/%f03.5-%f04.5.png", cbih.lat, cbih.lon);
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