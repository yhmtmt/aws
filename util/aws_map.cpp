#include "stdafx.h"
// Copyright(c) 2017 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_map.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_map.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_map.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <fstream>

using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "aws_coord.h"
#include "aws_stdlib.h"
#include "aws_map.h"


// line and sphere collision
// V0, V1 : terminal points of the line
// S: the central point of the sphere
// R: Radius of the sphere
// |V0-S| < R or |V1 - S| < R
// other wise
// s = n(V1-V0)* (S - V0)
// L = t(V1-V0) + V0 
// 0 < t < 1 and |S - L| < R
namespace AWSMap2 {

	inline bool det_collision_line_and_sphere_mid(const vec3 & v0, const vec3 & v1, const vec3 & s, const double r2)
	{
		double n = (l2Norm(v1, v0));
		double invn = (1.0 / n);

		vec3 d = (v1 - v0) * invn;

		double t =  dot(d, s - v0);

		if (t < 0.f || t > n){
			return false;
		}

		d *= t;
		d += v0;
		if (l2Norm2(d, s) < r2)
			return true;

		return false;
	}

	inline bool det_collision_line_and_sphere(const vec3 & v0, const vec3 & v1, const vec3 & s, const float r2)
	{
		if (l2Norm2(v0, s) < r2)
			return true;

		if (l2Norm2(v1, s) < r2)
			return true;

		return det_collision_line_and_sphere_mid(v0, v1, s, r2);
	}

	inline bool det_collision_tri_and_sphere(const vec3 & v0, const vec3 & v1, const vec3 & v2, const vec3 & s, const float r2)
	{
		// check around three points
		double d0 = l2Norm2(v0, s);
		if (d0 < r2)
			return true;

		double d1 = l2Norm2(v1, s);
		if (d1 < r2)
			return true;

		double d2 = l2Norm2(v1, s);
		if (d2 < r2)
			return true;

		if (d0 > BE * BE && d1 > BE * BE && d2 > BE * BE)
			return false;

		// inside flag is asserted when s projects on both edge 01 and edge 02
		bool binside = false;

		// check around edge 01
		double n01 = l2Norm(v0, v1);
		double invn01 = 1.0 / n01;
		vec3 d01 = (v1 - v0) * invn01;
		double t01 = dot(d01, s - v0);

		if (t01 > 0.f && t01 < n01) {
			d01 *= t01;
			d01 += v0;
			if (l2Norm2(d01, s) < r2) // s is near around edge 01
				return true;

			binside = true;
		}

		// check around edge 02
		double n02 = l2Norm(v2, v0);
		double invn02 = 1.0 / n02;
		vec3 d02 = (v2 - v0) * invn02;
		double t02 = dot(d02, s - v0);
		d02 *= t02;
		d02 += v0;
		if (t02 > 0.f && t02 < n02) {
			if (binside) // s is projected on both edge 01 and 02
				return true;

			d02 *= t02;
			d02 += v0;
			if (l2Norm2(d02, s) < r2) // s is near around edge 02
				return true;
		}

		// check outside near edge 12
		double n12 = l2Norm(v1, v2);
		double invn12 = 1.0 / n12;
		vec3 d12 = (v2 - v1) * invn12;
		double t12 = dot(d12, s - v1);
		d12 *= t12;
		d12 += v1;
		if (t12 > 0.f && t12 < n12) {
			d12 *= t12;
			d12 += v1;
			if (l2Norm2(d12, s) < r2) // s is near around edge 12 (actually only the outside case is reached here.)
				return true;
		}

		return false;
	}

	inline double det(const vec3 & a0, const vec3 & a1, const vec3 & a2)
	{
		// a0.x a1.x a2.x
		// a0.y a1.y a2.y
		// a0.z a1.z a2.z

		double d = a0.x * a1.y * a2.z + a0.z * a1.x * a2.y + a0.y * a1.z * a2.x
			- a0.z * a1.y * a2.x - a0.x * a1.z * a2.y - a0.y * a1.x * a2.z;
		return d;
	}
	
	inline bool det_collision(const vec3 & t2, const vec3 & t1, const vec3 & t0,
		const vec3 & l1, const vec3 & l0 = vec3(0, 0, 0), const double err = 0)
	{
		vec3 e1 = t1 - t0;
		vec3 e2 = t2 - t0;
		vec3 me3 = l0 - l1; // -e3
		vec3 e4 = l0 - t0;
		// u e1 + v e2 + t0 = w e3 + l0
		//   -> u e1 + v e2 - w e3 = l0 - t0
		//   -> u e1 + v e2 + w me3 = e4

		double invD = 1.0 / det(e1, e2, me3);
		// invD = 1.0 / |e1 e2 me3|

		double u = det(e4, e2, me3) * invD;
		// u = |e4 e2 me3| * invD, v = |e1 e4 me3| * invD, w = |e1 e2 e4| * invD 

		// 0 < u < 1
		if (u < -err || u > (1. + err))
			return false;

		double v = det(e1, e4, me3) * invD;
		// 0 < v < 1
		if (v < -err || v > (1. + err))
			return false;

		if (u + v > (1. + err))
			return false;

		double t = det(e1, e2, e4) * invD;
		// t > 0
		if (t < -err)
			return false;

		return true;
	}


  const char * strLayerType[lt_undef] =
    {
      "coast_line"
    };

	LayerType getLayerType(const char * str) {
		for (int lt = 0; lt < (int)lt_undef; lt++) {
			if (strcmp(strLayerType[lt], str) == 0)
				return (LayerType)lt;
		}

		return lt_undef;
	}

	////////////////////////////////////////////////////////////// MapDataBase
	unsigned int MapDataBase::maxSizeLayerData[lt_undef] =
	{
		0x000FFFFF
	};
	unsigned int MapDataBase::maxNumNodes = 64;
	unsigned int MapDataBase::maxTotalSizeLayerData = 0x4FFFFF0;

	char * MapDataBase::path = NULL;

	void MapDataBase::setPath(const char * _path)
	{
		if (path) {
			delete[] path;
			path = NULL;
		}

		path = new char[strnlen(_path, MAX_PATH_LEN - 1) + 1];
		strcpy(path, _path);
	}

	const char * MapDataBase::getPath()
	{
		return path;
	}



  MapDataBase::MapDataBase()
  {
  }
  
  MapDataBase::~MapDataBase()
  {
    for (int i = 0; i < 20; i++)
      delete pNodes[i];
  }

  bool MapDataBase::init()
  {
    bool bloaded = true;
    for (unsigned int id = 0; id < 20; id++){
      pNodes[id] = Node::load(NULL, id);
      if (!pNodes[id])
	bloaded = false;
    }
    
    if (bloaded)
      return true;
    
    for (unsigned int id = 0; id < 20; id++){
      if (pNodes[id])
	delete pNodes[id];
    }
    
    vec2 * q = new vec2[12];
    vec3 * v = new vec3[12];
    unsigned int ** f = new unsigned int*[20];
    
    float lat0 = (float)atan2(GR, 1), lat1 = (float)atan2(1, GR);
  
    q[0] = vec2(lat1, 0.5 * PI);  // x=0 y=GR (lon=90) z=1 (lat=32)
    q[1] = vec2(lat1, -0.5 * PI); // x=0 y=-GR (lon=-90) z=1 (lat=32)
    q[2] = vec2(-lat1, 0.5 * PI); // x=0 y=GR (lon=90) z=-1 (lat=-32)
    q[3] = vec2(-lat1, -0.5 * PI);// x=0 y=-GR (lon=-90) z=-1 (lat=-32)
    
    q[4] = vec2(0, atan2(1, GR)); // x=GR, y=1 (lon=32) z=0 (lat=0)
    q[5] = vec2(0, atan2(-1, GR)); // x=GR, y=-1 (lon=-32) z=0 (lat=0)
    q[6] = vec2(0, atan2(1, -GR)); // x=-GR, y=1 (lon=148) z=0 (lat=0)
    q[7] = vec2(0, atan2(-1, -GR)); // x=-GR y=-1 (lon=-148) z=0 (lat=0)
    
    q[8] = vec2(lat0, 0); // x=1 y=0 (lon=0) z=GR (lat=58)
    q[9] = vec2(-lat0, 0); // x=1 y=0 (lon=0) z=-GR (lat=-58)
    q[10] = vec2(lat0, PI); // x=-1 y=0 (lon=180) z=GR (lat=58)
    q[11] = vec2(-lat0, PI); // x=-1 y=0 (lon=180) z=-GR (lat=-58)

	// north half
	pNodes[0] = new Node(0, NULL, q[8], q[5], q[4]);
	pNodes[1] = new Node(1, NULL, q[8], q[4], q[0]);
	pNodes[2] = new Node(2, NULL, q[8], q[0], q[10]); // Japan?
	pNodes[3] = new Node(3, NULL, q[8], q[10], q[1]);
	pNodes[4] = new Node(4, NULL, q[8], q[1], q[5]);
	pNodes[5] = new Node(5, NULL, q[10], q[0], q[6]); // Japan?
	pNodes[6] = new Node(6, NULL, q[10], q[6], q[7]);
	pNodes[7] = new Node(7, NULL, q[10], q[7], q[1]);

	// midle
	pNodes[8] = new Node(8, NULL, q[0], q[4], q[2]); // shallow east
	pNodes[9] = new Node(9, NULL, q[0], q[2], q[6]); // deep east
	pNodes[10] = new Node(10, NULL, q[1], q[7], q[3]); // deep west
	pNodes[11] = new Node(11, NULL, q[1], q[3], q[5]); // shallow west

	// south half
	pNodes[12] = new Node(12, NULL, q[9], q[4], q[3]);
	pNodes[13] = new Node(13, NULL, q[9], q[5], q[3]);
	pNodes[14] = new Node(14, NULL, q[9], q[3], q[11]);
	pNodes[15] = new Node(15, NULL, q[9], q[11], q[2]);
	pNodes[16] = new Node(16, NULL, q[9], q[2], q[4]);
	pNodes[17] = new Node(17, NULL, q[11], q[3], q[7]);
	pNodes[18] = new Node(18, NULL, q[11], q[7], q[6]);
	pNodes[19] = new Node(19, NULL, q[11], q[6], q[2]);
    for (unsigned int id = 0; id < 20; id++){
      pNodes[id]->setId((unsigned char)id);
    }

    return true;
  }
  
  void MapDataBase::request(list<list<const LayerData*>> & layerDatum, const list<LayerType> & layerTypes,
			    const vec3 & center, const float radius,  const float resolution)
  {
    for (int iface = 0; iface < 20; iface++)
      {
	pNodes[iface]->getLayerData(layerDatum, layerTypes,
				    center, radius, resolution);
      }
    
    for (auto itrType = layerDatum.begin();
	 itrType != layerDatum.end(); itrType++){
      for (auto itrData = itrType->begin();
	   itrData != itrType->end(); itrData++)
	LayerData::accessed(const_cast<LayerData*>(*itrData));
    }			
  }
  
  bool MapDataBase::insert(const LayerData * layerData)
  {
    list<Node*> nodes;
    for (int iface = 0; iface < 20; iface++){
      if (!pNodes[iface]->collision(layerData->center(), layerData->radius()))
	continue;
      nodes.push_back(pNodes[iface]);
    }
    
    layerData->split(nodes);
    return true;
  }

	bool MapDataBase::erase(const LayerData * layerData)
	{
		if (!layerData){
			return false;
		}

		Node * pNode = layerData->getNode();

		pNode->deleteLayerData(layerData);

		return true;
	}

	void MapDataBase::restruct()
	{
		LayerData::restruct();
		Node::restruct();
	}

	bool MapDataBase::save()
	{
	  bool result = true;
	  for (int iface = 0; iface < 20; iface++){
	    result &= pNodes[iface]->save();
	  }

	  return result;
	}

	///////////////////////////////////////////////////////////////////// Node
	Node * Node::head = NULL;
	Node * Node::tail = NULL;
	unsigned int Node::numNodesAlive = 0;

	void Node::insert(Node * pNode)
	{
		if (!pNode->upLink) // top nodes should not be managed in the list.
			return;

		if (numNodesAlive >= MapDataBase::getMaxNumNodes())
			restruct();

		numNodesAlive++;
		if (head == NULL && tail == NULL){
			pNode->next = pNode->prev = NULL;
			head = tail = pNode;
			return;
		}
		pNode->next = NULL;
		pNode->prev = tail;
		tail->next = pNode;
		tail = tail->next;
	}

	void Node::pop(Node * pNode)
	{
		Node * prev = pNode->prev, *next = pNode->next;

		if (next == NULL){
			tail = prev;
		}
		else {
			next->prev = prev;
		}

		if (prev == NULL){
			head = next;
		}
		else{
			prev->next = next;
		}
		
		numNodesAlive--;
	}

	void Node::accessed(Node * pNode)
	{
		if (!pNode->upLink) // top nodes are not in the list.
			return;

		pop(pNode);
		insert(pNode);
	}

  void Node::restruct()
  {
    while (numNodesAlive >= MapDataBase::getMaxNumNodes())
      {
	Node * itr = head;
	for (; itr != NULL; itr = itr->next){
	  if (!itr->bdownLink ||
	      (itr->downLink[0] == NULL &&
	       itr->downLink[1] == NULL &&
	       itr->downLink[2] == NULL &&
	       itr->downLink[3] == NULL)){
	    break;
	  }
	}
	
	if (itr != NULL){
#ifdef _AWS_MAP_DEBUG
		char path[1024];
		itr->getPath(path, 1024);
		cout << "Release node " << path << endl;
#endif
	  itr->releaseLayerData();
	  pop(itr);
	  delete itr;
	}
      }
  }
  
  bool Node::deleteLayerData(const LayerData * layerData)
  {
    bupdate = true;
	Node::accessed(this);

    auto itr = layerDataList.find(layerData->getLayerType());
    if (itr == layerDataList.end())
      return false;
    itr->second->release();
    delete itr->second;
    
    layerDataList.erase(itr);
    return true;
  }
  
  void Node::releaseLayerData()
	{
		for (auto itr = layerDataList.begin(); itr != layerDataList.end(); itr++){
			itr->second->release();
			delete itr->second;
		}
		layerDataList.clear();
	}

Node::Node() : prev(NULL), next(NULL), level(0), upLink(NULL), bdownLink(false), bupdate(true)
{
	downLink[0] = downLink[1] = downLink[2] = downLink[3] = NULL;
}

Node::Node(const unsigned char _id, Node * _upLink, const vec2 vtx_bih0, const vec2 vtx_bih1, const vec2 vtx_bih2) : prev(NULL), next(NULL), id(_id), upLink(_upLink),bupdate(true), bdownLink(false)
{
		downLink[0] = downLink[1] = downLink[2] = downLink[3] = NULL;
		vtx_bih[0] = vtx_bih0;
		vtx_bih[1] = vtx_bih1;
		vtx_bih[2] = vtx_bih2;

		calc_ecef();
		
		if (upLink) {
			level = _upLink->level + 1;
		}
		else
			level = 0;

		char path[2048];
		getPath(path, 2048);
		aws_mkdir(path);
}

Node::~Node()
{
	save();

	for (auto itr = layerDataList.begin(); itr != layerDataList.end(); itr++){
		itr->second->release();
		delete itr->second;
	}

	for (int i = 0; i < 4; i++) {
		if (downLink[i])
			delete downLink[i];
		downLink[i] = NULL;
	}
}

void Node::calc_ecef()
{
	for (int i = 0; i < 3; i++) {
		bihtoecef(vtx_bih[i].x, vtx_bih[i].y, 0.0f, vtx_ecef[i].x, vtx_ecef[i].y, vtx_ecef[i].z);
	}
	vec_ecef[0] = vtx_ecef[1] - vtx_ecef[0];
	vec_ecef[1] = vtx_ecef[2] - vtx_ecef[0];
}

bool Node::save()
{
  if (!bupdate){
    return true;
  }
  
  char path[2048];
  char fname[2048];
  getPath(path, 2048);
  snprintf(fname, 2048, "%s/N%02d.index", path, (int)id);
  
  ofstream ofile;
#ifdef _AWS_MAP_DEBUG
  cout << "Saving " << fname << endl;
#endif

  ofile.open(fname, ios::binary);
  if (!ofile.is_open()){
    aws_mkdir(path);
    ofile.clear();
    ofile.open(fname, ios::binary);
    if (!ofile.is_open())
      return false;
  }
  
  bool result = true;
  
  ofile.write((const char*)&bdownLink, sizeof(bool));
  ofile.write((const char*)&vtx_bih, sizeof(vec2) * 3);
  
  unsigned int num_layer_datum = (unsigned int) layerDataList.size();
  ofile.write((const char*)&num_layer_datum, sizeof(unsigned int));
  for (auto itr = layerDataList.begin(); itr != layerDataList.end();
       itr++){
    LayerType layerType = itr->first;
    ofile.write((const char*)&layerType, sizeof(LayerType));
  }
  
  ofile.close();
  
  // save layer data
  for (auto itr = layerDataList.begin(); itr != layerDataList.end();
       itr++){
    result &= itr->second->save();
  }
  
  if (bdownLink){
    for (int idown = 0; idown < 4; idown++){
      if (downLink[idown])
	result &= downLink[idown]->save();
    }
  }
  
  return result;
}
  
Node * Node::load(Node * pNodeUp, unsigned int idChild)
{
  char fname[2048];
  
  if (pNodeUp == NULL){
    if (idChild >= 20)
      return NULL;
    
    // the top 20 nodes
    const char * path = MapDataBase::getPath();
    snprintf(fname, 2048, "%s/N%02d/N%02d.index", path, (int)idChild, (int)idChild);
  }
  else{
    char path[2048];
    pNodeUp->getPath(path, 2048);
    snprintf(fname, 2048, "%s/N%02d/N%02d.index", path, (int)idChild, (int)idChild);
  }
#ifdef _AWS_MAP_DEBUG
  cout << "Loading " << fname << endl;
#endif

  ifstream findex(fname, ios::binary);
  if (!findex.is_open()){
    return NULL;
  }
  
  Node * pNode = new Node();
  pNode->upLink = pNodeUp;
  pNode->id = idChild;
  if (pNodeUp)
	  pNode->level = pNodeUp->level + 1;

  ////////////////////// loading index file to the Node
  findex.read((char*)&(pNode->bdownLink), sizeof(bool));
  findex.read((char*)(pNode->vtx_bih), sizeof(vec2) * 3);
  pNode->calc_ecef();

  unsigned int num_layer_datum = 0;
  findex.read((char*)(&num_layer_datum), sizeof(unsigned int));
  
  while (!findex.eof() && num_layer_datum != 0){
    LayerType layerType;
    findex.read((char*)&layerType, sizeof(LayerType));
    LayerData * layerData = LayerData::create(layerType);
    if (layerData){
      layerData->setNode(pNode);
      pNode->insertLayerData(layerData);
    }
    num_layer_datum--;
  }

  pNode->bupdate = false;

  Node::insert(pNode);
  return pNode;
}
  
bool Node::createDownLink()
{
	vec3 vtx_ecef_mid[3];
	vec2 vtx_bih_mid[3];
	double alt;
	vtx_ecef_mid[0] = (vtx_ecef[0] + vtx_ecef[1]) * 0.5;
	vtx_ecef_mid[1] = (vtx_ecef[1] + vtx_ecef[2]) * 0.5;
	vtx_ecef_mid[2] = (vtx_ecef[2] + vtx_ecef[0]) * 0.5;
	for (int i = 0; i < 3; i++)
		eceftobih(vtx_ecef_mid[i].x, vtx_ecef_mid[i].y, vtx_ecef_mid[i].z, vtx_bih_mid[i].x, vtx_bih_mid[i].y, alt);

	downLink[0] = new Node(0, this, vtx_bih[0], vtx_bih_mid[2], vtx_bih_mid[0]);
	downLink[1] = new Node(1, this, vtx_bih[1], vtx_bih_mid[0], vtx_bih_mid[1]);
	downLink[2] = new Node(2, this, vtx_bih[2], vtx_bih_mid[1], vtx_bih_mid[2]);
	downLink[3] = new Node(3, this, vtx_bih_mid[0], vtx_bih_mid[2], vtx_bih_mid[1]);
	bdownLink = true;

	for(int i = 0; i < 4; i++)
		Node::insert(downLink[i]);

	return true;
}

void Node::getPath(char * path, unsigned int maxlen)
{
	list<unsigned char> path_id;

	getPath(path_id);
	unsigned int len = 0;
	len = snprintf(path + len, maxlen - len, "%s", MapDataBase::getPath());

	for (auto itr = path_id.begin(); itr != path_id.end(); itr++){
		if (len + 3 > maxlen)
			return;
		len += snprintf(path + len, maxlen - len, "/N%02d", (int)*itr);
	}
}

void Node::getPath(list<unsigned char> & path_id)
{
	if (upLink == NULL){
		path_id.push_back(id);
		return;
	}

	upLink->getPath(path_id);
	path_id.push_back(id);
}

bool Node::distributeLayerData(const LayerData & layerData)
{
	if (!bdownLink)
		return true;

	list<Node*> nodes;
	for (int idown = 0; idown < 4; idown++){
		if (!downLink[idown]){
			downLink[idown] = Node::load(this, idown);
		}
		nodes.push_back(downLink[idown]);
	}
	layerData.split(nodes, this);

	return true;
}

const bool Node::collision(const vec3 & location, const double err)
{
	return det_collision(vtx_ecef[0], vtx_ecef[1], vtx_ecef[2], location, vec3(0,0,0), err);
}

const bool Node::collision(const vec3 & center, const float radius)
{
	return det_collision_tri_and_sphere(vtx_ecef[0], vtx_ecef[1], vtx_ecef[2], center, radius * radius);
}

const void Node::collision_downlink(const vector<vec3> & pts, vector<char> & inodes)
{
	if (!bdownLink)
		return;

	inodes.resize(pts.size());

	vec3 vec0(-vtx_ecef[0].x, -vtx_ecef[0].y, -vtx_ecef[0].z);

	for (int ipt = 0; ipt < pts.size(); ipt++) {
		vec3 vpt(-pts[ipt].x, -pts[ipt].y, -pts[ipt].z);
		double invD = 1.0 / det(vec_ecef[0], vec_ecef[1], vpt);
		double u = det(vec0, vec_ecef[1], vpt) * invD;
		double v = det(vec_ecef[0], vec0, vpt) * invD;
		double uv = u + v;
		//double w = det(vec_ecef[0], vec_ecef[1], vec0);
		char inode = -1;
		if (uv < 0.5)
			inode = 0;
		else {
			if (u < 0.5) {
				if (v < 0.5) {
					inode = 3;
				}
				else {
					inode = 2;
				}
			}
			else {
				inode = 1;
			}
		}

		inodes[ipt] = inode;
	}
}


// return layer datum corresponding to layer types specified as a list of LayerType,
// datum with the largest resolutions less than the resolution specified are selected.
// If there is no data satisfying the constraint of the resolution, null data is returned.
  const void Node::getLayerData(
				list<list<const LayerData*>> & layerData,
				const list<LayerType> & layerType, 
				const vec3 & center, const float radius,
				const float resolution)
  {
    
    if (!collision(center, radius))
      return;

    if (layerData.size() != layerType.size()){
      layerData.resize(layerType.size());
    }

	list<bool> filled(layerType.size(), false);
	list<LayerType> detailedLayerType; 
	{
		auto itrData = layerData.begin();
		auto itrType = layerType.begin();
		auto itrFilled = filled.begin();
		for (; itrData != layerData.end(); itrData++, itrType++) {
			const LayerData * data = getLayerData(*itrType);
			if (!data) {
				continue;
			}
			if (data->resolution() < resolution || !bdownLink) {
				itrData->push_back(data);
				(*itrFilled) = true;
				continue;
			}
			detailedLayerType.push_back(*itrType);
		}
		if (detailedLayerType.size() == 0)
			return;
	}

    // retrieving more detailed data than this node
    list<list<const LayerData*>> detailedLayerData;

	detailedLayerData.resize(layerType.size());
    if(bdownLink){
      for (int idown = 0; idown < 4; idown++){
		  if (downLink[idown] == NULL) {
			  downLink[idown] = load(this, idown);
		  }
		downLink[idown]->getLayerData(detailedLayerData, detailedLayerType,
				      center, radius, resolution);
      }
    }
    
	auto itrData = layerData.begin();
	auto itrType = layerType.begin();
	auto itrFilled = filled.begin();

	auto itrDetailedType = detailedLayerType.begin();
	auto itrDetailedDatum = detailedLayerData.begin();
	for (; itrFilled != filled.end(); itrFilled++, itrData++, itrType++) {
		if (*itrFilled) // data of the type is filled in this layer.
			continue;

		assert(*itrType == *itrDetailedType);

		for (auto itrDetailedData = itrDetailedDatum->begin();
			itrDetailedData != itrDetailedDatum->end(); itrDetailedData++) {
			itrData->push_back((*itrDetailedData));
		}
		itrDetailedDatum++;
		itrDetailedType++;
	}
  }
  
  const LayerData * Node::getLayerData(const LayerType layerType)
  {
	auto itrLayerData = layerDataList.find(layerType);
	if (itrLayerData == layerDataList.end())
		return NULL;
	
	LayerData * layerData = itrLayerData->second;
	if (!layerData->isActive())
		layerData->load();
	Node::accessed(this);
	LayerData::accessed(layerData);
	return layerData;
  }

  void Node::insertLayerData(LayerData * pLayerData)
  {
	  layerDataList.insert(pair<LayerType, LayerData*>(pLayerData->getLayerType(), pLayerData));
  }

bool Node::addLayerData(const LayerData & layerData, 
			const size_t sz_node_data_lim)
{
	bupdate = true;
	Node::accessed(this);

	// merge new layer data to the layer data in this node. 
	// if the layer data object is not exist in this node, create empty that and merge.

	auto itrDstLayerData = layerDataList.find(layerData.getLayerType());
	LayerData * pDstLayerData = NULL; // The layer data object in this node.

	if (itrDstLayerData != layerDataList.end()){ 
		// LayerData exists in this node.
		pDstLayerData = itrDstLayerData->second;
		if (!pDstLayerData->isActive())
			// if the data is not active, activate it by loading.
			pDstLayerData->load();
	}
	else{ 
		// the layer data object is not in the node, create new one.
		pDstLayerData = LayerData::create(layerData.getLayerType());
		pDstLayerData->setNode(this);
		pDstLayerData->setActive();
		insertLayerData(pDstLayerData);
	}


#ifdef _AWS_MAP_DEBUG
	char path[1024];
	getPath(path, 1024);
#endif


	if (bdownLink) {
#ifdef _AWS_MAP_DEBUG
		cout << "Distributing " << strLayerType[layerData.getLayerType()] << " on " << path << endl;
#endif
		distributeLayerData(layerData);
#ifdef _AWS_MAP_DEBUG
		cout << "Distributed " << strLayerType[layerData.getLayerType()] << " on " << path << endl;
#endif
	}

	if (pDstLayerData){
		cout << "Merge: src size " << layerData.size() << " dst size " << pDstLayerData->size() << endl;
		pDstLayerData->merge(layerData);
		cout << "Merged: size " << pDstLayerData->size() << endl;
		if (pDstLayerData->size() > sz_node_data_lim){ 
			if (!bdownLink) {
				// the first experience the layer data in this node exceed the limit. 
				// create 4 lower nodes 
#ifdef _AWS_MAP_DEBUG
				cout << "Creating Downlink on " << path << endl;
#endif
				createDownLink();
				distributeLayerData(*pDstLayerData);
#ifdef _AWS_MAP_DEBUG
				cout << "Downlink Created on " << path << endl;
#endif
			}

			// if the data size exceeds the limit, 
			// reduce the data in this node with certain abstraction, 
			// and create downlink for detailed data.
#ifdef _AWS_MAP_DEBUG
			cout << strLayerType[pDstLayerData->getLayerType()] << " Reduction in " << path << endl;
#endif
			pDstLayerData->reduce(sz_node_data_lim);
#ifdef _AWS_MAP_DEBUG
			cout << "Finished " << strLayerType[pDstLayerData->getLayerType()] << " Reduction in " << path << endl;
#endif
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////// LayerData
LayerData * LayerData::head = NULL;
LayerData * LayerData::tail = NULL;
unsigned int LayerData::totalSize = 0;

void LayerData::insert(LayerData * pLayerData)
{
	if (pLayerData->next != NULL || pLayerData->prev != NULL)
		return; // the instance has already been loaded

	if (totalSize >= MapDataBase::getMaxTotalSizeLayerData())
		restruct();

	totalSize += (unsigned int) pLayerData->size();

	if (head == NULL && tail == NULL){
		pLayerData->next = pLayerData->prev = NULL;
		head = tail = pLayerData;
		return;
	}
	pLayerData->next = NULL;
	pLayerData->prev = tail;
	tail->next = pLayerData;
	tail = tail->next;
}

void LayerData::pop(LayerData * pLayerData)
{
  LayerData * prev = pLayerData->prev, *next = pLayerData->next;
  pLayerData->prev = pLayerData->next = NULL;
  
  if (next == NULL) 
	  tail = prev;
  else
	  next->prev = prev;

  if (prev == NULL) 
	  head = next;
  else
	  prev->next = next;

  totalSize -= (unsigned int) pLayerData->size();
}
  
void LayerData::accessed(LayerData * pLayerData)
{
	if (!pLayerData->isActive())
		return;
	pop(pLayerData);
	insert(pLayerData);
}

void LayerData::restruct()
{
	while (totalSize >= MapDataBase::getMaxTotalSizeLayerData())
	{
	  if(head){
	    LayerData * pLayerData = head;
	    pLayerData->release();
	  }
	}
}

LayerData * LayerData::create(const LayerType layerType)
{
	switch (layerType){
	case lt_coast_line:
		return new CoastLine;
	}
	return NULL;
}

bool LayerData::save(){
	if (!bupdate){
		return true;
	}

	if (!pNode)
		return false;

	char fname[2048];
	genFileName(fname, 2048);
#ifdef _AWS_MAP_DEBUG
	cout << "saving " << fname << endl;
#endif
	ofstream ofile(fname, ios::binary);
	if (!ofile.is_open())
		return false;

	if (!save(ofile))
		return false;

	bupdate = false;
	return true;
}

bool LayerData::load(){
	if (!pNode)
		return false;

	char fname[2048];
	genFileName(fname, 2048);
#ifdef _AWS_MAP_DEBUG
	cout << "loading " << fname << endl;
#endif
	ifstream ifile(fname, ios::binary);
	if (!ifile.is_open())
		return false;

	if (!load(ifile))
		return false;

	bupdate = false;

	setActive();

	return true;
}

void LayerData::release()
{
	if (!isActive())
		return;

#ifdef _AWS_MAP_DEBUG
	cout << "releasing " << strLayerType[getLayerType()] << ": " << endl;
	print();
#endif
	if (bupdate){
		save();
	}
	_release();
	LayerData::pop(this);
	bactive = false;
}

bool LayerData::reduce(const size_t sz_lim)
{	
	if (!isActive())
		return false;

	unsigned int size_prev = size();
	bool result = _reduce(sz_lim);
	resize(size() - size_prev);
	LayerData::accessed(this);
	bupdate = true;
	return result;
}

bool LayerData::merge(const LayerData & layerData)
{
	
	if (!isActive())
		return false;

	unsigned int size_prev = size();
	bool result = _merge(layerData);
	resize(size() - size_prev);
	LayerData::accessed(this);
	bupdate = true;
	return result;
}

///////////////////////////////////////////////////////////////////// CoastLine

  const vector<vec3> CoastLine::null_vec_vec3;
  const vector<vec2> CoastLine::null_vec_vec2;
  
  CoastLine::CoastLine() :dist_min(FLT_MAX), total_size(0)
{
}

CoastLine::~CoastLine()
{
}

bool CoastLine::save(ofstream & ofile)
{
	unsigned int nlines = (unsigned int) lines.size();
	ofile.write((const char*)&nlines, sizeof(unsigned int));
	for (auto itr = lines.begin(); itr != lines.end(); itr++){
		vector<vec2> & pts = (*itr)->pts;
		unsigned int length = (unsigned int) pts.size();
		ofile.write((const char*)&length, sizeof(unsigned int));
		for (auto itr_pt = pts.begin(); itr_pt != pts.end(); itr_pt++){	
			vec2 pt = *itr_pt;
			ofile.write((const char*)(&pt), sizeof(vec2));
		}
	}
	return true;
}

bool CoastLine::load(ifstream & ifile)
{
	unsigned int nlines = 0;
	ifile.read((char*)&nlines, sizeof(unsigned int));
	lines.resize(nlines);

	for (auto itr = lines.begin(); itr != lines.end(); itr++){
		(*itr) = new s_line;

		vector<vec2> & pts = (*itr)->pts;
		vector<vec3> & pts_ecef = (*itr)->pts_ecef;
		unsigned int length;
		ifile.read((char*)&length, sizeof(unsigned int));
		pts.resize(length);
		pts_ecef.resize(length);
		auto itr_pt_ecef = pts_ecef.begin();
		for (auto itr_pt = pts.begin(); itr_pt != pts.end(); itr_pt++, itr_pt_ecef++){
			vec2 pt;
			vec3 pt_ecef;
			ifile.read((char*)(&pt), sizeof(vec2));
			bihtoecef(pt.lat, pt.lon, 0, pt_ecef.x, pt_ecef.y, pt_ecef.z);
			(*itr_pt_ecef) = pt_ecef;
			(*itr_pt) = pt;
		}
	}

	update_properties();

	return true;
}

void CoastLine::print() const
{
	char path[2048] = "Not Active";
	if(pNode)
		pNode->getPath(path, 2048);
	cout << "CoastLine:" << path << endl;
	cout << "\t pos:" << pt_center_bih.lat * 180. / PI << "," << pt_center_bih.lon * 180. / PI
		<< " radius:" << radius()
		<< " mindim:" << dist_min
		<< " lines:" << lines.size() << endl;
}

void CoastLine::_release()
{
	for (auto itr = lines.begin(); itr != lines.end(); itr++)
	{
		delete (*itr);
	}
	lines.clear();
}

size_t CoastLine::size() const
{
	return total_size;
}

float CoastLine::resolution() const
{
	return dist_min;
}

float CoastLine::radius() const
{
	return pt_radius;
}

vec3 CoastLine::center() const
{
	return pt_center;
}

bool CoastLine::split(list<Node*> & nodes, Node * pParentNode) const
{
	vector<CoastLine> cls(nodes.size());
	vector<Node*> vnodes(nodes.size()); // vector version of nodes
	{
		int inode = 0;
		for (auto itr = nodes.begin(); itr != nodes.end(); itr++, inode++) {
			vnodes[inode] = *itr;
		}
	}

	for (int iline = 0; iline < lines.size(); iline++) {
		vector<vec3> & pts = lines[iline]->pts_ecef;
		// first find correspondance between points in the line and nodes.

		vector<char> asgnc(pts.size(), -1);
		if (pParentNode) {
			pParentNode->collision_downlink(pts, asgnc);
		}
		else {
			for (int ipt = 0; ipt < pts.size() - 1; ipt++) {
				int inode = 0;
				for (auto itr = nodes.begin(); itr != nodes.end(); itr++) {
					Node * pNode = *itr;
					if (pNode->collision(pts[ipt])) {
						asgnc[ipt] = inode;
						break;
					}
					inode++;
				}
			}
		}
		asgnc.back() = -1;

		// second split line into lines, while doubling boundary points not to miss links between nodes
		int in = asgnc[0], inn = -1; // node index "in" and next node index "inn"
		int ipts = 0, ipte = 0; // start point index "ipts" and end point index "ipte".
		list<vec2> line_new; // newly added line partially extracted from lines[iline]
		for (int ipt = 1; ipt < pts.size(); ipt++) {
			inn = asgnc[ipt];

			if (in != inn) {
				ipte = ipt + 1; // this "+1" allows to make overwrap on next part.

				// form new line contains points ipts to ipte.
				for (int iptl = ipts; iptl < ipte; iptl++) {
					line_new.push_back(lines[iline]->pts[iptl]);
				}

				cls[in].add(line_new);
				line_new.clear();
				in = inn;
				ipts = ipt;
				continue;
			}
		}
	}

	for (int inode = 0; inode < nodes.size(); inode++) {
		cls[inode].update_properties();
#ifdef _AWS_MAP_DEBUG
		cout << "Adding layer data: " << endl;
		cls[inode].print();
		cout << " \tsize: " << cls[inode].size() << endl;
#endif
		if (cls[inode].size() > 0)
			vnodes[inode]->addLayerData(cls[inode], MapDataBase::getMaxSizeLayerData(cls[inode].getLayerType()));
	}

	return true;
}


int CoastLine::try_reduce(int nred)
{
	// select nred points with shortest distance to both sides in the line (excluding terminal points)
	struct s_red_pt{
		int iline;
		int ipt;
		double dist;
		s_red_pt * prev, *next;
		s_red_pt() :iline(-1), ipt(-1), dist(DBL_MAX), prev(NULL), next(NULL){};
	};

	// creating distance data 
	int npts_list = 0;
	vector<s_red_pt*> redpts(lines.size());

	for (int iline = 0; iline < lines.size(); iline++){
		redpts[iline] = NULL;
		vector<vec3> & pts = lines[iline]->pts_ecef;
		if(pts.size() > 3)
			npts_list += (int)pts.size() - 3;
	}

	// creating sort list
	vector<s_red_pt> mblock(npts_list);
	vector<s_red_pt*> sortlist(npts_list, NULL);
	npts_list = 0;
	for (int iline = 0; iline < lines.size(); iline++){
		vector<vec3> & pts = lines[iline]->pts_ecef;
		s_red_pt * redpt = NULL, * redpt_prev = NULL;
		for (int ipt = 2; ipt < pts.size() - 1; ipt++){
			redpt = &mblock[npts_list];
			redpt->iline = iline;
			redpt->ipt = ipt;
			redpt->dist = l2Norm(pts[ipt], pts[ipt - 1]);
			sortlist[npts_list] = redpt;
			if (!redpt_prev) {
				redpts[iline] = redpt;
			}
			else {
				redpt_prev->next = redpt;
				redpt->prev = redpt_prev;
			}
			redpt_prev = redpt;
			npts_list++;
		}
	}

	struct {
		bool operator () (const s_red_pt * p0, const s_red_pt * p1) const
		{
			return p0->dist < p1->dist;
		}
	} lessthan;

	sort(sortlist.begin(), sortlist.end(), lessthan);

	// reduction phase
#ifdef _AWS_MAP_DEBUG
	int nredd = 0;
	int prog = 0;	
	cout << "Progress(";
	cout << nred << "/" << sortlist.size() << "):";
#endif
	while (nredd < nred){
#ifdef _AWS_MAP_DEBUG
		if ((nredd * 10 / nred) > prog) {
			prog ++;
			cout << "*";
		}
#endif
		s_red_pt * redpt = sortlist[nredd];
		s_red_pt * redpt0 = redpt->prev;
		s_red_pt * redpt1 = redpt->next;
		int iline = redpt->iline;
		int ipt = redpt->ipt;
		int ipt0 = (redpt0 ? redpt0->ipt : 1);
		int ipt1 = (redpt1 ? redpt1->ipt : lines[iline]->pts.size() - 1);

		s_line & line = *lines[iline];
		vector<vec2> & pts = line.pts;
		vector<vec3> & pts_ecef = line.pts_ecef;
		vec2 pt_m;
		vec3 pt_ecef_m;
		pt_ecef_m = (pts_ecef[ipt0] + pts_ecef[ipt]);
		pt_ecef_m *= 0.5;
		double alt;
		eceftobih(pt_ecef_m.x, pt_ecef_m.y, pt_ecef_m.z, pt_m.lat, pt_m.lon, alt);

		if (redpt0){
			int iptm1 = (redpt0->prev ? redpt0->prev->ipt : 1);
			redpt0->dist = l2Norm(pt_ecef_m, pts_ecef[iptm1]);
			redpt0->next = redpt1;
		}
		else {
			redpts[iline] = redpt1;
		}

		if (redpt1){
			redpt1->dist = l2Norm(pt_ecef_m, pts_ecef[ipt1]);
			redpt1->prev = redpt0;
		}
		pts[ipt0] = pt_m;
		pts_ecef[ipt0] = pt_ecef_m;

		if (redpts[iline] == NULL) { // points has already been 3
			vec2 & pt0 = pts.front();
			vec2 & pt1 = pts.back();
			if (pt0.x == pt1.x && pt0.y == pt1.y) {
				delete lines[iline];
				lines[iline] = NULL;
				nredd += 3;
			}
		}

		nredd++;

		sort(sortlist.begin() + nredd, sortlist.end(), lessthan);
		if (sortlist.size() == 0)
			break;
	}
#ifdef _AWS_MAP_DEBUG
	cout << endl;
#endif
	for (int iline = 0; iline < lines.size(); iline++) {
		if (!lines[iline])
			continue;
		
		vector<vec2> & pts = lines[iline]->pts;
		vector<vec3> & pts_ecef = lines[iline]->pts_ecef;

		if (pts.size() < 3)
			continue;

		s_red_pt * redpt = redpts[iline];
		int ipt = 2;
		for (; redpt != NULL; redpt = redpt->next, ipt++){
			pts[ipt] = pts[redpt->ipt];
			pts_ecef[ipt] = pts_ecef[redpt->ipt];
		}
		pts[ipt] = pts.back();
		pts_ecef[ipt] = pts_ecef.back();
		pts.resize(ipt + 1);
		pts_ecef.resize(ipt + 1);
	}

	update_properties();

	return nred - nredd;
}

bool CoastLine::_reduce(const size_t sz_lim)
{
	bupdate = true;
	if (size() < sz_lim)
		return true;

	// calculate nred; the number of points to be reduced
	unsigned int sz_pts_lim = (unsigned int)(sz_lim);	
	unsigned int sz_pt = (unsigned int)(sizeof(vec2)+sizeof(vec3));
	unsigned int npts = 0;
	for (unsigned int iline = 0; iline < lines.size(); iline++){
		npts += (unsigned int) lines[iline]->pts.size();
	}

	unsigned int nred = npts - (sz_pts_lim / sz_pt);
	if (try_reduce(nred) != 0){
		return false;
	}
	return true;
}

bool CoastLine::_merge(const LayerData & layerData)
{
	bupdate = true;
	const CoastLine * src = dynamic_cast<const CoastLine*>(&layerData);
	const vector<s_line*> & lines_src = src->lines;

	// this loop finds connection between lines in layerData and this object.
	for (int iline0 = 0; iline0 < lines_src.size(); iline0++){
		
		s_line & line_src = *lines_src[iline0];
		vector<vec2> & pts_src = line_src.pts;
		vec2 & pt_src_begin = pts_src.front() , & pt_src_end = pts_src.back();

		// iline_con_begin is the line index of existing line connected with newly added line lines_src[iline0]
		// and bdst_begin_con_begin is true if lines[iline_con_begin] and lines_src[iline0] is connected with their starting points.
		// iline_con_end is the line index of existing line connected with newly added line lines_src[iline0]
		// and bdst_begin_con_end is true if the start point of lines[iline_con_end] is connected with the end point of lines_src[iline0]
		int iline_con_begin = -1, iline_con_end = -1; 
		bool bdst_begin_con_begin = true, bdst_begin_con_end = true;
		// check connection
		for (int iline1 = 0; iline1 < lines.size(); iline1++){
			s_line & line_dst = *lines[iline1];
			vector<vec2> & pts_dst = line_dst.pts;
			vec2 & pt_dst_begin = pts_dst.front(), & pt_dst_end = pts_dst.back();
			if (pt_src_begin == pt_dst_begin){
				iline_con_begin = iline1;
				bdst_begin_con_begin = true;
			}
			else if (pt_src_begin == pt_dst_end){
				iline_con_begin = iline1;
				bdst_begin_con_begin = false;
			}
			else{
				iline_con_begin = -1;
			}

			if (pt_src_end == pt_dst_begin){
				iline_con_end = iline1;
				bdst_begin_con_end = true;
			}
			else if (pt_src_end == pt_dst_end){
				iline_con_end = iline1;
				bdst_begin_con_end = false;
			}
			else{
				iline_con_end = -1;
			}
		}

		s_line * pline_begin = NULL, *pline_end = NULL, *pline_new;

		if (iline_con_begin >= 0){
			pline_new = new s_line;
			// the starting point of lines_src[iline0] is connected with lines[iline_con_begin]
			pline_begin = lines[iline_con_begin];
			if (bdst_begin_con_begin){
				// if the connection is head to head, first reverse the lines[iline_con_begin]
				reverse(pline_begin->pts.begin(), pline_begin->pts.end());
				reverse(pline_begin->pts_ecef.begin(), pline_begin->pts_ecef.end());
			}
			// copy lines[iline_con_begin] to newly created object *pline_new
			*pline_new = *pline_begin;

			pline_new->pts.pop_back();
			pline_new->pts_ecef.pop_back();
			// insert lines_src[iline0] at the end of pline_new 
			// note that the end point is exactly the point of lines_src[iline0]. This is very important later if the end point is connected with other line.
			pline_new->pts.insert(pline_new->pts.end(), pts_src.begin(), pts_src.end());
			pline_new->pts_ecef.insert(pline_new->pts_ecef.end(), line_src.pts_ecef.begin(), line_src.pts_ecef.end());

			// delete old object lines[iline_con_begin] and replace it with pline_new
			delete lines[iline_con_begin];
			lines[iline_con_begin] = pline_new;
		}

		if (iline_con_end >= 0){
			

			// the end point of lines_src[iline0] is connected with lines[iline_con_end]
			pline_end = lines[iline_con_end];
			vector<vec2> & pts = pline_end->pts;
			vector<vec3> & pts_ecef = pline_end->pts_ecef;

			if (!bdst_begin_con_end){
				// if lines_src[iline0] is connected with the end point of lines[iline_con_end] reverse it.
				reverse(pts.begin(), pts.end());
				reverse(pts_ecef.begin(), pts_ecef.end());
			}

			if (iline_con_begin >= 0) {
				// if previously start point of lines_src[iline0] is connected with lines[iline_con_begin], 
				// the end point of lines_src[iline0] is reserved as that of lines[iline_con_begin]
				pline_new = lines[iline_con_begin];
				lines[iline_con_begin] = NULL;

			}else {
				pline_new = new s_line;
				*pline_new = line_src;
			}
			pline_new->pts.pop_back();
			pline_new->pts_ecef.pop_back();
			pline_new->pts.insert(pline_new->pts.end(), pts.begin(), pts.end());
			pline_new->pts_ecef.insert(pline_new->pts_ecef.end(), pts_ecef.begin(), pts_ecef.end());
			delete lines[iline_con_end];
			lines[iline_con_end] = pline_new;
		}

		if (iline_con_begin < 0 && iline_con_end < 0) {
			pline_new = new s_line;
			*pline_new = line_src;
			lines.push_back(pline_new);
		}
	}

	// erase null element from lines
	for (vector<s_line*>::iterator itr = lines.begin(); itr != lines.end();){
		if (*itr == NULL)
			itr = lines.erase(itr);
		else
			itr++;
	}

	update_properties();
	return true;
}

LayerData * CoastLine::clone() const
{
	CoastLine * pnew = new CoastLine;

	pnew->pNode = NULL;
	pnew->bupdate = bupdate;
	pnew->dist_min = dist_min;
	pnew->total_size = total_size;

	vector<s_line*> & lines_new = pnew->lines;
	lines_new.resize(lines.size());
	auto itr_new = lines_new.begin();
	for (auto itr = lines.begin(); itr != lines.end(); itr++, itr_new++){
		*itr_new = new s_line;
		*(*itr_new) = *(*itr);
	}

	return pnew;
}

	void CoastLine::add(list<vec2> & line)
	{
		s_line * pline = new s_line;

		pline->pts.resize(line.size());
		pline->pts_ecef.resize(line.size());
		list<vec2>::iterator itr_src = line.begin();
		vector<vec2>::iterator itr_dst = pline->pts.begin();
		vector<vec3>::iterator itr_dst_ecef = pline->pts_ecef.begin();
		for (; itr_src != line.end(); itr_src++, itr_dst++, itr_dst_ecef++) {
			*itr_dst = *itr_src;
			bihtoecef(itr_dst->lat, itr_dst->lon, 0., itr_dst_ecef->x, itr_dst_ecef->y, itr_dst_ecef->z);
		}
		lines.push_back(pline);

		// calculating resolution and size
		vector<vec3> & pts = pline->pts_ecef;
		for (int i = 1; i < pts.size(); i++) {
			vec3 & pt0 = pts[i - 1];
			vec3 & pt1 = pts[i];
			dist_min = min(dist_min, l2Norm(pt0, pt1));
		}
		total_size += pline->size();
	}

	void CoastLine::update_properties()
	{
		// removing null line
		for (auto itr = lines.begin(); itr != lines.end();) {
			if (*itr == NULL)
				itr = lines.erase(itr);
			else
				itr++;
		}

		// calculating center 	
		pt_center = vec3(0, 0, 0);
		unsigned int num_total_points = 0;
		for (int iline = 0; iline < lines.size(); iline++){
			vector<vec3> & pts = lines[iline]->pts_ecef;
			for (int i = 0; i < pts.size(); i++){
				pt_center += pts[i];
			}
			num_total_points += (unsigned int)pts.size();
		}
		pt_center *= (1.0 / (double)num_total_points);
		double alt;
		eceftobih(pt_center.x, pt_center.y, pt_center.z, pt_center_bih.lat, pt_center_bih.lon, alt);
		

		total_size = 0;
		pt_radius = 0;
		dist_min = DBL_MAX;
		for (int iline = 0; iline < lines.size(); iline++){
			// calculating resolution and size
			vector<vec3> & pts = lines[iline]->pts_ecef;
			pt_radius = max(pt_radius, l2Norm(pt_center, pts[0]));
			for (int i = 2; i < pts.size()-1; i++) {
				vec3 & pt0 = pts[i - 1];
				vec3 & pt1 = pts[i];
				double dist = l2Norm(pt0, pt1);
				if (dist == 0) {
					pts.erase(pts.begin() + i);
					i--;
					continue;
				}
				dist_min = min(dist_min, dist);
				pt_radius = max(pt_radius, l2Norm(pt_center, pts[i]));
			}
			total_size += lines[iline]->size();
		}
	}

	bool CoastLine::loadJPJIS(const char * fname)
	{
		ifstream fjpgis(fname);
		if (!fjpgis.is_open()) {
			cerr << "Failed to open file " << fname << "." << endl;
			return false;
		}

		char buf[1024];
		bool bline = false;
		list<vec2> line;
		while (!fjpgis.eof())
		{
			fjpgis.getline(buf, 1024);
			if (!bline) {
				if (strcmp(buf, "\t\t\t<gml:posList>") == 0) {
					bline = true;
				}
			}
			else {
				if (strcmp(buf, "\t\t\t</gml:posList>") == 0) {
					add(line);
					line.clear();
					bline = false;
					continue;
				}

				vec2 pt;
				char * p;
				for (p = buf; *p != ' ' && *p != '\0'; p++);
				*p = '\0';
				p++;
				pt.x = (float)(atof(buf) * PI / 180.);
				pt.y = (float)(atof(p) * PI / 180.);

				if (line.size() == 0 || (pt.x != line.back().x || pt.y != line.back().y))
					line.push_back(pt);
			}
		}

		update_properties();
		return true;
	}
}

//////////////////////////////////////////////////////////////////////////////////////c_icosahedron
c_icosahedron::c_icosahedron() :nv(12), nf(20), ne(30)
{
	q = new Point2f[12];
	v = new Point3f[12];
	f = new unsigned int*[20];

	f[0] = new unsigned int[60];
	for (int i = 0; i < 20; i++)
		f[i] = f[0] + i * 3;

	e = new unsigned int*[30];
	e[0] = new unsigned int[60];
	for (int i = 0; i < 30; i++){
		e[i] = e[0] + i * 2;
	}

	float lat0 = (float) atan2(GR, 1), lat1 = (float) atan2(1, GR);
	float lon = (float) atan2(GR, 1);

	q[0] = Point2f(lat0, 0.5 * PI);
	q[1] = Point2f(lat0, -0.5 * PI);
	q[2] = Point2f(-lat0, 0.5 * PI);
	q[3] = Point2f(-lat0, -0.5 * PI);

	q[4] = Point2f(0, atan2(GR, 1));
	q[5] = Point2f(0, atan2(GR, -1));
	q[6] = Point2f(0, atan2(-GR, 1));
	q[7] = Point2f(0, atan2(-GR, -1));

	q[8] = Point2f(lat1, 0);
	q[9] = Point2f(-lat1, 0);
	q[10] = Point2f(lat1, PI);
	q[11] = Point2f(-lat1, PI);

	for (int i = 0; i < 12; i++){
		bihtoecef(q[i].x, q[i].y, 0.f, v[i].x, v[i].y, v[i].z);
	}

	/*
	v[0] = Point3f(0, 1, GR);
	v[1] = Point3f(0, -1, GR);
	v[2] = Point3f(0, 1, -GR);
	v[3] = Point3f(0, -1, -GR);
	v[4] = Point3f(1, GR, 0);
	v[5] = Point3f(-1, GR, 0);
	v[6] = Point3f(1, -GR, 0);
	v[7] = Point3f(-1, -GR, 0);
	v[8] = Point3f(GR, 0, 1);
	v[9] = Point3f(GR, 0, -1);
	v[10] = Point3f(-GR, 0, 1);
	v[11] = Point3f(-GR, 0, -1);
	*/

	f[0][0] = 0; f[0][1] = 1; f[0][2] = 8; // A
	f[1][0] = 1; f[1][1] = 0; f[1][2] = 10; // A Rz(180)
	f[8][0] = 2; f[8][1] = 3; f[8][2] = 11; // A Ry(180)
	f[9][0] = 3; f[9][1] = 2; f[9][2] = 9; // A Rx(180)
	f[3][0] = 0; f[3][1] = 4; f[3][2] = 5; // B 
	f[6][0] = 1; f[6][1] = 7; f[6][2] = 6; // B Rz(180)
	f[11][0] = 2; f[11][1] = 5; f[11][2] = 4; // B Ry(180)
	f[14][0] = 3; f[14][1] = 6; f[14][2] = 7; // B Rx(180)
	f[2][0] = 0; f[2][1] = 8; f[2][2] = 4; // C1
	f[5][0] = 1; f[5][1] = 10; f[5][2] = 7; // C1 Rz(180)
	f[12][0] = 2; f[12][1] = 11; f[12][2] = 5; // C1 Ry(180)
	f[13][0] = 3; f[13][1] = 9; f[13][2] = 6; // C1 Rx(180)
	f[4][0] = 0; f[4][1] = 5; f[4][2] = 10; // C2 S(-x) I(1,2) (change the sign in x, swap the vertices 1 and 2)
	f[7][0] = 1; f[7][1] = 6; f[7][2] = 8; // C2 Rz(180)
	f[10][0] = 2; f[10][1] = 4; f[10][2] = 9; // C2 Ry(180)
	f[15][0] = 3; f[15][1] = 7; f[15][2] = 11; // C2 Rx(180
	f[16][0] = 4; f[16][1] = 8; f[16][2] = 9; // D
	f[17][0] = 5; f[17][1] = 11; f[17][2] = 10; // D Ry(180)
	f[18][0] = 6; f[18][1] = 9; f[18][2] = 8; // D Rx(180)
	f[19][0] = 7; f[19][1] = 10; f[19][2] = 11; // D Rz(180)

	e[0][0] = 0; e[0][1] = 1;
	e[1][0] = 0; e[1][1] = 4;
	e[2][0] = 0; e[2][1] = 5;
	e[3][0] = 0; e[3][1] = 8;
	e[4][0] = 0; e[4][1] = 10;
	e[5][0] = 1; e[5][1] = 6;
	e[6][0] = 1; e[6][1] = 7;
	e[7][0] = 1; e[7][1] = 8;
	e[8][0] = 1; e[8][1] = 10;
	e[9][0] = 2; e[9][1] = 3;
	e[10][0] = 2; e[10][1] = 4;
	e[11][0] = 2; e[11][1] = 5;
	e[12][0] = 2; e[12][1] = 9;
	e[13][0] = 2; e[13][1] = 11;
	e[14][0] = 3; e[14][1] = 6;
	e[15][0] = 3; e[15][1] = 7;
	e[16][0] = 3; e[16][1] = 9;
	e[17][0] = 3; e[17][1] = 11;
	e[18][0] = 4; e[18][1] = 5;
	e[19][0] = 4; e[19][1] = 8;
	e[20][0] = 4; e[20][1] = 9;
	e[21][0] = 5; e[21][1] = 10;
	e[22][0] = 5; e[22][1] = 11;
	e[23][0] = 6; e[23][1] = 7;
	e[24][0] = 6; e[24][1] = 8;
	e[25][0] = 6; e[25][1] = 9;
	e[26][0] = 7; e[26][1] = 10;
	e[27][0] = 7; e[27][1] = 11;
	e[28][0] = 8; e[28][1] = 9;
	e[29][0] = 10; e[29][1] = 11;
}

c_icosahedron::c_icosahedron(const c_icosahedron & icshdrn)
{
	nv = icshdrn.nv + icshdrn.ne;
	nf = 4 * icshdrn.nf;
	ne = icshdrn.ne * 2 + icshdrn.nf * 3;
	
	q = new Point2f[nv];
	v = new Point3f[nv];

	f = new unsigned int*[nf];
	f[0] = new unsigned int[nf * 3];
	for (unsigned int i = 0; i < nf; i++){
		f[i] = f[0] + i * 3;
	}

	e = new unsigned int*[ne];
	e[0] = new unsigned int[ne * 2];
	for (unsigned int i = 0; i < ne; i++){
		e[i] = e[0] + i * 2;
	}	

	// calculating vertices
	memcpy((void*)v, (void*)icshdrn.v, sizeof(Point3f)*icshdrn.nv);
	memcpy((void*)q, (void*)icshdrn.q, sizeof(Point2f)*icshdrn.nv);
	
	for (unsigned int ie = 0, iv = icshdrn.nv; ie < icshdrn.get_ne(); ie++, iv++){
		Point3f m = icshdrn.get_mid_point(ie);
		float alt;
		eceftobih(m.x, m.y, m.z, q[iv].x, q[iv].y, alt);
		bihtoecef(q[iv].x, q[iv].y, 0., v[iv].x, v[iv].y, v[iv].z);
	}

	// calculating edges and faces
	for (unsigned int i = 0; i < icshdrn.ne; i++){
		unsigned int ie = i * 2;
		e[ie][0] = icshdrn.e[i][0];
		e[ie][1] = icshdrn.nv + i;
		ie++;
		e[ie][0] = icshdrn.e[i][1];
		e[ie][1] = icshdrn.nv + i;
	}

	unsigned int ie = icshdrn.ne * 2;
	unsigned int ifc = 0;
	for (unsigned int i = 0; i < icshdrn.nf; i++){
		unsigned int * _f = icshdrn.f[i];
		unsigned int iv0 = icshdrn.get_edge(_f[0], _f[1]) + icshdrn.nv;
		unsigned int iv1 = icshdrn.get_edge(_f[1], _f[2]) + icshdrn.nv;
		unsigned int iv2 = icshdrn.get_edge(_f[2], _f[0]) + icshdrn.nv;

		if (iv0 < iv1){
			e[ie][0] = iv0;
			e[ie][1] = iv1;
		}
		else{
			e[ie][0] = iv1;
			e[ie][1] = iv0;
		}
		ie++;

		if (iv1 < iv2){
			e[ie][0] = iv1;
			e[ie][1] = iv2;
		}
		else{
			e[ie][0] = iv2;
			e[ie][1] = iv1;
		}
		ie++;

		if (iv2 < iv0){
			e[ie][0] = iv2;
			e[ie][1] = iv0;
		}
		else{
			e[ie][0] = iv0;
			e[ie][1] = iv2;
		}
		ie++;

		f[ifc][0] = _f[0]; f[ifc][1] = iv0; f[ifc][2] = iv2;
		ifc++;
		f[ifc][0] = iv0; f[ifc][1] = _f[1]; f[ifc][2] = iv1;
		ifc++;
		f[ifc][0] = iv2; f[ifc][1] = iv1; f[ifc][2] = _f[2];
		ifc++;
		f[ifc][0] = iv0; f[ifc][1] = iv1; f[ifc][2] = iv2;
		ifc++;
	}
}

c_icosahedron::~c_icosahedron()
{
	delete[]v;
	delete[]q;
	delete[] f[0];
	delete[] f;
	delete[] e[0];
	delete[] e;
	v = NULL;
	q = NULL;
	f = NULL;
	e = NULL;
}
