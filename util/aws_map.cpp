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


namespace AWSMap2 {
	LayerType getLayerType(const char * str) {
		for (int lt = 0; lt < (int)lt_undef; lt++) {
			if (strcmp(strLayerType[lt], str) == 0)
				return (LayerType)lt;
		}

		return lt_undef;
	}

	////////////////////////////////////////////////////////////// MapDataBase
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
	}

	const LayerData * MapDataBase::request(const Point3f & location, const float radius,
		const LayerType & layerType, const float resolution)
	{

		return NULL;
	}

	vector<const LayerData*> MapDataBase::request(const Point3f & location, const float radius,
		const vector<LayerType> & layerTypes, const float resolution)
	{

	}

	bool MapDataBase::insert(const Point3f & location, const LayerData * layerData)
	{
		return true;
	}

	bool MapDataBase::erase(const LayerData * layerData)
	{
		return true;
	}

	///////////////////////////////////////////////////////////////////// Node
Node::Node() : upLink(NULL)
{
	downLink[0] = downLink[1] = downLink[2] = downLink[3] = NULL;
}

Node::~Node()
{
	for (int i = 0; i < 4; i++) {
		if (downLink[i])
			delete downLink[i];
		downLink[i] = NULL;
	}
}

bool Node::save()
{
	return true;
}

bool Node::load()
{
	return true;
}

const Node * Node::collision(const Point3f & location)
{
	return NULL;
}

const LayerData * Node::getLayerData(const LayerType layerType)
{
	return NULL;
}

bool Node::addLayerData(const LayerData & layerData, const size_t sz_node_data_lim = 0x4FFFFF /* 4MB */)
{
	// seeks nodeList to be added
	// call split method of the layerData with nodeList

	return true;
}

///////////////////////////////////////////////////////////////////// LayerData

CoastLine::CoastLine() :dist_min(FLT_MAX)
{
}

CoastLine::~CoastLine()
{
}

bool CoastLine::save()
{
	return true;
}

bool CoastLine::load()
{
	return true;
}

size_t CoastLine::size()
{
	return total_size;
}

float CoastLine::resolution()
{
	return dist_min;
}

float CoastLine::radius()
{
	return 0;
}

Point3f CoastLine::center()
{
	return Point3f();
}

bool CoastLine::split(list<Node*> & nodes)
{
	vector<CoastLine*> cls(nodes.size(), NULL);
	list<Point2f> line_new;
	int inode_line_new = 0;
	Node * pNode_line_new = NULL;
	for (int iline = 0; iline < lines.size(); iline++) {

		vector<Point3f> & pts = lines[iline]->pts_ecef;
		for (int ipt = 0; ipt < pts.size(); ipt++) {
			if (pNode_line_new != NULL) {
				line_new.push_back(lines[iline]->pts[ipt]);
				if (!pNode_line_new->collision(pts[ipt])) {
					if (cls[inode_line_new] == NULL) {
						cls[inode_line_new] = new CoastLine;
					}
					cls[inode_line_new]->add(line_new);
					pNode_line_new = NULL;
					line_new.clear();
				}
			}
			else {
				inode_line_new = 0;
				for (list<Node*>::iterator itr = nodes.begin(); itr != nodes.end(); itr++) {
					Node * pNode = *itr;
					if (pNode->collision(pts[ipt])) {
						if (pNode_line_new == NULL) {
							line_new.push_back(lines[iline]->pts[ipt]);
							pNode_line_new = pNode;
							break;
						}
					}
					inode_line_new++;
				}
			}
		}
	}
	int inode = 0;
	for (list<Node*>::iterator itr = nodes.begin(); itr != nodes.end(); itr++) {
		(*itr)->addLayerData(*cls[inode]);
		inode++;
	}

	return true;
}

	void CoastLine::add(list<Point2f> & line)
	{
		s_line * pline = new s_line;

		pline->id = lines.size();
		pline->pts.resize(line.size());
		pline->pts_ecef.resize(line.size());
		list<Point2f>::iterator itr_src = line.begin();
		vector<Point2f>::iterator itr_dst = pline->pts.begin();
		vector<Point3f>::iterator itr_dst_ecef = pline->pts_ecef.begin();
		for (; itr_src != line.end(); itr_src++, itr_dst++) {
			*itr_dst = *itr_src;
			bihtoecef(itr_dst->x, itr_dst->y, 0., itr_dst_ecef->x, itr_dst_ecef->y, itr_dst_ecef->z);
		}
		lines.push_back(pline);

		// calculating resolution and size
		vector<Point3f> & pts = pline->pts_ecef;
		for (int i = 1; i < pts.size(); i++) {
			Point3f & pt0 = pts[i - 1];
			Point3f & pt1 = pts[i];
			dist_min = min(dist_min, (float)norm(pt0 - pt1));
			total_size += pline->size();
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
		list<Point2f> line;
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
					break;
				}

				Point2f pt;
				char * p;
				for (p = buf; *p != ' ' && *p != '\0'; p++);
				*p = '\0';
				p++;
				pt.x = (float)(atof(buf) * PI / 180.);
				pt.y = (float)(atof(p) * PI / 180.);
				line.push_back(pt);
			}
		}

		return true;
	}


}

using namespace AWSMap;

const char * LayerStr[amlc_undef] = {
	"cl"
};

//////////////////////////////////////////////////////////////////////////////////////////
c_layer::c_layer() :bupdate(false)
{
}

c_layer::~c_layer()
{
}

c_layer * c_layer::create(const LayerCode code)
{
	switch (code){
	case amlc_coast_line:
		return new c_coast_line;
	}

	return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////
c_coast_line::c_coast_line()
{
}

c_coast_line::~c_coast_line()
{
}

bool c_coast_line::save(const char * fname)
{

	char buf[1024];
	snprintf(buf, 1024, "%s/%s", c_map::path, fname);

	ofstream ofile(buf, ios::binary);
	if (!ofile.is_open()){
		cerr << "Cannot open " << fname << endl;
		return false;
	}
	
	int nLines = lines.size();
	ofile.write((char*)&nLines, sizeof(nLines));

	for (int iline = 0; iline < nLines; iline++){
		s_line & line = lines[iline];
		int nPoints = line.points.size();
		ofile.write((char*)&nPoints, sizeof(nPoints));
		for (int ipt = 0; ipt < nPoints; ipt++){
			ofile.write((char*)&line.points[ipt], sizeof(Point3f));
		}
	}

	bupdate = false;
	return	true;
}

bool c_coast_line::load(const char * fname)
{
	char buf[1024];
	snprintf(buf, 1024, "%s/%s", c_map::path, fname);

	ifstream ifile(buf, ios::binary);
	if (!ifile.is_open()){
		cerr << "Cannot open " << fname << endl;
		return false;
	}

	int nLines;
	ifile.read((char*)&nLines, sizeof(nLines));
	lines.resize(nLines);

	for (int iline = 0; iline < nLines; iline++){
		s_line & line = lines[iline];
		int nPoints;
		ifile.read((char*)&nPoints, sizeof(nPoints));
		line.points.resize(nPoints);
		for (int ipt = 0; ipt < nPoints; ipt++){
			ifile.read((char*)&line.points[ipt], sizeof(Point3f));
		}
	}
	return true;
}

c_coast_line * c_coast_line::convertFromJPGIS(const char * fname)
{
	c_coast_line * pCL = new c_coast_line;

	ifstream fjpgis(fname);
	if (!fjpgis.is_open()){
		cerr << "Failed to open file " << fname << "." << endl;
		return false;
	}

	char buf[1024];
	bool bline = false;
	list<Point2f> line;
	while (!fjpgis.eof())
	{
		fjpgis.getline(buf, 1024);
		if (!bline){
			if (strcmp(buf, "\t\t\t<gml:posList>") == 0){
				bline = true;
			}
		}
		else{
			if (strcmp(buf, "\t\t\t</gml:posList>") == 0){
				pCL->addLine(line);
				line.clear();
				break;
			}
			
			Point2f pt;
			char * p;
			for (p = buf; *p != ' ' && *p != '\0'; p++);
			*p = '\0';
			p++;
			pt.x = (float)(atof(buf) * PI / 180.);
			pt.y = (float)(atof(p) * PI / 180.);
			line.push_back(pt);
		}
	}

	return pCL;
}

bool c_coast_line::addLine(s_line & line)
{
	vector<Point3f> & newPoints = line.points;

	Point3f & pe2 = newPoints.back();
	Point3f & pb2 = newPoints.front();
	if (pb2 == pe2){// detecting single segment loop
		line.bclosed = true;
		lines.push_back(line);
		return true;
	}

	// finding the joint point in previous shapes
	int ibline = -1, ieline = -1;
	enum connection{ none, e2b, e2e, b2b, b2e } bcon = none, econ = none;
	for (int iline = 0; iline < lines.size(); iline++){
		s_line & cline = lines[iline];
		Point3f & pe1 = cline.points.back();
		Point3f & pb1 = cline.points.front();
		if (bcon == none){
			if (pe1 == pb2){
				ibline = iline;
				bcon = b2e;
			}
			else if (pb1 == pb2){
				ibline = iline;
				bcon = b2b;
			}
		}

		if (econ == none){
			if (pe1 == pe2){
				ieline = iline;
				econ = e2e;
			}
			else if (pb1 == pe2){
				ieline = iline;
				econ = e2b;
			}
		}
		if (bcon != none && econ != none)
			break;
	}

	if (bcon == none && econ == none){ // the line is a new fragment	
		lines.push_back(line);
		
		return true;
	}

	// merge lines
	if (bcon != none && econ == none){
		vector<Point3f> & cpts = lines[ibline].points;
		if (bcon == b2e){
			// merge start point in the newLine and the end point in the candidate line
			cpts.insert(cpts.end(), newPoints.begin(), newPoints.end());
		}
		else{
			vector<Point3f> pts;
			pts.resize(cpts.size() + newPoints.size());
			int ipt = 0;
			for (int i = newPoints.size() - 1; i >= 0; i--){
				pts[ipt] = newPoints[i];
				ipt++;
			}
			for (int i = 0; i < cpts.size(); i++){
				pts[ipt] = cpts[i];
				ipt++;
			}
			cpts = pts;
		}
	}
	if (bcon == none && econ != none){ // only the end point is connected to a shape
		vector<Point3f> & cpts = lines[ibline].points;
		if (econ == e2b){
			cpts.insert(cpts.begin(), newPoints.begin(), newPoints.end());
		}
		else{
			vector<Point3f> pts;
			pts.resize(cpts.size() + newPoints.size());
			int ipt = 0;
			for (int i = 0; i < newPoints.size(); i++){
				pts[ipt] = newPoints[i];
				ipt++;
			}
			for (int i = cpts.size() - 1; i >= 0; i--){
				pts[ipt] = cpts[i];
				ipt++;
			}
			cpts = pts;
		}
	}
	else if (bcon != none && econ != none){ 
		vector<Point3f> & cptb = lines[ibline].points, &cpte = lines[ieline].points;

	}

	setUpdate(true);

	return true;
}

bool c_coast_line::addLine(list<Point2f> & line)
{
	lines.push_back(s_line());
	s_line & newLine = lines.back();
	newLine.points.resize(line.size());
	vector<Point3f> & newPoints = newLine.points;

	int i = 0;
	for (list<Point2f>::iterator itr = line.begin(); itr != line.end(); itr++)
	{
		Point3f & pt = newPoints[i];
		bihtoecef((*itr).x, (*itr).y, 0.f, pt.x, pt.y, pt.z);
		i++;
	}

	return true;
}

bool c_coast_line::distribute(Lv & ml)
{

	vector<Point3f> gl; // gravity centers of lines 
	gl.resize(lines.size());
	Point3f gls(0, 0, 0); // gravity center of this set of lines

	Point3f pmin(FLT_MAX, FLT_MAX, FLT_MAX);
	Point3f pmax(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	// calculate maximum/minimum coordinate point to restrict search area
	int nLines = lines.size();
	for (int iline = 0; iline < nLines; iline++){
		s_line & line = lines[iline];
		Point3f g(0, 0, 0);
		for (int ipt = 0; ipt < line.points.size(); ipt++){
			Point3f pt = line.points[ipt];
			g += pt;
			pmin.x = min(pmin.x, pt.x);
			pmin.y = min(pmin.y, pt.y);
			pmin.z = min(pmin.z, pt.z);
			pmax.x = max(pmax.x, pt.x);
			pmax.y = max(pmax.y, pt.y);
			pmax.z = max(pmax.z, pt.z);
		}
		gl[iline] = g * (float)(1.0 / (double)line.points.size());
		gls += g;
	}

	gls *= (float)(1.0 / (double)nLines);

	// extracting candidate nodes
	double radius = 2 * max(norm(gls - pmin), norm(gls - pmax));
	/*
	vector<Node*> candidates;
	ml.getNode(gls.x, gls.y, gls.z, radius, candidates);

	for (int iline = 0; iline < nLines; iline++){
		s_line & line = lines[iline];
		vector<Point3f> & pts = line.points;
		vector<int> nn; // nearest node
		nn.resize(pts.size());

		for (int ipt = 0; ipt < pts.size(); ipt++){
			double Dmin = DBL_MAX;
			Point3f & pt = pts[ipt];

			for (int inode = 0; inode < candidates.size(); inode++){
				Node * pNode = candidates[inode];
				double D = pNode->getDist(pt.x, pt.y, pt.z);
				if (D < Dmin){
					Dmin = D;
					nn[ipt] = inode;
				}
			}
		}

		{
			int inode = nn[0];
			s_line newLine;
			newLine.points.push_back(pts[0]);
			for (int ipt = 1; ipt < pts.size(); ipt++){
				newLine.points.push_back(pts[ipt]);
				if (nn[ipt] != inode){
					c_coast_line * pCL = dynamic_cast<c_coast_line*>(candidates[inode]->getLayer(amlc_coast_line));
					pCL->addLine(newLine);
					newLine.points.clear();
					newLine.points.push_back(pts[ipt]);
					inode = nn[ipt];
					candidates[inode]->setUpdate(true);
				}
			}
		}
	}
	*/
	return true;
}

void c_coast_line::build(list<c_layer*> & ll, double spix)
{
	for (list<c_layer*>::iterator itr = ll.begin();
		itr != ll.end(); itr++){
		c_coast_line * pCL = dynamic_cast<c_coast_line*>(*itr);
		if (pCL == NULL)
			continue;
		vector<s_line> & lls = pCL->lines;
		for (vector<s_line>::iterator itrl = lls.begin(); itrl != lls.end(); itrl++){
			addLine(*itrl);
		}
	}

	for (vector<s_line>::iterator itr = lines.begin(); itr != lines.end();){
		vector<Point3f> & pts = (*itr).points;
		for (vector<Point3f>::iterator itrp1 = pts.begin(), itrp2 = pts.begin() + 1;
			itrp2 != pts.end();){
	
			double d = norm(*itrp2 - *itrp1);
			if (d < spix){
				itrp2 = pts.erase(itrp2);
			}
			else{
				itrp1 = itrp2;
				itrp2++;
			}
		}

		if (pts.size() < 2 || ((*itr).bclosed && pts.size() < 3))
			itr = lines.erase(itr);
		else
			itr++;
	}
}

//////////////////////////////////////////////////////////////////////////////////// s_grid

bool s_grid::init(const double _step, const int _ilevel, bool becef)
{
	lv = _ilevel;
	int nLatLines = (int)(RE * PI / _step);
	latStep = PI / (double)nLatLines;

	// 3. allocate lattitude grid
	grid.resize(nLatLines + 1);
	lonStep.resize(nLatLines + 1);
	lat.resize(nLatLines + 1);
	lon.resize(nLatLines + 1);

	// 4. allocate nodes on lattitude grid
	grid[0].resize(1);			// north pole
	lon[0].resize(1);
	lat[0] = 0.5 * PI;

	grid[nLatLines].resize(1);	// south pole
	lon[nLatLines].resize(1);
	lat[nLatLines] = -0.5 * PI;

	for (int ilat = 0; ilat < nLatLines; ilat++){
		lat[ilat] = 0.5 * PI - ilat * latStep;
		double lenLatLine = 2. * PI * RE * cos(lat[ilat]);
		int lonNodes = (int)(lenLatLine / _step);
		if (lonNodes == 0) // in south and north poles
			lonNodes = 1;

		lonStep[ilat] = 2. * PI / (double)lonNodes;
		grid[ilat].resize(lonNodes);
		lon[ilat].resize(lonNodes);
		for (int ilon = 0; ilon < lonNodes; ilon++){
			lon[ilat][ilon] = ilon * lonStep[ilat];
		}
	}

	int inode = 0;
	for (int ilat = 0; ilat < lat.size(); ilat++){
		for (int ilon = 0; ilon < lon[ilat].size(); ilon++){
			grid[ilat][ilon] = inode;
			inode++;
		}
	}
	nNodes = inode;

	if (becef){
		ecef.resize(grid.size());
		for (int ilat = 0; ilat < ecef.size(); ilat++){
			ecef[ilat].resize(grid[ilat].size());
			for (int ilon = 0; ilon < ecef[ilat].size(); ilon++){
				bihtoecef(lat[ilat], lon[ilat][ilon], 0, ecef[ilat][ilon].x, ecef[ilat][ilon].y, ecef[ilat][ilon].z);
			}
		}
	}
	return true;
}

unsigned char * gen_edge(int ilv, int ilat, int ilon, vector<s_grid> & grids,
	vector<vector<int>> & un, vector<vector<unsigned char>> & tn, unsigned short & szedge)
{
	if (ilv >= grids.size() || ilv < 0)
		return NULL;

	vector<vector<int>> & grid = grids[ilv].grid;

	if (ilat >= grid.size() || ilat < 0)
		return NULL;

	if (ilon >= grid[ilat].size() || ilon < 0)
		return NULL;

	list<int> nei;
	if (ilat == 0){ // north pole
		// south connection
		for (int inlon = 0; inlon < grid[1].size(); inlon++){
			nei.push_back(grid[1][inlon]);
		}
	}
	else if (ilat == (int)(grid.size() - 1)){
		// north connection
		for (int inlon = 0; inlon < grid[grid.size() - 2].size(); inlon++){
			nei.push_back(grid[grid.size() - 2][inlon]);
		}
	}
	else{
		if (ilon == 0){
			// north connection
			nei.push_back(grid[ilat - 1][0]);
			// south connection 
			nei.push_back(grid[ilat + 1][0]);
		}
		else{
			double _lon = grids[ilv].lon[ilat][ilon];
			int inlon1, inlon2;

			// north connection
			double _lonStep = grids[ilv].lonStep[ilat - 1];
			inlon1 = (int)(_lon / _lonStep);
			inlon2 = inlon1 + 1;

			nei.push_back(grid[ilat - 1][inlon1]);
			nei.push_back(grid[ilat - 1][inlon2]);

			// south connection
			_lonStep = grids[ilv].lonStep[ilat + 1];
			inlon1 = (int)(_lon / _lonStep);
			inlon2 = inlon1 + 1;

			nei.push_back(grid[ilat + 1][inlon1]);
			nei.push_back(grid[ilat + 1][inlon2]);
		}
		{
			int inlon1, inlon2;
			inlon1 = (ilon + 1) % grid[ilat].size();
			inlon2 = (ilon + grid[ilat].size() - 1) % grid[ilat].size();
			nei.push_back(grid[ilat][inlon1]);
			nei.push_back(grid[ilat][inlon2]);
		}
	}

	// constructing edge
	int ipnode = un[ilv][grid[ilat][ilon]];
	unsigned char * pid = gen_id(ilv, ipnode, un, tn);
	vector<unsigned char> edge_tmp;
	edge_tmp.resize(nei.size() * (grids.size() + 1) + 1);
	int iedge = 0;
	for (list<int>::iterator itr = nei.begin(); itr != nei.end(); itr++){
		unsigned char * id = gen_id(ilv, *itr, un, tn);
		int iid = 0;
		if (is_parent_id(pid, id)){
			while (id[iid]){
				iid++;
			}
			edge_tmp[iedge] = id[iid - 1];
			iedge++;
		}
		else{
			while (id[iid]){
				edge_tmp[iedge] = id[iid];
				id++;
				iedge++;
			}
		}
		edge_tmp[iedge] = 0;
		iedge++;

		free_id(id);
	}
	edge_tmp[iedge] = 0;
	iedge++;

	unsigned char * edge = new unsigned char[iedge];
	for (iedge = 0; edge_tmp[iedge]; iedge++){
		edge[iedge] = edge_tmp[iedge];
	}
	edge[iedge] = 0;
	iedge++;
	edge[iedge] = 0;
	free_id(pid);

	return edge;
}

///////////////////////////////////////////////////////////////////////////////////// s_node
unsigned int s_node::szpool = 0;
s_node * s_node::pchank = NULL;
s_node * s_node::pool = NULL;
s_node * s_node::phead = NULL;
s_node * s_node::ptail = NULL;

bool s_node::init(const unsigned char * _id, const unsigned char _lv)
{
	id = new unsigned char [idlen(_id) + 1];
	copy_id(_id, id);
	lv = _lv;
	return true;
}

bool s_node::init(const unsigned int _szpool)
{
	szpool = _szpool;
	pool = new s_node[szpool];
	if (!pool)
		return false;

	pchank = pool;
	s_node * ptmp = pool;
	for (int i = 0; i < szpool-1; i++){
		ptmp->pnext = ptmp + 1;
		ptmp++;
	}
	ptmp->pnext = NULL;
	return true;
}

s_node * s_node::alloc()
{
	if (!pchank){
		s_node * pndisc = disc();
		if (!pndisc)
			return NULL;
		free(pndisc);
	}
	s_node * pret = pchank;
	pchank = pchank->pnext;

	// connecting LRU list
	ptail->pnext = pret;
	pret->pnext = NULL;
	pret->pnext = NULL;
	ptail = pret;

	return pret;
}

void s_node::free(s_node * pn)
{
	if (pn->pnext){
		s_node * pnext = pn->pnext;
		pnext->pprev = pn->pprev;
	}

	if (pn->pprev){
		s_node * pprev = pn->pprev;
		pprev->pnext = pn->pnext;
	}

	pn->release();

	pn->pnext = pchank;
	pchank = pn;
}

s_node * s_node::disc()
{
	if (!phead)
		return NULL;

	s_node * ptmp;
	for (ptmp = phead; ptmp != NULL; ptmp = ptmp->pnext)
	{
		if (ptmp->lns.size() == 0)
			break;
	}

	return ptmp;
}

unsigned int s_node::get_sz_graph_data(unsigned int sz_edge_max)
{
	unsigned int sz = sizeof(unsigned int)+sizeof(unsigned char)+
		sizeof(x)+sizeof(y)+sizeof(z)+
		sizeof(lv)+sizeof(szlns)+
		sizeof(szedge)+sz_edge_max + sizeof(unsigned int)*(int)amlc_undef * 2;
	return sz;
}

bool s_node::save_grph(ofstream & fgrp, unsigned short sz_edge_max)
{
	unsigned short lv_grph = lv - 1;
	vector<list<unsigned int>> grph;
	grph.resize(lv_grph);

	init_grph(grph);

	fgrp.write((const char*)&sz_edge_max, sizeof(sz_edge_max));
	fgrp.write((const char*)&lv_grph, sizeof(lv_grph));
	for (int ilv = 0; ilv < grph.size(); ilv++){
		list<unsigned int> & lgrph = grph[ilv];
		list<unsigned int> lgrph_idx;
		unsigned int n = 0;
		lgrph_idx.push_back(n);
		for (list<unsigned int>::iterator itr = lgrph.begin(); itr != lgrph.end(); itr++){
			unsigned int _n = *itr;
			n += _n;
			lgrph_idx.push_back(n);
			itr = lgrph.erase(itr);
			for (int i = 0; i < _n; i++, itr++);
		}
		unsigned int sz_lgrph_idx = lgrph_idx.size();
		unsigned int sz_lgrph = lgrph.size();

		fgrp.write((const char*)&sz_lgrph_idx, sizeof(sz_lgrph_idx));
		for (list<unsigned int>::iterator itr = lgrph_idx.begin(); itr != lgrph_idx.end(); itr++){
			unsigned int v = *itr;
			fgrp.write((const char*)&v, sizeof(v));
		}
		fgrp.write((const char*)&sz_lgrph, sizeof(sz_lgrph));
		for (list<unsigned int>::iterator itr = lgrph.begin(); itr != lgrph.end(); itr++){
			unsigned int v = *itr;
			fgrp.write((const char*)&v, sizeof(v));
		}
	}

	return true;
}

bool s_node::load_grph(ifstream & fgrp, vector<unsigned int *> & grph, vector<unsigned int*> & grph_idx, unsigned short & sz_edge_max)
{
	unsigned short lv_grph;
	fgrp.read((char*)&sz_edge_max, sizeof(sz_edge_max));
	fgrp.read((char*)&lv_grph, sizeof(lv_grph));
	grph.resize(lv_grph);
	grph_idx.resize(lv_grph);
	for (int ilv = 0; ilv < grph.size(); ilv++){
		unsigned int n = 0;
		fgrp.read((char*)&n, sizeof(n));
		grph_idx[ilv] = new unsigned int[n];
		fgrp.read((char*)grph_idx[ilv], sizeof(unsigned int)* n);

		fgrp.read((char*)&n, sizeof(n));
		grph[ilv] = new unsigned int[n];
		fgrp.read((char*)grph[ilv], sizeof(unsigned int)* n);
	}
	return true;
}

bool s_node::init_grph(vector<list<unsigned int>> & grph)
{
	if (lv == 1){
		return true;
	}

	grph[lv - 2].push_back(lns.size());

	for (map<unsigned char, s_node*>::iterator itr = lns.begin();
		itr != lns.end(); itr++){
		unsigned int counts = itr->second->count_child_nodes();
		grph[lv - 2].push_back(counts);
		itr->second->init_grph(grph);
	}

	return true;
}

bool s_node::save(ofstream & fgrp, unsigned short sz_edge_max)
{
	unsigned int num_child_nodes = count_child_nodes();
	fgrp.write((const char*)num_child_nodes, sizeof(num_child_nodes));
	unsigned char idl = get_last_id(id);
	fgrp.write((const char*)&idl, sizeof(idl));

	fgrp.write((const char *)&x, sizeof(x));
	fgrp.write((const char *)&y, sizeof(y));
	fgrp.write((const char *)&z, sizeof(z));
	fgrp.write((const char*)&lv, sizeof(lv));
	fgrp.write((const char*)&szlns, sizeof(szlns));
	fgrp.write((const char*)offset, sizeof(unsigned int)* (int)amlc_undef);
	fgrp.write((const char*)size, sizeof(unsigned int)* (int)amlc_undef);
	fgrp.write((const char*)szedge, sizeof(szedge));
	unsigned char * edge_tmp = new unsigned char[sz_edge_max];
	memset((void*)edge_tmp, 0, sz_edge_max);
	memcpy((void*)edge_tmp, (void*)edge, szedge);
	fgrp.write((const char*)edge, sz_edge_max);
	delete[] edge_tmp;

	for (map<unsigned char, s_node*>::iterator itr = lns.begin();
		itr != lns.end(); itr++){
		itr->second->save(fgrp, sz_edge_max);
	}

	return true;
}

bool s_node::load(ifstream & fgrp, unsigned char * _id, const unsigned char _lv)
{
	id = _id;

	fgrp.seekg(0);
	unsigned short sz_edge_max = 0;
	vector<unsigned int*> grph, grph_idx;
	load_grph(fgrp, grph, grph_idx, sz_edge_max);

	unsigned int sz_node = get_sz_graph_data(sz_edge_max);
	unsigned int num_nodes = 0;
	unsigned char cid = 0;
	unsigned int sz_node_head = sizeof(num_nodes)+sizeof(id);
	unsigned int pos = fgrp.tellg();

	if (_id[1] != 0){
		int ilv_inv = 1;
		int ilv_grph = grph.size() - 1;
		unsigned int offset = 0;
		unsigned int ppos = 0;
		while (id[ilv_inv] && ilv_grph >= 0){
			unsigned int dpos = id[ilv_inv] - 1;
			unsigned int * cl = grph[ilv_grph] + grph_idx[ilv_grph][ppos];
			for (int i = 0; i < dpos; i++){
				offset += cl[i];
			}
			ppos = dpos;
			ilv_grph--;
			ilv_inv++;
		}

		if (id[ilv_inv]){
			offset += (id[ilv_inv] - 1);
		}

		pos += offset * sz_node;
	}
	fgrp.seekg(pos);
	fgrp.read((char*)&num_nodes, sizeof(num_nodes));
	fgrp.read((char*)&cid, sizeof(cid));
	fgrp.read((char *)&x, sizeof(x));
	fgrp.read((char *)&y, sizeof(y));
	fgrp.read((char *)&z, sizeof(z));
	fgrp.read((char*)&lv, sizeof(lv));
	fgrp.read((char*)&szlns, sizeof(szlns));
	fgrp.read((char*)offset, sizeof(unsigned int)* (int)amlc_undef);
	fgrp.read((char*)size, sizeof(unsigned int)* (int)amlc_undef);
	fgrp.read((char*)szedge, sizeof(szedge));

	edge = new unsigned char[szedge];
	fgrp.read((char*)edge, szedge);

	return true;
}

unsigned int s_node::count_child_nodes()
{
	unsigned int count = lns.size() + 1;
	for (map<unsigned char, s_node*>::iterator itr = lns.begin();
		itr != lns.end(); itr++){
		count += itr->second->count_child_nodes();
	}
	return count;
}

s_node * s_node::get_child(unsigned char * _id)
{
	unsigned char * idc = _id;
	unsigned char * idp = id;
	while (*idp && *idc){
		if (*idp != *idc)
			return NULL;
		idp++;
		idc++;
	}

	if (*idp == 0 && *idc == 0)
		return this;

	s_node * pn = NULL;
	for (map<unsigned char, s_node*>::iterator itr = lns.begin(); itr != lns.end(); itr++){
		if (pn = itr->second->get_child(_id))
			break;
	}
	return pn;
}

bool s_node::append(s_node * pn)
{
	unsigned char * idc = pn->id;
	unsigned char * idp = id;
	while (*idp && *idc){
		if (*idp != *idc)
			return false;
		idp++;
		idc++;
	}

	map<unsigned char, s_node*>::iterator itr = lns.find(*idc);

	if (itr == lns.end()){
		if (idc[1] == 0){
			lns.insert(map<unsigned char, s_node*>::value_type(idc[lv + 1], pn));
			return true;
		}
		return false;
	}

	if (idc[1] == 0)
		return false;

	return itr->second->append(pn);
}

void s_node::_release()
{
	free_id(id);
	free_edge(edge);
	for (map<unsigned char, s_node*>::iterator itr = lns.begin();
		itr != lns.end(); itr++){
		itr->second->release();
		delete itr->second;
	}
	lns.clear();
}

void s_node::release()
{
	free_id(id);
	free_edge(edge);
	if (un){
		map<unsigned char, s_node*>::iterator itr = un->lns.find(get_child_id(un->id, id));
		un->lns.erase(itr);
	}

	for (map<unsigned char, s_node*>::iterator itr = lns.begin();
		itr != lns.end(); itr++){
		itr->second->release();
		free(itr->second);
	}
	lns.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
char c_map::path[1024];

c_map::c_map() :nLevels(4), minMeterPerPix(1), minStep(1000), scale(10), iscale(1.0/scale)
{
	steps.resize(nLevels);
	steps[0] = minStep;
	for (int i = 1; i < nLevels; i++)
		steps[i] = steps[i - 1] * scale;
}

c_map::c_map(const int _nLevels, const double _minMeterPerPix, const double _minStep, const double _scale, const unsigned int _nCache)
{
	init(_nLevels, _minMeterPerPix, _minStep, _scale, _nCache);
	root.id = new unsigned char;
	root.id[0] = 0;
}

c_map::~c_map()
{
	delete root.id;
}

void c_map::release()
{
}

bool c_map::init(const int _nLevels, const double _minMeterPerPix, const double _minStep, const double _scale, const unsigned int _nCache, const char * _path)
{ 
	nLevels = _nLevels;
	minStep = _minStep;
	minMeterPerPix = _minMeterPerPix;
	scale = _scale;
	iscale = 1.0 / scale;
	strncpy(path, _path, 1024);
	nCache = _nCache;
	s_node::init(nCache);

	steps.resize(nLevels);
	pixels.resize(nLevels);
	steps[0] = minStep;
	pixels[0] = minMeterPerPix;
	for (int i = 1; i < nLevels; i++){
		steps[i] = steps[i - 1] * scale;
		pixels[i] = pixels[i - 1] * scale;
	}

	char fgrph[1024];
	snprintf(fgrph, 1024, "%s/%d_%03.3f_%03.3f_%03.3f.grp", path, nLevels, minMeterPerPix, minStep, scale);
	ifstream ifile(fgrph, ios::binary);

	if (!ifile.is_open()){ // if the graph file is not found, graph initialization is done.  
		vector<s_grid> grids(nLevels);
		vector<vector<float>> dist;
		vector<vector<int>> node;
		vector<vector<unsigned char>> tnode;
		vector<vector<unsigned char>> lcnt; // lower node count
		dist.resize(nLevels);
		node.resize(nLevels);
		tnode.resize(nLevels);
		for (int ilv = 0; ilv < nLevels; ilv++)
		{
			grids[ilv].init(steps[ilv], ilv);
			dist[ilv].resize(grids[ilv].nNodes, FLT_MAX);
			node[ilv].resize(grids[ilv].nNodes, INT_MAX);
			tnode[ilv].resize(grids[ilv].nNodes, 0);
			lcnt[ilv].resize(grids[ilv].nNodes, 0);
		}
		
		// calculate upper layer connection
		// Note:
		// 1. corresponding upper layer node is in bounding two lat lines in upper layer
		// 2. corresponding upper layer node is one of the four nodes bounding around the node on the two lat lines above mentioned.
		for (int ilv = 0; ilv < nLevels - 1; ilv++)
		{
			double latSteph = grids[ilv + 1].latStep;
			vector<double> & lonSteph = grids[ilv + 1].lonStep;
			vector<vector<int>> & gridh = grids[ilv + 1].grid;
			vector<double> & lath = grids[ilv + 1].lat;
			vector<vector<double>> & lonh = grids[ilv + 1].lon;

			double latStepl = grids[ilv].latStep;
			vector<double> & lonStepl = grids[ilv].lonStep;
			vector<vector<int>> & gridl = grids[ilv].grid;
			vector<double> & latl = grids[ilv].lat;
			vector<vector<double>> & lonl = grids[ilv].lon;

			vector<float> & dmin = dist[ilv];
			vector<int> & n = node[ilv];

			for (int ilat = 0; ilat < latl.size(); ilat++){
				int ilath_max, ilath_min;
				int ilonh_max, ilonh_min;
				double lat = latl[ilat];
				for (int ilon = 0; ilon < lonl[ilat].size(); ilon++){
					double lon = lonl[ilat][ilon];
					int inl = gridl[ilat][ilon];
					Point3f pl, ph;
					bihtoecef((float)lat, (float)lon, 0., pl.x, pl.y, pl.z);

					if (ilon == 0){ // determine lattitude to be searched
						ilath_min = (int) ((0.5 * PI - lat) / latSteph);
						ilath_max = ilath_min + 1;
						ilath_min = max(ilath_min, 0);
						ilath_max = min(ilath_max, (int)lath.size());
					}

					for (int ilath = ilath_min; ilath <= ilath_max; ilath++){
						ilonh_min = (int)(lon / lonSteph[ilath]);
						ilonh_max = (ilonh_min + 1) % (int) lonh[ilat].size();
						ilonh_min = max(0, ilonh_min);

						for (int ilonh = ilonh_min; ilonh <= ilonh_max; ilonh++){
							bihtoecef((float)lath[ilath], (float)lonh[ilath][ilonh], 0, ph.x, ph.y, ph.z);
							double d = norm(ph - ph);
							if (dmin[inl] > d){
								dmin[inl] = d;
								n[inl] = gridh[ilath][ilonh];
							}
						}
					}
				}
			}
		}

		// calculate true node index
		for (int ilv = 0; ilv < nLevels - 1; ilv++){
			vector<unsigned char> & _lcnt = lcnt[ilv + 1];
			vector<unsigned char> & _tnode = tnode[ilv];
			for (int inode = 0; inode < grids[ilv].nNodes; inode++)
			{
				int unode = node[ilv][inode];
				tnode[ilv][inode] = _lcnt[unode] + 1;
				_lcnt[unode]++;
			}
		}

		for (int inode = 0; inode < grids[nLevels - 1].nNodes; inode++)
			tnode[nLevels - 1][inode] = inode + 1;

		{
			vector<double> & clat = grids[nLevels - 1].lat;
			vector<vector<double>> & clon = grids[nLevels - 1].lon;
			vector<vector<int>> & cgrid = grids[nLevels - 1].grid;
			s_node root;
			unsigned char root_id[2];
			unsigned short sz_edge_max;
			root_id[1] = 0;

			for (int ilat = 0; ilat < clat.size(); ilat++){
				for (int ilon = 0; ilon < clon[ilat].size(); ilon++){
					root_id[0] = (unsigned char)cgrid[ilat][ilon];
					root.init(root_id, (unsigned char)(nLevels - 1));
					bihtoecef(clat[ilat], clon[ilat][ilon], 0., root.x, root.y, root.z);
					root.edge = gen_edge(nLevels - 1, ilat, ilon, grids, node, tnode, root.szedge);
					sz_edge_max = root.szedge;
					list<s_node*> queue;
					queue.push_back(&root);

					while (!queue.empty()){
						s_node * pn = queue.front();
						queue.pop_front();
						
						if (pn->lv == 0)
							continue;

						vector<double> & llat = grids[pn->lv - 1].lat;
						vector<vector<double>> & llon = grids[pn->lv - 1].lon;
						vector<vector<int>> & lgrid = grids[pn->lv - 1].grid;
						for (int illat = 0; illat < llat.size(); illat++){
							for (int illon = 0; illon < llon[illat].size(); illon++){
								unsigned char * idnew = gen_id(pn->lv - 1, lgrid[illat][illon], node, tnode);
								if (is_eq_id(pn->id, idnew)){
									s_node *pnnew = new s_node;
									pnnew->init(idnew, pn->lv - 1);
									pnnew->edge = gen_edge(pn->lv - 1, illat, illon, grids, node, tnode, pnnew->szedge);
									pn->lns.insert(map<unsigned char, s_node*>::value_type(tnode[pn->lv - 1][lgrid[illat][illon]], pnnew));

									sz_edge_max = max(pnnew->szedge, sz_edge_max);
									bihtoecef(llat[illat], llon[illat][illon], 0, pnnew->x, pnnew->y, pnnew->z);
									queue.push_back(pnnew);
								}
								else
									free_id(idnew);
							}
						}
					}

					// save to the file
					cout << "saving cell " << cgrid[ilat][ilon] <<  " edge size " << sz_edge_max << endl;
					char fname[1024];
					snprintf(fname, 1024, "%s/%d.grp", path, cgrid[ilat][ilon]);
					ofstream ofile(fname);
					ofile.write((const char*)sz_edge_max, sizeof(sz_edge_max));
					root.save_grph(ofile, sz_edge_max);
					root.save(ofile, sz_edge_max);
					root.release();
					delete[] root.edge;
					root.edge = NULL;
				}
			}
		}

	}

	cgrid.init(steps[nLevels - 1], nLevels - 1, true);

	return true;
}

bool c_map::addJPGIS(const char * fname)
{
	c_coast_line * pCL = c_coast_line::convertFromJPGIS(fname);

	delete pCL;

	return true;
}

bool c_map::load_with_ecef(const float x, const float y, const float z)
{
	float lat, lon, alt;
	eceftobih(x, y, z, lat, lon, alt);
	unsigned char id = (unsigned char)cgrid.get_near_index(lat, lon, x, y, z);

	return true;
}

bool c_map::load_with_bih(const float lat, const float lon, const float alt)
{
	float x, y, z;
	bihtoecef(lat, lon, alt, x, y, z);
	unsigned char id = (unsigned char) cgrid.get_near_index(lat, lon, x, y, z);
	return true;
}

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

	float lat0 = atan2(GR, 1), lat1 = atan2(1, GR);
	float lon = atan2(GR, 1);

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
