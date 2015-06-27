#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_inspector.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_inspector.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_inspector.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <Windowsx.h>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "../util/aws_sock.h"
#include "../util/aws_thread.h"

//#include "../util/aws_cminpack.h"
#include "../util/coord.h"
#include "../util/c_ship.h"
#include "../util/c_clock.h"

//#include "../util/c_nmeadec.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"

#include "f_inspector.h"

const DWORD ModelVertex::FVF = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1;  


bool is_equal(Mat & a, Mat & b)
{
	if(a.type() != b.type())
		return false;

	if(a.rows != b.rows)
		return false;

	if(a.cols != b.cols)
		return false;

	MatIterator_<Vec3b> itra = a.begin<Vec3b>();
	MatIterator_<Vec3b> itrb = b.begin<Vec3b>();
	MatConstIterator_<Vec3b> itr_end = a.end<Vec3b>();
	for(int i = 0;itra != itr_end; itra++, itrb++, i++){
		if((*itra)[0] != (*itrb)[0] || (*itra)[1] != (*itrb)[1] || (*itra)[2] != (*itrb)[2]){
			return false;
		}
	}

	return true;
}

void mat2csv(ofstream & out, Mat & m)
{
	if(m.type() != CV_64FC1){
		return;
	}

	double * ptr = m.ptr<double>(0);
	for(int i = 0; i < m.rows; i++){
		for(int j = 0; j< m.cols; j++){
			out << *ptr;
			if(j != m.cols - 1)
				out << ",";
			ptr++;
		}
		out << endl;
	}
}

//////////////////////////////////////////////////////////////// helper function
void get_cursor_point(vector<Point2f> & pt2ds, float x, float y, int & idx, double & dist)
{
	dist = DBL_MAX;
	idx = -1;
	for(int i = 0; i < pt2ds.size(); i++)
	{
		Point2f & pt = pt2ds[i];
		double dx = pt.x -  x;
		double dy = pt.y -  y;
		double d = dx * dx + dy * dy;
		if(d < dist){
			dist = d;
			idx = i;
		}
	}
	dist = sqrt(dist);
}

void render_prjpts(s_model & mdl, vector<Point2f> & pt2dprj,
	LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
	int pttype, int state, int cur_point)
{
	// state 0: NORMAL -> 127
	// state 1: STRONG -> 255
	// state 2: GRAY -> 127,127,127
	// state 3: WHITE -> 255,255,255
	vector<s_edge> & edges = mdl.edges;

	pttype %= 12; // {sq, dia, x, cross} x {red, green, blue}
	int shape = pttype % 4;
	int val;
	if(state == 0 || state == 2){
		val = 128;
	}else if(state == 1 || state == 3){
		val = 255;
	}

	D3DCOLOR color;
	if(state < 2){
		switch(pttype / 4){
		case 0:
			color = D3DCOLOR_RGBA(val, 0, 0, 255);
			break;
		case 1:
			color = D3DCOLOR_RGBA(0, val, 0, 255);
			break;
		case 2:
			color = D3DCOLOR_RGBA(0, 0, val, 255);
			break;
		}
	}else{
		color = D3DCOLOR_RGBA(val, val, val, 255);
	}
	pline->SetAntialias(TRUE);

	pline->Begin();
	for(int iedge = 0; iedge < edges.size(); iedge++){
		D3DXVECTOR2 v[2];
		Point2f & pt1 = pt2dprj[edges[iedge].s];
		Point2f & pt2 = pt2dprj[edges[iedge].e];
		v[0] = D3DXVECTOR2(pt1.x, pt1.y);
		v[1] = D3DXVECTOR2(pt2.x, pt2.y);
		pline->Draw(v, 2, color);
	}
	D3DXVECTOR2 v[5];
	int size = 5;
	for(int ipt = 0; ipt < pt2dprj.size(); ipt++){
		Point2f & pt = pt2dprj[ipt];
		if(ipt == cur_point){
			v[0] = D3DXVECTOR2((float)(pt.x - 1.0), (float)(pt.y));
			v[1] = D3DXVECTOR2((float)(pt.x - 3.0), (float)(pt.y));
			pline->Draw(v, 2, color);
			v[0].x += 4.0;
			v[1].x += 4.0;
			pline->Draw(v, 2, color);
			v[0] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - 1.0));
			v[1] = D3DXVECTOR2((float)(pt.x), (float)(pt.y - 3.0));
			pline->Draw(v, 2, color);
			v[0].y += 4.0;
			v[1].y += 4.0;
			pline->Draw(v, 2, color);
			continue;
		}

		switch(shape){
		case 0: // square
			xsquare(pline, pt, 2, color);
			break;
		case 1: // diamond
			xdiamond(pline, pt, 2, color);
			break;
		case 2: // X
			xdiagonal(pline, pt, 2, color);
			break;
		case 3:
			xcross(pline, pt, 2, color);
			break;
		}

		if(ptxt != NULL){
			char buf[10];
			sprintf(buf, "%d", ipt);
			ptxt->render(pd3dev, buf, pt.x, (float)(pt.y + 3.0), 1.0, 0.0, EDTC_CB, color); 
		}
	}
	pline->End();
}

/////////////////////////////////////////////////////////////////// struct s_model

double s_model::get_max_dist()
{
	float dx = (float)(xmax - xmin);
	float dy = (float)(ymax - ymin);
	float dz = (float)(zmax - zmin);

	return sqrt(dz * dz + dy * dy + dx * dx);
}

void s_model::proj(vector<Point2f> & pt2ds,  Mat & cam_int, Mat & cam_dist, Mat & rvec_cam, Mat & tvec_cam, 
	Mat & rvec_obj, Mat & tvec_obj)
{
	Mat rvec, tvec;
	composeRT(rvec_cam, tvec_cam, rvec_obj, tvec_obj, rvec, tvec);
	//projectPoints(pts, rvec, tvec, cam_int, cam_dist, pt2ds);
	awsProjPts(pts, pt2ds, cam_int, cam_dist, rvec, tvec);
}

bool s_model::load(const char * afname)
{
	strcpy(fname, afname);
	FileStorage fs;
	fs.open(fname, FileStorage::READ);
	if(!fs.isOpened()){
		return false;
	}

	FileNode fn;

	fn = fs["ModelName"];
	string nameModel;
	if(fn.empty()){
		cerr << "Cannot find node ModelName." << endl;
		return false;
	}
	fn >> name;

	// decode the model name. Some names are reserved for specific models
	// If model head is "chsbd", the name should have the format "chsbd_<x>_<y>_<p>". Here <x>, <y> are non-negative integer, <p> is fixed point number.
	if(par_chsbd.parse(name.c_str(), type, pts, edges)){
		pts_deformed.resize(pts.size());
		for(int i = 0; i < pts.size(); i++)
			pts_deformed[i] = pts[i];
		calc_bounds();
		return true;
	}

	int numPoints;
	fn = fs["NumPoints"];
	if(fn.empty()){
		cerr << "Cannot find node NumPoints." << endl;
		return false;
	}
	fn >> numPoints;
	
	int numEdges; 
	fn = fs["NumEdges"];
	if(fn.empty()){
		cerr << "Cannot find node NumEdges." << endl;
		return false;
	}
	fn >> numEdges;

	int numParts;
	fn = fs["NumParts"];
	if(fn.empty()){
		cerr << "Cannot find node NumParts" << endl;
	}
	fn >> numParts;

	fn = fs["Points"];
	if(fn.empty()){
		cerr << "Cannot find node Points." << endl;
		return false;
	}

	char buf[64];
	pts.resize(numPoints);
	pts_deformed.resize(numPoints);
	for(int ip = 0; ip < numPoints; ip++){
		snprintf(buf, 63, "Point%05d", ip);
		FileNode fpt = fn[buf];
		if(fpt.empty()){
			cerr << "Cannot find node " << buf << "." << endl;
			return false;
		}
		fpt["x"] >> pts[ip].x;
		fpt["y"] >> pts[ip].y;
		fpt["z"] >> pts[ip].z;
		pts_deformed[ip] = pts[ip];
	}

	fn = fs["Edges"];
	if(fn.empty()){
		cerr << "Cannot find node Edges." << endl;
		return false;
	}

	edges.resize(numEdges);
	for(int ie =0; ie < numEdges; ie++){
		snprintf(buf, 63, "Edge%05d", ie);
		FileNode fe = fn[buf];
		if(fe.empty()){
			cerr << "Cannot find node " << buf << "." << endl;
			return false;
		}
		fe["s"] >> edges[ie].s;
		fe["e"] >> edges[ie].e;
	}

	fn = fs["Parts"];
	parts.resize(numParts);
	for(int ipart = 0; ipart < numParts; ipart++){
		snprintf(buf, 63, "Part%05d", ipart);
		FileNode fpart = fn[buf];
		if(fpart.empty()){
			cerr << "Cannot find part " << buf << "." << endl;
			return false;
		}

		FileNode fpts = fpart["pts"];
		if(fpts.empty()){
			cerr << "Cannot find points in " << buf << "." << endl;
			return false;
		}

		FileNodeIterator itr = fpts.begin();
		for(; itr != fpts.end(); itr++){
			int val;
			*itr >> val;
			parts[ipart].pts.push_back(val);
			
		}

		FileNode faxis = fpart["axis"];
		if(faxis.empty()){
			cerr << "Cannot find axis in " << buf << "." << endl;
			return false;
		}

		Point3f & axis = parts[ipart].axis;
		faxis["x"] >> axis.x;
		faxis["y"] >> axis.y;
		faxis["z"] >> axis.z;

		FileNode ftrn = fpart["trn"];
		if(ftrn.empty()){
			cerr << "Cannot find trn in " << buf << "." << endl;
			return false;
		}
		ftrn >> parts[ipart].trn;

		FileNode frot = fpart["rot"];
		if(frot.empty()){
			cerr << "Cannot find rot in " << buf << "." << endl;
			return false;
		}
		frot >> parts[ipart].rot;

		if(parts[ipart].rot){
			FileNode forg = fpart["org"];
			if(forg.empty()){
				cerr << "Cannot find org in " << buf << "." << endl;
				return false;
			}
			forg >> parts[ipart].org;
		}
	}

	calc_bounds();
	return true;
}

void s_model::calc_bounds()
{
	xmin = ymin = zmin = FLT_MAX;
	xmax = ymax = zmax = -FLT_MAX;
	for(int i = 0; i < pts.size(); i++){
		xmin = min(xmin, pts[i].x);
		xmax = max(xmax, pts[i].x);

		ymin = min(ymin, pts[i].y);
		ymax = max(ymax, pts[i].y);

		zmin = min(zmin, pts[i].z);
		zmax = max(ymax, pts[i].z);
	}
}

s_obj * s_model::detect(Mat & img, s_obj * pobj)
{
	if(type == EMT_CHSBD){
		if(!pobj)
			pobj = new s_obj;
		pobj->pmdl = this;

		if(par_chsbd.detect(img, pobj->pt2d)){
			int len_name = (int) strlen(name.c_str()) + 4 + 1;
			pobj->name = new char[len_name];
			if(pobj->name == NULL){
				delete pobj;
				return NULL;
			}
			snprintf(pobj->name, len_name, "%s_%03d", name.c_str(), ref);
			ref++;
			pobj->visible.resize(pobj->pt2d.size(), true);
			return pobj;
		}
		delete pobj;
	}
	return NULL;
}

////////// s_model's sub-struct
bool s_model::s_chsbd::parse(const char * name, e_model_type & type, 
	vector<Point3f> & pts, vector<s_edge> & edges)
{
	char * tok[4];
	char buf[128];
	tok[0] = buf;
	int i, itok;
	for(i = 0, itok = 0; name[i]; i++){
		if(name[i] == '_'){
			buf[i] = '\0';
			itok++;
			tok[itok] = buf + i + 1;
		}else{
			buf[i] = name[i];
		}
	}
	buf[i] = '\0';
	if(itok != 3)
		return false;
	if(strcmp(tok[0], "chsbd") != 0)
		return false;

	w = atoi(tok[1]);
	h = atoi(tok[2]);
	p = (float) atof(tok[3]);

	int numpts = w * h;
	int numedges = w * (h - 1) + (w - 1) * h;
	pts.resize(numpts);
	edges.resize(numedges);
	i = 0;
	int j = 0;
	for (int y = 0; y < h; y++){
		for(int x = 0; x < w; x++){
			Point3f & pt = pts[i];
			pt.x = (float)(x * p);
			pt.y = (float)(y * p);
			pt.z = 0;
			if(x < w - 1){
				s_edge & e = edges[j];
				e.s = i;
				e.e = i + 1;
				j++;
			}
			if(y < h - 1){
				s_edge & e = edges[j];
				e.s = i;
				e.e = i + w;
				j++;
			}
			i++;
		}
	}
	type = EMT_CHSBD;
	return true;
}

bool s_model::s_chsbd::detect(Mat & img, vector<Point2f> & pt2d)
{
	if(!findChessboardCorners(img, Size(w, h), pt2d))
		return false;
	Size winSize = Size( 5, 5 );
	Size zeroZone = Size( -1, -1 );
	TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
	cornerSubPix(img, pt2d, winSize, zeroZone, criteria); 
	return true;
}

//////////////////////////////////////////////////////////////////////////// s_obj member
bool s_obj::init(s_model * apmdl, long long at, const Mat & camint, const Mat & camdist,
	const double width, const double height)
{
	pmdl = apmdl;	
	t = at;
	int len_name = (int) strlen(apmdl->name.c_str()) + 4 /* under bar and three digit */ + 1 /* termination character */;

	name = new char[len_name];
	if(name == NULL)
		return false;
	snprintf(name, len_name, "%s_%03d", apmdl->name.c_str(), apmdl->ref);

	double xsize = pmdl->get_xsize();
	double ysize = pmdl->get_ysize();
	double zsize = pmdl->get_zsize();

	// To fit inside the window, z should be determined 
	// to satisfy both fpix * xsize / z < width and  fpix * ysize / z < height
	// This means z > fpix * xsize / width and z > fpix * ysize / height.
	// Actually the z should be
	double fpix_x = camint.at<double>(0, 0);
	double fpix_y = camint.at<double>(1, 1);
	double dist_z = max(fpix_x * xsize / width, fpix_y * ysize / height);

	// now rvec is zero and tvec is (0, 0, dist_z)
	rvec = Mat::zeros(3, 1, CV_64FC1);
	tvec = Mat::zeros(3, 1, CV_64FC1);
	tvec.at<double>(2, 0) = dist_z;

	pt2d.resize(pmdl->pts.size());
	pt2dprj.resize(pmdl->pts.size());
	visible.resize(pmdl->pts.size(), false);

	int num_parts = (int) pmdl->parts.size();
	dpart.resize(num_parts, 0.0);

	apmdl->ref++;
	return true;
}

bool s_obj::init(const s_obj & obj)
{
	pmdl = obj.pmdl;
	t = obj.t;

	if(name != NULL)
		delete [] name;
	name = NULL;
	name = new char[strlen(obj.name) + 1];
	if(name == NULL)
		return false;
	strcpy(name, obj.name);

	pt2d = obj.pt2d;
	pt2dprj = obj.pt2dprj;
	visible = obj.visible;
	is_attitude_fixed = obj.is_attitude_fixed;
	obj.tvec.copyTo(tvec);
	obj.rvec.copyTo(rvec);
	obj.jacobian.copyTo(jacobian);
	obj.hessian.copyTo(hessian);
	obj.dp.copyTo(dp);
	obj.err.copyTo(err);
	ssd = obj.ssd;
	match_count = obj.match_count;

	int num_parts = (int) pmdl->parts.size();
	dpart.resize(num_parts, 0.0);

	return true;
}

// this method calculate positions of the part's points in the model coordinate
// deformation parameter dparts should be calculated before calling this.
void s_obj::calc_part_deformation()
{
	vector<s_part> & parts = pmdl->parts;
	vector<Point3f> & pt3d = pmdl->pts;
	vector<Point3f> & pt3d_deformed = pmdl->pts_deformed;
	int num_parts = (int) parts.size();

	for(int ipart = 0;ipart < num_parts;ipart++){
		s_part & part = parts[ipart];
		vector<int> & ptis = parts[ipart].pts;
		Point3f & axis = part.axis;
		double d = dpart[ipart];
		if(part.trn){
			for(int ipt = 0; ipt < ptis.size(); ipt++){
				int pti = ptis[ipt];
				pt3d_deformed[pti] = d * axis;
				pt3d_deformed[pti] += pt3d[pti];
			}
		}else if(part.rot){
			Point3f & org = pt3d[part.org];

			// calcurating rotatin matrix 
			double * ptr;
			Mat rvec = Mat(3, 1, CV_64FC1);
			ptr = rvec.ptr<double>();
			ptr[0] = d * axis.x;
			ptr[1] = d * axis.y;
			ptr[2] = d * axis.z;
			Mat R;
			Rodrigues(rvec, R);
			ptr = R.ptr<double>();

			for(int ipt = 0; ipt < ptis.size(); ipt++){
				int pti = ptis[ipt];
				Point3f & pt = pt3d_deformed[pti];
				// 1. subtract origin
				pt = pt3d[pti];
				pt -= org;

				// 2. rotate
				double rx, ry, rz;
				rx = ptr[0] * pt.x + ptr[1] * pt.y + ptr[2] * pt.z;
				ry = ptr[3] * pt.x + ptr[4] * pt.y + ptr[5] * pt.z;
				rz = ptr[6] * pt.x + ptr[7] * pt.y + ptr[8] * pt.z;

				// 3. add origin
				pt.x = (float)(rx + org.x);
				pt.y = (float)(ry + org.y);
				pt.z = (float)(rz + org.z);
			}
		}
	}
}

bool s_obj::load(FileNode & fnobj, vector<s_model*> & mdls)
{
	FileNode fn;

	fn = fnobj["ObjName"];
	if(fn.empty())
		return false;

	string str;
	fn >> str;

	if(name != NULL)
		delete [] name;
	name = NULL;
	name = new char[str.length() + 1];

	if(name == NULL)
		return false;
	strcpy(name, str.c_str());

	fn = fnobj["Model"];
	if(fn.empty())
		return false;

	fn >> str;
	pmdl = NULL;
	for(int i = 0; i < mdls.size(); i++){
		if(mdls[i]->fname == str){
			pmdl = mdls[i];
			break;
		}
	}
	if(pmdl == NULL){
		mdls.push_back(new s_model());
		vector<s_model*>::iterator itr = mdls.end() - 1;
		if(!(*itr)->load(str.c_str())){
			mdls.pop_back();
			cerr << "Failed to load model " << str << endl;
			return false;
		}
		pmdl = (*itr);
	}

	// allocating memories
	int num_points = (int) pmdl->pts.size();
	pt2d.resize(num_points, Point2f(0, 0));
	pt2dprj.resize(num_points, Point2f(0, 0));
	visible.resize(num_points, false);

	fn = fnobj["rvec"];
	if(fn.empty()){
		return false;
	}
	fn >> rvec;

	fn = fnobj["tvec"];
	if(fn.empty()){
		return false;
	}
	fn >> tvec;

	fn = fnobj["Matched"];
	if(fn.empty())
		return false;
	FileNodeIterator itr = fn.begin();
	for(int i = 0; i < num_points && itr != fn.end(); i++, itr++){
		if((*itr).empty())
			return false;
		int b;
		(*itr) >> b;
		visible[i] = b;
	}

	fn = fnobj["Points"];
	if(fn.empty())
		return false;
	itr = fn.begin();
	for(int i = 0; i < num_points && itr != fn.end(); i++, itr++){
		if((*itr).empty())
			return false;
		(*itr) >> pt2d[i];
	}

	int num_parts = (int) pmdl->parts.size();
	dpart.resize(num_parts, 0.0);
	fn = fnobj["Deformation"];
	if(fn.empty()){
		return true;
	}
	itr = fn.begin();
	for(int ipart = 0; ipart < num_parts && itr != fn.end(); ipart++, itr++){
		if((*itr).empty())
			return false;
		(*itr) >> dpart[ipart];
	}

	return true;
}

bool s_obj::save(FileStorage & fs)
{
	// time, model file
	fs << "ObjName" << name;
	fs << "Model" << pmdl->fname;
	fs << "rvec" << rvec;
	fs << "tvec" << tvec;

	// piont correspondance
	fs << "Matched" << "[";
	for(int i = 0; i < visible.size(); i++){
		fs << visible[i];
	}
	fs << "]";

	fs << "Points" << "[";
	for(int i = 0; i < pt2d.size(); i++){
		fs << pt2d[i];
	}
	fs << "]";

	fs << "Deformation" << "[";
	for(int ipart = 0; ipart < pmdl->parts.size(); ipart++){
		fs << dpart[ipart];
	}
	fs << "]";
	return true;
}


int s_obj::calc_num_matched_points(){
	match_count = 0;
	for(int i = 0; i < visible.size(); i++)
		if(visible[i])
			match_count++;
	return match_count;
}

double s_obj::calc_ssd(){
	ssd = 0;
	err = Mat::zeros((int) visible.size() * 2, 1, CV_64FC1);
	double * ptr = err.ptr<double>(0);
	for(int i = 0; i < visible.size(); i++, ptr+=2){
		//			ptr[0] = (double) visible[i] * (pt2d[i].x - pt2dprj[i].x);
		//			ptr[1] = (double) visible[i] * (pt2d[i].y - pt2dprj[i].y);
		ptr[0] = (double) visible[i] * (pt2dprj[i].x - pt2d[i].x);
		ptr[1] = (double) visible[i] * (pt2dprj[i].y - pt2d[i].y);
		ssd += (ptr[0] * ptr[0] + ptr[1] * ptr[1]);
	}
	return ssd;
}

void s_obj::get_bb_pt2d(Rect & bb)
{
	float xmin, ymin, xmax, ymax;
	xmin = ymin = FLT_MAX;
	xmax = ymax = 0;
	bb = Rect(0, 0, 0, 0);
	for(int i = 0; i < visible.size(); i++){
		if(!visible[i])
			continue;
		xmin = min(pt2d[i].x, xmin);
		xmax = max(pt2d[i].x, xmax);
		ymin = min(pt2d[i].y, ymin);
		ymax = max(pt2d[i].y, ymax);
	}
	bb.x = (int) xmin;
	bb.y = (int) ymin;
	bb.width = (int)(xmax - xmin);
	bb.height = (int)(ymax - ymin);
}


void s_obj::proj(Mat & camint, Mat & camdist, bool bjacobian, bool fix_aspect_ratio)
{
	// parts transformation
	calc_part_deformation();

	if(bjacobian){
		projectPoints(pmdl->pts_deformed, rvec, tvec, camint, camdist, pt2dprj, jacobian,
			fix_aspect_ratio ? 1.0 : 0.0);
		// jacobian 
		// rows: 2N 
		// cols: r, t, f, c, k
		if(!test_awsProjPtsj(camint, camdist, rvec, tvec, pmdl->pts_deformed,
			visible, jacobian, fix_aspect_ratio ? 1.0 : 0.0)){
			cerr << "Jacobian calculated by awsProjPts may be wrong." << endl;
		}

		calc_ssd();
		mulTransposed(jacobian, hessian, true);
		jterr = jacobian.t() * err;
	}else{
	//	projectPoints(pmdl->pts_deformed, rvec, tvec, camint, camdist, pt2dprj);
		awsProjPts(pmdl->pts_deformed, pt2dprj, camint, camdist, rvec, tvec);
		calc_ssd();
	}

	/*
	cout << "Camint = " << endl << camint << endl;
	cout << "Camdist = " << endl << camdist << endl;
	cout << "rvec = " << endl << rvec << endl;
	cout << "tvec = " << endl << tvec << endl;
	cout << "Projecting " << name << endl;
	cout << "J=" << jacobian << endl;
	*/	
}

void s_obj::render(LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
	int pttype, int state, int cur_point)
{
	render_prjpts(*pmdl, pt2dprj, pd3dev, ptxt, pline, pttype, state, cur_point);
}

void s_obj::render(Mat & img)
{
	for (int ipt = 0; ipt < pt2d.size(); ipt++){
		if (visible[ipt])
			circle(img, pt2d[ipt], 4, CV_RGB(255, 0, 0));
	}

	for (int ipt = 0; ipt < pt2d.size(); ipt++){
		circle(img, pt2dprj[ipt], 4, CV_RGB(0,255, 0));
	}

	for (int iedge = 0; iedge < pmdl->edges.size(); iedge++){
		s_edge & edge = pmdl->edges[iedge];
		line(img, pt2dprj[edge.s], pt2dprj[edge.e], CV_RGB(255, 255, 0));
	}
}

void s_obj::render_axis(Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
	LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline, int axis)
{
	float fac = (float) pmdl->get_max_dist();

	vector<Point3f> p3d(4, Point3f(0.f, 0.f, 0.f));
	vector<Point2f> p2d;

	p3d[1].x = fac; // pt3d[1] is x axis
	p3d[2].y = fac; // pt3d[2] is y axis
	p3d[3].z = fac; // pt3d[3] is z axis

//	projectPoints(p3d, rvec_cam, tvec_cam, cam_int, cam_dist, p2d);
	awsProjPts(p3d, p2d, cam_int, cam_dist, rvec_cam, tvec_cam);
	pline->Begin();
	D3DXVECTOR2 v[2];
	D3DCOLOR color;
	int arpha = (axis < 0 ? 255 : 128);

	// origin of the axis
	v[0] = D3DXVECTOR2(p2d[0].x, p2d[0].y);

	// x axis
	v[1] = D3DXVECTOR2(p2d[1].x, p2d[1].y);
	if(axis == 0)
		color = D3DCOLOR_RGBA(255, 0, 0, 255);
	else
		color = D3DCOLOR_RGBA(255, 0, 0, arpha);
	pline->Draw(v, 2, color);

	// y axis
	v[1] = D3DXVECTOR2(p2d[2].x, p2d[2].y);
	if(axis == 1)
		color = D3DCOLOR_RGBA(0, 255, 0, 255);
	else
		color = D3DCOLOR_RGBA(0, 255, 0, arpha);
	pline->Draw(v, 2, color);

	// z axis
	v[1] = D3DXVECTOR2(p2d[3].x, p2d[3].y);
	if(axis == 2)
		color = D3DCOLOR_RGBA(0, 0, 255, 255);
	else
		color = D3DCOLOR_RGBA(0, 0, 255, arpha);
	pline->Draw(v, 2, color);

	pline->End();
}

void s_obj::render_vector(Point3f & vec, 
	Mat & rvec_cam, Mat & tvec_cam, Mat & cam_int, Mat & cam_dist,
	LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline)
{
	// calculating scale factor
	double fac = pmdl->get_max_dist() / sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	vec *= fac;

	// projection 
	vector<Point3f> vec3d(2);
	vector<Point2f> vec2d;
	vec3d[0] = Point3f(0, 0, 0);
	vec3d[1] = vec;
	Mat rvec_comp, tvec_comp;
	composeRT(rvec_cam, tvec_cam, rvec, tvec, rvec_comp, tvec_comp);
	//projectPoints(vec3d, rvec_comp, tvec_comp, cam_int, cam_dist, vec2d);
	awsProjPts(vec3d, vec2d, cam_int, cam_dist, rvec_comp, tvec_comp);
	pline->Begin();
	D3DCOLOR color = D3DCOLOR_RGBA(255, 255, 255, 255);

	D3DXVECTOR2 v[2];
	v[0] = D3DXVECTOR2(vec2d[0].x, vec2d[0].y);
	v[1] = D3DXVECTOR2(vec2d[1].x, vec2d[1].y);
	pline->Draw(v, 2, color);

	// draw cross at the origin 
	xcross(pline, vec2d[0], 3.0, color);
	pline->End();
}

void s_obj::sample_tmpl(Mat & img, Size & sz)
{
	int ox = sz.width >> 1;
	int oy = sz.height >> 1;
	Rect roi;
	roi.width = sz.width;
	roi.height = sz.height;

	ptx_tmpl.resize(pt2d.size());

	for(int i = 0; i < pt2d.size(); i++){
		if(!visible[i])
			continue;
		Point2f & pt = pt2d[i];

		roi.x = (int)(pt.x + 0.5) - ox;
		roi.y = (int)(pt.y + 0.5) - oy;
		if(roi.x < 0 || roi.y < 0 || roi.x + roi.width >= img.cols || roi.y + roi.height >= img.rows){
			visible[i] = false;
			continue;
		}

		img(roi).copyTo(ptx_tmpl[i]);
	}
}

//////////////////////////////////////////////////////////////////// s_frame
s_frame * s_frame::pool = NULL;

bool s_frame::init(const long long atfrm, 
	s_frame * pfobj0, s_frame * pfobj1, 
	vector<Mat> & impyr, c_imgalign * pia, int & miss_tracks)
{
	tfrm = atfrm;
	vector<s_obj*> & objs_prev = pfobj0->objs;
	objs.resize(objs_prev.size());
	for(int i = 0; i < objs.size(); i++){
		s_obj * pobj = new s_obj();
		if(!pobj->init(*objs_prev[i])){
			delete pobj;
			i--;
			for(; i >= 0; i--)
				delete objs[i];
			return false;
		}
		objs[i] = pobj;
	}
	pfobj0->camint.copyTo(camint);
	pfobj0->camdist.copyTo(camdist);

	// tracking points
	vector<Mat> tmplpyr;
	Rect roi;
	Mat Warp, I;
	I = Mat::eye(3, 3, CV_64FC1);
	miss_tracks = 0;
	//ia.set_wt(EWT_RGD);
	for(int iobj = 0; iobj < objs.size(); iobj++){
		vector<Point2f> & pt2d = objs[iobj]->pt2d;
		vector<Point2f> & pt2d_prev = objs_prev[iobj]->pt2d;
		vector<int> & visible = objs[iobj]->visible;
		vector<int> & visible_prev = objs_prev[iobj]->visible;
		vector<Mat> & tmpl = objs_prev[iobj]->ptx_tmpl;
		for(int ipt = 0; ipt < pt2d.size(); ipt++){
			if(!visible_prev[ipt]){
				visible[ipt] = 0;
				continue;
			}
			
			Point2f & pt_prev = pt2d_prev[ipt];

			//char buf[128];
			Point2f & pt = pt2d[ipt];
			if(pfobj1){
				Point2f & pt_next = pfobj1->objs[iobj]->pt2d[ipt];
				long long tprev, tnext;
				tprev = pfobj0->tfrm;
				tnext = pfobj1->tfrm;
				double fac = abs((double) (tfrm - tprev) / (double)(tnext - tprev));

				// linear interpolation
				pt.x = (float)(fac * pt_prev.x + (1.0 - fac) * pt_next.x);
				pt.y = (float)(fac * pt_prev.y + (1.0 - fac) * pt_next.y);
			}

			if(pia && tmpl.size() != 0){
				//snprintf(buf, 128, "t%d.png", ipt);
				//imwrite(buf, tmpl[ipt]);
				buildPyramid(tmpl[ipt], tmplpyr, (int) impyr.size() - 1);
				roi.width = tmpl[ipt].cols;
				roi.height = tmpl[ipt].rows;
				roi.x = (int)(pt_prev.x + 0.5) - (roi.width >> 1);
				roi.y = (int)(pt_prev.y + 0.5) - (roi.height >> 1);
				//snprintf(buf, 128, "r%d.png", ipt);
				//imwrite(buf, impyr[0](roi));
				//cout << roi << endl;
				Warp = pia->calc_warp(tmplpyr, impyr, roi, I);
				if(!pia->is_conv()){
					cerr << "Tracker does not converged." << endl;
					miss_tracks++;
				}
				afn(Warp, pt_prev, pt);
			}
			
			//cout << pt_prev << endl;
			//cout << pt << endl;
		}
		tmpl.clear();
	}
	return true;
}


bool s_frame::save(const char * aname)
{
	char buf[1024];
	snprintf(buf, 1024, "%s_%lld.yml", aname, tfrm);
	FileStorage fs(buf, FileStorage::WRITE);
	if(!fs.isOpened())
		return false;
	// save number of objects
	fs << "NumObjs" << (int) objs.size();

	// save camera parameters
	fs << "CamInt" << camint;
	fs << "CamDist" << camdist;

	// save objects
	fs << "Objs" << "{";
	for(int iobj = 0; iobj < objs.size(); iobj++){
		snprintf(buf, 1024, "Obj%03d", iobj);
		fs << buf << "{";
		objs[iobj]->save(fs);
		fs << "}";
	}
	fs << "}";

	fs << "KeyFrame" << kfrm;
	if(kfrm && !img.empty()){
		snprintf(buf, 1024, "%s_%lld.png", aname, tfrm);
		imwrite(buf, img);
		fs << "ImageFile" << buf;
	}
	return true;
}

bool s_frame::load(const char * aname, long long atfrm, vector<s_model*> & mdls)
{
	tfrm = atfrm;

	char buf[1024];
	snprintf(buf, 1024, "%s_%lld.yml", aname, tfrm);
	FileStorage fs(buf, FileStorage::READ);
	if(!fs.isOpened())
		return false;

	FileNode fn;
	int num_objs;
	// load number of objects
	fn = fs["NumObjs"];
	if(fn.empty())
		return false;
	fn >> num_objs;
	// load camera parameters
	fn = fs["CamInt"];
	if(fn.empty())
		return false;
	fn >> camint;

	fn = fs["CamDist"];
	if(fn.empty())
		return false;
	fn >> camdist;

	// allocate objects
	objs.resize(num_objs);

	// load objects
	fn = fs["Objs"];
	if(fn.empty())
		return false;
	
	FileNode fnobj;
	for(int iobj = 0; iobj < objs.size(); iobj++){
		snprintf(buf, 1024, "Obj%03d", iobj);
		fnobj = fn[buf];
		s_obj * pobj = new s_obj;
		objs[iobj] = pobj;
		if(!pobj->load(fnobj, mdls)){
			for(; iobj >= 0; iobj--){
				delete objs[iobj];
			}
			objs.clear();
			return false;
		}
	}

	fn = fs["KeyFrame"];
	if(!fn.empty()){
		fn >> kfrm;
	}

	fn = fs["ImageFile"];
	string imgpath;
	if(kfrm && !fn.empty()){
		fn >> imgpath;
		img = imread(imgpath);
		if(img.empty())
			return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////// class f_inspector
const char * f_inspector::m_str_op[VIEW3D+1]
	= {"model", "obj", "part", "point", "camera", "camtbl", "estimate", "frame", "v3d"};

const char * f_inspector::m_str_sop[SOP_AWSCMD+1]
	= {"null", "save", "load", "guess", "det", "ins", "del", "kf", "icp", "awscmd"};

const char * f_inspector::m_axis_str[AX_Z + 1] = {
	"x", "y", "z"
};

const char * f_inspector::m_str_campar[ECP_K6 + 1] = {
	"fx", "fy", "cx", "cy", "k1", "k2", "c1", "c2", "k3", "k4", "k5", "k6"
};

const char * f_inspector::m_str_emd[EMD_SEL + 1] = {
	"stop", "full", "rt", "rtfull", "sel"
};

const char * f_inspector::m_str_view[EV_FREE + 1] = {
	"cam", "objx", "objy", "objz", "free"
};

f_inspector::f_inspector(const char * name):f_ds_window(name), m_pin(NULL), m_timg(-1),
	m_sh(1.0), m_sv(1.0), m_btrack_obj(true), m_sz_vtx_smpl(128, 128), m_miss_tracks(0), m_wt(EWT_TRN),
	m_lvpyr(2), m_sig_gb(3.0),m_bauto_load_fobj(false), m_bauto_save_fobj(false),
	m_bundistort(false), m_bcam_tbl_loaded(false), m_cur_camtbl(-1),
	m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_campar(false), m_bcalib_fix_focus(false), m_bcalib_fix_principal_point(false),
	m_bcalib_fix_aspect_ratio(false),
	m_bcalib_zero_tangent_dist(true), m_bcalib_fix_k1(true), m_bcalib_fix_k2(true), m_bcalib_fix_k3(true),
	m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false),
	/*m_cur_frm(-1),*/ m_int_cyc_kfrm(30), m_cur_campar(0),  m_cur_model(-1), m_depth_min(0.5), m_depth_max(1.5), 
	m_num_cur_objs(0), m_cur_obj(-1), m_cur_part(-1), m_cur_point(-1), 
	m_op(OBJ), m_bkfrm(false), m_sop(SOP_NULL), m_mm(MM_NORMAL), m_axis(AX_X), m_adj_pow(0), m_adj_step(1.0),
	m_main_offset(0, 0), m_main_scale(1.0), m_theta_z_mdl(0.0), m_dist_mdl(0.0), m_cam_erep(-1.0),
	m_eest(EES_CONV), m_num_max_itrs(30), m_num_max_frms_used(100), m_err_range(3),
	m_ev(EV_CAM), m_int_kfrms(SEC), m_num_kfrms(100), m_cur_kfrm(-1), m_sel_kfrm(-1), m_pfrm(NULL), m_pfrm_int(NULL),
	m_rat_z(1.0)
{
	m_name_obj[0] = '\0';
	m_fname_model[0] = '\0';
	m_fname_campar[0] = '\0';
	m_fname_campar_tbl[0] = '\0';

	m_cam_int = Mat::eye(3, 3, CV_64FC1);
	m_cam_dist = Mat::zeros(1, 8, CV_64FC1);
	m_rvec_cam = Mat::zeros(3, 1, CV_64FC1);
	m_tvec_cam = Mat::zeros(3, 1, CV_64FC1);

	register_fpar("fmodel", m_fname_model, 1024, "File path of 3D model frame.");
	register_fpar("fcp", m_fname_campar, 1024, "File path of camera parameter.");
	register_fpar("op", (int*)&m_op, VIEW3D+1, m_str_op,"Operation ");
	register_fpar("sop", (int*)&m_sop, SOP_AWSCMD+1, m_str_sop, "Sub operation.");
	register_fpar("asave", &m_bauto_save_fobj, "Automatically save frame object.");
	register_fpar("aload", &m_bauto_load_fobj, "Automatically load frame object.");
	register_fpar("sh", &m_sh, "Horizontal scaling value. Image size is multiplied by the value. (default 1.0)");
	register_fpar("sv", &m_sv, "Vertical scaling value. Image size is multiplied by the value. (default 1.0)");

	// point tracking parameter
	register_fpar("wt", (int*)&m_wt, EWT_UNKNOWN+1, wtname, "Warp type of the point tracker.");
	register_fpar("lvpyr", &m_lvpyr, "Level of the pyramid for point tracking.");
	register_fpar("vgb", &m_sig_gb, "Variance for gaussian blur.");
	register_fpar("szptx", &m_sz_vtx_smpl.width, "Horizontal size of a point template.");
	register_fpar("szpty", &m_sz_vtx_smpl.height, "Vertical size of a point template.");
	register_fpar("miss_tracks", &m_miss_tracks, "Miss tracking counts of the points.");
	register_fpar("track", &m_btrack_obj, "Flag enables inter frame object tracking.");

	// Key frame related parameter
	register_fpar("ldkf", &m_bald_kfrms, "Flag auto loading key frames");
	register_fpar("svkf", &m_basv_kfrms, "Flag auto saving key frames");
	register_fpar("kfrms", &m_num_kfrms, "Number of key frames cached in the memory.");
	register_fpar("cyckf", &m_int_cyc_kfrm, "Key frame interval in cycle counts.");
	register_fpar("intkf", &m_int_kfrms, "Key frame interval in 10e-7 second, subjects to cyckf");

	// model related parameters
	register_fpar("mdl", &m_cur_model, "Model with specified index is selected.");
	register_fpar("pt", &m_cur_point, "Model point of specified index is selected.");
	register_fpar("part", &m_cur_part, "Index of the currentlly selected object part.");

	// object related parameter
	register_fpar("num_objs", &m_num_cur_objs, "Number of current objects in the frame");

	// camera calibration and parameter
	register_fpar("max_itrs", &m_num_max_itrs, "Number of maximum iteration of Gauss-Newton Iteration.");
	register_fpar("max_frms", &m_num_max_frms_used, "Number of maximum frames used for the Gauss-Newton optimization.");
	register_fpar("err_rng", &m_err_range, "Error range to discard frames used for optimization. Specified as number of standard deviation.");

	// camera parameter
	register_fpar("fx", m_cam_int.ptr<double>(0, 0), "x-directional focal length in milimeter");
	register_fpar("fy", m_cam_int.ptr<double>(1, 1), "y-directional focal length in milimeter");
	register_fpar("cx", m_cam_int.ptr<double>(0, 2), "x-coordinate of camera center in pixel.");
	register_fpar("cy", m_cam_int.ptr<double>(1, 2), "y-coordinate of camera center in pixel.");
	register_fpar("px", m_cam_dist.ptr<double>(0) + 2, "x coefficient of tangential distortion.");
	register_fpar("py", m_cam_dist.ptr<double>(0) + 3, "y coefficient of tangential distortion.");
	register_fpar("k1", m_cam_dist.ptr<double>(0), "Radial distortion coefficient k1.");
	register_fpar("k2", m_cam_dist.ptr<double>(0) + 1, "Radial distortion coefficient k2.");
	register_fpar("k3", m_cam_dist.ptr<double>(0) + 4, "Radial distortion coefficient k3.");
	register_fpar("k4", m_cam_dist.ptr<double>(0) + 5, "Radial distortion coefficient k4.");
	register_fpar("k5", m_cam_dist.ptr<double>(0) + 6, "Radial distortion coefficient k5.");
	register_fpar("k6", m_cam_dist.ptr<double>(0) + 7, "Radial distortion coefficient k6.");
	register_fpar("campar", &m_cur_campar, ECP_K6+1, m_str_campar, "Current camera parameter selected.");
	register_fpar("erep", &m_erep, "Reprojection error.");

	register_fpar("dmin", &m_depth_min, "Minimum Scene Depth in meter");
	register_fpar("dmax", &m_depth_max, "Maximum Scene Depth in meter");

	register_fpar("use_intrinsic_guess", &m_bcalib_use_intrinsic_guess, "Use intrinsic guess.");
	register_fpar("fix_campar", &m_bcalib_fix_campar, "Fix camera parameters");
	register_fpar("fix_focus", &m_bcalib_fix_focus, "Fix camera's focal length");
	register_fpar("fix_principal_point", &m_bcalib_fix_principal_point, "Fix camera center as specified (cx, cy)");
	register_fpar("fix_aspect_ratio", &m_bcalib_fix_aspect_ratio, "Fix aspect ratio as specified fx/fy. Only fy is optimized.");
	register_fpar("zero_tangent_dist", &m_bcalib_zero_tangent_dist, "Zeroify tangential distortion (px, py)");
	register_fpar("fix_k1", &m_bcalib_fix_k1, "Fix k1 as specified.");
	register_fpar("fix_k2", &m_bcalib_fix_k2, "Fix k2 as specified.");
	register_fpar("fix_k3", &m_bcalib_fix_k3, "Fix k3 as specified.");
	register_fpar("fix_k4", &m_bcalib_fix_k4, "Fix k4 as specified.");
	register_fpar("fix_k5", &m_bcalib_fix_k5, "Fix k5 as specified.");
	register_fpar("fix_k6", &m_bcalib_fix_k6, "Fix k6 as specified.");
	register_fpar("rational_model", &m_bcalib_rational_model, "Enable rational model (k4, k5, k6)");
	register_fpar("emd", (int*)&m_emd, EMD_SEL + 1, m_str_emd, "Estimation mode.");
	register_fpar("undist", &m_bundistort, "Undistort source image according to the camera parameter.");

	register_fpar("fcptbl", m_fname_campar_tbl, 1024, "File path of table of the camera parameters with multiple magnifications.");
	register_fpar("cptbl", &m_cur_camtbl, "Current camera parameter index in thetable.");

	// scene view paramters
	register_fpar("view", (int*)&m_ev, EV_FREE + 1, m_str_view, "Camera position in scene view.");

	// object/camera manipulation
	register_fpar("axis", (int*)&m_axis, (int)AX_Z + 1, m_axis_str, "Axis for rotation and translation. {x, y, z}");
	register_fpar("sadj", &m_adj_step, "Parameter adjustment step for the camera and objects.");
}

f_inspector::~f_inspector()
{
/*	for (int ifrm = 0; ifrm < m_fobjs.size(); ifrm++)
		delete m_fobjs[ifrm];*/
	for (int imdl = 0; imdl < m_models.size(); imdl++)
		delete m_models[imdl];
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++)
		s_frame::free(m_kfrms[ikf]);

/*	m_fobjs.clear();*/
	m_kfrms.clear();
	m_models.clear();
}

bool f_inspector::alloc_d3dres()
{
	if(!f_ds_window::alloc_d3dres()){
		return false;
	}
	if(!m_model_view.init(m_pd3dev,
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
		(float) m_ViewPort.Width, (float) m_ViewPort.Height))
		return false;

	return true;
}

void f_inspector::release_d3dres()
{
	f_ds_window::release_d3dres();

	m_model_view.release();
	return;
}

// if a new frame is recieved, a new frame object is created.
// The corresponding frame object is in the list, it is sat as current frame object.
bool f_inspector::new_frame(Mat & img, long long & timg)
{
	if(is_equal(m_img, img)){
		cout << "Same frame with previous frame." << endl;
	}

	int bfound = false;
	// resize the original image to adjust the original aspect ratio.
	// (Some video file should change the aspect ratio to display correctly)
	resize(img, m_img_s, Size(), m_sh, m_sv);

	// generate gray scale image
	cvtColor(m_img_s, m_img_gry, CV_BGR2GRAY);

	// build image pyramid to track between frames
	GaussianBlur(m_img_gry, m_img_gry_blur, Size(0, 0), m_sig_gb);
	buildPyramid(m_img_gry_blur, m_impyr, m_lvpyr - 1);

	m_img = img;
	m_timg = timg;
	s_frame * pfrm_new;
	int next_kfrm = (m_cur_kfrm  + 1) % m_kfrms.size();

	if(m_kfrms[next_kfrm] && m_kfrms[next_kfrm]->tfrm == m_timg){ // the frame is already in the key frame cache
		pfrm_new = m_kfrms[next_kfrm];
	}else{
		pfrm_new = s_frame::alloc();

		// new frame can be loaded from file
		if(m_bauto_load_fobj && pfrm_new->load(m_name, timg, m_models)){
			pfrm_new->camint.copyTo(m_cam_int);
			pfrm_new->camdist.copyTo(m_cam_dist);
		}else if(m_pfrm){ // if previous frame is not null, initialize new frame with previous frame
			// frame tracking
			if(m_btrack_obj)
				pfrm_new->init(timg, m_pfrm, NULL, m_impyr, &m_ia, m_miss_tracks);
			else
				pfrm_new->init(timg, m_cam_int, m_cam_dist);
		}else{ 
			pfrm_new->init(timg, m_cam_int, m_cam_dist);
		}
	}

	if(m_pfrm){
		if(!m_pfrm->kfrm){
			s_frame::free(m_pfrm);
		}
		if(m_bauto_save_fobj){
			m_pfrm->save(m_name);
		}
	}

	m_pfrm = pfrm_new;

	return true;

	/*
	if(m_cur_frm >= 0){
		if(is_equal(m_img, img)){
			cout << "Same frame with previous frame." << endl;
		}

		// save current frame object
		if(m_bauto_save_fobj){
			if(!m_pfrm->save(m_name)){
				cerr << "Failed to save filter objects in time " << m_pfrm->tfrm << "." << endl;
			}
		}

		// if the next frame object is in the m_fobjs, we do not insert the new object
		if(m_pfrm->tfrm < timg){ // for larger time
			m_cur_frm++;
			for(;  m_cur_frm < m_fobjs.size(); m_cur_frm++){
				long long tfrm = m_pfrm->tfrm;
				if(tfrm == timg){
					bfound = true;
					break;
				}else if(tfrm > timg){
					break;
				}
			}
		}else{ // for smaller time 
			m_cur_frm--;
			for(; m_cur_frm >= 0; m_cur_frm--){
				long long tfrm = m_pfrm->tfrm;
				if(tfrm == timg){
					bfound = true;
					break;
				}else if(tfrm < timg){
					m_cur_frm++;
					break;
				}
			}
		}
	}else{
		m_cur_frm = 0;
	}

	if(!bfound){
		// new frame object added
		m_fobjs.insert(m_fobjs.begin() + m_cur_frm, new s_frame);
		if(m_pfrm == NULL){
			cerr << "Cannot allocate memory for frame object" << endl;
			return false;
		}

		m_cam_int.copyTo(m_pfrm->camint);
		m_cam_dist.copyTo(m_pfrm->camdist);

		if(m_bauto_load_fobj && m_pfrm->load(m_name, timg, m_models)){
			if (!m_pfrm->camint.empty())
				m_pfrm->camint.copyTo(m_cam_int);
			if (!m_pfrm->camdist.empty())
				m_pfrm->camdist.copyTo(m_cam_dist);
		}else if(m_btrack_obj){
			// determining reference frame.
			int iref_prev = m_cur_frm - 1;
			int iref_next = m_cur_frm + 1;

			if(iref_next < m_fobjs.size()){
				if(iref_prev >= 0)
					m_pfrm->init(timg, m_fobjs[iref_prev], m_fobjs[iref_next], m_impyr, &m_ia, m_miss_tracks);
				else
					m_pfrm->init(timg, m_fobjs[iref_next], NULL, m_impyr, &m_ia, m_miss_tracks);
			}else{
				if(iref_prev >= 0)
					m_pfrm->init(timg, m_fobjs[iref_prev], NULL, m_impyr, &m_ia, m_miss_tracks);
			}
			if (!m_pfrm->camint.empty())
				m_pfrm->camint.copyTo(m_cam_int);
			if (!m_pfrm->camdist.empty())
				m_pfrm->camdist.copyTo(m_cam_dist);
		}

		m_cur_obj = (int) m_pfrm->objs.size() - 1;
		if (m_cur_obj >= 0)
			m_cur_point = (int)m_pfrm->objs[m_cur_obj]->pt2d.size() - 1;
		else
			m_cur_point = 0;
	}

	// update time and image.
	m_timg = timg;
	m_img = img;

	return true;
	*/
}

bool f_inspector::proc()
{
	pthread_lock lock(&m_d3d_mtx);

	////////////////// updating pvt information ///////////////////////
	long long timg;	
	Mat img;
	img = m_pin->get_img(timg);
	if(img.empty() || timg < 0)
		return true;

	if(m_timg != timg){ // new frame arrived
		if(!new_frame(img, timg))
			return false;

		if(!m_pfrm)
			return true;

		// sample point templates
		m_pfrm->sample_tmpl(m_img_gry_blur, m_sz_vtx_smpl);

		m_int_kfrms = m_int_cyc_kfrm * f_base::m_paws->get_cycle_time();

		// initial frame is forced to be key frame
		if(m_cur_kfrm < 0){
			m_cur_kfrm = 0;
			m_kfrms[m_cur_kfrm] = m_pfrm;
			m_cur_kfrm = 0;
			m_kfrms[m_cur_kfrm]->set_as_key(m_img_s);
		}else if(m_kfrms[m_cur_kfrm]->tfrm + m_int_kfrms <= m_timg){ // after interval m_int_kfrms, the frame is key frame.
			m_cur_kfrm = (m_cur_kfrm + 1) % m_num_kfrms;
			if(m_kfrms[m_cur_kfrm] != NULL){
				if(m_kfrms[m_cur_kfrm]->tfrm != m_cur_time){ // if the frame is already in cache
					if(m_basv_kfrms)
						m_kfrms[m_cur_kfrm]->save(m_name);

					s_frame::free(m_kfrms[m_cur_kfrm]);
					m_kfrms[m_cur_kfrm] = NULL;
					// current implementation, this condition could not occur, because we dont have any method to load future key frame.
				}
			}

			if(!m_kfrms[m_cur_kfrm]){ // if the key frame is not allocated 
				m_kfrms[m_cur_kfrm] = s_frame::alloc();
				if(m_bald_kfrms && m_kfrms[m_cur_kfrm]->load(m_name, m_cur_time, m_models)){ // if the key frame is loaded from file
					if(!m_kfrms[m_cur_kfrm]->img.empty())
						m_img_s = m_kfrms[m_cur_kfrm]->img;
					m_timg = m_kfrms[m_cur_kfrm]->tfrm;
				}else{ // otherwise, current frame is sat as key frame.
					s_frame::free(m_kfrms[m_cur_kfrm]);
					m_kfrms[m_cur_kfrm] = m_pfrm;
					m_kfrms[m_cur_kfrm]->set_as_key(m_img_s);
				}
			}
		}
	}

	switch(m_sop){
	case SOP_LOAD:
		handle_sop_load();
		break;
	case SOP_SAVE:
		handle_sop_save();
		break;
	case SOP_DELETE:
		handle_sop_delete();
		break;
	case SOP_GUESS:
		handle_sop_guess();
		break;
	case SOP_INST_OBJ:
		handle_sop_inst_obj();
		break;
	case SOP_INS_CPTBL:
		handle_sop_ins_cptbl();
		break;
	case SOP_DET:
		handle_sop_det();
		break;
	case SOP_SET_KF:
		handle_sop_set_kf();
		break;
	case SOP_AWSCMD:
		handle_sop_awscmd();
		break;
	}

	// selecting frame of interest (we can choose from video stream and key frame)
	if(m_bkfrm){
		if(m_sel_kfrm >= 0)
			m_pfrm_int = m_kfrms[m_sel_kfrm];
	}else{
		m_pfrm_int = m_pfrm;
	}

	if(!m_pfrm_int){
		m_cur_obj = -1;
		m_cur_point = -1;
	}else if(m_cur_obj >= m_pfrm_int->objs.size()){
		m_cur_obj = m_pfrm_int->objs.size() - 1;
		m_cur_point = -1;
		// projection 
		m_num_cur_objs = (int) m_pfrm_int->objs.size();

		m_pfrm_int->proj_objs(true, m_bcalib_fix_aspect_ratio);
		calc_jmax();
	}

	// estimate
	if(m_op == ESTIMATE){
		m_kfrm_used.resize(m_kfrms.size(), false);
		switch(m_emd){
		case EMD_FULL:
			estimate_levmarq();
			break;
		case EMD_RT:
			estimate_rt_levmarq(m_pfrm_int);
			break;
		case EMD_RT_FULL:
			estimate_rt_full_levmarq();
			break;
		case EMD_SEL:
			estimate_rt_full_and_sel_cptbl();
			break;
		}
		calc_erep();
		m_emd = EMD_STOP;
	}


	// fit the viewport size to the image
	if(m_sz_img.width !=  m_img_s.cols &&
		m_sz_img.height != m_img_s.rows){
		if(!init_viewport(m_img_s)){
			return false;
		}
		m_main_scale = (float) m_rat;
		m_main_scale_inv = (float) (1.0 / m_main_scale);
		m_main_offset = Point2f(0., 0.);
		m_sz_img.width = m_img_s.cols;
		m_sz_img.height = m_img_s.rows;
	}

	// fit the direct 3d surface to the image
	if(m_img_s.cols != m_maincam.get_surface_width() ||
		m_img_s.rows != m_maincam.get_surface_height())
	{
		m_maincam.release();
		if(!m_maincam.init(m_pd3dev,  
			(float) m_img_s.cols, (float) m_img_s.rows, 
			(float) m_img_s.cols, (float) m_img_s.rows, 
			(float) m_ViewPort.Width, (float) m_ViewPort.Height))
			return false;
	}

	// fit the direct 3d surface of the model view to the image
	if((m_img_s.cols != m_model_view.get_surface_width() || 
		m_img_s.rows != m_model_view.get_surface_height()))
	{
		m_model_view.release();
		if(!m_model_view.init(m_pd3dev,
			(float) m_img_s.cols, (float) m_img_s.rows,
			(float) m_img_s.cols, (float) m_img_s.rows,
			(float) m_ViewPort.Width, (float) m_ViewPort.Height))
			return false;
	}

	// rendering main view
	if(m_pfrm_int){
		m_cam_int.copyTo(m_pfrm_int->camint);
		m_cam_dist.copyTo(m_pfrm_int->camdist);

		// calcurate roll pitch yaw surge sway heave relative to current object
		m_pfrm_int->calc_rpy(m_cur_obj);

		render(m_pfrm_int->img);
		m_pfrm_int->set_update();
	}else{
		Mat black = Mat::zeros(m_img_s.rows, m_img_s.cols, m_img_s.type());
		render(black);
	}
	
	return true;
}

bool f_inspector::addCamparTbl()
{
	double fx = m_cam_int.at<double>(0, 0);
	int icp;
	for(icp = 0; icp < m_cam_int_tbl.size(); icp++){
		Mat & ci = m_cam_int_tbl[icp];
		double * ptr = ci.ptr<double>();
		if(ptr[0] > fx){
			break;
		}
	}

	// insert current camera parameter here
	m_cam_int_tbl.insert(m_cam_int_tbl.begin() + icp, Mat());
	m_cam_dist_tbl.insert(m_cam_dist_tbl.begin() + icp, Mat());
	m_cam_int.copyTo(m_cam_int_tbl[icp]);
	m_cam_dist.copyTo(m_cam_dist_tbl[icp]);
	return true;
}

bool f_inspector::delCamparTbl()
{
	if(m_cur_camtbl >= 0 && m_cam_int_tbl.size() > 0 ){
		m_cam_int_tbl.erase(m_cam_int_tbl.begin() + m_cur_camtbl);
		m_cam_dist_tbl.erase(m_cam_dist_tbl.begin() + m_cur_camtbl);
		if(m_cur_camtbl >= m_cam_int_tbl.size())
			m_cur_camtbl = (int) m_cam_int_tbl.size() - 1;
	}
	return true;
}

bool f_inspector::saveCamparTbl()
{
	FileStorage fs(m_fname_campar_tbl, FileStorage::WRITE);
	if(!fs.isOpened())
		return false;

	fs << "CamPars" << (int) m_cam_int_tbl.size();
	fs << "Pars" << "{";
	for(int ipar = 0; ipar < m_cam_int_tbl.size(); ipar++){
		char buf[128];
		snprintf(buf, 128, "Par%03d", ipar);
		fs << buf << "{";
		fs << "Int" << m_cam_int_tbl[ipar];
		fs << "Dist" << m_cam_dist_tbl[ipar];
		fs << "}";
	}

	fs << "}";
	return true;
}

bool f_inspector::loadCamparTbl()
{
	clearCamparTbl();

	FileStorage fs(m_fname_campar_tbl, FileStorage::READ);
	if(!fs.isOpened())
		return false;

	int num_pars;
	FileNode fn = fs["CamPars"];
	if(fn.empty())
		return false;

	fn >> num_pars;
	m_cam_int_tbl.resize(num_pars);
	m_cam_dist_tbl.resize(num_pars);

	fn = fs["Pars"];
	if(fn.empty())
		return false;

	FileNodeIterator itr = fn.begin();
	for(int ipar = 0; ipar < num_pars; ipar++, itr++){
		char buf[128];
		snprintf(buf, 128, "Par%03d", ipar);
		FileNode fnpar = fn[buf];
		FileNode fnsub;

		if(fnpar.empty())
			return false;

		fnsub = fnpar["Int"];
		if(fnsub.empty())
			return false;
		fnsub >> m_cam_int_tbl[ipar];

		fnsub = fnpar["Dist"];
		if(fnsub.empty())
			return false;
		fnsub >> m_cam_dist_tbl[ipar];

	}

	m_bcam_tbl_loaded = true;
	return true;
}

void f_inspector::clearCamparTbl()
{
	m_cam_int_tbl.clear();
	m_cam_dist_tbl.clear();
	m_bcam_tbl_loaded = false;
}

bool f_inspector::load_model()
{
	for (int imdl = 0; imdl < m_models.size(); imdl++){
		if (strcmp(m_models[imdl]->fname, m_fname_model) == 0){
			m_cur_model = imdl;
			return true;
		}
	}

	s_model * mdl = new s_model();
	if (!mdl)
		return false;

	if(mdl->load(m_fname_model)){
		m_models.push_back(mdl);
		m_cur_model = (int)(m_models.size() - 1);
		return true;
	}
	return false;
}

//////////////////////////////////////////////// renderer
void f_inspector::render(Mat & imgs)
{
	// undistort if the flag is enabled.
	if(m_bundistort){
		Mat img;
		undistort(imgs, img, m_cam_int, m_cam_dist);
		imgs = img;
	}

	////////////////////// Direct 3D based renderer //////////////////
	m_pd3dev->BeginScene();

	//////////////////// clear back buffer ///////////////////////////
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);

	/////////////////////// render main view //////////////////////////
	m_maincam.SetAsRenderTarget(m_pd3dev);

	m_maincam.blt_offsrf(m_pd3dev, imgs);

	if(m_bkfrm){
		if(m_sel_kfrm >= 0 && m_kfrms[m_sel_kfrm]){
			renderObj(m_kfrms[m_sel_kfrm]);
		}
	}else
		renderObj(m_pfrm);

	renderCampar();

	m_maincam.ResetRenderTarget(m_pd3dev);

	////////////////////// render 3D model ///////////////////////////

	switch(m_op){
	case MODEL:
	case OBJ:
		renderModel();
		break;
	case VIEW3D:
		renderScene();
		break;
	default:
		break;
	}

	//////////////////// render total view port /////////////////////
	m_sz_img_view.width = (int)(m_sz_img.width * m_main_scale + 0.5);
	m_sz_img_view.height = (int)(m_sz_img.height * m_main_scale + 0.5);
	m_maincam.show(m_pd3dev, (float)(0. + m_main_offset.x),
		(float) ((float) m_sz_img_view.height + m_main_offset.y), m_main_scale);

	switch(m_op){
	case MODEL:
	case VIEW3D:
		m_model_view.show(m_pd3dev, 0, (float) m_ViewPort.Height);
		break;
	case OBJ:
		m_model_view.show(m_pd3dev, 0, (float) m_ViewPort.Height, 0.25);
		break;
	default:
		break;
	}

	renderInfo();

	renderCursor();

	////////////////////// grab rendered back surface ////////////////
	if(m_grab_name)
		grab();

	////////////////////// presentation //////////////////////////////
	if(m_pd3dev->Present(NULL, NULL, 
		NULL, NULL) == D3DERR_DEVICELOST){
			cerr << "device lost" << endl;
			m_blost = true;
	}
}

// Drawing text based information to show filter state
void f_inspector::renderInfo()
{
	// <mode> Mode Transition Info
	// Allowed key list
	// Cursor position
	char information[1024];
	int y = 0, x = 0;
	if(m_bkfrm && m_sel_kfrm >= 0){
		if(m_kfrms[m_sel_kfrm]){
			snprintf(information, 1023, "AWS Time %s (Key Frame Time %lld) Adjust Step x%f", 
				m_time_str, m_kfrms[m_sel_kfrm]->tfrm, (float) m_adj_step);
		}else{
			snprintf(information, 1023, "AWS Time %s (Key Frame Time NaN) Adjust Step x%f", 
				m_time_str, (float) m_adj_step);
		}
	}else{
		snprintf(information, 1023, "AWS Time %s (Image Time %lld) Adjust Step x%f", 
			m_time_str, m_timg, (float) m_adj_step);
	}
	m_d3d_txt.render(m_pd3dev, information, 0.f, (float) y, 1.0, 0, EDTC_LT);
	y += 20;
	snprintf(information, 1023, "Operation: %s (M)->Model (O)->Obj (Q)->Part (P)->Point (E)->Estimate (C)->Camera (T)->CamTable", m_str_op[m_op]);
	m_d3d_txt.render(m_pd3dev, information, 0.f, (float) y, 1.0, 0, EDTC_LT);
	y += 20;	
	if(m_pfrm_int){
		vector<s_obj*> & objs = m_pfrm_int->objs;
		snprintf(information, 1023, "%d Objects, %d Models, %d Miss Tracking", 
			objs.size(), m_models.size(), m_miss_tracks);
		m_d3d_txt.render(m_pd3dev, information, 0.f, (float)y, 1.0, 0, EDTC_LT);
	}else{
		snprintf(information, 1023, "%d Models, %d Miss Tracking", 
			m_models.size(), m_miss_tracks);
		m_d3d_txt.render(m_pd3dev, information, 0.f, (float)y, 1.0, 0, EDTC_LT);
	}
	y += 20;

	switch(m_op){
	case MODEL:
		renderModelInfo(information, 1023, y);
		break;
	case OBJ:
		if(m_pfrm_int){
			vector<s_obj*> & objs = m_pfrm_int->objs;
			renderObjInfo(objs, information, 1023, y);
		}else{
			renderObjInfo(vector<s_obj*>(), information, 1023, y);
		}
		break;
	case PARTS:
		if(m_pfrm_int){
			vector<s_obj*> & objs = m_pfrm_int->objs;
			renderPartsInfo(objs, information, 1023, y);
		}else{
			renderPartsInfo(vector<s_obj*>(), information, 1023, y);
		}
		break;
	case POINT:
		if(m_pfrm_int){
			vector<s_obj*> & objs = m_pfrm_int->objs;
			renderPointInfo(objs, information, 1023, y);
		}else{
			renderPointInfo(vector<s_obj*>(), information, 1023, y);
		}
		break;
	case CAMERA:
	case CAMTBL:
		renderCamparInfo(information, 1023, y);

		if(m_op != CAMTBL){
			break;
		}

		renderCamparTblInfo(information, 1023, y);
		break;
	case ESTIMATE:
		renderEstimateInfo(information, 1023, y);
		break;
	case FRAME:
		renderFrameInfo(information, 1023, y);
		break;
	case VIEW3D:
		renderSceneInfo(information, 1023, y);
		break;
	}
}

// helper functions for renderInfo
void f_inspector::renderModelInfo(char *buf, int len, int & y)
{
	if(m_cur_model < 0 || m_cur_model >= m_models.size())
		snprintf(buf, len, "Model[]=NULL");
	else
		snprintf(buf, len, "Model[%d]=%s (%d Points, %d Edges)", m_cur_model, 
		m_models[m_cur_model]->fname,
		m_models[m_cur_model]->pts.size(), m_models[m_cur_model]->edges.size());
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderObjInfo(vector<s_obj*> & objs, char * buf, int len, int & y)
{
	if(objs.size() == 0)
		snprintf(buf, len, "Obj[]=NULL");
	else if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
		s_obj & obj = *objs[m_cur_obj];
		snprintf(buf, len, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f rvec=(%f,%f,%f) tvec=(%f,%f,%f)",
			m_cur_obj, obj.name, obj.pmdl->fname, obj.match_count, obj.ssd,
			obj.rvec.at<double>(0), obj.rvec.at<double>(1), obj.rvec.at<double>(2),
			obj.tvec.at<double>(0), obj.tvec.at<double>(1), obj.tvec.at<double>(2));
	}
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderPartsInfo(vector<s_obj*> & objs, char * buf, int len, int & y)
{
	if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
		s_obj & obj = * objs[m_cur_obj];
		if(m_cur_part < 0 || m_cur_part >= obj.dpart.size()){
			snprintf(buf, len, "Part[]=NULL");
		}else{
			snprintf(buf, len, "Part[%d] parameter=%f", m_cur_part, obj.dpart[m_cur_part]);
		}
	}
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderPointInfo(vector<s_obj*> & objs, char * buf, int len, int & y)
{
	if(m_cur_obj < 0 || m_cur_obj >= objs.size())
		snprintf(buf, len, "Obj[]=NULL");
	else{
		s_obj & obj = *objs[m_cur_obj];
		if(m_cur_point < 0){
			snprintf(buf, len, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f", 
				m_cur_obj, obj.name, obj.pmdl->fname, obj.match_count, obj.ssd);
		}else{
			Point3f & pt3d = obj.pmdl->pts[m_cur_point];
			Point2f & pt2d = obj.pt2d[m_cur_point];
			int matched = obj.visible[m_cur_point];
			if(matched){
				snprintf(buf, len, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f Point[%d]=(%f,%f,%f)->(%f,%f)", 
					m_cur_obj, obj.name, obj.pmdl->fname, 
					obj.match_count, obj.ssd,
					m_cur_point, pt3d.x, pt3d.y, pt3d.z, pt2d.x, pt2d.y);
			}else{
				snprintf(buf, len, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f Point[%d]=(%f,%f,%f)->NULL", 
					m_cur_obj, obj.name, obj.pmdl->fname, 
					obj.match_count, obj.ssd,
					m_cur_point, pt3d.x, pt3d.y, pt3d.z);
			}
		}
	}
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderCamparInfo(char * buf, int len, int & y)
{
	float sx, sy;
	int cr, cg, cb;
	int val;
	bool sel;

	// camera intrinsics, fx,fy,cx,cy,p0,p1,k1-k6 
	int x = 0;
	snprintf(buf, len, "Camera ");

	m_d3d_txt.render(m_pd3dev, buf, (float)x, (float)y, 1.0, 0, EDTC_LT);
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	// focal length
	sel = m_cur_campar == ECP_FX || m_cur_campar == ECP_FY;
	val = sel ? 255 : 128;
	snprintf(buf, len, "fx=%f fy=%f", m_cam_int.at<double>(0,0), m_cam_int.at<double>(1,1));

	if(m_bcalib_fix_campar || m_bcalib_fix_focus){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	// principal points
	sel = m_cur_campar == ECP_CX;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_principal_point){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "cx=%f", m_cam_int.at<double>(0,2));
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_CY;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_principal_point){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "cy=%f", m_cam_int.at<double>(1,2));
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	//k1 to k6
	double * ptr = m_cam_dist.ptr<double>(0);
	sel = m_cur_campar == ECP_K1;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k1){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k1=%f", (float)ptr[0]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_K2;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k2){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k2=%f", (float)ptr[1]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_P1;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_zero_tangent_dist){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "p1=%f", (float)ptr[2]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_P2;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_zero_tangent_dist){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "p2=%f", (float)ptr[3]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_K3;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k3){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k3=%f", (float)ptr[4]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_K4;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k4){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k4=%f", (float)ptr[5]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_K5;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k5){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k5=%f", (float)ptr[6]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;

	sel = m_cur_campar == ECP_K6;
	val = sel ? 255 : 128;
	if(m_bcalib_fix_campar || m_bcalib_fix_k6){
		cr = val; cg = 0; cb = 0;
	}else{
		cr = 0; cg = val; cb = 0;
	}
	snprintf(buf, len, "k6=%f", (float)ptr[7]);
	m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
	m_d3d_txt.get_text_size(sx, sy, buf);
	x += (int)sx + 10;
	y += 20;
}

void f_inspector::renderCamparTblInfo(char * buf, int len, int & y)
{
	float sx, sy;
	int x;
	for(int icp = 0; icp < m_cam_int_tbl.size(); icp++, y += 20){
		Mat camint = m_cam_int_tbl[icp];
		Mat camdist = m_cam_dist_tbl[icp];
		int cr, cg, cb;

		cr = cg = cb = (icp == m_cur_camtbl ? 255 : 128);
		// camera intrinsics, fx,fy,cx,cy,p0,p1,k1-k6 
		x = 0;
		snprintf(buf, len, "Param[%d]:", icp);

		m_d3d_txt.render(m_pd3dev, buf, (float)x, (float)y, 1.0, 0, EDTC_LT);
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		// focal length
		snprintf(buf, len, "fx=%f fy=%f", camint.at<double>(0,0), camint.at<double>(1,1));			
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		// principal points
		snprintf(buf, len, "cx=%f", camint.at<double>(0,2));
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "cy=%f", camint.at<double>(1,2));
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		//k1 to k6
		double * ptr = camdist.ptr<double>(0);
		snprintf(buf, len, "k1=%f", (float)ptr[0]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "k2=%f", (float)ptr[1]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "p1=%f", (float)ptr[2]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "p2=%f", (float)ptr[3]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "k3=%f", (float)ptr[4]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "k4=%f", (float)ptr[5]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "k5=%f", (float)ptr[6]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;

		snprintf(buf, len, "k6=%f", (float)ptr[7]);
		m_d3d_txt.render(m_pd3dev, buf, (float)x,  (float)y, 1.0, 0., EDTC_LT, D3DCOLOR_RGBA(cr, cg, cb, 255));
		m_d3d_txt.get_text_size(sx, sy, buf);
		x += (int)sx + 10;
	}
	y += 20;
}

void f_inspector::renderEstimateInfo(char * buf, int len, int & y)
{
	snprintf(buf, len, "Estimate (1): All KF's RT and CamPars (2): Current Frame's RT (3): All KF's RT");
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
	snprintf(buf, len, "State: %s", m_str_emd[m_emd]);
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
	snprintf(buf, len, "NumItr: %d ErrPerPix: %f", m_num_max_itrs, m_cam_erep);
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderFrameInfo(char * buf, int len, int & y)
{
	snprintf(buf, len, "Frame Step: %d", (int) m_int_cyc_kfrm);
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderKeyFrameInfo(char * buf, int len, int & y)
{
	snprintf(buf, len, "%d Key Frames", (int)m_kfrms.size());
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;

	if(m_sel_kfrm >= 0 && m_kfrms[m_sel_kfrm]){
		snprintf(buf, len, "Key Frame Time %lld", m_kfrms[m_sel_kfrm]->tfrm);
	}else{
		snprintf(buf, len, "Key Frame NULL");
	}
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;

	snprintf(buf, len, "Key Frame Interval: %f(sec), Cache Size: %d",
		(float)((double)m_int_kfrms / (double) SEC), m_num_kfrms);
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
}

void f_inspector::renderSceneInfo(char * buf, int len, int & y)
{
	snprintf(buf, len, "Scene mode = %s (1): Camera (2): Current Obj X (3): Current Obj Y (4): Current Obj Z", m_str_view[m_ev]);
	m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y+= 20;

	// object attitude 
	if(m_pfrm_int){
		vector<s_obj*> & objs = m_pfrm_int->objs;
		if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
			for(int iobj = 0; iobj < objs.size(); iobj++){
				s_obj & obj = *objs[iobj];
				snprintf(buf, len, "%s Roll: %5.2f, Pitch: %5.2f, Yaw: %5.2f X: %5.2f Y: %5.2f Z: %5.2f", 
					obj.name, (float)obj.roll * 180./CV_PI, (float)obj.pitch * 180./CV_PI, (float)obj.yaw * 180./CV_PI,
					(float)obj.pos.x, (float)obj.pos.y, (float)obj.pos.z);
				m_d3d_txt.render(m_pd3dev, buf, 0.f, (float)y, 1.0, 0, EDTC_LT);
				y += 20;
			}	
		}
	}
}

// Decorating mouse cursor
void f_inspector::renderCursor()
{
	// vertial and horizontal lines are drawn as crossing at mouse cursor
	D3DXVECTOR2 v[2];
	m_pline->Begin();
	v[0] = D3DXVECTOR2((float) 0, (float) m_mc.y);
	v[1] = D3DXVECTOR2((float)m_ViewPort.Width - 1, (float) m_mc.y);
	m_pline->Draw(v, 2, D3DCOLOR_RGBA(0, 255, 0, 255));
	v[0] = D3DXVECTOR2((float) m_mc.x, (float) m_ViewPort.Height - 1);
	v[1] = D3DXVECTOR2((float) m_mc.x, (float) 0);
	m_pline->Draw(v, 2, D3DCOLOR_RGBA(0, 255, 0, 255));
	m_pline->End();

	m_pd3dev->EndScene();
	char buf[128];
	Point2f ptimg;
	cnv_view2img(m_mc, ptimg);
	snprintf(buf, 127, "(%f, %f)", ptimg.x, ptimg.y);
	e_d3d_txt_center etxtc;
	Point2f txtofst;
	if(m_mc.x < (float) (m_ViewPort.Width >> 1)){
		if(m_mc.y < (float) (m_ViewPort.Height >> 1)){
			etxtc = EDTC_LT;
			txtofst = Point2f(20, 20);
		}else{
			etxtc = EDTC_LB;
			txtofst = Point2f(20, -20);
		}
	}else{
		if(m_mc.y < (float) (m_ViewPort.Height >> 1)){
			etxtc = EDTC_RT;
			txtofst = Point2f(-20, 20);
		}else{
			etxtc = EDTC_RB;
			txtofst = Point2f(-20, -20);
		}		
	}
	m_d3d_txt.render(m_pd3dev, buf, (float)(m_mc.x + txtofst.x), (float)(m_mc.y + txtofst.y), 1.0, 0, etxtc);
}

void f_inspector::renderObj(s_frame * pfrm)
{
	// Drawing object (2d and 3d)
	vector<s_obj*> & objs = pfrm->objs;
	if(m_cur_obj >= objs.size())
		return;

	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		if(m_op == OBJ || m_op == PARTS){
			if(iobj == m_cur_obj){
				drawPoint2d(m_pd3dev, 
					NULL, m_pline,
					obj.pt2d, obj.visible, iobj, 1);
				obj.render(
					m_pd3dev, NULL, m_pline, 
					iobj, 0, m_cur_point);
			}
		}else if(m_op == POINT){
			if(iobj == m_cur_obj){
				drawPoint2d(m_pd3dev, 
					NULL, m_pline,
					obj.pt2d, obj.visible, iobj, 1);
				obj.render(
					m_pd3dev, NULL, m_pline, 
					iobj, 0, m_cur_point);
			}
		}else{
			drawPoint2d(m_pd3dev, 
				NULL, m_pline,
				obj.pt2d, obj.visible, 0);
			obj.render(
				m_pd3dev, NULL, m_pline, 
				iobj, 0);
		}

		vector<Point2f> & pt2d = obj.pt2d;
		vector<Point2f> & pt2dprj = obj.pt2dprj;
		vector<int> & matched = obj.visible;
		D3DXVECTOR2 v[2];
		m_pline->Begin();
		for(int i = 0; i < pt2d.size(); i++){
			if(!matched[i]){
				continue;
			}
			v[0] = D3DXVECTOR2(pt2d[i].x, pt2d[i].y);
			v[1] = D3DXVECTOR2(pt2dprj[i].x, pt2dprj[i].y);
			m_pline->Draw(v, 2, D3DCOLOR_RGBA(128, 128, 128, 128));
		}
		m_pline->End();

		// render selected axis
		obj.render_axis(
			m_cam_int, m_cam_dist,
			m_pd3dev, m_pline, (int) m_axis);
	}

	m_pline->Begin();
	if(m_op == POINT && m_cur_obj != -1 && m_cur_point != -1){
		D3DXVECTOR2 v[2];
		Point2f pt1;
		cnv_view2img(m_mc, pt1);
		Point2f & pt2 = objs[m_cur_obj]->pt2dprj[m_cur_point];
		v[0] = D3DXVECTOR2(pt1.x, pt1.y);
		v[1] = D3DXVECTOR2(pt2.x, pt2.y);
		m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 0, 0, 255));
	}
	m_pline->End();
}

void f_inspector::renderScene()
{
	if(!m_pfrm_int || m_cur_obj < 0 || m_cur_obj >= m_pfrm_int->objs.size())
		return;

	m_model_view.SetAsRenderTarget(m_pd3dev);
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.5f, 0.5f, 0.5f, 0.0f), 1.0f, 0);
	Mat rvec, tvec;
	double d;
	Mat Rcam, Rorg;

	// EV_OBJX, EV_OBJY, EV_OBJZ view object's attitude with a coordinate frame fixed at the current object.
	switch(m_ev){
	case EV_CAM:
		m_cam_int.copyTo(m_cam_int_view);
		m_cam_dist.copyTo(m_cam_dist_view);
		m_tvec_view = Mat::zeros(3, 1, CV_64FC1);
		m_rvec_view = Mat::zeros(3, 1, CV_64FC1);
		m_ev = EV_CAM;
		break;
	case EV_OBJX:
		{
			vector<s_obj*> & objs = m_pfrm_int->objs;
			if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
				// preparing rotation matrix viewing from x-axis of the current object.
				double * ptr0, * ptr1;
				Rodrigues(objs[m_cur_obj]->rvec, Rorg);
				Rcam = Mat(3, 3, CV_64FC1);
				ptr0 = Rorg.ptr<double>();
				ptr1 = Rcam.ptr<double>();
		
				// x-axis to z-axis
				ptr1[0] = ptr0[1];
				ptr1[3] = -ptr0[2];
				ptr1[6] = -ptr0[0];
				ptr1[1] = ptr0[4];
				ptr1[4] = -ptr0[5];
				ptr1[7] = -ptr0[3];
				ptr1[2] = ptr0[7];
				ptr1[5] = -ptr0[8];
				ptr1[8] = -ptr0[6];

				ptr0 = objs[m_cur_obj]->tvec.ptr<double>();
				d = ptr0[2]; // Z value is the scene depth for the camera 
			}
		}
		break;
	case EV_OBJY:
		{
			vector<s_obj*> & objs = m_pfrm_int->objs;
			if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
				// preparing rotation matrix viewing from y-axis of the current object.
				double * ptr0, * ptr1;
				Rodrigues(objs[m_cur_obj]->rvec, Rorg);
				Rcam = Mat(3, 3, CV_64FC1);
				ptr0 = Rorg.ptr<double>();
				ptr1 = Rcam.ptr<double>();
		
				// y-axis to z-axis
				ptr1[0] = -ptr0[0];
				ptr1[3] = -ptr0[2];
				ptr1[6] = -ptr0[1];
				ptr1[1] = -ptr0[3];
				ptr1[4] = -ptr0[5];
				ptr1[7] = -ptr0[4];
				ptr1[2] = -ptr0[6];
				ptr1[5] = -ptr0[8];
				ptr1[8] = -ptr0[7];

				ptr0 = objs[m_cur_obj]->tvec.ptr<double>();
				d = ptr0[2]; // Z value is the scene depth for the camera 
			}
		}
		break;
	case EV_OBJZ:
		{
			vector<s_obj*> & objs = m_pfrm_int->objs;
			if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
				// preparing rotation matrix viewing from z-axis of the current object.
				double * ptr0, * ptr1;
				Rodrigues(objs[m_cur_obj]->rvec, Rorg);
				Rcam = Mat(3, 3, CV_64FC1);
				ptr0 = Rorg.ptr<double>();
				ptr1 = Rcam.ptr<double>();
		
				// x-axis to z-axis
				ptr1[0] = ptr0[0];
				ptr1[3] = ptr0[1];
				ptr1[6] = ptr0[2];
				ptr1[1] = ptr0[3];
				ptr1[4] = ptr0[4];
				ptr1[7] = ptr0[5];
				ptr1[2] = ptr0[6];
				ptr1[5] = ptr0[7];
				ptr1[8] = ptr0[8];

				ptr0 = objs[m_cur_obj]->tvec.ptr<double>();
				d = ptr0[2]; // Z value is the scene depth for the camera 
			}
		}
		break;
	case EV_FREE:
		Rodrigues(m_rvec_view, Rcam);
		break;
	}

	vector<Point2f> pts;
	if(m_pfrm_int){
		vector<s_obj*> & objs = m_pfrm_int->objs;
		Mat tvec_org = objs[m_cur_obj]->tvec;
		for(int iobj = 0; iobj < objs.size(); iobj++){
			Mat R;
			Rodrigues(objs[iobj]->rvec, R);
			if (m_ev == EV_CAM){
				rvec = objs[iobj]->rvec;
				tvec = objs[iobj]->tvec;

			}else if(m_ev == EV_FREE){
				tvec = Rcam * (objs[iobj]->tvec - tvec_org);
				tvec += m_tvec_view;

				R = Rcam * R;
				Rodrigues(R, rvec);
			}else{
				tvec = Rcam * (objs[iobj]->tvec - tvec_org);
				tvec.ptr<double>()[2] += m_rat_z * d;

				R = Rcam * R;
				Rodrigues(R, rvec);
				// calculating roll pitch yaw relative to the cur_obj coordinate frame
				// relative rotation matrix (here Rorg is the transpose of the rotation matrix of the current object
			}
			
			awsProjPts(objs[iobj]->pmdl->pts, pts, m_cam_int, m_cam_dist, rvec, tvec);
			if(m_cur_obj == iobj)
				render_prjpts(*objs[iobj]->pmdl, pts, m_pd3dev, NULL, m_pline, iobj, 0, -1);	
			else
				render_prjpts(*objs[iobj]->pmdl, pts, m_pd3dev, NULL, m_pline, iobj, 1, -1);

			objs[iobj]->render_axis(rvec, tvec, m_cam_int, m_cam_dist, m_pd3dev, m_pline);
		}
	}
	m_model_view.ResetRenderTarget(m_pd3dev);
}

void f_inspector::renderModel()
{
	m_model_view.SetAsRenderTarget(m_pd3dev);
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.5f, 0.5f, 0.5f, 0.0f), 1.0f, 0);
	// 3D model is rotated in the model view with the angle speed of 1 deg/sec
	// Camera parameter is set as 
	if(m_cur_model != -1){
		m_theta_z_mdl += (1./6.) * CV_PI / 180.;
		// calculating rotation vector
		m_rvec_mdl = Mat(3, 1, CV_64F);
		double * ptr = m_rvec_mdl.ptr<double>();
		ptr[0] = 0.;
		ptr[1] = m_theta_z_mdl;
		ptr[2] = 0.;

		// twice the maximum length of the model
		m_dist_mdl = 2 * m_models[m_cur_model]->get_max_dist(); 

		// calculating translation vector
		m_tvec_mdl = Mat(3, 1, CV_64F);
		ptr = m_tvec_mdl.ptr<double>();
		ptr[0] = 0;
		ptr[1] = 0;
		ptr[2] = m_dist_mdl;

		// calculating camera matrix 
		// set the model at distance "dist" can be completely inside the view port
		// fov should be larger than 2*atan(0.5)
		// fov_w = atan(wpix * pitch / f)
		// fov_h = atan(hpix * pitch / f)
		// f = min((wpix * pitch)/tan(fov), (hpix * pitch)/tan(fov))
		// fpix = min(wpix, hpix) / tan(fov)
		float wpix = m_model_view.get_surface_width();
		float hpix = m_model_view.get_surface_height();
		float fpix = (float)(min(wpix, hpix) / tan(atan(0.5)));
		m_cam_int_mdl = Mat(3, 3, CV_64F);
		m_cam_int_mdl.at<double>(0, 0) = fpix;
		m_cam_int_mdl.at<double>(1, 1) = fpix;
		m_cam_int_mdl.at<double>(0, 2) = 0.5 * wpix;
		m_cam_int_mdl.at<double>(1, 2) = 0.5 * hpix;

		// calculating camera distortion (set at zero)
		m_cam_dist_mdl = Mat::zeros(8, 1, CV_64FC1);

		// calculating camera rotation
		m_rvec_cam_mdl = Mat::zeros(3, 1, CV_64FC1);

		// calculating camera translation (set as zero)
		m_tvec_cam_mdl = Mat::zeros(3, 1, CV_64FC1);

		vector<Point2f> pts;
		m_models[m_cur_model]->proj(pts, m_cam_int_mdl, m_cam_dist_mdl, m_rvec_cam_mdl, m_tvec_cam_mdl, m_rvec_mdl, m_tvec_mdl);
		render_prjpts(*m_models[m_cur_model], pts, m_pd3dev, NULL, m_pline, m_cur_model, 0, -1);	
	}
	m_model_view.ResetRenderTarget(m_pd3dev);
}

void f_inspector::renderCampar()
{
	if(m_op != CAMERA)
		return ;

	double w, h;
	w = m_maincam.get_surface_width();
	h = m_maincam.get_surface_height();

	double fx, fy, cx, cy;
	fx = m_cam_int.at<double>(0, 0);
	fy = m_cam_int.at<double>(1, 1);
	cx = m_cam_int.at<double>(0, 2);
	cy = m_cam_int.at<double>(1, 2);

	int  mw, mh, cw, ch;
	cw = (int)(w / 20.0);
	ch = (int)(h / 20.0);
	mw = 2 * cw + 1;
	mh = 2 * ch + 1;
	vector<Point3f> pt3d;
	vector<Point2f> pt2d;

	pt3d.resize(mw * mh);
	int ipt = 0, ipt_dst;
	Point3f org(-(float) (cw * 20.0), - (float) (ch * 20.0), (float) fx);
	for(int i = 0; i < mw; i++){
		for(int j = 0; j < mh; j++){
			pt3d[ipt] = Point3f((float)(org.x + 20.0 * i), (float)(org.y + 20.0 * j), (float)fx);
			ipt++;
		}
	}

	//projectPoints(pt3d, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), m_cam_int, m_cam_dist, pt2d);
	double R[9] = {1,0,0, 0,1,0, 0,0,1};
	double t[3] = {0, 0, 0};
	awsProjPts(pt3d, pt2d, fx, fy, cx, cy, m_cam_dist.ptr<double>(), R, t);

	D3DXVECTOR2 v[2];
	m_pline->Begin();
	ipt = 0;
	ipt_dst = ipt + mh;
	// horizontal line
	for(int i = 0; i < mw - 1; i++){
		for(int j = 0; j < mh; j++){
			v[0] = D3DXVECTOR2(pt2d[ipt].x, pt2d[ipt].y);
			v[1] = D3DXVECTOR2(pt2d[ipt_dst].x, pt2d[ipt_dst].y);
			m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 255, 255, (j == ch ? 255 : 128)));
			ipt++;
			ipt_dst++;
		}
	}

	ipt = 0;
	ipt_dst = ipt + 1;
	// vertical line
	for(int i = 0; i < mw; i++){
		for(int j = 0; j < mh - 1; j++){
			v[0] = D3DXVECTOR2(pt2d[ipt].x, pt2d[ipt].y);
			v[1] = D3DXVECTOR2(pt2d[ipt_dst].x, pt2d[ipt_dst].y);
			m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 255, 255, (i == cw ? 255 : 128)));
			ipt++;
			ipt_dst++;
		}
		ipt++;
		ipt_dst++;
	}

	m_pline->End();
}

//About the parameter estimation
//m: 2d point projected
//M: 3d point in object coordinate
//r: rotation params (including world/camera rotation)
//   vector in R^3 
//t: translation params (including world/camera translation)
//   vector in R^3
//p: projection params
//   vector in R^4 (focal length and principal point for both x,y direction)
//k: distortion params
//   vector in R^6
//
//The projection function is
//
//m = P(D(T(R(M;r);t),k),p)
//
//Here I ommit the indices of the points and objects
//
//The estimation of parameters of camera and objects means the optimization,
//
//    min Sum(m-m')^2
// [r, t, k, k]^T for all points
//
//First, to enable iterative minimization, linearize the function D around [r,t,p,k]^T
//
//D(P(T(R(M;r+dr);t+dt);p+dp)k+dk) 
//  <=> J[dr, dt, dk, dp]^T + P(D(T(R(M;r);t);k)p)
//
//J is the Jacobian. Then we minimize
//
//     min Sum{m - J[dr, dt, dk, dp]^T - D(P(T(R(M;r);t);k)p)}^2
// [dr, dt, dk, dp]^T for all points
//
//and update parameters with
//
//[r, t, k, p]^T <= [r, t, k, p]^T + [dr, dt, dk, dp]^T
//
//The [dr ,dt, dk, dp]^T is actually,
//
//Sum{2J^T{m - J[dr, dt, dk, dp]^T - P(D(T(R(M;r);t);k)p)}=0 (the derivative equals zero)
//Sum {J^TJ [dr, dt, dk, dp]^T} = Sum {J^Tm - P(D(T(R(M;r);t);k)p)}
//[dr, dt, dk, dp]^T = Sum {J^TJ}^(-1) Sum J^T{m - P(D(T(R(M;r);t);k)p)} 
//
//yes,it's Gauss-Newton method. And the Jacobian can be calculated as,
//
//J=dD/d[r, t, k, p]^T = [dD/dr, dD/dt, dD/dk, dD/dp]
//
//dP/dr=dP/dD * dD/dT * dT/dR * dR/dr
//dP/dt=dP/dD * dD/dT * dT/dt
//dP/dp=dP/dD * dD/dT
//dP/dk=dP/dk
//
//The Jacobian can also be calculated with cv::projectPoints

void f_inspector::estimate()
{
	if(!m_pfrm_int)
		return;

	Mat Hcamint = Mat::zeros(12, 12, CV_64FC1);
	vector<s_obj*> & objs = m_pfrm_int->objs;
	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = *objs[i];
		cout << "obj[" << i << "].hessian=" << obj.hessian << endl;
		// accumulating camera intrinsic part of hessians of all objects
		Hcamint += obj.hessian(Rect(6, 6, 12, 12));
	}
	cout << "Hcamint=" << Hcamint << endl;
	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = *objs[i];
		// copy a part of hessian corresponding to the camera intrinsics 
		Hcamint.copyTo(obj.hessian(Rect(6, 6, 12, 12)));
	}

	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = *objs[i];
		cout << "Accumulated obj[" << i << "].hessian=" << obj.hessian << endl;
		cout << "Previous params" << endl;
		cout << "rvec=" << obj.rvec << endl;
		cout << "tvec=" << obj.tvec << endl;
		Mat Hinv, eigenval, eigenvec;
		double det = determinant(obj.hessian);
		eigen(obj.hessian, eigenval, eigenvec);
		cout << "eigenval=" << eigenval << endl;
		//cout << "eigenvec=" << eigenvec << endl;
		cout << "detH=" << det << endl;
		invert(obj.hessian, Hinv, DECOMP_CHOLESKY);
		cout << "Hinv=" << Hinv << endl;
		cout << "Err=" << obj.err << endl;
		Mat Grad = obj.jacobian.t() * obj.err;;
		cout << "Grad=" << Grad << endl;
		obj.dp = Hinv * Grad;
		double * ptr_dp = obj.dp.ptr<double>(0);
		double * ptr; 
		ptr = obj.rvec.ptr<double>(0);
		ptr[0] += ptr_dp[0]; // rx
		ptr[1] += ptr_dp[1]; // ry
		ptr[2] += ptr_dp[2]; // rz
		ptr = obj.tvec.ptr<double>(0);
		ptr[0] += ptr_dp[3]; // tx
		ptr[1] += ptr_dp[4]; // ty
		ptr[2] += ptr_dp[5]; // tz
		ptr = m_cam_int.ptr<double>(0);
		ptr[0] += ptr_dp[6]; // fx
		ptr[2] += ptr_dp[8]; // cx
		ptr[4] += ptr_dp[7]; // fy
		ptr[5] += ptr_dp[9]; // cy
		// Updating distortion parameters
		ptr = m_cam_dist.ptr<double>(0);
		ptr_dp += 10;
		for(int i = 0; i < 8; i++, ptr++, ptr_dp++){
			*ptr += *ptr_dp;
		}

		cout << "New params" << endl;
		cout << "rvec=" << obj.rvec << endl;
		cout << "tvec=" << obj.tvec << endl;
	}
	cout << "camint=" << m_cam_int << endl;
	cout << "camdist=" << m_cam_dist << endl;
}

void f_inspector::estimate_rt_full_and_sel_cptbl()
{
	int icp_min = 0;
	double ssd_min = DBL_MAX;
	for(int icp = 0; icp < m_cam_int_tbl.size(); icp++){
		m_cam_int_tbl[icp].copyTo(m_cam_int);
		m_cam_dist_tbl[icp].copyTo(m_cam_dist);
		estimate_rt_full_levmarq();
		double ssd = 0.;
		for(int ikfrm = 0; ikfrm < m_kfrms.size(); ikfrm++){
			if(!m_kfrm_used[ikfrm])
				continue;

			ssd += m_kfrms[ikfrm]->ssd;
		}
		if(ssd < ssd_min){
			icp_min = icp;
		}
	}

	m_cam_int_tbl[icp_min].copyTo(m_cam_int);
	m_cam_dist_tbl[icp_min].copyTo(m_cam_dist);
	estimate_rt_full_levmarq();
}

void f_inspector::estimate_rt_full_levmarq()
{
	for(int ikfrm = 0; ikfrm < m_kfrms.size(); ikfrm++){
		if(m_kfrms[ikfrm] == NULL){
			m_kfrm_used[ikfrm] = false;
			continue;
		}
		estimate_rt_levmarq(m_kfrms[ikfrm]);
		m_kfrm_used[ikfrm] = true;
	}
}

void f_inspector::estimate_rt_levmarq(s_frame * pfrm)
{
	if(pfrm == NULL)
		return;

	// counting extrinsic parameters (6 x number of objects)
	int nparams = 0; // we assume the camera intrinsics are the same for every frames.

	// current camera intrinsic parameters (including distortion) are used 
	m_cam_int.copyTo(pfrm->camint);
	m_cam_dist.copyTo(pfrm->camdist);

	pfrm->proj_objs(true, m_bcalib_fix_aspect_ratio);

	vector<s_obj*> & objs = pfrm->objs;		
	nparams += (int) objs.size() * 6;
	if(nparams == 0)
		return ;

	// initializing LM-solver
	CvTermCriteria tc;
	tc.epsilon = FLT_EPSILON;
	tc.max_iter = m_num_max_itrs;
	tc.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
	//m_solver.init(nparams, 0, tc);
	m_solver.initEx(nparams, 0, tc);

	//set initial parameter and parameter mask
	double * param = m_solver.param->data.db;
	uchar * mask = m_solver.mask->data.ptr;

	// setting extrinsics
	for(int iobj = 0; iobj < objs.size(); iobj++){
		double * pext;
		// copy rvec
		s_obj & obj = *objs[iobj];
		pext = obj.rvec.ptr<double>();
		memcpy((void*)param, (void*)pext, sizeof(double) * 3);

		// copy tvec
		pext = obj.tvec.ptr<double>();
		memcpy((void*)(param + 3), (void*)pext, sizeof(double) * 3);

		param += 6;
	}

	// Compounds of Jacobian
	// J = 
	//     | Cam | Obj1 | Obj2 | ... | Obj n |
	// OP1   C1    OP11
	// OP2   C2           OP22 
	// ...
	// OPn   Cn                         OPnn
	// 
	// Jt = 
	//     | OP1   | OP2   | ... | OP4  |
	// Cam   C1^t    C2^t          C4^t
	// Obj1  OP11^t
	// Obj2         OP22^t 
	// ...
	// Objn                        OPnn^t
	// 
	// Psuedo Hessian (JtJ)
	//      | Cam         |  Obj1   |  Obj2   | .... |  Obj n   |
	// Cam    Hcam         C1^tOP11  C2^tOP22          Cn^tOPnn
	// Obj1   (C1^tOP11)^t  OP11^2
	// Obj2   (C2^tOP22)^t            OP22^2
	// ...
	// Objn   (Cn^tOPnn)^t                              OPnn^2
	//
	// Hcam = C1^2+...Cn^2
	// 
	// Err =
	//     |Err|
	//      OE1  
	//      OE2  
	//       ...
	//      OEn  
	//
	// JtErr
	//     Ecam
	//     OP11^tOE1
	//     OP22^tOE2
	//     ...
	//     OPnn^tOEn
	// Ecam = C1^tOE1 + C2^tOE2 + ... + Cn^tOEn

	const CvMat * _param = NULL;
	double * pparam = NULL;
	int itr = 0;
#ifdef VERB_LM
	ofstream log("levmarq.csv");
#endif
	while(1){
#ifdef VERB_LM
		log << "Iteration " << itr << " state=" << m_solver.state << endl;
#endif
		double * errNorm = NULL;
		CvMat * _JtJ = NULL, *_JtErr = NULL; // for each iteration, these data structure should be initialized.
											 // This is because the ERROR_CHECK state of CvLevMarq assume the 
											 // outerloop does not update their JtJ and JtErr
		param = m_solver.param->data.db;
		pparam = m_solver.prevParam->data.db;
		//bool proceed = m_solver.updateAlt(_param, _JtJ, _JtErr, errNorm);
		bool proceed = m_solver.updateAltEx(_param, _JtJ, _JtErr, errNorm);
#ifdef VERB_LM
		//mat2csv(log, Mat(m_solver.JtJN));
		log << "V = " << endl;
		mat2csv(log, Mat(m_solver.JtJV));
		log << "W = " << endl;
		mat2csv(log, Mat(m_solver.JtJW));
#endif

		for(int iobj = 0; iobj < objs.size(); iobj++){
			double * pext;
			// copy rvec
			s_obj & obj = *objs[iobj];
			pext = obj.rvec.ptr<double>();
			memcpy((void*)pext, (void*)param, sizeof(double) * 3);
#ifdef VERB_LM
			log << "rvec[" << iobj << "]=" << endl;
			mat2csv(log, obj.rvec);
#endif
			// copy tvec
			pext = obj.tvec.ptr<double>();
			memcpy((void*)pext, (void*)(param + 3), sizeof(double) * 3);
#ifdef VERB_LM
			log << "tvec[" << iobj << "]=" << endl;
			mat2csv(log, obj.tvec);
#endif
			obj.update = false;
			param += 6;
		}

		// calculate projection
		double ssd = 0.0;
#ifdef VERB_LM
		log << "Cam_int = " << endl;
		mat2csv(log, m_cam_int);
		log << "Cam_dist = " << endl;
		mat2csv(log, m_cam_dist);
#endif
		bool updateJ = _JtJ != NULL && _JtErr != NULL;
		pfrm->update = false;
		// projection
		pfrm->proj_objs(updateJ, m_bcalib_fix_aspect_ratio);
		// accumulating frame's projection ssd
		ssd += pfrm->ssd;

		if(!proceed){
			return ;
		}
		if(errNorm){
			*errNorm = sqrt(ssd);
#ifdef VERB_LM
			log << "errNorm," << *errNorm << endl;
#endif
		}

		if(updateJ){
			Mat JtJ(_JtJ);
			Mat JtErr(_JtErr);

			// Calculating Psuedo Hessian JtJ and JtErr
			for(int iobj = 0, i = 0; iobj < objs.size(); iobj++, i+=6){
				s_obj & obj = *objs[iobj];
#ifdef VERB_LM
				log << "J[" << iobj <<"] = " << endl;
				mat2csv(log, obj.jacobian);
				// JtJ
				log << "H[" << iobj <<"] = " << endl;
				mat2csv(log, obj.hessian);
#endif
				// JtJFrame
				//      6  
				// 6   RTRT
				//
				// JtJ(12+6ifrm:12+6ifrm+6, 12+6ifrm:12+6ifrm+6) = Hfrm(0:6, 0:6)
				obj.hessian(Rect(0, 0, 6, 6)).copyTo(JtJ(Rect(i, i, 6, 6)));
#ifdef VERB_LM
				// JtErr
				log << "JtErr[" << iobj << "] = " << endl;
				mat2csv(log, obj.jterr);
#endif
				// JtErrFrame
				// 6  ERT
				// 12 ECAM
				obj.jterr(Rect(0, 0, 1, 6)).copyTo(JtErr(Rect(0, i, 1, 6)));
			}
#ifdef VERB_LM
			log << "JtJ =" << endl;
			mat2csv(log, JtJ);
			Mat JtJN;
			JtJ.copyTo(JtJN);
			double Log10 = ::log(10.0);
			double lambda = exp(m_solver.lambdaLg10 * Log10);
			for(int i = 0; i < JtJN.cols; i++)
				JtJN.at<double>(i, i) *= 1.0 + lambda;
			log << "lambda, " << lambda << endl;
			Mat JtJinv = JtJN.inv(DECOMP_CHOLESKY);
			log << "JtJinv =" << endl;
			mat2csv(log, JtJinv);
			Mat evl, evc;
			eigen(JtJ, evl, evc);
			log << "Eval =" << endl;
			mat2csv(log, evl);
			log << "Evec =" << endl;
			mat2csv(log, evc);
			log << "JtErr =" << endl;
			mat2csv(log, JtErr);
#endif
		}
		itr++;
	}
}

// estimation methods written with Levenberg Marquardt implementation in OpenCV (CvLevMarq)
void f_inspector::estimate_levmarq()
{
//	double ssd_avg, ssd_range;
	// calculating m_frm_used flag. if the flag is asserted, the frame is used for the optimization	double step =  (double)m_kfrms.size() / (double) num_valid_frms; 
	int num_valid_frms = 0;
	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm ++){
		if(!m_kfrms[ifrm])
			continue;
		if(m_kfrms[ifrm]->objs.size() == 0)
			continue;

		m_kfrm_used[ifrm] = true;
		num_valid_frms++;
	}

	// current frame is forced to be used for the optimization
	if(!m_kfrm_used[m_cur_kfrm]){
		m_kfrm_used[m_cur_kfrm] = true;
		num_valid_frms++;
	}

	// counting extrinsic parameters (6 x number of objects)
	int nparams = 12; // we assume the camera intrinsics are the same for every frames.
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		if(!m_kfrm_used[ikf])
			continue;

		// setting initial camera parameter 
		m_cam_int.copyTo(m_kfrms[ikf]->camint);
		m_cam_dist.copyTo(m_kfrms[ikf]->camdist);

		// and initial projection with full jacobian calculation
		m_kfrms[ikf]->proj_objs(true, m_bcalib_fix_aspect_ratio);

		vector<s_obj*> & objs = m_kfrms[ikf]->objs;		
		nparams += (int) objs.size() * 6;
	}

	// initializing LM-solver
	CvTermCriteria tc;
	tc.epsilon = FLT_EPSILON;
	tc.max_iter = m_num_max_itrs;
	tc.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
	m_solver.initEx(nparams, 0, tc);

	//set initial parameter and parameter mask

	double * param = m_solver.param->data.db;
	uchar * mask = m_solver.mask->data.ptr;

	// Setting camera intrinsics
	double * ptr_int, * ptr_dist;
	ptr_int = m_cam_int.ptr<double>(0);
	ptr_dist = m_cam_dist.ptr<double>(0);

	if(m_bcalib_fix_campar){
		for(int ipar = 0; ipar < 12; ipar++)
			mask[ipar] = 0;
	}
	if(m_bcalib_fix_focus){
		mask[0] = 0;
		mask[1] = 0;
	}
	param[0] = ptr_int[0];
	param[1] = ptr_int[4];

	if(m_bcalib_fix_principal_point){
		mask[2] = 0;
		mask[3] = 0;
	}
	param[2] = ptr_int[2];
	param[3] = ptr_int[5];

	if(m_bcalib_zero_tangent_dist){
		mask[6] = 0;
		mask[7] = 0;
	}
	param[6] = ptr_dist[2];
	param[7] = ptr_dist[3];

	if(m_bcalib_fix_k1)
		mask[4] = 0;
	param[4] = ptr_dist[0];

	if(m_bcalib_fix_k2)
		mask[5] = 0;
	param[5] = ptr_dist[1];

	if(m_bcalib_fix_k3)
		mask[8] = 0;
	param[8] = ptr_dist[4];

	if(m_bcalib_fix_k4)
		mask[9] = 0;
	param[9] = ptr_dist[5];

	if(m_bcalib_fix_k5)
		mask[10] = 0;
	param[10] = ptr_dist[6];

	if(m_bcalib_fix_k6)
		mask[11] = 0;
	param[11] = ptr_dist[7];

	// setting extrinsics
	param += 12;
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		if(!m_kfrm_used[ikf])
			continue;
		vector<s_obj*> & objs = m_kfrms[ikf]->objs;
		for(int iobj = 0; iobj < objs.size(); iobj++){
			double * pext;
			// copy rvec
			s_obj & obj = *objs[iobj];
			pext = obj.rvec.ptr<double>();
			memcpy((void*)param, (void*)pext, sizeof(double) * 3);

			// copy tvec
			pext = obj.tvec.ptr<double>();
			memcpy((void*)(param + 3), (void*)pext, sizeof(double) * 3);

			param += 6;
		}
	}

	// Compounds of Jacobian
	// J = 
	//     | Cam | Obj1 | Obj2 | ... | Obj n |
	// OP1   C1    OP11
	// OP2   C2           OP22 
	// ...
	// OPn   Cn                         OPnn
	// 
	// Jt = 
	//     | OP1   | OP2   | ... | OP4  |
	// Cam   C1^t    C2^t          C4^t
	// Obj1  OP11^t
	// Obj2         OP22^t 
	// ...
	// Objn                        OPnn^t
	// 
	// Psuedo Hessian (JtJ)
	//      | Cam         |  Obj1   |  Obj2   | .... |  Obj n   |
	// Cam    Hcam         C1^tOP11  C2^tOP22          Cn^tOPnn
	// Obj1   (C1^tOP11)^t  OP11^2
	// Obj2   (C2^tOP22)^t            OP22^2
	// ...
	// Objn   (Cn^tOPnn)^t                              OPnn^2
	//
	// Hcam = C1^2+...Cn^2
	// 
	// Err =
	//     |Err|
	//      OE1  
	//      OE2  
	//       ...
	//      OEn  
	//
	// JtErr
	//     Ecam
	//     OP11^tOE1
	//     OP22^tOE2
	//     ...
	//     OPnn^tOEn
	// Ecam = C1^tOE1 + C2^tOE2 + ... + Cn^tOEn

	const CvMat * _param = NULL;
	double * pparam = NULL;
	int itr = 0;
#ifdef VERB_LM
	ofstream log("levmarq.csv");
#endif
	while(1){
#ifdef VERB_LM
		log << "Iteration " << itr << " state=" << m_solver.state << endl;
#endif
		double * errNorm = NULL;
		CvMat * _JtJ = NULL, *_JtErr = NULL; // for each iteration, these data structure should be initialized.
											 // This is because the ERROR_CHECK state of CvLevMarq assume the 
											 // outerloop does not update their JtJ and JtErr
		param = m_solver.param->data.db;
		pparam = m_solver.prevParam->data.db;
		bool proceed = m_solver.updateAltEx(_param, _JtJ, _JtErr, errNorm);
#ifdef VERB_LM
		//mat2csv(log, Mat(m_solver.JtJN));

		log << "V = " << endl;
		mat2csv(log, Mat(m_solver.JtJV));
		log << "W = " << endl;
		mat2csv(log, Mat(m_solver.JtJW));
#endif
		// if we need to use fixed aspect ratio, this codes should be rewritten to multiply the aspect ratio
		param[0] = param[1];
		pparam[0] = pparam[1];

		// updating intrinsic parameters
		ptr_int[0] = param[0];
		ptr_int[4] = param[1];
		ptr_int[2] = param[2];
		ptr_int[5] = param[3];
		for(int idist = 0; idist < 12; idist++){
			ptr_dist[idist] = param[4 + idist];
		}

		// updating extrinsic parameters 
		param += 12;
		for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
			if(!m_kfrm_used[ikf])
				continue;
			vector<s_obj*> & objs = m_kfrms[ikf]->objs;
			for(int iobj = 0; iobj < objs.size(); iobj++){
				double * pext;
				s_obj & obj = *objs[iobj];
				// copy rvec
				pext = obj.rvec.ptr<double>();
				memcpy((void*)pext, (void*)param, sizeof(double) * 3);

				// copy tvec
				pext = obj.tvec.ptr<double>();
				memcpy((void*)pext, (void*)(param + 3), sizeof(double) * 3);

				obj.update = false;
				param += 6;
			}
		}

		// calculate projection
		double ssd = 0.0;
#ifdef VERB_LM
		log << "Cam_int = " << endl;
		mat2csv(log, m_cam_int);
		log << "Cam_dist = " << endl;
		mat2csv(log, m_cam_dist);
#endif
		bool updateJ = _JtJ != NULL && _JtErr != NULL;
		for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
			if(!m_kfrm_used[ifrm])
				continue;

			// copy camera parameters
			m_cam_int.copyTo(m_kfrms[ifrm]->camint);
			m_cam_dist.copyTo(m_kfrms[ifrm]->camdist);
			m_kfrms[ifrm]->update = false;

			// projection
			m_kfrms[ifrm]->proj_objs(updateJ, m_bcalib_fix_aspect_ratio);

			// accumulating frame's projection ssd
			ssd += m_kfrms[ifrm]->ssd;
		}

		if(!proceed){
			return ;
		}
		if(errNorm){
			*errNorm = sqrt(ssd);
#ifdef VERB_LM
			log << "errNorm," << *errNorm << endl;
#endif
		}
		if(updateJ){
			Mat JtJ(_JtJ);
			Mat JtErr(_JtErr);

			// Calculating Psuedo Hessian JtJ and JtErr
			int i = 12;
			for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
				if(!m_kfrm_used[ifrm])
					continue;
				vector<s_obj*> & objs = m_kfrms[ifrm]->objs;
				for(int iobj = 0; iobj < objs.size(); iobj++, i+=6){
					s_obj & obj = *objs[iobj];
#ifdef VERB_LM
					log << "J[" << ifrm << "][" << iobj <<"] = " << endl;
					mat2csv(log, obj.jacobian);
					// JtJ
					log << "H[" << ifrm << "][" << iobj <<"] = " << endl;
					mat2csv(log, obj.hessian);
#endif
					// JtJFrame
					//      6     12
					// 6   RTRT   CRTt
					// 12  CRT    CC
					//
					// JtJ(0:12,0:12) += Hfrm(6:18, 6:18)
					// JtJ(12+6ifrm:12+6ifrm+6, 12+6ifrm:12+6ifrm+6) = Hfrm(0:6, 0:6)
					// JtJ(12+6ifrm:12+6ifrm+6, 0:12) = Hfrm(0:6, 6:18)
					// JtJ(0:12, 12+6ifrm:12+6ifrm+6) = Hfrm(6:18, 0:6)
					JtJ(Rect(0, 0, 12, 12)) += obj.hessian(Rect(6, 6, 12, 12));
					obj.hessian(Rect(0, 0, 6, 6)).copyTo(JtJ(Rect(i, i, 6, 6)));
					obj.hessian(Rect(6, 0, 12, 6)).copyTo(JtJ(Rect(0, i, 12, 6)));
					obj.hessian(Rect(0, 6, 6, 12)).copyTo(JtJ(Rect(i, 0, 6, 12)));
#ifdef VERB_LM
					// JtErr
					log << "JtErr[" << ifrm << "][" << iobj << "] = " << endl;
					mat2csv(log, obj.jterr);
#endif
					// JtErrFrame
					// 6  ERT
					// 12 ECAM
					JtErr(Rect(0, 0, 1, 12)) += obj.jterr(Rect(0, 6, 1, 12));
					obj.jterr(Rect(0, 0, 1, 6)).copyTo(JtErr(Rect(0, i, 1, 6)));
				}
			}
#ifdef VERB_LM
			log << "JtJ =" << endl;
			mat2csv(log, JtJ);
			Mat JtJN;
			JtJ.copyTo(JtJN);
			double Log10 = ::log(10.0);
			double lambda = exp(m_solver.lambdaLg10 * Log10);
			for(int i = 0; i < JtJN.cols; i++)
				JtJN.at<double>(i, i) *= 1.0 + lambda;
			log << "lambda, " << lambda << endl;
			Mat JtJinv = JtJN.inv(DECOMP_CHOLESKY);
			log << "JtJinv =" << endl;
			mat2csv(log, JtJinv);
			Mat evl, evc;
			eigen(JtJ, evl, evc);
			log << "Eval =" << endl;
			mat2csv(log, evl);
			log << "Evec =" << endl;
			mat2csv(log, evc);
			log << "JtErr =" << endl;
			mat2csv(log, JtErr);
#endif
		}
		itr++;
	}
}

void f_inspector::calc_erep()
{
	if(m_emd == EMD_STOP)
		return;

	m_num_pts_used = 0;
	m_cam_erep = 0.0;
	if(m_emd == EMD_RT && m_pfrm_int){
		vector<s_obj*> & objs = m_pfrm_int->objs;
		for(int iobj = 0; iobj < objs.size(); iobj++){
			m_num_pts_used += objs[iobj]->calc_num_matched_points();
		}
		m_cam_erep += m_pfrm_int->ssd;
	}else{
		for(int ifrm = 0; ifrm < m_kfrm_used.size(); ifrm++){
			if(!m_kfrm_used[ifrm])
				continue;
			vector<s_obj*> & objs = m_kfrms[ifrm]->objs;
			for(int iobj = 0; iobj < objs.size(); iobj++){
				m_num_pts_used += objs[iobj]->calc_num_matched_points();
			}

			m_cam_erep += m_kfrms[ifrm]->ssd;
		}
	}
	if(m_num_pts_used){
		m_cam_erep /= (double) m_num_pts_used;
		m_cam_erep = sqrt(m_cam_erep);
	}else{
		m_cam_erep = 0.;
	}
}


// About Hessian structure
// There are multiple objects with different attitudes, 
// We do not need to combine the hessian (actually JJ^t),
// in stead of that, we can calculate attitude part of the hessian
// independent of that of the other objects. 
// Of course, we need to unify the cammera intrinsic part of the hessian.
void f_inspector::estimate_fulltime()
{
	// full projection with current camera parameters
	double ssd = 0.0;
	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
		// copy camera parameters
		m_cam_int.copyTo(m_kfrms[ifrm]->camint);
		m_cam_dist.copyTo(m_kfrms[ifrm]->camdist);

		// projection
		m_kfrms[ifrm]->proj_objs(true, m_bcalib_fix_aspect_ratio);

		// accumulating frame's projection ssd
		ssd += m_kfrms[ifrm]->ssd;
	}

	// calculating average ssd 
	double ssd_avg = ssd / (double)m_kfrms.size();
	double cam_erep = sqrt(ssd_avg);
	double diff = m_cam_erep - cam_erep;
	if(diff < -ERROR_TOL){
		m_eest = EES_DIV;
		return;
	}else if(diff < ERROR_TOL){
		m_eest = EES_CONV;
		return;
	}else{
		m_eest = EES_CONT;
	}

	m_cam_erep = cam_erep;

	// calcurate variance of the ssd
	ssd = 0;
	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
		double ssdd = (m_kfrms[ifrm]->ssd - ssd_avg);
		ssd += ssdd * ssdd;
	}
	ssd /= (double)m_kfrms.size();
	double s_ssd = sqrt(ssd);
	double ssd_range = s_ssd * (double) m_err_range; 

	// calculating valid flag. if the flag is asserted, the frame is used for the optimization
	int num_valid_frms = min((int)m_kfrms.size(), m_num_max_frms_used);
	double step =  (double)m_kfrms.size() / (double) num_valid_frms; 
	m_kfrm_used.resize(m_kfrms.size(), false);
	num_valid_frms = 0;
	for(double ifrm = 0.; ifrm < (double) m_kfrms.size(); ifrm += step){
		int i = (int) ifrm;
		if(m_kfrms[i]->ssd - ssd_avg < ssd_range){
			m_kfrm_used[i] = true;
			num_valid_frms++;
		}
	}

	// current frame is forced to be used for the optimization
	m_kfrm_used[m_cur_kfrm] = true;
	num_valid_frms++;

	// accumulating camera intrinsics part of the hessian
	Mat Hcamint = Mat::zeros(12, 12, CV_64FC1);
	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
		if(!m_kfrm_used[ifrm])
			continue;
		acc_Hcamint(Hcamint, m_kfrms[ifrm]->objs);
	}

	// copying camera intrinsic part 
	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
		if(!m_kfrm_used[ifrm])
			continue;
		copy_Hcamint(Hcamint, m_kfrms[ifrm]->objs);
	}

	for(int ifrm = 0; ifrm < m_kfrms.size(); ifrm++){
		if(!m_kfrm_used[ifrm])
			continue;
		update_params(m_kfrms[ifrm]->objs);
	}	
}

void s_frame::proj_objs(bool bjacobian, bool fix_aspect_ratio)
{
	ssd = 0.0;
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		if(update && obj.update)
			continue;
		obj.proj(camint, camdist, bjacobian, fix_aspect_ratio);
	
		ssd += obj.ssd;
	}
}

// calc_rpy calculates roll pitch yaw serge sway heave of the objects relative to the base_obj.
void s_frame::calc_rpy(int base_obj)
{
	if(update)
		return;

	if(base_obj >= 0 && base_obj < objs.size()){
		Mat Rorg, R, T;
		Mat & Torg = objs[base_obj]->tvec;
		Rodrigues(objs[base_obj]->rvec, Rorg);
		Rorg = Rorg.t();
		for(int iobj = 0; iobj < objs.size(); iobj++){
			s_obj & obj = *objs[iobj];

			if(iobj == base_obj){
				obj.roll = obj.pitch = obj.yaw = 0.;
				obj.pos.x = obj.pos.y = obj.pos.z = 0.;
				continue;
			}

			double * p0;
			Rodrigues(obj.rvec, R);
			obj.tvec.copyTo(T);

			// translation Relative to the base_obj in the camera coordinate frame
			T -= Torg;

			// projection to the object's coordinate frame
			T = Rorg * T;

			p0 = T.ptr<double>();
			obj.pos.x = (float)(p0[0]);
			obj.pos.y = (float)(p0[1]);
			obj.pos.z = (float)(p0[2]);

			// Calculate relative rotation to the base_obj as R
 			R = Rorg * R;
			p0 = R.ptr<double>();

			angleRxyz(p0, obj.roll, obj.pitch, obj.yaw);
		}
	}
}

void f_inspector::acc_Hcamint(Mat & Hcamint, vector<s_obj*> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		// accumulating camera intrinsic part of hessians of all objects
		Hcamint += obj.hessian(Rect(6, 6, 12, 12));
	}
}

void f_inspector::copy_Hcamint(Mat & Hcamint, vector<s_obj*> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		Hcamint.copyTo(obj.hessian(Rect(6, 6, 12, 12)));
	}
}

// Since we do not update all parameters according to flags,
// we need to calculate indices of the parameters to calculate hessian.
void f_inspector::make_param_indices(vector<int> & ipars)
{
	int ipar;

	// indexing parameters
	int num_params = 0;
	// counting phase
	num_params = 6; // rvec, tvec;
	if(!m_bcalib_fix_campar){
		num_params += 2;
		if(!m_bcalib_fix_principal_point)
			num_params += 2; // cx cy

		if(!m_bcalib_zero_tangent_dist)
			num_params += 2; // px py

		if(!m_bcalib_fix_k1)
			num_params += 1;
		if(!m_bcalib_fix_k2)
			num_params += 1;
		if(!m_bcalib_fix_k3)
			num_params += 1;
		if(!m_bcalib_fix_k4)
			num_params += 1;
		if(!m_bcalib_fix_k5)
			num_params += 1;
		if(!m_bcalib_fix_k6)
			num_params += 1;
	}

	// allocating phase
	ipars.resize(num_params);

	// loading phase
	for(ipar = 0; ipar< 6; ipar++)
		ipars[ipar] = ipar;

	if(!m_bcalib_fix_focus){
		ipars[ipar] = 6; ipar++;
		ipars[ipar] = 7; ipar++;
	}
	if(!m_bcalib_fix_principal_point){
		ipars[ipar] = 8; ipar++;
		ipars[ipar] = 9; ipar++;
	}

	if(!m_bcalib_fix_k1){
		ipars[ipar] = 10; ipar++;
	}
	num_params += 1;
	if(!m_bcalib_fix_k2){
		ipars[ipar] = 11; ipar++;
	}

	if(!m_bcalib_zero_tangent_dist){
		ipars[ipar] = 12; ipar++;
		ipars[ipar] = 13; ipar++;
	}

	if(!m_bcalib_fix_k3){
		ipars[ipar] = 14; ipar++;
	}

	if(!m_bcalib_fix_k4){
		ipars[ipar] = 15; ipar++;
	}

	if(!m_bcalib_fix_k5){
		ipars[ipar] = 16; ipar++;
	}

	if(!m_bcalib_fix_k6){
		ipars[ipar] = 17; ipar++;
	}
}

void f_inspector::update_params(vector<s_obj*> & objs)
{
	vector<int> ipars;
	make_param_indices(ipars);
	Mat H, Hinv, Jt, dp;
	H = Mat((int)ipars.size(), (int)ipars.size(), CV_64FC1);

	// updating each object
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];

		// setting reduced Hessian. 
		// In this loop ipar, jpar represents reduced hessian row,col indices, 
		// and i, j represents original hessian's row, col indices.
		for(int ipar = 0; ipar < ipars.size(); ipar++){
			int i = ipars[ipar];
			double * ptr0 = obj.hessian.ptr<double>(i);
			double * ptr1 = H.ptr<double>(ipar);
			for(int jpar = 0; jpar < ipars.size(); jpar++){
				int j = ipars[jpar];
				ptr1[jpar] = ptr0[j];
			}
		}

		// settig reduced transposed Jacobian
		Jt = Mat((int)ipars.size(), obj.jacobian.rows, CV_64FC1);
		for(int ipar = 0; ipar < ipars.size(); ipar++){
			int i = ipars[ipar];
			double * ptr0 = obj.jacobian.ptr<double>(0) + i;
			double * ptr1 = Jt.ptr<double>(ipar);
			for(int jpar = 0; jpar < Jt.cols; jpar++, ptr0 += obj.jacobian.cols){
				ptr1[jpar] = *ptr0;
			}
		}

		//double det = determinant(obj.hessian);
		//eigen(obj.hessian, eigenval, eigenvec);

		// calculating Hessian inverse
		invert(H, Hinv, DECOMP_CHOLESKY);
		
		// calculating parameter changes
		Mat Grad = Jt * obj.err;
		dp = Hinv * Grad;

		cout << "Hinv=" << Hinv;
		cout << "dp=" << dp << endl;

		// updating parameters
		double * ptr_dp = dp.ptr<double>(0);
		double * ptr; 
		ptr = obj.rvec.ptr<double>(0);
		ptr[0] += ptr_dp[0]; // rx
		ptr[1] += ptr_dp[1]; // ry
		ptr[2] += ptr_dp[2]; // rz
		ptr = obj.tvec.ptr<double>(0);
		ptr[0] += ptr_dp[3]; // tx
		ptr[1] += ptr_dp[4]; // ty
		ptr[2] += ptr_dp[5]; // tz

		int ipar = 6;
		if(!m_bcalib_fix_focus){
			ptr = m_cam_int.ptr<double>(0);
			ptr[0] += ptr_dp[ipar]; ipar++;
			ptr[4] += ptr_dp[ipar]; ipar++;
		}
		if(!m_bcalib_fix_principal_point){
			ptr[2] += ptr_dp[ipar]; ipar++;
			ptr[5] += ptr_dp[ipar]; ipar++;
		}

		ptr = m_cam_dist.ptr<double>(0);

		if(!m_bcalib_fix_k1){
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}
		if(!m_bcalib_fix_k2){
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}

		if(!m_bcalib_zero_tangent_dist){
			ptr[ipar] += ptr_dp[ipar]; ipar++;
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}

		if(!m_bcalib_fix_k3){			
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}

		if(!m_bcalib_fix_k4){			
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}

		if(!m_bcalib_fix_k5){			
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}

		if(!m_bcalib_fix_k6){			
			ptr[ipar] += ptr_dp[ipar]; ipar++;
		}
	}
}

void f_inspector::calc_jmax(){
	m_jcam_max = Mat::zeros(1, 12, CV_64FC1);
	double * ptr_max, * ptr;
	vector<s_obj*> & objs = m_pfrm_int->objs;
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		obj.jmax = Mat::zeros(1, obj.jacobian.cols, CV_64FC1);
		for(int i = 0; i < obj.jacobian.rows; i++){
			ptr = obj.jacobian.ptr<double>(i);
			ptr_max = obj.jmax.ptr<double>(0);
			for(int j = 0; j < obj.jacobian.cols; j++, ptr++, ptr_max++)
				*ptr_max = max(*ptr_max, abs(*ptr));
		}

		ptr = obj.jmax.ptr<double>(0) + 6;
		ptr_max = m_jcam_max.ptr<double>(0);
		for(int i = 0; i < 12; i++, ptr++, ptr_max++){
			*ptr_max = max(*ptr_max, *ptr);
		}
	}
}


///////////////////////////////////////////////////////// message handler

void f_inspector::handle_lbuttondown(WPARAM wParam, LPARAM lParam)
{
	extractPointlParam(lParam, m_mc);
	switch(m_op){
	case OBJ:
		if(GET_KEYSTATE_WPARAM(wParam) & MK_SHIFT){ // scroll
			m_mm = MM_SCROLL;
			m_pt_sc_start = m_mc;
		}
		break;
	case POINT:
		if(GET_KEYSTATE_WPARAM(wParam) & MK_SHIFT){ // scroll
			m_mm = MM_SCROLL;
			m_pt_sc_start = m_mc;
		}else{
			m_mm = MM_POINT;
		}
		break;
	case MODEL:
		break;
	}
}

void f_inspector::handle_lbuttonup(WPARAM wParam, LPARAM lParam)
{
	extractPointlParam(lParam, m_mc);
	switch(m_mm){
	case MM_SCROLL:
		scroll_screen();
		break;
	case MM_POINT:
		assign_point2d();
		break;
	}
	m_mm = MM_NORMAL;
};

void f_inspector::handle_mousemove(WPARAM wParam, LPARAM lParam)
{
	extractPointlParam(lParam, m_mc);
	switch(m_mm){
	case MM_SCROLL:
		scroll_screen();
		break;
	default:
		break;
	}
}

void f_inspector::handle_mousewheel(WPARAM wParam, LPARAM lParam)
{
	// Notice: Screen coordinate in WM_MOUSEWHEEL is different from other 
	// mouse related message. We need to subtract origin of the client screen
	// from the point sent by the message. 
	extractPointlParam(lParam, m_mc);
	ScreenToClient(m_hwnd, (LPPOINT)&m_mc);
//	RECT rc;
//	GetWindowRect(m_hwnd, &rc);
//	m_mc.x -= (rc.left + m_client_org.x);
//	m_mc.y -= (rc.top + m_client_org.y);

	short delta = GET_WHEEL_DELTA_WPARAM(wParam);
	if(GET_KEYSTATE_WPARAM(wParam) & MK_SHIFT){
		switch(m_op){
		case OBJ:
		case POINT:
		case VIEW3D:
			zoom_screen(delta);
			break;
		}
	}else if(GET_KEYSTATE_WPARAM(wParam) & MK_CONTROL){
		switch(m_op){
		case OBJ:
		case POINT:
		case VIEW3D:
			rotate_obj(delta);
			break;
		}
	}else{
		switch(m_op){
		case OBJ:
		case POINT:
		case VIEW3D:
			translate_obj(delta);
			break;
		case CAMERA:
			adjust_cam(delta);
			// change the focal length of the camera
			break;
		case PARTS:
			adjust_part(delta);
			break;
		}
	}
}

void f_inspector::handle_keydown(WPARAM wParam, LPARAM lParam)
{
	switch(wParam){
	case VK_DELETE:
		m_sop = SOP_DELETE;
		break;
	case VK_UP:
		handle_vk_up();
		break;
	case VK_DOWN:
		handle_vk_down();
		break;
	case VK_LEFT:
		handle_vk_left();
		break;
	case VK_RIGHT:
		handle_vk_right();
		break;
	default:
		break;
	}
}

void f_inspector::assign_point2d()
{
	if(!m_pfrm_int)
		return;

	vector<s_obj*> & objs = m_pfrm_int->objs;
	if(objs.size() == 0)
		return;

	if(m_cur_point < 0)
		return;

	Point2f pt;
	cnv_view2img(m_mc, pt);
	s_obj & obj = *objs[m_cur_obj];
	obj.pt2d[m_cur_point] = pt;
	obj.visible[m_cur_point] = 1;
}

void f_inspector::scroll_screen()
{
	m_main_offset.x += (float)(m_mc.x - m_pt_sc_start.x);
	m_main_offset.y += (float)(m_mc.y - m_pt_sc_start.y);
	m_pt_sc_start = m_mc;
}

void f_inspector::shift_cam_center()
{
	m_cam_int.at<double>(0, 2) += (float)(m_mc.x - m_pt_sc_start.x);
	m_cam_int.at<double>(1, 2) += (float)(m_mc.y - m_pt_sc_start.y);
}

void f_inspector::zoom_screen(short delta)
{
	short step = delta / WHEEL_DELTA;
	float scale = (float) pow(1.1, (double) step);
	if(m_op == VIEW3D){
		m_rat_z *= scale;
	}else{
		m_main_scale *=  (float) scale;
		m_main_scale_inv = (float) (1.0 / m_main_scale);
		float x, y;
		x = (float)(m_main_offset.x - m_mc.x) * scale;
		y = (float)(m_main_offset.y - m_mc.y) * scale;
		m_main_offset.x =  (float)(x + (double) m_mc.x);
		m_main_offset.y =  (float)(y + (double) m_mc.y);
	}
}

void f_inspector::translate_obj(short delta)
{
	if(m_cur_obj < 0 || !m_pfrm_int)
		return ;
	vector<s_obj*> & objs = m_pfrm_int->objs;
	s_obj & obj = *objs[m_cur_obj];
	double * ptr = obj.jmax.ptr<double>(0) + 3;
	double step = (double)(delta / WHEEL_DELTA) * m_adj_step;
	double div = ptr[m_axis];
	if(div != div){// to avoid Not a number
		div = 1.0;
		cout << "Not a number detected." << endl;
	}
	
	Mat tvec = Mat::zeros(3, 1, CV_64FC1);

	switch(m_axis){
	case AX_X:
		tvec.at<double>(0, 0) = step / div;
		break;
	case AX_Y:
		tvec.at<double>(1, 0) = step / div;
		break;
	case AX_Z:
		tvec.at<double>(2, 0) = step / div;
		break;
	}
	if(m_op == VIEW3D){
		m_tvec_view += tvec;
	}else{
		obj.tvec += tvec;
		double * tptr = obj.tvec.ptr<double>(0);
		if(tptr[m_axis] != tptr[m_axis]){
			tptr[m_axis] = 1.0;
			cout << "Not a number detected." << endl;
		}
		m_pfrm_int->update = false;
		obj.update = false;
	}
}

void f_inspector::rotate_obj(short delta)
{
	if(m_cur_obj < 0 || !m_pfrm_int)
		return;	

	vector<s_obj*> & objs = m_pfrm_int->objs;
	// m_rot_step degree per wheel step
	double step = (double) (delta / WHEEL_DELTA) * (CV_PI / 180.) * m_adj_step;
	Mat rvec = Mat::zeros(3, 1, CV_64FC1);
	switch(m_axis){
	case AX_X:
		rvec.at<double>(0, 0) = step;
		break;
	case AX_Y:
		rvec.at<double>(1, 0) = step;
		break;
	case AX_Z:
		rvec.at<double>(2, 0) = step;
		break;
	}
	if(m_op == VIEW3D){
		Mat R1, R2;
		Rodrigues(m_rvec_view, R1);
		Rodrigues(rvec, R2);
		Mat R = R2 * R1;
		Rodrigues(R, m_rvec_view);
	}else{
		s_obj & obj = *objs[m_cur_obj];
		Mat R1, R2;
		Rodrigues(obj.rvec, R1);
		Rodrigues(rvec, R2);
		Mat R = R2 * R1;
		Rodrigues(R, obj.rvec);
		m_pfrm_int->update = false;
		obj.update = false;
	}
}

void f_inspector::adjust_part(short delta)
{
	if(!m_pfrm_int)
		return ;
	s_frame & fobj = *m_pfrm;

	if(m_cur_obj < 0 || m_cur_obj >= fobj.objs.size())
		return;

	s_obj & obj = *m_pfrm_int->objs[m_cur_obj];
	if(m_cur_part < 0 || m_cur_part >= obj.dpart.size())
		return;

	obj.dpart[m_cur_part] += m_adj_step * delta / WHEEL_DELTA;
	m_pfrm_int->update = false;
}

void f_inspector::adjust_cam(short delta)
{
	short step = delta / WHEEL_DELTA;
	double * ptr = m_jcam_max.ptr<double>(0);
	//float scale = (float) pow(m_zoom_step, (double) step);
	switch(m_cur_campar){
	case ECP_FX:
	case ECP_FY:
		m_cam_int.at<double>(0, 0) += m_adj_step * step / ptr[m_cur_campar];
		m_cam_int.at<double>(1, 1) += m_adj_step * step / ptr[m_cur_campar];
		break;
	case ECP_CX:
		m_cam_int.at<double>(0, 2) += m_adj_step * step / ptr[m_cur_campar];
		break;
	case ECP_CY:
		m_cam_int.at<double>(1, 2) += m_adj_step * step / ptr[m_cur_campar];
		break;
	case ECP_K1:
	case ECP_K2:
	case ECP_P1:
	case ECP_P2:
	case ECP_K3:
	case ECP_K4:
	case ECP_K5:
	case ECP_K6:
		{
			double * ptr_dist = m_cam_dist.ptr<double>(0);
			ptr_dist[m_cur_campar - ECP_K1] += m_adj_step * step / ptr[m_cur_campar];
		}
		break;
	}
	if(m_pfrm_int)
		m_pfrm_int->update = false;
}

void f_inspector::handle_vk_up()
{
	m_adj_pow += 1;
	m_adj_step = pow(10., (double)m_adj_pow);
}

void f_inspector::handle_vk_down()
{
	m_adj_pow -= 1;
	m_adj_step = pow(10., (double)m_adj_pow);
}

void f_inspector::handle_vk_left()
{	
	if(!m_pfrm_int){
		return;
	}

	vector<s_obj*> & objs = m_pfrm_int->objs;
	switch(m_op){
	case OBJ:
	case VIEW3D:
		{
			if(objs.size() == 0)
				return ;

			m_cur_obj = m_cur_obj - 1;
			if(m_cur_obj < 0){
				m_cur_obj += (int) objs.size();
			}
			// the current object point index is initialized
			m_cur_point = objs[m_cur_obj]->get_num_points() - 1;
			m_pfrm_int->set_update();
		}
		break;
	case PARTS:
		{
			if(m_cur_obj < 0 || m_cur_obj >= objs.size())
				return;
			vector<double> & dpart = objs[m_cur_obj]->dpart;
			if(m_cur_part < 0 || m_cur_part >= dpart.size())
				return;
			m_cur_part--;
			if(m_cur_part < 0)
				m_cur_part += (int) dpart.size();
		}
		break;
	case POINT: // decrement current object point. 3d-object point is also. 
		if(m_cur_obj < 0)
			return;
		m_cur_point = m_cur_point  - 1;
		if(m_cur_point < 0){
			m_cur_point += (int) objs[m_cur_obj]->get_num_points();
		}
		break;
	case MODEL: // decrement the current model index
		if(m_models.size() == 0)
			return;
		m_cur_model = m_cur_model - 1;
		if(m_cur_model < 0){
			m_cur_model += (int) m_models.size();
		}
		break;
	case CAMERA:
		m_cur_campar = m_cur_campar - 1;
		if(m_cur_campar < 0){
			m_cur_campar = ECP_K6;
		}
		break;
	case CAMTBL:
		if(m_cam_int_tbl.size()){
			m_cur_camtbl = m_cur_camtbl - 1;
			if(m_cur_camtbl < 0)
				m_cur_camtbl = (int) m_cam_int_tbl.size() - 1;
		}
		break;
	case FRAME:
		{
			snprintf(m_cmd_buf, CMD_LEN, "step c -%d", m_int_cyc_kfrm);
			m_sop = SOP_AWSCMD;
		}
		break;
	}
}

void f_inspector::handle_vk_right()
{
	if(!m_pfrm_int)
		return;

	vector<s_obj*> & objs = m_pfrm_int->objs;
	switch(m_op){
	case OBJ:
	case VIEW3D:
		{
			if(objs.size() == 0)
				return ;
			m_cur_obj = m_cur_obj + 1;
			m_cur_obj %= (int) objs.size();
			s_obj & obj = *objs[m_cur_obj];
			m_cur_point = obj.get_num_points() - 1;
			m_pfrm_int->set_update();
		}
		break;
	case PARTS:
		{
			if(m_cur_obj < 0 || m_cur_obj >= objs.size())
				return;
			vector<double> & dpart = objs[m_cur_obj]->dpart;
			if(m_cur_part < 0 || m_cur_part >= dpart.size())
				return;
			m_cur_part++;
			m_cur_part %= (int) dpart.size();
		}
	case POINT: // increment the current object point index. The 3d-object point index as well.
		{
			if(m_cur_obj < 0)
				return;
			s_obj & obj = *objs[m_cur_obj];
			m_cur_point = m_cur_point + 1;
			m_cur_point %= (int) obj.get_num_points();
		}
		break;
	case MODEL: // increment the current model index.
		if(m_models.size() == 0)
			return;
		m_cur_model = m_cur_model + 1;
		m_cur_model %= (int) m_models.size();
		break;
	case CAMERA:
		m_cur_campar = m_cur_campar + 1;
		m_cur_campar %= (ECP_K6 + 1);
		break;
	case CAMTBL:
		if(m_cam_int_tbl.size()){
			m_cur_camtbl = m_cur_camtbl + 1;
			m_cur_camtbl %= m_cam_int_tbl.size();
		}
		break;
	case FRAME:
		{
			snprintf(m_cmd_buf, CMD_LEN, "step c %d", m_int_cyc_kfrm);
			m_sop = SOP_AWSCMD;
		}
		break;
	}
}

void f_inspector::handle_char(WPARAM wParam, LPARAM lParam)
{
	switch(wParam){
	case 'C':
		m_op = CAMERA;
		break;
	case 'D':
		if(m_op == MODEL){
			m_sop = SOP_DET;
		}
		break;
	case 'E':
		m_op = ESTIMATE;
		m_emd = EMD_STOP;
		break;
	case 'F':
		m_op = FRAME;
		break;
	case 'G':
		switch(m_op){
		case MODEL:
		case FRAME:
		case OBJ:
			m_sop = SOP_GUESS;
			break;
		}
		break;
	case 'I':
		switch(m_op){
		case MODEL:
			m_sop = SOP_INST_OBJ;
			break;
		case CAMERA:
		case CAMTBL:
			m_sop = SOP_INS_CPTBL;
			break;
		}
		break;
	case 'K':
		m_bkfrm = !m_bkfrm;
		m_sel_kfrm = m_cur_kfrm;
		break;
	case 'L':
		m_sop = SOP_LOAD;
		break;
	case 'M':
		m_op = MODEL;
		break;
	case 'O': /* O key */ 
		m_op = OBJ;
		if(m_cur_obj < 0 && m_pfrm_int)
			m_cur_obj = (int) m_pfrm_int->objs.size() - 1;
		break;
	case 'P':
		m_op = POINT;
		break;
	case 'Q':
		m_op = PARTS;
		if(m_cur_obj >= 0 && m_pfrm_int && m_cur_obj < m_pfrm_int->objs.size()){
			m_cur_part = (int) m_pfrm_int->objs[m_cur_obj]->dpart.size() - 1;
		}
		break;
	case 'R': // reset window
		if(m_op == VIEW3D){
			m_rat = 1.0;
		}else{
			m_main_offset = Point2f(0., 0.);
			m_main_scale = (float) m_rat;
			m_main_scale_inv = (float)(1.0 / m_main_scale);
		}
		break;
	case 'S':
		m_sop = SOP_SAVE;
		break;
	case 'T':
		m_op = CAMTBL;
		break;
	case 'V':
		m_op = VIEW3D;
		break;
	case 'f':
		handle_char_f();
		break;
	case 's':
		if(m_op == CAMTBL && m_pfrm_int){
			if(m_cur_camtbl >= 0 && m_cur_camtbl < m_cam_int_tbl.size()){
				m_cam_int_tbl[m_cur_camtbl].copyTo(m_cam_int);
				m_cam_dist_tbl[m_cur_camtbl].copyTo(m_cam_dist);
				m_pfrm_int->update = false;
			}
		}
		break;
	case 'k':
		m_sop = SOP_SET_KF;
		break;
	case 'x':
		m_axis = AX_X;
		break;
	case 'y':
		m_axis = AX_Y;
		break;
	case 'z':
		m_axis = AX_Z;
		break;
	case '>':
		if(m_bkfrm){
			m_sel_kfrm += 1;
			m_sel_kfrm %= m_kfrms.size();
		}else{
			snprintf(m_cmd_buf, CMD_LEN, "step c %d", m_int_cyc_kfrm);
			m_sop = SOP_AWSCMD;
		}
		break;
	case '<':
		if(m_bkfrm){
			m_sel_kfrm -= 1;
			if(m_sel_kfrm < 0)
				m_sel_kfrm += m_kfrms.size();
		}else{
			snprintf(m_cmd_buf, CMD_LEN, "step c -%d", m_int_cyc_kfrm);
			m_sop = SOP_AWSCMD;
		}
		break;
	case '1':
		if(m_op == ESTIMATE){
			m_emd = EMD_FULL;
		}
		else if (m_op == VIEW3D){
			m_ev = EV_CAM;
		}
		break;
	case '2':
		if(m_op == ESTIMATE){
			m_emd = EMD_RT;
		}
		else if (m_op == VIEW3D){
			m_ev = EV_OBJX;
		}
		break;
	case '3':
		if(m_op == ESTIMATE){
			m_emd = EMD_RT_FULL;
		}
		else if (m_op == VIEW3D){
			m_ev = EV_OBJY;
		}
		break;
	case '4':
		if(m_op == ESTIMATE){
			m_emd = EMD_SEL;
		}else if (m_op == VIEW3D){
			m_ev = EV_OBJZ;
		}
		break;
	case '5':
		if (m_op == VIEW3D){
			m_ev = EV_FREE;
		}
		break;
	default:
		break;
	}
}

void f_inspector::handle_char_f()
{
	// fix parameter
	switch(m_op){
	case OBJ: 
	case POINT:
		// fix selected object's attitude
		break;
	case CAMERA:
		// fix selected camera parameter
		switch(m_cur_campar){
		case ECP_FX:
		case ECP_FY:
			m_bcalib_fix_focus = !m_bcalib_fix_focus;
			break;
		case ECP_CX:
		case ECP_CY:
			m_bcalib_fix_principal_point = !m_bcalib_fix_principal_point;
			break;
		case ECP_K1:
			m_bcalib_fix_k1 = !m_bcalib_fix_k1;
			break;
		case ECP_K2:
			m_bcalib_fix_k2 = !m_bcalib_fix_k2;
			break;
		case ECP_P1:
		case ECP_P2:
			m_bcalib_zero_tangent_dist = !m_bcalib_zero_tangent_dist;
			break;
		case ECP_K3:
			m_bcalib_fix_k3 = !m_bcalib_fix_k3;
			break;
		case ECP_K4:
			m_bcalib_fix_k4 = !m_bcalib_fix_k4;
			break;
		case ECP_K5:
			m_bcalib_fix_k5 = !m_bcalib_fix_k5;
			break;
		case ECP_K6:
			m_bcalib_fix_k6 = !m_bcalib_fix_k6;
			break;
		}
		break;
	default:
		break;
	}
}

void f_inspector::handle_sop_delete(){
	if(!m_pfrm_int)
		return;
	vector<s_obj*> & objs = m_pfrm_int->objs;
	switch(m_op){
	case MODEL:
		// delete current Model
		if(m_cur_model >= 0){
			vector<s_model*>::iterator itr = m_models.begin() + m_cur_model;
			if((*itr)->ref > 0)
				break;
			delete (*itr);
			m_models.erase(itr);
			m_cur_model = (int) m_models.size() - 1;
		}
		break;
	case OBJ:
		// delete current object
		if(m_cur_obj >= 0){
			vector<s_obj*>::iterator itr = objs.begin() + m_cur_obj;
			(*itr)->pmdl->ref--;
			objs.erase(itr);
			m_cur_obj = (int) objs.size() - 1;
		}
		break;
	case POINT:
		//reset current point
		if(m_cur_obj >= 0 && m_cur_point >= 0){
			s_obj & obj = *objs[m_cur_obj];
			if(m_cur_point < obj.get_num_points())
				obj.visible[m_cur_point] = 0;
		}
		break;
	case CAMERA:
	case CAMTBL:
		// delete current camera parameter
		delCamparTbl();
		break;
	}
	m_sop = SOP_NULL;
}

void f_inspector::handle_sop_save()
{
	switch(m_op){
	case OBJ:
	case POINT:
		m_pfrm_int->save(m_name);
		break;
	case CAMERA:
	case CAMTBL:
		saveCamparTbl();
		break;
	case FRAME:
		if(!save_kfrms()){
			cerr << "Failed to save fobjs." << endl;
		}
		break;
	}	
	m_sop = SOP_NULL;
}

bool f_inspector::save_kfrms()
{
	ofstream file;
	char fname[1024];
	snprintf(fname, 1024, "%s.inspector", m_name);
	file.open(fname);
	if(!file.is_open())
		return false;

	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		if(!m_kfrms[ikf])
			continue;
		m_kfrms[ikf]->save(m_name);

		file << m_kfrms[ikf]->tfrm << endl;
	}

	return true;
}

void f_inspector::handle_sop_load()
{
	switch(m_op){
	case MODEL:
		load_model();
		break;
	case OBJ:
	case POINT:
	case CAMERA:
	case CAMTBL:
		loadCamparTbl();
		break;
	case FRAME:	
		if(!load_kfrms()){
			cerr << "Failed to load fobjs." << endl;
		}
		break;
	}
	m_sop = SOP_NULL;
}

bool f_inspector::load_kfrms()
{
	ifstream file;
	char fname[1024];
	snprintf(fname, 1024, "%s.inspector", m_name);
	file.open(fname);
	if(!file.is_open())
		return false;
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		s_frame::free(m_kfrms[ikf]);
		m_kfrms[ikf] = NULL;
	}

	int ikf = 0;
	m_cur_kfrm = -1;

	long long tfrm_prev = 0;
	while(!file.eof()){
		file.getline(fname, 1024);
		long long tfrm = atoll(fname);
		if(tfrm <= tfrm_prev){
			continue;
		}
		tfrm_prev = tfrm;

		if(m_kfrms[ikf] != NULL){
			if(m_cur_kfrm > 0)// current key frame is found and the cache of key frame is full.
				break;
		}else{
			s_frame::free(m_kfrms[ikf]); 
			m_kfrms[ikf] = NULL;
		}

		// first key frame with the time exceeding current time is the current key frame
		if(m_cur_kfrm < 0 && tfrm >= m_cur_time){
			m_cur_kfrm = ikf;
		}

		m_kfrms[ikf] = s_frame::alloc();
		m_kfrms[ikf]->tfrm = tfrm;
		ikf = (ikf + 1) % m_num_kfrms;
	}

	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		if(m_kfrms[ikf] == NULL)
			break;
		m_kfrms[ikf]->load(m_name, m_kfrms[ikf]->tfrm, m_models);
	}

	if(m_pfrm->tfrm == m_kfrms[m_cur_kfrm]->tfrm && m_pfrm != m_kfrms[m_cur_kfrm]){
		s_frame::free(m_pfrm);
		m_pfrm = m_kfrms[m_cur_kfrm];
	}

	return true;
}

void f_inspector::handle_sop_inst_obj()
{
	if(!m_pfrm_int)
		return;

	vector<s_obj*> & objs = m_pfrm_int->objs;
	if(m_op == MODEL){
		if(m_cur_model < 0){
			return;
		}

		double width =(double) m_maincam.get_surface_width();
		double height = (double) m_maincam.get_surface_height();
		s_obj * pobj = new s_obj;
		if(!pobj->init(m_models[m_cur_model], m_timg, m_cam_int, m_cam_dist, width, height)){
			delete pobj;
			cerr << "Failed to create an instance of model " << m_models[m_cur_model]->name << endl;		
		}else{
			objs.push_back(pobj);
			m_cur_obj = (int) objs.size() - 1;
			m_cur_point = 0;
			m_op = OBJ;
		}
	}
	m_sop = SOP_NULL;
}

void f_inspector::handle_sop_guess()
{
	// translation of objects 
	// the depth of the objects are all set as (dmin + dmax) / 2
	int num_objs;
	double cx, cy;
	cx = m_cam_int.ptr<double>()[2];
	cy = m_cam_int.ptr<double>()[5];
	double fx, fy, sfx, sfy;
	double z = 0.5 * (m_depth_min + m_depth_max);
	Rect bb;
	sfx = sfy = 0.0;
	num_objs = 0;
	switch(m_op){
	case FRAME:
		for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
			if(!m_kfrms[ikf])
				continue;
			vector<s_obj*> & objs = m_kfrms[ikf]->objs;
			m_kfrms[ikf]->update = false;
			for(int iobj = 0; iobj < objs.size(); iobj++){
				help_guess(*objs[iobj], z, cx, cy, sfx, sfy, m_bcalib_fix_aspect_ratio);
				num_objs++;
			}
		}
	case OBJ:
		if(m_cur_obj >= 0 && m_pfrm_int){
			vector<s_obj*> & objs = m_pfrm_int->objs;
			if(m_cur_obj < objs.size()){
				help_guess(*objs[m_cur_obj], z, cx, cy, sfx, sfy);
				objs[m_cur_obj]->update = false;
				num_objs++;
			}else{
				return;
			}
		}
	}
	fx = sfx / (double) num_objs;
	fy = sfy / (double) num_objs;
	if(m_bcalib_fix_aspect_ratio){
		fx = fy * m_cam_int.ptr<double>()[0] / m_cam_int.ptr<double>()[4];
	}
	m_cam_int.ptr<double>()[0] = fx;
	m_cam_int.ptr<double>()[4] = fy;
	m_sop = SOP_NULL;

	switch(m_op){
	case FRAME:
		for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
			if(!m_kfrms[ikf])
				continue;

			m_cam_int.copyTo(m_kfrms[ikf]->camint);
			m_cam_dist.copyTo(m_kfrms[ikf]->camdist);
			m_kfrms[ikf]->proj_objs(true, m_bcalib_fix_aspect_ratio);
		}
		break;
	case OBJ:
		if(m_cur_obj >= 0 && m_pfrm_int){
			m_cam_int.copyTo(m_pfrm_int->camint);
			m_cam_dist.copyTo(m_pfrm_int->camdist);
		}
		break;
	}
}

void f_inspector::help_guess(s_obj & obj, double z, double cx, double cy, double & sfx, double & sfy, bool fix_aspect_ratio)
{
	double * ptr = obj.tvec.ptr<double>(0);
	s_model * pmdl = obj.pmdl;
	double sx = pmdl->get_xsize();
	double sy = pmdl->get_ysize();
	ptr[2] = z;
	Rect bb;
	obj.get_bb_pt2d(bb);
	double fx = (double) bb.width * z / (double) sx;
	double fy = (double) bb.height * z / (double) sy;

	ptr[0] = ((double) bb.x + (double) bb.width * (-pmdl->xmin / sx) - cx) / fx;
	ptr[1] = ((double) bb.y + (double) bb.height * (-pmdl->ymin / sy) - cy) / fy;

	sfx += fx;
	sfy += fy;
}


void f_inspector::handle_sop_ins_cptbl()	
{
	addCamparTbl();
	m_sop = SOP_NULL;
}

void f_inspector::handle_sop_det(){
	if(!m_pfrm_int)
		return;

	switch(m_op){
	case MODEL:
		if(m_cur_model >= 0 && m_cur_model < m_models.size()){
			s_obj * pobj;
			s_model * pmdl = m_models[m_cur_model];
			pobj = pmdl->detect(m_img_gry);
			if(pobj == NULL)
				return;
			vector<s_obj*> & objs = m_pfrm_int->objs;
			objs.push_back(pobj);
		}
		m_sop = SOP_NULL;
		m_op = OBJ;
		if(m_cur_obj < 0){
			m_cur_obj = (int) m_pfrm_int->objs.size() - 1;
		}
		break;
	case OBJ:
		{
			vector<s_obj*> & objs = m_pfrm_int->objs;
			if(m_cur_obj >= 0 && m_cur_obj < objs.size()){
				s_obj * pobj = objs[m_cur_obj];
				pobj = pobj->pmdl->detect(m_img_gry, pobj);
				if(!pobj)
					objs.erase(objs.begin() + m_cur_obj);
			}
		}
		m_sop = SOP_NULL;
		m_op = OBJ;
		break;
	}
}

void f_inspector::handle_sop_awscmd()
{
	bool ret;
	if(!m_paws->push_command(m_cmd_buf, m_cmd_ret, ret))
		cerr << "Unknown command issued." << endl;
	if(!ret){
		cerr << "Command " << m_cmd_buf << " Failed." << endl;
	}
	m_sop = SOP_NULL;
	return;
}

void f_inspector::handle_sop_set_kf()
{
	if(!m_pfrm)
		return;
	if(m_cur_kfrm != 0 &&  m_pfrm == m_kfrms[m_cur_kfrm])
		return;

	m_cur_kfrm = (m_cur_kfrm + 1) % m_num_kfrms;
	if(m_kfrms[m_cur_kfrm] != NULL){
		if(m_basv_kfrms){
			m_kfrms[m_cur_kfrm]->save(m_name);
		}
		s_frame::free(m_kfrms[m_cur_kfrm]);
		m_kfrms[m_cur_kfrm] = NULL;
	}

	m_kfrms[m_cur_kfrm] = m_pfrm;
	m_kfrms[m_cur_kfrm]->set_as_key(m_img_s);
	m_sop = SOP_NULL;
}

