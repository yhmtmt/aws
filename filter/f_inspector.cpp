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

#include "../util/aws_cminpack.h"
#include "../util/coord.h"
#include "../util/c_ship.h"
#include "../util/c_clock.h"

//#include "../util/c_nmeadec.h"
#include "../channel/ch_base.h"
#include "../channel/ch_image.h"

#include "f_inspector.h"

const DWORD ModelVertex::FVF = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1;  

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
///////////////////////////////////////////////////////////////// for cminpack
struct s_package {
	int num_models;
	vector<int> & num_pts;
	vector<vector<Point2f > > & p2d;
	vector<vector<Point2f > > p2dprj;
	vector<vector<Point3f > > & p3d;
	vector<Mat> & cam_int_tbl;
	vector<Mat> & cam_dist_tbl;
	Mat cam_int, cam_dist;
	s_package(int anum_models, vector<int> & anum_pts, 
		vector<vector<Point2f > > & ap2d,
		vector<vector<Point3f > > & ap3d, 
		vector<Mat> & acam_int_tbl, 
		vector<Mat> & acam_dist_tbl,
		Mat & acam_int, Mat & acam_dist): 
	num_models(anum_models), num_pts(anum_pts), p2d(ap2d), p3d(ap3d),
		cam_int_tbl(acam_int_tbl), cam_dist_tbl(acam_dist_tbl), 
		cam_int(acam_int), cam_dist(acam_dist){
	}
};

int prj_pause_and_cam(void * p, int m, int n, const __cminpack_real__ *x,
	__cminpack_real__ *fvec, int iflag)
{
	// input layout
	// x[0] : fx, x[1] : fy, x[2] : cx, x[3] : cy
	// x[4] : k1, x[5] : k2, x[6] : px, x[7] : py
	// x[8] : k3, x[9] : k4, x[10]: k5, x[11]: k6
	// x[12 ~] : rvec and tvec

	s_package * pkg = (s_package *) p;
	Mat & cam_int = pkg->cam_int;
	Mat & cam_dist = pkg->cam_dist;

	// loading projection matrix
	double * ptr = cam_int.ptr<double>(0);
	ptr[0] = x[0]; //fx
	ptr[2] = x[2]; // cx
	ptr[4] = x[1]; // fy
	ptr[5] = x[3]; // cy

	// loading distortino parameter
	ptr = cam_dist.ptr<double>(0);
	ptr[0] = x[4]; // k1
	ptr[1] = x[5]; // k2
	ptr[2] = x[6]; // px
	ptr[3] = x[7]; // py
	ptr[4] = x[8]; // k3
	ptr[5] = x[9]; // k4
	ptr[6] = x[10]; // k5
	ptr[7] = x[11]; // k6;

	const double * px = &(x[12]);
	double * pf = fvec;
	for(int im = 0; im < pkg->num_models; im++){
		Mat rvec = Mat(3, 1, CV_64FC1, (void*) px);
		Mat tvec = Mat(3, 1, CV_64FC1, (void*) (px+3));
		projectPoints(pkg->p3d[im], rvec, tvec, cam_int, 
			cam_dist, pkg->p2dprj[im]);
		for(int ipt = 0; ipt < pkg->num_pts[im]; ipt++){
			pf[0] = pkg->p2dprj[im][ipt].x - pkg->p2d[im][ipt].x;
			pf[1] = pkg->p2dprj[im][ipt].y - pkg->p2d[im][ipt].y;
			pf += 2;
		}
		px += 6;
	}
	return 0;
}

int prj_pause_and_cam_with_tbl(void * p, int m, int n, const __cminpack_real__ *x,
	__cminpack_real__ *fvec, int iflag)
{
	// input layout
	// x[0] : camera parameter table index
	// x[1 ~ ]: rvec and tvec

	s_package * pkg = (s_package *) p;
	Mat & cam_int = pkg->cam_int;
	Mat & cam_dist = pkg->cam_dist;

	// loading projection matrix
	double * ptr = cam_int.ptr<double>(0);
	int x0_l = max(0, (int) x[0]);  
	double rat = x[0] - (double) x0_l;
	double rat_i = 1.0 - rat;
	int x0_u = min((int)(pkg->cam_int_tbl.size() - 1), x0_l + 1);

	double * ptr_u = pkg->cam_int_tbl[x0_u].ptr<double>(0);
	double * ptr_l = pkg->cam_int_tbl[x0_l].ptr<double>(0);

	ptr[0] = rat_i * ptr_l[0] + rat * ptr_u[0]; //fx
	ptr[2] = rat_i * ptr_l[2] + rat * ptr_u[2]; // cx
	ptr[4] = rat_i * ptr_l[4] + rat * ptr_u[4]; // fy
	ptr[5] = rat_i * ptr_l[5] + rat * ptr_u[5]; // cy

	// loading distortion parameter
	ptr = cam_dist.ptr<double>(0);
	ptr_u = pkg->cam_dist_tbl[x0_u].ptr<double>(0);
	ptr_l = pkg->cam_dist_tbl[x0_l].ptr<double>(0);
	for(int i = 0; i < 8; i++){
		ptr[i] = rat_i * ptr_l[i] + rat * ptr_u[i];; // k1
	}

	const double * px = &(x[1]);
	double * pf = fvec;
	for(int im = 0; im < pkg->num_models; im++){
		Mat rvec = Mat(3, 1, CV_64FC1, (void*) px);
		Mat tvec = Mat(3, 1, CV_64FC1, (void*) (px+3));
		projectPoints(pkg->p3d[im], rvec, tvec, cam_int, 
			cam_dist, pkg->p2dprj[im]);
		for(int ipt = 0; ipt < pkg->num_pts[im]; ipt++){
			pf[0] = pkg->p2dprj[im][ipt].x - pkg->p2d[im][ipt].x;
			pf[1] = pkg->p2dprj[im][ipt].y - pkg->p2d[im][ipt].y;
			pf += 2;
		}
		px += 6;
	}
	return 0;
}

int prj_pause(void * p, int m, int n, const __cminpack_real__ *x,
	__cminpack_real__ *fvec, int iflag)
{
	s_package * pkg = (s_package *) p;
	Mat & cam_int = pkg->cam_int;
	Mat & cam_dist = pkg->cam_dist;

	const double * px = &(x[0]);
	double * pf = fvec;
	for(int im = 0; im < pkg->num_models; im++){
		Mat rvec = Mat(3, 1, CV_64FC1, (void*) px);
		Mat tvec = Mat(3, 1, CV_64FC1, (void*) (px+3));
		projectPoints(pkg->p3d[im], rvec, tvec, cam_int, 
			cam_dist, pkg->p2dprj[im]);
		for(int ipt = 0; ipt < pkg->num_pts[im]; ipt++){
			pf[0] = pkg->p2dprj[im][ipt].x - pkg->p2d[im][ipt].x;
			pf[1] = pkg->p2dprj[im][ipt].y - pkg->p2d[im][ipt].y;
			pf += 2;
		}
		px += 6;
	}
	return 0;
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
	projectPoints(pts, rvec, tvec, cam_int, cam_dist, pt2ds);
}

bool s_model::load(const char * afname)
{
	fname = afname;
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

	fn = fs["Points"];

	if(fn.empty()){
		cerr << "Cannot find node Points." << endl;
		return false;
	}

	char buf[64];
	pts.resize(numPoints);
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

	apmdl->ref++;
	return true;
}

bool s_obj::init(const s_obj & obj)
{
	pmdl = obj.pmdl;
	t = obj.t;

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
	return true;
}

bool s_obj::load(const char * aname, long long at, vector<s_model> & mdls)
{
	char fname[1024];
	snprintf(fname, 1024, "%s_%lld.yml", aname, at);
	FileStorage fs(fname, FileStorage::READ);
	if(!fs.isOpened()){
		return false;
	}

	FileNode fn;

	fn = fs["ObjName"];
	if(fn.empty())
		return false;

	string str;
	fn >> str;
	if(str != name)
		return false;

	fn = fs["Model"];
	if(fn.empty())
		return false;

	fn >> str;
	pmdl = NULL;
	for(int i = 0; i < mdls.size(); i++){
		if(mdls[i].fname == str){
			pmdl = &mdls[i];
			break;
		}
	}
	if(pmdl == NULL){
		mdls.push_back(s_model());
		vector<s_model>::iterator itr = mdls.end() - 1;
		if(!itr->load(str.c_str())){
			mdls.pop_back();
			cerr << "Failed to load model " << str << endl;
			return false;
		}
		pmdl = &(*itr);
	}

	// allocating memories
	int num_points = (int) pmdl->pts.size();
	pt2d.resize(num_points);
	pt2dprj.resize(num_points);
	visible.resize(num_points);

	fn = fs["rvec"];
	if(fn.empty()){
		return false;
	}
	fn >> rvec;

	fn = fs["tvec"];
	if(fn.empty()){
		return false;
	}
	fn >> tvec;

	fn = fs["Matched"];
	if(fn.empty())
		return false;
	FileNodeIterator itr = fn.begin();
	for(int i = 0; i < num_points; i++, itr++){
		if((*itr).empty())
			return false;
		int b;
		(*itr) >> b;
		visible[i] = b;
	}

	fn = fs["Points"];
	if(fn.empty())
		return false;
	itr = fn.begin();
	for(int i = 0; i < num_points; i++, itr++){
		if((*itr).empty())
			return false;
		(*itr) >> pt2d[i];
	}

	t = at;
	if(name){
		if(strcmp(name, aname) == 0){
			return true;
		}
		delete[] name;
		name = new char[strlen(aname)+1];
		if(!name)
			return false;
		strcpy(name, aname);
	}

	return true;
}

bool s_obj::save()
{
	char fname[1024];
	snprintf(fname, 1024, "%s_%lld.yml", name, t);
	FileStorage fs(fname, FileStorage::WRITE);
	if(!fs.isOpened()){
		return false;
	}

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
	return true;
}

bool s_obj::load(FileNode & fnobj, vector<s_model> & mdls)
{
	FileNode fn;

	fn = fnobj["ObjName"];
	if(fn.empty())
		return false;

	string str;
	fn >> str;
	if(str != name)
		return false;

	fn = fnobj["Model"];
	if(fn.empty())
		return false;

	fn >> str;
	pmdl = NULL;
	for(int i = 0; i < mdls.size(); i++){
		if(mdls[i].fname == str){
			pmdl = &mdls[i];
			break;
		}
	}
	if(pmdl == NULL){
		mdls.push_back(s_model());
		vector<s_model>::iterator itr = mdls.end() - 1;
		if(!itr->load(str.c_str())){
			mdls.pop_back();
			cerr << "Failed to load model " << str << endl;
			return false;
		}
		pmdl = &(*itr);
	}

	// allocating memories
	int num_points = (int) pmdl->pts.size();
	pt2d.resize(num_points);
	pt2dprj.resize(num_points);
	visible.resize(num_points);

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
	for(int i = 0; i < num_points; i++, itr++){
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
	for(int i = 0; i < num_points; i++, itr++){
		if((*itr).empty())
			return false;
		(*itr) >> pt2d[i];
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
	return true;
}

void s_obj::proj(Mat & camint, Mat & camdist)
{
	projectPoints(pmdl->pts, rvec, tvec, camint, camdist, pt2dprj, jacobian);

	//calculating maximum values for each parameter
	jmax = Mat::zeros(1, jacobian.cols, CV_64FC1);
	double * ptr_max, * ptr;
	for(int i = 0; i < jacobian.rows; i++){
		ptr = jacobian.ptr<double>(i);
		ptr_max = jmax.ptr<double>(0);
		for(int j = 0; j < jacobian.cols; j++, ptr++, ptr_max++){
			*ptr_max = max(*ptr_max, abs(*ptr));
		}
	}
	mulTransposed(jacobian, hessian, true);
	calc_ssd();
	calc_num_matched_points();
}

void s_obj::render(LPDIRECT3DDEVICE9 pd3dev, c_d3d_dynamic_text * ptxt, LPD3DXLINE pline,
	int pttype, int state, int cur_point)
{
	render_prjpts(*pmdl, pt2dprj, pd3dev, ptxt, pline, pttype, state, cur_point);
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

	Mat rvec_comp, tvec_comp;
	composeRT(rvec_cam, tvec_cam, rvec, tvec, rvec_comp, tvec_comp);
	projectPoints(p3d, rvec_comp, tvec_comp, cam_int, cam_dist, p2d);
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
	projectPoints(vec3d, rvec_comp, tvec_comp, cam_int, cam_dist, vec2d);
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
//////////////////////////////////////////////////////////////////// s_frame_obj

bool s_frame_obj::init(const long long atfrm, const vector<s_obj> & aobjs, const Mat & acamint, const Mat & acamdist)
{
	tfrm = atfrm;
	objs.resize(aobjs.size());
	for(int i = 0; i < aobjs.size(); i++){
		if(!objs[i].init(aobjs[i])){
			return false;
		}
	}
	acamint.copyTo(camint);
	acamdist.copyTo(camdist);
	return true;
}

bool s_frame_obj::save(const char * aname)
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
		objs[iobj].save(fs);
		fs << "}";
	}
	fs << "}";

	return true;
}

bool s_frame_obj::load(const char * aname, vector<s_model> & mdls)
{
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
		if(!objs[iobj].load(fnobj, mdls))
			return false;
	}
	return true;
}

//////////////////////////////////////////////////////////////////// class f_inspector
const char * f_inspector::m_str_op[ESTIMATE+1]
= {"model", "obj", "point", "camera", "estimate"};

const char * f_inspector::m_axis_str[AX_Z + 1] = {
	"x", "y", "z"
};

const char * f_inspector::m_str_campar[ECP_K6 + 1] = {
	"fx", "fy", "cx", "cy", "k1", "k2", "c1", "c2", "k3", "k4", "k5", "k6"
};

f_inspector::f_inspector(const char * name):f_ds_window(name), m_pin(NULL), m_timg(-1),
	m_sh(1.0), m_sv(1.0), m_bundistort(false), m_bcam_tbl_loaded(false),
	m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_principal_point(false), m_bcalib_fix_aspect_ratio(false),
	m_bcalib_zero_tangent_dist(true), m_bcalib_fix_k1(true), m_bcalib_fix_k2(true), m_bcalib_fix_k3(true),
	m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false),
	m_badd_model(false), m_cur_frm(-1), m_cur_campar(0),  m_cur_model(-1), m_cur_obj(-1), m_cur_point(-1), 
	m_op(OBJ), m_sop(SOP_NULL), m_mm(MM_NORMAL), m_axis(AX_X), m_adj_pow(0), m_adj_step(1.0),
	m_main_offset(0, 0), m_main_scale(1.0), m_theta_z_mdl(0.0), m_dist_mdl(0.0)
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
	register_fpar("add_model", &m_badd_model, "If the flag is asserted, model is loaded from fmodel");
	register_fpar("fcp", m_fname_campar, 1024, "File path of camera parameter.");
	register_fpar("fcptbl", m_fname_campar_tbl, 1024, "File path of table of the camera parameters with multiple magnifications.");
	register_fpar("op", (int*)&m_op, ESTIMATE+1, m_str_op,"Operation ");
	register_fpar("sh", &m_sh, "Horizontal scaling value. Image size is multiplied by the value. (default 1.0)");
	register_fpar("sv", &m_sv, "Vertical scaling value. Image size is multiplied by the value. (default 1.0)");

	// model related parameters
	register_fpar("mdl", &m_cur_model, "Model with specified index is selected.");
	register_fpar("pt", &m_cur_point, "Model point of specified index is selected.");

	// camera calibration and parameter
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

	register_fpar("use_intrinsic_guess", &m_bcalib_use_intrinsic_guess, "Use intrinsic guess.");
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

	register_fpar("undist", &m_bundistort, "Undistort source image according to the camera parameter.");

	// object/camera manipulation
	register_fpar("axis", (int*)&m_axis, (int)AX_Z + 1, m_axis_str, "Axis for rotation and translation. {x, y, z}");
	register_fpar("sadj", &m_adj_step, "Parameter adjustment step for the camera and objects.");
}

f_inspector::~f_inspector()
{
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

bool f_inspector::proc()
{
	pthread_lock lock(&m_d3d_mtx);

	////////////////// updating pvt information ///////////////////////
	long long timg;	
	Mat img;
	if(!is_pause()){
		img = m_pin->get_img(timg);
		if(img.empty())
			return true;

		if(m_timg != timg){ // new frame arrived
			// auto save camera parameter and objects
			if(m_timg > 0){
				// new frame object added
				m_fobjs.push_back(s_frame_obj());
				m_cur_frm = (int) m_fobjs.size() - 1;
				m_fobjs[m_cur_frm].tfrm = timg;

				// if the frame object is not the first one, save the previous frame object 
				// and initialize current frame object with the previous frame object.
				if(m_cur_frm > 0){
					// save previous frame object
					s_frame_obj & fobj = m_fobjs[m_cur_frm - 1];
					if(!fobj.save(m_name)){
						cerr << "Failed to save filter objects in time " << fobj.tfrm << "." << endl;
					}

					// To initialize new frame object, it firstly seeks for the file for the frame via time stamp.
					// If the trial failed, the new frame object is simply initalized with previous frame object.
					if(!m_fobjs[m_cur_frm].load(m_name, m_models)){
						m_fobjs[m_cur_frm].init(timg, fobj.objs, fobj.camint, fobj.camdist);
					}
				}
			}

			// update time and image.
			m_timg = timg;
			m_img = img;
		}
	}else{
		timg = m_timg;
		img = m_img;
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
	case SOP_INST_OBJ:
		handle_sop_inst_obj();
		break;
	}

	// projection 
	proj_objs(m_fobjs[m_cur_frm].objs);
	calc_jcam_max();

	// estimate
	if(m_op == ESTIMATE){
		estimate();
	}
	// input source is not ready. but it tends to happen usually.
	if(img.empty())
		return true;

	Mat img_s;
	resize(img, img_s, Size(), m_sh, m_sv);

	// fit the viewport size to the image
	if(img_s.cols != m_ViewPort.Width ||
		img_s.rows != m_ViewPort.Height){
			if(!init_viewport(img_s)){
				return false;
			}
	}

	// fit the direct 3d surface to the image
	if(img_s.cols != m_maincam.get_surface_width() ||
		img_s.rows != m_maincam.get_surface_height())
	{
		m_maincam.release();
		if(!m_maincam.init(m_pd3dev,  
			(float) img_s.cols, (float) img_s.rows, 
			(float) m_ViewPort.Width, (float) m_ViewPort.Height, 
			(float) m_ViewPort.Width, (float) m_ViewPort.Height))
			return false;
	}

	// fit the direct 3d surface of the model view to the image
	if((img_s.cols != m_model_view.get_surface_width() || 
		img_s.rows != m_model_view.get_surface_height()))
	{
		m_model_view.release();
		if(!m_model_view.init(m_pd3dev,
			(float) img_s.cols, (float) img_s.rows,
			(float) m_ViewPort.Width, (float) m_ViewPort.Height,
			(float) m_ViewPort.Width, (float) m_ViewPort.Height))
			return false;
	}

	//////////////// Model related code //////////////////////////////
	if(m_badd_model){
		if(!load_model()){
			cerr << "Failed to load model " << m_fname_model << endl;
		}
		m_badd_model = false;
	}

	// rendering main view
	render(img_s, timg);

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
		fs << "Err" << m_cam_erep[ipar];
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
	m_cam_erep.resize(num_pars);

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

		fnsub = fnpar["Err"];
		if(fnsub.empty())
			continue;
		fnsub >> m_cam_erep[ipar];
	}

	m_bcam_tbl_loaded = true;
	return true;
}

void f_inspector::clearCamparTbl()
{
	m_cam_int_tbl.clear();
	m_cam_dist_tbl.clear();
	m_cam_erep.clear();

	m_bcam_tbl_loaded = false;
}

bool f_inspector::load_model()
{
	s_model mdl;
	if(mdl.load(m_fname_model)){
		m_models.push_back(mdl);
		m_cur_model = (int)(m_models.size() - 1);
		return true;
	}
	return false;
}

//////////////////////////////////////////////// renderer
void f_inspector::render(Mat & imgs, long long timg)
{
	// Image level rendering

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

	renderObj();
	renderCampar();

	m_maincam.ResetRenderTarget(m_pd3dev);

	////////////////////// render 3D model ///////////////////////////

	switch(m_op){
	case MODEL:
	case OBJ:
		renderModel(timg);
	default:
		break;
	}

	//////////////////// render total view port /////////////////////
	m_maincam.show(m_pd3dev, (float)(0. + m_main_offset.x),
		(float) ((float) m_ViewPort.Height + m_main_offset.y), m_main_scale);

	switch(m_op){
	case MODEL:
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
	int y = 0;
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;

	snprintf(information, 1023, "AWS Time %s (Image Time %lld) Adjust Step x%f", 
		m_time_str, m_timg, (float) m_adj_step);
	m_d3d_txt.render(m_pd3dev, information, 0.f, (float) y, 1.0, 0, EDTC_LT);
	y += 20;
	snprintf(information, 1023, "Operation: %s (M)->Model (O)->Obj (E)->Estimate (C)->Camera", m_str_op[m_op]);
	m_d3d_txt.render(m_pd3dev, information, 0.f, (float) y, 1.0, 0, EDTC_LT);
	y += 20;
	snprintf(information, 1023, "%d Objects %d Models", objs.size(), m_models.size());
	m_d3d_txt.render(m_pd3dev, information, 0.f, (float)y, 1.0, 0, EDTC_LT);
	y += 20;
	switch(m_op){
	case MODEL:
		if(m_cur_model < 0)
			snprintf(information, 1023, "Model[]=NULL");
		else
			snprintf(information, 1023, "Model[%d]=%s (%d Points, %d Edges)", m_cur_model, 
			m_models[m_cur_model].fname,
			m_models[m_cur_model].pts.size(), m_models[m_cur_model].edges.size());
		break;
	case OBJ:
		if(m_cur_obj < 0)
			snprintf(information, 1023, "Obj[]=NULL");
		else{
			s_obj & obj = objs[m_cur_obj];
			snprintf(information, 1023, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f rvec=(%f,%f,%f) tvec=(%f,%f,%f)",
				m_cur_obj, obj.name, obj.pmdl->fname, obj.match_count, obj.ssd,
				obj.rvec.at<double>(0), obj.rvec.at<double>(1), obj.rvec.at<double>(2),
				obj.tvec.at<double>(0), obj.tvec.at<double>(1), obj.tvec.at<double>(2));
		}
		break;
	case POINT:
		if(m_cur_obj < 0)
			snprintf(information, 1023, "Obj[]=NULL");
		else{
			s_obj & obj = objs[m_cur_obj];
			if(m_cur_point < 0){
				snprintf(information, 1023, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f", 
					m_cur_obj, objs[m_cur_obj].name, objs[m_cur_obj].pmdl->fname, obj.match_count, obj.ssd);
			}else{
				Point3f & pt3d = objs[m_cur_obj].pmdl->pts[m_cur_point];
				Point2f & pt2d = objs[m_cur_obj].pt2d[m_cur_point];
				int matched = objs[m_cur_obj].visible[m_cur_point];
				if(matched){
					snprintf(information, 1023, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f Point[%d]=(%f,%f,%f)->(%f,%f)", 
						m_cur_obj, objs[m_cur_obj].name, objs[m_cur_obj].pmdl->fname, 
						obj.match_count, obj.ssd,
						m_cur_point, pt3d.x, pt3d.y, pt3d.z, pt2d.x, pt2d.y);
				}else{
					snprintf(information, 1023, "Obj[%d]=%s (Model=%s) Matched=%d SSD=%f Point[%d]=(%f,%f,%f)->NULL", 
						m_cur_obj, objs[m_cur_obj].name, objs[m_cur_obj].pmdl->fname, 
						obj.match_count, obj.ssd,
						m_cur_point, pt3d.x, pt3d.y, pt3d.z);
				}
			}
		}
		break;
	case CAMERA:
		// camera intrinsics, fx,fy,cx,cy,p0,p1,k1-k6 
		snprintf(information, 1023, "Camera fx=%f fy=%f cx=%f cy=%f k1=%f k2=%f p1=%f p2=%f k3=%f k4=%f k5=%f k6=%f",
			(float)m_cam_int.at<double>(0, 0), (float)m_cam_int.at<double>(1, 1), 
			(float)m_cam_int.at<double>(0, 2), (float)m_cam_int.at<double>(1, 2),
			(float)m_cam_dist.at<double>(0, 0), (float)m_cam_dist.at<double>(0, 1), 
			(float)m_cam_dist.at<double>(0, 2), (float)m_cam_dist.at<double>(0, 3), 
			(float)m_cam_dist.at<double>(0, 4), (float)m_cam_dist.at<double>(0, 5), 
			(float)m_cam_dist.at<double>(0, 6), (float)m_cam_dist.at<double>(0, 7));
			m_d3d_txt.render(m_pd3dev, information, 0.f, (float)y, 1.0, 0, EDTC_LT);
			y += 20;
		snprintf(information, 1023, "Current Parameter %s J=%f", 
			m_str_campar[m_cur_campar], m_jcam_max.at<double>(m_cur_campar));
		break;
	case ESTIMATE:
		snprintf(information, 1023, "Estimate");
		break;
	}

	m_d3d_txt.render(m_pd3dev, information, 0.f, (float)y, 1.0, 0, EDTC_LT);

	snprintf(information, 1023, "(%d, %d)", m_mc.x, m_mc.y);

	m_d3d_txt.render(m_pd3dev, information, (float)(m_mc.x + 20), (float)(m_mc.y + 20), 1.0, 0, EDTC_LT);
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
}

void f_inspector::renderObj()
{
	// Drawing object (2d and 3d)
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		if(m_op == OBJ){
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
		objs[iobj].render_axis(
			m_rvec_cam, m_tvec_cam, m_cam_int, m_cam_dist,
			m_pd3dev, m_pline, (int) m_axis);
	}

	m_pline->Begin();
	if(m_op == POINT && m_cur_obj != -1 && m_cur_point != -1){
		D3DXVECTOR2 v[2];
		Point2f pt1 = Point2f((float) (m_mc.x - m_main_offset.x) / m_main_scale, 
			(float) (m_mc.y - (int) m_ViewPort.Height -m_main_offset.y) / m_main_scale + m_ViewPort.Height);
		Point2f & pt2 = objs[m_cur_obj].pt2dprj[m_cur_point];
		v[0] = D3DXVECTOR2(pt1.x, pt1.y);
		v[1] = D3DXVECTOR2(pt2.x, pt2.y);
		m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 0, 0, 255));
	}
	m_pline->End();
}

void f_inspector::renderModel(long long timg)
{
	m_model_view.SetAsRenderTarget(m_pd3dev);
	m_pd3dev->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,
		D3DCOLOR_COLORVALUE(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0);
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
		m_dist_mdl = 2 * m_models[m_cur_model].get_max_dist(); 

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
		m_models[m_cur_model].proj(pts, m_cam_int_mdl, m_cam_dist_mdl, m_rvec_cam_mdl, m_tvec_cam_mdl, m_rvec_mdl, m_tvec_mdl);
		render_prjpts(m_models[m_cur_model], pts, m_pd3dev, NULL, m_pline, m_cur_model, 0, -1);	
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

	projectPoints(pt3d, Mat::zeros(3, 1, CV_64FC1), Mat::zeros(3, 1, CV_64FC1), m_cam_int, m_cam_dist, pt2d);

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

void f_inspector::estimate()
{
	Mat Hcamint = Mat::zeros(12, 12, CV_64FC1);
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = objs[i];
		cout << "obj[" << i << "].hessian=" << obj.hessian << endl;
		// accumulating camera intrinsic part of hessians of all objects
		Hcamint += obj.hessian(Rect(6, 6, 12, 12));
	}
	cout << "Hcamint=" << Hcamint << endl;
	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = objs[i];
		// copy a part of hessian corresponding to the camera intrinsics 
		Hcamint.copyTo(obj.hessian(Rect(6, 6, 12, 12)));
	}

	for(int i = 0; i < objs.size(); i++){
		s_obj & obj = objs[i];
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

void f_inspector::estimate_fulltime()
{
	// full projection with current camera parameters
	for(int ifrm = 0; ifrm < m_fobjs.size(); ifrm++){
		vector<s_obj> & objs = m_fobjs[ifrm].objs;
		proj_objs(m_fobjs[ifrm].objs);
	}

	// accumulating camera intrinsics part of the hessian
	Mat Hcamint = Mat::zeros(12, 12, CV_64FC1);
	for(int ifrm = 0; ifrm < m_fobjs.size(); ifrm++){
		acc_Hcamint(Hcamint, m_fobjs[ifrm].objs);
	}

	// copying camera intrinsic part 
	for(int ifrm = 0; ifrm < m_fobjs.size(); ifrm++){
		copy_Hcamint(Hcamint, m_fobjs[ifrm].objs);
	}

	for(int ifrm = 0; ifrm < m_fobjs.size(); ifrm++){
		update_params(m_fobjs[ifrm].objs);
	}	
}

void f_inspector::proj_objs(vector<s_obj> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		obj.proj(m_cam_int, m_cam_dist);
	}
}

void f_inspector::acc_Hcamint(Mat & Hcamint, vector<s_obj> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		// accumulating camera intrinsic part of hessians of all objects
		Hcamint += obj.hessian(Rect(6, 6, 12, 12));
	}
}

void f_inspector::copy_Hcamint(Mat & Hcamint, vector<s_obj> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		Hcamint.copyTo(obj.hessian(Rect(6, 6, 12, 12)));
	}
}

void f_inspector::update_params(vector<s_obj> & objs){
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		Mat Hinv, eigenval, eigenvec;
		double det = determinant(obj.hessian);
		eigen(obj.hessian, eigenval, eigenvec);
		invert(obj.hessian, Hinv, DECOMP_CHOLESKY);
		Mat Grad = obj.jacobian.t() * obj.err;;
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
	}
}

void f_inspector::calc_jcam_max(){
	m_jcam_max = Mat::zeros(1, 12, CV_64FC1);
	double * ptr_max, * ptr;
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = objs[iobj];
		Mat & jmax = obj.jmax;
		ptr = jmax.ptr<double>(0) + 6;
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
	case CAMERA:
		m_mm = MM_CAMINT;
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
	case MM_CAMINT:
		shift_cam_center();
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
	case MM_CAMINT:
		shift_cam_center();
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
	m_mc.x -= m_client_org.x;
	m_mc.y -= m_client_org.y;

	short delta = GET_WHEEL_DELTA_WPARAM(wParam);
	if(GET_KEYSTATE_WPARAM(wParam) & MK_SHIFT){
		switch(m_op){
		case OBJ:
		case POINT:
			zoom_screen(delta);
			break;
		}
	}else if(GET_KEYSTATE_WPARAM(wParam) & MK_CONTROL){
		switch(m_op){
		case OBJ:
		case POINT:
			rotate_obj(delta);
			break;
		}
	}else{
		switch(m_op){
		case OBJ:
		case POINT:
			translate_obj(delta);
			break;
		case CAMERA:
			adjust_cam(delta);
			// change the focal length of the camera
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
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	if(objs.size() == 0)
		return;

	if(m_cur_point < 0)
		return;

	Point2f pt;
	double iscale = 1.0 / m_main_scale;
	pt.x = (float)((m_mc.x - m_main_offset.x) * iscale);
	pt.y = (float)(m_mc.y - (int) m_ViewPort.Height - m_main_offset.y); 
	pt.y *= (float) iscale;
	pt.y += (float) m_ViewPort.Height;

	objs[m_cur_obj].pt2d[m_cur_point] = pt;
	objs[m_cur_obj].visible[m_cur_point] = 1;
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
	m_cam_int.at<double>(1, 2)  += (float)(m_mc.y - m_pt_sc_start.y);
}

void f_inspector::zoom_screen(short delta)
{
	short step = delta / WHEEL_DELTA;
	float scale = (float) pow(1.1, (double) step);
	m_main_scale *=  scale;
	float x, y;
	x = (float)(m_main_offset.x - m_mc.x) * scale;
	y = (float)((int) m_ViewPort.Height + m_main_offset.y - m_mc.y) * scale;
	m_main_offset.x =  (float)(x + (double) m_mc.x);
	m_main_offset.y =  (float)(y + (double) m_mc.y - (double) m_ViewPort.Height);
}

void f_inspector::translate_obj(short delta)
{
	if(m_cur_obj < 0)
		return ;
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	double * ptr = objs[m_cur_obj].jmax.ptr<double>(0) + 3;

	double step = (double)(delta / WHEEL_DELTA) * m_adj_step;
	Mat tvec = Mat::zeros(3, 1, CV_64FC1);
	switch(m_axis){
	case AX_X:
		tvec.at<double>(0, 0) = step / ptr[0];
		break;
	case AX_Y:
		tvec.at<double>(1, 0) = step / ptr[1];
		break;
	case AX_Z:
		tvec.at<double>(2, 0) = step / ptr[2];
		break;
	}

	objs[m_cur_obj].tvec += tvec;
}

void f_inspector::rotate_obj(short delta)
{
	if(m_cur_obj < 0)
		return;	

	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
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
	Mat R1, R2;
	Rodrigues(objs[m_cur_obj].rvec, R1);
	Rodrigues(rvec, R2);
	Mat R = R2 * R1;
	Rodrigues(R, objs[m_cur_obj].rvec);
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
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	switch(m_op){
	case OBJ:
		{
			m_cur_obj = m_cur_obj - 1;
			if(m_cur_obj < 0){
				m_cur_obj += (int) objs.size();
			}
			// the current object point index is initialized
			m_cur_point = objs[m_cur_obj].get_num_points() - 1;
		}
		break;
	case POINT: // decrement current object point. 3d-object point is also. 
		m_cur_point = m_cur_point  - 1;
		if(m_cur_point < 0){
			m_cur_point += (int) objs[m_cur_obj].get_num_points();
		}

		break;
	case MODEL: // decrement the current model index
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
	}
}

void f_inspector::handle_vk_right()
{
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	switch(m_op){
	case OBJ:
		{
			m_cur_obj = m_cur_obj + 1;
			m_cur_obj %= (int) objs.size();
			m_cur_point = objs[m_cur_obj].get_num_points() - 1;
		}
		break;
	case POINT: // increment the current object point index. The 3d-object point index as well.
		m_cur_point = m_cur_point + 1;
		m_cur_point %= (int) objs[m_cur_obj].get_num_points();
		break;
	case MODEL: // increment the current model index.
		m_cur_model = m_cur_model + 1;
		m_cur_model %= (int) m_models.size();
		break;
	case CAMERA:
		m_cur_campar = m_cur_campar + 1;
		m_cur_campar %= (ECP_K6 + 1);
		break;
	}
}

void f_inspector::handle_char(WPARAM wParam, LPARAM lParam)
{
	switch(wParam){
	case 'R': // reset window
		m_main_offset = Point2f(0., 0.);
		m_main_scale = 1.0;
		break;
	case 'L':
		m_sop = SOP_LOAD;
		break;
	case 'S':
		m_sop = SOP_SAVE;
		break;
	case 'O': /* O key */ 
		m_op = OBJ;
		break;
	case 'M':
		m_op = MODEL;
		break;
	case 'P':
		m_op = POINT;
		break;
	case 'E':
		m_op = ESTIMATE;
		break;
	case 'I':
		if(m_op == MODEL){
			m_sop = SOP_INST_OBJ;
		}
		break;
	case 'C':
		m_op = CAMERA;
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
	case 'f':
		// fix parameter
		switch(m_op){
		case OBJ: 
		case POINT:
			// fix selected object's attitude
			break;
		case CAMERA:
			// fix selected camera parameter
		default:
			break;
		}

	default:
		break;
	}
}


void f_inspector::handle_sop_delete(){
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	switch(m_op){
	case MODEL:
		// delete current Model
		if(m_cur_model >= 0){
			vector<s_model>::iterator itr = m_models.begin() + m_cur_model;
			if(itr->ref > 0)
				break;
			m_models.erase(itr);
			m_cur_model = (int) m_models.size() - 1;
		}
		break;
	case OBJ:
		// delete current object
		if(m_cur_obj >= 0){
			vector<s_obj>::iterator itr = objs.begin() + m_cur_obj;
			itr->pmdl->ref--;
			objs.erase(itr);
			m_cur_obj = (int) objs.size() - 1;
		}
		break;
	case POINT:
		//reset current point
		if(m_cur_obj >= 0 && m_cur_point >= 0){
			s_obj & obj = objs[m_cur_obj];
			if(m_cur_point < obj.get_num_points())
				obj.visible[m_cur_point] = 0;
		}
		break;
	case CAMERA:
		// delete current camera parameter
		break;
	}
	m_sop = SOP_NULL;
}

void f_inspector::handle_sop_save()
{
	switch(m_op){
	case OBJ:
	case POINT:
	case CAMERA:
		m_fobjs[m_cur_frm].save(m_name);
		break;
	}	
	m_sop = SOP_NULL;
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
		m_fobjs[m_cur_frm].load(m_name, m_models);
		//load camera intrinsics
		break;
	}
	m_sop = SOP_NULL;
}

void f_inspector::handle_sop_inst_obj()
{
	vector<s_obj> & objs = m_fobjs[m_cur_frm].objs;
	if(m_op == MODEL){
		if(m_cur_model < 0){
			return;
		}

		double width =(double) m_maincam.get_surface_width();
		double height = (double) m_maincam.get_surface_height();
		objs.push_back(s_obj());
		m_cur_obj = (int) objs.size() - 1;
		m_cur_point = 0;
		if(!objs[m_cur_obj].init(&m_models[m_cur_model], m_timg, m_cam_int, m_cam_dist, width, height)){
			objs.pop_back();
			cerr << "Failed to create an instance of model " << m_models[m_cur_model].name << endl;		
		}else{
			m_op = OBJ;
		}
	}
	m_sop = SOP_NULL;
}
