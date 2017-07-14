#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "c_imgalign.h"
#include "aws_stdlib.h"
#include "aws_vobj.h"
#include "aws_vlib.h"

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
	prjPts(pts, pt2ds, cam_int, cam_dist, rvec, tvec);
}

bool s_model::load(const char * afname)
{
	strcpy(fname, afname);
	if(!load())
		return false;

	return true;
}

bool s_model::load()
{
	if(fname[0] == '\0')
		return false;

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
		FileNode fpt = fn[(const char*)buf];
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
		FileNode fe = fn[(const char*)buf];
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
		FileNode fpart = fn[(const char*)buf];
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
		bool balloc = false;
		if(!pobj){
			pobj = new s_obj;
			balloc = true;
		}

		pobj->pmdl = this;

		if(par_chsbd.detect(img, pobj->pt2d)){
			int len_name = (int) strlen(name.c_str()) + 6 + 1;
			if (pobj->name)
				delete [] pobj->name;
			pobj->name = new char[len_name];
			if(pobj->name == NULL){
				if(balloc)
					delete pobj;
				return NULL;
			}
			snprintf(pobj->name, len_name, "%s_%03d", name.c_str(), ref);
			ref++;
			pobj->visible.resize(pobj->pt2d.size(), true);
			return pobj;
		}

		if(balloc){
			delete pobj;
		}
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
	obj.R.copyTo(R);
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
			Mat R = Mat(3, 3, CV_64FC1);
			exp_so3(rvec.ptr<double>(), R.ptr<double>());
			//			Rodrigues(rvec, R);
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
		Point2f pt;
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
		prjPts(pmdl->pts_deformed, 
		// jacobian 
		// rows: 2N 
		// cols: r, t, f, c, k
		R = Mat::zeros(3, 3, CV_64FC1);
		exp_so3(rvec.ptr<double>(), R.ptr<double>());
		//		Rodrigues(rvec, R);

		if(!test_prjPtsj(camint, camdist, rvec, tvec, pmdl->pts_deformed,
			visible, jacobian, fix_aspect_ratio ? 1.0 : 0.0)){
			cerr << "Jacobian calculated by prjPts may be wrong." << endl;
		}

		calc_ssd();
		mulTransposed(jacobian, hessian, true);
		jterr = jacobian.t() * err;
	}else{
	//	projectPoints(pmdl->pts_deformed, rvec, tvec, camint, camdist, pt2dprj);
		prjPts(pmdl->pts_deformed, pt2dprj, camint, camdist, rvec, tvec);
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

void s_obj::sample_pt_tmpl(int ipt, Mat & img, Size & sz)
{
	int ox = sz.width >> 1;
	int oy = sz.height >> 1;
	Rect roi;
	roi.width = sz.width;
	roi.height = sz.height;

	if(ptx_tmpl.size() != pt2d.size())
		ptx_tmpl.resize(pt2d.size());

	if(!visible[ipt])
		return;
	Point2f & pt = pt2d[ipt];

	roi.x = (int)(pt.x + 0.5) - ox;
	roi.y = (int)(pt.y + 0.5) - oy;
	if(roi.x < 0 || roi.y < 0 || roi.x + roi.width >= img.cols || roi.y + roi.height >= img.rows){
		visible[ipt] = false;
		return;
	}

	img(roi).copyTo(ptx_tmpl[ipt]);
}

void s_obj::analyze_error(double fx, double fy)
{
	cout << "Objetct " << name << "'s error profile." << endl;
	vector<Point3f> & pt3d = pmdl->pts_deformed;
	vector<Point3f> pt3dtrn;

	int num_pts = (int) pt2d.size();
	pt3dtrn.resize(num_pts);

	Mat R(3, 3, CV_64FC1);
	exp_so3(rvec.ptr<double>(), R.ptr<double>());
	trnPts(pt3d, pt3dtrn, visible, R, tvec);

	// calculating the maximum radius of the pixel error.
	double reperr_x_max, reperr_x_min;
	double reperr_y_max, reperr_y_min;
	double reperr_radius;

	reperr_x_max = reperr_y_max = -DBL_MAX;
	reperr_x_min = reperr_y_min = DBL_MAX;
	for(int ipt = 0; ipt < num_pts; ipt++){			
		if(!visible[ipt])
			continue;
		reperr_x_max = max(reperr_x_max, (double)(pt2d[ipt].x - pt2dprj[ipt].x));
		reperr_x_min = min(reperr_x_min, (double)(pt2d[ipt].x - pt2dprj[ipt].x));
		reperr_y_max = max(reperr_y_max, (double)(pt2d[ipt].y - pt2dprj[ipt].y));
		reperr_y_min = min(reperr_y_min, (double)(pt2d[ipt].y - pt2dprj[ipt].y));
	}

	reperr_radius = max(abs(reperr_x_max), abs(reperr_x_min));
	reperr_radius = max(reperr_radius, max(abs(reperr_y_max), abs(reperr_y_min)));

	// counting visible pairs.
	int num_visibles = 0;
	for(int ipt = 0; ipt < visible.size(); ipt++)
		if(visible[ipt] != 0)
			num_visibles++;

	int num_pairs = num_visibles * (num_visibles - 1) / 2;
	int ipair = 0;

	/*
	vector<double> delta_fx, delta_fy, delta_Tzx, delta_Tzy;	
	vector<Point2i> idx_pair;
	delta_fx.resize(num_pairs);
	delta_fy.resize(num_pairs);
	delta_Tzx.resize(num_pairs);
	delta_Tzy.resize(num_pairs);
	idx_pair.resize(num_pairs);
	*/

	delta_f_rmax = delta_Tz_rmax = DBL_MAX;
	delta_Tx_rmax = delta_Ty_rmax = -DBL_MAX;

	/*
	double Lprjx_max = -DBL_MAX, Lprjy_max = -DBL_MAX;
	int iprjx_max, iprjy_max;
	*/

	for(int ipt0 = 0; ipt0 < num_pts; ipt0++){
		if(!visible[ipt0])
			continue;
		double iZ0 = 1.0 / pt3dtrn[ipt0].z;
		double dxdTx, dydTy;
		dxdTx = iZ0 * fx;
		dydTy = iZ0 * fy;
		delta_Tx_rmax = max(delta_Tx_rmax, abs(reperr_radius / dxdTx));
		delta_Ty_rmax = max(delta_Ty_rmax, abs(reperr_radius / dydTy));

		for(int ipt1 = ipt0 + 1; ipt1 < num_pts; ipt1++){
			if(!visible[ipt1])
				continue;

			/*
			idx_pair[ipair].x = ipt0;
			idx_pair[ipair].y = ipt1;
			*/

			double iZ1 = 1.0 / pt3dtrn[ipt1].z;
			
			double iZ0sq = iZ0 * iZ0, iZ1sq = iZ1 * iZ1;

			/*
			double Lxprj = (pt2dprj[ipt1].x - pt2dprj[ipt0].x), Lyprj = (pt2dprj[ipt1].y - pt2dprj[ipt0].y);
			double delta_Lx = pt2d[ipt1].x - pt2d[ipt0].x - Lxprj;
			double delta_Ly = pt2d[ipt1].y - pt2d[ipt0].y - Lyprj;
			*/

			double dLxdfx = iZ1 * pt3dtrn[ipt1].x -  iZ0 * pt3dtrn[ipt0].x;
			double dLydfy = iZ1 * pt3dtrn[ipt1].y -  iZ0 * pt3dtrn[ipt0].y;
			double dLxdTz = (iZ0sq * pt3dtrn[ipt0].x - iZ1sq * pt3dtrn[ipt1].x) * fx;	
			double dLydTz = (iZ0sq * pt3dtrn[ipt0].y - iZ1sq * pt3dtrn[ipt1].y) * fy;

			/*
			delta_fx[ipair] = delta_Lx / dLxdfx;
			delta_fy[ipair] = delta_Ly / dLydfy;
			delta_Tzx[ipair] = delta_Lx / dLxdTz;
			delta_Tzy[ipair] = delta_Ly / dLydTz;
			*/

			delta_f_rmax = min(delta_f_rmax, 
				min(abs(2 * reperr_radius / dLxdfx), abs(2 * reperr_radius / dLydfy)));
			delta_Tz_rmax = min(delta_Tz_rmax,
				min(abs(2 * reperr_radius / dLxdTz), abs(2 * reperr_radius / dLydTz)));

			/*
			if(Lprjx_max < abs(Lxprj)){
				Lprjx_max = abs(Lxprj);
				iprjx_max = ipair;
			}

			if(Lprjy_max < abs(Lyprj)){
				Lprjy_max = abs(Lyprj);
				iprjy_max = ipair;
			}
			*/

			ipair++;
		}
	}

	/*
	double delta_fx_min, delta_fx_max, delta_fy_min, delta_fy_max, delta_Tz_max, delta_Tz_min;
	double delta_fx_avg, delta_fy_avg, delta_Tz_avg;
	double delta_fx_var, delta_fy_var, delta_Tz_var;
	delta_fx_min = delta_fy_min = delta_Tz_min = DBL_MAX;
	delta_fx_max = delta_fy_max = delta_Tz_max = -DBL_MAX;
	delta_fx_avg = delta_fy_avg = delta_Tz_avg = 0.;
	delta_fx_var = delta_fy_var = delta_Tz_var = 0.;
	for(ipair = 0; ipair < num_pairs; ipair++){
		delta_fx_min = min(delta_fx_min, abs(delta_fx[ipair]));
		delta_fx_max = max(delta_fx_max, abs(delta_fx[ipair]));
		delta_fy_min = min(delta_fy_min, abs(delta_fy[ipair]));
		delta_fy_max = max(delta_fy_max, abs(delta_fy[ipair]));
		delta_Tz_min = min(delta_Tz_min, abs(delta_Tzx[ipair]));
		delta_Tz_max = max(delta_Tz_max, abs(delta_Tzx[ipair]));
		delta_Tz_min = min(delta_Tz_min, abs(delta_Tzy[ipair]));
		delta_Tz_max = max(delta_Tz_max, abs(delta_Tzy[ipair]));

		delta_fx_avg += delta_fx[ipair];
		delta_fy_avg += delta_fy[ipair];
		delta_Tz_avg += delta_Tzx[ipair] + delta_Tzy[ipair];
		delta_fx_var += delta_fx[ipair] * delta_fx[ipair];
		delta_fy_var += delta_fy[ipair] * delta_fy[ipair];
		delta_Tz_var += delta_Tzy[ipair] * delta_Tzy[ipair] + delta_Tzx[ipair] * delta_Tzx[ipair];
	}

	double div = 1.0/ (double) num_pairs;

	delta_fx_avg *= div;
	delta_fy_avg *= div;
	delta_Tz_avg *= div * 0.5;
	double sq_delta_fx_avg = delta_fx_avg * delta_fx_avg;
	double sq_delta_fy_avg = delta_fy_avg * delta_fy_avg;
	double sq_delta_Tz_avg = delta_Tz_avg * delta_Tz_avg;
	delta_fx_var *= div;
	delta_fy_var *= div;
	delta_Tz_var *= div * 0.5;
	delta_fx_var -= sq_delta_fx_avg;
	delta_fy_var -= sq_delta_fy_avg;
	delta_Tz_var -= sq_delta_Tz_avg;

	cout << "\t Max pix error: (" << reperr_x_max << "," << reperr_y_max << ")" << 
		" Min pix error: (" << reperr_x_min << "," << reperr_y_min << ")" << endl;
	cout << "\t fx-> avg " << delta_fx_avg << " stdev " << sqrt(delta_fx_var) 
		<< " min " << delta_fx_min << " max " << delta_fx_max 
		<< " maxlenerr " << delta_fx[iprjx_max] << "(" << Lprjx_max << ")" << endl;
	cout << "\t fy-> avg " << delta_fy_avg << " stdev " << sqrt(delta_fy_var) 
		<< " min " << delta_fy_min << " max " << delta_fy_max 
		<< " maxlenerr " << delta_fy[iprjy_max] << "(" << Lprjy_max << ")" << endl;
	cout << "\t Tz-> avg " << delta_Tz_avg << " stdev " << sqrt(delta_Tz_var) 
		<< " min " << delta_Tz_min << " max " << delta_Tz_max 
		<< " maxlenerr x " << delta_Tzx[iprjx_max] << " maxlenerr y " << delta_Tzy[iprjy_max] << endl;
	*/

	cout << "\t radius reperr " << reperr_radius << " radius ferr " << delta_f_rmax 
		<< " radius Terr (" << delta_Tx_rmax << "," << delta_Ty_rmax << "," << delta_Tz_rmax << ")" << endl;
}
