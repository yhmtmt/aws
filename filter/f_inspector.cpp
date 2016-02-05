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

void render_axis(s_model * pmdl, Mat & cam_int, Mat & cam_dist, Mat & rvec_cam, Mat & tvec_cam, 
				 LPDIRECT3DDEVICE9 pd3dev, LPD3DXLINE pline, int axis)
{
	float fac = (float) pmdl->get_max_dist();

	vector<Point3f> p3d(4, Point3f(0.f, 0.f, 0.f));
	vector<Point2f> p2d;

	p3d[1].x = fac; // pt3d[1] is x axis
	p3d[2].y = fac; // pt3d[2] is y axis
	p3d[3].z = fac; // pt3d[3] is z axis

//	projectPoints(p3d, rvec_cam, tvec_cam, cam_int, cam_dist, p2d);
	prjPts(p3d, p2d, cam_int, cam_dist, rvec_cam, tvec_cam);
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

//////////////////////////////////////////////////////////////////// class f_inspector
const char * f_inspector::m_str_op[VIEW3D+1]
	= {"model", "obj", "part", "point", "camera", "camtbl", "estimate", "frame", "v3d"};

const char * f_inspector::m_str_sop[SOP_AWSCMD+1]
	= {"null", "save", "load", "guess", "det", "ins", "del", "kf", "skff", "skfb", "icp", "awscmd"};

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

f_inspector::f_inspector(const char * name):f_ds_window(name), m_bnew_frm(false),  m_pin(NULL), m_timg(-1),
	m_sh(1.0), m_sv(1.0), m_btrack_obj(true), m_btrack_obj_3d(false), m_sz_vtx_smpl(128, 128), 
	m_brender_grid(false),
	m_miss_tracks(0), m_wt(EWT_TRN), m_lvpyr(2), m_sig_gb(3.0),m_bauto_load_fobj(false),
	m_bauto_save_fobj(false), m_bundistort(false), m_bcam_tbl_loaded(false), m_cur_camtbl(-1),
	m_bcalib_use_intrinsic_guess(false), m_bcalib_fix_campar(false), m_bcalib_fix_focus(false),
	m_bcalib_fix_principal_point(false), m_bcalib_fix_aspect_ratio(false),
	m_bcalib_zero_tangent_dist(true), m_bcalib_fix_k1(true), m_bcalib_fix_k2(true), m_bcalib_fix_k3(true),
	m_bcalib_fix_k4(true), m_bcalib_fix_k5(true), m_bcalib_fix_k6(true), m_bcalib_rational_model(false),
	/*m_cur_frm(-1),*/ m_int_cyc_kfrm(30), m_cur_campar(0), m_cur_model(-1), m_depth_min(0.5), m_depth_max(1.5), 
	m_num_cur_objs(0), m_cur_obj(-1), m_cur_part(-1), m_cur_point(-1), 
	m_op(OBJ), m_bkfrm(false), m_sop(SOP_NULL), m_mm(MM_NORMAL), m_axis(AX_X), m_adj_pow(0), m_adj_step(1.0),
	m_main_offset(0, 0), m_main_scale(1.0), m_theta_z_mdl(0.0), m_dist_mdl(0.0), m_cam_erep(-1.0),
	m_eest(EES_CONV), m_num_max_itrs(30), m_num_max_frms_used(100), m_err_range(3),
	m_ev(EV_CAM), m_int_kfrms(SEC), m_num_kfrms(100), m_cur_kfrm(-1), m_sel_kfrm(-1), m_bsave_objpts(true), m_pfrm(NULL), m_pfrm_int(NULL),
	m_rat_z(1.0), m_bdecomp_xyz(true)
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
	register_fpar("track3d", &m_btrack_obj_3d, "Flag enables 3D model tracking.");

	// Key frame related parameter
	register_fpar("ldkf", &m_bald_kfrms, "Flag auto loading key frames");
	register_fpar("svkf", &m_basv_kfrms, "Flag auto saving key frames");
	register_fpar("svobjpts", &m_bsave_objpts, "Flag saving transformed object points.");
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
	register_fpar("vkey", &m_bkfrm, "Image source is from key frame cache.");
	register_fpar("decxyz", &m_bdecomp_xyz, "If true, decomposition of rotation matrix is x-y-z, otherwise z-y-x. ");

	// object/camera manipulation
	register_fpar("axis", (int*)&m_axis, (int)AX_Z + 1, m_axis_str, "Axis for rotation and translation. {x, y, z}");
	register_fpar("sadj", &m_adj_step, "Parameter adjustment step for the camera and objects.");
}

f_inspector::~f_inspector()
{
	for (int imdl = 0; imdl < m_models.size(); imdl++)
		delete m_models[imdl];
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++)
		s_frame::free(m_kfrms[ikf]);

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
		pfrm_new->img = m_img_s;

		// new frame can be loaded from file
		if(m_bauto_load_fobj && pfrm_new->load(m_name, timg, m_models)){
			pfrm_new->camint.copyTo(m_cam_int);
			pfrm_new->camdist.copyTo(m_cam_dist);
		}else if(m_pfrm){ // if previous frame is not null, initialize new frame with previous frame
			// frame tracking
			if(m_btrack_obj)
				if(m_btrack_obj_3d)
					pfrm_new->init(timg, m_pfrm, m_impyr, &m_mdlTrck, m_miss_tracks);
				else
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
}

bool f_inspector::proc()
{
	m_bnew_frm = false;
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

		m_bnew_frm = true;

		if(!m_pfrm)
			return true;

		// sample point templates
		m_pfrm->sample_tmpl(m_img_gry_blur, m_sz_vtx_smpl);

		m_int_kfrms = m_int_cyc_kfrm * f_base::m_paws->get_cycle_time();
		if(m_cur_kfrm < 0){ // initial frame is forced to be key frame
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

	// executing sub-operation
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
	case SOP_SEEK_KF_BK:
		handle_sop_seek_kf(false);
		break;
	case SOP_SEEK_KF_FWD:
		handle_sop_seek_kf();
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

	// re-selecting current object and point if needed.
	if(!m_pfrm_int){
		m_cur_obj = -1;
		m_cur_point = -1;
	}else{
		if(m_cur_obj < 0 || m_cur_obj >= m_pfrm_int->objs.size()){
			m_cur_obj = (int) m_pfrm_int->objs.size() - 1;
			m_cur_point = -1;
		}
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
		m_num_cur_objs = (int) m_pfrm_int->objs.size();
		// projection 
		m_pfrm_int->proj_objs(true, m_bcalib_fix_aspect_ratio);
		calc_jmax();

		// calcurate roll pitch yaw surge sway heave relative to current object
		m_pfrm_int->calc_rpy(m_cur_obj, m_bdecomp_xyz);

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

	if(m_brender_grid)
		renderGrid();

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
			tmex tmkfrm;
			gmtimeex(m_kfrms[m_sel_kfrm]->tfrm / MSEC + m_time_zone_minute * 60000, tmkfrm);
			snprintf(information, 1023, "AWS Time %s (Key Frame Time %s %s %02d %02d:%02d:%02d.%03d %d) Adjust Step x%f", 
				m_time_str, 		
				getWeekStr(tmkfrm.tm_wday), 
				getMonthStr(tmkfrm.tm_mon),
				tmkfrm.tm_mday,
				tmkfrm.tm_hour,
				tmkfrm.tm_min,
				tmkfrm.tm_sec,
				tmkfrm.tm_msec,
				tmkfrm.tm_year + 1900, 
				(float) m_adj_step);
		}else{
			snprintf(information, 1023, "AWS Time %s (Key Frame Time NaN) Adjust Step x%f", 
				m_time_str, (float) m_adj_step);
		}
	}else{
		if(m_pfrm->kfrm)
			snprintf(information, 1023, "AWS Time %s (Image Time %lld, Key Frame) Adjust Step x%f", 
				m_time_str, m_timg, (float) m_adj_step);
		else
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

void f_inspector::renderGrid()
{
	Point pt0, pt1, pt2, pt3;
	pt0.x = pt1.x = (m_img_s.cols >> 1);
	pt2.y = pt3.y = (m_img_s.rows >> 1);
	pt0.y = pt2.y - 10;
	pt1.y = pt2.y + 10;
	pt2.x = pt0.x - 10;
	pt3.x = pt0.x + 10;

	D3DXVECTOR2 v[5];
	m_pline->Begin();
	v[0] = D3DXVECTOR2((float)(pt0.x), (float)(pt0.y));
	v[1] = D3DXVECTOR2((float)(pt1.x), (float)(pt1.y));
	m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 0, 0, 255));
	v[0] = D3DXVECTOR2((float)(pt2.x), (float)(pt2.y));
	v[1] = D3DXVECTOR2((float)(pt3.x), (float)(pt3.y));
	m_pline->Draw(v, 2, D3DCOLOR_RGBA(255, 0, 0, 255));
	m_pline->End();

#ifdef DEBUG_IMALIGN
	line(m_img_s, pt0, pt1, CV_RGB(0, 0, 255));
	line(m_img_s, pt2, pt3, CV_RGB(0, 0, 255));
#endif
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
				snprintf(buf, len, "%s Ferr +/-%5.2f, Roll: %5.2f, Pitch: %5.2f, Yaw: %5.2f X: %5.2f(+/-%5.2f) Y: %5.2f(+/-%5.2f) Z: %5.2f(+/-%5.2f)", 
					 obj.name, (float)obj.delta_f_rmax,
					(float)obj.roll * 180./CV_PI, (float)obj.pitch * 180./CV_PI, (float)obj.yaw * 180./CV_PI,
					(float)obj.pos.x, (float)obj.delta_Tx_rmax, 
					(float)obj.pos.y, (float)obj.delta_Ty_rmax,
					(float)obj.pos.z, (float)obj.delta_Tz_rmax);

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
				render_prjpts(*obj.pmdl, obj.pt2dprj, m_pd3dev, NULL, m_pline, iobj, 0, m_cur_point);
			}
		}else if(m_op == POINT){
			if(iobj == m_cur_obj){
				drawPoint2d(m_pd3dev, 
					NULL, m_pline,
					obj.pt2d, obj.visible, iobj, 1);
				render_prjpts(*obj.pmdl, obj.pt2dprj, m_pd3dev, NULL, m_pline, iobj, 0, m_cur_point);
			}
		}else{
			drawPoint2d(m_pd3dev, 
				NULL, m_pline,
				obj.pt2d, obj.visible, 0);
			render_prjpts(*obj.pmdl, obj.pt2dprj, m_pd3dev, NULL, m_pline, iobj, 0, -1);
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
		render_axis(obj.pmdl, m_cam_int, m_cam_dist, obj.rvec, obj.tvec,
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
				ptr1[0] = -ptr0[0];
				ptr1[3] = ptr0[1];
				ptr1[6] = -ptr0[2];
				ptr1[1] = -ptr0[3];
				ptr1[4] = ptr0[4];
				ptr1[7] = -ptr0[5];
				ptr1[2] = -ptr0[6];
				ptr1[5] = ptr0[7];
				ptr1[8] = -ptr0[8];

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
			
			prjPts(objs[iobj]->pmdl->pts, pts, m_cam_int, m_cam_dist, rvec, tvec);
			if(m_cur_obj == iobj)
				render_prjpts(*objs[iobj]->pmdl, pts, m_pd3dev, NULL, m_pline, iobj, 0, -1);	
			else
				render_prjpts(*objs[iobj]->pmdl, pts, m_pd3dev, NULL, m_pline, iobj, 1, -1);
			render_axis(objs[iobj]->pmdl, m_cam_int, m_cam_dist, 
//				objs[iobj]->rvec, objs[iobj]->tvec, m_pd3dev, m_pline, -1);
				rvec, tvec, m_pd3dev, m_pline, -1);
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
	prjPts(pt3d, pt2d, fx, fy, cx, cy, m_cam_dist.ptr<double>(), R, t);

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
			break ;
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

	double * pcov = m_solver.Cov->data.db;

	// calculating covariance of the optimization error (hessian inverse of the last iteration)
	for(int iobj = 0; iobj < objs.size(); iobj++){
		double erx = *pcov;
		pcov += nparams + 1;
		double ery = *pcov;
		pcov += nparams + 1;
		double erz = *pcov;
		pcov += nparams + 1;
		double etx = *pcov;
		pcov += nparams + 1;
		double ety = *pcov;
		pcov += nparams + 1;
		double etz = *pcov;
		objs[iobj]->err_r = sqrt(erx + ery + erz) * 180./PI;
		objs[iobj]->err_t = sqrt(etx + ety + etz);
		cout << objs[iobj]->name << " Erot: " << objs[iobj]->err_r 
			<< " Etrn: " << objs[iobj]->err_t << endl;
	}

	// estimatin error in focal length and object depth using edge length analysis.
	// for all pairs of visible points, the differences between projected length and actual length are
	// calculated as delta_L. Then the delta_L is devided by dL/df or dL/dTz, and determines delta_f and delta_Tz for each pair.
	// Then the average, variance, min, max is reported.

	// dL/df is calculated with following formulae
	// [dLx/dfx dLx/dfx] = [1/(Z2+Tz)[X2+Tx] - 1/(Z1+Tz)[X1+Tx]                  0                   ]
	// [dLy/dfy dLy/dfy]   [              0                       1/(Z2+Tz)[Y2+Ty] - 1/(Z1+Tz)[Y1+Ty]]

	// dLx/dTz = [fx 0 ] [1/(Z1+Tz)^2[X1+Tx] - 1/(Z2+Tz)^2[X2+Tx]]
	// dLy/dTz   [0  fy] [1/(Z1+Tz)^2[Y1+Ty] - 1/(Z2+Tz)^2[Y2+Ty]]

	// we calculate four values
	// delta_Lx / (dLx/dfx) = delta fx
	// delta_Ly / (dLy/dfy) = delta_fy
	// delta_Lx / (dLx/dTz) = delta_Tzx
	// delta_Ly / (dLy/dTz) = delta_Tzy

	// counting total counts of pairs of visible points 
	for(int iobj = 0; iobj < objs.size(); iobj++){
		s_obj & obj = *objs[iobj];
		obj.analyze_error(pfrm->camint.at<double>(0, 0), pfrm->camint.at<double>(1, 1));
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
		if(m_pfrm_int->update && obj.update)
			continue;

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
	obj.sample_pt_tmpl(m_cur_point, m_img_gry_blur, m_sz_vtx_smpl);
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
	case 'g':
		m_brender_grid = !m_brender_grid;
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
	case 'u':
		if(m_pfrm_int){
			m_pfrm_int->update = false;
		}
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
			m_sop = SOP_SEEK_KF_FWD;
			m_bstep_issued = false;
		}
		break;
	case '<':
		if(m_bkfrm){
			m_sel_kfrm -= 1;
			if(m_sel_kfrm < 0)
				m_sel_kfrm += (int) m_kfrms.size();
		}else{
			m_sop = SOP_SEEK_KF_BK;
			m_bstep_issued = false;
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

	int num_objs = 0;
	long long tmin = LLONG_MAX;
	int ikf0 = 0;
	for(int ikf = 0; ikf < m_kfrms.size(); ikf++){
		if(!m_kfrms[ikf])
			continue;

		if(tmin > m_kfrms[ikf]->tfrm){
			tmin = m_kfrms[ikf]->tfrm;
			ikf0 = ikf;
		}
	}

	for(int i = 0, ikf = ikf0 ; i < m_kfrms.size(); i++, ikf = (ikf + 1) % m_kfrms.size()){
		if(!m_kfrms[ikf])
			continue;
		m_kfrms[ikf]->save(m_name);

		file << m_kfrms[ikf]->tfrm << endl;
	}
	
	file.close();

	vector<s_obj*> objptr;
	for(int i = 0, ikf = ikf0 ; i < m_kfrms.size(); i++, ikf = (ikf + 1) % m_kfrms.size()){
		if(!m_kfrms[ikf])
			continue;

		for(int iobj1 = 0; iobj1 < m_kfrms[ikf]->objs.size(); iobj1++){
			s_obj * pobj = m_kfrms[ikf]->objs[iobj1];
			bool found = false;
			for(int iobj2 = 0; iobj2 < objptr.size(); iobj2++){
				if(strcmp(objptr[iobj2]->name, pobj->name) == 0){
					found = true;
					break;
				}
			}

			if(!found){
				objptr.push_back(pobj);
				num_objs++;
			}
		}
	}

	if(!save_analytics(file, fname, ikf0, tmin, objptr, num_objs, -1, true))
		return false;
	if(!save_analytics(file, fname, ikf0, tmin, objptr, num_objs, -1, false))
		return false;
	if(!save_analytics(file, fname, ikf0, tmin, objptr, num_objs, m_cur_obj, true))
		return false;
	if(!save_analytics(file, fname, ikf0, tmin, objptr, num_objs, m_cur_obj, false))
		return false;

	return true;
}

bool f_inspector::save_analytics(ofstream & file, char * fname, int ikf0, long long tmin,
								 vector<s_obj*> & objptr,
								 int num_objs, int base_obj, bool bxyz)
{
	if(bxyz){
		if(base_obj < 0 || base_obj >= num_objs){
			snprintf(fname, 1024, "%s_cam_xyz.csv", m_name);
		}else{
			snprintf(fname, 1024, "%s_%s_xyz.csv", m_name, objptr[base_obj]->name);
		}
	}else{
		if(base_obj < 0 || base_obj >= num_objs){
			snprintf(fname, 1024, "%s_cam_zyx.csv", m_name);
		}else{
			snprintf(fname, 1024, "%s_%s_zyx.csv", m_name, objptr[base_obj]->name);
		}
	}

	file.open(fname);
	if(!file.is_open())
		return false;

	file << "Time(sec),cam,roll,pitch,yaw,x,y,z,";
	for(int iobj = 0; iobj < num_objs; iobj++){
		file << objptr[iobj]->name << ",";
		file << "roll,pitch,yaw,x,y,z,ferr,xerr,yerr,zerr,";
		if(m_bsave_objpts){
			for(int ipt = 0; ipt < objptr[iobj]->pmdl->pts_deformed.size(); ipt++){
				file << "pt[" << ipt << "].x,";
				file << "pt[" << ipt << "].y,";
				file << "pt[" << ipt << "].z,";
			}
		}
	}
	file << endl;

	Mat Rcam, Tcam;
	for(int i = 0, ikf = ikf0 ; i < m_kfrms.size(); i++, ikf = (ikf + 1) % m_kfrms.size()){
		if(!m_kfrms[ikf])
			continue;
		m_kfrms[ikf]->update = false;
		vector<vector<Point3f> > pts;
		if(m_bsave_objpts){
			m_kfrms[ikf]->calc_rpy_and_pts(pts, Rcam, Tcam, base_obj, bxyz);
		}else{
			m_kfrms[ikf]->calc_rpy(base_obj, bxyz);
		}

		file << (double)(m_kfrms[ikf]->tfrm - tmin) / (double)SEC << ",,";
		{
			double r,p,y;
			if(bxyz){
				angleRxyz(Rcam.ptr<double>(), r, p, y);
			}else{
				angleRzyx(Rcam.ptr<double>(), r, p, y);
			}

			file << r * 180. / CV_PI << "," << p * 180. / CV_PI << "," << y * 180. / CV_PI << ",";
			file << Tcam.ptr<double>()[0] << "," 
				<< Tcam.ptr<double>()[1] << ","
				<< Tcam.ptr<double>()[2] << ",";
		}

		vector<s_obj*> & objs = m_kfrms[ikf]->objs;
		for(int iobj1 = 0; iobj1 < objptr.size(); iobj1++){
			int iobj = -1;
			for(int iobj2 = 0; iobj2 < objs.size(); iobj2++){
				if(strcmp(objs[iobj2]->name, objptr[iobj1]->name) == 0){
					iobj = iobj2;
					break;
				}
			}
			if(iobj != -1){
				file << ",";
				file << objs[iobj]->roll * 180 / PI << ",";
				file << objs[iobj]->pitch * 180 / PI<< ",";
				file << objs[iobj]->yaw * 180 / PI << ",";
				file << objs[iobj]->pos.x << ",";
				file << objs[iobj]->pos.y << ",";
				file << objs[iobj]->pos.z << ",";
				file << objs[iobj]->delta_f_rmax << ",";
				file << objs[iobj]->delta_Tx_rmax_trn << ",";
				file << objs[iobj]->delta_Ty_rmax_trn << ",";
				file << objs[iobj]->delta_Tz_rmax_trn << ",";
				if(m_bsave_objpts){
					for(int ipt = 0; ipt < pts[iobj].size(); ipt++){
						file << pts[iobj][ipt].x << ",";
						file << pts[iobj][ipt].y << ",";
						file << pts[iobj][ipt].z << ",";
					}
				}
			}else{
				file << ",,,,,,,,,,,";
				if(m_bsave_objpts)
					file << ",,,";
			}
		}
		file << endl;
	}
	file.close();
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
		m_kfrms[ikf]->kfrm = true;
	}

	if(m_pfrm->tfrm == m_kfrms[m_cur_kfrm]->tfrm && m_pfrm != m_kfrms[m_cur_kfrm]){
		s_frame::free(m_pfrm);
		m_pfrm = m_kfrms[m_cur_kfrm];
	}

	if(m_kfrms[0] != NULL)
		m_sel_kfrm = 0;

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
#ifdef DEBUG_CHSBDDET
			if(pmdl->type == s_model::e_model_type::EMT_CHSBD && pobj){
				Mat chsbd = m_img_s.clone();
				drawChessboardCorners(chsbd, Size(pmdl->par_chsbd.w, pmdl->par_chsbd.h), pobj->pt2d, true);
				imwrite("chsbd_det.png", chsbd);
			}
#endif

			if(pobj == NULL)
				return;
			vector<s_obj*> & objs = m_pfrm_int->objs;
			pobj->sample_tmpl(m_img_gry_blur, m_sz_vtx_smpl);
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

void f_inspector::handle_sop_seek_kf(bool fwd)
{
	if(m_bstep_issued){
		if(m_bnew_frm){
			m_bstep_issued = false;
			if(m_pfrm->kfrm){
				m_sop = SOP_NULL;
				return;
			}
		}else{
			return;
		}
	}


	if(fwd){
		snprintf(m_cmd_buf, CMD_LEN, "step c %d", 1);
	}else{
		snprintf(m_cmd_buf, CMD_LEN, "step c -%d", 1);
	}

	bool ret;

	if(!m_paws->push_command(m_cmd_buf, m_cmd_ret, ret))
		cerr << "Unknown command issued." << endl;
	if(!ret){
		cerr << "Command " << m_cmd_buf << " Failed." << endl;
	}else{
		m_bstep_issued = true;
	}
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

