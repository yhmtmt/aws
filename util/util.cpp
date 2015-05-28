#include "stdafx.h"

// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// util.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// util.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with util.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "aws_thread.h"

#include "util.h"


bool test_exp_so3(const double * r, double * Rcv, double * jRcv)
{
	// R and jRcv are the OpenCV's rotation matrix and jacobian.
	double jR[27];
	double errjR[27];
	double R[9];
	double errR[9];
	exp_so3(r, R, jR);
	memset((void*)errjR, 0, sizeof(errjR));
	memset((void*)errR, 0, sizeof(errR));

	// jacobian is [dR/drx, dR/dry, dR/drz]
	// Note: OpenCV's jacobian assumes stacked row vector
	// exp_so3's jacobian assumes stacked column vector (if evaluated at zero, both are the same.)
	for(int i = 0; i < 9; i++){
		errR[i] = rerr(R[i], Rcv[i]);
	}

	for (int i = 0; i < 9; i++){
		for(int j = 0; j < 3; j++){
			errjR[i*3 + j] = rerr(jR[i*3 + j], jRcv[j*9+i]);
		}
	}

	bool res = true;
	for(int i = 0; i < 27; i++){
		if(errjR[i] > 0.1){
			cerr << "Jacobian[" << i << "] exp_so3 is erroneous." << endl;
			res = false;
		}
	}

	for(int j = 0; j < 9; j++){
		if(errR[j] > 0.001){
			cerr << "R[" << j << "] is erroneous." << endl;
			res = false;
		}
	}

	return res;
}

bool test_awsProjPtsj(Mat & camint, Mat & camdist, Mat & rvec, Mat & tvec, 
	vector<Point3f> & pt3d, vector<int> & valid, Mat & jacobian, double arf)
{
	int neq = pt3d.size() * 2;
	vector<Point2f> pt2d(pt3d.size());
	double * jf, * jc, * jk, * jp, * jr, * jt;
	jf = new double [neq * 2];
	jc = new double [neq * 2];
	jk = new double [neq * 6];
	jp = new double [neq * 2];
	jr = new double [neq * 3];
	jt = new double [neq * 3];

	awsProjPts(pt3d, pt2d, valid, camint, camdist, rvec, tvec,
		jf, jc, jk, jp, jr, jt, arf);
	// comparation with OpenCV's jacobian
	// OpenCV's jacobian has cols | rotation | translation | focal length | principal point | distortion coefficient |
	//                            |     3    |      3      |       2      |        2        |            8           |
	//                                                                                      |  2   |  2  |    4      |
	//                                                                                      | k1 k2 p1 p2 k3 k4 k5 k6
	int row_step = sizeof(double) * 18;
	double * jcv = jacobian.ptr<double>();
	double err[18] = {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

	double * _jf = jf, * _jc = jc, * _jk = jk, * _jp = jp, * _jr = jr, * _jt = jt;

	bool res = true;

	for(int i = 0; i < neq; i++){
		if(!valid[i/2])
			continue;

		// focal length
		err[6] = rerr(jf[0], jcv[6]);
		err[7] = rerr(jf[1], jcv[7]);
		_jf += 2;

		// principal point
		err[8] = rerr(jc[0], jcv[8]);
		err[9] = rerr(jc[0], jcv[9]);
		_jc += 2;

		// distortion coefficient
		err[10] = rerr(jk[0], jcv[10]);
		err[11] = rerr(jk[1], jcv[11]);
		err[12] = rerr(jk[0], jcv[12]);
		err[13] = rerr(jk[0], jcv[13]);
		err[14] = rerr(jk[0], jcv[14]);
		err[15] = rerr(jk[0], jcv[15]);
		err[16] = rerr(jk[0], jcv[16]);
		err[17] = rerr(jk[0], jcv[17]);
		_jk += 6;
		_jp += 2;

		err[0] = rerr(jr[0], jcv[0]);
		err[1] = rerr(jr[1], jcv[1]);
		err[2] = rerr(jr[2], jcv[2]);
		_jr += 3;

		err[3] = rerr(jt[0], jcv[3]);
		err[4] = rerr(jt[1], jcv[4]);
		err[5] = rerr(jt[2], jcv[5]);
		_jt += 3;

		for(int j = 0; j < 18; j++){
			if(err[j] > 0.001){
				cerr << "Jacobian parameter " << j << " in awsProjPts is erroneous." << endl;
				res = false;
			}
		}

		jcv += 18;
	}

	delete[] jf;
	delete[] jc;
	delete[] jk;
	delete[] jp;
	delete[] jr;
	delete[] jt;

	return res;
}


// awsProjPts
void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m,
	const Mat & camint, const Mat & camdist, const Mat & rvec, const Mat & tvec)
{
	const double * R, * t, * k;
	Mat _R;
	Rodrigues(rvec, _R);
	R = _R.ptr<double>();
	t = tvec.ptr<double>();
	k = camdist.ptr<double>();
	const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
		cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

	m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		awsProjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
	}
}

void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m,
		const double fx, const double fy, const double cx, const double cy,
	const double * k, const double * R, const double * t)
{
	m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		awsProjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
	}
}


void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m, const vector<int> & valid,
	const Mat & camint, const Mat & camdist, const Mat & rvec, const Mat & tvec)
{
	const double * R, * t, * k;
	Mat _R;
	Rodrigues(rvec, _R);
	R = _R.ptr<double>();
	t = tvec.ptr<double>();
	k = camdist.ptr<double>();
	const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
		cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

	m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		if(!valid[i])
			continue;

		awsProjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
	}
}

void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m, const vector<int> & valid,
		const double fx, const double fy, const double cx, const double cy,
	const double * k, const double * R, const double * t)
{
	m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		if(!valid[i])
			continue;
		awsProjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
	}
}

void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m, const vector<int> & valid,
		const Mat & camint, const Mat & camdist,
		const Mat & rvec, const Mat & tvec,
		double * jf, double * jc, double * jk, double * jp,
		double * jr, double * jt, double arf)
{
	// if arf equals to 0.0, the jacobian for jf is calculated for fx and fy distinctively. (jf is 2Nx2 array)
	// Otherwise, jf is caluclated related only to fy. (jf is 2Nx1 array)

	const double * t, * k;

	double R[9], jR[27];
	Mat _R, _jR;
	Rodrigues(rvec, _R, _jR);
	if(!test_exp_so3(rvec.ptr<double>(), _R.ptr<double>(), _jR.ptr<double>())){
		cerr << "exp_so3 may be erroneous." << endl;
	}

	t = tvec.ptr<double>();
	k = camdist.ptr<double>();
	const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
		cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

	if(M.size() != m.size())
		m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		if(!valid[i])
			continue;

		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
		double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
		double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2, D, D2;
		double xd, yd;

		z = z ? 1./z : 1;
		x *= z; y *= z;
		
		r2 = x*x + y*y;
		r4 = r2*r2;
		r6 = r4*r2;
		a1 = 2*x*y;
		a2 = r2 + 2*x*x;
		a3 = r2 + 2*y*y;
		cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
		icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
		D = cdist * icdist2;
		D2 = D * icdist2;
		xd = x * D + k[2]*a1 + k[3]*a2;
		yd = y * D + k[2]*a3 + k[3]*a1;

		m[i].x = (float)(xd*fx + cx);
		m[i].y = (float)(yd*fy + cy);

		// calculate jacboian for fx, fy ( this version is free aspect ratio)
		if(arf == 0.0){
			jf[0] = xd; jf[1] = 0;
			jf += sizeof(double) * 2;
			jf[0] = 0; jf[1] = yd;
			jf += sizeof(double) * 2;
		}else{
			jf[0] = xd * arf;
			jf += sizeof(double);
			jf[0] = yd;
			jf += sizeof(double);
		}

		// calculate jacobian for cx, cy
		jc[0] = 1; jc[1] = 0;
		jc += sizeof(double) * 2;
		jc[0] = 0; jc[1] = 1;
		jc += sizeof(double) * 2;

		// calculate jacobian for radial distortion coefficient
		jk[0] = fx * x * r2 * icdist2; 
		jk[1] = fx * x * r4 * icdist2; 
		jk[2] = fx * x * r6 * icdist2;
		jk[3] = -fx * x * r2 * D2;
		jk[4] = -fx * x * r4 * D2;
		jk[5] = -fx * x * r6 * D2;
		jk += sizeof(double) * 6;
		jk[0] = fy * y * r2 * icdist2; 
		jk[1] = fy * y * r4 * icdist2; 
		jk[2] = fy * y * r6 * icdist2;
		jk[3] = -fy * y * r2 * D2;
		jk[4] = -fy * y * r4 * D2;
		jk[5] = -fy * y * r6 * D2;
		jk += sizeof(double) * 6;

		// calculate jacobian for tangential distortion coefficient
		jp[0] = a1; jp[1] = a2;
		jp += sizeof(double) * 2;
		jp[0] = a3; jp[1] = a1;
		jp += sizeof(double) * 2;

		double dDdl2 = (3 * k[4] * r4 + 2 * k[1] * r2 + k[0]) * icdist2 
			- (3 * k[7] * r4 + 2 * k[6] * r2 + k[5]) * D2;
		double dl2dxp = 2*x;
		double dl2dyp = 2*y;

		double dxdz = -x * z; // here x and y has already been mutiplied with 1/z. And note that z is actuall 1/z here.
		double dydz = -y * z;

		// calculate jacobian for translation
		double dxdxp = fx * (D + x * dDdl2 * dl2dxp + 2 * k[2] * y + k[3] * (dl2dxp + 4 * x));
		double dxdyp = fx * (x * dDdl2 * dl2dyp + 2 * k[2] * x + k[3] * dl2dyp);
		double dydxp = fy * (y * dDdl2 * dl2dxp + 2 * k[3] * y + k[2] * dl2dxp);
		double dydyp = fy * (D + y * dDdl2 * dl2dxp + 2 * k[3] * x + k[2] * (dl2dyp + 4 * y));
		jt[0] = dxdxp * z; jt[1] = dxdyp * z; jt[2] = dxdxp * dxdz + dxdyp * dydz;
		jt[3] = dydxp * z; jt[4] = dydyp * z; jt[5] = dydxp * dxdz + dydyp * dydz;

		// calculate jacobian for rotation 
		double dXdr[3] = {
			X * jR[0] + Y * jR[3] + Z * jR[6], 
			X * jR[1] + Y * jR[4] + Z * jR[7], 
			X * jR[2] + Y * jR[5] + Z * jR[8]
		};

		double dYdr[3] = {
			X * jR[9] +  Y * jR[12] + Z * jR[15], 
			X * jR[10] + Y * jR[13] + Z * jR[16], 
			X * jR[11] + Y * jR[14] + Z * jR[17]
		};

		double dZdr[3] = {
			X * jR[18] + Y * jR[21] + Z * jR[24], 
			X * jR[19] + Y * jR[22] + Z * jR[25], 
			X * jR[20] + Y * jR[23] + Z * jR[26]
		};

		jr[0] = jt[0] * dXdr[0] + jt[1] * dYdr[0] + jt[2] * dZdr[0];
		jr[1] = jt[0] * dXdr[1] + jt[1] * dYdr[1] + jt[2] * dZdr[1];
		jr[2] = jt[0] * dXdr[2] + jt[1] * dYdr[2] + jt[2] * dZdr[2];

		jr[3] = jt[3] * dXdr[0] + jt[4] * dYdr[0] + jt[5] * dZdr[0];
		jr[4] = jt[3] * dXdr[1] + jt[4] * dYdr[1] + jt[5] * dZdr[1];
		jr[5] = jt[3] * dXdr[2] + jt[4] * dYdr[2] + jt[5] * dZdr[2];
	}
};


void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m, const vector<int> & valid,
		const Mat & camint, const Mat & camdist,
		const Mat & rvec, const Mat & tvec,
		double * jr, double * jt)
{
	const double * t, * k;

	double R[9], jR[27];
	//Rodrigues(rvec, _R, _jR);
	exp_so3(rvec.ptr<double>(), R, jR);

	t = tvec.ptr<double>();
	k = camdist.ptr<double>();
	const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
		cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

	if(M.size() != m.size())
		m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		if(!valid[i])
			continue;

		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
		double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
		double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2, D, D2;
		double xd, yd;

		z = z ? 1./z : 1;
		x *= z; y *= z;
		
		r2 = x*x + y*y;
		r4 = r2*r2;
		r6 = r4*r2;
		a1 = 2*x*y;
		a2 = r2 + 2*x*x;
		a3 = r2 + 2*y*y;
		cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
		icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
		D = cdist * icdist2;
		D2 = D * icdist2;
		xd = x * D + k[2]*a1 + k[3]*a2;
		yd = y * D + k[2]*a3 + k[3]*a1;

		m[i].x = (float)(xd*fx + cx);
		m[i].y = (float)(yd*fy + cy);

		double dDdl2 = (3 * k[4] * r4 + 2 * k[1] * r2 + k[0]) * icdist2 
			- (3 * k[7] * r4 + 2 * k[6] * r2 + k[5]) * D2;
		double dl2dxp = 2*x;
		double dl2dyp = 2*y;

		double dxdz = -x * z; // here x and y has already been mutiplied with 1/z. And note that z is actuall 1/z here.
		double dydz = -y * z;

		// calculate jacobian for translation
		double dxdxp = fx * (D + x * dDdl2 * dl2dxp + 2 * k[2] * y + k[3] * (dl2dxp + 4 * x));
		double dxdyp = fx * (x * dDdl2 * dl2dyp + 2 * k[2] * x + k[3] * dl2dyp);
		double dydxp = fy * (y * dDdl2 * dl2dxp + 2 * k[3] * y + k[2] * dl2dxp);
		double dydyp = fy * (D + y * dDdl2 * dl2dxp + 2 * k[3] * x + k[2] * (dl2dyp + 4 * y));
		jt[0] = dxdxp * z; jt[1] = dxdyp * z; jt[2] = dxdxp * dxdz + dxdyp * dydz;
		jt[3] = dydxp * z; jt[4] = dydyp * z; jt[5] = dydxp * dxdz + dydyp * dydz;

		// calculate jacobian for rotation 
		double dXdr[3] = {
			X * jR[0] + Y * jR[3] + Z * jR[6], 
			X * jR[1] + Y * jR[4] + Z * jR[7], 
			X * jR[2] + Y * jR[5] + Z * jR[8]
		};

		double dYdr[3] = {
			X * jR[9] +  Y * jR[12] + Z * jR[15], 
			X * jR[10] + Y * jR[13] + Z * jR[16], 
			X * jR[11] + Y * jR[14] + Z * jR[17]
		};

		double dZdr[3] = {
			X * jR[18] + Y * jR[21] + Z * jR[24], 
			X * jR[19] + Y * jR[22] + Z * jR[25], 
			X * jR[20] + Y * jR[23] + Z * jR[26]
		};

		jr[0] = jt[0] * dXdr[0] + jt[1] * dYdr[0] + jt[2] * dZdr[0];
		jr[1] = jt[0] * dXdr[1] + jt[1] * dYdr[1] + jt[2] * dZdr[1];
		jr[2] = jt[0] * dXdr[2] + jt[1] * dYdr[2] + jt[2] * dZdr[2];

		jr[3] = jt[3] * dXdr[0] + jt[4] * dYdr[0] + jt[5] * dZdr[0];
		jr[4] = jt[3] * dXdr[1] + jt[4] * dYdr[1] + jt[5] * dZdr[1];
		jr[5] = jt[3] * dXdr[2] + jt[4] * dYdr[2] + jt[5] * dZdr[2];
	}
};



// Synthesize two affine trasformations
// res = l * r 
bool synth_afn(Mat & l, Mat & r, Mat & res)
{
	if(l.cols != 3 || r.cols != 3)
		return false;
	if(l.rows != 2 || r.rows != 2)
		return false;
	if(l.type() != r.type())
		return false;

	double * lptr, * rptr[2], * resptr;
	int irow, icol;
	res = Mat::zeros(2, 3, CV_64FC1);

	rptr[0] = r.ptr<double>(0);
	rptr[1] = r.ptr<double>(1);
	for(irow = 0; irow < 2; irow++){
		lptr = l.ptr<double>(irow);
		resptr = res.ptr<double>(irow);
		for(icol = 0; icol < 3; icol++)
			resptr[icol] += lptr[0] * rptr[0][icol] + lptr[1] * rptr[1][icol];

		resptr[2] += lptr[2];
	}
	return true;
}

bool afn(Mat & A, Point2f & pt_in, Point2f & pt_out)
{
	if(A.cols != 3 || A.rows != 2)
		return false;

	double *ptr;

	ptr = A.ptr<double>(0);
	pt_out.x = (float)(pt_in.x * ptr[0] + pt_in.y * ptr[1] + ptr[2]);
	ptr = A.ptr<double>(1);
	pt_out.y = (float)(pt_in.x * ptr[0] + pt_in.y * ptr[1] + ptr[2]);
	return true;
}


// R G
// G B
void cnvBayerRG8ToBGR8(Mat & src, Mat & dst)
{
	if(src.type() != CV_8UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_8UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
	int next_skip = src.cols - step_size + 2;

	unsigned char * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned char*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned char*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}

void cnvBayerRG16ToBGR16(Mat & src, Mat & dst)
{
	if(src.type() != CV_16UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_16UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
	int next_skip = src.cols - step_size + 2;

	unsigned short * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned short*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned short*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}


// G R
// B G
void cnvBayerGR8ToBGR8(Mat & src, Mat & dst)
{
	if(src.type() != CV_8UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_8UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
	int next_skip = src.cols - step_size + 2;

	unsigned char * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned char*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned char*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}

void cnvBayerGR16ToBGR16(Mat & src, Mat & dst)
{
	if(src.type() != CV_16UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_16UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
	int next_skip = src.cols - step_size + 2;

	unsigned short * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned short*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned short*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;


			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}


// B G
// G R
void cnvBayerBG8ToBGR8(Mat & src, Mat & dst)
{
	if(src.type() != CV_8UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_8UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
	int next_skip = src.cols - step_size + 2;

	unsigned char * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned char*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned char*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}

void cnvBayerBG16ToBGR16(Mat & src, Mat & dst)
{
	if(src.type() != CV_16UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_16UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
	int next_skip = src.cols - step_size + 2;

	unsigned short * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned short*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned short*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}

// G B
// R G
void cnvBayerGB8ToBGR8(Mat & src, Mat & dst)
{
	if(src.type() != CV_8UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_8UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned char));
	int next_skip = src.cols - step_size + 2;

	unsigned char * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned char*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned char*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}

void cnvBayerGB16ToBGR16(Mat & src, Mat & dst)
{
	if(src.type() != CV_16UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_16UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
	int next_skip = src.cols - step_size + 2;

	unsigned short * psrc0, * psrc1, * psrc2, * pdst;
	psrc0 = (unsigned short*) src.data + 1;
	psrc1 = psrc0 + step_size;
	psrc2 = psrc1 + step_size;
	pdst = (unsigned short*) dst.data;

	for(int y = 0; y < dst.rows; y++){
		for(int x = 0; x < dst.cols; x++){ // even y
			//even x
			//blue
// y % 2 == 0 && x % 2 == 1 -> b(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 1 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 0 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;
			//green
// y % 2 == 0 && x % 2 == 0 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 0 && x % 2 == 0 -> r(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		y++;
		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;

		for(int x = 0; x < dst.cols; x++){ // odd y
			//even x
			//blue
// y % 2 == 1 && x % 2 == 1 -> b(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 1 -> g(x,y) = (bayer(x-1,y) + bayer(x+1,y) + bayer(x,y-1) + bayer(x,y+1)) >> 2
			*pdst = (psrc1[-1]+psrc1[+1]+psrc0[0]+psrc2[0]) >> 2;
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 1 -> r(x,y) = (bayer(x-1,y-1) + bayer(x-1,y+1) + bayer(x+1,y-1) + bayer(x+1,y+1)) >> 2
			*pdst = (psrc0[-1]+psrc0[+1]+psrc2[-1]+psrc2[+1]) >> 2;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
			x++;

			// odd x
			//blue
// y % 2 == 1 && x % 2 == 0 -> b(x,y) = (bayer(x-1,y) + bayer(x+1,y)) >> 1
			*pdst = (psrc1[-1]+psrc1[+1]) >> 1;
			pdst++;
			//green
// y % 2 == 1 && x % 2 == 0 -> g(x,y) = bayer(x,y)
			*pdst = psrc1[0];
			pdst++;
			//red
// y % 2 == 1 && x % 2 == 0 -> r(x,y) = (bayer(x,y-1) + bayer(x,y+1)) >> 1
			*pdst = (psrc0[0]+psrc2[0]) >> 1;
			pdst++;

			psrc0++;
			psrc1++;
			psrc2++;
		}

		psrc0 += next_skip;
		psrc1 += next_skip;
		psrc2 += next_skip;
	}
}
