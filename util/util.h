// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// util.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// util.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with util.h.  If not, see <http://www.gnu.org/licenses/>. 
#ifndef _UTIL_H_
#define _UTIL_H_
#include "aws_sock.h"
#include "aws_thread.h"
#include "aws_stdlib.h"

// Lie-gropu to Lie-algebra mapping function
// SO(3)->so(3) log Rodrigues gives this mapping 
// so(3)->SO(3) exp Rodrigues gives this mapping
// SE(3)->se(3) log 
// se(3)->SE(3) exp
inline void exp_se3(const double * r, const double * v, double * T)
{
	double rx = r[0], ry = r[1], rz = r[2];
	double theta = sqrt(rx*rx + ry*ry + rz*rz);
	if(theta < DBL_EPSILON){
		memset((void*) T, 0, sizeof(double) * 16);
		T[0] = T[5] = T[10] = T[15] = 1.0;
		return;
	}

	double itheta = 1.0 / theta;

	rx *= itheta;
	ry *= itheta;
	rz *= itheta;

	double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
		rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

	// skew symmetric matrix A
	// 0   -rz   ry
	// rz    0  -rx
	// -ry  rx    0

	// A^2
	// -rz2-ry2 rxry     rxrz
	// rxry     -rx2-rz2 ryrz
	// rxrz     ryrz     -rx2-ry2
	//
	// rx2+ry2+rz2=1.0 simplify A^2 as
	// rx2-1    rxry     rxrz
	// rxry     ry2-1    ryrz
	// rxrz     ryrz     rz2-1

	// Resulting matrix
	// T0  T1  T2  T3
	// T4  T5  T6  T7
	// T8  T9  T10 T11
	// T12 T13 T14 T15
	// 
	// R t
	// 0 1
	
	// R =
	// T0  T1  T2 
	// T4  T5  T6 
	// T8  T9  T10 
	//
	// t = 
	// T3
	// T7
	// T11

	//
	// R=I + s A + (1-c) A^2
	// ic(rx2-1) + c   ic rxry - s rz  ic rxrz + s ry
	// ic rxry + s rz  ic(ry2-1) + c   ic ryrz - s rx
	// ic rxrz - s ry  ic ryrz + s rx  ic(rz2-1) + c
	double s = sin(theta);
	double c = cos(theta);
	double ic = 1.0 - c;
	double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
	double srx = s * rx, sry = s * ry, srz = s * rz;
	T[0] = ic * (rx2 - 1.) + c; T[1] = icrxry - srz;        T[2] = icrxrz + sry;
	T[4] = icrxry + srz;        T[5] = ic * (ry2 - 1.) + c; T[6] = icryrz - srx;
	T[8] = icrxrz - sry;        T[9] = icryrz + srx;        T[10] = ic * (rz2 - 1) +  c;

	// V=I + (1-c)/th A + (1-s/th) A^2
	// (1-s/th) rx2 + s/th          (1-s/th) rxry - (1-c)/th rz (1-s/th) rxrz + (1-c)/th ry
	// (1-s/th) rxry + (1-c)/th rz  (1-s/th) ry2 + s/th         (1-s/th) ryrz - (1-c)/th rx
	// (1-s/th) rxrz - (1-c)/th ry  (1-s/th) ryrz + (1-c)/th rx (1-s/th)rz2 + s/th
	//
	// t = Vv
	double ict = ic * itheta, st = s * itheta, ist = 1 - st;
	double istrxry = ist * rxry, istrxrz = ist * rxrz, istryrz = ist * ryrz;
	double ictrx = ict * rx, ictry = ict * ry, ictrz = ict * rz;
	T[3]  = (ist * rx2 + st)  * v[0] + (istrxry - ictrz) * v[1] + (istrxrz + ictry) * v[2];
	T[7]  = (istrxry + ictrz) * v[0] + (ist * ry2 + st)  * v[1] + (istryrz - ictrx) * v[2];
	T[11] = (istrxrz - ictry) * v[0] + (istryrz + ictrx) * v[1] + (ist * rz2 + st)  * v[2];

	T[12] = T[13] = T[14] = 0.;
	T[15] = 1.0;
}

// SIM(3)->sim(3) log
// sim(3)->SIM(3) exp

// these codes are partially from OpenCV calibrate.cpp
inline void awsProjPt(const Point3f & M, Point2f & m, 
	const Mat & camint, const Mat & camdist, 
	const Mat & rvec, const Mat & tvec)
{
	const double * R, * t, * k;
	Mat _R;
	Rodrigues(rvec, _R);
	R = _R.ptr<double>();
	t = tvec.ptr<double>();
	k = camdist.ptr<double>();
	const double fx = camint.at<double>(0,0), fy = camint.at<double>(1,1),
		cx = camint.at<double>(0,2), cy = camint.at<double>(1,2);

	double X = M.x, Y = M.y, Z = M.z;
	double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
	double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
	double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
	double r2, r4, r6, a1, a2, a3, cdist, icdist2;
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
	xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
	yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

	m.x = (float)(xd*fx + cx);
	m.y = (float)(yd*fy + cy);
}

inline void awsProjPt(const Point3f & M, Point2f & m,
	const double fx, const double fy, const double cx, const double cy,
	const double * k, const double * R, const double * t)
{
	double X = M.x, Y = M.y, Z = M.z;
	double x = R[0]*X + R[1]*Y + R[2]*Z + t[0];
	double y = R[3]*X + R[4]*Y + R[5]*Z + t[1];
	double z = R[6]*X + R[7]*Y + R[8]*Z + t[2];
	double r2, r4, r6, a1, a2, a3, cdist, icdist2;
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
	xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2;
	yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1;

	m.x = (float)(xd*fx + cx);
	m.y = (float)(yd*fy + cy);
}

inline void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m,
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

inline void awsProjPts(const vector<Point3f> & M, vector<Point2f> & m,
		const double fx, const double fy, const double cx, const double cy,
	const double * k, const double * R, const double * t)
{
	m.resize(M.size());

	for(int i = 0; i < M.size(); i++){
		awsProjPt(M[i], m[i], fx, fy, cx, cy, k, R, t);
	}
}



bool synth_afn(Mat & l, Mat & r, Mat & res);
bool afn(Mat & A, Point2f & in, Point2f & pt_out);

void cnvBayerRG8ToBGR8(Mat & src, Mat & dst);
void cnvBayerRG16ToBGR16(Mat & src, Mat & dst);
void cnvBayerGR8ToBGR8(Mat & src, Mat & dst);
void cnvBayerGR16ToBGR16(Mat & src, Mat & dst);
void cnvBayerGB8ToBGR8(Mat & src, Mat & dst);
void cnvBayerGB16ToBGR16(Mat & src, Mat & dst);
void cnvBayerBG8ToBGR8(Mat & src, Mat & dst);
void cnvBayerBG16ToBGR16(Mat & src, Mat & dst);

// box-muller random normal variable
double nrand(double u, double s);

// comparison function used in the map 
struct cmp { 
	bool operator () (const char *a,const char *b) const 
	{
		return strcmp(a,b) < 0;
	} 
};

// HEX char to integer

inline unsigned char h2i(char h){
	unsigned char i;
	i = h - '0';
	if(i < 10)
		return i;
	i = h - 'A' + 10;
	return i;
}
#endif