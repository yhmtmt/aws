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

inline void angleRxyz(const double * R, double & x, double & y, double &z)
{
	// R0  R1  R2 
	// R3  R4  R5
	// R6  R7  R8

	x = asin(-R[2]);
	z = atan(-R[1]/R[0]);
	y = atan(-R[5]/R[7]);
}

// Lie-gropu to Lie-algebra mapping function

// SO(3)->so(3) log Rodrigues gives this mapping
inline void log_so3(const double * R, double * r)
{
	double rx, ry, rz, s, c, theta;

	// Note rx, ry, rz are often used as temporal variable. Be careful.
	// here calculating
	// 2s * rx     R32 - R23
	// 2s * ry  =  R13 - R31
	// 2s * rz     R21 - R12
	rx = R[7] - R[5];
	ry = R[2] - R[6];
	rz = R[3] - R[1];

	// s^2 = sqrt((2s * rx)^2 + (2s * ry)^2 + (2s * rz)^2)/2
	// note: here the sign of the sin is missed. But we dont need to distinguish it.
	s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);

	// trace(R)-1 gives 2cos(theta)
	c = (R[0] + R[4] + R[8] - 1)*0.5;

	// clamping cosine in (-1,1)
	c = c > 1. ? 1. : c < -1. ? -1. : c;
	theta = acos(c);

	if( s < 1e-5 )
	{
		if( c > 0 ) // theta -> 0
			rx = ry = rz = 0;
		else // theta -> PI
		{
			// This relys on quarternion.

			// SO(3) to SU(2)
			// Quarternion |q| = (q0, q1, q2, q3)
			// Here I assume sgn(q0) is positive
			// q0 = sqrt(0.25(R11+R22+R33+1))=sqrt(0.25*(2+2cos th)=sqrt(cos^2th/2)
			// q1 = sqrt(0.25(R11-R22-R33+1)) sgn(R32-R23) = sqrt(R11-cos(th))sgn(R32-R23) = sin th/2 rx sgn(q0q1)
 			// q2 = sqrt(0.25(-R11+R22-R33+1)) sgn(R13-R31) = sqrt(R22-cos(th))sgn(R13-R31) = sin th/2 ry sgn(q0q2)
			// q3 = sqrt(0.25(-R11-R22+R33+1)) sgn(R21-R12) = sqrt(R33-cos(th))sgn(R21-R12) = sin th/2 rz sgn(q0q3)

			// SU(2) to so(3)
			// q = cos th/2 + n sin th/2 ... where n is unit vector
			// cos th/2 = q0
			// sin th/2 = |(q1,q2, q3)|
			// r = (q1,q2,q3) / (sin th/2)

/* These are the OpenCV's Rodrigues. It may be coded assuming quarternion, but I think it's wrong.
			t = (R[0] + 1)*0.5;  
			rx = sqrt(MAX(t,0.));
			t = (R[4] + 1)*0.5; 
			ry = sqrt(MAX(t,0.))*(R[1] < 0 ? -1. : 1.);
			t = (R[8] + 1)*0.5; 
			rz = sqrt(MAX(t,0.))*(R[2] < 0 ? -1. : 1.);
			if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R[5] > 0) != (ry*rz > 0) )
				rz = -rz;
*/		
			// calculatign q1,q2,q3, we do not need to calculate q0 because theta has already been calculated as theta.
			// at this line, rx = R32-R23, ry = R13-R31, rz = R21-R12, calculated in the begining
			// their sign is just those of resulting q1, q2, q3 respectively!
			// Furthermore, the cosine value has also been calculated as c.
			rx = sqrt(max(R[0] - c, 0.)) * (rx < 0 ? -1.: 1.); // q1 
			ry = sqrt(max(R[4] - c, 0.)) * (ry < 0 ? -1.: 1.); // q2
			rz = sqrt(max(R[8] - c, 0.)) * (rz < 0 ? -1. :1.); // q3 

			theta /= sqrt(rx*rx + ry*ry + rz*rz); // sin th/2
			rx *= theta; // * theta / sin th/2 => theta rx
			ry *= theta; // * theta / sin th/2 => theta ry
			rz *= theta; // * theta / sin th/2 => theta rz
		}
	}
	else
	{
		double vth = 1/(2*s);
		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}
}

// so(3)->SO(3) exp Rodrigues gives this mapping
inline void exp_so3(const double * r, double * R)
{
	double rx = r[0], ry = r[1], rz = r[2];
	double theta = sqrt(rx*rx + ry*ry + rz*rz);
	if(theta < DBL_EPSILON){
		memset((void*) R, 0, sizeof(double) * 9);
		R[0] = R[4] = R[8] = 1.0;
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

	// Resulting matrix R
	// R0  R1  R2 
	// R3  R4  R5
	// R6  R7  R8

	//
	// R=I + s A + (1-c) A^2
	// ic rx2 + c      ic rxry - s rz  ic rxrz + s ry
	// ic rxry + s rz  ic ry2 + c      ic ryrz - s rx
	// ic rxrz - s ry  ic ryrz + s rx  ic rz2 + c
	double s = sin(theta);
	double c = cos(theta);
	double ic = 1.0 - c;
	double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
	double srx = s * rx, sry = s * ry, srz = s * rz;
	R[0] = ic * rx2 + c; R[1] = icrxry - srz; R[2] = icrxrz + sry;
	R[3] = icrxry + srz; R[4] = ic * ry2 + c; R[5] = icryrz - srx;
	R[6] = icrxrz - sry; R[7] = icryrz + srx; R[8] = ic * rz2 +  c;
}

// SE(3)->se(3) log 
inline void log_se3(const double * T, double * r, double * v)
{
	double rx, ry, rz, s, c, theta;

	// T0  T1  T2  T3
	// T4  T5  T6  T7
	// T8  T9  T10 T11
	// T12 T13 T14 T15

	// Note rx, ry, rz are often used as temporal variable. Be careful.
	// here calculating
	// 2s * rx     R32 - R23
	// 2s * ry  =  R13 - R31
	// 2s * rz     R21 - R12
	rx = T[9] - T[6];
	ry = T[2] - T[8];
	rz = T[4] - T[1];

	// s^2 = sqrt((2s * rx)^2 + (2s * ry)^2 + (2s * rz)^2)/2
	s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);

	// trace(R)-1 gives 2cos(theta)
	c = (T[0] + T[5] + T[10] - 1)*0.5;

	// clamping cosine in (-1,1)
	c = c > 1. ? 1. : c < -1. ? -1. : c;
	theta = acos(c);

	if( s < 1e-5 )
	{
		if( c > 0 ) // theta -> 0
			rx = ry = rz = 0;
		else // theta -> PI
		{
			rx = sqrt(max(T[0] - c, 0.)) * (rx < 0 ? -1.: 1.); // q1 
			ry = sqrt(max(T[5] - c, 0.)) * (ry < 0 ? -1.: 1.); // q2
			rz = sqrt(max(T[10] - c, 0.)) * (rz < 0 ? -1. :1.); // q3 

			double vth = 1.0 / sqrt(rx*rx + ry*ry + rz*rz);
			rx *= vth; // / sin th/2 => rx
			ry *= vth; // / sin th/2 => ry
			rz *= vth; // / sin th/2 => rz
		}
	}
	else
	{
		double vth = 1/(2*s);
		rx *= vth; ry *= vth; rz *= vth;
	}

	double rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz, 
		rxry = rx * ry, rxrz = rx * rz, ryrz = ry * rz;

	double itheta = 1.0 / theta;
	double ic = 1.0 - c;
	double icrxry = ic * rxry, icryrz = ic * ryrz, icrxrz = ic * rxrz;
	double srx = s * rx, sry = s * ry, srz = s * rz;
	double ict = ic * itheta, st = s * itheta, ist = 1 - st;
	double istrxry = ist * rxry, istrxrz = ist * rxrz, istryrz = ist * ryrz;
	double ictrx = ict * rx, ictry = ict * ry, ictrz = ict * rz;

	//multiply V^t to T. (V is the rotation matrix appeared in the exponential map)
	v[0] = (ist * rx2 + st)  * T[3] + (istrxry + ictrz) * T[7] + (istrxrz - ictry) * T[11];
	v[1] = (istrxry - ictrz) * T[3] + (ist * ry2 + st)  * T[7] + (istryrz + ictrx) * T[11];
	v[2] = (istrxrz + ictry) * T[3] + (istryrz - ictrx) * T[7] + (ist * rz2 + st)  * T[11];
	r[0] = theta * rx;
	r[1] = theta * ry;
	r[2] = theta * rz;
}

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
	T[0] = ic * rx2 + c; T[1] = icrxry - srz; T[2] = icrxrz + sry;
	T[4] = icrxry + srz; T[5] = ic * ry2 + c; T[6] = icryrz - srx;
	T[8] = icrxrz - sry; T[9] = icryrz + srx; T[10] = ic * rz2 +  c;

	// calculating rotation of the translation
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
inline void log_sim3(const double * S, double & s, double * r, double * v)
{
	double es = sqrt(S[0] * S[0] + S[1] * S[1] + S[2] * S[2]);
	double ies = 1.0 / es;
	double T[16];
	memcpy((void*) T, (void*) S, sizeof(double) * 16);

	T[0] *= ies;
	T[1] *= ies;
	T[2] *= ies;
	T[4] *= ies;
	T[5] *= ies;
	T[6] *= ies;
	T[8] *= ies;
	T[9] *= ies;
	T[10] *= ies;

	log_se3(T, r, v);
	s = log(es);
}

// sim(3)->SIM(3) exp
inline void exp_sim3(double & s, const double * r, const double * v, double * S)
{
	exp_se3(r, v, S);
	double es = exp(s);
	S[0] *= es;
	S[1] *= es;
	S[2] *= es;
	S[4] *= es;
	S[5] *= es;
	S[6] *= es;
	S[8] *= es;
	S[9] *= es;
	S[10] *= es;
}

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