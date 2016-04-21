#include "stdafx.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// coord.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// coord.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with coord.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
using namespace cv;

#include "aws_coord.h"

void bihtoecef(const float lat, const float lon, const float alt, 
			   float & x, float & y, float & z)
{
	double slat = sin(lat);
	double clat = cos(lat);
	double slon = sin(lon);
	double clon = cos(lon);
	double N = AE / sqrt(1 - EE2 * slat * slat);

	double tmp = (N + alt) * clat;
	x = (float)(tmp * clon);
	y = (float)(tmp * slon);
	z = (float)((N * (1 - EE2) + alt) * slat);
}

void bihtoecef(const s_bihpos & Xbih, Point3d & Xecef)
{
	double slat = sin(Xbih.lat);
	double clat = cos(Xbih.lat);
	double slon = sin(Xbih.lon);
	double clon = cos(Xbih.lon);
	double N = AE / sqrt(1 - EE2 * slat * slat);

	double tmp = (N + Xbih.alt) * clat;
	Xecef.x = tmp * clon;
	Xecef.y = tmp * slon;
	Xecef.z = (N * (1 - EE2) + Xbih.alt) * slat;
}

void eceftobih(const float x, const float y, const float z, float & lat, float & lon, float & alt)
{
	double p = sqrt(x* x + y * y);
	double th = atan(z * AE / (p * BE));
	double s = sin(th);
	double c = cos(th);
	s = s * s * s;
	c = c * c* c;
	lat = (float)atan2(z + EE2_B * s, p - EE2_A * c);
	lon = (float)atan2(y, x);
	s = (float)sin(lat);
	alt = (float)(p / cos(lat) - AE / sqrt(1 - EE2 * s * s));
}

void eceftobih(Point3d & Xecef, s_bihpos & Xbih)
{
	double p = sqrt(Xecef.x* Xecef.x + Xecef.y * Xecef.y);
	double th = atan(Xecef.z * AE / (p * BE));
	double s = sin(th);
	double c = cos(th);
	s = s * s * s;
	c = c * c* c;
	Xbih.lat = atan2(Xecef.z + EE2_B * s, p - EE2_A * c);
	Xbih.lon = atan2(Xecef.y, Xecef.x);
	s = sin(Xbih.lat);
	Xbih.alt = p / cos(Xbih.lat) - AE / sqrt(1 - EE2 * s * s);
}

void eceftobih(Mat & Xecef, Mat & Xbih)
{
	double * ptr0 = Xecef.ptr<double>(0);
	double * ptr1 = Xbih.ptr<double>(0);
	double p = sqrt(ptr0[0]*ptr0[0] + ptr0[1]*ptr0[1]);
	double th = atan(ptr0[2]*AE / (p*BE));
	double s = sin(th);
	double c = cos(th);
	s = s*s*s;
	c = c*c*c;
	ptr1[0] = atan(ptr0[2]+EE2_B * s) / (p - EE2_A * c);
	ptr1[1] = atan(ptr0[1]/ptr0[0]);

	s = sin(ptr1[0]);
	ptr1[2] = p / cos(ptr1[0]) - AE / sqrt(1 - EE2 * s * s);
}

void bihtowrld(Point3d & Xorg, Mat & Xrot, s_bihpos & Xbih, Point3d & Xwrld)
{
	Point3d Xecef;

	double slat = sin(Xbih.lat);
	double clat = cos(Xbih.lat);
	double slon = sin(Xbih.lon);
	double clon = cos(Xbih.lon);
	double N = AE / sqrt(1 - EE2 * slat * slat);
	double tmp = (N + Xbih.alt) * clat;
	Xecef.x = tmp * clon;
	Xecef.y = tmp * slon;
	Xecef.z = (N * (1 - EE2) + Xbih.alt) * slat;

	Xecef -= Xorg;

	double * ptr0;
	ptr0 = Xrot.ptr<double>(0);
	Xwrld.x = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
	ptr0 = Xrot.ptr<double>(1);
	Xwrld.y = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
	ptr0 = Xrot.ptr<double>(2);
	Xwrld.z = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
}

void wrldtobih(Mat & Xorg, Mat & Xrot, Mat & Xwrld,  Mat & Xbih)
{
	Mat Xecef = (Xrot.t() * Xwrld);
	Xecef += Xorg;
	eceftobih(Xecef, Xbih);
}

void wrldtobih(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, s_bihpos & Xbih)
{
	Point3d Xecef;
	wrldtoecef(Xorg, Xrot, Xwrld, Xecef);
	eceftobih(Xecef, Xbih);
}

void wrldtoecef(const Mat & Rrot, 
				const float xorg, const float yorg, const float zorg, 
				const float xwrld, const float ywrld, const float zwrld,
				float & xecef, float & yecef, float & zecef
				)
{
	const double * ptr = Rrot.ptr<double>();

	float x = (float)(xwrld - xorg);	
	float y = (float)(ywrld - yorg);
	float z = (float)(zwrld - zorg);

	xecef = (float)(ptr[0] * x + ptr[3] * y + ptr[6] * z);
	yecef = (float)(ptr[1] * x + ptr[4] * y + ptr[7] * z);
	zecef = (float)(ptr[2] * x + ptr[5] * y + ptr[8] * z);
}

void eceftowrld(const Mat & Rrot, 
				const float xorg, const float yorg, const float zorg, 
				const float xecef, const float yecef, const float zecef,
				float & xwrld, float & ywrld, float & zwrld
				)
{
	const double * ptr = Rrot.ptr<double>();
	xwrld = (float)(ptr[0] * xecef + ptr[1] * yecef + ptr[2] * zecef + xorg);
	ywrld = (float)(ptr[3] * xecef + ptr[4] * yecef + ptr[5] * zecef + yorg);
	zwrld = (float)(ptr[6] * xecef + ptr[7] * yecef + ptr[8] * zecef + zorg);
}


// wrldtoecef converts a world point Xwrld to a ECEF point Xecef
// Xorg is the origin in the ECEF coordinate and Xrot is the rotation matrix
// for ECEF to world which can be calculated by getwrldrot(). 
void wrldtoecef(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, Point3d & Xecef)
{
	//Xecef = (Xrot.t() * Xwrld);
	double * ptr = Xrot.ptr<double>(0);
	Xecef.x = ptr[0] * Xwrld.x;
	Xecef.y = ptr[1] * Xwrld.x;
	Xecef.z = ptr[2] * Xwrld.x;

	ptr = Xrot.ptr<double>(1);
	Xecef.x += ptr[0] * Xwrld.y;
	Xecef.y += ptr[1] * Xwrld.y;
	Xecef.z += ptr[2] * Xwrld.y;

	ptr = Xrot.ptr<double>(2);
	Xecef.x += ptr[0] * Xwrld.z;
	Xecef.y += ptr[1] * Xwrld.z;
	Xecef.z += ptr[2] * Xwrld.z;

	Xecef += Xorg;
}

// eceftowrld converts a ECEF point Xecef to a world point Xwrld.
// Xorg is the origin in the ECEF coordinate and Xrot is the rotation matrix
// for ECEF t world conversion which can be calculated by getwrldrot().
void eceftowrld(Point3d & Xorg, Mat & Xrot, Point3d & Xecef, Point3d & Xwrld)
{
	Xecef -= Xorg;
	double * ptr0;
	ptr0 = Xrot.ptr<double>(0);
	Xwrld.x = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
	ptr0 = Xrot.ptr<double>(1);
	Xwrld.y = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
	ptr0 = Xrot.ptr<double>(2);
	Xwrld.z = ptr0[0] * Xecef.x
		+ ptr0[1] * Xecef.y + ptr0[2] * Xecef.z;
}

void getwrldrotf(const float lat, const float lon, Mat & Rwrld)
{
	float c, s;

	// pi/2
	c = 0;
	s = 1;

	Rwrld = Mat::eye(3, 3, CV_32FC1);
	Rwrld.at<float>(0, 0) = c;
	Rwrld.at<float>(1, 1) = c;
	Rwrld.at<float>(0, 1) = s;
	Rwrld.at<float>(1, 0) = -s;

	// pi/2 - lat
	c = sin(lat);
	s = cos(lat);
	Mat tmp = Mat::eye(3, 3, CV_32FC1);
	tmp.at<float>(0, 0) = c;
	tmp.at<float>(2, 2) = c;
	tmp.at<float>(0, 2) = -s;
	tmp.at<float>(2, 0) = s;

	Rwrld *= tmp;

	// lon
	c = cos(lon);
	s = sin(lon);
	tmp = Mat::eye(3, 3, CV_32FC1);
	tmp.at<float>(0, 0) = c;
	tmp.at<float>(1, 1) = c;
	tmp.at<float>(0, 1) = s;
	tmp.at<float>(1, 0) = -s;

	Rwrld *= tmp;
}

void getwrldrot(const float lat, const float lon, Mat & Rwrld)
{
	double c, s;

	// pi/2
	c = 0;
	s = 1;

	Rwrld = Mat::eye(3, 3, CV_64FC1);
	Rwrld.at<double>(0, 0) = c;
	Rwrld.at<double>(1, 1) = c;
	Rwrld.at<double>(0, 1) = s;
	Rwrld.at<double>(1, 0) = -s;

	// pi/2 - lat
	c = sin(lat);
	s = cos(lat);
	Mat tmp = Mat::eye(3, 3, CV_64FC1);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(2, 2) = c;
	tmp.at<double>(0, 2) = -s;
	tmp.at<double>(2, 0) = s;

	Rwrld *= tmp;

	// lon
	c = cos(lon);
	s = sin(lon);
	tmp = Mat::eye(3, 3, CV_64FC1);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(1, 1) = c;
	tmp.at<double>(0, 1) = s;
	tmp.at<double>(1, 0) = -s;

	Rwrld *= tmp;
}

// getwrldrot calculates world rotation matrix from bih 
// resulting matrix is used in wrldtoecef() or eceftowrld()
void getwrldrot(const s_bihpos & Xbih, Mat & Rwrld)
{
	double c, s;

	// pi/2
	c = 0;
	s = 1;

	Rwrld = Mat::eye(3, 3, CV_64FC1);
	Rwrld.at<double>(0, 0) = c;
	Rwrld.at<double>(1, 1) = c;
	Rwrld.at<double>(0, 1) = s;
	Rwrld.at<double>(1, 0) = -s;

	// pi/2 - lat
	c = sin(Xbih.lat);
	s = cos(Xbih.lat);
	Mat tmp = Mat::eye(3, 3, CV_64FC1);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(2, 2) = c;
	tmp.at<double>(0, 2) = -s;
	tmp.at<double>(2, 0) = s;

	Rwrld *= tmp;

	// lon
	c = cos(Xbih.lon);
	s = sin(Xbih.lon);
	tmp = Mat::eye(3, 3, CV_64FC1);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(1, 1) = c;
	tmp.at<double>(0, 1) = s;
	tmp.at<double>(1, 0) = -s;

	Rwrld *= tmp;
}

void getmatrotRPY(s_rotpar par, Mat & R)
{
	double c, s;

	// rolling (z)
	R = Mat::eye(3, 3, CV_64FC1);
	c = cos(par.roll);
	s = sin(par.roll);
	R.at<double>(0, 0) = c;
	R.at<double>(1, 1) = c;
	R.at<double>(0, 1) = -s;
	R.at<double>(1, 0) = s;

	Mat tmp;
	// pitching (x)
	c = cos(par.pitch);
	s = sin(par.pitch);
	tmp = Mat::eye(3, 3, CV_64FC1);

	tmp.at<double>(1, 1) = c;
	tmp.at<double>(2, 2) = c;
	tmp.at<double>(1, 2) = -s;
	tmp.at<double>(2, 1) = s;
	R *= tmp;

	// yawing (y)
	tmp = Mat::eye(3, 3, CV_64FC1);
	c = cos(par.yaw);
	s = sin(par.yaw);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(2, 2) = c;
	tmp.at<double>(0, 2) = -s;
	tmp.at<double>(2, 0) = s;
	R *= tmp;
}

void getmatrotYPR(s_rotpar par, Mat & R)
{
	double c, s;

	// yawing (y)
	R = Mat::eye(3, 3, CV_64FC1);
	c = cos(par.yaw);
	s = sin(par.yaw);
	R.at<double>(0, 0) = c;
	R.at<double>(2, 2) = c;
	R.at<double>(0, 2) = -s;
	R.at<double>(2, 0) = s;

	Mat tmp;
	// pitching (x)
	c = cos(par.pitch);
	s = sin(par.pitch);
	tmp = Mat::eye(3, 3, CV_64FC1);

	tmp.at<double>(1, 1) = c;
	tmp.at<double>(2, 2) = c;
	tmp.at<double>(1, 2) = -s;
	tmp.at<double>(2, 1) = s;
	R *= tmp;

	// rolling (z)
	tmp = Mat::eye(3, 3, CV_64FC1);
	c = cos(par.roll);
	s = sin(par.roll);
	tmp.at<double>(0, 0) = c;
	tmp.at<double>(1, 1) = c;
	tmp.at<double>(0, 1) = -s;
	tmp.at<double>(1, 0) = s;
	R *= tmp;
}


void swpYZ(Mat & R)
{
	double * ptr0, *ptr1;
	double tmp;
	ptr0 = R.ptr<double>(1);
	ptr1 = R.ptr<double>(2);

	tmp = *ptr0;
	*ptr0 = *ptr1;
	*ptr1 = tmp;
	ptr0++; ptr1++;
	tmp = *ptr0;
	*ptr0 = *ptr1;
	*ptr1 = tmp;
	ptr0++; ptr1++;
	tmp = *ptr0;
	*ptr0 = *ptr1;
	*ptr1 = tmp;
}


void trans(Mat & Mtrn, Point3d & Xwrld, Point2d & Xview)
{
	double * ptr;

	ptr = Mtrn.ptr<double>(0);
	Xview.x = ptr[0] * Xwrld.x + ptr[1] * Xwrld.y + ptr[2] *Xwrld.z + ptr[3];
	ptr = Mtrn.ptr<double>(1);
	Xview.y = ptr[0] * Xwrld.x + ptr[1] * Xwrld.y + ptr[2] *Xwrld.z + ptr[3];
	ptr = Mtrn.ptr<double>(2);
	double s = ptr[0] * Xwrld.x + ptr[1] * Xwrld.y + ptr[2] *Xwrld.z + ptr[3];
	s = 1/s;
	Xview.x *= s;
	Xview.y *= s;
}

