// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_coord.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_coord.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_coord.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef AWS_COORD_H
#define AWS_COORD_H

#include "aws_const.h"

// s_bihpos defines bih coordinate; latitude, longitude, altitude
struct s_bihpos{
	double lat, lon, alt;
	s_bihpos():lat(0), lon(0), alt(0){};
};

struct s_rotpar
{
	double roll, pitch, yaw;
	s_rotpar():roll(0), pitch(0), yaw(0){};
};

void bihtoecef(const float lat, const float lon, const float alt, float & x, float & y, float & z);
void eceftobih(const float x, const float y, const float z, float & lat, float & lon, float & alt);
void wrldtoecef(const Mat & Rrot, 
				const float xorg, const float yorg, const float zorg, 
				const float xwrld, const float ywrld, const float zwrld,
				float & xecef, float & yecef, float & zecef
				);
void eceftowrld(const Mat & Rrot, 
				const float xorg, const float yorg, const float zorg, 
				const float xecef, const float yecef, const float zecef,
				float & xwrld, float & ywrld, float & zwrld
				);
void getwrldrot(const float lat, const float lon, Mat & Rwrld);


void bihtoecef(const s_bihpos & Xbih, Point3d & Xecef);
void eceftobih(Mat & Xecef, Mat & Xbih);
void bihtowrld(Point3d & Xorg, Mat & Xrot, s_bihpos & Xbih, Point3d & Xwrld);
void wrldtobih(Mat & Xorg, Mat & Xrot, Mat & Xwrld, Mat & Xbih);
void wrldtobih(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, s_bihpos & Xbih);
void wrldtoecef(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, Point3d & Xecef);
void eceftowrld(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, Point3d & Xecef);

void getwrldrotf(const float lat, const float lon, Mat & Rwrld);
void getwrldrot(const s_bihpos & Xbih, Mat & Rwrld);
void getmatrotRPY(s_rotpar par, Mat & R);
void getmatrotYPR(s_rotpar par, Mat & R);
void getmatrotRPY(float r, float p, float y, Mat & R);
void getmatrotYPR(float y, float p, float r, Mat & R);
void swpYZ(Mat & R);
void trans(Mat & Mtrn, Point3d & Xwrld, Point2d & Xview);

#endif