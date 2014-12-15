// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// coord.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// coord.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with coord.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef COORD_H
#define COORD_H

#define PI 3.1415926535898			
#define AE 6378137.					// radius at the equator  
#define FE 1/298.257223563			// earth flatness
#define BE (AE*(1.-FE))
#define EE2 (2.*FE-FE*FE) 
#define EE2_A ((AE*AE-BE*BE)/(AE))
#define EE2_B ((AE*AE-BE*BE)/(BE))
#define RE 6371229.3				// radius of the earth
#define TERREF 0.0784				// terrestrial refraction coefficient
#define TERREF2 2.07
#define KNOT 0.514444444			/* in meter/sec*/
#define MILE 1852					/* in meter */

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

void bihtoecef(const s_bihpos & Xbih, Point3d & Xecef);
void eceftobih(Mat & Xecef, Mat & Xbih);
void bihtowrld(Point3d & Xorg, Mat & Xrot, s_bihpos & Xbih, Point3d & Xwrld);
void wrldtobih(Mat & Xorg, Mat & Xrot, Mat & Xwrld, Mat & Xbih);
void wrldtobih(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, s_bihpos & Xbih);
void wrldtoecef(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, Point3d & Xecef);
void eceftowrld(Point3d & Xorg, Mat & Xrot, Point3d & Xwrld, Point3d & Xecef);
void getwrldrot(const s_bihpos & Xbih, Mat & Rwrld);
void getmatrotRPY(s_rotpar par, Mat & R);
void getmatrotYPR(s_rotpar par, Mat & R);
void swpYZ(Mat & R);
void trans(Mat & Mtrn, Point3d & Xwrld, Point2d & Xview);

#endif