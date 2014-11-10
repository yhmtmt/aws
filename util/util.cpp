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

#include "thread_util.h"

#include "util.h"


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


// R G
// G B
void cnvBayerRG8ToBGR8(Mat & src, Mat & dst)
{
	if(src.type() != CV_8UC1)
		return;

	dst = Mat(src.rows - 2, src.cols - 2, CV_8UC3);

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
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

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
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

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
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

	int step_size = (int) (src.step.p[0] / sizeof(unsigned short));
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
