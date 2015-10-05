// Copyright(c) 2015 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_const.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_const.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_const.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _AWS_CONST_H_
#define _AWS_CONST_H_

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

#endif