// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// aws_thread.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// aws_thread.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with aws_thread.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef _AWS_THREAD_H_
#define _AWS_THREAD_H_
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

class pthread_lock{
	pthread_mutex_t * pmtx;
public:
	pthread_lock(pthread_mutex_t * p):pmtx(p){
		pthread_mutex_lock(p);
	}

	~pthread_lock(){
		pthread_mutex_unlock(pmtx);
	}
};

#endif