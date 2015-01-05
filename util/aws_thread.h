#ifndef _AWS_THREAD_H_
#define _AWS_THREAD_H_
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