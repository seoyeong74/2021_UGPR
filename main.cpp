// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

// #define _CRT_SECURE_NO_WARNINGS

//#include <winsock.h>
//#include <windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <queue>

#pragma comment(lib, "Ws2_32.lib")
#include "main.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

// timer definition
unsigned long long int _timeOffset;
unsigned long long int tic(void);
unsigned long long int toc(void);

// host clock definition
unsigned long long int _clockOffset;
double timeHost(void);


int main(void)
{
	int th_id;
	int status;
	int argument=0;
	int ret;
	
	sched_param param;

	pthread_t sensor0;
	//pthread_t sensor1;
	//pthread_t sensor2;
	//pthread_t sensor3;
	pthread_t peakView;

	// timer initialization
	_timeOffset = tic();
	_clockOffset = _timeOffset;

	// Thread initiation
	pthread_attr_t tattrCritical;
	ret = pthread_attr_init(&tattrCritical);
	ret = pthread_attr_getschedparam(&tattrCritical, &param);
	param.sched_priority = THREAD_PRIORITY_TIME_CRITICAL;
	ret = pthread_attr_setschedparam(&tattrCritical, &param);
	th_id = pthread_create(&sensor0, &tattrCritical, tSerial0, (void*)argument);
	//th_id = pthread_create(&sensor1, &tattrCritical, tSerial1, (void*)argument);
	//th_id = pthread_create(&sensor2, &tattrCritical, tSerial2, (void*)argument);
	//th_id = pthread_create(&sensor3, &tattrCritical, tSerial3, (void*)argument);
	if(th_id < 0) perror("Thread create error : ");
	
	//multiplearg* viewarg;
	//viewarg = (multiplearg*)malloc(sizeof(multiplearg));
	//viewarg->argument = (void*)argument;
	//viewarg->curr = pthread_self();
	//viewarg->sensor0 = sensor0;
	//if (pthread_equal(pthread_self(), sensor0))
	th_id = pthread_create(&peakView, NULL, tPeakView, (void*)argument);
	
	// Thread termination
	pthread_join(sensor0, (void**)&status);
	//pthread_join(sensor1, (void**)&status);
	//pthread_join(sensor2, (void**)&status);
	//pthread_join(sensor3, (void**)&status);
	//if (pthread_equal(pthread_self(), sensor0))
	pthread_join(peakView, (void**)&status);

	return 0;
}

unsigned long long int tic(void)
{
    LARGE_INTEGER l0, l1;
    double freq;
    if (!QueryPerformanceFrequency(&l0)) { printf("Failed to get frequency\n"); return 0; }
    freq = (double)(l0.QuadPart) / 1e6; // microseconds
    QueryPerformanceCounter(&l1);
    return (unsigned long long int)(l1.QuadPart / freq);
}

unsigned long long int toc(void)
{
    uint64_t t1 = tic();
    uint64_t dt = t1 - _timeOffset;
    return dt;
}

double timeHost(void)
{
    uint64_t t1 = tic();
    uint64_t dt = t1 - _clockOffset;
    return ((double)dt)/1000000.;
}
