

// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

#define _CRT_SECURE_NO_WARNINGS


#ifndef MAIN_H
#define MAIN_H

#define HAVE_STRUCT_TIMESPEC
#include "pthreads/pthread.h"
#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Timer function
unsigned long long int tic(void);
unsigned long long int toc(void);
double timeHost(void);

void* tSerial0(void* thread_argument);
void *tSerial1(void *thread_argument);
void* tSerial2(void* thread_argument);
void* tSerial3(void* thread_argument);
void *tPeakView(void *thread_argument);
struct multiplearg
{
	void* argument;
	pthread_t sensor0;
	pthread_t curr;
};
#endif



