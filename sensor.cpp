
// Hybo-KT LIDAR Development Project
// Viewer/Logger program
// 15 March 2021
// OpenCV 2.4.0 / pthread
// Son, Youngbin
// tech@hybo.co

#define _CRT_SECURE_NO_WARNINGS

//#include <windows.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "main.h"
#include "serial.h"

// Shared variables, not synchronized
int imgReady[4] = { 0, 0, 0, 0 };
unsigned char sat[2][320][160];
short img[4][320][160][2];

// Row count for each image type
// Currently 80 for depth, 160 for grayscale
const int cnt_row[2] = { 40, 160 };

/* Added for CRC */
static const uint16_t CRC16Table[256] = {
	0x0000,	0x1021,	0x2042,	0x3063,	0x4084,	0x50a5,	0x60c6,	0x70e7,
	0x8108,	0x9129,	0xa14a,	0xb16b,	0xc18c,	0xd1ad,	0xe1ce,	0xf1ef,
	0x1231,	0x0210,	0x3273,	0x2252,	0x52b5,	0x4294,	0x72f7,	0x62d6,
	0x9339,	0x8318,	0xb37b,	0xa35a,	0xd3bd,	0xc39c,	0xf3ff,	0xe3de,
	0x2462,	0x3443,	0x0420,	0x1401,	0x64e6,	0x74c7,	0x44a4,	0x5485,
	0xa56a,	0xb54b,	0x8528,	0x9509,	0xe5ee,	0xf5cf,	0xc5ac,	0xd58d,
	0x3653,	0x2672,	0x1611,	0x0630,	0x76d7,	0x66f6,	0x5695,	0x46b4,
	0xb75b,	0xa77a,	0x9719,	0x8738,	0xf7df,	0xe7fe,	0xd79d,	0xc7bc,
	0x48c4,	0x58e5,	0x6886,	0x78a7,	0x0840,	0x1861,	0x2802,	0x3823,
	0xc9cc,	0xd9ed,	0xe98e,	0xf9af,	0x8948,	0x9969,	0xa90a,	0xb92b,
	0x5af5,	0x4ad4,	0x7ab7,	0x6a96,	0x1a71,	0x0a50,	0x3a33,	0x2a12,
	0xdbfd,	0xcbdc,	0xfbbf,	0xeb9e,	0x9b79,	0x8b58,	0xbb3b,	0xab1a,
	0x6ca6,	0x7c87,	0x4ce4,	0x5cc5,	0x2c22,	0x3c03,	0x0c60,	0x1c41,
	0xedae,	0xfd8f,	0xcdec,	0xddcd,	0xad2a,	0xbd0b,	0x8d68,	0x9d49,
	0x7e97,	0x6eb6,	0x5ed5,	0x4ef4,	0x3e13,	0x2e32,	0x1e51,	0x0e70,
	0xff9f,	0xefbe,	0xdfdd,	0xcffc,	0xbf1b,	0xaf3a,	0x9f59,	0x8f78,
	0x9188,	0x81a9,	0xb1ca,	0xa1eb,	0xd10c,	0xc12d,	0xf14e,	0xe16f,
	0x1080,	0x00a1,	0x30c2,	0x20e3,	0x5004,	0x4025,	0x7046,	0x6067,
	0x83b9,	0x9398,	0xa3fb,	0xb3da,	0xc33d,	0xd31c,	0xe37f,	0xf35e,
	0x02b1,	0x1290,	0x22f3,	0x32d2,	0x4235,	0x5214,	0x6277,	0x7256,
	0xb5ea,	0xa5cb,	0x95a8,	0x8589,	0xf56e,	0xe54f,	0xd52c,	0xc50d,
	0x34e2,	0x24c3,	0x14a0,	0x0481,	0x7466,	0x6447,	0x5424,	0x4405,
	0xa7db,	0xb7fa,	0x8799,	0x97b8,	0xe75f,	0xf77e,	0xc71d,	0xd73c,
	0x26d3,	0x36f2,	0x0691,	0x16b0,	0x6657,	0x7676,	0x4615,	0x5634,
	0xd94c,	0xc96d,	0xf90e,	0xe92f,	0x99c8,	0x89e9,	0xb98a,	0xa9ab,
	0x5844,	0x4865,	0x7806,	0x6827,	0x18c0,	0x08e1,	0x3882,	0x28a3,
	0xcb7d,	0xdb5c,	0xeb3f,	0xfb1e,	0x8bf9,	0x9bd8,	0xabbb,	0xbb9a,
	0x4a75,	0x5a54,	0x6a37,	0x7a16,	0x0af1,	0x1ad0,	0x2ab3,	0x3a92,
	0xfd2e,	0xed0f,	0xdd6c,	0xcd4d,	0xbdaa,	0xad8b,	0x9de8,	0x8dc9,
	0x7c26,	0x6c07,	0x5c64,	0x4c45,	0x3ca2,	0x2c83,	0x1ce0,	0x0cc1,
	0xef1f,	0xff3e,	0xcf5d,	0xdf7c,	0xaf9b,	0xbfba,	0x8fd9,	0x9ff8,
	0x6e17,	0x7e36,	0x4e55,	0x5e74,	0x2e93,	0x3eb2,	0x0ed1,	0x1ef0
};
static uint16_t getCRC16(const uint8_t* packet, int length) {
	// use CRC-16 CCITT standard
	register uint16_t crc = 0;
	register int i;
	for (i = 0; i < length; i++) {
		crc = (crc << 8) ^ (CRC16Table[((crc >> 8) ^ packet[i]) & 0xFF]);
	}
	return crc;
}
/* Added for CRC */

int logFlag = 0;

// tSerial
// Get data from the sensor over serial port
// Should be called as a real-time thread (high priority)
void* tSerial0(void* thread_argument)
{
	int camNum = 0;
	unsigned char buffer[5000];			// Row buffer
	unsigned char serialArray[2048];	// Scratchpad buffer
	unsigned short c;
	int ptr = 0;
	int i, j;
	int bufPtr;

	short buf;

	char comnumber[100] = "COM4";
	FILE* output = fopen("log1.bin", "wb");

	serialClose(camNum);

	if (serialOpen(camNum, (char*)comnumber, 12000000 )) {
		printf("Serial Open Fail!\n");
		return 0;
	}
	printf("%s : Sensor Serial Port Opened\n", comnumber);

	while (1) {

		// Get some data to scratchpad
		int maxCnt = serialReadBlock(camNum, serialArray, 1000);

		// Iterate over the received data
		for (bufPtr = 0; bufPtr < maxCnt; bufPtr++) {

			// Get a byte
			c = serialArray[bufPtr];

			// Fill the buffer with the byte
			buffer[ptr] = (char)(c & 0xff);

			// Synchronization: 'buffer' array should look like: {0x5A 0xA5 0x5A 0xA5 ... }
			// Discard data if such pattern is not made at the first part of the buffer.
			// Current packet looks like...
			// Preamble			Row#		ImgType			Pixel0		Pixel1		...		Pixel319
			// 4B:0x5AA55AA5	1B:0~159	1B:0 or 1		2B			2B			...		2B
			if (ptr > 9000
				|| (ptr == 0 && ((buffer[0] != 0x5A)))
				|| (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
				|| (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
				|| (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
				) {
				// Reset the pointer
				ptr = -1;
			}

			// If the buffer is properly filled...
			if (ptr == 320 * 2 + 9) {
				// Process the data

				// Check the packet is arrived with correct CRC16
				if (*(uint16_t*)&buffer[648] == getCRC16(&buffer[4], 644))
				{
					// Print current row # to see what is going on
					//printf("%d ", buffer[4]);

					int row = buffer[4];		// Row #
					int imgType = buffer[5];	// Type of the image - 0 = Depth, 1 = Grayscale
					int frame = buffer[6];
					int hdr = buffer[7];
					int pixel = 0;

					// If image type is valid...
					if (hdr == 0)
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							img[camNum][i][row][0] = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
						}
					}
					else
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							pixel = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
							if (pixel != 0) {
								img[camNum][i][row][1] = pixel;
							}
							else {
								img[camNum][i][row][1] = img[camNum][i][row][0];
							}
						}
					}

					// If this row is the final row...
					if (buffer[4] == cnt_row[0] - 1 && hdr == 1)
					{
						// Print a message to see what is going on
						printf("CAM:%d - FRAME: %d\n", camNum, frame);

						// Signal to the viewer thread / weak synchronization
						imgReady[camNum] = 1;
					}

					if (logFlag == 1) {
						fwrite(buffer, 1, 650, output);
						fflush(output);
					}
				}

				// Reset the pointer
				ptr = -1;
			}

			// Get to next place
			ptr++;
		}
	}

	serialClose(camNum);

	return 0;
}

// tSerial
// Get data from the sensor over serial port
// Should be called as a real-time thread (high priority)
void* tSerial1(void* thread_argument)
{
	int camNum = 1;
	unsigned char buffer[5000];			// Row buffer
	unsigned char serialArray[2048];	// Scratchpad buffer
	unsigned short c;
	int ptr = 0;
	int i, j;
	int bufPtr;

	short buf;

	char comnumber[100] = "COM7";
	FILE* output = fopen("log2.bin", "wb");

	serialClose(camNum);

	if (serialOpen(camNum, (char*)comnumber, 12000000)) {
		printf("Serial Open Fail!\n");
		return 0;
	}
	printf("%s : Sensor Serial Port Opened\n", comnumber);

	while (1) {

		// Get some data to scratchpad
		int maxCnt = serialReadBlock(camNum, serialArray, 1000);

		// Iterate over the received data
		for (bufPtr = 0; bufPtr < maxCnt; bufPtr++) {

			// Get a byte
			c = serialArray[bufPtr];

			// Fill the buffer with the byte
			buffer[ptr] = (char)(c & 0xff);

			// Synchronization: 'buffer' array should look like: {0x5A 0xA5 0x5A 0xA5 ... }
			// Discard data if such pattern is not made at the first part of the buffer.
			// Current packet looks like...
			// Preamble			Row#		ImgType			Pixel0		Pixel1		...		Pixel319
			// 4B:0x5AA55AA5	1B:0~159	1B:0 or 1		2B			2B			...		2B
			if (ptr > 9000
				|| (ptr == 0 && ((buffer[0] != 0x5A)))
				|| (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
				|| (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
				|| (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
				) {
				// Reset the pointer
				ptr = -1;
			}

			// If the buffer is properly filled...
			if (ptr == 320 * 2 + 9) {
				// Process the data

				// Check the packet is arrived with correct CRC16
				if (*(uint16_t*)&buffer[648] == getCRC16(&buffer[4], 644))
				{
					// Print current row # to see what is going on
					//printf("%d ", buffer[4]);

					int row = buffer[4];		// Row #
					int imgType = buffer[5];	// Type of the image - 0 = Depth, 1 = Grayscale
					int frame = buffer[6];
					int hdr = buffer[7];
					int pixel = 0;

					// If image type is valid...
					if (hdr == 0)
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							img[camNum][i][row][0] = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
						}
					}
					else
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							pixel = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
							if (pixel != 0) {
								img[camNum][i][row][1] = pixel;
							}
							else {
								img[camNum][i][row][1] = img[camNum][i][row][0];
							}
						}
					}

					// If this row is the final row...
					if (buffer[4] == cnt_row[0] - 1 && hdr == 1)
					{
						// Print a message to see what is going on
						printf("CAM:%d - FRAME: %d\n", camNum, frame);

						// Signal to the viewer thread / weak synchronization
						imgReady[camNum] = 1;
					}

					if (logFlag == 1) {
						fwrite(buffer, 1, 650, output);
						fflush(output);
					}
				}

				// Reset the pointer
				ptr = -1;
			}

			// Get to next place
			ptr++;
		}
	}

	serialClose(camNum);

	return 0;
}

// tSerial
// Get data from the sensor over serial port
// Should be called as a real-time thread (high priority)
void* tSerial2(void* thread_argument)
{
	int camNum = 2;
	unsigned char buffer[5000];			// Row buffer
	unsigned char serialArray[2048];	// Scratchpad buffer
	unsigned short c;
	int ptr = 0;
	int i, j;
	int bufPtr;

	short buf;

	char comnumber[100] = "COM8";
	FILE* output = fopen("log3.bin", "wb");

	serialClose(camNum);

	if (serialOpen(camNum, (char*)comnumber, 12000000)) {
		printf("Serial Open Fail!\n");
		return 0;
	}
	printf("%s : Sensor Serial Port Opened\n", comnumber);

	while (1) {

		// Get some data to scratchpad
		int maxCnt = serialReadBlock(camNum, serialArray, 1000);

		// Iterate over the received data
		for (bufPtr = 0; bufPtr < maxCnt; bufPtr++) {

			// Get a byte
			c = serialArray[bufPtr];

			// Fill the buffer with the byte
			buffer[ptr] = (char)(c & 0xff);

			// Synchronization: 'buffer' array should look like: {0x5A 0xA5 0x5A 0xA5 ... }
			// Discard data if such pattern is not made at the first part of the buffer.
			// Current packet looks like...
			// Preamble			Row#		ImgType			Pixel0		Pixel1		...		Pixel319
			// 4B:0x5AA55AA5	1B:0~159	1B:0 or 1		2B			2B			...		2B
			if (ptr > 9000
				|| (ptr == 0 && ((buffer[0] != 0x5A)))
				|| (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
				|| (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
				|| (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
				) {
				// Reset the pointer
				ptr = -1;
			}

			// If the buffer is properly filled...
			if (ptr == 320 * 2 + 9) {
				// Process the data

				// Check the packet is arrived with correct CRC16
				if (*(uint16_t*)&buffer[648] == getCRC16(&buffer[4], 644))
				{
					// Print current row # to see what is going on
					//printf("%d ", buffer[4]);

					int row = buffer[4];		// Row #
					int imgType = buffer[5];	// Type of the image - 0 = Depth, 1 = Grayscale
					int frame = buffer[6];
					int hdr = buffer[7];
					int pixel = 0;

					// If image type is valid...
					if (hdr == 0)
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							img[camNum][i][row][0] = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
						}
					}
					else
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							pixel = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
							if (pixel != 0) {
								img[camNum][i][row][1] = pixel;
							}
							else {
								img[camNum][i][row][1] = img[camNum][i][row][0];
							}
						}
					}

					// If this row is the final row...
					if (buffer[4] == cnt_row[0] - 1 && hdr == 1)
					{
						// Print a message to see what is going on
						printf("CAM:%d - FRAME: %d\n", camNum, frame);

						// Signal to the viewer thread / weak synchronization
						imgReady[camNum] = 1;
					}

					if (logFlag == 1) {
						fwrite(buffer, 1, 650, output);
						fflush(output);
					}
				}

				// Reset the pointer
				ptr = -1;
			}

			// Get to next place
			ptr++;
		}
	}

	serialClose(camNum);

	return 0;
}

// tSerial
// Get data from the sensor over serial port
// Should be called as a real-time thread (high priority)
void* tSerial3(void* thread_argument)
{
	int camNum = 3;
	unsigned char buffer[5000];			// Row buffer
	unsigned char serialArray[2048];	// Scratchpad buffer
	unsigned short c;
	int ptr = 0;
	int i, j;
	int bufPtr;

	short buf;

	char comnumber[100] = "COM9";
	FILE* output = fopen("log4.bin", "wb");

	serialClose(camNum);

	if (serialOpen(camNum, (char*)comnumber, 12000000)) {
		printf("Serial Open Fail!\n");
		return 0;
	}
	printf("%s : Sensor Serial Port Opened\n", comnumber);

	while (1) {

		// Get some data to scratchpad
		int maxCnt = serialReadBlock(camNum, serialArray, 1000);

		// Iterate over the received data
		for (bufPtr = 0; bufPtr < maxCnt; bufPtr++) {

			// Get a byte
			c = serialArray[bufPtr];

			// Fill the buffer with the byte
			buffer[ptr] = (char)(c & 0xff);

			// Synchronization: 'buffer' array should look like: {0x5A 0xA5 0x5A 0xA5 ... }
			// Discard data if such pattern is not made at the first part of the buffer.
			// Current packet looks like...
			// Preamble			Row#		ImgType			Pixel0		Pixel1		...		Pixel319
			// 4B:0x5AA55AA5	1B:0~159	1B:0 or 1		2B			2B			...		2B
			if (ptr > 9000
				|| (ptr == 0 && ((buffer[0] != 0x5A)))
				|| (ptr == 1 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5)))
				|| (ptr == 2 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A)))
				|| (ptr >= 3 && ((buffer[0] != 0x5A) || (buffer[1] != 0xA5) || (buffer[2] != 0x5A) || (buffer[3] != 0xA5)))
				) {
				// Reset the pointer
				ptr = -1;
			}

			// If the buffer is properly filled...
			if (ptr == 320 * 2 + 9) {
				// Process the data

				// Check the packet is arrived with correct CRC16
				if (*(uint16_t*)&buffer[648] == getCRC16(&buffer[4], 644))
				{
					// Print current row # to see what is going on
					//printf("%d ", buffer[4]);

					int row = buffer[4];		// Row #
					int imgType = buffer[5];	// Type of the image - 0 = Depth, 1 = Grayscale
					int frame = buffer[6];
					int hdr = buffer[7];
					int pixel = 0;

					// If image type is valid...
					if (hdr == 0)
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							img[camNum][i][row][0] = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
						}
					}
					else
					{
						// Copy this row to display buffer
						for (i = 0; i < 320; i++) {
							pixel = ((((int)buffer[8 + i * 2 + 1]) << 8) | ((int)(buffer[8 + i * 2 + 0]))) & 0xFFFF;
							if (pixel != 0) {
								img[camNum][i][row][1] = pixel;
							}
							else {
								img[camNum][i][row][1] = img[camNum][i][row][0];
							}
						}
					}

					// If this row is the final row...
					if (buffer[4] == cnt_row[0] - 1 && hdr == 1)
					{
						// Print a message to see what is going on
						printf("CAM:%d - FRAME: %d\n", camNum, frame);

						// Signal to the viewer thread / weak synchronization
						imgReady[camNum] = 1;
					}

					if (logFlag == 1) {
						fwrite(buffer, 1, 650, output);
						fflush(output);
					}
				}

				// Reset the pointer
				ptr = -1;
			}

			// Get to next place
			ptr++;
		}
	}

	serialClose(camNum);

	return 0;
}

// tPeakView
// Display and log the received image
void* tPeakView(void* thread_argument)
{
	int y;

	// Images with original sizes..
	//cv::Mat imgDepth0 = cv::Mat::zeros(cnt_row[0], 320, CV_32F);
	//cv::Mat imgDepth1 = cv::Mat::zeros(cnt_row[0], 320, CV_32F);
	//cv::Mat imgDepth2 = cv::Mat::zeros(cnt_row[0], 320, CV_32F);
	//cv::Mat imgDepth3 = cv::Mat::zeros(cnt_row[0], 320, CV_32F);

	// Images enlarged
	int multCount = 5;
	//cv::Mat imgDepthBig0 = cv::Mat::zeros(imgDepth0.rows * multCount, imgDepth0.cols * multCount, CV_32F);
	//cv::Mat imgDepthBig1 = cv::Mat::zeros(imgDepth1.rows * multCount, imgDepth1.cols * multCount, CV_32F);
	//cv::Mat imgDepthBig2 = cv::Mat::zeros(imgDepth2.rows * multCount, imgDepth2.cols * multCount, CV_32F);
	//cv::Mat imgDepthBig3 = cv::Mat::zeros(imgDepth3.rows * multCount, imgDepth3.cols * multCount, CV_32F);

	// Stitched 
	cv::Mat imgStitched = cv::Mat::zeros(cnt_row[0] * 4, 320, CV_32F);
	cv::Mat imgStitchedBig = cv::Mat::zeros(imgStitched.rows * multCount, imgStitched.cols * multCount, CV_32F);

	while (1) {
		int i, j;

		// Depth image..
		for (int camNum = 0; camNum < 4; camNum++) {																																																																																																																																																																																																						
			//if (imgReady[camNum] == 1) {
			//	imgReady[camNum] = 0;
			for (i = 0; i < 320; i++) {
				for (j = 0; j < cnt_row[0]; j++) {
					imgStitched.at<float>(j + cnt_row[0] * camNum, i) = ((float)(img[camNum][i][j][1])) / 3000.;
				}
			}
			//}
		}

		// Make big sized images
		cv::resize(imgStitched, imgStitchedBig, cv::Size(imgStitched.cols * multCount, imgStitched.rows * multCount), 0, 0, CV_INTER_NN);

		cv::imshow("print", imgStitchedBig);

		// Log the image if below key is pressed (focus should be at some opencv window)
		// The images are 16bit unsigned. Some viewer programs may not cope with this.
		// Do not delete waitKey function or this will not work
		char ch = cv::waitKey(1);
		if (ch == 'c') {
			if (logFlag == 0) {
				logFlag = 1;
				printf("====================START\n");
			}
			else {
				logFlag = 0;
				printf("====================FIN\n");
			}
		}
		else
			sleep(1);

	}

	return 0;
}

