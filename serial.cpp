

#define _CRT_SECURE_NO_WARNINGS

//#include <Windows.h>
#include "serial.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cstring>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>

//static HANDLE hCom0;
//static HANDLE hCom1;
//static HANDLE hCom2;
//static HANDLE hCom3;

static int hCom0;
static int hCom1;
static int hCom2;
static int hCom3;

#define _CRT_SECURE_NO_WARNINGS 1

void c2wc( const char* pstrSrc, wchar_t pwstrDest[])
{
  int nLen = (int)strlen(pstrSrc) + 1;
  mbstowcs( pwstrDest, pstrSrc, nLen );
}

int serialOpen(int num, char port[], int baud){

	//DCB dcb;
	struct termios properties;
	//HANDLE hCom;
	int hCom;
	unsigned int dwError;
	bool fSucess;

	//wchar_t port_wchar[10];
	//c2wc(port, port_wchar);

	//hCom = CreateFile(port_wchar,
	//			GENERIC_READ | GENERIC_WRITE,
	//			0, // comm devices must be opened w/exclusive access
	//			NULL, // no security attrs 
	//			OPEN_EXISTING, 
	//			0, // not overlapped I/O
	//			NULL
	//			);

	//mode_t mode = O_RDWR; 
	hCom = open(port, O_RDWR);

	// if(hCom == INVALID_HANDLE_VALUE){
	// 			dwError = GetLastError();                       
	// 			return (-1);
	// }

    if(hCom < 0)
	{
		return -1;
	}
// Omit the call to SetupComm to use the default queue sizes.
// Get the current configuration.

	// fSucess = GetCommState(hCom, &properties);

	// if(!fSucess) return(-2);
	
// Fill in the DCB: baud=whatever, 8 data bits, no parity, 1 stop bit.  

	// dcb.BaudRate = baud; 
	// dcb.fBinary = TRUE;
	// dcb.fParity = FALSE; 
	// dcb.ByteSize = 8;
	// dcb.Parity = NOPARITY;
	// dcb.StopBits = ONESTOPBIT; 
	//fSucess = SetCommState(hCom, &dcb); 

	tcgetattr(hCom, &properties);
	properties.c_cflag = CS8 | B9600 | ~CSTOPB | CREAD | CLOCAL | ~PARENB;
	properties.c_iflag = IGNPAR; 
	properties.c_oflag = 0;
	properties.c_lflag = 0;
	properties.c_cc[VMIN] = 1;
	properties.c_cc[VTIME] = 0;

	tcsetattr(hCom, TCSANOW, &properties);
 
	if (!fSucess) return (-3);

	switch (num)
	{
	case 0:
		hCom0 = hCom;
		break;
	case 1:
		hCom1 = hCom;
		break;
	case 2:
		hCom2 = hCom;
		break;
	case 3:
		hCom3 = hCom;
		break;
	default:
		break;
	}
	return (0);
} 

int serialClose(int num)
{
	switch (num)
	{
	case 0:
		close(hCom0);
		break;
	case 1:
		close(hCom1);
		break;
	case 2:	
		close(hCom2);
		break;
	case 3:
		close(hCom3);
		break;
	default:
		break;
	}
	return 0;
}

int setreadtimeout(int num, int initialTimeout)
{ 
	COMMTIMEOUTS timeouts; 

	timeouts.ReadIntervalTimeout = 0;         
	timeouts.ReadTotalTimeoutMultiplier = initialTimeout;         
	timeouts.ReadTotalTimeoutConstant = 0; 

	switch (num)
	{
	case 0:
		if (!SetCommTimeouts(hCom0, &timeouts)) return -1;
		break;
	case 1:
		if (!SetCommTimeouts(hCom1, &timeouts)) return -1;
		break;
	case 2:
		if (!SetCommTimeouts(hCom2, &timeouts)) return -1;
		break;
	case 3:
		if (!SetCommTimeouts(hCom3, &timeouts)) return -1;
		break;
	default:
		break;
	}

	return 0;
} 



// read byte from serial port.
// returns 0x00xx on success
// returns 0x01xx on failure
unsigned short serialReadByte(int num)
{
	unsigned char tmp;
	unsigned long numread=0; 


	switch (num)
	{
	case 0:
		ReadFile(hCom0, (void*)&tmp, 1, &numread, NULL);
		break;
	case 1:
		ReadFile(hCom1, (void*)&tmp, 1, &numread, NULL);
		break;
	case 2:
		ReadFile(hCom2, (void*)&tmp, 1, &numread, NULL);
		break;
	case 3:
		ReadFile(hCom3, (void*)&tmp, 1, &numread, NULL);
		break;
	default:
		break;
	}

    if (numread != 1){
		//printf("!"); 
		return 0x100;
	}

	return tmp;
}

int serialReadBlock(int num, unsigned char * dst, int size)
{
	unsigned long count;

	switch (num)
	{
	case 0:
		ReadFile(hCom0, (void*)dst, size, &count, NULL);
		break;
	case 1:
		ReadFile(hCom1, (void*)dst, size, &count, NULL);
		break;
	case 2:
		ReadFile(hCom2, (void*)dst, size, &count, NULL);
		break;
	case 3:
		ReadFile(hCom3, (void*)dst, size, &count, NULL);
		break;
	default:
		count = 0;
		break;
	}

	return count;
}

