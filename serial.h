

#define _CRT_SECURE_NO_WARNINGS

#ifndef SERIAL_H
#define SERIAL_H

int serialOpen(int num, char port[], int baud);
int serialClose(int num);

int setreadtimeout(int num, int initialtimeout);

unsigned short serialReadByte(int num);
int   serialReadBlock(int num, unsigned char * dst, int size);

#endif