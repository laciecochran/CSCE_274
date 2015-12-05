/*
** This software is may be freely distributed and modified for
** noncommercial purposes.  It is provided without warranty of
** any kind.  Copyright 2006, Jason O'Kane and the University
** of Illinois.
**
*/

// This file declares some basic functions for serial communications.

#ifndef INCLUDE_SERIAL_H
#define INCLUDE_SERIAL_H

#include <termios.h>
#include <sys/ioctl.h>

typedef struct {
	int fd;
	int verbose;
} Serial;

void serialOpen(Serial *s, char *device, int baudCode, int _verbose);
void serialClose(Serial *s);
void serialSetBaud(Serial *s, int baudCode);
int serialSend(Serial *s, unsigned char c);
int  serialNumBytesWaiting(Serial *s);
int serialGetChar(Serial *s, unsigned char *c);

int serialGetSignal(Serial *s, int sig);
void serialSetSignal(Serial *s, int sig);
void serialClearSignal(Serial *s, int sig);

#endif

