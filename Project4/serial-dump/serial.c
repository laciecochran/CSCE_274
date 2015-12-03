/*
** This software is may be freely distributed and modified for
** noncommercial purposes.  It is provided without warranty of
** any kind.  Copyright 2006, Jason O'Kane and the University
** of Illinois.
**
*/

// This file declares some basic functions for serial communications.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <assert.h>
#include <ctype.h>

#include "serial.h"

void serialOpen(Serial *s, char *device, int baudCode, int _verbose) {
	struct termios options;
	int r;

	s->verbose = _verbose;

	// Open the serial port.
	if(s->verbose) printf("Serial: opening serial device %s\n", device);
	//s->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	s->fd = open(device, O_RDWR | O_NOCTTY);
	if(s->fd == -1) {
		int errsv = errno;
		fprintf(stderr, "Serial: ERROR: Could not open serial port %s\n", device);
		fprintf(stderr, "Serial: ERROR: %s\n", strerror(errsv));
		exit(1);
	}
	
	// Non-blocking reads.
	r = fcntl(s->fd, F_SETFL, FNDELAY);
	assert(r != -1);
	
	// Baud rate.
	serialSetBaud(s, baudCode);	
	
	// Other options:
	r = tcgetattr(s->fd, &options);
	assert(r != -1);
	
	// - 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// Don't "own" the port.
	options.c_cflag |= CLOCAL;

	// - Allow reads.
	options.c_cflag |= CREAD;

	// - Raw input.
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// - Disable parity checking.
	options.c_iflag &= ~(INPCK | ISTRIP);

	// - No hardware flow control.
	options.c_cflag &= ~CRTSCTS;

	// - No software flow control.
	options.c_iflag &= ~(IXON | IXOFF | IXANY);	

	// - Raw output.
	options.c_oflag &= ~OPOST;
	
	// Actually set the options.
	r = tcsetattr(s->fd,TCSANOW, &options);
	assert(r != -1);

}

void serialClose(Serial *s){
	close(s->fd);
}

void serialSetBaud(Serial *s, int baudCode) {
	int r;

	if(s->verbose) printf("Serial: set baud to code %d\n", baudCode);
	
	// Get the current port settings.
	struct termios options;
	r = tcgetattr(s->fd, &options);
	assert(r != -1);

	// Change the baud rate.
	cfsetispeed(&options, baudCode);
	cfsetospeed(&options, baudCode);

	// Apply the changes.
	r = tcsetattr(s->fd, TCSANOW, &options);
	assert(r != -1);
}

int serialSend(Serial *s, unsigned char c) {

	// Send one character to the serial port.
	if(s->verbose) printf("Serial: send character (%d)\n", (int)((unsigned char)c));

	while(1) {
		int n = write(s->fd, &c, 1);
		int errsv = errno;
		
		if(n == 1) {
			return 1;
		} else if(errno != EAGAIN) {
			fprintf(stderr, "Serial: ERROR: Could not write character: (%d) %c \n", (int)(c), isprint(c) ? c : ' ');
			fprintf(stderr, "Serial: ERROR: %s\n", strerror(errsv));
			return 0;
		} else {
			if(s->verbose) printf("Serial: Error EAGAIN on write... trying again.\n");
			usleep(100);
		}
	}
	return 1;
}

int serialNumBytesWaiting(Serial *s) {
	// Return the number of bytes in the input buffer.
	int bytes;
	ioctl(s->fd, FIONREAD, &bytes);
	return bytes;	
}

int serialGetChar(Serial *s, unsigned char *buf) {
	// Tries to get one character.  Returns true and fills in its parameter if successful.
	// Returns false if there are no characters to be read.
	static int successfulLastTime = 1;	
	
	if(s->verbose && successfulLastTime) {
		printf("Serial: get character (%d) bytes in buffer.\n", serialNumBytesWaiting(s));
	}
	
	// Try to read.
	errno = 0;
	int r = read(s->fd, buf, 1);
	int errsv = errno;

	// Got a character?
	if(r == 1) {
		if(s->verbose) printf("Serial: got character (%d)\n", (int) (unsigned char) *buf);
		successfulLastTime = 1;
		return 1;
	}

	// Didn't get a character.  Why not?
	if(r < 0 && errsv == EAGAIN) {
		// No characters at the moment.  Try again later.
		successfulLastTime = 0;
		return 0;
	} else { 
		fprintf(stderr, "Serial: ERROR: Could not read character\n");
		fprintf(stderr, "Serial: ERROR: %s\n", strerror(errsv));
		successfulLastTime = 0;
		return 0;
	}
}

void serialSetSignal(Serial *s, int sig) {
	if(s->verbose) printf("Serial: set signal %d\n", sig);
	int status;
	ioctl(s->fd, TIOCMGET, &status);
	status |= sig;
	ioctl(s->fd, TIOCMSET, &status);
}

void serialClearSignal(Serial *s, int sig) {
	if(s->verbose) printf("Serial: clear signal %d\n", sig);
	int status;
	ioctl(s->fd, TIOCMGET, &status);
	status &= ~sig;
	ioctl(s->fd, TIOCMSET, &status);
}

int serialGetSignal(Serial *s, int sig) {
	if(s->verbose) printf("Serial: get signal %d\n", sig);
	int status;
	ioctl(s->fd, TIOCMGET, &status);
	return (int)(status & sig);
}

