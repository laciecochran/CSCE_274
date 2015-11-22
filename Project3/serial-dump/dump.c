#include "serial.h"
#include <stdio.h>

int main() {
	Serial s;
	serialOpen(&s, "/dev/ttyUSB0", B57600, 0);
	unsigned char c;
	while(1) {
		if(serialGetChar(&s, &c)) {
			printf("%c", c);
			fflush(stdout);
		}
	}
}
