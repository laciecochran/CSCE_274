serial-dump: serial.o dump.o
	gcc -o serial-dump serial.o dump.o
serial.o: serial.c serial.h
	gcc -c $(CFLAGS) serial.c
dump.o: dump.c serial.h
	gcc -c $(CFLAGS) dump.c
