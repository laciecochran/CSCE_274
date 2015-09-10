#ifndef INCLUDE_CMOD_H
#define INCLUDE_CMOD_H

  #include <avr/io.h>
  #include <avr/interrupt.h>

  // Setup the I/O pins.
  void setupIOPins(void);

  // Setup the serial port: Baud rate, transmit/recieve, packet size.
  void setupSerialPort(void);

  // Contains a collection of commands that allows me to "start" immediately
  // after calling this command.
  void initializeCommandModule(void);

  // Send and receive data from the Command Module
  void byteTx(uint8_t value);
  uint8_t byteRx(void);

  // Switch the baud rate on both Create and module  
  void baud(uint8_t baud_code);
#endif
