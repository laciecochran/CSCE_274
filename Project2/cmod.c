#include "cmod.h"
#include "oi.h"
#include "timer.h"

void initializeCommandModule(void){
  // Disable interrupts. ("Clear interrupt bit")
  cli();

  // One-time setup operations.
  setupIOPins();
  setupTimer();
  setupSerialPort();

  // Enable interrupts. ("Set interrupt bit")
  sei();
}

void setupIOPins(void) {
  // Set I/O pins
  DDRB  = 0x10;
  PORTB = 0xCF;
  DDRC  = 0x00;
  PORTC = 0xFF;
  DDRD  = 0xE6;
  PORTD = 0x7D;
}

void setupSerialPort(void) {
  // Set the transmission speed to 57600 baud, which is what the Create expects,
  // unless we tell it otherwise.
  UBRR0 = 19;
  
  // Enable both transmit and receive.
  UCSR0B = (_BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0));
    // UCSR0B = 0x18;
  
  // Set 8-bit data.
  UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));
    // UCSR0C = 0x06;
}

void byteTx(uint8_t value) {
  // Transmit one byte to the robot.
  // Wait for the buffer to be empty.
  while(!(UCSR0A & 0x20)) ;

  // Send the byte.
  UDR0 = value;
}

uint8_t byteRx(void) {
  // Receive one byte from the robot.
  // Call setupSerialPort() first.
  // Wait for a byte to arrive in the recieve buffer.
  while(!(UCSR0A & 0x80)) ;
  
  // Return that byte.
  return UDR0;
}

void baud(uint8_t baud_code) {
  // Switch the baud rate on both Create and module
  if(baud_code <= 11)
  {
    byteTx(CmdBaud);
    UCSR0A |= _BV(TXC0);
    byteTx(baud_code);
    // Wait until transmit is complete
    while(!(UCSR0A & _BV(TXC0))) ;

    cli();

    // Switch the baud rate register
    if(baud_code == Baud115200) {
      UBRR0 = Ubrr115200;
    } else if(baud_code == Baud57600) {
      UBRR0 = Ubrr57600;
    } else if(baud_code == Baud38400) {
      UBRR0 = Ubrr38400;
    } else if(baud_code == Baud28800) {
      UBRR0 = Ubrr28800;
    } else if(baud_code == Baud19200) {
      UBRR0 = Ubrr19200;
    } else if(baud_code == Baud14400) {
      UBRR0 = Ubrr14400;
    } else if(baud_code == Baud9600) {
      UBRR0 = Ubrr9600;
    } else if(baud_code == Baud4800) {
      UBRR0 = Ubrr4800;
    } else if(baud_code == Baud2400) {
      UBRR0 = Ubrr2400;
    } else if(baud_code == Baud1200) {
      UBRR0 = Ubrr1200;
    } else if(baud_code == Baud600) {
      UBRR0 = Ubrr600;
    } else if(baud_code == Baud300) {
      UBRR0 = Ubrr300;
    }
    sei();

    delayMs(100);
  }
}

