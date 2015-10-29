/* drive.c
 * Designed to run on Create Command Module
 *
 * The basic architecture of this program can be re-used to easily 
 * write a wide variety of Create control programs.  All sensor values
 * are polled in the background (using the serial rx interrupt) and 
 * stored in the sensors array as long as the function 
 * delayAndUpdateSensors() is called periodically.  Users can send commands
 * directly a byte at a time using byteTx() or they can use the 
 * provided functions, such as baud() and drive().
 */



// Includes
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "oi.h"



// Global variables
volatile uint16_t delayTimerCount   = 0;
volatile uint8_t  delayTimerRunning = 0;
  // timing variables
volatile uint8_t ledState = 0;
enum{LEDg = 0, LEDr = 1};

// Functions
void byteTx(uint8_t value);
void initializeCommandModule(void);
void powerOnRobot(void);
void powerOffRobot(void);
void baud(uint8_t baud_code);

int main (void) {
  initializeCommandModule();
    // Set up Create and module
  powerOnRobot();
    // Is the Robot on
  byteTx(CmdStart);
    // Start the create
  baud(Baud57600);
    // Set the baud rate for the Create and Command Module
  byteTx(CmdControl);
    // Deprecated form of safe mode. I use it because it will
    // turn of all LEDs, so it's essentially a reset.
  byteTx(CmdFull);
    // We are operating in FULL mode.

  for(;;)
  {
    // Turn off ?
    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}

// Timer 0 interrupt 
// SIGNAL(SIG_OUTPUT_COMPARE0A)
ISR(TIMER0_COMPA_vect) {
  if(delayTimerCount)
    delayTimerCount--;
  else
    delayTimerRunning = 0;
}

// Delay for the specified time in ms without updating sensor values
void delayMs(uint16_t time_ms)
{
  delayTimerRunning = 1;
  delayTimerCount = time_ms;
  while(delayTimerRunning) ;
}


// Transmit a byte over the serial port
void byteTx(uint8_t value)
{
  while(!(UCSR0A & _BV(UDRE0))) ;
  UDR0 = value;
}

/*-----------------------------------------------------
|      Initialize the ATmega168 microcontroller       |
| Command Module/Create Reserves the I/O Lines below  |
|------------------------------------------------------
|  PIN  |              DESCRIPTION              | DIR |
|------------------------------------------------------
| PB0-3 | Command Module ePorts                 |     |
|  PB4  | Ser Port Sel (1=USB, 0=Create)        | Out |
|  PB5  | Create Pwr Detect (1 = Create ON)     | In  |
| PB6-7 | CM internal use only. Set in hardware |     |
| PC0-5 | Command Module ePorts                 |     |
| PC6-7 | CM internal use only. set in hardware |     |
|  PD0  | Serial Rx                             | In  |
|  PD1  | Serial TX                             | Out |
|  PD2  | Create Dev Det Inp. Chg Creates Baud  | Out |
|  PD3  | USB Detect                            | In  |
|  PD4  | Command Module Soft Button            | In  |
|  PD5  | Command Module LED 1                  | Out |
|  PD6  | Command Module LED 2                  | Out |
|  PD7  | Create Pwr Toggle.                    | OUT |
|       |   (Low->Hi Toggles Pwr)               |     |
|----------------------------------------------------*/

/* ----------------------------------------------------
 * The frequency of the Command module is 18.432 MHz.
 *(Page 20 of the irobot Create command module)
 * --------------------------------------------------*/

void initializeCommandModule(void){
  // Disable interrupts. ("Clear interrupt bit")
  cli();

  // ---------------------------------------
  //  Configure I/O Pins
  // ---------------------------------------
  // Set I/O pins
  DDRB  = 0x10;
  PORTB = 0xCF;

  DDRC  = 0x00; 
  PORTC = 0xFF; // Enabes all PUs on Port C
  
  DDRD  = 0xE6; // Port D Reserved for Create/Cmd Functions
  PORTD = 0x7D; // See Table for "Initialize ATMega168 microcontroller"

  // ---------------------------------------------------
  //  Set up timer 0 to generate interrupts @ 1000Hz
  // ---------------------------------------------------
  TCCR0A = _BV(WGM01);              // Mode = CTC
  TCCR0B = (_BV(CS00) | _BV(CS02)); // CS0 | CS2 = CLK/1024
  OCR0A  = 17;                      // 18432000/(1024*1000) = 18

  // Turn on interrupts
  sei();
}

// Ensure that the robot is OFF.
void powerOffRobot(void) {
  // If Create's power is on, turn it off
  if(RobotIsOn) {
    while(RobotIsOn) {
      RobotPwrToggleLow;
      delayMs(500);  // Delay in this state
      RobotPwrToggleHigh;  // Low to high transition to toggle power
      delayMs(100);  // Delay in this state
      RobotPwrToggleLow;
    }
  }
}

void powerOnRobot(void)
{
  // If Create's power is off, turn it on
  if(!RobotIsOn)
  {
      while(!RobotIsOn)
      {
          RobotPwrToggleLow;
          delayMs(500);  // Delay in this state
          RobotPwrToggleHigh;  // Low to high transition to toggle power
          delayMs(100);  // Delay in this state
          RobotPwrToggleLow;
      }
      delayMs(3500);  // Delay for startup
  }
}

// Switch the baud rate on both Create and module
void baud(uint8_t baud_code)
{
  if(baud_code <= 11)
  {
    byteTx(CmdBaud);
    UCSR0A |= _BV(TXC0);
    byteTx(baud_code);
    // Wait until transmit is complete
    while(!(UCSR0A & _BV(TXC0))) ;

    cli();

    // Switch the baud rate register
    if(baud_code == Baud115200)
      UBRR0 = Ubrr115200;
    else if(baud_code == Baud57600)
      UBRR0 = Ubrr57600;
    else if(baud_code == Baud38400)
      UBRR0 = Ubrr38400;
    else if(baud_code == Baud28800)
      UBRR0 = Ubrr28800;
    else if(baud_code == Baud19200)
      UBRR0 = Ubrr19200;
    else if(baud_code == Baud14400)
      UBRR0 = Ubrr14400;
    else if(baud_code == Baud9600)
      UBRR0 = Ubrr9600;
    else if(baud_code == Baud4800)
      UBRR0 = Ubrr4800;
    else if(baud_code == Baud2400)
      UBRR0 = Ubrr2400;
    else if(baud_code == Baud1200)
      UBRR0 = Ubrr1200;
    else if(baud_code == Baud600)
      UBRR0 = Ubrr600;
    else if(baud_code == Baud300)
      UBRR0 = Ubrr300;

    sei();

    delayMs(100);
  }
}
