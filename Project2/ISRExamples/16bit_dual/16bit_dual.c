/* 16bit_dual.c
 * Designed to run on Create Command Module
 *
 */



// Includes
#include <avr/interrupt.h>
#include <avr/io.h>
#include "oi.h"

// Global variables
volatile uint16_t delayTimerCount   = 0;
volatile uint8_t  delayTimerRunning = 0;
  // timing variables

// Functions
void byteTx(uint8_t value);
void initializeCommandModule(void);
void powerOnRobot(void);
void powerOffRobot(void);
void baud(uint8_t baud_code);
void delayMs(uint16_t ms);

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

  delayMs(1000);
  LED1On;
  for(;;) {

    // Turn off ?
    if(UserButtonPressed) {
      powerOffRobot();
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

//-------------------------------------------------
//    Timer 1 ISR 
//-------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  byteTx(139);  // leds
  byteTx(0);    // set advance and play to OFF
  byteTx(255);  // red
  byteTx(255);  // full intensity 
}

ISR(TIMER1_COMPB_vect) { 
  byteTx(139);  // leds
  byteTx(0);    // set advance and play to OFF
  byteTx(0);    // green
  byteTx(255);  // full intensity 
}



// Delay for the specified time in ms without updating sensor values
void delayMs(uint16_t time_ms) {
  cli();
  delayTimerRunning = 1;
  delayTimerCount = time_ms;
  sei();

  while(delayTimerRunning) {
    asm("nop");
  }
}


// Transmit a byte over the serial port
void byteTx(uint8_t value) {
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
  TIMSK0 = _BV(OCIE0A);             // Enable output compare A interrupt

  // ---------------------------------------------------
  //  Set up timer 1 to generate interrupts @ 1Hz
  // ---------------------------------------------------
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS10) | _BV(CS12));  // WGM12 | CS10 | CS12 = CTC Mode, CLK/1024
  OCR1A  = 35999;                                 // 18432000/(1024*.5) = 36,000
  OCR1B  = 17999;                                 // 18432000/(1024*1)  = 18,000
  TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);             // Enable output compare A and B interrupt

  // ---------------------------------------------------
  //  Set up the serial port with rx interrupt
  // ---------------------------------------------------
  // Set the transmission speed to 57600 baud, which is what the Create expects,
  // unless we tell it otherwise.
  UBRR0 = 19;
  
  // Enable both transmit and receive.
  UCSR0B = (_BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0));
     // UCSR0B = 0x18;
  
  // Set 8-bit data.
  UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));
     // UCSR0C = 0x06;
      
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
