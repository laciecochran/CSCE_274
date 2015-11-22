#include "timer.h"    // Declaration made available here
#include "cmod.h"

// Timer variables defined here
volatile uint16_t delayTimerCount = 0;            // Definition checked against declaration
volatile uint16_t ToggleCMDTimerCount = 0; 
volatile uint16_t driveTimerCount = 0;           // Definition checked against declaration
volatile uint16_t rotateTimerCount = 0;
volatile uint8_t  delayTimerRunning = 0;          // Definition checked against declaration
volatile uint16_t sensorTimerCount = 0;
volatile uint16_t pidTimerCount = 0;

ISR(USART_RX_vect) {  //SIGNAL(SIG_USART_RECV) 
  // Serial receive interrupt to store sensor values
  
  // CSCE 274 students, I have only ever used this method 
  // when retrieving/storing a large amount of sensor data. 
  // You DO NOT need it for this assignment. If i feel it 
  // becomes relevant, I will show you how/when to use it.
}

//SIGNAL(SIG_OUTPUT_COMPARE1A)
ISR(TIMER0_COMPA_vect) {
  // Interrupt handler called every 1ms.
  // Decrement the counter variable, to allow delayMs to keep time.
  if(delayTimerCount != 0) {
    delayTimerCount--;
  } else {
    delayTimerRunning = 0;
  } 
  if(ToggleCMDTimerCount  != 0) {
    ToggleCMDTimerCount--;
  }
  if(sensorTimerCount != 0) {
    sensorTimerCount--;
  }
  if(pidTimerCount != 0) {
    pidTimerCount--;
  }
  if(driveTimerCount != 0) {
    driveTimerCount--;
  }

  if(rotateTimerCount != 0) {
    rotateTimerCount--;
  }
}
/*
ISR(TIMER1_COMPA_vect) {
  updateSensors();
}
*/

/*
ISR(TIMER2_COMPA_vect) {
  toggleCMDLeds() 
}

ISR(TIMER2_COMPB_vect) {
  toggleCMDLeds() 
}
*/

void setupTimer(void) {
// Set up timer 0
// Use: cause an interrupt every 1ms
// Mode: CTC
// Prescalar: 1024

TCCR0A = _BV(WGM01); //mode
TCCR0B = (_BV(CS00) | _BV(CS02)); //scalar
OCR0A = 17;
TIMSK0 = _BV(OCIE0A);
}

/*
void setupTimerOne(void){
// Set up the timer 1
// Use: cause an interrupt to get all sensor data every 30ms
// Mode: CTC
// Prescalar: 1024
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS10) | _BV(CS12)); //mode and prescalar
    // TCCR1B = 0x0C;
  OCR1A = 1799;
  TIMSK1 = _BV(OCIE1A);
    // TIMSK1 = 0x02;
}
*/

/*
void setupTimerTwo(void){
// Set up the timer 2
// Use: cause an interrupt every 15ms
// Mode: CTC
// Prescalar: 1024
  TCCR2A = _BV(WGM21); //Mode
  TCCR2B = (_BV(CS22) | _BV(CS21) | _BV(CS20)); //prescalar
  OCR2A = 255;
  OCR2B = 255;
  TIMSK2 = _BV(OCIE2A);
}
*/

// Delay for the specified time in ms without updating sensor values
void delayMs(uint16_t time_ms)
{
  delayTimerRunning = 1;
  delayTimerCount = time_ms;
  while(delayTimerRunning) ;
}
