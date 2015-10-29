#include "timer.h"    // Declaration made available here


// Timer variables defined here
volatile uint16_t delayTimerCount = 0;            // Definition checked against declaration
volatile uint16_t ToggleCMDTimerCount = 0;            // Definition checked against declaration

volatile uint8_t  delayTimerRunning = 0;          // Definition checked against declaration

//http://www.avrfreaks.net/forum/solved-serial-echo-using-usartrxvect-interrupt-issue
ISR(USART_RX_vect) {  //SIGNAL(SIG_USART_RECV) 
  // Serial receive interrupt to store sensor values
  
  // CSCE 274 students, I have only ever used this method 
  // when retrieving/storing a large amount of sensor data. 
  // You DO NOT need it for this assignment. If i feel it 
  // becomes relevant, I will show you how/when to use it.
}

ISR(TIMER1_COMPA_vect) {
  //byteTx(CmdSensors);
  //byteTx(6); //get all the data!
  //move to timer 2 which interrupts at every 50ms

  
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

}

void setupTimer(void) {
// Set up timer 0
// Use: cause an interrupt every 1ms
// Mode: CTC
// Prescalar: 1024

TCCR0A = _BV(WGM01); //mode
TCCR0B = (_BV(CS00) | _BV(CS02)); //scalar
OCR0A = 17;
TIMSK0 = _BV(OCIE0A);

// Set up the timer 1
// Use: cause an interrupt to get all sensor data every 1s
// Mode: CTC
// Prescalar: 1024
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS10) | _BV(CS12)); //mode and prescalar
    // TCCR1B = 0x0C;
  OCR1A = 17999;
  TIMSK1 = _BV(OCIE1A);
    // TIMSK1 = 0x02;
}

// Delay for the specified time in ms without updating sensor values
void delayMs(uint16_t time_ms)
{
  delayTimerRunning = 1;
  delayTimerCount = time_ms;
  while(delayTimerRunning) ;
}
