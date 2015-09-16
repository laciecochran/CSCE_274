#include "timer.h"    // Declaration made available here


// Timer variables defined here
volatile uint16_t delayTimerCount = 0;            // Definition checked against declaration
volatile uint16_t ToggleCMDTimerCount = 0;            // Definition checked against declaration

volatile uint8_t  delayTimerRunning = 0;          // Definition checked against declaration
volatile uint8_t  ToggleCMDTimerRunning = 0;          // Definition checked against declaration



ISR(USART_RX_vect) {  //SIGNAL(SIG_USART_RECV) 
  // Serial receive interrupt to store sensor values
  
  // CSCE 274 students, I have only ever used this method 
  // when retrieving/storing a large amount of sensor data. 
  // You DO NOT need it for this assignment. If i feel it 
  // becomes relevant, I will show you how/when to use it.
}

//SIGNAL(SIG_OUTPUT_COMPARE1A)
ISR(TIMER1_COMPA_vect) {
  // Interrupt handler called every 1ms.
  // Decrement the counter variable, to allow delayMs to keep time.
  if(delayTimerCount != 0) {
    delayTimerCount--;
  } else {
    delayTimerRunning = 0;
  }
    if(ToggleCMDTimerRunning  != 0) {
    ToggleCMDTimerCount--;
    ToggleCMDTimerRunning  = 1;
  } else {
    ToggleCMDTimerRunning  = 0;
  }

}

void setupTimer(void) {
// Set up the timer 1 interupt to be called every 1ms.
// It's probably best to treat this as a black box.
// Basic idea: Except for the 71, these are special codes, for which details
// appear in the ATMega168 data sheet. The 71 is a computed value, based on
// the processor speed and the amount of "scaling" of the timer, that gives
// us the 1ms time interval.
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS12));
    // TCCR1B = 0x0C;
  OCR1A = 71;
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
void ToggleCMDTimer(uint16_t time_ms2)
{
  if(ToggleCMDTimerRunning  == 0){
    PORTD ^= (3 << 5);
    ToggleCMDTimerCount = time_ms2;
  //toggleCMDLeds();
  }
}
