#ifndef INCLUDE_TIMER_H
#define INCLUDE_TIMER_H

  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include "iroblib.h"
  #include "oi.h"
  #include <stdio.h>

  // Interrupts.
  ISR(TIMER0_COMPA_vect);
  ISR(TIMER1_COMPA_vect);
  ISR(TIMER2_COMPA_vect);
  
  // Timer functions
  void setupTimer(void);
  void delayMs(uint16_t timeMs);
  void ToggleCMDTimer(uint16_t time_ms2);

  // Declaration of timer variables
  extern volatile uint16_t delayTimerCount;
  extern volatile uint8_t  delayTimerRunning;
  extern volatile uint16_t  ToggleCMDTimerCount;
  extern volatile uint8_t sensors[Sen6Size];
  volatile uint8_t timer2Scale;
  extern volatile uint8_t canPrint;
  extern volatile uint8_t canSense;
  extern volatile uint16_t driveTimerCount;
  extern volatile uint16_t rotateTimerCount;
  extern volatile uint16_t detectIR;

#endif
