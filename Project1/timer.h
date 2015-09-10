#ifndef INCLUDE_TIMER_H
#define INCLUDE_TIMER_H

  #include <avr/io.h>
  #include <avr/interrupt.h>

  // Interrupts.
  ISR(TIMER1_COMPA_vect);

  // Timer functions
  void setupTimer(void);
  void delayMs(uint16_t timeMs);

  // Declaration of timer variables
  extern volatile uint16_t delayTimerCount;
  extern volatile uint8_t  delayTimerRunning;
#endif