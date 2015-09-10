#ifndef INCLUDE_IROBLIB_H
#define INCLUDE_IROBLIB_H

#include <avr/io.h>
#include <avr/interrupt.h>

// Constants
#define RESET_SONG 0
#define START_SONG 1

void defineSongs(void);
  // Songs
  // Indicator that the robot is Powered on and has reset.

void powerOnRobot(void);
void powerOffRobot(void);
void powerLed(uint8_t color);
void setupCMDLeds(void);
void toggleCMDLeds(void);
void bumperLedsNotif(void);
void robotLeftLedOn(uint8_t color);
void robotRightLedOn(uint8_t color);
void robotLeftLedOff(uint8_t color);
void robotRightLedOff(uint8_t color);
  // Power the create On/Off
#endif