#ifndef INCLUDE_IROBLIB_H
#define INCLUDE_IROBLIB_H

#include <avr/io.h>
#include <avr/interrupt.h>

// Constants
#define RESET_SONG 0
#define START_SONG 1
#define ROTATE_ANGLE 180-108
#define V_HIGH 0x06
#define V_LOW 0x04

void defineSongs(void);
  // Songs
  // Indicator that the robot is Powered on and has reset.

void powerOnRobot(void);
void powerOffRobot(void);
// Power the create On/Off
void powerLed(uint8_t color);
//turn on and change color of power Led
void setupCMDLeds(void);
//setup to be able to use CMD Leds
void toggleCMDLeds(void);
//toggle the CDM Leds (need to make this toggle left/right, not just on/off)
void bumperLedsNotif(void);
//bumper detection
void robotLeftLedOn(void);
void robotRightLedOn(void);
void robotLedsOff(void);
void robotLedsOn(void);
//turn on/off the play and advance Leds

void drivePentagonCW(void);
void drivePentagonCCW(void);
void driveStraight(uint8_t vr_high, uint8_t vr_low, uint8_t vl_high, uint8_t vl_low);
void rotate(uint8_t vr_high, uint8_t vr_low, uint8_t vl_high, uint8_t vl_low);
//drive the create around a pentagon
#endif
