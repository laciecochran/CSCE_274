#ifndef INCLUDE_IROBLIB_H
#define INCLUDE_IROBLIB_H

#include <avr/io.h>
#include <avr/interrupt.h>

// Constants
#define RESET_SONG 0
#define START_SONG 1
//distance between create wheels
#define L 260
//angle for pentagon
#define ROTATE_ANGLE 72 //180-108
//these are hex values for 100 mm/sec
#define V_HIGH 0x00
#define V_LOW 0x64
#define V_HIGH_N 0x00
#define V_LOW_N 0x9c
#define V 100

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
void buttonDetect(void);
//detect play and advance buttons
void robotLeftLedOn(void);
void robotRightLedOn(void);
void robotLedsOff(void);
void robotLedsOn(void);
//turn on/off the play and advance Leds

void drivePentagonCW(void);
void drivePentagonCCW(void);
void driveStraight(uint16_t vr, uint16_t vl);
void rotate(uint16_t vr, uint16_t vl);
//drive the create around a pentagon
void stopCreate(void);
//stop the create's motion
#endif
