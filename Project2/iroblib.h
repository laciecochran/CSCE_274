#ifndef INCLUDE_IROBLIB_H
#define INCLUDE_IROBLIB_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

// Constants
#define RESET_SONG 0
#define START_SONG 1
//distance between create wheels
#define L 260
//angle for pentagon
#define ROTATE_ANGLE 72 //180-108
//define the drive velocity for the create
#define V 100
//drive and rotate delays
#define DRIVE_D 8400
#define ROTATE_72_D 1715
#define ROTATE_108_D 2570 //1715*1.5=2573


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
void bumperLedsNotif(void);
//bumper detection
void buttonDetect(void);
//detect play and advance buttons
void robotLeftLedOn(void);
void robotRightLedOn(void);
void robotLedsOff(void);
void robotLedsOn(void);
//toggle cmd leds
void toggleCMDLeds(uint16_t time);

void driveStraight(uint16_t v);
void rotate(int16_t vr, int16_t vl);
//drive the create around a pentagon
void stopCreate(void);
//stop the create's motion
void printToConsole(char printData[]);

void updateSensors(void);
#endif
