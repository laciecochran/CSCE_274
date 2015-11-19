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

//PID constants
#define dt 1
#define pTerm 20
#define iTerm 0
#define dTerm 0
#define refPoint 50

//PID variables
int16_t errorTerm;
int16_t errorTermPrev;
int32_t sumOfError;
int16_t controlOut;

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
void printSensorData(void);
//print to terminal on workstation
void updateSensors(void);
//get all sensor values
int8_t findWall(void);
int8_t alignWall(void);
int16_t pid(int16_t sensor);
//PID controller functions
#endif
