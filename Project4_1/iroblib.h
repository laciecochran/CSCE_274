#ifndef INCLUDE_IROBLIB_H
#define INCLUDE_IROBLIB_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "oi.h"

// Constants
#define RESET_SONG 0
#define START_SONG 1
//distance between create wheels
#define L 260
//define the drive velocity for the create
#define V 100
#define lilV 20

//PID constants
#define dt 0.25 //Seconds x/0.25 = x << 4 and x*0.25 = x >> 4
#define pTerm 2
#define iTerm 2
#define dTerm 1
#define cGain 3 
#define refPoint 80
#define max 100
#define min 0
#define AntiWToleranceLow -500
#define AntiWToleranceHigh 500

//docking constants
#define bouyError 20
#define dockRefPoint 254
#define dockPTerm 4
#define dockITerm 2
#define dockDTerm 1
#define dockCGain 2

//field constants
#define medV 30
#define fieldError 30
#define fieldPTerm 4
#define fieldITerm 2
#define fieldDTerm 1
#define fieldCGain 2

//PID variables
int16_t errorTerm;
int16_t errorTermPrev;
int32_t sumOfError;
int16_t controlOut;
int8_t usingPID;
int16_t totalWall;
int16_t velocityLeft;
int16_t velocityRight;

//docking variables
uint8_t IRValue;
uint8_t homeBase;
int8_t inForceField;

//Sensor 
uint8_t sensors[Sen6Size];

void defineSongs(void);
  // Songs
  // Indicator that the robot is Powered on and has reset.

// Power the create On/Off
void powerOnRobot(void);
void powerOffRobot(void);

//turn on and change color of power Led
void powerLed(uint8_t color);

//setup to be able to use CMD Leds
void setupCMDLeds(void);

//toggle cmd leds
void toggleCMDLeds(uint16_t time);

//Robot LEDs
void robotLeftLedOn(void);
void robotRightLedOn(void);
void robotLedsOn(void);
void robotLedsOff(void);

//drive commands
void driveStraight(uint16_t v);
void rotate(int16_t v);
void driveCreate(int16_t vr, int16_t vl);
void stopCreate(void);

//print to terminal on workstation
void printToConsole(char printData[]);
void printSensorData(void);

//get all sensor values
void updateSensors(void);


//PID controller functions
int8_t findWall(void);
int8_t alignWall(void);
void clearPIDVar(void);
int8_t pid(void);
uint16_t getTotalWall(void);

//docking
int8_t runDockOver(void);
int8_t pidDock(void);
#endif
