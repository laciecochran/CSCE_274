#include "timer.h"
#include "cmod.h"
#include "iroblib.h"
#include "oi.h"
#include <stdlib.h>

// Declare Global variables 
uint8_t isDriving = 0;
uint8_t isRotating = 0;
uint16_t wall = 0;
int8_t state = 0;

int main(void) {
  // Set up Create and module
  initializeCommandModule();
    
  powerOnRobot();
    // Is the Robot on
  byteTx(CmdStart);
    // Start the create
  baud(Baud57600);
    // Set the baud rate for the Create and Command Module
  defineSongs();
    // Define some songs so that we know the robot is on.
  byteTx(CmdControl);
    // Deprecated form of safe mode. I use it because it will
    // turn of all LEDs, so it's essentially a reset.
  byteTx(CmdFull);
    // We are operating in FULL mode.

  // Setup
  //Set up cmd leds
  DDRD |= (3 <<5);
  
  // CSCE 274 students: I would make sure the robot stops. 
  //                    As a precaution for the robot and your grade.
  stopCreate();
  delayMs(20);

  // Play the reset song and wait while it plays.
  byteTx(CmdPlay);
  byteTx(RESET_SONG);
  delayMs(750);

  // Turn the power button on to something.
  powerLed(0); //green
  //powerLed(255); //red

  // Initialize global variables
  delayMs(100);
  //turn on one CMD Led so that it will toggle left and right
  PORTD &= ~(0x40);

  //set up timer 1 so we don't ask for sensors before the create is initiallized
  //setupTimerOne();
  
  // Infinite operation loop
  for(;;) {
    // toggle command module Leds using a timer
    toggleCMDLeds(1000);
    //USING HERSHEL<F5>!!!
    updateSensors();
    delayMs(25);
    clearPIDVar();
    
    //Project 3 tasks: PID
    switch(state){
      case 0:
        //finding wall
        robotLeftLedOn();
        state = findWall();
        break;
      case 1:
        //We have a wall!
        robotRightLedOn();
        state = alignWall();
        break;
      case 2:
        //follow wall
        robotLedsOn();
        state = pid();
        break;
      case 3:
         //dock on charger
         powerLed(255);
         state = runDockOver();
         powerLed(0);
        break;
      case 4:
	//we are docked and charging
         stopCreate();
         powerLed(127);
         robotLedsOff();
         //happy duck song...and he waddled away
         break;
      default: robotLedsOff();
    }
    //printSensorData();
    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}
