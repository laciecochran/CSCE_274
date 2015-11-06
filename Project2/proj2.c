#include "timer.h"
#include "cmod.h"
#include "iroblib.h"
#include "oi.h"
#include <stdlib.h>

// Declare Global variables 
volatile uint8_t IRValue;

uint8_t isDriving = 0;
uint8_t isRotating = 0;

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

  // Infinite operation loop
  for(;;) {
    // toggle command module Leds using a timer
    toggleCMDLeds(1000);
    //USING HERSHEL<F5>!!!
    if(canSense) {
      updateSensors();
    }
	
		if(isDriving){
			// unsafe
			if(sensors[SenBumpDrop] | sensors[SenCliffL] | sensors[SenCliffFL] | sensors[SenCliffFR] | sensors[SenCliffR]) {
				driveTimerCount=0;		
			}
			if(driveTimerCount==0) {
				stopCreate();
				isDriving=0;
			}
		}
		if(isRotating){
			if(sensors[SenBumpDrop] & 28) {
				rotateTimerCount=0;
			}
			if(rotateTimerCount==0) {
				stopCreate();
				isRotating=0;
			}
		}


		if((detectIR==0) && !isDriving && !isRotating) {
			IRValue = sensors[SenIRChar];
			if(IRValue == IRForward) {
			//if(!(sensors[SenBumpDrop] | sensors[SenCliffL] | sensors[SenCliffFL] | sensors[SenCliffFR] | sensors[SenCliffR])){
				driveStraight(V);
				driveTimerCount = DRIVE_D;
				isDriving=1;
			//}
			}
			else if(IRValue == IRLeft) {
			//if(!(sensors[SenBumpDrop] & 28)) {
				rotate(V, ~V);
				rotateTimerCount = ROTATE_30_D; 
				isRotating=1;
			//}
		
	    }
	    else if(IRValue == IRRight) {
			//if(!(sensors[SenBumpDrop] & 28)) {
				rotate(~V, V);
				rotateTimerCount =  ROTATE_30_D; 
				isRotating=1;
			}
			detectIR=100;
		}	 

    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}
