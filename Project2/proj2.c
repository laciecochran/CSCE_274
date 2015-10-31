#include "timer.h"
#include "cmod.h"
#include "iroblib.h"
#include "oi.h"
#include <stdlib.h>

// Declare Global variables 
volatile uint8_t IRValue;

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
    /*if(canPrint) {
      setSerialDestination(SERIAL_USB);
      cli();
      char printL[printLSize];
      // Sensor Nick wants #1: 16-bit value
      sprintf(printL,"IR Signal: %u %u %u\n", (uint8_t)(sensors[SenIRChar]), (uint8_t)(IRValue), (uint8_t) (driveTimerCount));
      printToConsole(printL);
      sprintf(printL,"Bumps and Wheeldrops: %u\n\n", (uint8_t)(sensors[0]));
      printToConsole(printL);
      sei();
      setSerialDestination(SERIAL_CREATE);
      canPrint=0;
    }*/

	    if((sensors[SenIRChar] == IRForward) || (sensors[SenIRChar] == IRLeft) || (sensors[SenIRChar] == IRRight)) {
	      IRValue = sensors[SenIRChar];
	    } else {
	      IRValue = 255;
	    }


	    if(IRValue == IRForward) {
		if(!(sensors[SenBumpDrop] | sensors[SenWall] | sensors[SenCliffL] | sensors[SenCliffFL] | sensors[SenCliffFR] | sensors[SenCliffR])) {
		  if(driveTimerCount == 0) {
		    driveStraight(V);
		    driveTimerCount = 25;
		  }
		}
	       else {
		 stopCreate();
		 driveTimerCount = 2;//0;
	       }
	    }
	    else if(IRValue == IRLeft) {
		if(!(sensors[SenBumpDrop] & 28)) {
		  if(driveTimerCount == 0) {
		    rotate(V, ~V);
		    driveTimerCount = 39; 
		  }
		}
		else {
		  stopCreate();
		  driveTimerCount = 0;
		}	
	    }
	    else if(IRValue == IRRight) {
		if(!(sensors[SenBumpDrop] & 28)) {
		  if(driveTimerCount == 0) {
		    rotate(~V, V);
		    driveTimerCount = 39; 
		  }
		}
		else {
		  stopCreate();
		  driveTimerCount = 0;
		}
	    }
	    else {
	      stopCreate();
	    }

	    if(driveTimerCount == 0) {
	      IRValue = 255;
	      stopCreate();
	    }

    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}

