#include "timer.h"
#include "cmod.h"
#include "iroblib.h"
#include "oi.h"
#include <stdlib.h>

// Declare Global variables 

int main() {
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
    //USING HERSHEL!!!
    
    
    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}

