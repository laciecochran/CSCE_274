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

  // CSCE 274 students: I would make sure the robot stops. 
  //                    As a precaution for the robot and your grade.

  // Play the reset song and wait while it plays.
  byteTx(CmdPlay);
  byteTx(RESET_SONG);
  delayMs(750);

  // Turn the power button on to something.
  //powerLed(0); //green
  powerLed(255); //red

  // Initialize global variables

  // Setup leds
  setupCMDLeds();

  // Infinite operation loop
  for(;;) {
    // toggle command module Leds
    toggleCMDLeds();
    delayMs(500);
    if(UserButtonPressed) {
      powerOffRobot();
      exit(1);
    }
  }
}

