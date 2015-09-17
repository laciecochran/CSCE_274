#include "iroblib.h"
#include "oi.h"
#include "cmod.h"

// Define songs to be played later
void defineSongs(void) {
  // Reset song
  byteTx(CmdSong);
  byteTx(RESET_SONG);
  byteTx(4);
  byteTx(60);
  byteTx(6);
  byteTx(72);
  byteTx(6);
  byteTx(84);
  byteTx(6);
  byteTx(96);
  byteTx(6);

  // Start song
  byteTx(CmdSong);
  byteTx(START_SONG);
  byteTx(6);
  byteTx(69);
  byteTx(18);
  byteTx(72);
  byteTx(12);
  byteTx(74);
  byteTx(12);
  byteTx(72);
  byteTx(12);
  byteTx(69);
  byteTx(12);
  byteTx(77);
  byteTx(24);
}

// Ensure that the robot is On.
void powerOnRobot(void) {
  // If Create's power is off, turn it on
  if(!RobotIsOn) {
    while(!RobotIsOn) {
      RobotPwrToggleLow;
      delayMs(500);  // Delay in this state
      RobotPwrToggleHigh;  // Low to high transition to toggle power
      delayMs(100);  // Delay in this state
      RobotPwrToggleLow;
    }
    delayMs(3500);  // Delay for startup
  }

  // Flush the buffer
  while( (UCSR0A & 0x80) && UDR0);
}

// Ensure that the robot is OFF.
void powerOffRobot(void) {
  // If Create's power is on, turn it off
  if(RobotIsOn) {
    while(RobotIsOn) {
      RobotPwrToggleLow;
      delayMs(500);  // Delay in this state
      RobotPwrToggleHigh;  // Low to high transition to toggle power
      delayMs(100);  // Delay in this state
      RobotPwrToggleLow;
    }
  }
}

//Turn on power Led given a specified color.
void powerLed(uint8_t color) {

  byteTx(CmdLeds); 
  byteTx(0x00); //both robot Leds off
  byteTx(color);
  byteTx(255); //intensity

}

//Setup the command module Leds
void setupCMDLeds(void) {

  //Set fifth and sixth bits of direction register of port D to 1.
  //pin 5 controls right Led, pin 6 controls left Led
  DDRD |= (3 << 5);

}

//Toggle the command module Leds
void toggleCMDLeds(void) {
  //turn the Leds off/on using exclusive or
  
  PORTD ^= (3 << 5);

}

//Detect left or right bumper. Set corresponding Led
void bumperLedsNotif(void) {

  //Ask about bump sensors
  byteTx(CmdSensors); //"read sensors"
  byteTx(7); //sensor packet 7 for bumps and wheels

  //read response and extract relevant information
  uint8_t bumps = byteRx();
  uint8_t bumpRight = bumps & (1 << 0);
  uint8_t bumpLeft  = bumps & (1 << 1);

  //set robot Leds
  if(bumpLeft && bumpRight) {robotLedsOn();}
  else if(bumpLeft) {robotLeftLedOn();}
  else if(bumpRight) {robotRightLedOn();}
  else {robotLedsOff();}

}

//Turn on robot's left Led
void robotLeftLedOn(void) {

  byteTx(CmdLeds); 
  byteTx(0x02); //
  byteTx(0);
  byteTx(255); //intensity

}

//Turn on robot's right Led
void robotRightLedOn(void) {

  byteTx(CmdLeds); 
  byteTx(0x08); //
  byteTx(0);
  byteTx(255); //intensity

}
//Turn on both play and advance Leds
void robotLedsOn(void) {

  byteTx(CmdLeds);
  byteTx(0x0a);
  byteTx(0);
  byteTx(255);

}

//Turn off robot's left Led
void robotLedsOff(void) {

  byteTx(CmdLeds); 
  byteTx(0x00); //
  byteTx(0);
  byteTx(255);
}

//drive the create around a pentagon clockwise
//vl needs to be negative
void drivePentagonCW(void) {

  for(uint8_t numRotates = 0; numRotates < 5; numRotates++) {

    driveStraight(V_HIGH, V_LOW, V_HIGH, V_LOW);
    delayMS(800);
    rotate();

  }

}

//drive the create around a pentagon counter clockwise
void drivePentagonCCW(void) {

  

}


//drive create straight for a specified distance
void driveStraight(uint8_t vr_high, uint8_t vr_low, uint8_t vl_high, uint8_t vl_low) {

  byteTx(CmdDriveWheels);
  byteTx(vr_high);
  byteTx(vr_low);
  byteTx(vl_high);
  byteTx(vl_low);
}

void rotate(uint8_t vr_high, uint8_t vr_low, uint8_t vl_high, uint8_t vl_low) {

  

}
