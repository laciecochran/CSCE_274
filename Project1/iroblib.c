#include "iroblib.h"
#include "oi.h"
#include "cmod.h"
#include "timer.h"

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

//Turn off robot's Led
void robotLedsOff(void) {

  byteTx(CmdLeds); 
  byteTx(0x00); //
  byteTx(0);
  byteTx(255);
}

//detect the play and advance buttons
void buttonDetect(void) {

  //Ask about bump sensors
  byteTx(CmdSensors); //"read sensors"
  byteTx(18); //sensor packet 18 for play and advance buttons

  uint8_t buttons = byteRx();
  uint8_t play = buttons & (1 << 0);
  uint8_t advance  = buttons & (1 << 2);

  if(play) {drivePentagonCW();}
  else if(advance) {drivePentagonCCW();}
  else {stopCreate();}


}

//drive the create around a pentagon clockwise
//vl needs to be positive
void drivePentagonCW(void) {

  for(uint8_t numRotates = 0; numRotates < 5; numRotates++) {

    driveStraight(V, V);
    delayMs(DRIVE_D);
    stopCreate();
    rotate(~V, V);
    delayMs(ROTATE_72_D);
    stopCreate();
  }
}

//drive the create around a pentagon counter clockwise
//vl needs to be negative
//The rotates before/after the for loop account for beginning/ending
//orientation.
void drivePentagonCCW(void) {

  rotate(~V, V);
  delayMs(ROTATE_108_D);  
  stopCreate();
  for(uint8_t numRotates = 0; numRotates < 5; numRotates++) {

    driveStraight(V, V);
    delayMs(DRIVE_D);
    stopCreate();
    rotate(V, ~V);
    delayMs(ROTATE_72_D);
    stopCreate();
  }
  rotate(V, ~V);
  delayMs(ROTATE_108_D);
  stopCreate();
}


//drive create straight for a specified distance
void driveStraight(uint16_t vr, uint16_t vl) {

  byteTx(CmdDriveWheels);
  byteTx((vr>>8)&0xFF);
  byteTx(vr&0xFF);
  byteTx((vl>>8)&0xFF);
  byteTx(vl&0xFF);
}

void rotate(uint16_t vr, uint16_t vl) {

  byteTx(CmdDriveWheels);
  byteTx((vr>>8)&0xFF);
  byteTx(vr&0xFF);
  byteTx((vl>>8)&0xFF);
  byteTx(vl&0xFF);

}

void stopCreate(void) {

  byteTx(CmdDriveWheels);
  byteTx(0);
  byteTx(0);
  byteTx(0);
  byteTx(0);

}
