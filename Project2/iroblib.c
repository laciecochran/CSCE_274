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

//Detect left or right bumper. Set corresponding Led
//Probably get rid of for Project 2
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

//toggle cmd leds
void toggleCMDLeds(uint16_t time) {
  if(ToggleCMDTimerCount  == 0){
    PORTD ^= (3 << 5);	
    ToggleCMDTimerCount = time;
  }
}

//detect the play and advance buttons
void buttonDetect(void) {

  //Ask about bump sensors
  byteTx(CmdSensors); //"read sensors"
  byteTx(18); //sensor packet 18 for play and advance buttons

  uint8_t buttons = byteRx();
  uint8_t play = buttons & (1 << 0);
  uint8_t advance  = buttons & (1 << 2);

  if(play) {}
  else if(advance) {}
  else {stopCreate();}


}

//drive create straight for a specified distance
void driveStraight(uint16_t v) {

  byteTx(CmdDriveWheels);
  byteTx((v>>8)&0xFF);
  byteTx(v&0xFF);
  byteTx((v>>8)&0xFF);
  byteTx(v&0xFF);
}

void rotate(int16_t vr, int16_t vl) {

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

void printToConsole(char printData[printLSize]){
  uint8_t i = 0;
  for (i = 0; i < strlen(printData); i++) {
    byteTx(printData[i]);
  }
}

void printSensorData(void){
  setSerialDestination(SERIAL_USB);
  char printL[printLSize];
  // Sensor Nick wants #1: 16-bit value
  sprintf(printL,"Left Cliff Signal: %u\n", (uint16_t)((sensors[SenCliffLSig1]<<8)| sensors[SenCliffLSig0]));
  printToConsole(printL);
  // Sensor Nick Wants #2
  sprintf(printL,"Left Front Cliff Signal: %u\n", (uint16_t)((sensors[SenCliffFLSig1]<<8)| sensors[SenCliffFLSig0]));
  printToConsole(printL);
  // Sensor Nick Wants #3
  sprintf(printL,"Right Front Cliff Signal: %u\n", (uint16_t)((sensors[SenCliffFRSig1]<<8)| sensors[SenCliffFRSig0]));
  printToConsole(printL);
  // Sensor Nick Wants #4
  sprintf(printL,"Right Cliff Signal: %u\n", (uint16_t)((sensors[SenCliffRSig1]<<8 | sensors[SenCliffRSig0])));
  printToConsole(printL);
  // Sensor Nick Wants #5
  sprintf(printL,"Wall Signal: %u\n", (uint16_t)((sensors[SenWallSig1]<<8) | (sensors[SenWallSig0])));
  printToConsole(printL);
  // Sensor Nick Wants #6
  sprintf(printL,"Charging State: %u\n", (uint16_t)(sensors[SenChargeState]));
  printToConsole(printL);
  // Sensor Nick Wants #7
  sprintf(printL,"Battery Voltage: %u mV\n", (uint16_t)((sensors[SenCharge1]<<8) | (sensors[SenCharge0])));
  printToConsole(printL);
  // Sensor Nick Wants #8
  sprintf(printL,"Battery Current: %i mA\n", (uint16_t)((sensors[SenCurr1]<<8) | (sensors[SenCurr0])));
  printToConsole(printL);
  // Sensor Nick Wants #9
  sprintf(printL,"Battery Temperature: %i degrees C.\n", (uint16_t)((sensors[SenTemp])));
  printToConsole(printL);
  // Sensor Nick Wants #10
  sprintf(printL,"Battery Charge: %u mAh\n", (uint16_t)((sensors[SenCharge1]<<8) | (sensors[SenCharge0])));
  printToConsole(printL);
  // Sensor Nick Wants #11
  sprintf(printL,"Battery Capacity: %u mAh\n", (uint16_t)((sensors[SenCap1]<<8) | (sensors[SenCap0])));
  printToConsole(printL);
  // End line for Formatting
  sprintf(printL,"\n");
  printToConsole(printL);
  //Change Back
  setSerialDestination(SERIAL_CREATE);
  canPrint=0;
}

void updateSensors(void){
  //cli();
  byteTx(CmdSensors);
  byteTx(6); //get all the data!
  for(uint8_t i = 0; i < 52; i++){
    sensors[i] = byteRx();  // read each sensor byte 
  } 
  canSense=0;
  //sei();  
}
