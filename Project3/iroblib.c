#include "iroblib.h"
#include "oi.h"
#include "cmod.h"
#include "timer.h"

//define PID variables
errorTerm = 0;
errorTermPrev = 0;
sumOfError = 0;
controlOut = 0;

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

void printToConsole(char printData[]){
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

int8_t findWall(void){
  isDriving = 1;
  driveTimerCount = 100;
  driveStraight(V);
  int16_t totalWall = (sensors[SenWallSig1] << 8) & sensors[SenWallSig0];
  if(sensors[SenBumpDrop] && (totalWall > refPoint)){
    driveTimerCount = 0;
  }
  if(driveTimerCount == 0) {
    stopCreate();
    isDriving = 0;
  }
}

int8_t alignWall(void){

}

int16_t pid(int16_t sensor){
  errorTerm = sensor - refPoint;
  //poportional Term
  controlOut = (int16_t)(pTerm * errorTerm);
  //Integral Anti-Windup
  //if(//output not maxed){
    //integral Term
    sumOfError += errorTerm;
    controlOut += (int16_t)(iTerm * dt * sumOfError);
  //}
  //drivative term aka that thingy that does tha thing
  controlOut += (int16_t)(dTerm * (errorTerm - errorTermPrev)/dt);
  errorTermPrev = errorTerm;
  return controlOut;
}
