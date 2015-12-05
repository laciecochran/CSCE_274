#include "iroblib.h"
#include "oi.h"
#include "cmod.h"
#include "timer.h"

//initiallize variables
int16_t errorTerm = 0;
int16_t errorTermPrev = 0;
int32_t sumOfError = 0;
int16_t controlOut = 0;
int8_t usingPID = 0;
int16_t totalWall = 0;
int16_t velocityLeft = 0;
int16_t velocityRight = 0;
uint8_t IRValue = 255; //initially not reading anything
uint8_t homeBase = 0; //initially not charging
int8_t inForceField = 0; //1=true, 0=false

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

//toggle command module leds
void toggleCMDLeds(uint16_t time) {
  if(ToggleCMDTimerCount  == 0){
    PORTD ^= (3 << 5);	
    ToggleCMDTimerCount = time;
  }
}

//drive create straight at a specified speed
void driveStraight(uint16_t v) {

  byteTx(CmdDriveWheels);
  byteTx((v>>8)&0xFF);
  byteTx(v&0xFF);
  byteTx((v>>8)&0xFF);
  byteTx(v&0xFF);
}

//rotate in place at a specified speed
void rotate(int16_t v) {

  byteTx(CmdDriveWheels);
  byteTx((v>>8)&0xFF);
  byteTx(v&0xFF);
  byteTx((-v>>8)&0xFF);
  byteTx(-v&0xFF);

}

//Drive the Create at specified speed for both wheels
void driveCreate(int16_t vr, int16_t vl) {

  byteTx(CmdDriveWheels);
  byteTx((vr>>8)&0xFF);
  byteTx(vr&0xFF);
  byteTx((vl>>8)&0xFF);
  byteTx(vl&0xFF);

}

//stop the Create
void stopCreate(void) {

  byteTx(CmdDriveWheels);
  byteTx(0);
  byteTx(0);
  byteTx(0);
  byteTx(0);

}

//Sends data a byte at a time to console
void printToConsole(char printData[]){
  uint8_t i = 0;
  for (i = 0; i < strlen(printData); i++) {
    byteTx(printData[i]);
  }
}

//Sensor data desired to be send to console
void printSensorData(void){
  setSerialDestination(SERIAL_USB);
  char printL[100];
  // Sensor Nick wants #1: 16-bit value
  sprintf(printL,"Wall Signal: %u\n", (uint16_t)((sensors[SenWallSig1]<<8) | (sensors[SenWallSig0])));
  printToConsole(printL);
  // End line for Formatting
  sprintf(printL,"\n");
  printToConsole(printL);
  //Change Back
  setSerialDestination(SERIAL_CREATE);
}

void updateSensors(void){
  if(sensorTimerCount == 0) {
    byteTx(CmdSensors);
    byteTx(6); //get all the data!
    for(uint8_t i = 0; i < 52; i++){
      sensors[i] = byteRx();  // read each sensor byte 
    }

    // reset timer
    sensorTimerCount=75;
  }
}
/*int8_t saftyDriveCheck(){
  cliffCheck = (sensors[SenCliffL] | sensors[SenCliffFL] | sensors[SenCliffFR] | sensors[SenCliffR]);
  
  if(sensors[SenBumpDrop] | ){

  }
}*/

uint16_t getTotalWall(void) {
  return (sensors[SenWallSig1] << 8) | sensors[SenWallSig0];
}

int8_t findWall(void){
  driveTimerCount = 1000;
  driveStraight(V);
  while(driveTimerCount != 0){
    updateSensors();
    delayMs(25);
    totalWall = getTotalWall();
    if(sensors[SenBumpDrop]){
      stopCreate();
      return 1;
    }
    else if(totalWall > refPoint){
      return 2;
    }
    /*else if(sensors[SenIRChar] > 241 && sensors[SenIRChar] < 255) {
      clearPIDVar();
      stopCreate();
      return 3;
    }*/

  }
  return 0;
}

int8_t alignWall(void){
  rotateTimerCount = 1000;
  while((rotateTimerCount != 0)){
    updateSensors();
    delayMs(25);
    totalWall = getTotalWall();
    if(sensors[SenBumpDrop]) {
      rotate(V);
    }
    else if(!(sensors[SenBumpDrop]) && totalWall > refPoint){
      return 2;
    }
    /*else if(sensors[SenIRChar] > 241 && sensors[SenIRChar] < 255) {
      clearPIDVar();
      stopCreate();
      return 3;
    }*/

    else{ return 0; }
  }
  return 0;
}

void clearPIDVar(void){
  errorTerm = 0;
  errorTermPrev = 0;
  sumOfError = 0;
  controlOut = 0;
}

int8_t pid(void){
  int16_t sensor = getTotalWall();
  while(1+1==2){
	  updateSensors();
	  delayMs(25);
	  sensor = getTotalWall();
	  if(pidTimerCount == 0) {
	    errorTerm = sensor - refPoint;
	    //poportional Term
	    controlOut = (int16_t)(pTerm * errorTerm);
	    //Integral Anti-Windup check
	    if(sumOfError > AntiWToleranceLow || sumOfError < AntiWToleranceHigh){
	      //integral Term
	      sumOfError += errorTerm;
	      controlOut += (int16_t)(dt * sumOfError) >> iTerm;
	    }
	    //drivative term
	    controlOut += (int16_t)(dTerm * (errorTerm - errorTermPrev)/dt);
	    controlOut = controlOut >> cGain;
	    errorTermPrev = errorTerm;
	    
	    //reset timer
	    pidTimerCount = dt*1000; //dt is in Seconds thus a convertion is needed
	  }
	  //
	  velocityLeft = V - controlOut;
	  velocityRight = V + controlOut;
	  driveCreate(velocityRight, velocityLeft);//(vr, vl)
	  if(sensors[SenBumpDrop]){  clearPIDVar(); stopCreate(); return 1; }
	  if(sensors[SenIRChar] == 242 || sensors[SenIRChar] == 246 || sensors[SenIRChar] == 250 || sensors[SenIRChar] == 254) {
	    clearPIDVar();
	    stopCreate();
	    return 3;
	  }
	}
}

int8_t pidField(void){
  int16_t sensor = sensors[SenIRChar];
  while(1+1==2){
	  updateSensors();
	  delayMs(25);
	  sensor = sensors[SenIRChar];
            if(sensor == 254 || sensor == 252 || sensor == 246 || sensor == 244){
              robotLedsOn();
              return 1;
            }
	    //Inside the Field            
	    else if(sensor == 242 || sensor == 250 || sensor == 246){
              errorTerm = bouyError;
              robotRightLedOn();
            }
            //Outside the field
            else if(sensor == 255 || sensor == 244 || sensor == 248){
              errorTerm = -bouyError;
              robotLeftLedOn();

            }
            //Inside the Field and Both Bouys
            else{
              robotLedsOff();
              return 0;
            }
          if(pidTimerCount == 0) {
	    //poportional Term
	    controlOut = (int16_t)(fieldPTerm * errorTerm);
	    //Integral Anti-Windup check
	    if(!((velocityLeft < 0 || velocityLeft > (medV*2)) && (velocityRight < 0 || velocityRight > (medV)))){
	      //integral Term
	      sumOfError += errorTerm;
	      controlOut += (int16_t)(dt * sumOfError) >> fieldITerm;
	    }
	    //drivative term
	    controlOut += (int16_t)(fieldDTerm * (errorTerm - errorTermPrev)/dt);
	    controlOut = controlOut >> fieldCGain;
	    errorTermPrev = errorTerm;
	    
	    //reset timer
	    pidTimerCount = dt*1000; //dt is in Seconds thus a convertion is needed
	  } 
	  //
	  velocityLeft = medV - controlOut;
	  velocityRight = medV + controlOut;
	  driveCreate(velocityRight, velocityLeft);//(vr, vl)
	  if(sensors[SenBumpDrop]){
	    clearPIDVar();
	    stopCreate();
	    return 0;
	  }
  }
}


int8_t pidDock(void){
  int16_t sensor = sensors[SenIRChar];
  while(1+1==2){
	  updateSensors();
	  delayMs(25);
	  sensor = sensors[SenIRChar];
	  if(pidTimerCount == 0) {
            //Red sensor is picked up
	    if(sensor == 248 || sensor == 250){
              errorTerm = bouyError;
              robotRightLedOn();
              }
            //Green sensor is picked up
            else if(sensor == 244 || sensor == 246){ 
              errorTerm = -bouyError;
              robotLeftLedOn();
              }
            //Both sensors are picked up
            else if(sensor == 252 || sensor == 254){ 
              errorTerm = 0;
              robotLedsOn();
              }
            else{
              errorTerm = 0;
              robotLedsOn();
            }
	    //poportional Term
	    controlOut = (int16_t)(dockPTerm * errorTerm);
	    //Integral Anti-Windup check
	    if(!((velocityLeft < 0 || velocityLeft > (lilV*2)) && (velocityRight < 0 || velocityRight > (lilV)))){
	      //integral Term
	      sumOfError += errorTerm;
	      controlOut += (int16_t)(dt * sumOfError) >> dockITerm;
	    }
	    //drivative term
	    controlOut += (int16_t)(dockDTerm * (errorTerm - errorTermPrev)/dt);
	    controlOut = controlOut >> dockCGain;
	    errorTermPrev = errorTerm;
	    
	    //reset timer
	    pidTimerCount = dt*1000; //dt is in Seconds thus a convertion is needed
	  } 
	  //
	  velocityLeft = lilV - controlOut;
	  velocityRight = lilV + controlOut;
	  driveCreate(velocityRight, velocityLeft);//(vr, vl)
	  if(sensors[SenBumpDrop]){
	    clearPIDVar();
	    stopCreate();
            delayMs(500);
            updateSensors();
	    delayMs(25);
	    homeBase = sensors[SenChAvailable];
	    if(homeBase == 2) {
	      return 4;
	    } 
	    else {
	      driveCreate(~lilV, ~lilV+5);
	      delayMs(6000);
	    }
	  }
  }
}

int8_t runDockOver(void) {
  while(1+1 == 2){
    if(pidField()){
      stopCreate();
      rotate(~V);
      delayMs(1000);
      return pidDock();
    }
  }
}
