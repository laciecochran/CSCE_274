/* oi.h
 *
 * Definitions for the Open Interface
 */


// Command values
#define CmdStart        128
#define CmdBaud         129
#define CmdControl      130
#define CmdSafe         131
#define CmdFull         132
#define CmdSpot         134
#define CmdClean        135
#define CmdDemo         136
#define CmdDrive        137
#define CmdMotors       138
#define CmdLeds         139
#define CmdSong         140
#define CmdPlay         141
#define CmdSensors      142
#define CmdDock         143
#define CmdPWMMotors    144
#define CmdDriveWheels  145
#define CmdOutputs      147
#define CmdSensorList   149
#define CmdIRChar       151


// CSCE 274 students - This ``could'' be useful if we ever decide 
//                     to use the serial interrupt. 
//      Short version: Long array to store values, these constants are 
//                     the indices for specific sensor values.
// Sensor byte indices - offsets in packets 0, 5 and 6
#define SenBumpDrop     0            
#define SenWall         1
#define SenCliffL       2
#define SenCliffFL      3
#define SenCliffFR      4
#define SenCliffR       5
#define SenVWall        6
#define SenOverC        7
#define SenIRChar       10
#define SenButton       11
#define SenDist1        12
#define SenDist0        13
#define SenAng1         14
#define SenAng0         15
#define SenChargeState  16
#define SenVolt1        17
#define SenVolt0        18
#define SenCurr1        19
#define SenCurr0        20
#define SenTemp         21
#define SenCharge1      22
#define SenCharge0      23
#define SenCap1         24
#define SenCap0         25
#define SenWallSig1     26
#define SenWallSig0     27
#define SenCliffLSig1   28
#define SenCliffLSig0   29
#define SenCliffFLSig1  30
#define SenCliffFLSig0  31
#define SenCliffFRSig1  32
#define SenCliffFRSig0  33
#define SenCliffRSig1   34
#define SenCliffRSig0   35
#define SenInputs       36
#define SenAInput1      37
#define SenAInput0      38
#define SenChAvailable  39
#define SenOIMode       40
#define SenOISong       41
#define SenOISongPlay   42
#define SenStreamPckts  43
#define SenVel1         44
#define SenVel0         45
#define SenRad1         46
#define SenRad0         47
#define SenVelR1        48
#define SenVelR0        49
#define SenVelL1        50
#define SenVelL0        51


// Sensor packet sizes
#define Sen0Size        26
#define Sen1Size        10
#define Sen2Size        6
#define Sen3Size        10
#define Sen4Size        14
#define Sen5Size        12
#define Sen6Size        52

// Baud codes
#define Baud300         0
#define Baud600         1
#define Baud1200        2
#define Baud2400        3
#define Baud4800        4
#define Baud9600        5
#define Baud14400       6
#define Baud19200       7
#define Baud28800       8
#define Baud38400       9
#define Baud57600       10
#define Baud115200      11

// Baud UBRRx values
#define Ubrr300         3839
#define Ubrr600         1919
#define Ubrr1200        959
#define Ubrr2400        479
#define Ubrr4800        239
#define Ubrr9600        119
#define Ubrr14400       79
#define Ubrr19200       59
#define Ubrr28800       39
#define Ubrr38400       29
#define Ubrr57600       19
#define Ubrr115200      9


// Command Module button (black button beside reset)
#define UserButton        0x10
#define UserButtonPressed (!(PIND & UserButton))

// Create Port
#define RobotPwrToggle      0x80
#define RobotPwrToggleHigh (PORTD |= 0x80)
#define RobotPwrToggleLow  (PORTD &= ~0x80)

#define RobotPowerSense    0x20
#define RobotIsOn          (PINB & RobotPowerSense)
#define RobotIsOff         !(PINB & RobotPowerSense)
