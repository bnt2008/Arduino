#include <PID_v1.h>

#include <yawAndPitch.h>
#include <Wire.h>
#include <motorControl.h>


#include <IRLibRecvPCI.h>   
#include <IRLibDecodeBase.h>  
#include <IRLib_HashRaw.h>  
#include <IRLibCombo.h>
  
/////////////////////
///Set up the PID ///
/////////////////////
double curDir, curRatio, commandDir, curPitch, curBalOut, commandPitch;
const double dirKp = 15;
const double dirKi = 3;
const double dirKd = 2;
const double balKp = 50;
const double balKi = 10;
const  double balKd = 0;
const int _fellOverAngle = 20; //more than this angle requires recovery program
// For communicating balance
boolean _unBalanced = false;
unsigned long lastUnBalanced;
int recoveryTime = 1000; //time in millisecs. robot must be balanced to clear the unbalanced flag

// Holds the commands recieved
int _curCommand = 0;
int _curComSpeed = 0;
double  _curComDir = 0; 

//for rotation
boolean _rotating = false;
const int rotThresh = 10; // number of degrees in which to accept that rotation should be done
unsigned long _lastOffTarget;

motorControl mainMotor;
yawAndPitch myMPU; 
PID dirPID(&curDir, &curRatio, &_curComDir, dirKp, dirKi, dirKd, DIRECT);
PID balPID(&curPitch, &curBalOut, &commandPitch, balKp, balKi, balKd, DIRECT);

/////////////////////////
//IR Decoder/////////////
/////////////////////////

IRrecvPCI myReceiver(2); //Create the receiver. Use pin 2    
IRdecode myDecoder; //create decoder 



void setup() {
   myReceiver.enableIRIn(); // Start the receiver  
   mainMotor.setUp(10,9,11,5,4,6);
   myMPU.setUp(0x68);
   myMPU.findOffsets(50);
  myMPU.switchYaw(false);  //make sure forward and to the right are positive
  myMPU.switchPitch(true);
  myMPU.switchPitchGyro(false);
  myMPU.setWeight(.85);
  myMPU.useDelay(20);
  commandPitch = -6; //myMPU.Pitch();
   curDir = 0;
   curRatio = 0;
   dirPID.SetMode(AUTOMATIC);
   dirPID.SetOutputLimits(-255,255);
   dirPID.SetSampleTime(30);
   balPID.SetMode(AUTOMATIC);
   balPID.SetOutputLimits(-255, 255);
   balPID.SetControllerDirection(REVERSE);
   balPID.SetSampleTime(30);
   Serial.begin(9600);
}

void loop() {

  communicate();
  getIR();
  myMPU.Update();
  balance();
  curDir = myMPU.Yaw();
  dirPID.Compute();
  
  switch (_curCommand){
    case 0: //do nothing
   
      break;
    case 1: //go forward
      mainMotor.drive(_curComSpeed, curRatio);
      break;
    case 3: //rotate
     
      if (curDir < _curComDir - rotThresh || curDir > _curComDir + rotThresh){_lastOffTarget = millis();}
      if (millis() > _lastOffTarget + recoveryTime) {
          _curCommand = 0;
          _curComSpeed = 0;
          _rotating = false;
          mainMotor.stop(255);
      } else {
          mainMotor.rotate(curRatio);
      }
      break;
  }
}

void communicate(){
    byte recieveByte;
    byte mask = 7;
    boolean recieved = false;
    byte comType, comValue;
    boolean recStop = false;
    boolean recGo = false;
    boolean recTurn = false;
    
    while (Serial.available()){
      recieveByte = Serial.read();
      comType = mask & recieveByte;
      comValue = recieveByte - comType;
      switch (comType){
          case 0: // indicates that the command is to stop  (last 3 bits 000)
            _curCommand = 0;
            _curComSpeed = 0;
            recStop = true;
            break;
          case 1: // 1 indicates that the commad is to go in whatever direction is in curComDir at the indicated speed (last 3 bits 001)
            if (_curCommand == 0) {myMPU.resetYaw();} // If this is a new command, reset the Yaw to 0
            _curCommand = 1;
            _curComSpeed = comValue;
            recGo = true;
            break;
          case 2: //2 indicates that the command is a leftward bearing it will not start the robot moving (last 3 bits 010)
            myMPU.resetYaw(); // Changes is direction will always be interpreted as being from where the robot is at the time the command is recieved
            _curComDir = -1 * comValue;
            recTurn = true;
            break;
          case 3: //3 indicates that the command is a rightward bearing (last 3 bits 011)
            myMPU.resetYaw();
            _curComDir = comValue;
            recTurn = true;
            break;
          case 4: //indicates rotate to left (last 3 bits 100)
            myMPU.resetYaw();
            _curCommand = 3;
            _curComDir = comValue * -1;
            _rotating = true;
            _lastOffTarget = millis();
            break;
          case 5: //indicates rotate to right (last 3 bits 101)
            myMPU.resetYaw();
            _curCommand = 3;
            _curComDir = comValue;
            _rotating = true;
            _lastOffTarget = millis();
            break;
      }
      recieved = true;
    }
// Now send out the byte to indicate status
/* Byte map (1st bit is least sig):
 *  1: Balance (0 = balanced)
 *  2: recieved command (1 = recieved command)
 *  3: recieved a stop (1)
 *  4: recieved a go (1)
 *  5: recieved a turn(1)
 *  6: rotating (1 is performing rotation)
 *  7: Is any motor moving? (1 yes, 0 no)
 *  8: not yet used
 */
    byte byteToSend1 = 0;    
    byteToSend1 = byteToSend1 | _unBalanced;
    if (recieved) {byteToSend1 = byteToSend1 | B00000010;}  
    if (recStop) {byteToSend1 = byteToSend1  | B00000100;}
    if (recGo)   {byteToSend1 = byteToSend1  | B00001000;}
    if (recTurn) {byteToSend1 = byteToSend1  | B00010000;}
    if (_rotating){byteToSend1 = byteToSend1 | B00100000;}
 //   Serial.println(mainMotor.isMoving());
    if (mainMotor.isMoving()) {byteToSend1 = byteToSend1 | B01000000;}
    Serial.write(byteToSend1);
}

  


void balance(){
  myMPU.Update();
  curPitch = myMPU.Pitch();
  
  if (curPitch > commandPitch){
    if (curPitch > _fellOverAngle){
      mainMotor.drive(255, 0);
      delay(200);
    }
    balPID.Compute();
    mainMotor.drive(curBalOut, 0);
    lastUnBalanced = millis();
    _unBalanced = true;
  } else {
    if (_curCommand == 0) {mainMotor.drive(0,0);};
    if (millis() - lastUnBalanced > recoveryTime){_unBalanced = false;}
  }
}

///////////////////////////////////////////////////
/// Get any recieved IR codes and translate into///
/// an action command                           ///
///////////////////////////////////////////////////

void getIR(){
    int option = 0; 
    if (myReceiver.getResults()) {  
        myDecoder.decode();           //Decode it  
        uint32_t result = myDecoder.value;  //the results of a Now print results. 
        switch (result){
          case 2331063592: // button
           _curCommand = 0;
           _curComSpeed = 0;
           
           break;
          case 3261853764: //Up Button
            _curCommand = 1;
            _curComDir = 0;
            myMPU.resetYaw();
            break;
          case 3305092678: //Dn Button
            myMPU.findOffsets(50);
            break;
          case 1972149634: // Left button
            _curComDir = _curComDir - 5;
           
            break;
          case 1400905448: //right button
            _curComDir = _curComDir + 5;
         
            break;
          case 3778927144: //#1
            _curComSpeed = 120;
           break;
          case 2908251746: //#2
            _curComSpeed = 190;
           break;
          case 657459652: //#3
            myMPU.resetYaw();
            _curComDir = -90;
            _curCommand = 3;
            _lastOffTarget = millis();
            break;
          case 4120482440:  //#4
            myMPU.resetYaw();
            _curComDir = 90;
            _curCommand = 3;
            _lastOffTarget =  millis();
            break;
          case 1931099650: //#5
            myMPU.resetYaw();
            _curComDir = 180;
            _curCommand = 3;
            _lastOffTarget = millis();
            break;
        }   
    myReceiver.enableIRIn();      //Restart receiver   } 
    } 
 
}

