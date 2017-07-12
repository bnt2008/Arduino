#include <pinglib.h>
#include <Servo.h>
#include <Wire.h>

pinglib myPing;


byte _curCommand;
boolean _newCommand;
boolean _readyToMove = false;
byte _moveDir;
byte _commandAngle;
byte _comMask = B11111000; //reserves three bits for command

//Flags for preparing a move
byte _moveStage = 0; //What stage of the move are we at 
boolean _turnedOnce = false;
unsigned long _moveStart;
int _turnTO = 2000;
boolean _rotComplete = false;
boolean _rotStart = false;
int _distThresh = 20;

//Status flags from base
boolean _balanced;
boolean _motorRunning;

//Commands for the ears
byte _fromEars = 0;
byte _toEars = 0;

//Deterine the length of move
int _moveLen = 10000; //move for 10s
unsigned long _moveTime; // when the move started

void setup() {
  // put your setup code here, to run once:
  myPing.setUp(4, 2, 5, 3, 13);
  Serial.begin(9600);
  Wire.begin(2);
  Wire.onRequest(sendToEars);
  Wire.onReceive(getFromEars);
}

void loop() {

decideCommand();
getByte();

    switch(_curCommand){
      case 1: // sit and wait; stop moving, run the red pulsing lights
         if (_newCommand){
            Serial.write(0);
            _newCommand = false;
         } 
         myPing.findDistance(2);
      break;
      case 2: // Executing  move
          executeMove();
      break;
    }      
        
      
}


byte moveAngle(){ // Decides which direction the robot should go when it is standing still
      //considers whether there is something immediately in the direction, a random element and closeness of objects in adjacent directions
      int distThresh = 40;  //min distance required for the robot to consider the direction
      int defaultTurnWeight = 50; //default weight applied the to left and rightmost directions
      double Kf = .3;  //these will wieght the contributions of the different readings, after Ks is multiplied by 2, these should add to 1
      double Ks = .3;
      double Kr = .1;
      boolean canGo[5];
      boolean hasADir = false;
      int adjacentFact;
      int leftWeight, rightWeight, frontWeight, finalWeight, randWeight;
      int curMax = -1;
      int maxWeight = 0;
      // Check the 5 direction points, and ask, can I go anywhere? is any individual point clear and apply a randomized factor
      for (int i = 0; i < 5; i++){
         myPing.findDistance(i);
        
         if (myPing.getDistance(i) > distThresh) {
            canGo[i] = true;
            hasADir = true;
         } else {
          canGo[i] = false;
         }
      }
      myPing.findDistance(2);
      if (!hasADir) {return -1;} //escape out if there is 
      
      //apply weights for adjacent obstacles.
      for (int i = 0; i < 5; i++){
         if (i == 0){leftWeight = defaultTurnWeight;}
         if (i == 5){rightWeight = defaultTurnWeight;}
         if (i > 0){leftWeight = map(myPing.getDistance(i - 1), 0, 500, 100, 0);}
         if (i < 5){rightWeight = map(myPing.getDistance(i + 1), 0, 500, 100, 0);}
         frontWeight = map(myPing.getDistance(i), 0, 500, 100, 0);
         randWeight = random(0, 100);
         finalWeight = canGo[i]   * (  (Kf * frontWeight) + (Ks * leftWeight) + (Ks * rightWeight) + (Kr * randWeight)  );
         if (finalWeight > maxWeight) {
            maxWeight = finalWeight;
            curMax = i;
         }
         if (finalWeight == maxWeight && random(1,2) == 2){  //resolve ties with a coin-flip
            maxWeight = finalWeight;
            curMax = i;
         }
      }
    return(curMax);
  
}
void executeMove(){
    switch(_moveStage){
      case 1: //plan out the move. If there is no where to go, then do a 180 turn
        {
        byte chooseAngle = moveAngle();
        if (chooseAngle == -1){
          if (_turnedOnce){
            _moveStage = 10;
          } else {
            _commandAngle = 180;
            _moveStage = 11;
          }
         } else {
            _commandAngle = myPing.getBearing(chooseAngle);
            _moveStage = 2;
         }
  
        }break;
 
       case 2: //write the command to the base
          turnTo(_commandAngle);
         _moveStage = 3;
         _moveStart = millis();
       break;
       
      case 3: //check to see if rotation is complete
        if (_rotComplete) {
            _moveStage = 4;
            _rotComplete = false;
         } else {
           if (millis()  > _turnTO + _moveStart){_moveStage = 4;}
         }
      break;
      case 4: //begin the move send a byte to say angle = 0, other byte to set speed and say go
         driveCommand(255, myPing.getBearing(2));
         _moveStage = 5;
         _moveTime = millis();
      break;
      case 5: //driving
         myPing.findDistance(2); //read the distance straight ahead
         if (myPing.getDistance(2) < _distThresh) {  //If we are too close to something, let's stop (we can make this more complex later
             byte outByte = 0;
             Serial.write(outByte);
             _moveStage = 6;
             _moveStart = millis();
 
         }
         if (millis() - _moveTime > _moveLen){ //also give it a chance to stop on its own
            byte outByte = 0;
            Serial.write(outByte);
 
            _moveStage = 6;
            _moveStart = millis();
   

         }
       break;
       case 6: // Consider turning around; for now, just pick a random angle between 120 and 250
         if (millis() > _moveStart + 1000){
             turnTo(random(120, 250));
            _moveStart = millis();
            _moveStage = 7;
  

         }
       break;
       case 7: // finish up
          if (_rotComplete) {
            _moveStage = 0;
            _rotComplete = false;
         } else {
           if (millis() - _moveStart > _turnTO){_moveStage = 0;}
         }
      break;
      case 10: //failed
       _curCommand = 0;
       _moveStage = 0;
      break;
      case 11: //execute a 180 degree turn
          turnTo(180);
         _moveStart = millis();
         _moveStage = 12;
         _turnedOnce = true;
      break; 
      case 12: //finish the 180 degree turn
         if (_rotComplete) {
            _moveStage = 1;
            _rotComplete = false;
         } else {
           if (millis() - _moveStart > _turnTO){_moveStage = 1;}
         }
       break;
      
    }
 } // end of moving program



void decideCommand(){
  if (_moveStage > 0){
    _curCommand = 2;
  } else {
    switch (_fromEars){
      case 1: 
          {
          _moveStage = 1;
          _turnedOnce = false;
          _curCommand = 2;
          }
       break;
       default:
         _curCommand = 0;
       break;
    }  
  }
}

void turnTo(byte angle){ //sends a rotate command to the base
  byte comByte;
  byte outAngle;
  byte midAngle = myPing.getBearing(2); 
  if (angle > midAngle){
    outAngle = angle - midAngle;
    comByte = 4;
    outAngle = outAngle & _comMask;
    outAngle = outAngle + comByte;
    Serial.write(outAngle);
  } 
  if (angle == midAngle){
    _rotComplete = true;
  }
  if (angle < midAngle){
    comByte = 5;
    outAngle = midAngle - angle;
    outAngle = outAngle & _comMask;
    outAngle = outAngle + comByte;
    Serial.write(outAngle);
  }
  
 
}

void driveCommand(byte comSpeed, byte angle){
 //1st send a direction byte
    byte comByte;
    byte outAngle;
    byte midAngle = myPing.getBearing(2); 
  if (angle >= midAngle){
    outAngle = angle - midAngle;
    comByte = 2;
    outAngle = outAngle & _comMask;
    outAngle = outAngle + comByte;
  } 
  if (angle < midAngle){
    comByte = 3;
    outAngle = midAngle - angle;
    outAngle = outAngle & _comMask;
    outAngle = outAngle + comByte;
  }
  Serial.write(outAngle);

  //Now send out the speed;
  comSpeed = comSpeed & _comMask;
  comSpeed = comSpeed + 1;
  Serial.write(comSpeed);
 
}

void getByte(){
  byte curByte = 0;
  boolean rotating;
  while (Serial.available()){
    curByte = Serial.read();
    //Set the balance flag
    _balanced = curByte & B00000001;

    //Set the motor running flag
    _motorRunning = (curByte & B01000000) >> 7;
    //Check to see if we are rotating
    rotating = (curByte & B00100000) >> 6;
    if (!_rotStart && rotating){_rotStart = true;}
    if (_rotStart && !rotating){
       _rotStart = false;
       _rotComplete = true;
    }
  }
}


//If we are just sitting, send 0, otherwise give the move stage
void sendToEars(){
  if (_curCommand == 2){
    _toEars = _moveStage;     
  } else {
    _toEars = 0;
  }
  
  Wire.write(_toEars);
}


//Get the command from the ears -- for now is 1 or 0
void getFromEars(int rBytes){
  _fromEars = Wire.read();
}



