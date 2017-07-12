#include <soundProcess.h>
#include <RobotLEDControl.h>
#include <Wire.h>

soundProcess mySound(A1, A2);
RobotLEDControl myLED(11, 10, 9);


//for sound detection
float _sBuffer[7];
unsigned long _bufStart;
byte _bufPos = 0;
const byte _numToCalc = 3;
const int _timeLen = 1000;

//for communication
byte _sCommand = 0;
byte _rCommand = 0;
byte _curCommand = 0;

void setup() {

  Wire.begin();
  myLED.Start();
  Serial.begin(9600);
}

void loop() {
    myLED.Update();
    
    switch (_curCommand){
      case 0:
        {
            mySound.captureSound();
            
            boolean isSound = mySound.getPeaks();
            
            //Put the sound in the buffer
            if (isSound){
              if (_bufPos == 0) {_bufStart = millis();}
              mySound.getFreq();
              float curSound = mySound.reportFreq();
              if (curSound > 550){
                _sBuffer[_bufPos] = mySound.reportFreq(); //need to filter the interference from PWM
                _bufPos++;  
              } 
            }
           Serial.println(mySound.reportFreq());
            
            //reset if it's been too long
            if (_bufPos > 0 && millis() - _bufStart > _timeLen) { _bufPos = 0; }
            
            //check to see if we have a command in the buffer
            if (checkBuffer()){
                _sCommand = 1;
                _curCommand = 0;
                _bufPos = 0;
                }
            
            //reset if at end of buffer
            if (_bufPos == 7){_bufPos = 0;}
            
        }
        break;
        default:
        break;
    }
    

   //Communicate with the brain
   Serial.print(_sCommand);
   Wire.beginTransmission(2);
   Wire.write(_sCommand);
   Wire.endTransmission(2);
   _sCommand = 0;
   Wire.requestFrom(2, 1);
   _rCommand = Wire.read();
   
   Serial.print("|");
   Serial.println(_rCommand);
   if (_curCommand != _rCommand){setPattern();}; //if issued a new command, change the pattern   
   _curCommand = _rCommand; //For now it will be the same, but some day, we may change this

}


//checks the sound buffer to see if there is a command
boolean checkBuffer(){
  boolean answer = false;
  if (_bufPos >= _numToCalc){
    byte counter = 0;
    for (int i = 0; i < _bufPos; i++){
      if (_sBuffer[i] > 800 && _sBuffer[i] < 2000){counter++;}
    }
    if (counter > _numToCalc) {answer = true;}
  }
  return (answer);
}

void setPattern(){
  switch(_rCommand){
      case 0: //robot is listening
         myLED.setPattern(0); // Default pulsing red
         myLED.Start();
      break;
      case 1: //robot is planning a move
        myLED.setPattern(4); //Solid blue
        myLED.Start();
      break;
      case 5: // robot is driving
        myLED.setPattern(2); //plusing green
        myLED.Start();
      break; 
  }
}


