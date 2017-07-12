#include "Arduino.h"
#include "pinglib.h"
#include "Servo.h"

pinglib::pinglib(){};

int pinglib::getBearing(int a){
 return _bearings[a];
}
int pinglib::getDistance(int angle){
   return(_distances[angle]);
}

int pinglib::getChange(int angle){
   return(_changes[angle]);
}
void pinglib::setBearings(int a, int b, int c, int d, int e){
_bearings[0] = a;
_bearings[1] = b;
_bearings[2] = c;
_bearings[3] = d;
_bearings[4] = e;

for (int i = 0; i < 5; i++){
  _changes[i] = 0;
  _distances[i] = -1;
}
}

void pinglib::setUp(int tL, int eL, int tR, int eR, int ServoPin){
  _trigPinL = tL;
  _trigPinR = tR;
  _echoPinL = eL;
  _echoPinR = eR;
  _servoPin = ServoPin;
  setBearings(50,75,95,115,140);
  _changeTolerance = .2;
  pinMode(_trigPinL, OUTPUT);
  pinMode(_trigPinR, OUTPUT);
  pinMode(_echoPinL, INPUT);
  pinMode(_echoPinR, INPUT);

  headServo.attach(_servoPin);
  delay(50);
  _curAngle = _bearings[2];
  headServo.write(_curAngle);
  delay(300);
  headServo.detach();
}

void pinglib::findDistance(byte angle){
  int useDur;
  if (_curAngle != _bearings[angle]){
  
     headServo.attach(_servoPin);
     delay(15);
     _curAngle = _bearings[angle];
     headServo.write(_curAngle);
     delay(400);
     headServo.detach();
  }
  
  digitalWrite(_trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPinR, HIGH);
  delayMicroseconds(100);
  digitalWrite(_trigPinR, LOW);

  int rDuration = pulseIn(_echoPinR, HIGH);

  digitalWrite(_trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPinL, HIGH);
  delayMicroseconds(100);
  digitalWrite(_trigPinL, LOW);
  int lDuration = pulseIn(_echoPinL, HIGH);

  _lastUpdates[angle] = millis();

  //If both sensors have a reading, use the smaller value
  if (rDuration > 0 && lDuration > 0){
      if (rDuration > 0 && rDuration < lDuration) {useDur = rDuration;}
      if (lDuration > 0 && lDuration < rDuration) {useDur = lDuration;}
  }

  //if only one sensor has a reading,use that one
  if (rDuration > 0 && lDuration < 600) {useDur = rDuration;};
  if (lDuration > 0 && rDuration < 600) {useDur = lDuration;};
  
  //If neither sensor has a reading, send error code
  if (lDuration < 600 && rDuration <  600){
    if (_distances[angle] != -1) {_changes[angle] = 1;}  else {_changes[angle] = 0;}
    _distances[angle] = -1;
    return;
  }
  
  int dist = .5 * useDur/29;
  _changes[angle] = 0;
  
  if (dist > _changeTolerance * _distances[angle] + _distances[angle]){_changes[angle] = 3;}
  if (dist < _distances[angle] - _changeTolerance * _distances[angle]){_changes[angle] = 4;}
  if (_distances[angle] == -1){_changes[angle] = 2;}//set to object appeared
  _distances[angle] = dist;
}