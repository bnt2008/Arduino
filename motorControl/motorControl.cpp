// our convention will be that + numbers generally mean forward and right, negative numbers are revese and left
#include "Arduino.h"
#include "motorControl.h"

motorControl::motorControl(){};

void motorControl::setUp(int M1F, int M1R, int M1S, int M2F, int M2R, int M2S){
  pinMode(M1F, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2R, OUTPUT);
  
  _mLF = M1F;
  _mLR = M1R;
  _mLS = M1S;
  
  _mRF = M2F;
  _mRR = M2R;
  _mRS = M2S;
  
  digitalWrite(_mLF, LOW);
  digitalWrite(_mLR, LOW);
  digitalWrite(_mRF, LOW);
  digitalWrite(_mRR, LOW);
  
}

boolean motorControl::isMoving(){
boolean answer;
if (_lSpeed > 0 || _rSpeed > 0){answer = true;} else {answer = false;}
return answer;
}
void motorControl::drive(int speed, int ratioLR){
  boolean forward;
  int lSpeed, rSpeed;
  
  if (speed > 0){
     forward = true;
  } else if (speed < 0){
     forward = false;
     speed = -1 * speed;
  } else {
    digitalWrite(_mLF, LOW);
    digitalWrite(_mLR, LOW);
    digitalWrite(_mRF, LOW);
    digitalWrite(_mRR, LOW);
    _lSpeed = 0;
    _rSpeed = 0;
    return;
  }
  
  if (ratioLR > 0){
      lSpeed = speed;
      rSpeed = map(ratioLR, 255, 0, 0, speed);  
  } else if (ratioLR < 0){
      rSpeed = speed;
      lSpeed = map(ratioLR, -255, 0, 0, speed);
  } else{
      rSpeed = speed;
      lSpeed = speed;
  }
  
  analogWrite(_mLS, lSpeed);
  analogWrite(_mRS, rSpeed);
  if (forward){
     digitalWrite(_mLF, HIGH);
     digitalWrite(_mRF, HIGH);
     digitalWrite(_mLR, LOW);
     digitalWrite(_mRR, LOW);
   } else {
     digitalWrite(_mLR, HIGH);
     digitalWrite(_mRR, HIGH);
     digitalWrite(_mLF, LOW);
     digitalWrite(_mRF, LOW);
   }
   _lSpeed = lSpeed;
   _rSpeed = rSpeed;
}

void motorControl::rotate(int Rspeed){
  int useSpeed;
  if (Rspeed < 0){ useSpeed = Rspeed * -1;} else {useSpeed = Rspeed;}
  analogWrite(_mLS, useSpeed);
  analogWrite(_mRS, useSpeed);
  if (Rspeed > 0){
    digitalWrite(_mLF, HIGH);
    digitalWrite(_mLR, LOW);
    digitalWrite(_mRF, LOW);
    digitalWrite(_mRR, HIGH);
  } else if (Rspeed < 0){
    digitalWrite(_mLF, LOW);
    digitalWrite(_mLR, HIGH);
    digitalWrite(_mRF, HIGH);
    digitalWrite(_mRR, LOW);
  } else {
    digitalWrite(_mLF, LOW);
    digitalWrite(_mLR, LOW);
    digitalWrite(_mRF, LOW);
    digitalWrite(_mRR, LOW);
  }
  _lSpeed = useSpeed;
  _rSpeed = useSpeed;
}

void motorControl::stop(int dropBy){
  
  while (_rSpeed > 0 || _lSpeed > 0){
     if (_rSpeed > dropBy){_rSpeed = _rSpeed - dropBy;} else {_rSpeed = 0;}
     if (_lSpeed > dropBy){_lSpeed = _lSpeed - dropBy;} else {_lSpeed = 0;}
     analogWrite(_mLS, _lSpeed);
     analogWrite(_mRS, _rSpeed);
     delay(30);
  }
     digitalWrite(_mLF, LOW);
     digitalWrite(_mLR, LOW);
     digitalWrite(_mRF, LOW);
     digitalWrite(_mRR, LOW);
}