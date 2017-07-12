#include "arduino.h"
#include "tunes.h"


tunes::tunes(int buzzerPin){
_buzzPin = buzzerPin;
}

void tunes::Silence(){noTone(_buzzPin);}

void tunes::makePattern(int lowPitch, int highPitch, float upSpeed, float dnSpeed, int numOfWobs, boolean startUp, boolean endUp){
   _lowPitch = lowPitch;
   _highPitch = highPitch;
   _upSpeed = (highPitch - lowPitch)/ (upSpeed * 1000);
 
   _dnSpeed = (highPitch - lowPitch) / (dnSpeed * 1000);
   _numOfWobs = numOfWobs;
   _wobCounter = 0;
   _isDone = false;
   _lastUpdateTime = 0;
   if (startUp){
      _moveUp = true;
      _curPitch = _lowPitch;
   } else {
      _moveUp = false;
      _curPitch = _highPitch;
   }
   _endUp = endUp;
}

void tunes::Update(){
  if (_lastUpdateTime == 0){_lastUpdateTime = millis();}
  if (!_isDone){
    int msPassed = millis() - _lastUpdateTime;
    if (_moveUp){
      _curPitch = constrain(_curPitch + msPassed * _upSpeed, _lowPitch, _highPitch);
      if (_curPitch == _highPitch) {
          _moveUp = false;
          if (_endUp){_wobCounter++;}
      }
    } else {
      _curPitch = constrain(_curPitch - msPassed * _dnSpeed, _lowPitch, _highPitch);
      if (_curPitch == _lowPitch) {
        _moveUp = true;
        if (!_endUp){_wobCounter++;}
      }
    }
    if (_wobCounter == _numOfWobs){_isDone = true;}
    tone(_buzzPin, _curPitch);
    _lastUpdateTime = millis();
  }
}
