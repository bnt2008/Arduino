#include "Arduino.h"
#include "RobotLEDControl.h"

RobotLEDControl::RobotLEDControl(int rPin, int gPin, int bPin)
/* Call to function includes pins and the time-scale; That is the amount of time in millis between colors given 11 (3) as last pair of bits
 *  
 */
{
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  _redScale = 255; 
  _blueScale = 255;
  _greenScale = 255;
  setPattern(0);
};

void RobotLEDControl::setPattern(byte patternNo){
	switch(patternNo){
		case 0: //default -- red 
		  _numOfPats = 2;
		  _pats[0] = B11000011;
		  _pats[1] = B00000011;
		  _tScale = 2000;
		break;
		case 1: //blue
		  _numOfPats = 2;
		  _pats[0] = B00001111;
		  _pats[1] = B00000011;
		  _tScale = 2000;
 		break;
		case 2: //Green
		  _numOfPats = 2;
		  _pats[0] = B00110011;
		  _pats[1] = B00000011;
		  _tScale = 2000;
		break;
		case 3: //solid red
		  _numOfPats = 1;
		  _pats[0] = B11000011;
 		break;
		case 4: //solid blue
		  _numOfPats = 1;
		  _pats[0] = B00001111;
 		break;
		case 5: //solid green
		  _numOfPats = 1;
		  _pats[0] = B00110011;
 		break;
		case 6: //White
		  _numOfPats = 2;
		  _pats[0] = 11111111;
		  _pats[1] = 00000011;
		break;
	}

}

void RobotLEDControl::whiteBalance(byte R, byte G, byte B){
	_redScale = R; 
	_blueScale = G;
	_greenScale = B;
}
void RobotLEDControl::Start()
{
  _lastUpdateTime = millis();
  _curPat = _pats[0];
  _nextPat = _pats[1];
  _patNum = 1;
  _curRed = getCol(_pats[0], 0, 255);
  _curGreen = getCol(_pats[0], 1, 255);
  _curBlue = getCol (_pats[0], 2, 255);
    _rStep = ((float) getCol(_nextPat, 0, 255) - (float) _curRed)/ (float) getCol(_curPat, 3, _tScale);
    _gStep = ((float) getCol(_nextPat, 1, 255) - (float) _curGreen)/ (float) getCol(_curPat, 3, _tScale);
    _bStep = ((float) getCol(_nextPat, 2, 255) - (float) _curBlue)/ (float) getCol(_curPat, 3, _tScale);
  lightUp();
}

void RobotLEDControl::Update()
{
  _curTime = millis();
  _maxTime = getCol(_curPat, 3, _tScale);

  if (_curTime > _maxTime + _lastUpdateTime){
    if (_patNum < _numOfPats - 1) {
        _curPat = _nextPat;
        _nextPat = _pats[_patNum + 1];  
        _patNum++;         
        _lastUpdateTime = _curTime;
     } else {
      _curPat = _nextPat;
      _nextPat = _pats[0];
      _patNum = 0;
      _lastUpdateTime = _curTime;
      }
    _curRed = getCol(_curPat, 0, 255);
    _curGreen = getCol(_curPat, 1, 255);
    _curBlue = getCol (_curPat, 2, 255);
    _rStep = ((float) getCol(_nextPat, 0, _redScale) - (float) _curRed)/ (float) getCol(_curPat, 3, _tScale);
    _gStep = ((float) getCol(_nextPat, 1, _greenScale) - (float) _curGreen)/ (float) getCol(_curPat, 3, _tScale);
    _bStep = ((float) getCol(_nextPat, 2, _blueScale) - (float) _curBlue)/ (float) getCol(_curPat, 3, _tScale);
   
    
  } else {
    _curRed =  getCol(_curPat, 0, _redScale) + ((_curTime - _lastUpdateTime) * _rStep);
    _curGreen =  getCol(_curPat, 1, _greenScale) + ((_curTime - _lastUpdateTime) * _gStep);
    _curBlue =  getCol(_curPat, 2, _blueScale) + ((_curTime - _lastUpdateTime) * _bStep);     
  }
  lightUp();
  
}

/* Bytes are encoded RRGGBBTT (T = time)
*/

int RobotLEDControl::getCol(byte myByte, int curPart, int scale){
  for (int i = 3; i > curPart; i--){
    myByte = myByte >> 2;
  }
  myByte = myByte & 3;
 return(myByte * scale/3); 
}


 void RobotLEDControl::lightUp(){
  _rPin = constrain(_rPin, 0, 255);
  _bPin = constrain(_bPin, 0, 255);
  _gPin = constrain(_gPin, 0, 255);
  
  analogWrite(_rPin, _curRed);
  analogWrite(_gPin, _curGreen);
  analogWrite(_bPin, _curBlue);
}