#ifndef RobotLEDControl_h

#define RobotLEDControl_h


#include "Arduino.h"



class RobotLEDControl
{
  
	public:
   
	  RobotLEDControl(int rPin, int gPin, int bPin);
     
	  void Start();
    
	  void Update();

 
	  void whiteBalance(byte R, byte G, byte B);
	  void setPattern(byte patternNo);
	  
	private:
  
	  int getCol(byte myByte, int curPart, int scale);
  
          
    
	  byte _rPin, _gPin, _bPin; 
  
	  int _tScale;
  
	  byte _pats[5];
  
	 unsigned long _lastUpdateTime, _curTime;
  
	byte _curPat, _nextPat;
  
	byte _patNum;
  
	byte _curRed, _curGreen, _curBlue;
  
	int _maxTime;
  float _rStep, _bStep, _gStep;

	byte _numOfPats;
	byte _blueScale, _redScale, _greenScale;

	void lightUp();

}
;

#endif