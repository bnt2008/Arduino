#ifndef tunes_h
#define tunes_h


#include "Arduino.h"


class tunes{
	public:
	  tunes(int buzzerPin);
	  void makePattern(int lowPitch, int highPitch, float upSpeed, float dnSpeed, int numOfWobs, boolean startUp, boolean endUp);
	  void Update();
	  void Silence();
	private:
	   int _buzzPin;
 	   int _curPitch, _lowPitch, _highPitch, _delPitch, _wobCounter, _numOfWobs;
 	   float _upSpeed, _dnSpeed;
           boolean _isDone, _moveUp, _endUp;
           unsigned long _lastUpdateTime;
           
};

#endif