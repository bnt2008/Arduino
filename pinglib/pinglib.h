#ifndef pinglib_h
#define pinglib_h



#include "Arduino.h"
#include "Servo.h"

class pinglib{
	public:
	  pinglib();
	  void setUp(int tL, int eL, int tR, int eR, int ServoPin);
	  void findDistance(byte angle);
	  void setBearings(int a, int b, int c, int d, int e);
	  int getDistance(int angle);
	  int getChange(int angle);
          int getBearing(int a);
	private:
	   int _trigPinR, _echoPinR, _servoPin, _trigPinL, _echoPinL;
	   byte _curAngle;
	   Servo headServo;
	   int _bearings[5]; 
	   int _distances[5];
 	   byte _changes[5]; // 1 = Something disappeared 2= something appeared 3= something got farther 4= something got closer
	   unsigned long _lastUpdates[5];
	   float _changeTolerance; // requires this much % change to register a change
	  
};

#endif