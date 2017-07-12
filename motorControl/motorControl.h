#ifndef motorControl_h
#define motorControl_h



#include "Arduino.h"


class motorControl{
	public:
          motorControl();
	  void setUp(int M1F, int M1R, int M1S, int M2F, int M2R, int M2S);
	  void drive(int speed, int ratioLR);
	  void rotate(int Rspeed);
	  void stop(int dropBy);
          boolean isMoving();
	private:
	   int _mLF;
	   int _mLR;
	   int _mLS;
	   int _mRF;
	   int _mRR; 
	   int _mRS;
	   int _lSpeed;
	   int _rSpeed;

};

#endif