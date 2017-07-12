#ifndef yawAndPitch_h
#define yawAndPitch_h



#include "Arduino.h"

class yawAndPitch{
	public:
	   yawAndPitch();
	   void setWeight(float newWeight);
	   void setUp(int MPU_addr);
	   void findOffsets(int numOfCycles);
	   void Update();
	   void resetYaw();
	   float Yaw();
	   float Pitch();
	   void useInterrupts();
	   void useDelay(int delay);
	   void gyroAction();
	   void switchYaw(boolean yaw);
	   void switchPitch(boolean pitch);
 	   void switchPitchGyro(boolean pitch);
	private:
	   boolean reverseYaw;
	   boolean reversePitch;
	   boolean reversePitchGyro;
	   int useInt;
	   float _gyoffsetX;
	   float _gyoffsetY;
	   float _gyoffsetZ;
	   float _weight;
	   float _yaw;
	   float _pitch;
	   int _MPU_addr;
	   volatile boolean _dataReady;
	   int _cycleDelay;
	   unsigned long _dataTime;
	   boolean _useInt;
};

#endif
	   

