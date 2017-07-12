#include "Arduino.h"
#include "yawAndPitch.h"
#include "Wire.h"

yawAndPitch::yawAndPitch(){

}
void yawAndPitch::switchYaw(boolean yaw){
  reverseYaw = yaw;
}

void yawAndPitch::switchPitch(boolean pitch){
   reversePitch = pitch;
}

void yawAndPitch::switchPitchGyro(boolean pitch){
  reversePitchGyro = pitch;
}

float yawAndPitch::Yaw(){
return(_yaw);
}

float yawAndPitch::Pitch(){
return(_pitch);
}

void yawAndPitch::resetYaw(){
_yaw = 0;
}

//Interrupt routine
void yawAndPitch::gyroAction(){
  _dataReady = true;
}

void yawAndPitch::useInterrupts(){
		  _useInt = true;
		  Wire.beginTransmission(_MPU_addr);
 		  Wire.write(0x38);  // Interrupt register
		  Wire.write(1);     // set to 1 (data ready)
		  Wire.endTransmission(true);
		 
}

void yawAndPitch::useDelay(int delay){
_cycleDelay = delay;
_useInt = false;
}
void yawAndPitch::setWeight(float newWeight){
_weight = newWeight;
}
void yawAndPitch::setUp(int MPU_addr = 0x68){
	_MPU_addr = MPU_addr;
	reverseYaw = false;
	reversePitch = false;
	reversePitchGyro = false;
	_useInt = false;
	_gyoffsetX = 0;
	_gyoffsetY = 0;
	_gyoffsetZ = 0;
	_weight = .95;
	_dataReady = false;
	_cycleDelay = 20; //if not using interrupts, this will ensure a certain time per cycle
	  Wire.begin();
	  Wire.beginTransmission(_MPU_addr);
 	  Wire.write(0x6B);  // PWR_MGMT_1 register
  	  Wire.write(0);     // set to zero (wakes up the MPU-6050)
          Wire.endTransmission(true);

      	  Wire.beginTransmission(_MPU_addr);
	  Wire.write(0x1B);  // Register for gyroscope sensitivity
	  Wire.write(B00010000);     // set to 2000
	  Wire.endTransmission(true);

	  Wire.beginTransmission(_MPU_addr);
	  Wire.write(0x1C);  // Register for Accelerometer sensitivity
	  Wire.write(B00000000);     // set to 2g
	  Wire.endTransmission(true);

	_dataTime = micros();	  
}

//Function to automatically calculate the offsets and write them to the appropriate variables
void yawAndPitch::findOffsets(int numOfCycles){
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Storage for values from MPU

  for (int i = 0; i < numOfCycles; i++){
    Wire.beginTransmission(_MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(_MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    _gyoffsetX += GyX;
    _gyoffsetY += GyY;
    _gyoffsetZ += GyZ;    
  }
  _gyoffsetX = -1 * _gyoffsetX / numOfCycles;
  _gyoffsetY = -1 * _gyoffsetY / numOfCycles;
  _gyoffsetZ = -1 * _gyoffsetZ / numOfCycles;

}


//Function to perform the update
void yawAndPitch::Update(){
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Storage for values from MPU
  unsigned long curTime, dTime;
  float acPitch; //Stores pitch calculated from accellerometer
  float dYaw, dPitch; //Stores the change in yaw and pitch calculated from the gyroscope

  //Wait for the dataReady flag, or the appropriate amount of time
  if (_useInt){
	while (!_dataReady){}
  } else {
	while (micros() < _dataTime + _cycleDelay * 1000){}
  }


  curTime = micros();
  Wire.beginTransmission(_MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(_MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  GyY = GyY + _gyoffsetY;
  GyZ = GyZ + _gyoffsetZ;
  GyX = GyX + _gyoffsetX;
  
  if (reverseYaw) {
	GyZ = GyZ * -1;  }
  if (reversePitchGyro) {GyY = GyY * -1;}
 
  acPitch = atan2(AcX, 16384) * 2 * 4068/71;
  if (reversePitch){acPitch = acPitch * -1;};
  dPitch = 0.030518 * GyY;   //Current delta Radians/Sec
  dYaw = 0.030518 * GyZ;   //Current delta Radians/Sec
  dTime = curTime - _dataTime;

  
  //For pitch, blend the accelerometer and gyro
  _pitch = (1 - _weight) * acPitch + _weight * (_pitch + (dPitch * dTime/1000000));
  //for yaw, just keep on integratin'
  _yaw = _yaw + (dYaw * dTime/-1000000);
  _dataTime = micros();
  
//Reset the interrupt
  /*
  Wire.beginTransmission(_MPU_addr);
  Wire.write(0x58);
  Wire.endTransmission(false);
  Wire.requestFrom(_MPU_addr, 1, true);
  Wire.read();
  */
}

