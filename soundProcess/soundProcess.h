#ifndef soundProcess_h

#define soundProcess_h


#include "Arduino.h"




class soundProcess{

    public:
	soundProcess(int pinA, int pinB);
	float reportFreq();
	float reportPShift();
	float reportAmplitude(int side);
	void captureSound();
	boolean getPeaks();
	void getFreq();
	void getAmpDiff();
	void xcorr();

    private:
	//Sound Capture
	byte _sampleNum; 
	byte _threshold;
	byte _peakReport;
	byte _threshOpen[20][2];
	int _sensorPinA;
	int _sensorPinB;
	unsigned int _data[180][2];
	unsigned int _cycleTime;
	boolean _isGoodCapture;

	//Peak Analysis (assigned by getPeaks)
	byte _peakNum[2];
	unsigned int _peaks[20][2];
	unsigned int _peakPos[20][2];

	//Frequency (assigned by getFreq)
	float _freq;
	float _peakWidth;

	//Amplitude difference (getAmpDiff)
	float _ampDiff;
	float _amplitude[2];

	//cross correlation analysis
	float _phaseShift;
};	

#endif