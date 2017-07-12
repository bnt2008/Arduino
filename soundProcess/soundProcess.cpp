#include "Arduino.h"
#include "soundProcess.h"



soundProcess::soundProcess(int pinA, int pinB){
	_sampleNum = 180; //can't change unless also change the .h file
	_threshold = 20;
	_peakReport = 3; //number of peaks needed to call it a good capture
	_sensorPinA = pinA;
	_sensorPinB = pinB;
        _isGoodCapture = false;

}

float soundProcess::reportFreq(){
	return(_freq);
}

float soundProcess::reportPShift(){
	return(_phaseShift);
}

float soundProcess::reportAmplitude(int side){
	float amp = _amplitude[side];
	return(amp);
}

void soundProcess::captureSound(){ //Captures two channels of sounds and saves to buffer
    unsigned long startTime;
    _isGoodCapture = false;
    startTime = micros();    
    for (int i = 0; i < _sampleNum; i++){
     _data[i][0] = analogRead(_sensorPinA); 
     _data[i][1] = analogRead(_sensorPinB);
    };
    _cycleTime = micros() - startTime;

   _freq = 0;
   _peakWidth = 0;
   _ampDiff = 0;
   _amplitude[0] = 0;
   _amplitude[1] = 0;
   _phaseShift = 0;
}

boolean soundProcess::getPeaks(){
  int curPeakPos;
  int curMax = 0;
  
  for (int side = 0; side < 2; side++){
     //first make sure arrays are clear
     _peakNum[side] = 0;
     for (int i = 0; i < 20; i++){
         _peaks[i][side] = 0;
         _peakPos[i][side] = 0;
	 _threshOpen[i][side] = 0;
     }

     boolean overThresh = false;
      for (int i = 1; i < _sampleNum; i++){
        if (!overThresh){
           if(_data[i-1][side] <= _threshold && _data[i][side] > _threshold){
              overThresh = true;
              _threshOpen[_peakNum[side]][side] = i;
           }
        } else { // if we are overthresh
           if (_data[i-1][side] > _threshold && _data[i][side] <= _threshold){
             overThresh = false;
             _peakPos[_peakNum[side]][side] = curPeakPos;
             _peaks[_peakNum[side]][side] = curMax;
             _peakNum[side]++;
             curMax = 0;
             if (_peakNum[side] > 19){i = _sampleNum + 1;} // only capture up to 20
           } else { // we are still looking for this peak to end
             if (_data[i][side] > curMax){
              curMax = _data[i][side];
              curPeakPos = i;
             }
           }
        }      
     }// Go to the next value in the data     
    if (_peakNum[side] >=  _peakReport){_isGoodCapture = true;} else {_isGoodCapture = false;}
  }// go to next side
  return(_isGoodCapture);
}//End of getPeaks


void soundProcess::getFreq(){
  //Here we will find the frequency of the sounds
   int useSide;
  _freq = 0;
  _peakWidth = 0 ;
  if (_peakNum[0] > _peakNum[1]){useSide = 0;}else{useSide = 1;}
  
  for (int i = 1; i < _peakNum[useSide]; i++){
    _peakWidth = _peakWidth + (_threshOpen[i][useSide] - _threshOpen[i-1][useSide]);
  }
  _peakWidth = _peakWidth / (_peakNum[useSide] - 1);
  _freq = 1/_peakWidth * 1000000 * _sampleNum/_cycleTime;

}

void soundProcess::getAmpDiff(){
   _amplitude[0] = 0;
   _amplitude[1] = 0;	
   for (int side = 0; side < 2; side++){
      for (int i = 0; i < _peakNum[side]; i++){
        _amplitude[side] = _amplitude[side] + _peaks[i][side];
      }
      _amplitude[side] = _amplitude[side]/(_peakNum[side] - 1);
   }
  _ampDiff = _amplitude[0] - _amplitude[1];
}


void soundProcess::xcorr(){
 
   int numOfPeaks = min(_peakNum[0], _peakNum[1]);
   float minDiffL = 0;
   float minDiffR = 0; // MinDiff and diffOff hold the min Offsets
   int diffOffL = 0;
   int diffOffR = 0;

   int offset = _peakWidth * 3;
   long diff = 0;
 
   
   for (int curOff = 0; curOff < offset; curOff++){
     for (int i = 0; i < (200 - offset); i++){
       diff = diff +  abs(_data[i][1] - _data[i + curOff][0]);
     
     } 
     if (curOff == 0){minDiffL = diff;} else {if (diff < minDiffL){minDiffL = diff; diffOffL = curOff;}}
     diff = 0;
   }
   _phaseShift = diffOffL/_peakWidth;
   _phaseShift = _phaseShift - (int) _phaseShift;
   if (_phaseShift > .5){_phaseShift = 1 - _phaseShift;}
}
