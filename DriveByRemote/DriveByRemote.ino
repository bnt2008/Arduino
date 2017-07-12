#include <PID_v1.h>

#include <yawAndPitch.h>
#include <Wire.h>
#include <motorControl.h>


#include <IRLibRecvPCI.h>   
#include <IRLibDecodeBase.h>  
#include <IRLib_HashRaw.h>  
#include <IRLibCombo.h>
  
/////////////////////
///Set up the PID ///
/////////////////////
double curDir, curRatio, commandDir, curPitch, curBalOut, commandPitch;
const double dirKp = 70;
const double dirKi = 50;
const double dirKd = 0;
 double balKp = 50;
 double balKi = 10;
 double balKd = 0;
float balRatio = 0;
unsigned long cycleTime[10];
boolean unBalanced = false;
unsigned long lastUnBalanced;
int recoveryTime = 1000; //time in millisecs. robot must be balanced to clear the unbalanced flag
byte byteToSend1, byteToSend2;

motorControl mainMotor;
yawAndPitch myMPU; 
PID dirPID(&curDir, &curRatio, &commandDir, dirKp, dirKi, dirKd, DIRECT);
PID balPID(&curPitch, &curBalOut, &commandPitch, balKp, balKi, balKd, DIRECT);

/////////////////////////
//IR Decoder/////////////
/////////////////////////

IRrecvPCI myReceiver(2); //Create the receiver. Use pin 2    
IRdecode myDecoder; //create decoder 



void setup() {
   myReceiver.enableIRIn(); // Start the receiver  
   mainMotor.setUp(10,9,11,5,4,6);
   myMPU.setUp(0x68);
   myMPU.findOffsets(50);
  myMPU.switchYaw(false);  //make sure forward and to the right are positive
  myMPU.switchPitch(true);
  myMPU.switchPitchGyro(false);
  myMPU.setWeight(.85);
  myMPU.useDelay(20);
  commandPitch = -6; //myMPU.Pitch();
   curDir = 0;
   curRatio = 0;
   dirPID.SetMode(AUTOMATIC);
   dirPID.SetOutputLimits(-255,255);
   dirPID.SetSampleTime(30);
   balPID.SetMode(AUTOMATIC);
   balPID.SetOutputLimits(-190, 190);
   balPID.SetControllerDirection(REVERSE);
   balPID.SetSampleTime(30);
   Serial.begin(9600);
}

void loop() {

  
  int curCommand = getIR();
  switch (curCommand){
    case 0: //do nothing
      myMPU.Update();
      balance();
      break;
    case 1: //go forward
      drive(0, curCommand);
      break;
    case 3: //go left
      drive(-30, curCommand);
      break;
    case 4: // go right
      drive(30, curCommand);
      break;
    case 7: // Decrease Kp
      if (balKp > 0) {balKp--;}
      balKi = balKp  + balRatio;
      balKi = constrain(balKi, 0, 4 * balKp);
      balPID.SetTunings(balKp, balKi, balKd);
      break;
    case 8: // Increase Kp
      if (balKp < 250) { balKp++;}
      balKi = balKp  + balRatio;
      balKi = constrain(balKi, 0, 4 * balKp);
      balPID.SetTunings(balKp, balKi, balKd);
      break;
    case 10: //decrease Ki
      balRatio--;
      balKi = balKp  + balRatio;
      balKi = constrain(balKi, 0, 4 * balKp);
      balPID.SetTunings(balKp, balKi, balKd);
      break;
    case 11: // increase Ki
      balRatio++;
      balKi = balKp  + balRatio;
      balKi = constrain(balKi, 0, 4 * balKp);
      balPID.SetTunings(balKp, balKi, balKd);
      break;
  }
  communicate();

}

void communicate(){

byteToSend1 = 0;

byteToSend1 = byteToSend1 | unBalanced;

Serial.write(byteToSend1 + 48);
}

  
void drive(double angle, int command){
  int newCommand = command;
  commandDir = angle;
  myMPU.resetYaw(); 
  unsigned long endCommand = millis() + 300;
  while (millis() < endCommand){
    myMPU.Update();
    curDir = myMPU.Yaw();
    dirPID.Compute();
    Serial.print(curDir);
    Serial.print("|");
    Serial.println(curRatio);
    
    mainMotor.drive(150, curRatio);
    
    newCommand = getIR();
  }
  mainMotor.stop(20);
  balance();
  myMPU.resetYaw();
}

void balance(){
  myMPU.Update();
  curPitch = myMPU.Pitch();
  
  if (curPitch > commandPitch){
    balPID.Compute();
    mainMotor.drive(curBalOut, 0);
    lastUnBalanced = millis();
    unBalanced = true;
  } else {
    mainMotor.drive(0,0);
    if (millis() - lastUnBalanced > recoveryTime){unBalanced = false;}
  }
}

///////////////////////////////////////////////////
/// Get any recieved IR codes and translate into///
/// an action command                           ///
///////////////////////////////////////////////////

int getIR(){
    int option = 0; 
    if (myReceiver.getResults()) {  
        myDecoder.decode();           //Decode it  
        uint32_t result = myDecoder.value;  //the results of a Now print results. 
        switch (result){
          case 2331063592: // button
           option = 255;
           break;
          case 3261853764: //Up Button
            option = 1;
            break;
          case 3305092678: //Dn Button
            option = 2;
            break;
          case 1972149634: // Left button
           option = 3;
            break;
          case 1400905448: //right button
           option = 4;
            break;
          case 3778927144: //#1
           option = 7;
           break;
          case 2908251746: //#2
           option = 8;
           break;
          case 657459652: //#3
            option = 9;
            break;
          case 4120482440:  //#4
            option = 10;
            break;
          case 1931099650: //#5
            option = 11;
            break;
        }   
    myReceiver.enableIRIn();      //Restart receiver   } 
    } 
  return(option);
}

