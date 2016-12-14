//Include
#include <Goertzel.h>

//Define
#define  IS_1  A0
#define  IS_2  A1
#define  IN_1  3
#define  IN_2  11
#define  INH_1 12
#define  INH_2 13
byte photEmitterPin = 2; 
byte photoReceiverPin=A2;

//Functions
void printReadyMessage();
void startTAC();
void stopTAC();
void setTargetTemp();
void setTargetTurb();
void setRefreshRate();
void setMotorSpeed();
void calibrateTurb(); 


//Global variables
int incomingByte= 0;   // for incoming serial data

int targetTemp=25;
int targetTurb=50;
long refreshRate=1;
int motorSpeed=50;
float turb_0=50;
float turb_100=25000;

bool standAloneMode =true;

bool modeRun;
bool startInput=false;
bool previousStartInput=false;
bool readyMessageIsPrinted=false;

const float TARGET_FREQUENCY = 491.07; 
const int N = 200;   
const float THRESHOLD = 4000;  
const float SAMPLING_FREQUENCY = 8928.571; 
Goertzel goertzel = Goertzel(TARGET_FREQUENCY, N, SAMPLING_FREQUENCY);

int loopNumber=0;


void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  pinMode(IN_1,OUTPUT);
  pinMode(IN_2,OUTPUT);
  pinMode(INH_1,OUTPUT);
  pinMode(INH_2,OUTPUT);
  
  reset_ports();
  digitalWrite(INH_1,1);
  digitalWrite(INH_2,1);
  analogWrite(IN_1,128);
  
  analogReference(INTERNAL1V1);
  // signal ready to start by turning on LED 13
  pinMode(photEmitterPin, OUTPUT); 
  analogWrite(photEmitterPin, 127);
}

void loop() {
  if(!readyMessageIsPrinted){
    printReadyMessage();
    readyMessageIsPrinted=true;
  }
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if(!modeRun){
      if(incomingByte=='s'){
        startInput=true;
      }else{
         Serial.println("TAC module not started.  Send \"s\" to start.");
      }
    }else{
      switch(incomingByte) {
        case 's'  :
          stopTAC();
          Serial.println("TAC module stopped...");
          readyMessageIsPrinted=false;
          startInput=false;
          modeRun=false;
          break; 
        
        case 't'  :
          setTargetTemp();
          //
          break;
          
        case 'u'  :
          setTargetTurb();
          break;
          
        case 'r'  :
          setRefreshRate();
          break; 
          
        case 'm'  :
          setMotorSpeed();
          break;
          
        case 'c'  :
          calibrateTurb();
          break;
        
        default : 
        Serial.println("Not a valid command");
      }
    }
  }
  
  if(!previousStartInput && startInput){
    setTargetTemp();
    setTargetTurb();
    setRefreshRate();
    setMotorSpeed();
    calibrateTurb();
    startTAC();
    Serial.println("TAC module started...");
    modeRun=true;
    }
    
  
  if(modeRun){
    loopNumber++;
    Serial.print("Loop "); Serial.print(loopNumber);Serial.println(", ");
    int turbNumber=0;
  //  long startTime=millis();
    float magnitude;
    int totalTurbNumber=refreshRate/100*5;
    while(turbNumber<totalTurbNumber){
      turbNumber++;
      goertzel.sample(photoReceiverPin); //Will take n samples
       magnitude += goertzel.detect()/totalTurbNumber;  //check them for target_freq
    //  Serial.println(magnitude); 
    //  Serial.println(turbNumber);
    //  Serial.println(millis());
    }
     float percentage=(magnitude-turb_0)/(turb_100-turb_0)*100;
     Serial.print("Abolute : "); Serial.println(magnitude);
     Serial.print("Percentage : "); Serial.print(percentage); Serial.println("%");
     percentage=percentage-100; 
     float turbidity=round((0.00011542916254452*percentage*percentage*percentage+ 0.00778701306193152*percentage*percentage + 0.624723603554344*percentage+100)*10);
     turbidity/=10;
     Serial.print("Turbidity : "); Serial.print(turbidity,1); Serial.println("%"); 
    // long totalTime=millis()-startTime;
    // Serial.println(totalTime);
     //Serial.print("Turb "); Serial.println(turbNumber);
  }
     
    previousStartInput=startInput;
}



 void printReadyMessage(){  
  Serial.println("TAC module ready...");
  Serial.println("Send \"s\" to start.  While the TAC is running : ");
  Serial.println("send \"s\" again to stop,");
  Serial.println("send \"t\" to change target temperature,");
  Serial.println("send \"u\" to change turbidity target,");
  Serial.println("send \"r\" to change refresh rate,");
  Serial.println("send \"m\" to change motor speed,");
  Serial.println("send \"c\" to recalibrate turbidity analysis.");
 }

 void startTAC(){
  //start motor pwm
  //start fan pwm
  //start led pwm
  //send temp and PID to TEC controller
  //start TEC controller
 }
 
void stopTAC(){
   //stop motor pwm
  //stop fan pwm
  //stop led pwm
  //stop TEC controller
}

void setTargetTemp(){
    Serial.println("Send target temperature");
    while(Serial.available()==0){}
    targetTemp=Serial.parseInt();
    Serial.print("Target temperature set to "); Serial.println(targetTemp);
}

void setTargetTurb(){
    Serial.println("Send target turbidity (%)");
    while(Serial.available()==0){}
    targetTurb=Serial.parseInt();
    Serial.print("Target turbidity set to "); Serial.println(targetTurb); Serial.println("%");
}

void setRefreshRate(){
  Serial.println("Send refresh rate in milliseconds");
  while(Serial.available()==0){}
  refreshRate=Serial.parseInt();
  Serial.print("Refresh rate set to "); Serial.println(refreshRate);
}

void setMotorSpeed(){
  Serial.println("Send motor speed in seconds [0-100]");
  while(Serial.available()==0){}
  motorSpeed=Serial.parseInt();
  Serial.print("Motor speed set to "); Serial.println(motorSpeed);
}

void calibrateTurb(){
    int calibrationTurbNumber=100;
    while(incomingByte!='y'){
      Serial.println("Place 0% sample and send \"y\""); 
      while(Serial.available()==0){}
      incomingByte=Serial.read();
      if(!incomingByte=='y'){
        Serial.println("Invalid command");
      }
    }
     incomingByte=0;
     int turbNumber=0;
     turb_0=0;
     while(turbNumber<calibrationTurbNumber){
      turbNumber++;
      goertzel.sample(photoReceiverPin); //Will take n samples
       turb_0 += goertzel.detect()/calibrationTurbNumber;  //check them for target_freq
     }
     Serial.print("Turbidity 0% set to "); Serial.println(turb_0);
     
     while(incomingByte!='y'){
       Serial.println("Place 100% sample and send \"y\""); 
       while(Serial.available()==0){}
       incomingByte=Serial.read();
       if(!incomingByte=='y'){
         Serial.println("Invalid command");
       }
     }
     incomingByte=0;
     turbNumber=0;
     turb_100=0;
     while(turbNumber<calibrationTurbNumber){
      turbNumber++;
      goertzel.sample(photoReceiverPin); //Will take n samples
       turb_100 += goertzel.detect()/calibrationTurbNumber;  //check them for target_freq
     }
    Serial.print("Turbidity 100% set to "); Serial.println(turb_100);
}

void reset_ports()
{
  digitalWrite(IN_1,0);
  digitalWrite(IN_2,0);
}


