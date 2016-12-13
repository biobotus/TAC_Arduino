//Include
#include <Goertzel.h> //library for goertzel algorithm. 
#if (ARDUINO >= 100) //check for older Arduino version
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h> //Include library for rosserial
#include <std_msgs/String.h> //Include string msg for rosserial
#include <ArduinoJson.h> //Include arduino json library to be able to send and receive json message

//Define
//All the following are required by the hat BTN8982, see it's datasheet for more details
#define  IS_1  0
#define  IS_2  1
#define  IN_1  3 //pwm for output 1
#define  IN_2  11 //pwm for output 2
#define  INH_1 12
#define  INH_2 13
byte photoEmitterPin = 2; //pin for PWM of the LED (photo emitter)
byte photoReceiverPin = A2; //pin for output of photoreceiver (amplified)

//Functions
void callback_tac1_to_serialnode( const std_msgs::String& json_msg); //callback when a ros message is received from tac1_node
void startTac(); //start TAC function
void stopTac(); //stop TAC function
void config1(); //function to configure parameters of thermoregulation and motor speed
void calibrateTurb(); //calibrate turbidity values
void turbidityAnalysis(); // Function that compute the turbidity in % of the liquid
void reset_ports(); //function of the BTN8982
void getTecParameters(); //get actual parameters (like actual temperature) from the TEC controller
void startTec(); //start heating or cooling the TEC
void stopTec(); //stop heating or cooling the TEC
void sendTecParameters(); //send target parameters to TEC controller
void sendActualValues(); //send actual values to the Tac1_node

//Global variables

//All the target parameters received from tac1_node
float targetTemp = 25;
int targetTurb = 50;
long refreshRate = 1000;
int motorSpeed = 50;
//All the target parameters when turbidity goal is obtained, received from tac1_node
float targetTempGoal = 25;
bool targetTurbGoal;
long refreshRateGoal = 10000;
int motorSpeedGoal = 0;

//previous values used in config1
int previousTargetTemp = 0;
int previousTargetTurb = 0;
long previousRefreshRate = 0;
int previousMotorSpeed = 0;

//max and min values of the motor
int motorMaxPwm = 255;
int motorMinPwm = 120;

//PID and max and min temperature used for the TEC controller
int P;
int I;
int D;
int Tmin;
int Tmax;

float turbidity_actual; //actual value of turbidity
float turb_0 = 50; //result of calibration at 0% tubidity
float turb_100 = 25000;//result of calibration at 100% tubidity
int calibrationTurbNumber = 100; //number of turbidity measure used of calibration


bool modeRun = false; //mode Run, true when tac is running
bool modeConfig = false; //used to stop reading turbidity when a set of parameters is received from tac1_node.  Otherwise a long strin is not received.
bool turbGoalReached = false; //true when turbidity goal is reached

//all these parameters are used by goertzel algorithm to compute the energy at the target frequency
const float TARGET_FREQUENCY = 491.07; //frequency to look at (LED)
const int N = 200; //number of samples
const float THRESHOLD = 4000; //not used
const float SAMPLING_FREQUENCY = 8928.571; //sampling frequency of the arduino mega
Goertzel goertzel = Goertzel(TARGET_FREQUENCY, N, SAMPLING_FREQUENCY);

const char* action; //type of action sent by the tac1_node
int calibrationLevel; //0 or 100% calibration
std_msgs::String debug_msg; //message used for debug
std_msgs::String json_msg1; //used for json message
char buffer1[256]; //used for json message
ros::NodeHandle  nh; //node handle used for rosserial
ros::Publisher pub_debug("debug", &debug_msg); //publisher output results on debug topic
ros::Publisher serialnode_to_tac1("SerialNode_To_Tac1", &json_msg1); //publisher to send data to tac1_node
char parseFailed[25] = "parseObject() failed";

bool tecIsOn = false; //true if TEC controller is heating or cooling
//actual values acquired from TEC controller
float Tsetpoint_actual;
float P_actual;
float I_actual;
float D_actual;
float Tmin_actual;
float Tmax_actual;
float Tmeasured_actual;
int Cooling_actual;
int Heating_actual;
int OC_actual;
int PWM_actual;

//timer used in serial communication with tec controller
unsigned long startTimer = 0;
const long maxTime = 1000;

//integer that keeps track of the number of loop done since the beginnning (for debug only)
int loopNumber = 0;


void callback_tac1_to_serialnode( const std_msgs::String& json_msg) {

  //creates a static Json buffer
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json_msg.data);

  /* if(!root.success()) {
     debug_msg.data=parseFailed;
     pub_debug.publish(&debug_msg);
    return;
    }*/
  action = root["a"]; //get the action sent by the tac1_node
  bool params;
  if (action[0] == 'g') { //if action is configure : take a pause to remove the bug while receiving a long string
    modeConfig = root["params"]; //set modeConfig to true of false
  } else if (action[0] == 'p') { //if action is a sending a set of parameters
    //get all the target paramters
    targetTemp = root["t"];
    targetTurb = root["u"];
    refreshRate = root["r"];
    motorSpeed = root["m"];
    targetTempGoal = root["tg"];
    targetTurbGoal = root["ug"];
    refreshRateGoal = root["rg"];
    motorSpeedGoal = root["mg"];
    P = root["p"];
    I = root["i"];
    D = root["d"];
    Tmin = root["Tmin"];
    Tmax = root["Tmax"];
    config1(); //start configuration

  } else if (action[0] == 'c') { //if action is calibration
    params = root["params"];
    if (!params) { //if 0% calibration is required
      calibrationLevel = 0;
    } else { //if 100% calibration is required
      calibrationLevel = 100;
    }
    calibrateTurb(); //start calibration
  } else if (action[0] == 's') { //if action is start/stop
    params = root["params"];
    if (params) {
      startTac();
    } else {

      stopTac();
    }
  } else {
    //output message that action is not valid
  }
}

ros::Subscriber<std_msgs::String> tac1_to_serialnode("Tac1_To_SerialNode", callback_tac1_to_serialnode);


void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps (used by rosserial)
  Serial1.begin(9600);// opens serial port 1, sets data rate to 9600 bps (used by tec controller)
  pinMode(IN_1, OUTPUT); //for pwm of the motor
  pinMode(IN_2, OUTPUT); //for pwm of the fan
  pinMode(INH_1, OUTPUT); //used by BTN8982
  pinMode(INH_2, OUTPUT);//used by BTN8982

  reset_ports(); //required by BTN8982
  digitalWrite(INH_1, 1);//required by BTN8982
  digitalWrite(INH_2, 1);//required by BTN8982

  analogReference(INTERNAL1V1); //set internal reference to 1.1V to have a better dynamic range for the photoreceiver reading
  pinMode(photoEmitterPin, OUTPUT);

  nh.getHardware()->setBaud(115200); //set baud rate of rosserial at 115200
  nh.initNode(); //start ros ndoe
  nh.advertise(pub_debug); //start a publisher on pub_debug
  nh.advertise(serialnode_to_tac1); //start a publisher on serial_node_to_tac1
  nh.subscribe(tac1_to_serialnode); //start a suscriber on tac1_to_serialnode
  stopTec(); //stop the Tec controller on startup
}

void loop() {
  nh.spinOnce(); //required for rosserial
  StaticJsonBuffer<400> jsonBuffer1;
  JsonObject& root2 = jsonBuffer1.createObject();
  if (!modeConfig) { //check if we are not receiving parameters from tac1_node
    getTecParameters(); //get actual paramters from tec controller
    if (!tecIsOn && PWM_actual != 0) { //be sure that tec controller is off if it is ment to (it is automaticaly ON on startup)
      stopTec();
    }
    if (modeRun) {
      if (!turbGoalReached) {
        //loopNumber++;
        turbidityAnalysis();
        if (turbidity_actual < targetTurb) { //check if target turbidity is reached
          turbGoalReached = true;
          //update target parameters to goal parameters
          refreshRate = refreshRateGoal;
          motorSpeed = motorSpeedGoal;
          if (targetTempGoal == -1) {
            stopTec();
          } else {
            targetTemp = targetTempGoal;
          }
          config1(); //configure parameters to goal parameters
        }
        sendActualValues(); //send actual values to tac1_node

      } else { //if target turbidity is reached
        if (targetTurbGoal) {
          turbidityAnalysis();
        } else {
          delay(refreshRate);
        }
        if (targetTempGoal != -1) {
          getTecParameters();
        }
        if (targetTempGoal != -1 || targetTurbGoal) {
          sendActualValues();
        }
      }

    } else {
      delay(1000);
    }
  }
  delay(1);
}

//function used to start tac, when start is received from tac1_node
void startTac() {
  modeRun = true;
  config1();
  analogWrite(photoEmitterPin, 127); //start led
  startTec();
  turbGoalReached = false;
  modeRun = true;
}

//function used to stop tac, when stop is received from tac1_node
void stopTac() {
  analogWrite(photoEmitterPin, 0); //stop led
  analogWrite(IN_1, 0); //stop motor
  stopTec();
  modeRun = false;
}

//set parameters to target_parameters
void config1() {
  if (previousTargetTemp != targetTemp) {
    sendTecParameters();
  }
  previousTargetTemp = targetTemp;
  if (previousMotorSpeed != motorSpeed) {
    int pwm;
    if (modeRun) {
      if (motorSpeed) {
        pwm = motorSpeed * (motorMaxPwm - motorMinPwm) / 100 + motorMinPwm;
      } else {
        pwm = 0;
      }
      analogWrite(IN_1, pwm);
      previousMotorSpeed = motorSpeed;
    }
  }


}


//do the calibration of the turbidity analysis
void calibrateTurb() {
  StaticJsonBuffer<200> jsonBuffer1;
  JsonObject& root1 = jsonBuffer1.createObject();
  root1["action"] = "calibration_result";
  int turbNumber = 0;
  float turb_calib = 0;

  if (calibrationLevel == 100) {
    analogWrite(photoEmitterPin, 127); //start led
    delay(10);
  }
  while (turbNumber < calibrationTurbNumber) {
    turbNumber++;
    goertzel.sample(photoReceiverPin); //Will take n samples
    turb_calib += goertzel.detect() / calibrationTurbNumber; //compute energy at this frequency
  }
  if (calibrationLevel == 0) {
    turb_0 = turb_calib;
    root1["turb_0"] = turb_0;
    root1.printTo(buffer1, sizeof(buffer1));
    debug_msg.data = buffer1;
    //    pub_debug.publish(&debug_msg);
    serialnode_to_tac1.publish(&debug_msg);//publish 0% turbidity result to tac1_node
  } else if (calibrationLevel == 100) {
    turb_100 = turb_calib;
    root1["turb_100"] = turb_100;
    root1.printTo(buffer1, sizeof(buffer1));
    debug_msg.data = buffer1;
    serialnode_to_tac1.publish(&debug_msg); //publish 100% turbidity result to tac1_node
  }
  if (!modeRun) {
    analogWrite(photoEmitterPin, 0); //stop led
  }
}


//do the turbidity analysis with goertzel
void turbidityAnalysis() {
  int turbNumber = 0;
  //  long startTime=millis();
  float magnitude = 0;
  int totalTurbNumber = refreshRate / 100 * 5;
  while (turbNumber < totalTurbNumber) {
    turbNumber++;
    goertzel.sample(photoReceiverPin); //Will take n samples
    magnitude += goertzel.detect() / totalTurbNumber; //compute energy at this frequency
  }
  float percentage = (magnitude - turb_0) / (turb_100 - turb_0) * 100; //compute percentage in relation to the calibration values
  percentage = percentage - 100;

  /////********************************////////////
  ///The following line is to be edited if a new polynomial regression is computed with other samples//
  //The relation was found with excel and the method is explained of the github repository
  turbidity_actual = round((0.00011542916254452 * percentage * percentage * percentage + 0.00778701306193152 * percentage * percentage + 0.624723603554344 * percentage + 100) * 10); //round the value
  turbidity_actual /= 10; //for an unknown reason, the division by ten was not working when it was on the line before.

  // long totalTime=millis()-startTime;
}

//used by BTN8982
void reset_ports()
{
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 0);
}

//Get parameters from the TEC controller, see datasheet of the TEC controller to see how to communicate with it
void getTecParameters() {
  int counter2 = 0;
  bool dataReceived = false;
  char inByte;
  Serial1.print('o'); //send 'o' to tec controller to tell it to send actual parameters
  startTimer = millis();
  while (!Serial1.available()) {
    if (millis() - startTimer > maxTime) {
      break;
    }

  }
  if (Serial1.available()) {
    dataReceived = true;
    startTimer = millis();
    while (inByte != '<') {
      if (millis() - startTimer > maxTime) {
        break;
      }
      inByte = Serial1.read();
    }
    if (inByte == '<') {
      Tsetpoint_actual = Serial1.parseFloat();
      P_actual  = Serial1.parseFloat();
      I_actual  = Serial1.parseFloat();
      D_actual  = Serial1.parseFloat();
      Tmin_actual  = Serial1.parseFloat();
      Tmax_actual  = Serial1.parseFloat();
      Tmeasured_actual  = Serial1.parseFloat();
      Heating_actual  = Serial1.parseInt();
      Cooling_actual  = Serial1.parseInt();
      OC_actual  = Serial1.parseInt();
      PWM_actual  = Serial1.parseInt();
    }
  }
}



//turn On TEC, see it's datasheet to see how to communicate with it
void startTec() {
  analogWrite(IN_2, 255); //start fan
  Serial1.print('A');
  //Add some code to check that PWM is not 0
  tecIsOn = true;
}

//turn Off TEC, see it's datasheet to see how to communicate with it
void stopTec() {
  analogWrite(IN_2, 0); //stop fan
  Serial1.print('a');
  //Add some code to check that PWM is 0
  tecIsOn = false;
}


//Send parameters to TEC, see it's datasheet to see how to communicate with it
void sendTecParameters() {
  char inByte;
  char charCommand[32];
  char str_targetTemp[5];
  char str_P[5];
  char str_I[5];
  char str_D[5];
  char str_Tmin[5];
  char str_Tmax[5];
  dtostrf(targetTemp, 3, 1, str_targetTemp);
  dtostrf(P, 3, 1, str_P);
  dtostrf(I, 3, 1, str_I);
  dtostrf(D, 3, 1, str_D);
  dtostrf(Tmin, 3, 1, str_Tmin);
  dtostrf(Tmax, 3, 1, str_Tmax);
  sprintf(charCommand, "<%s %s %s %s %s %s>", str_targetTemp, str_P, str_I, str_D, str_Tmin, str_Tmax);
  for (int i = 0; i < 31; i++) {
    Serial1.print(charCommand[i]);
    delay(4);
  }
  startTimer = millis();
  while (!Serial1.available()) {
    if (millis() - startTimer > maxTime) {
      break;
    }
  }
  while (Serial1.available()) {
    inByte = Serial1.read();
  }
}

//send actual values to the tac1_node
void sendActualValues() {
  StaticJsonBuffer<400> jsonBuffer1;
  JsonObject& root2 = jsonBuffer1.createObject();
  root2["action"] = "actual_values";
  root2["actual_temperature"] = Tmeasured_actual;
  root2["actual_turbidity"] = turbidity_actual;
  /* root2["t"] = targetTemp;
    root2["tu"] = targetTurb;
    root2["r"] = refreshRate;
    root2["m"] = motorSpeed;
    root2["tg"] = targetTempGoal;
    root2["ug"] = targetTurbGoal;
    root2["rg"] = refreshRateGoal;
    root2["mg"] = motorSpeedGoal;
    root2["tu0"] = turb_0;
    root2["tu1"] = turb_100;*/
  root2.printTo(buffer1, sizeof(buffer1));
  serialnode_to_tac1.publish(&debug_msg);
}


