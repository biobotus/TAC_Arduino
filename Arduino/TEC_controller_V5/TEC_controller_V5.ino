//Includes
#define  IS_1  0
#define  IS_2  1
#define  IN_1  3
#define  IN_2  11
#define  INH_1 12
#define  INH_2 13

//Variables
//Example of data sent by TEC controller : <20.0 10.0 4.0 1.0 0.0 50.0 23.1 0 1 0 100>
bool continousReadOut = false;
char inCommand;
char inData;

bool tecIsOn = true;
float targetTemp = 40.0;
float P = 10.0;
float I = 4.0;
float D = 1.0;
float Tmin = 0.0;
float Tmax = 70.0;
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

//Variables for timeout in while
unsigned long startTimer = 0;
const long maxTime = 1000;


//Functions
void getParameters();
void startStopTEC();
void toggleAutomaticReadOut(bool continousReadOut);
void sendParameters(int temp);

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(INH_1, OUTPUT);
  pinMode(INH_2, OUTPUT);

  reset_ports();

  digitalWrite(INH_1, 1);
  digitalWrite(INH_2, 1);

  analogWrite(IN_2, 255);
}

//Main function
void loop() {

  //Listen to commands : to be replace with command from ROS
  if (Serial.available()) {
    Serial.print('\n');
    Serial.println("Receiving command ... \n");
    inCommand = Serial.read();
    switch (inCommand) {
      case 'o'  :
        //getParametersCommand=true;
        getParameters();
        break;

      case 'q'  :
        startStopTEC();
        break; /* optional */

      case 'r'  :
        continousReadOut = false;
        toggleAutomaticReadOut(continousReadOut);
        break; /* optional */

      case 'R'  :
        continousReadOut = true;
        toggleAutomaticReadOut(continousReadOut);
        break; /* optional */

      case 't'  :
        while (!Serial.available()) {}
        targetTemp = Serial.parseInt();
        Serial.println(targetTemp);
        sendParameters();
        break; /* optional */

      case 'n'  :
        targetTemp = 25;
        sendParameters();
        break; /* optional */

      case 'h'  :
        targetTemp = 40;
        sendParameters();
        break; /* optional */

      case 'c'  :
        targetTemp = 15;
        sendParameters();
        break; /* optional */

      default :
        Serial.println("Not a valid command");
    }
  }

  //Display parameters if a new set of parameters were received from TEC
  if (continousReadOut) {
    if (Serial1.available()) {
      inData = Serial1.read();
      Serial.write(inData);
    }
  }

}

//Get parameters from the TEC controller
void getParameters() {
  int counter2 = 0;
  bool dataReceived = false;
  char inByte;
  Serial.print('\n');
  Serial.println("Getting parameters ... \n");
  Serial1.print('o');
  startTimer = millis();
  while (!Serial1.available()) {
    if (millis() - startTimer > maxTime) {
      Serial.print("No data received");
      break;
    }

    // Serial.print('.');
  }
  if (Serial1.available()) {
    dataReceived = true;
    startTimer = millis();
    while (inByte != '<') {
      if (millis() - startTimer > maxTime) {
        Serial.print("No parameters in DataReceived");
        break;
      }
      inByte = Serial1.read();
      Serial.print("Recu : ");
      Serial.print(inByte);
      Serial.print('\n');
    }
    if (inByte == '<') {
      Tsetpoint_actual = Serial1.parseFloat();
      P_actual  = Serial1.parseFloat();
      I_actual  = Serial1.parseFloat();
      D_actual  = Serial1.parseFloat();
      Tmin_actual  = Serial1.parseFloat();
      Tmax_actual  = Serial1.parseFloat();
      Tmeasured_actual  = Serial1.parseFloat();
      Cooling_actual  = Serial1.parseInt();
      Heating_actual  = Serial1.parseInt();
      OC_actual  = Serial1.parseInt();
      PWM_actual  = Serial1.parseInt();
      Serial.print("Tsetpoint "); Serial.println(Tsetpoint_actual);
      Serial.print("P "); Serial.println(P_actual);
      Serial.print("I "); Serial.println(I_actual);
      Serial.print("D "); Serial.println(D_actual);
      Serial.print("Tmin "); Serial.println(Tmin_actual);
      Serial.print("Tmax "); Serial.println(Tmax_actual);
      Serial.print("Tmeasured "); Serial.println(Tmeasured_actual);
      Serial.print("Cooling "); Serial.println(Cooling_actual);
      Serial.print("Heating "); Serial.println(Heating_actual);
      Serial.print("OC "); Serial.println(OC_actual);
      Serial.print("PWM  "); Serial.println(PWM_actual);
    }
  }
}



//Toggle TEC supply (ON if it was OFF and OFF if it was ON)
void startStopTEC() {
  if (tecIsOn) {
    Serial1.print('a');
    //Add some code to check that PWM is 0
  } else {
    Serial1.print('A');
    //Add some code to check that PWM is not 0
  }
  tecIsOn = !tecIsOn;
}


void toggleAutomaticReadOut(bool continousReadOut) {
  if (continousReadOut) {
    Serial1.print('R');
    //Add verification that parameters are received automatically
  } else {
    Serial1.print('r');
    //Add verification that nothing is received
  }

}

void sendParameters() {
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
      Serial.print("No data received");
      break;
    }

    // Serial.print('.');
  }
  while (Serial1.available()) {
    inByte = Serial1.read();
    Serial.print(inByte);

  }
}

void reset_ports()
{
  digitalWrite(IN_1, 0);
  digitalWrite(IN_2, 0);
}

