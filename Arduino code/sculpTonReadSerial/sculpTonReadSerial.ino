//***************************************************************************************
//***************************************************************************************
//Sculpton 
//Alberto Boem 2012-2013
//Interface Culture Linz
//Arduino Mega 2560
//FreeIMU Library http://www.varesano.net/projects/hardware/FreeIMU
//adapted for DF Robot 10DoF Imu (ADXL345, ITG3200, HMC5883L, BMP085)
//***************************************************************************************
//***************************************************************************************

#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>

//***************************************************************************************

//On Off state led
int ledCheck = 6;

//Output LED
int led1_pin = 8;
int led2_pin = 9;
int led3_pin = 10;
int led4_pin = 11;

int brightnessLed = 0;
int fadeAmount = 5;

//Pushbutton
const int button_pin = 26; //24
int buttonState = 0;      // variable for reading the pushbutton status
int buttonCounter = 0;
int lastButtonState = 0;

//Reading Sensors
//1 - Distance sensors :
//LDR
int sensorPin1 = A0;    //LDR 1 (white cable)
int sensorPin2 = A1;    //LDR 2
int sensorPin3 = A2;    //LDR 3
int sensorPin4 = A3;    //LDR 4
int sensorPin5 = A4;    //LDR 5
int sensorPin6 = A5;    //LDR 6

//LED
int ledPin1 = 40;       //LED 1 (green cable)
int ledPin2 = 38;       //LED 2
int ledPin3 = 36;       //LED 3
int ledPin4 = 34;       //LED 4
int ledPin5 = 32;       //LED 5
int ledPin6 = 30;       //LED 6

//Distance values
//store the value coming from the sensor (ldr)
int sensorValueA = 0;  
int sensorValueB = 0;
int sensorValueC = 0;
int sensorValueD = 0;
int sensorValueE = 0;
int sensorValueF = 0;

//2 -  10 DoF IMU :
float accGyro_values[6]; //acc+gyro data (x,y,z,g)
float angles[3]; //yaw pitch roll (y,p,r)

FreeSixIMU sixDOF = FreeSixIMU(); //initialize the IMU (ADXL345/ITG3200)


//***************************************************************************************

void setup() {
  
  Serial.begin(9600); //19200
  Wire.begin();
  
  delay(5);
  sixDOF.init(true);  //'true' for fast mode
  delay(5);
  
  //On Off led state
  pinMode(ledCheck, OUTPUT); 
  
  //Led output (Distance sensors)
  pinMode(ledPin1, OUTPUT);  
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin6, OUTPUT);
  
  //Output led
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(led3_pin, OUTPUT);
  pinMode(led4_pin, OUTPUT);
  
  //Button for calibration
  pinMode(button_pin, INPUT);
}

//***************************************************************************************

void loop() {
  
  startDistanceSensors(); //turn led on and read data from the ldr
  
  sixDOF.getValues(accGyro_values); //get acc+gyro values
  
  sixDOF.getYawPitchRoll(angles); //get yaw pitch roll values 
  
  checkPushButton(); //check the state of the pushbutton
  
  fadeLed();
  
  printData(); //print data on the serial port
  
  delay(10);
  
}

//***************************************************************************************

void startDistanceSensors() {
  
  // turn led on
  digitalWrite(ledPin1, HIGH); 
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH); 
  digitalWrite(ledPin5, HIGH);
  digitalWrite(ledPin6, HIGH);
  
  // read the value from the sensor:
  sensorValueA = analogRead(sensorPin1);  
  sensorValueB = analogRead(sensorPin2);
  sensorValueC = analogRead(sensorPin3);
  sensorValueD = analogRead(sensorPin4);
  sensorValueE = analogRead(sensorPin5);
  sensorValueF = analogRead(sensorPin6);
  
}

//***************************************************************************************

void printData() {
  
  //Serial print to pd
  Serial.print(F("A "));
  
  //Values  
  //Distance sensors
  Serial.print(sensorValueA, DEC); //Distance sensors
  Serial.print(F(" "));
  Serial.print(sensorValueB, DEC); 
  Serial.print(F(" "));
  Serial.print(sensorValueC, DEC);
  Serial.print(F(" "));
  Serial.print(sensorValueD, DEC); 
  Serial.print(F(" "));
  Serial.print(sensorValueE, DEC); 
  Serial.print(F(" "));
  Serial.print(sensorValueF, DEC); 
  Serial.print(F(" "));
  
  //
  Serial.print(accGyro_values[0]); //ADXL345 acc X
  Serial.print(F(" "));
  Serial.print(accGyro_values[1]); //ADXL345 acc Y
  Serial.print(F(" "));
  Serial.print(accGyro_values[2]); //ADXL345 acc Z
  Serial.print(F(" "));
  Serial.print(accGyro_values[3]); //ITG3200 gyro
  Serial.print(F(" "));
  
  //Yaw Pitch Roll
  Serial.print(angles[0]); //yaw (z)
  Serial.print(F(" "));
  Serial.print(angles[1]); //pitch (y)
  Serial.print(F(" "));
  Serial.print(angles[2]); //roll (x)
  Serial.print(F(" "));
  Serial.print(buttonCounter); //button state (0, no, 1, yes)
  Serial.print(F(" "));
  
  Serial.println();
  
}

//***************************************************************************************

void checkPushButton() {
  
  buttonState = digitalRead(button_pin);

  // check if the pushbutton is pressed.
  if(buttonState != lastButtonState){
  if (buttonState == HIGH) {    
    //on    
    buttonCounter = 1;
    //blink led
    digitalWrite(ledCheck, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(50);               // wait for a second
    digitalWrite(ledCheck, LOW);    // turn the LED off by making the voltage LOW
    delay(50);  
  } 
  else {
    //off:
    buttonCounter = 0;
    digitalWrite(ledCheck, HIGH);
  }
 }
 
  lastButtonState = buttonState;
  
  if (buttonState == LOW) {    
    //blink led
    digitalWrite(ledCheck, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(50);               // wait for a second
    digitalWrite(ledCheck, LOW);    // turn the LED off by making the voltage LOW
    delay(50);  
  } 
  else {
    //off:
    digitalWrite(ledCheck, HIGH);
  }
  
}

//***************************************************************************************

void fadeLed() {
  
  analogWrite(led1_pin, brightnessLed);
  analogWrite(led2_pin, brightnessLed);
  analogWrite(led3_pin, brightnessLed);
  analogWrite(led4_pin, brightnessLed);
  
  brightnessLed = brightnessLed + fadeAmount;
  
  if(brightnessLed == 0 || brightnessLed == 255) {
    fadeAmount = -fadeAmount;
  }
  
  //delay(30);
}

//***************************************************************************************
/*  
    Mapping of the distance sensors (ldr + led)

    SensorValue A = sensorPin1 + ledPin5
    SensorValue B = sensorPin2 + ledPin2
    SensorValue C = sensorPin3 + ledPin6
    SensorValue D = sensorPin4 + ledPin3
    SensorValue E = sensorPin5 + ledPin1
    SensorValue F = sensorPin6 + ledPin4   
                                             */
//***************************************************************************************
