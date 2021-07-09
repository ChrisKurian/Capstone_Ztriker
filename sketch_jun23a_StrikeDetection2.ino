
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <string>
#include <stdio.h>


#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>


// Reset pin, MFIO pin
int resPin = 2;
int mfioPin = 3;
// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);
bioData body;


//RGB LED 1
int redPin = 9;
int greenPin = 6;
int bluePin = 5;
int delayChange = 5;  // in ms, how long to delay each loop of brightness change
int loopDelta = 1;    // how much to change brightness each loop
int ledLow = 0;       // lowest value for each LED
int ledHigh = 150;    // highest value for each color
int invertColor(int color) {
  return (color * -1) + 255;
}
int brightness = 220;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

unsigned long startTime = micros();
float xA, yA, zA;
float xG, yG, zG;
float HR, BO, conf, state; 
float TotalA;
char buffer[200];
int16_t data[6];

// DELTA ARRAYS
int16_t Xacceldelta[3];
int16_t Yacceldelta[3];
int16_t Zacceldelta[3];
int16_t TotalAccelDelta[3]; 
int16_t Xgyrodelta[3];
int16_t Ygyrodelta[3];
int16_t Zgyrodelta[3];
int timebetweenhits[2] = {0,0}; 
int16_t latestStrikeForce[2] = {0,0};
int16_t latestStrikeSpeed[2] = {0,0};

float latestStrikeForceFloat;
float latestStrikeSpeedFloat; 

float averageStrikeForce ; 
float averageStrikeSpeed ; 

float toAccel = 0.00479;
float toGyro = 2000 / 32768;

//Button settings for start/stop recording
int LEDState = 0;
int LEDPin = 13;
int buttonPin = 12;
int buttonNew;
int buttonOld = 1;
int dt = 100;

int strikeCounter = 0; 
unsigned long lastrecordedTime; 
unsigned long elapsedTime; 

BLEService customService("1102");
//BLEFloatCharacteristic ts("2101", BLERead | BLENotify);
BLEFloatCharacteristic ts("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic ax("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify);
BLEFloatCharacteristic ay("19B10001-E8F2-537E-4F6C-D104768A1216", BLERead | BLENotify);
BLEFloatCharacteristic az("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLENotify);
BLEFloatCharacteristic gx("19B10001-E8F2-537E-4F6C-D104768A1218", BLERead | BLENotify);
BLEFloatCharacteristic gy("19B10001-E8F2-537E-4F6C-D104768A1219", BLERead | BLENotify);
BLEFloatCharacteristic gz("19B10001-E8F2-537E-4F6C-D104768A1210", BLERead | BLENotify);

void setup() {
  startTime = micros();
  
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(redPin, HIGH);
  digitalWrite(bluePin, HIGH);
  digitalWrite(greenPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);
  //HR sensor setup
  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!!!");

  Serial.println("Configuring Sensor....");
  int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings.
  if (error == 0) { // Zero errors!
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: ");
    Serial.println(error);
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up.
  Serial.println("Loading up the buffer with data....");
  //HR sensor setup end


  while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU set up");
  BLESetup();
  Serial.println("BLE set up");
  //  Serial.println("t,xA,yA,zA,xG,yG,zG");
  Serial.println(ax.uuid());
  Serial.println(ay.uuid());
  Serial.println(az.uuid());
  Serial.println(gx.uuid());
  Serial.println(gy.uuid());
  Serial.println(gz.uuid());
}




void loop() {
BLEDevice central = BLE.central();
  if (!central){
  analogWrite(redPin, 220);
  analogWrite(bluePin, 256);
  analogWrite(greenPin, 256);
  BLEDevice central = BLE.central();
  checkButtonState();
  }
  if (central &&  LEDState == 0) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    Serial.println(central.localName());
    digitalWrite(LED_BUILTIN, HIGH);
    analogWrite(redPin, 256);
    analogWrite(bluePin, 220);
    analogWrite(greenPin, 256);
    checkButtonState();
  }
  if (central && LEDState == 1) {
    
    digitalWrite(LED_BUILTIN, HIGH);
    analogWrite(redPin, 256);
    analogWrite(bluePin, 256);
    analogWrite(greenPin, 220);
    readIMU();
    Xacceldelta[0] = data[0];
    Yacceldelta[0] = data[1];
    Zacceldelta[0] = data[2];
    Xgyrodelta[0] = data[3];
    Ygyrodelta[0] = data[4];
    Zgyrodelta[0] = data[5];
    TotalAccelDelta[0] = sqrt(sq(data[0])+sq(data[1])+ sq(data[2])); 
    //Read and save IMU values to second slot in Delta arrays
    readIMU();
    Xacceldelta[1] = data[0];
    Yacceldelta[1] = data[1];
    Zacceldelta[1] = data[2];
    Xgyrodelta[1] = data[3];
    Ygyrodelta[1] = data[4];
    Zgyrodelta[1] = data[5];
    TotalAccelDelta[1] = sqrt(sq(data[0])+sq(data[1])+ sq(data[2])); 
    readIMU();
    Xacceldelta[2] = data[0];
    Yacceldelta[2] = data[1];
    Zacceldelta[2] = data[2];
    Xgyrodelta[2] = data[3];
    Ygyrodelta[2] = data[4];
    Zgyrodelta[2] = data[5];
    TotalAccelDelta[2] = sqrt(sq(data[0])+sq(data[1])+ sq(data[2])); 
    checkButtonState();
  }
  while (central.connected() && LEDState == 1) {  
    analogWrite(redPin, 256);
    analogWrite(bluePin, 256);
    analogWrite(greenPin, 220); 
    unsigned long elapsedTime = micros() - startTime; 
    
    for (int i = 0; i <= 300; i++) {
      unsigned long lastrecordedTime = elapsedTime ;
      unsigned long elapsedTime = micros() - startTime; 
      readIMU();
      //updating BLE values
      
      ax.writeValue(xA);      ay.writeValue(yA);      az.writeValue(zA);
      gx.writeValue(xG);      gy.writeValue(yG);      gz.writeValue(zG);
      Xacceldelta[2] = Xacceldelta[1];
      Yacceldelta[2] = Xacceldelta[1];
      Zacceldelta[2] = Xacceldelta[1];
      Xgyrodelta[2] = Xacceldelta[1];
      Ygyrodelta[2] = Xacceldelta[1];
      Zgyrodelta[2] = Xacceldelta[1];
      TotalAccelDelta[2] = TotalAccelDelta[1]; 

      Xacceldelta[1] = Xacceldelta[0];
      Yacceldelta[1] = Xacceldelta[0];
      Zacceldelta[1] = Xacceldelta[0];
      Xgyrodelta[1] = Xacceldelta[0];
      Ygyrodelta[1] = Xacceldelta[0];
      Zgyrodelta[1] = Xacceldelta[0];
      TotalAccelDelta[1] = TotalAccelDelta[0]; 

      Xacceldelta[0] = data[0];
      Yacceldelta[0] = data[1];
      Zacceldelta[0] = data[2];
      Xgyrodelta[0] = data[3];
      Ygyrodelta[0] = data[4];
      Zgyrodelta[0] = data[5];
      TotalAccelDelta[0] = sqrt(sq(data[0])+sq(data[1])+ sq(data[2])); 
  

      if ( (TotalAccelDelta[1] >= 5000) && (TotalAccelDelta[0] < TotalAccelDelta[1]) && (TotalAccelDelta[1] > TotalAccelDelta[2]) ) {
         
                  timebetweenhits[0]=timebetweenhits[1];
                  timebetweenhits[1] = elapsedTime; 

                
                if ((timebetweenhits[1]-timebetweenhits[0])>=60000) {
                  strikeCounter = strikeCounter + 1 ; 
                  latestStrikeForce [0] = latestStrikeForce [1];
                  latestStrikeForce [1] = TotalAccelDelta[1]*toAccel ; 
                  latestStrikeForceFloat = latestStrikeForce [1];
                  latestStrikeSpeed [0] = latestStrikeSpeed [1]; 
                  latestStrikeSpeed [1] = ((latestStrikeForce [1])*(elapsedTime - lastrecordedTime)/1000000); 
                  latestStrikeSpeedFloat = latestStrikeSpeed [1];
                  averageStrikeForce  =  (averageStrikeForce + (latestStrikeForceFloat))/2  ;
                  averageStrikeSpeed  =  (averageStrikeSpeed + (latestStrikeSpeedFloat))/2 ;     
                   }

                 if ((timebetweenhits[1]-timebetweenhits[0]<=60000)&&((latestStrikeForceFloat*toAccel) < (TotalAccelDelta[1]*toAccel))) {
                      latestStrikeForce [0] = latestStrikeForce [1];
                      latestStrikeForce[1] = TotalAccelDelta[1]*toAccel ; 
                      latestStrikeForceFloat = latestStrikeForce [1];
                      latestStrikeSpeed [0] = latestStrikeSpeed [1]; 
                      latestStrikeSpeed [1] = ((latestStrikeForce [1])*(elapsedTime - lastrecordedTime)/1000000);    
                      latestStrikeSpeedFloat = latestStrikeSpeed [1];
                      averageStrikeForce =  ((averageStrikeForce + (latestStrikeForceFloat))/2);
                      averageStrikeSpeed  =  ((averageStrikeSpeed + (latestStrikeSpeedFloat))/2 );                  
                 }
            }
            //  sprintf(buffer,"$,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f, %.2f", elapsedTime, data[0], data[1], data[2], data[3], data[4], data[5],strikeCounter , latestStrikeForce[1],latestStrikeSpeed [1], averageStrikeForce, state, HR,BO,conf );
           //   Serial.println(buffer);
              sprintf(buffer," %.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.2f,%.2f, %.2f ,%.2f , %.2f, %.2f", xA, yA, zA, xG, yG, zG,strikeCounter,latestStrikeSpeedFloat,averageStrikeSpeed, latestStrikeForceFloat,averageStrikeForce,state,HR,BO,conf);
              Serial.println(buffer);
           
      }

      readHR(); 
      checkButtonState();

    }

  }
 

void BLESetup() {
  if (!BLE.begin()) {
    Serial.println("BLE failed to Initiate");
    delay(500);
    while (1);
  }

  BLE.setLocalName("Arduino IMU2");
  Serial.println("BLE Address");
  Serial.println(BLE.address());
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(ts);
  customService.addCharacteristic(ax); customService.addCharacteristic(ay); customService.addCharacteristic(az);
  customService.addCharacteristic(gx); customService.addCharacteristic(gy); customService.addCharacteristic(gz);
  BLE.addService(customService);
  ts.writeValue(0);
  ax.writeValue(0); ay.writeValue(0); az.writeValue(0);
  gx.writeValue(0); gy.writeValue(0); gz.writeValue(0);

  BLE.advertise();
  Serial.println("Bluetooth device is now active, waiting for connections...");
}

void readIMU() {
  unsigned long elapsedTime = micros() - startTime;
  IMU.readRegisters(0x22, (uint8_t*)data, sizeof(data));
  xA = data[0];
  yA = data[1];
  zA = data[2];
  xG = data[3];
  yG = data[4];
  zG = data[5];
 // TotalA = sqrt(sq(xA)+sq(yA)+ sq(zA)); 
  //  sprintf(buffer, "%d , %d , %d , %d , %d , %d , %d",elapsedTime, data[0]*toGyro, data[1]*toGyro, data[2]*toGyro, data[3]*toAccel, data[4]*toAccel, data[5]*toAccel);
 // sprintf(buffer, "%d , %d , %d , %d , %d , %d , %d", data[0], data[1], data[2], data[3], data[4], data[5]);
 // Serial.println(buffer);
}


void readHR() {
  body = bioHub.readBpm();
  delay(8);
  if(body.heartRate > 0 ){
  HR = body.heartRate;
  }
  if(body.oxygen > 0 ){
  BO = body.oxygen;
  }  
  conf = body.confidence; 
  state = body.status; 
 // Serial.println(HR); 
}

void checkButtonState() {
  buttonNew = digitalRead(buttonPin);
  if (buttonOld == 0 && buttonNew == 1) {
    if (LEDState == 0) {
      digitalWrite(LEDPin, HIGH);
      LEDState = 1;
    }
    else {
      digitalWrite(LEDPin, LOW);
      LEDState = 0;
    }
  }
  buttonOld = buttonNew;
  delay(1);
}
