//TODO: 
// COUNTER FOR PUMP TIME (TEST WHETHER WATER I/O RATE IS CONSTANT IN THE FIRST PLACE)
// IMPLEMENT TEMPERATURE MEASUREMENT SYSTEM
// EEPROM STAT TRACKING
// CHANGE MOVING AVERAGE SYSTEM TO PRECALCULATE VALS AND SEND AS SENSOR DATA (NOT THE CURRENT MEASUREMENT)

/* 
 * EXCEPTION NUMS:
 * 0: Possibly water detected
 * 1: Water detected
 * 2: Maximum depth reached
 * 3: Communication problem
 * 4: Timeout detected
 * 5: Minimum battery voltage detected
 * 6: Possibly temperature error
 * 7: Temperature error
*/

#include "Servo.h"
#include "SerialTransfer.h"

#define PRESSURE 0
#define WATERF 1
#define WATERB 2
#define VOLTAGE 3
#define TEMP 4

#define PIN_WP 0
#define PIN_WF 1
#define PIN_WB 2
#define PIN_VOLT 3
#define PIN_TEMP 4

#define PIN_SWITCH 6
#define PIN_LED 7
#define PIN_MOTOR 8
#define PIN_THRUST 9
#define PIN_PITCH 10
#define PIN_YAW 11
#define PIN0_RELAIS 12
#define PIN1_RELAIS 13

#define WPmax 1170
#define WFmax 500
#define WBmax 1600
#define WP_STOP 112
#define voltMin 2560

#define timeoutWater 10000
#define timeoutCom 45000
#define timeoutTemp 25000

Servo thrust;
Servo yaw;
Servo pitch;

SerialTransfer tr;

uint16_t timer = 0;
uint32_t timer0 = 0;

uint16_t timeoutTimer = 0;
uint32_t startTime = millis();

bool waterDetected = false;
bool timeoutDetected = false;
bool maxDepthReached = false;
bool lowVoltDetected = false;

bool messageSent = false;
bool emergencySurfacing = false;

uint16_t sensorTable[4][5] = {0};
uint16_t movingAverages[5] = {0};
uint16_t movingAverageCounters[5] = {0};

uint8_t buffIndex = 0;

enum exceptions {
  possibly_water_detected,
  water_detected,
  max_depth_reached,
  com_problem,
  timeout_detected,
  min_volt_reached,
  possibly_temp_error,
  temp_error
};

struct exception {
  uint16_t start = 0xF0F0;
  uint8_t exceptionNum;
  uint16_t finish = 0x0F0F;
} exception;

struct controls {
  uint8_t thrust = 90;
  uint8_t pitch = 90;
  uint8_t yaw = 90;
  uint8_t pump = 0;
  uint8_t motor = 0;
} ctrl;

struct sensors {
  uint16_t pressure;
  uint16_t waterFront;
  uint16_t waterBack;
  uint16_t batteryLevel;
  uint16_t temperature;
} sens;

void setup() {
  Serial.begin(115200);
  tr.begin(Serial);
  
  pinMode(PIN_SWITCH,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_MOTOR,OUTPUT);
  pinMode(PIN_THRUST,OUTPUT);
  pinMode(PIN_PITCH,OUTPUT);
  pinMode(PIN_YAW,OUTPUT);
  pinMode(PIN0_RELAIS,OUTPUT);
  pinMode(PIN1_RELAIS,OUTPUT);

  thrust.attach(PIN_THRUST);
  pitch.attach(PIN_PITCH);
  yaw.attach(PIN_YAW);

  thrust.write(90);
  pitch.write(90);
  yaw.write(90);

  initSensorTable();
  
  delay(1000);
}

void loop() {
  timer0 = micros();

  communicationHandler();
  securityHandler();
  peripheralHandler();
  
  while(micros()-timer0 < 100000);
}

void communicationHandler(){
  if(tr.available()){
    tr.rxObj(ctrl,0);
    timeoutTimer = 0;
  }
  else {
    if(timeoutTimer){
      if(millis()-timeoutTimer > timeoutCom){
        timeoutDetected = true;
        timeoutTimer = 0;
        sendExc(timeout_detected);
      }
    }
    else {
      sendExc(com_problem);
      timeoutTimer = millis();
    }
  }
  tr.txObj(sens,0);
  tr.sendData(sizeof(sens));
}

void securityHandler(){
  if(movingAvgCounterWF < WFmax or movingAvgCounterWB > WBmax){
    if(timer){
      if(millis()-timer > timeoutWater){
        waterDetected = true;
        timer = 0;
        sendExc(water_detected);
      }
    }
    else {
      sendExc(possibly_water_detected);
      timer = millis();
    }
  }

  if(movingAvgCounterWP > WPmax and !messageSent){
    messageSent = true;
    maxDepthReached = true;
    sendExc(max_depth_reached);
  }

  if(movingAvgCounterVOLT < voltMin){
    lowVoltDetected = true;
    sendExc(min_volt_reached);
  }

  uint16_t batteryRes = movingAvgCounterVOLT/4;
  if(batteryRes < batteryVoltage){
    batteryVoltage = batteryRes; 
  }
}

void peripheralHandler(){
  for(uint8_t sensor = 0; sensor < 5; sensor++){
    movingAverageCounters[sensor] -= sensorTable[sensor][buffIndex];
    sensorTable[sensor][buffIndex] = analogRead(sensor);
    movingAverageCounters[sensor] += sensorTable[sensor][buffIndex];
    movingAverages[sensor] = movingAverageCounters[sensor]/5;
  }
  
  buffIndex = ++buffIndex%4;
    
  sens.pressure = movingAverages[PRESSURE];
  sens.waterFront = movingAverages[WATERF];
  sens.waterBack = movingAverages[WATERB];
  sens.batteryLevel = movingAverages[VOLTAGE];
  sens.temperature = movingAverages[TEMP];
  
  thrust.write(ctrl.thrust);
  pitch.write(ctrl.pitch);
  yaw.write(ctrl.yaw);

  if(normal){
    digitalWrite(PIN0_RELAIS,(ctrl.pump > 1));
    delay(10);
    digitalWrite(PIN1_RELAIS,(ctrl.pump == 1));
  }
  else {
    if(maxDepthReached or waterDetected or timeoutDetected){
      if(emergencySurfacing){
        digitalWrite(PIN0_RELAIS,HIGH);
        delay(10);
        digitalWrite(PIN1_RELAIS,LOW);
      }
      else {
        maxDepthReached = false;
        waterDetected = false;
        messageSent = false;
      }
    }
  }
  digitalWrite(PIN_MOTOR,ctrl.motor);
}

void sendExc(uint8_t num){
  exception.exceptionNum = num;
  tr.txObj(exception,0);
  tr.sendData(sizeof(exception));
}

void initSensorTable(){
  for(uint8_t sensor = 0; sensor < 5; sensor++);
    for(uint8_t pos = 0; pos < 4; pos++){
      sensorTable[sensor][pos] = analogRead(sensor);
      movingAverageCounters[sensor] += sensorTable[sensor][pos];
      delay(10);
    }
}

void debug(){
  Serial.println("Received:");
  Serial.print(ctrl.thrust);
  Serial.print(" ,");
  Serial.print(ctrl.pitch);
  Serial.print(" ,");
  Serial.print(ctrl.yaw);
  Serial.print(" ,");
  Serial.println(ctrl.pump);
  
  Serial.println();
  
  Serial.println("Measured:");
  Serial.print(sens.pressure);
  Serial.print(" ,");
  Serial.print(sens.waterFront);
  Serial.print(" ,");
  Serial.println(sens.waterBack);
  
  Serial.println();
  Serial.println();
}
