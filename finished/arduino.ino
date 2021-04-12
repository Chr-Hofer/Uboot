//TODO: 
// COUNTER FOR PUMP TIME
// EXCEPTION FUNC

/* EXCEPTION NUMS:
 * 1: Possibly water detected
 * 2: Water detected
 * 3: Maximum depth reached
 * 4: Communication problem
 * 5: Timeout detected
 * 6: Minimum battery voltage detected
 */

#include "Servo.h"
#include "SerialTransfer.h"

#define PIN_WP A0
#define PIN_WF A1
#define PIN_WB A2
#define PIN_VOLT A3
#define PIN_MOTOR 8
#define PIN_THRUST 9
#define PIN_PITCH 10
#define PIN_YAW 11
#define PIN0_RELAIS 12
#define PIN1_RELAIS 13

#define WPmax 1170
#define WFmax 500
#define WBmax 1600
#define voltMin 2560

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

uint16_t sensWP_buff[4] = {105,105,105,105};
uint16_t sensWF_buff[4] = {800,800,800,800};
uint16_t sensWB_buff[4] = {0};
uint16_t sensVOLT_buff[4] = {870,870,870,870};
uint16_t movingAvgCounterWP = 420;
uint16_t movingAvgCounterWF = 3200;
uint16_t movingAvgCounterWB = 0;
uint16_t movingAvgCounterVOLT = 3480;
uint16_t batteryVoltage = 870;
uint8_t buffIndex = 0;

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
} sens;

void setup() {
  Serial.begin(115200);
  tr.begin(Serial);

  pinMode(PIN0_RELAIS,OUTPUT);
  pinMode(PIN1_RELAIS,OUTPUT);

  thrust.attach(PIN_THRUST);
  pitch.attach(PIN_PITCH);
  yaw.attach(PIN_YAW);

  thrust.write(90);
  pitch.write(90);
  yaw.write(90);
  
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
  uint16_t res = 0;
  uint8_t cnt = 0;

  for(cnt = 0; cnt < 16; cnt++){
    res += analogRead(PIN_WP);
    delay(4);
  }
    
  sens.pressure = res/cnt;
  sens.waterFront = analogRead(PIN_WF);
  sens.waterBack = analogRead(PIN_WB);
  sens.batteryLevel = batteryVoltage;
  batteryVoltage = analogRead(PIN_VOLT);

  if(tr.available()){
    tr.rxObj(ctrl,0);
    timeoutTimer = 0;
  }
  else {
    if(timeoutTimer){
      if(millis()-timeoutTimer > 10000){
        timeoutDetected = true;
        timeoutTimer = 0;
        exception.exceptionNum = 5;
        tr.txObj(exception,0);
        tr.sendData(sizeof(exception));
      }
    }
    else {
      exception.exceptionNum = 4;
      tr.txObj(exception,0);
      tr.sendData(sizeof(exception));
      timeoutTimer = millis();
    }
  }
  tr.txObj(sens,0);
  tr.sendData(sizeof(sens));
}

void securityHandler(){
  movingAvgCounterWP -= sensWP_buff[buffIndex];
  movingAvgCounterWF -= sensWF_buff[buffIndex];
  movingAvgCounterWB -= sensWB_buff[buffIndex];
  movingAvgCounterVOLT -= sensVOLT_buff[buffIndex];

  sensWP_buff[buffIndex] = sens.pressure;
  sensWF_buff[buffIndex] = sens.waterFront;
  sensWB_buff[buffIndex] = sens.waterBack;
  sensVOLT_buff[buffIndex] = sens.batteryLevel

  movingAvgCounterWP += sensWP_buff[buffIndex];
  movingAvgCounterWF += sensWF_buff[buffIndex];
  movingAvgCounterWB += sensWB_buff[buffIndex];
  movingAvgCounterVOLT += sensVOLT_buff[buffIndex];

  buffIndex = ++buffIndex%4;
    
  if(movingAvgCounterWF < WFmax or movingAvgCounterWB > WBmax){
    if(timer){
      if(millis()-timer > 8000){
        waterDetected = true;
        timer = 0;
        exception.exceptionNum = 2;
        tr.txObj(exception,0);
        tr.sendData(sizeof(exception));
      }
    }
    else {
      exception.exceptionNum = 1;
      tr.txObj(exception,0);
      tr.sendData(sizeof(exception));
      timer = millis();
    }
  }

  if(movingAvgCounterWP > WPmax and !messageSent){
    messageSent = true;
    maxDepthReached = true;
    exception.exceptionNum = 3;
    tr.txObj(exception,0);
    tr.sendData(sizeof(exception));
  }

  if(movingAvgCounterVOLT < voltMin){
    lowVoltDetected = true;
    exception.exceptionNum = 6;
    tr.txObj(exception,0);
    tr.sendData(sizeof(exception));
  }

  uint16_t batteryRes = movingAvgCounterVOLT/4;
  if(batteryRes < batteryVoltage){
    batteryVoltage = batteryRes; 
  }
}

void peripheralHandler(){
  thrust.write(ctrl.thrust);
  pitch.write(ctrl.pitch);
  yaw.write(ctrl.yaw);

  if(maxDepthReached or waterDetected or timeoutDetected){
    if(sens.pressure > 112){
      digitalWrite(PIN0_RELAIS,HIGH);
      delay(5);
      digitalWrite(PIN1_RELAIS,LOW);
    }
    else {
      maxDepthReached = false;
      waterDetected = false;
      messageSent = false;
    }
  }
  else {
    digitalWrite(PIN0_RELAIS,(ctrl.pump > 1));
    delay(5);
    digitalWrite(PIN1_RELAIS,(ctrl.pump == 1));
  }
  digitalWrite(PIN_MOTOR,ctrl.motor);
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
