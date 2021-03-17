#include <Servo.h>
#include "SerialTransfer.h"
#define PIN_THRUST 5
#define PIN_PITCH 9
#define PIN_YAW 3
#define PIN_WP A0
#define PIN_WF A1
#define PIN_WB A2
#define PIN0_RELAIS 12
#define PIN1_RELAIS 13

Servo thrust;
Servo yaw;
Servo pitch;

SerialTransfer tr;
uint8_t lastError = 0;
uint16_t timer = 0;
bool waterDetected = false;

struct controls {
  uint8_t thrust = 90;
  uint8_t pitch = 90;
  uint8_t yaw = 90;
  uint8_t pump = 0;
} ctrl;

struct sensors {
  uint16_t pressure;
  uint16_t waterFront;
  uint16_t waterBack;
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
  processData();
}

void processData(){
  uint16_t sensWF_buff[4] = {800,800,800,800};
  uint16_t sensWB_buff[4] = {0};
  uint16_t movingAvgCounterWF = 3200;
  uint16_t movingAvgCounterWB = 0;
  uint8_t buffIndex = 0;
  
  while(true){
    uint32_t timer0 = micros();
    uint16_t res = 0;
    uint8_t cnt = 0;

    for(cnt = 0; cnt < 16; cnt++){
      res += analogRead(PIN_WP);
      delay(4);
    }
      
    sens.pressure = res/cnt;
    sens.waterFront = analogRead(PIN_WF);
    sens.waterBack = analogRead(PIN_WB);

    if(tr.available())
      tr.rxObj(ctrl,0);
    tr.txObj(sens,0);
    tr.sendData(sizeof(sens));

    movingAvgCounterWF -= sensWF_buff[buffIndex];
    movingAvgCounterWB -= sensWB_buff[buffIndex];

    sensWF_buff[buffIndex] = sens.waterFront;
    sensWB_buff[buffIndex] = sens.waterBack;

    movingAvgCounterWF += sensWF_buff[buffIndex];
    movingAvgCounterWB += sensWB_buff[buffIndex];

    buffIndex = ++buffIndex%4;

    if(movingAvgCounterWF < 500 or movingAvgCounterWB > 1600){
      if(waterDetected){
        //send packet
        //set pump to 2
      }
      else {
        if(timer){
          if(millis()-timer > 10000)
            waterDetected = true;
        }
        else {
          //encountered unexpected behaviour
          timer = millis();
        }
      }
    }
      

    thrust.write(ctrl.thrust);
    pitch.write(ctrl.pitch);
    yaw.write(ctrl.yaw);
    
    digitalWrite(PIN0_RELAIS,(ctrl.pump > 1));
    digitalWrite(PIN1_RELAIS,(ctrl.pump == 1));

    /*
    Serial.print(ctrl.thrust);
    Serial.print(" ,");
    Serial.print(ctrl.pitch);
    Serial.print(" ,");
    Serial.print(ctrl.yaw);
    Serial.print(" ,");
    Serial.println(ctrl.pump);
    */
    
    /*
    Serial.print(sens.pressure);
    Serial.print(" ,");
    Serial.print(sens.waterFront);
    Serial.print(" ,");
    Serial.println(sens.waterBack);
    */

    while(micros()-timer0 < 100000);
  }
}
