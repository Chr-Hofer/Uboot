#include <HardwareSerial.h>
#include "SerialTransfer.h"

HardwareSerial ser(2);
SerialTransfer tr_cam;
SerialTransfer tr_ard;
uint16_t switchInput;
uint8_t lastError = 0;

struct controls {
  uint8_t thrust;
  uint8_t pitch;
  uint8_t yaw;
  uint8_t pump;
} ctrl;

struct sensors {
  uint16_t pressure = 100;
  uint16_t waterFront = 1000;
  uint16_t waterBack = 1000;
} sens;

void setup() {
  Serial.begin(115200);
  tr_ard.begin(Serial);
  
  ser.begin(2000000,SERIAL_8N1,16,17);
  tr_cam.begin(ser);
}

void loop() {
  processData();
}

void processData(){
  uint32_t timer;
  
  while(true){
    while(!((digitalRead(4) == 1)));
    timer = micros();
    getInputSignals();

    tr_ard.txObj(ctrl,0);
    tr_ard.sendData(sizeof(ctrl));
    if(tr_ard.available())
      tr_ard.rxObj(sens,0);


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
    
    while(micros()-timer < 99900);
    
  }
}

void getInputSignals(){
  uint16_t timer0 = micros();
  uint8_t cnt = 0;
  bool detected[4] = {false};
  uint16_t times[4];
  
  while(cnt != 4){
    if(!(digitalRead(4)) and !(detected[0])){
      cnt++;
      detected[0] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.thrust = map(tdiff,999,1985,0,180);
    }
    if(!(digitalRead(5)) and !(detected[1])){
      cnt++;
      detected[1] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.pitch = map(tdiff,999,2000,0,180);
    }
    if(!(digitalRead(18)) and !(detected[2])){
      cnt++;
      detected[2] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.yaw = map(tdiff,999,2000,0,180);
    }
    if(!(digitalRead(19)) and !(detected[3])){
      cnt++;
      detected[3] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.pump = 1 - (tdiff < 1400) + (tdiff > 1600);
    }
  }
}
