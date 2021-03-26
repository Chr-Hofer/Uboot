#include "HardwareSerial.h"
#include "SerialTransfer.h"

#define PIN_THRUST 4
#define PIN_PITCH 5
#define PIN_YAW 18
#define PIN_PUMP 19
#define PIN_MOTOR 21
#define UART2_RX 16
#define UART2_TX 17

HardwareSerial ser(2);
SerialTransfer tr_cam;
SerialTransfer tr_ard;

bool mod = true;
bool newImage = false;

uint8_t lastError = 0;
uint8_t payloadSize = MAX_PACKET_SIZE - 1;
uint8_t imData0[32768];
uint8_t imData1[32768];

struct imgDescriptor {
  uint16_t start;
  uint16_t totalSize;
  uint8_t lSize;
  uint8_t packetAmount;
  uint32_t finish;
} descriptor;

struct controls {
  uint8_t thrust;
  uint8_t pitch;
  uint8_t yaw;
  uint8_t pump;
  uint8_t motor;
} ctrl;

struct sensors {
  uint16_t pressure = 100;
  uint16_t waterFront = 1000;
  uint16_t waterBack = 1000;
} sens;

void setup() {
  Serial.begin(115200);
  tr_ard.begin(Serial);
  
  ser.begin(2000000,SERIAL_8N1,UART2_RX,UART2_TX);
  tr_cam.begin(ser);
  
  //xTaskCreatePinnedToCore(processData,"proc",16384,NULL,0,NULL,0);
}

void loop() {
  processImages(NULL);
}

void processImages(void * params){
  while(true){
    receiveImages();
  }
}

void receiveImages(){
  while(true){
    if(tr_cam.available()){
      tr_cam.rxObj(descriptor,0);
      
      if((descriptor.start == 0xFFFF) and (descriptor.finish == 0xF0F0F0F0) and (descriptor.totalSize <= 32000)){
        Serial.println();
        uint32_t timer = micros();
        uint8_t packetID = 0;
        uint8_t * imgPtr;
        
        if(mod){
          imgPtr = &imData0[0];
          mod = false;
        }
        else {
          imgPtr = &imData1[0];
          mod = true;
        }
        
        uint32_t total = 0;
        uint32_t t1 = 0;
        uint32_t t2 = 0;
        
        while(packetID != descriptor.packetAmount){

          //
          t1 = micros();
          while(!(tr_cam.available()));
          t2 = micros();
          total = total + (t2 - t1);
          //
          
          if(tr_cam.packet.rxBuff[0] != packetID){
            
            Serial.println("Packet ID error: ");
            Serial.print("Expected: ");
            Serial.println(packetID);
            Serial.print("Got:      ");
            Serial.println(tr_cam.packet.rxBuff[0]);
          
            return;
	  }

          memcpy(imgPtr,&tr_cam.packet.rxBuff[1],payloadSize);
          imgPtr += payloadSize;
          packetID++;
        }

        //
        t1 = micros();
        while(!(tr_cam.available()));
        t2 = micros();
        total = total + (t2 - t1);
        //
        
        if(tr_cam.packet.rxBuff[0] != packetID){
          
          Serial.println("Packet ID error: ");
          Serial.print("Expected: ");
          Serial.println(packetID);
          Serial.print("Got:      ");
          Serial.println(tr_cam.packet.rxBuff[0]);
        
          return;
        }

        memcpy(imgPtr,&tr_cam.packet.rxBuff[1],descriptor.lSize);

        uint32_t timer0 = micros()-timer;

        Serial.print("Transmission took: ");
        Serial.println(timer0);
        Serial.print("Avg. waiting : ");
        Serial.println(total/descriptor.packetAmount);
        Serial.println();
        
        newImage = true;
      }
    }
  }
}

void processData(void * params){
  Serial.println("Data");
  Serial.println(xPortGetCoreID());
  uint32_t timer;
  
  while(true){
    while(!digitalRead(PIN_THRUST));
    timer = micros();
    getInputSignals();

    tr_ard.txObj(ctrl,0);
    tr_ard.sendData(sizeof(ctrl));
    if(tr_ard.available())
      tr_ard.rxObj(sens,0);

    delay(90);
    uint32_t t = micros();
    while(micros()-timer < 99900);
    Serial.print("Free time: ");
    Serial.println(micros()-t);
  }
}

void getInputSignals(){
  uint16_t timer0 = micros();
  uint8_t cnt = 0;
  bool detected[5] = {false};
  
  while(cnt != 5){
    if(!digitalRead(PIN_THRUST) and !detected[0]){
      cnt++;
      detected[0] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.thrust = map(tdiff,999,1985,0,180);
    }
    if(!digitalRead(PIN_PITCH) and !detected[1]){
      cnt++;
      detected[1] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.pitch = map(tdiff,999,2000,0,180);
    }
    if(!digitalRead(PIN_YAW) and !detected[2]){
      cnt++;
      detected[2] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.yaw = map(tdiff,999,2000,0,180);
    }
    if(!digitalRead(PIN_PUMP) and !detected[3]){
      cnt++;
      detected[3] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.pump = 1 - (tdiff < 1400) + (tdiff > 1600);
    }
    if(!digitalRead(PIN_MOTOR) and !detected[4]){
      cnt++;
      detected[4] = true;
      uint16_t tdiff = micros()-timer0;
      ctrl.motor = 1 - (tdiff > 1600);
  }
}

void debug(){
  Serial.print(ctrl.thrust);
  Serial.print(" ,");
  Serial.print(ctrl.pitch);
  Serial.print(" ,");
  Serial.print(ctrl.yaw);
  Serial.print(" ,");
  Serial.println(ctrl.pump);

  Serial.print(sens.pressure);
  Serial.print(" ,");
  Serial.print(sens.waterFront);
  Serial.print(" ,");
  Serial.println(sens.waterBack);
}