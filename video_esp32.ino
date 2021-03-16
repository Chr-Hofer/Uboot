#include <HardwareSerial.h>
#include "SerialTransfer.h"

HardwareSerial ser(2);
SerialTransfer tr;
uint8_t payloadSize = MAX_PACKET_SIZE - 1;

uint8_t imData[32000];

struct imgDescriptor {
  uint16_t start;
  uint16_t totalSize;
  uint8_t lSize;
  uint8_t packetAmount;
} descriptor;

void setup() {
  Serial.begin(115200);
  ser.begin(2000000,SERIAL_8N1,16,17);
  tr.begin(ser);
}

void loop() {
  processImages();
}


void processImages(){
  while(true){
    if(tr.available()){
      tr.rxObj(descriptor,0);
      
      if(descriptor.start == 0xFFFF and descriptor.totalSize <= 32000){
        uint32_t timer = micros();
        uint8_t packetID = 0;
        uint8_t * imgPtr = &imData[0];

        uint32_t total = 0;
        uint32_t t1 = 0;
        uint32_t t2 = 0;
        
        while(packetID != descriptor.packetAmount){

          //
          t1 = micros();
          while(!(tr.available()));
          t2 = micros();
          total = total + (t2 - t1);
          //
          
          if(tr.packet.rxBuff[0] != packetID){
            Serial.println("Packet ID error: ");
            Serial.print("Expected: ");
            Serial.println(packetID);
            Serial.print("Got:      ");
            Serial.println(tr.packet.rxBuff[0]);
            return;  }
          memcpy(imgPtr,&tr.packet.rxBuff[1],payloadSize);
          imgPtr += payloadSize;
          packetID++;
        }

        //
        t1 = micros();
        while(!(tr.available()));
        t2 = micros();
        total = total + (t2 - t1);
        //
        
        if(tr.packet.rxBuff[0] != packetID){
          Serial.println("Packet ID error: ");
          Serial.print("Expected: ");
          Serial.println(packetID);
          Serial.print("Got:      ");
          Serial.println(tr.packet.rxBuff[0]);
          return;}
        memcpy(imgPtr,&tr.packet.rxBuff[1],descriptor.lSize);

        uint32_t timer0 = micros()-timer;

        Serial.println("Received image");
        Serial.print("Avg. waiting : ");
        Serial.println(total/descriptor.packetAmount);
        Serial.print("Transmission took: ");
        Serial.println(timer0);
      }
    }
  }
}

void processData(){
  while(true){
    
  }
}
