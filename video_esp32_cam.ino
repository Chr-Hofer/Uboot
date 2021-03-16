#include "esp_camera.h"
#include "SerialTransfer.h"

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 10,
    .fb_count = 1
};

SerialTransfer tr;

uint8_t payloadSize = MAX_PACKET_SIZE - 1;

camera_fb_t * pic0 = NULL;
camera_fb_t * pic1 = NULL;

bool mod = true;
bool newImage = false;

struct imgDescriptor {
  uint16_t start;
  uint16_t totalSize;
  uint8_t lSize;
  uint8_t packetAmount;
} descriptor;

void setup() {
  Serial.begin(2000000);
  tr.begin(Serial);
  descriptor.start = 0xFFFF;
  
  init_camera();
  pic0 = esp_camera_fb_get();
  
  xTaskCreatePinnedToCore(sendPic,"send",64000,NULL,0,NULL,0);
}

void loop() {
  getPic();
}

void getPic(){
  while(true){
    while(!newImage){
      delay(1);
    }
    newImage = false;
    //Serial.println();
    //Serial.print("Taking pic on ");
    //Serial.println(xPortGetCoreID());
    if(mod){
      //Serial.println("Writing to pic1");
      pic1 = esp_camera_fb_get();
      mod = false;
    }
    else {
      //Serial.println("Writing to pic0");
      pic0 = esp_camera_fb_get();
      mod = true;
    }
    //Serial.println();
  }
}

void sendPic(void * params){
  while(true){
    uint32_t t = micros();
    //Serial.print("Sending pic on ");
    //Serial.print(xPortGetCoreID());
    newImage = true;
    camera_fb_t * pic;
    if(mod){
      //Serial.println("Reading pic0");
      pic = pic0;}
    else{
      //Serial.println("Reading pic1");
      pic = pic1;}
    //Serial.println();
      
    if(pic){
      uint16_t picIndex = 0;
      uint16_t picSize = pic->len;
      uint8_t * imPtr = pic->buf;
      uint8_t lastSize = picSize % payloadSize;
      uint8_t numPackets = picSize/payloadSize;
  
      descriptor.totalSize = picSize;
      descriptor.lSize = lastSize;
      descriptor.packetAmount = numPackets;
      tr.txObj(descriptor,0);
      tr.sendData(sizeof(descriptor));
  
      for(uint8_t packetID = 0; packetID < numPackets; packetID++){
        tr.packet.txBuff[0] = packetID;
        memcpy(&tr.packet.txBuff[1],imPtr,payloadSize);
        imPtr += payloadSize;
        tr.sendData(MAX_PACKET_SIZE);
        uint32_t timer0x = micros();
        while(micros()-timer0x < 1100-packetID*11);
      }
  
      tr.packet.txBuff[0] = numPackets;
      memcpy(&tr.packet.txBuff[1],imPtr,lastSize);
      tr.sendData(lastSize+1);
      Serial.println();
      esp_camera_fb_return(pic);

      /*
      Serial.println("Descriptor:");
      Serial.print("Total Size: ");
      Serial.println(descriptor.totalSize);
      Serial.print("lSize:      ");
      Serial.println(descriptor.lSize);
      Serial.print("Packet #:   ");
      Serial.println(descriptor.packetAmount);
      Serial.println();
      */
  
      //while(micros()-t < 1000000);
    }
  }
}

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        //TODO: IMPLEMENT ERROR PACKET TRANSMISSION
        //Serial.println("Camera Init Failed");
        return err;
    }

    return ESP_OK;
}