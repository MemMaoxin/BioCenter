/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
//SPI define
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define VSPI_MISO   2
#define VSPI_MOSI   4
#define VSPI_SCLK   0
#define VSPI_SS     33
SPIClass * vspi = NULL;
File file;

#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "Sensors";
const char * networkPswd = "sensors123";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "255.255.255.255";
const int udpPort = 8765;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}
void setup(){
  // Initilize hardware serial:
  delay(1);
  Serial.begin(115200);
  Serial.setTimeout(1);
  // setup spi
  pinMode(VSPI_SS, OUTPUT); //VSPI SS
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  if(!SD.begin(VSPI_SS, *vspi)){
      Serial.println("Card Mount Failed");
//      return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
//      return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
//      return;
  }
//  if (SD.remove("/saved.data")) {
//    Serial.println("old data deleted");
//  }
//  file = SD.open("/saved.data", FILE_APPEND);
//  if (!file) {
//    Serial.println("file not opened!");
//  }
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
//  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}


#define CACHESIZE 1024
void loop(){
  //only send data when connected
  if (Serial.available()==0) {
    delay(2);
    return;
  }
  char cache[CACHESIZE];
  int n = Serial.readBytes(cache, CACHESIZE);
  //Serial.printf("Serial.readBytes => %d\n", n);
  if (n == 0) {
    return;
  }
  // write file
  File file = SD.open("/saved.data", FILE_APPEND);
  if(file){
    int writed = file.write((const uint8_t*)cache, n);
    file.close();
    //Serial.printf("file.write => %d\n", writed);
  } else {
    //Serial.println("file not opened!");
  }

  // send data
  if(connected){
    //Send a packet
    udp.beginPacket(udpAddress,udpPort);
    int writed = udp.write((const uint8_t*)cache, n);
    //Serial.printf("udp.write => %d\n", writed);
    udp.endPacket();
  } else {
    //Serial.println("wifi disconnected!");
  }
}
