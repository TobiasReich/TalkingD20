#include <Adafruit_Soundboard.h>
#include "Adafruit_TinyUSB.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#define SFX_TX 6
#define SFX_RX 7
#define SFX_RST 8

SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);   // <---- this worked, hardware did not?!

Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);

void setup() {
  pinMode(SFX_TX, OUTPUT);
  pinMode(SFX_RX, INPUT);

  Serial.begin(115200); 
  ss.begin(9600);
  if (!sfx.reset()) {
    Serial.println("Not found. Waiting...");
    while (!ss) { /* wait until it is connected*/ }
  }
  Serial.println("SFX board found");
  delay(1000);
  uint8_t files = sfx.listFiles();
  Serial.println("File Listing");
  Serial.println("========================");
  Serial.println();
  Serial.print("Found "); Serial.print(files); Serial.println(" Files");
  for (uint8_t f=0; f<files; f++) {
    Serial.print(f); 
    Serial.print("\tname: "); Serial.print(sfx.fileName(f));
    Serial.print("\tsize: "); Serial.println(sfx.fileSize(f));
  }
  Serial.println("========================"); 
}

void loop(){
  //Serial.println("Playing..."); 
  //sfx.playTrack(1);
  //sfx.playTrack("T00     OGG");
  //delay(3000);
}