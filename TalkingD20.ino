#include <Adafruit_Soundboard.h>
#include "Adafruit_TinyUSB.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#include "LSM6DS3.h"
#include "Wire.h"

//Instance of class LSM6DS3 of the nRF52840
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// Adafruit SoundBoard
#define SFX_TX 6
#define SFX_RX 7
#define SFX_RST 8

/* Note: This is software serial only for now until I figure out why this is not working.
I guess I might be missing something simple here but I'm not sure yet, what it is.  */
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

  Serial.println("starting IMU...");
  if (myIMU.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }
  Serial.println("IMU started!");
}

void loop(){
  //Serial.println("Playing..."); 
  //sfx.playTrack(1);
  //sfx.playTrack("T00     OGG");

  Serial.println("---IMU---");

  //Accelerometer
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  //Gyroscope
  Serial.print("\nGyroscope:\n");
  Serial.print(" X1 = ");
  Serial.println(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);

  delay(3000);
}