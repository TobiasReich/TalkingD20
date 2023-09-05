#include <Adafruit_Soundboard.h>
#include "Adafruit_TinyUSB.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <limits.h>  // required for LONG_MAX etc.

#include "LSM6DS3.h"
#include <Wire.h>


// ---------------- Sound ----------------
// Adafruit SoundBoard
#define SFX_TX 6
#define SFX_RX 7
#define SFX_RST 8

#define AUDIO_ACT_PIN 5  // "Act" on Audio FX -> Used for checking whether there is some audio playing

/* Note: This is software serial only for now until I figure out why this is not working.
I guess I might be missing something simple here but I'm not sure yet, what it is.  */
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);  // <---- this worked, hardware did not?!

Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);

char filename[12] = "        OGG";  // Tail end of filename NEVER changes

// string arrays are wretched, and sfx.playTrack() expects a
// goofball fixed-length space-padded filename...we take care of both by
// declaring all the filenames inside one big contiguous string
// (notice there are no commas here, it's all concatenated), and copying
// an 8-byte section as needed into filename[].  Some waste, but we're
// not hurting for space.  If you change or add any filenames, they MUST
// be padded with spaces to 8 characters, else there will be...trouble.
static const char bigStringTable[] =  // play() index
  "01      "
  "02      "
  "03      "
  "04      "  //  0- 3
  "05      "
  "06      "
  "07      "
  "08      "  //  4- 7
  "09      "
  "10      "
  "11      "
  "12      "  //  8-11
  "13      "
  "14      "
  "15      "
  "16      "  // 12-15
  "17      "
  "18      "
  "19      "
  "20      "  // 16-19
  "ANNC1   "
  "ANNC2   "
  "ANNC3   "  // 20-22
  "BAD1    "
  "BAD2    "
  "BAD3    "  // 23-25
  "GOOD1   "
  "GOOD2   "
  "GOOD3   "  // 26-28
  "STARTUP "
  "03ALT   "
  "BATT1   "
  "BATT2   ";  // 29-32


void audioOn(void) {
  //pinMode(AMP_SHUTDOWN, INPUT_PULLUP);
  pinMode(SFX_RST , INPUT_PULLUP);
  Serial.println("Audio ON (TODO)");
}

void audioOff(void) {
  //digitalWrite(AMP_SHUTDOWN, LOW);
  //pinMode(     AMP_SHUTDOWN, OUTPUT);
  digitalWrite(SFX_RST , LOW); // Hold low = XRESET (low power)
  pinMode(SFX_RST, OUTPUT);
  Serial.println("Audio OFF (TODO)");
}

/* Plays the track defined in the array above */
void play(uint16_t i) {
  Serial.print("Playing Track (TODO): ");
  Serial.println(i);

  memcpy_P(filename, &bigStringTable[i * 8], 8);

  Serial.print("Playing Filename: ");
  Serial.println(filename);

  sfx.playTrack(filename);
  delay(250); // Need this -- some delay before ACT LED is valid

  while(analogRead(AUDIO_ACT_PIN) < 500);  // Wait for sound to finish
  //while(digitalRead(AUDIO_ACT) == LOW);  // Wait for sound to finish
  Serial.print("Playing audio track finished!");
}


// ---------------- ACCELEROMETER ----------------

volatile boolean wasThrown = false;
boolean calculatingResult = false;

/** defines how different the values might be that we still consider
    The die to be stable (e.g. if the table is still shaking or the 
    die is held in a hand) */
#define STABILIZE_DELTA 0.01
#define int1Pin PIN_LSM6DS3TR_C_INT1

/* How many millis to wait, longer values allow the die
   to roll further but increase the risk of false positives */
#define STABILIZE_TIMEOUT 3000

/* How many millis does the die have to stand still before it's
   motion is considered finished. Higher values prevent false
   positives (e.g. when the die is just paused but continues to roll)
   but lets you wait longer for the result */
#define STABILIZE_THRESHOLD 200

//Instance of LSM6DS3 of the nRF52840
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A


// Waits for accelerometer output to stabilize, indicating movement stopped
boolean stabilize() {
  Serial.print("Stabilizing... ");

  uint32_t startTime, prevTime, currentTime;
  float_t prevX, prevY, prevZ;
  float_t dX, dY, dZ;

  // Get initial orientation and time
  prevX = myIMU.readFloatAccelX();
  prevY = myIMU.readFloatAccelY();
  prevZ = myIMU.readFloatAccelZ();
  prevTime = startTime = millis();

  // Then make repeated readings until orientation settles down.
  // A normal roll should only take a second or two...if things do not
  // stabilize before timeout, probably being moved or carried.
  while (((currentTime = millis()) - startTime) < STABILIZE_TIMEOUT) {
    delay(10); // no need to check this too often -> conserves some energy

    if ((currentTime - prevTime) >= STABILIZE_THRESHOLD) {
      Serial.println("Stable!");
      return true;  // Stable!
    }

    // compare with the last orientation check
    dX = myIMU.readFloatAccelX() - prevX;
    dY = myIMU.readFloatAccelY() - prevY;
    dZ = myIMU.readFloatAccelZ() - prevZ;

    // This compares the distance. sqrt() can be avoided by squaring distance
    // to be compared. i.e. X²+Y²+Z² ...
    if ((dX * dX + dY * dY + dZ * dZ) >= STABILIZE_DELTA) {  
      prevX = myIMU.readFloatAccelX();
      prevY = myIMU.readFloatAccelY();
      prevZ = myIMU.readFloatAccelZ();
      prevTime = millis();  // Reset timer for the next check
    }
  }
  Serial.println("Still rolling!");
  return false;  // Didn't stabilize, probably false trigger
}


/* Face gravity vectors (accelerometer installed FACE DOWN)
 * Note the original values for the MMA8451 were 14 bit (until 4096)
 * but since this IMU gives me -1..1 values I have to divide it by 4096 */
static const float_t gtable[20][3] = {
  { 0.362, -0.476, 0.784 },    //  1
  { -0.590, 0.804, -0.189 },   //  2
  { 0.906, -0.286 - 0.194 },   //  3
  { -0.935, -0.274, -0.224 },  //  4
  { -0.595, 0.213, 0.758 },    //  5
  { -0.012, -0.584, -0.797 },  //  6
  { 0.559, 0.187, 0.783 },     //  7
  { 0.336, 0.513, -0.787 },    //  8
  { 0.001, -0.975, -0.206 },   //  9
  { 0.561, 0.808, -0.169 },    // 10
  { -0.762, -0.778, 0.148 },   // 11
  { -0.006, 0.987, 0.203 },    // 12
  { -0.369, -0.495, 0.771 },   // 13
  { -0.592, -0.162, -0.793 },  // 14
  { -0.015, 0.603, 0.784 },    // 15
  { 0.551, -0.196, -0.781 },   // 16
  { 0.913, 0.292, 0.189 },     // 17
  { -0.932, 0.312, 0.174 },    // 18
  { 0.545, -0.793, 0.191 },    // 19
  { -0.391, 0.507, -0.789 }    // 20
};


/* Find nearest face to accelerometer reading. */
uint8_t getFace(void) {
  float dX, dY, dZ, d, dMin = INT_MAX;
  float fX, fY, fZ;
  uint8_t i, iMin = 0;

  for (i = 0; i < 20; i++) {  // For each face...
    fX = gtable[i][0];        // Read face X/Y/Z
    fY = gtable[i][1];        // from PROGMEM
    fZ = gtable[i][2];
    dX = myIMU.readFloatAccelX() - fX;  // Delta between accelerometer & face
    dY = myIMU.readFloatAccelY() - fY;
    dZ = myIMU.readFloatAccelZ() - fZ;
    d = dX * dX + dY * dY + dZ * dZ;  // Distance^2
    // Check if this face is the closest match so far.  Because
    // we're comparing RELATIVE distances, sqrt() can be avoided.
    if (d < dMin) {  // New closest match?
      dMin = d;      // Save closest distance^2
      iMin = i;      // Save index of closest match
    }
  }

  return iMin;  // Index of closest matching face
}


// ---------------- POWER SAVING STUFF ----------------

uint8_t batt = 0;  // Low battery announcement counter

// This will be required in order to disable everything until the "fall detection"
// causes an interrupt

// Battery monitoring idea adapted from JeeLabs article:
// jeelabs.org/2012/05/04/measuring-vcc-via-the-bandgap/
// Code from Adafruit TimeSquare project.
static uint16_t readVoltage() {
  /* int      i, prev;
  uint8_t  count;
  uint16_t mV;

  power_adc_enable();
  ADMUX  = _BV(REFS0) |                        // AVcc voltage reference
           _BV(MUX3)  | _BV(MUX2) | _BV(MUX1); // Bandgap (1.8V) input
  ADCSRA = _BV(ADEN)  |             // Enable ADC
           _BV(ADPS2) | _BV(ADPS1); // 1/64 prescaler (8 MHz -> 125 KHz)
  // Datasheet notes that the first bandgap reading is usually garbage as
  // voltages are stabilizing.  It practice, it seems to take a bit longer
  // than that (perhaps due to sleep).  Tried various delays, but this was
  // still inconsistent and kludgey.  Instead, repeated readings are taken
  // until four concurrent readings stabilize within 10 mV.
  for(prev=9999, count=0; count<4; ) {
    for(ADCSRA |= _BV(ADSC); ADCSRA & _BV(ADSC); ); // Start, await ADC conv.
    i  = ADC;                                       // Result
    mV = i ? (1100L * 1023 / i) : 0;                // Scale to millivolts
    if(abs((int)mV - prev) <= 10) count++;   // +1 stable reading
    else                          count = 0; // too much change, start over
    prev = mV;
  }
  ADCSRA = 0; // ADC off
  power_adc_disable();
  return mV; */
  return 3500;  // TODO: mocked for now until I check the voltage
}




// ---------------- Initialization ----------------



void setupAudio() {
  pinMode(SFX_TX, OUTPUT);
  pinMode(SFX_RX, INPUT);
  
  pinMode(AUDIO_ACT_PIN, INPUT);
  //pinMode(AUDIO_ACT_PIN, INPUT_PULLDOWN_SENSE);
  //pinMode(AUDIO_ACT_PIN, INPUT_PULLUP_SENSE);

  ss.begin(9600);
  if (!sfx.reset()) {
    Serial.println("Not found. Waiting...");
    while (!ss) { /* wait until it is connected*/
    }
  }
  Serial.println("SFX board found");
  delay(1000);
  uint8_t files = sfx.listFiles();
  Serial.println("File Listing");
  Serial.println("========================");
  Serial.println();
  Serial.print("Found ");
  Serial.print(files);
  Serial.println(" Files");
  for (uint8_t f = 0; f < files; f++) {
    Serial.print(f);
    Serial.print("\tname: ");
    Serial.print(sfx.fileName(f));
    Serial.print("\tsize: ");
    Serial.println(sfx.fileSize(f));
  }
  Serial.println("========================");
}

void setupIMU() {
  Serial.println("(re)starting IMU...");

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.gyroEnabled = 0;  // Gyro currently not used, disabled to save power

  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
  } else {
    Serial.println("IMU OK!");
  }

  setupFreeFallInterrupt();
  pinMode(int1Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(int1Pin), onFreefallDetected, RISING);

  Serial.println("IMU setup finished!");
}


void setupFreeFallInterrupt() {
  Serial.println("Setup for Free-Fall Interrupt");

  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;   // 0000 0001  200Hz
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;      // 0000 0000  2g
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;  // 0100 0000  104Hz

  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_SRC, 0b00100000);
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81);       // LATCHED
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0b00010000);  // 00010000
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0b00000000);

  /*Documentation states:
    In the LSM6DS3, the accelerometer can be configured in four different operating modes:
    power-down, low-power, normal mode and high-performance mode. The operating mode
    selected depends on the value of the XL_HM_MODE bit in CTRL6_C (15h). If
    XL_HM_MODE is set to ‘0’, high-performance mode is valid for all ODRs (from 12.5 Hz up
    to 6.66 kHz).
    To enable the low-power and normal mode, the XL_HM_MODE bit has to be set to ‘1’. Low-
    power mode is available for lower ODRs (12.5, 26, 52 Hz) while normal mode is available
    for ODRs equal to 104 and 208 Hz. */
  // Switch to low-power mode (or normal mode if 104Hz or higher)
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0b00010000);


  // p.80 ->
  // Time and threshold for the
  // 00011 : 3 events (@52Hz = 57ms)
  // 011 : 312 mg threshold
  error += myIMU.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0b00011011);  // 00110 011

  if (error) {
    Serial.println("Problem configuring the device.");
  } else {
    Serial.println("Device O.K.");
  }
}


/* Reads from the LSM6DS3_ACC_GYRO_WAKE_UP_SRC register 
   in order to "reset" the free fall trigger */
void resetFreeFallTrigger() {
  Serial.println("Resetting free-fall trigger");
  uint8_t readDataByte = 0;
  myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
}


/* Function that gets called once the interrupt for free-fall ist triggered
   The device was thrown, now estimate on which side.
    
   NOTE: I set only a flag instead of executing code.
   This is because (I need to understand why) the interrupt
   function is only running for a short period of time.
   I suspect only while the interrupt is triggered or so.
   Thus, if you execute longer code here it just won't get
   executed after some millis.
   With the flag being set, we can use this in the loop afterwards. */
void onFreefallDetected() {
  Serial.println("Interrupt received!");
  wasThrown = true;
}


/** This is waiting for the device to stabilize so it can estimate the side it fell on.
    Then it notifies the user */
void estimateSideAndNotify() {
  Serial.println("Waiting to stabilize");
  audioOn();

  if (stabilize()) {
    uint8_t f = getFace();
    Serial.print("Audio Index: ");
    Serial.print(f);
    Serial.print(" Face: ");
    Serial.println(f + 1);
    play(f);  // Face #

    if (f != 30) {             // If not the alt face...
      if (f <= 3) {            // 0-3 (1-4) = bad
        play(23 + random(3));  // Random jab
      } else if (f >= 16) {    // 16-19 (17-20) = good
        play(26 + random(3));  // Random praise
      }
    }

    // Estimate voltage from battery, report if low.
    // This is "ish" and may need work.
    if ((readVoltage() < 3000) && !(batt++ & 1)) {  // Annc on every 2nd roll
      delay(500);
      play(31 + random(2));
    }
  }
  audioOff();

  Serial.println("estimateSideAndNotify() - done!");
}


// ---------------- SETUP & LOOP ----------------


void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("\nSTART PROGRAM");

  setupAudio();
  setupIMU();
}


void loop() {
  Serial.print("."); // Health check
  
  // Semaphore so we don't trigger it multiple times
  if (wasThrown && !calculatingResult) {
    calculatingResult = true;
    estimateSideAndNotify();
    resetFreeFallTrigger();
    calculatingResult = false;
    wasThrown = false;
  }
  delay(1000);
}

/** For now turn off (and restart)
    
    TODO: I don't know how to enable the gyro again so we will try things like that.
*/
void goToPowerOff() {
  //setLedRGB(false, false, false);
  Serial.println("Going to System OFF");
  setupFreeFallInterrupt();  // not needed here, if already applied..
  delay(100);                // delay seems important to apply settings, before going to System OFF
  //Ensure interrupt pin from IMU is set to wake up device
  nrf_gpio_cfg_sense_input(digitalPinToInterrupt(int1Pin), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  // Trigger System OFF
  NRF_POWER->SYSTEMOFF = 1;
}