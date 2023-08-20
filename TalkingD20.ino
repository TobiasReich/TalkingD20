#include <Adafruit_Soundboard.h>
#include "Adafruit_TinyUSB.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <limits.h> // required for LONG_MAX etc.

#include "LSM6DS3.h"
//#include "Wire.h"
#include <Wire.h>






// ---------------- Sound ----------------
// Adafruit SoundBoard
#define SFX_TX 6
#define SFX_RX 7
#define SFX_RST 8

#define AUDIO_ACT     4 // "Act" on Audio FX -> Used for checking whether there is some audio playing

/* Note: This is software serial only for now until I figure out why this is not working.
I guess I might be missing something simple here but I'm not sure yet, what it is.  */
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);   // <---- this worked, hardware did not?!

Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);


char filename[12] = "        OGG"; // Tail end of filename NEVER changes

// PROGMEM string arrays are wretched, and sfx.playTrack() expects a
// goofball fixed-length space-padded filename...we take care of both by
// declaring all the filenames inside one big contiguous PROGMEM string
// (notice there are no commas here, it's all concatenated), and copying
// an 8-byte section as needed into filename[].  Some waste, but we're
// not hurting for space.  If you change or add any filenames, they MUST
// be padded with spaces to 8 characters, else there will be...trouble.  
static const char PROGMEM bigStringTable[] =   // play() index
  "01      " "02      " "03      " "04      "  //  0- 3
  "05      " "06      " "07      " "08      "  //  4- 7
  "09      " "10      " "11      " "12      "  //  8-11
  "13      " "14      " "15      " "16      "  // 12-15
  "17      " "18      " "19      " "20      "  // 16-19
  "ANNC1   " "ANNC2   " "ANNC3   "             // 20-22
  "BAD1    " "BAD2    " "BAD3    "             // 23-25
  "GOOD1   " "GOOD2   " "GOOD3   "             // 26-28
  "STARTUP " "03ALT   " "BATT1   " "BATT2   "; // 29-32



void audioOn(void) {
  //pinMode(AMP_SHUTDOWN, INPUT_PULLUP);
  //pinMode(AUDIO_RESET , INPUT_PULLUP);
  Serial.println("Audio ON (TODO)");
}

void audioOff(void) {
  //digitalWrite(AMP_SHUTDOWN, LOW);
  //pinMode(     AMP_SHUTDOWN, OUTPUT);
  //digitalWrite(AUDIO_RESET , LOW); // Hold low = XRESET (low power)
  //pinMode(     AUDIO_RESET , OUTPUT);
  Serial.println("Audio OFF (TODO)");
}

/* Plays the track defined in the array above */
void play(uint16_t i) {
  /*memcpy_P(filename, &bigStringTable[i * 8], 8);

    sfx.playTrack(filename);
    delay(250); // Need this -- some delay before ACT LED is valid
    while(digitalRead(AUDIO_ACT) == LOW);  // Wait for sound to finish
  */
  Serial.print("Playing Track (TODO): ");
  Serial.println(i);
}



// ---------------- ACCELEROMETER ----------------

//Instance of class LSM6DS3 of the nRF52840
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A


// Waits for accelerometer output to stabilize, indicating movement stopped
boolean stabilize(uint32_t ms, uint32_t timeout) {
  uint32_t startTime, prevTime, currentTime;
  int32_t  prevX, prevY, prevZ;
  int32_t  dX, dY, dZ;

  // Get initial orientation and time
  prevX    = myIMU.readFloatAccelX();
  prevY    = myIMU.readFloatAccelY();
  prevZ    = myIMU.readFloatAccelZ();
  prevTime = startTime = millis();

  // Then make repeated readings until orientation settles down.
  // A normal roll should only take a second or two...if things do not
  // stabilize before timeout, probably being moved or carried.
  while(((currentTime = millis()) - startTime) < timeout) {
    if((currentTime - prevTime) >= ms) return true; // Stable!
    dX = myIMU.readFloatAccelX() - prevX; // X/Y/Z delta from last stable position
    dY = myIMU.readFloatAccelY() - prevY;
    dZ = myIMU.readFloatAccelZ() - prevZ;
    // Compare distance.  sqrt() can be avoided by squaring distance
    // to be compared; about 100 units on X+Y+Z axes ((100^2)*3 = 30K)
    if((dX * dX + dY * dY + dZ * dZ) >= 30000) { // Big change?
      prevX    = myIMU.readFloatAccelX();    // Save new position
      prevY    = myIMU.readFloatAccelY();
      prevZ    = myIMU.readFloatAccelZ();
      prevTime = millis(); // Reset timer
    }
  }

  return false; // Didn't stabilize, probably false trigger
}

// Face gravity vectors (accelerometer installed FACE DOWN)
// Note the original values for the MMA8451 were 14 bit (until 4096)
// but since this IMU gives me -1..1 values I have to divide it by 4096
static const float_t gtable[20][3] = {
  {  0.362, -0.476,  0.784 }, //  1
  { -0.590,  0.804, -0.189 }, //  2
  {  0.906, -0.286  -0.194 }, //  3
  { -0.935, -0.274, -0.224 }, //  4
  { -0.595,  0.213,  0.758 }, //  5
  { -0.012, -0.584, -0.797 }, //  6
  {  0.559,  0.187,  0.783 }, //  7
  {  0.336,  0.513, -0.787 }, //  8
  {  0.001, -0.975, -0.206 }, //  9
  {  0.561,  0.808, -0.169 }, // 10
  { -0.762, -0.778,  0.148 }, // 11
  { -0.006,  0.987,  0.203 }, // 12
  { -0.369, -0.495,  0.771 }, // 13
  { -0.592, -0.162, -0.793 }, // 14
  { -0.015,  0.603,  0.784 }, // 15
  {  0.551, -0.196, -0.781 }, // 16
  {  0.913,  0.292,  0.189 }, // 17
  { -0.932,  0.312,  0.174 }, // 18
  {  0.545, -0.793,  0.191 }, // 19
  { -0.391,  0.507, -0.789 }  // 20
};

/* // Old face values stored in 14 bit
static const int16_t PROGMEM gtable[20][3] = {
  {  1475, -1950,  3215 }, //  1
  { -2420,  3295,  -775 }, //  2
  {  3715, -1175   -795 }, //  3
  { -3830, -1125,  -920 }, //  4
  { -2440,   875,  3105 }, //  5
  {   -50, -2395, -3265 }, //  6
  {  2290,   770,  3210 }, //  7
  {  1380,  2105, -3225 }, //  8
  {    50, -3995,  -845 }, //  9
  {  2300,  3310,  -695 }, // 10
  { -2430, -3190,   610 }, // 11
  {   -25,  4045,   835 }, // 12
  { -1515, -2030,  3160 }, // 13
  { -2425,  -665, -3250 }, // 14
  {   -65,  2470,  3215 }, // 15
  {  2260,  -805, -3200 }, // 16
  {  3740,  1200,   775 }, // 17
  { -3820,  1280,   715 }, // 18
  {  2235, -3250,   785 }, // 19
  { -1605,  2080, -3235 }  // 20
};
*/

// Find nearest face to accelerometer reading.
uint8_t getFace(void) {
  float  dX, dY, dZ, d, dMin = INT_MAX;
  float  fX, fY, fZ;
  uint8_t  i, iMin = 0;

  for(i=0; i<20; i++) { // For each face...
    fX = gtable[i][0]; // Read face X/Y/Z
    fY = gtable[i][1]; // from PROGMEM
    fZ = gtable[i][2];
    dX = myIMU.readFloatAccelX() - fX; // Delta between accelerometer & face
    dY = myIMU.readFloatAccelY() - fY;
    dZ = myIMU.readFloatAccelZ() - fZ;
    d  = dX * dX + dY * dY + dZ * dZ; // Distance^2
    // Check if this face is the closest match so far.  Because
    // we're comparing RELATIVE distances, sqrt() can be avoided.
    if(d < dMin) { // New closest match?
      dMin = d;    // Save closest distance^2
      iMin = i;    // Save index of closest match
    }
  }

  return iMin; // Index of closest matching face
}


// ---------------- POWER SAVING STUFF ----------------

uint8_t batt = 0; // Low battery announcement counter

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
  return 3500; // TODO: mocked for now until I check the voltage
}




// ---------------- Initialization ----------------



void setupAudio(){
  pinMode(SFX_TX, OUTPUT);
  pinMode(SFX_RX, INPUT);
 
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

void setupIMU(){
  Serial.println("starting IMU...");

  // We only need the accelerometer
  myIMU.settings.gyroEnabled = false;
  myIMU.settings.tempEnabled = false;
  myIMU.settings.accelEnabled = true;

  if (myIMU.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }

  Serial.println("IMU started!");
}


void setup() {
  Serial.begin(115200); 
  
  setupAudio();

  setupIMU();
}

void loop(){
  Serial.println("---");

  //Serial.println("Playing..."); 
  //sfx.playTrack(1);
  //sfx.playTrack("T00     OGG");

  /*Serial.println("---IMU---");

    //Accelerometer
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatAccelX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatAccelY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatAccelZ(), 4);
  */

  if(stabilize(250, 3000)) {
    uint8_t f = getFace();
  
    /*if(f == 2) {              // If '3' face
      if(!random(10)) {       // 1-in-10 chance of...
        f = 30;               // Alternate 'face 3' track
      }                       // LOL
    }*/
  
    //Serial.println("Anouncement!");
    //play(20 + random(3));     // One of 3 random announcements

  
    Serial.print("Audio Index: ");
    Serial.print(f);   
    Serial.print(" Face: ");
    Serial.println(f+1); 
    play(f);                  // Face #
  
    if(f != 30) {             // If not the alt face...
      if(f <= 3) {            // 0-3 (1-4) = bad
        play(23 + random(3)); // Random jab
      } else if(f >= 16) {    // 16-19 (17-20) = good
        play(26 + random(3)); // Random praise
      }
    }
  
    // Estimate voltage from battery, report if low.
    // This is "ish" and may need work.
    if((readVoltage() < 3000) && !(batt++ & 1)) { // Annc on every 2nd roll
      delay(500);
      play(31 + random(2));
    }
  }

  delay(3000);
}
