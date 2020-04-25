// Tutorial 13a. (RX) Remote communication and control with RFM69HCW packet radio

// Main parts: Adafruit Feather M0 RFM69HCW, Pololu HD-1900A servo,
// CdS photoresistor, 10k trim potentiometer, momentary switch, LED,
// 8 Ohm loudspeaker, ON Semiconductor TIP120

// Libraries required to interface with the transceiver via SPI
// and to manage TX/RX communication; use the latest versions
#include <SPI.h>
#include <RH_RF69.h>

// Variables that remain constant
#define RF69_FREQ 915.0 // Carrier frequency
#define RFM69_CS 8 // Feather M0 chip select pin
#define RFM69_INT 3 // Feather M0 interrupt pin
#define RFM69_RST 4 // Feather M0 reset pin
// Instances a packet radio object from the library
RH_RF69 rf69(RFM69_CS, RFM69_INT);
const byte pinServo = 11;
const byte pinSpeaker = 9;
const byte pinLED = 5;
const int toneFrequency = 880; // Note A5 in Hz
const byte toneDuration = 50; // Tone duration in milliseconds

// Variables that can change
// A struct can bundle two or more different variables (members),
// and each can be of a different data type, quite like a shopping
// bag can contain different goods. Here, the struct is first named
// (transmission), then two variables (members) are declared, finally
// the struct is assigned to a variable (nodeTX) so it can be used
struct transmission
{
  bool momentaryswitch; // Values 0 or 1
  byte potentiometer; // Values 10 - 170 (for this particular servo)
  int photoresistor; // Values 0 - 1023 (1023 at maximum brightness)
} nodeTX;

void setup()
{
  // Initialise Feather M0 reset pin
  pinMode(RFM69_RST, OUTPUT);
  // Reset the Feather M0's radio IC
  digitalWrite(RFM69_RST, LOW);

  // Only needed for debugging with computer USB connection
  Serial.begin(115200);
  // Only needed for debugging with computer USB connection
  if (!rf69.init())
  {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  // Only needed for debugging with computer USB connection
  if (!rf69.setFrequency(RF69_FREQ))
  {
    Serial.println("RFM69 radio setFrequency failed");
  }

  // The radio's power in dBm; valid values are -2 to +20; always
  // set to the lowest power level needed
  rf69.setTxPower(9, true);
  //Define an encryption key; it must be the same for TX and RX
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  // Use the encryption key defined above
  rf69.setEncryptionKey(key);

  // Initialise servo pin
  pinMode(pinServo, OUTPUT);
  // Initialise loudspeaker pin
  pinMode(pinSpeaker, OUTPUT);
  // Initialise LED pin
  pinMode(pinLED, OUTPUT);
}

void loop()
{
  // Use the packet radio object's method that checks if some data
  // transmission has arrived from TX
  if (rf69.available())
  {
    // Seek the struct's space taken up in memory
    uint8_t len = sizeof(nodeTX);
    // Use the packet radio object's method that receives the actual
    // data and then puts it in the struct by pointing at its memory
    // location
    if (rf69.recv((uint8_t *)&nodeTX, &len))
    {
      // A call to this function sounds the loudspeaker, based on if
      // either a 0  (off) or 1 (on) was transmitted and received; it
      // puts it into the struct variable nodeTX.momentaryswitch
      soundSpeaker(pinSpeaker, nodeTX.momentaryswitch);

      // A call to this function rotates the servo based on the angle
      // value that was transmitted and received; it puts into the
      // struct variable nodeTX.potentiometer
      rotateServo(pinServo, nodeTX.potentiometer);

      // A call to this function brightens or darkens an LED
      driveLED(pinLED, nodeTX.photoresistor);
    }
  }
}

void soundSpeaker(byte pin, bool pressed)
{
  // If a 1 (on) was received and stored in the struct variable
  // nodeTX.momentaryswitch
  if (pressed == true)
  {
    // Then sound the loudspeaker at the frequency for a duration
    // both defined in the variables section
    tone(pin, toneFrequency, toneDuration);
  }
}

void rotateServo(byte pin, byte angle)
{
  // The Arduino Servo.h library is incompatible with the RadioHead
  // library, as they both use the same Feather M0 internal timer.
  // Instead, the servo is simply driven programmatically, without a
  // library, useful for other timer conflict projects, too. First,
  // the angle value received from TX is mapped onto a microseconds
  // range, where for this particular servo, 640 equals 10° and 2120
  // equals 170°. These values are different for every servo, so one
  // should check by trial before
  int pulseDelay = map(angle, 10, 170, 640, 2120);
  // Now enable the servo
  digitalWrite(pin, HIGH);
  // Wait
  delayMicroseconds(pulseDelay);
  // Then disable the servo
  digitalWrite(pin, LOW);
  // Finally wait again (50Hz PWM = pulse period of 20ms)
  delay(20);
}

void driveLED(byte pin, int level)
{
  // Brighten or darken the LED via PWM values between 0 - 255
  analogWrite(pin, level);
}
