// Tutorial 13a. (TX) Remote communication and control with RFM69HCW packet radio

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
const byte pinSwitch = A0; // Analog input pin from momentary switch
const byte pinPotentiometer = A1; // Analog input pin from potentiometer
const byte pinSensor = A2; // Analog input pin from photoresistor

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
byte lastSwitchState = HIGH; // Momentary switch assumed open at start

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

  // Initialise momentary switch pin with an internal pull-up resistor
  // so that the momentary switch is read as open (= HIGH) at start
  pinMode (pinSwitch, INPUT_PULLUP);

  // Initialise potentiometer pin with an internal pull-up resistor
  pinMode (pinPotentiometer, INPUT_PULLUP);

  // Initialise sensor pin with an internal pull-up resistor
  pinMode (pinSensor, INPUT_PULLUP);
}

void loop()
{
  // Hardcoded fudge-factor to compensate the RX servo's PWM; without
  // it, the servo would twitch, as the potentiometer data sent from
  // TX would become out of sync with what RX executes
  delay(22);

  // A call to this function fetches a reading from the momentary switch pin
  readMomentarySwitch();

  // A call to this function fetches a reading from the potentiometer pin
  readPotentiometer();

  // A call to this function fetches a reading from the sensor pin
  readPhotoresistor();

  // Use the packet radio object's method that sends (TX) data while
  // pointing at the struct's memory location and seeking its space
  // taken up in memory
  rf69.send((uint8_t *)&nodeTX, sizeof(nodeTX));
  // Use the packet radio object's method that waits until the data
  // was successfully transmitted to the receiver (RX)
  rf69.waitPacketSent();

  // Immediately set the momentary switch button variable to 0 (off),
  // so that each button press only transmits a single 1 (on) pressing
  // the button down; otherwise, a 1 would be sent for as long as the
  // button is pressed, and the loudspeaker on RX would remain on for
  // as long as the button is pressed also
  nodeTX.momentaryswitch = false;
}

void readMomentarySwitch()
{
  // The momentary switch is hardware debounced with a 1uF capacitor; no
  // debouncing code is necessary. See http://www.gammon.com.au/switches
  // Read the voltage from the momentary switch pin to see if something
  // has changed (was the button pressed or released?)
  byte switchState = digitalRead (pinSwitch);

  // Has the momentary switch state changed since the last time it was
  // checked (once every loop() iteration)?
  if (switchState != lastSwitchState)
  {
    // First, store the current switch state for the next time around
    lastSwitchState = switchState;

    // Next, test if the switch was closed (button pressed)
    if (switchState == LOW)
    { // If it was, set the transmission struct variable to 1
      nodeTX.momentaryswitch = true;
    }
    else
    {
      // If it wasn't, set the transmission struct variable to 0
      nodeTX.momentaryswitch = false;
    }
  }
}

void readPotentiometer()
{
  // Read the voltage from the potentiometer pin and map the output
  // to the RX servo's mechanically achievable angle range
  nodeTX.potentiometer = map(analogRead(pinPotentiometer), 0, 1023, 10, 170);
}

void readPhotoresistor()
{
  // Read the voltage from the sensor pin and map the output to the
  // RX LEDs PWM value range of 0 - 255. The minimum and maximum values
  // of 350 and 900 used here are specific to the room and daytime where
  // this set-up was put together and used, the theoretically achievable
  // range is 0 - 1023. Constraining the output of map() clips outlier
  // values passed into the map() function
  nodeTX.photoresistor = constrain(map(analogRead(pinSensor), 350, 900, 255, 0), 0, 255);
}
