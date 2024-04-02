// 
// MSE 2202 Milestone 4 Sorting Code
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Lab 03 Team 5
//  Date:     2024 04 01 
//



#define SERIAL_STUDIO                              // print formatted string, that can be captured and parsed by Serial-Studio

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);

// Switch structure
struct Switch {
  const int pin;                                   // GPIO pin for switch
  unsigned int numberPresses;                      // counter for number of switch presses
  unsigned int lastPressTime;                      // time of last switch press in ms
  bool pressed;                                    // flag for switch press event
};

// Constants
const int cHeartbeatInterval = 75;                 // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                 // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                  // number of Smart LEDs in use
const long cDebounceDelay    = 150;                // switch debounce delay in milliseconds
const int cIN1Pin = 35;                            // GPIO pin(s) for INT
const int cIN1Chan = 0;                            // PWM channel for IN1
const int c2IN2Pin = 36;                           // GPIO pin for IN2
const int cIN2Chan = 1;                            // PWM channel for IN2
const int cPWMRes = 8;                             // bit resolution for PWM
const int cMinPWM = 0;                             // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;           // PWM value for maximum speed
const int cPWMFreq = 20000;                        // frequency of PWM signal
const int cServoPin = 41;                             // GPIO pin for servo motor
const int cServoChannel = 5;
const int cSDA               = 6;                    // GPIO pin for I2C data
const int cSCL               = 7;                    // GPIO pin for I2C clock
const int cTCSLED            = 5;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725
const int pwm                = 100;

// Variables
boolean heartbeatState       = true;               // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                  // time of last heartbeat state change
unsigned long curMillis      = 0;                  // current time, in milliseconds
boolean timeUp2sec = false;  
unsigned long timerCount2sec = 0; 
unsigned long prevMillis     = 0;                  // start time for delay cycle, in milliseconds
Switch button = {0, 0, 0, false};                  // NO pushbutton PB1 on GPIO 0, low state when pressed                        // last time of motor control was updated
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
#if defined SERIAL_STUDIO
  Serial.begin(115200);                            // Standard baud rate for ESP32 serial monitor
#endif
  // Set up SmartLED
  SmartLEDs.begin();                               // initialize smart LEDs object
  SmartLEDs.clear();                               // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                     // set brightness [0-255]
  SmartLEDs.show();                                // update LED

  // Set up motors
  ledcAttachPin(cIN1Pin, cIN1Chan);                // attach IN1 GPIO to PWM channel
  ledcSetup(cIN1Chan, cPWMFreq, cPWMRes);          // configure PWM channel frequency and resolution
  ledcAttachPin(c2IN2Pin, cIN2Chan);               // attach IN2 GPIO to PWM channel
  ledcSetup(cIN2Chan, cPWMFreq, cPWMRes);          // configure PWM channel frequency and resolution

  setMotor(1, pwm, cIN1Chan, cIN2Chan);

  // Set up push button
  pinMode(button.pin, INPUT_PULLUP);               // configure GPIO for button pin as an input with pullup resistor

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  pinMode(cServoPin, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);         // assign servo pin to servo channel

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

}

void loop() {
 
currentMicros = micros();                                                   // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
      previousMicros = currentMicros;

    timerCount2sec = timerCount2sec + 1; 
  if (timerCount2sec > 2000) {                                             // if 2 seconds have elapsed
         timerCount2sec = 0;                                                   // reset 2 second timer count
         timeUp2sec = true;                                                    // indicate that 2 seconds have elapsed
      }
   }

  ledcWrite(cServoChannel, 90);                                    // set initial position to 90 degrees

  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)

  if (timeUp2sec) {
    timeUp2sec = false;
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
    if (g > r && g > b) {                             // if green is the dominant colour
     ledcWrite(cServoChannel, 180);                            // turn servo to the right
    } else {
      ledcWrite(cServoChannel, 0);                                 // turn servo to the left
    }
  }
  }

  
  doHeartbeat();                                   // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                            // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                     // update the heartbeat time for the next update
    LEDBrightnessIndex++;                          // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                      // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                              // update LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                  // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                            // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                           // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}