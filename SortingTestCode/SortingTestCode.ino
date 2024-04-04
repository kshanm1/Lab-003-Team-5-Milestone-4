// MSE2202B Milestone 4 Sorting Code
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Lab 03 Team 5
//  Date:     2024 04 01 
//

#include <ESP32MX1508.h> // Library for MX1508 motor driver
#include <ESP32Servo.h>  // Library for servo motor
#include "Wire.h"     // Library for I2C/SPI communication
#include "Adafruit_TCS34725.h"  // Library for TCS34725 colour sensor

#define PINA 35                                                   //Define DC motor driver pins
#define PINB 36
#define PINA2 37
#define PINB2 38
#define CH1 0                                                     // 16 Channels (0-15) are availible
#define CH2 1                                                     // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)
#define CH3 2
#define CH4 3
#define servoPin 42                                               
#define pinSCL 14                                                 //Colour sensor pin declarations
#define pinSDA 13

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);              //Set custom integration times and gain for colour sensor

Servo Servo;

unsigned int timeupsec = 1;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long timerCountsec = 0;
unsigned int servoTarget = 90;

MX1508 motorA(PINA, PINB, CH1, CH2);                               // Default- 8 bit resoluion at 2500 Hz
MX1508 motorB(PINA2, PINB2, CH3, CH4);


void setup() {
  Servo.attach(servoPin);                                               //Associate Servo control with GPIO pin 42
  Serial.begin(115200);
  Wire.setPins(pinSDA, pinSCL);                                   //Set pins for colour sensor input using included wire library

    if (tcs.begin()) {                                            //Test for and confirm connection with colour sensor
      Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1);
    }                                                             // Now we're ready to get readings!
}


void loop() {
  currentMillis = millis();
  motorA.motorGo(200);                                           // Pass the speed to the motor: 0-255 for 8 bit resolution (drive constantly)
  motorB.motorGo(200);
  uint16_t r, g, b, c, colorTemp, lux;
    
  tcs.getRawData(&r, &g, &b, &c);


        timerCountsec += 1;                                                    // increment second timer count
      if (timerCountsec > 100 && timeupsec == 0) {                                             // if 1 second has elapsed
         timeupsec = 1;                                                       // indicate that 2 seconds have elapsed
         servoTarget = 90;
         Serial.printf("Time's up!\n");
      }

for(int i = 0; i<500; i++){                                     //500 Iteration for loop who's sole purpose is to give the servo enough time to reach the desired position
  Servo.write(servoTarget);
}

motorA.motorGo(200);
motorB.motorGo(200);

  if (currentMillis - previousMillis > 2000 && timeupsec == 1) {                  //Timer to limit speed of operation
    Serial.printf("Current Milliseconds: %d\n", currentMillis);
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);      //Read and print detected values from colour sensor
    Serial.printf("Scanning...\n");

        if (g > r && g > b+2 && c < 50){                           //Turn left or right based on colour value detected
          Serial.println("Green!");                                //Serial monitor print statement for confirming which value is detected
          servoTarget = 0;
          Serial.println("Turned.");
          timeupsec = 0;
          timerCountsec = 0;
          Serial.println("Begin reset timer");
        } else {
          Serial.println("Not Green :(");
          servoTarget = 180;
          Serial.println("Turned.");
          timeupsec = 0;
          timerCountsec = 0;
          Serial.println("Begin reset timer");
        }
    previousMillis = currentMillis;
   }
}