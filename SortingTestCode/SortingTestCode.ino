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
#define CH1 0                                                     // 16 Channels (0-15) are availible
#define CH2 1                                                     // Make sure each pin is a different channel and not in use by other PWM devices (servos, LED's, etc)
#define servoPin 42                                               
#define pinSCL 14                                                 //Colour sensor pin declarations
#define pinSDA 13

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);              //Set custom integration times and gain for colour sensor

Servo Servo;                                                      // Initialize servo motor

// Variables to store current and previous time
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

MX1508 motorA(PINA,PINB, CH1, CH2);                               // Default- 8 bit resoluion at 2500 Hz


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
  motorA.motorGo(180);                                           // Pass the speed to the motor: 0-255 for 8 bit resolution (drive constantly)

  uint16_t r, g, b, c, colorTemp, lux;
  currentMillis = millis();
    
  tcs.getRawData(&r, &g, &b, &c);
  Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);      //Read and print detected values from colour sensor

 //Servo Code
  if (currentMillis - previousMillis > 2000) {                  //Timer to limit speed of operation
   if(r > 5 || g > 5 || b > 5){                                 //If colour is detected, turn left or right based on if values is green or not and reset timer
     if (g > r && g > b){
       Serial.println("Green!");                                //Serial monitor print statement for confirming which value is detected
       Servo.write(0);
       previousMillis = currentMillis;
     } else {
       Serial.println("Not Green :(");
       Servo.write(180);
       previousMillis = currentMillis;
     }
   } else {
     Servo.write(90);                                           //Reset servo position to centre
     }
   }
}