/*

  01/12/20: NK
  Sketch written to mimic the BMW CAN messages DMcG used in v7 GS450H VCU code. Used to test CAN message creation, CAN sending, and CAN receiving. 
  Also used to allow testing of full v7 code without modification.

  Version 001: Incorporate button for T15 Run/Stop and potentiometer for P/R/N/D rotary gear selector
  Version 002: Add LED indicators for Run and P/R/N/D. Add interlock so can only activate Run when 
               gear is Park. If powered up while in Run position, switch has to be cycled off and then on 
               once in Park
  Version 003: Include CAN message creation and transmit
  Version 004: Expand to include High/Low range solenoid control and references for cruise control             

*/


#include <mcp_can.h>
#include <SPI.h>


/* Global variables for Run switch control */
const int run_select = 2;                                             // The pin that the pushbutton is attached to
const int ledPin = 7;                                                 // The pin that the LED is attached to
int run_state = 0;                                                    // Current state of the button
int last_run_state = 0;                                               // Previous state of the button

/* Global variables for gear selector control */
int gear_select = 0;                                                  // Potentiometer gear selector reading
int gear = 4;                                                         // Gear selection from potentiometer. Initial setting = Park
int last_gear = 4;                                                    // Previous state of the button
int gearindicatorled;                                                 // 0 = Park, 1 = Reverse, 2 = Neutral, 3 = Drive
const int sensorMin = 0;                                              // Min potentiometer value
const int sensorMax = 900;                                            // Max potentiometer value

/* Global variables for CAN communication */
const int SPI_CS_PIN = 9;                                             // 9 for version 1.1 or later, 10 for earlier versions
unsigned char run_can[8] = {0, 0, 0, 0, 0, 0, 0, 0};                  // CAN message for run signal
unsigned char gear_can[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // CAN message for gear selection

/* SAMD core - CAN communication */
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif


MCP_CAN CAN(SPI_CS_PIN);                                              // Set CS pin


void setup() {

  pinMode(run_select, INPUT_PULLUP);                                  // Configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(ledPin, OUTPUT);
  for (gearindicatorled = 3; gearindicatorled < 7; gearindicatorled++) {
    pinMode(gearindicatorled, OUTPUT);
  }
  
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {                          // Initialise CAN bus @ 500k baudrate
    SERIAL.println("CAN BUS Shield init fail");
    //SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }
    //SERIAL.println("CAN BUS Shield init ok!");  
}



void run_command() {
  
  run_state = digitalRead(run_select);                                // Read the run switch input pin

  if (run_state != last_run_state) {                                  // Compare the run state to its previous state
    if ((run_state == LOW) && (gear == 0)) {                          // If the state has changed, perform function
      digitalWrite(ledPin, HIGH);
      Serial.println("T15 Run");
      run_can[0] = 0x45;
    } else {
      digitalWrite(ledPin, LOW);                                      // If the state has changed, perform function
      Serial.println("T15 Stop");
      run_can[0] = 0;
    }
    
    run_CAN_transmit();
    //delay(50);                                                      // Delay a little bit to avoid bouncing
  }
  
  last_run_state = run_state;                                         // Save the current state as the last state, for next time through the loop

}



void gear_command() {
   
  gear_select = analogRead(A0);                                       // Read gear select potentiometer value
  if (gear_select > sensorMax) {
    gear_select = sensorMax;
  }
  gear = map(gear_select, sensorMin, sensorMax, 0, 3);                // Map the potentiometer range to a range of four options

  if (gear != last_gear) {                                            // If the gear selection has changed, perform function
    switch (gear) {                                                   // Do something different depending on the range value
      case 0:                                             
        Serial.println("Park");
        for (gearindicatorled = 3; gearindicatorled < 7; gearindicatorled++) {
          digitalWrite(gearindicatorled, LOW);
        }
        digitalWrite(3, HIGH);
        gear_can[0] = 0x6a;
        gear_can[1] = 0x50;
        gear_can[2] = 0x80;      
      break;
      
      case 1:                                             
        Serial.println("Reverse");
        for (gearindicatorled = 3; gearindicatorled < 7; gearindicatorled++) {
            digitalWrite(gearindicatorled, LOW);
          }
        digitalWrite(4, HIGH);
        gear_can[0] = 0x47;
        gear_can[1] = 0x01;
        gear_can[2] = 0x80;
      break;
      
      case 2:                                             
        Serial.println("Neutral");
        for (gearindicatorled = 3; gearindicatorled < 7; gearindicatorled++) {
            digitalWrite(gearindicatorled, LOW);
          }
        digitalWrite(5, HIGH);
        gear_can[0] = 0x2d;
        gear_can[1] = 0x04;
        gear_can[2] = 0x80;
      break;
      
      case 3:                                             
        Serial.println("Drive");
        for (gearindicatorled = 3; gearindicatorled < 7; gearindicatorled++) {
            digitalWrite(gearindicatorled, LOW);
          }
        digitalWrite(6, HIGH);
        gear_can[0] = 0x59;
        gear_can[1] = 0x02;
        gear_can[2] = 0x80;
      break;
    } 
    gear_CAN_transmit();
    //delay(50);                                                      // Delay a little bit to avoid bouncing
  }
  
  last_gear = gear;                                                   // Save the current state as the last state, for next time through the loop
  //delay(10);                                                        // Delay in between reads for stability  
}



void run_CAN_transmit() {

  CAN.sendMsgBuf(0x130, 0, 8, run_can);                               // Send data:  id = 0x00, standrad frame, data len = 8, run_can: data buf
  //delay(100);                                                       // Send data every 100ms
}


void gear_CAN_transmit() {
  
  CAN.sendMsgBuf(0x192, 0, 8, gear_can);                              // Send data:  id = 0x00, standrad frame, data len = 8, run_can: data buf
  //delay(100);                                                       // Send data every 100ms
}


void loop() {
  
  run_command();
  gear_command();
  delay(10); 
}
