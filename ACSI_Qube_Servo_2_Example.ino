/*
Quanser QUBE Servo Startup - ACSI

An example using the Arduino UNO board to communicate with the Quanser QUBE Servo
through the microcontroller interface panel.  This implements a pseudo-controller
to demonstrate sensor read and motor write functionality.

Select 250000 baud on the Arduino Serial Tested with Arduino Software (IDE) 1.6.7.

Original file created 2016 by Quanser Inc.
www.quanser.com

Modified for ACSI 2017 by Bin Xu
www.cmu.edu
*/

// include the QUBE Servo library
#include "QUBEServo.h"

// include ACSI_lib header
#include "ACSI_lib.h"

// include the SPI library and the math library
#include <SPI.h>
#include <math.h>

// include the ILC commands
//#include "u_ilc.h"

// Don't touch the setup unless you decide to use interrupts
void setup() {
  // set the slaveSelectPin as an output
  pinMode (slaveSelectPin, OUTPUT);
  
  // initialize SPI
  SPI.begin();
  
  // initialize serial communication at 250000 baud
  // (Note that 250000 baud must be selected from the drop-down list on the Arduino
  // Serial Monitor for the data to be displayed properly.)
  Serial.begin(250000);
}

void loop() {
  
  // after the Arduino power is cycled or the reset pushbutton is pressed, call the resetQUBEServo function
  // Don't touch
  if (startup) {
    resetQUBEServo();
    startup = false;
  }
  
  // if the difference between the current time and the last time an SPI transaction
  // occurred is greater than the sample time, start a new SPI transaction
  // Alternatively, use a timer interrupt
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= sampleTime) {
    previousMicros = previousMicros + sampleTime;  
    elapsedTime += sampleTime;
    if (elapsedTime > (2 * M_PI)*1e6){
      elapsedTime -= (2 * M_PI)*1e6;
    }
    // Simple function for reading from sensors.  Data are read into global variables.  For variable definitions, see ACSI_lib.h.
    readSensors();
    // This will define the data that will be displayed at the serial terminal.
    displayData.buildString(theta, alpha, currentSense, moduleID, moduleStatus);
  
    calcSetpoints();
    updateStates();
    calcMotorVoltage();
    driveMotor();
  }

  
  // print data to the Arduino Serial Monitor in between SPI transactions
  // (Note that the Serial.print() function is time consuming.  Printing the entire
  // string at once would exceed the sample time required to balance the pendulum.)
  else {  //We're in between samples
    // only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
    if ( (displayData.dDataReady) && (currentMicros - previousMicros <= (sampleTime - 100)) ) {
      // if there is room available in the serial buffer, print one character
      if(Serial.availableForWrite() > 0) {
        { // this block prints out the arm and pendulum angles. use for capturing data output
//            Serial.print("th:");
//            Serial.print(theta);
//            Serial.print(" al:");
//            Serial.println(alpha);
        }
          {
//            Serial.print(elapsedTime);
//            Serial.print(" ");
//            Serial.print(IS_delays[0]);
//            Serial.print(" ");
//            Serial.print(IS_delays[1]);
//            Serial.print(" ");
//            Serial.println(IS_delays[2]);
          }
//        Serial.print(displayData.dData[displayData.dDataIndex]);
        displayData.dDataIndex = displayData.dDataIndex + 1;
        // if the entire string has been printed, clear the flag so a new string can be obtained
        if(displayData.dDataIndex == displayData.dData.length()) {
          displayData.dDataReady = false;
        }
      }
    }
  }
}
