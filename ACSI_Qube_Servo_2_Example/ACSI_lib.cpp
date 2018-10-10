/*
  24-774 Advanced Control System Integration
*/

#include <Arduino.h>
#include "QUBEServo.h"
#include "ACSI_lib.h"
#include <SPI.h>
#include <math.h>

bool startup = true;  // true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed

unsigned long previousMicros = 0;  // used to store the last time the SPI data was written
const long sampleTime = 2000;  // set the sample time (the interval between SPI transactions) to 1000us = 1ms, was 1000 originally
unsigned long elapsedTime = 0;

// set pin 10 as the slave select for the Quanser QUBE
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin = 10;

// initialize the SPI data to be written
byte mode = 1;                      // normal mode = 1
byte writeMask = B00011111;         // Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder0ByteSet[3] = {0, 0, 0}; // encoder0 is set to this value only when writes are enabled with writeMask
byte encoder1ByteSet[3] = {0, 0, 0}; // encoder1 is set to this value only when writes are enabled with writeMask
byte motorMSB = 0x80;               // motor command MSB must be B1xxxxxxx to enable the amplifier
byte motorLSB = 0;                  // motor command LSB

// initialize the SPI data to be read
byte moduleIDMSB = 0;               // module ID MSB (module ID for the QUBE Servo is 777 decimal)
byte moduleIDLSB = 0;               // module ID LSB
byte encoder0Byte[3] = {0, 0, 0};   // arm encoder counts
byte encoder1Byte[3] = {0, 0, 0};   // pendulum encoder counts
byte tach0Byte[3] = {0, 0, 0};      // arm tachometer
byte moduleStatus = 0;              // module status (the QUBE Servo sends status = 0 when there are no errors)
byte currentSenseMSB = 0;           // motor current sense MSB
byte currentSenseLSB = 0;           // motor current sense LSB

// global variables for LED intensity (999 is maximum intensity, 0 is off)
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;

// Setup global variables for wrap up function
float alpha = 0.0;  // pendulum angle in radians
float theta = 0.0;  // arm angle in radians
float currentSense = 0.0;
int moduleID = 0;

float motorVoltage = 0.0;
float maxVoltage = 10.0;

// State and LQR gain vectors
float desired[4] = {0, 0, 0, 0}; // upright pendulum
float Error[4] = {0, 0, 0, 0};
float StateX[4] = {0, 0, 0, 0};
//float gain[4] = { -2.2361, 37.6175, -1.5005, 3.3789};

float A[1][1] = {{0.9048}};
float B[1][2] = {{1.699, -3.825}};
float C[1][1] = {{4}};
float D[1][2] = {{ -73.64, 198.4}};

float intState = 0.0;
float intStateOld = 0.0;

float theta_n_k1 = 0;
float theta_dot_k1 = 0;
float alpha_n_k1 = 0;
float alpha_dot_k1 = 0;

float sin_freq = 0.2; // Hz
float sin_period = 1 / sin_freq * 1e6; // microseconds
float sin_amp = 0.5; // rad

float IS_A[3] = {0.2524, 0.5000, 0.2476}; // update these to real values
float IS_delays[3] = {0.0 * 1e3, 0.2805 * 1e3, 0.5610 * 1e3}; // update these to real values
float IS_prop = 0.0; // proportion of the total signal at a given timestep
int IS_num_steps = 3;
int IS_num_steps_count = 0;
//float IS_cmd = 0.0;
bool IS_setpt_pos = false;
bool IS_setpt_pos_k1 = false;
float thetaFreq = 1.0;
float thetaAmp = PI/6.0;
long prev_time = millis();

//Setup serial builder
Display displayData;

void calcSetpoints() {
  // this section for baseline motor angle control
  // desired[0] is the motor angle
  //  desired[0] = sin_amp * sin(elapsedTime / sin_period * 2 * M_PI); // sine wave
//  desired[0] = 0; // middle
  
  // show LEDs for debugging
//  LEDRed = (desired[0] * 400) + 500;

//  // IS square wave
  {
    IS_setpt_pos_k1 = IS_setpt_pos; // capture it before it gets changed below
    if ((millis() % (int)((2*PI)/thetaFreq*1000)) <= (int)(PI/thetaFreq*1000)){
      IS_setpt_pos = true;
      desired[0] = thetaAmp;
      LEDBlue = 999;
    } else {
      IS_setpt_pos = false;
      desired[0] = -thetaAmp;
      LEDBlue = 0;
    }
  }
}

void readSensors() {
  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  // take the slave select pin low to select the device
  digitalWrite(slaveSelectPin, LOW);

  // send and receive the data via SPI (except for the motor command, which is sent after the pendulum control code)
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0

  // combine the received bytes to assemble the sensor values

  /*Module ID*/
  moduleID = (moduleIDMSB << 8) | moduleIDLSB;

  /*Motor Encoder Counts*/
  long encoder0 = ((long)encoder0Byte[2] << 16) | (long)(encoder0Byte[1] << 8) | encoder0Byte[0];
  if (encoder0 & 0x00800000) {
    encoder0 = encoder0 | 0xFF000000;
  }
  // convert the arm encoder counts to angle theta in radians
  theta = (float)encoder0 * (-2.0 * M_PI / 2048);

  /*Pendulum Encoder Counts*/
  long encoder1 = ((long)encoder1Byte[2] << 16) | (long)(encoder1Byte[1] << 8) | encoder1Byte[0];
  if (encoder1 & 0x00800000) {
    encoder1 = encoder1 | 0xFF000000;
  }
  // wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
  encoder1 = encoder1 % 2048;
  if (encoder1 < 0) {
    encoder1 = 2048 + encoder1;
  }
  // convert the pendulum encoder counts to angle alpha in radians
  alpha = (float)encoder1 * (2.0 * M_PI / 2048) - M_PI;

  /*Current Sense Value*/
  currentSense = (currentSenseMSB << 8) | currentSenseLSB;
}

void updateStates() {
  StateX[0] = theta;
  StateX[1] = alpha;

  float theta_n = theta;
  float theta_dot = (46 * theta_n) - (46 * theta_n_k1) + (0.839 * theta_dot_k1);
  StateX[2] = theta_dot;
  theta_n_k1 = theta_n;
  theta_dot_k1 = theta_dot;

  float alpha_n = alpha;
  float alpha_dot = (46 * alpha_n) - (46 * alpha_n_k1) + (0.839 * alpha_dot_k1);
  StateX[3] = alpha_dot;
  alpha_n_k1 = alpha_n;
  alpha_dot_k1 = alpha_dot;

  for (int i = 0; i < 4; i++) {
    Error[i] = desired[i] - StateX[i];
  }
}

void calcMotorVoltage() {
  //  float Out = 0;
  //  for (int it = 0; it < 4; it++) {
  //    Out = Out + Error[it] * gain[it];
  //  }

  { // state space controller
    // y = Cx + Du
    // Vmotors = C*intState + D*Error
    float Vmotor = 0.0;
    Vmotor += C[0][0] * intState;
    Vmotor += D[0][0] * Error[0] + D[0][1] * Error[1];
  
    // x = Ax + Bu
    // intState = A*intStateOld + B*Error
    float intState = 0.0;
    intState += A[0][0] * intStateOld;
    intState += B[0][0] * Error[0] + B[0][1] * Error[1];
    intStateOld = intState;
    
    motorVoltage = 0 * Vmotor;
//    Serial.println(Vmotor);
  }

  { // apply voltage limits
    if (motorVoltage > maxVoltage) motorVoltage = maxVoltage;
    if (motorVoltage < -maxVoltage) motorVoltage = -maxVoltage;
  }

  { // this section for input shaping
//    static long start_time = millis();
    long time_since_step = millis() - prev_time;
//    Serial.print(time_since_step);
//    Serial.print(" ");
//    Serial.println(IS_delays[1]);
    if(IS_setpt_pos != IS_setpt_pos_k1){ // if the setpoint has changed between this timestep and the last
        // reset everything
        IS_prop = IS_A[0];
        time_since_step = 0;
        prev_time = millis(); // update timer to now
//        Serial.println("new step");
    } else {
        if (time_since_step >= IS_delays[0] && time_since_step < IS_delays[1]){
//          Serial.println("step 1");
          IS_prop = IS_A[0];
        } else if (time_since_step >= IS_delays[1] && time_since_step < IS_delays[2]){
//          Serial.println("step 2");
          IS_prop = IS_A[0] + IS_A[1];          
        } else {
//          Serial.println("step 3");
          IS_prop = IS_A[0] + IS_A[1] + IS_A[2]; // should be 1.0
        }
    }
//    Serial.println(IS_prop);
    motorVoltage = desired[0] * IS_prop * 0; // may need to include a gain
  }
}

void driveMotor() {
  // convert the analog value to the PWM duty cycle that will produce the same average voltage
  float motorPWM = motorVoltage * (625.0 / 15.0);

  int motor = (int)motorPWM;  // convert float to int (2 bytes)
  motor = motor | 0x8000;  // motor command MSB must be B1xxxxxxx to enable the amplifier
  motorMSB = (byte)(motor >> 8);
  motorLSB = (byte)(motor & 0x00FF);

  // convert the LED intensities to MSB and LSB
  LEDRedMSB = (byte)(LEDRed >> 8);
  LEDRedLSB = (byte)(LEDRed & 0x00FF);
  LEDGreenMSB = (byte)(LEDGreen >> 8);
  LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
  LEDBlueMSB = (byte)(LEDBlue >> 8);
  LEDBlueLSB = (byte)(LEDBlue & 0x00FF);

  // send the motor data via SPI
  SPI.transfer(motorMSB);
  SPI.transfer(motorLSB);

  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, HIGH);
  SPI.endTransaction();
}

// This function is used to clear the stall error and reset the encoder values to 0.
// The motor and LEDs are turned off when this function is called.
void resetQUBEServo() {

  // enable the motor and LEDs, and enable writes to the encoders
  writeMask = B01111111;

  // turn off the LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;

  // reset the encoder values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;

  // turn off the motor, and clear the stall error by disabling the amplifier
  motorMSB = 0;  // motor command MSB is B0xxxxxxx to disable the amplifier
  motorLSB = 0;

  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, LOW);   // take the slave select pin low to select the device

  // send and receive the data via SPI
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
  SPI.transfer(motorMSB);                              // send the motor MSB
  SPI.transfer(motorLSB);                              // send the motor LSB

  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();

  writeMask = B00011111;  // enable the motor and LEDs, disable writes to the encoders
  motorMSB = 0x80;  // enable the amplifier
}

//float calcStdDev(const int data[], const int numElms){
//  float sum = 0;
//  for (int i = 0; i < numElms; i++){
//    sum += data[i];
//  }
//  float mean = sum/numElms;
//  float dev = 0;
//  for (int i = 0; i < numElms; i++){
//    dev += pow((data[i] - mean), 2);
//  }
//  float std = sqrt(dev / numElms);
//  return std;
//}
