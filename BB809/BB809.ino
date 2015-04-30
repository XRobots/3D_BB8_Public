
#include <SPI.h> // Included for SFE_LSM9DS0 library 
#include <Wire.h> // Needed for I2C
#include <SFE_LSM9DS0.h> // IMU Library https://github.com/sparkfun/LSM9DS0_Breakout/tree/master/Libraries/Arduino/SFE_LSM9DS0
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

#define dt 0.01          // time constant for complimentery filter

long previousMillis = 0;    // set up timers
unsigned long currentMillis;    
long interval = 10;        // time constant for timers

float AccelX;
float AccelY;
float AccelZ;
float GyroX;
float GyroY;
float GyroZ;
float mixX;
float mixY;

float x;
float y;
float z;

float pitchAccel, rollAccel;

float Ch1PWM = 0;
float Ch2PWM = 0;
float Ch3PWM = 0;
float Ch4PWM = 0;

double deadSpot = 3;        // variable for deadspot
int motorStart = 0;         // variable to make the motor start at something else other than zero
double motorGain = 9;       // post PID motor gain

double SetpointY, InputY, OutputY;    // PID variables
double SetpointX, InputX, OutputX;    // PID variables

PID myPIDY(&InputY, &OutputY, &SetpointY, 8, 0, 0.4, DIRECT);    // Y axis PID
PID myPIDX(&InputX, &OutputX, &SetpointX, 8, 0, 0.4, DIRECT);    // X Axis PID

void setup()
{
    pinMode(3, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);   

    SetpointY =0;      // Desired value for PID loops
    SetpointX =0; 

    myPIDY.SetMode(AUTOMATIC);
    myPIDY.SetOutputLimits(-255, 255);    // limits for PID loops
    myPIDX.SetMode(AUTOMATIC);
    myPIDX.SetOutputLimits(-255, 255);

  
  Serial.begin(115200); // Start serial at 115200 bps  // only used for debugging and calibration
  
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  uint16_t status = dof.begin();

}

void loop()
{
  dof.readGyro();                       // read Gyro
  GyroY = dof.calcGyro(dof.gx);         // read X Gyro into Y variable and vice versa
  GyroX = dof.calcGyro(dof.gy);         // read X Gyro into Y variable and vice versa
  GyroZ = dof.calcGyro(dof.gz);
  
  GyroX = GyroX*-1;                     // invert one channel
 
  dof.readAccel();                      // read Accel
  AccelX = dof.calcAccel(dof.ax);
  AccelY = dof.calcAccel(dof.ay);
  AccelZ = dof.calcAccel(dof.az);
  
  AccelX = AccelX+0.08;                // zero out sensors (use serial terminal to check)
  AccelY = AccelY+0.02;
  GyroX = GyroX+0.2;
  GyroY = GyroY+0.12;
  
  //Serial.print("AccelX ");
  //Serial.print(AccelX );
  //Serial.print(", ");
  //Serial.print("AccelY ");
  //Serial.print(AccelY);
  //Serial.println(", ");
  
  //Serial.print("GyroX ");
  //Serial.print(", ");
  //Serial.print("GyroY ");
  //Serial.print(GyroY);
  //Serial.println(", ");
  
  x=AccelX;                          // extra variables to make the next section look cleaner
  y=AccelY;
  z=AccelZ;
  
  pitchAccel = atan2(x, sqrt(y * y) + (z * z));
  rollAccel = atan2(y, sqrt(x * x) + (z * z));
  pitchAccel *= 180.0 / PI;
  rollAccel *= 180.0 / PI;
  
  pitchAccel = pitchAccel -0.08;    // zero out pitch and roll based on IMU mounting
  rollAccel = rollAccel -0.35;
  
  //Serial.print("Pitch ");
  //Serial.print(pitchAccel);
  //Serial.print(", ");
  //Serial.print("roll");
  //Serial.print(rollAccel);
  //Serial.println();
  
  currentMillis = millis();          // bookmark current time
  
  if (currentMillis - previousMillis >= interval)      // if the right amount of time has passed then...
  {
  previousMillis = currentMillis;                      // boomark the time again
 
  mixY = 0.98 *(mixY+GyroY*dt) + 0.02*rollAccel;       // Complimentary filter for Y, mix Gyro and Accel with average over time dt
  mixX = 0.98 *(mixX+GyroX*dt) + 0.02*pitchAccel;      // Complimentary filter for X
  
  InputY = mixY;                                       // take output of compimentary filter and put it into the PID inut variable
  myPIDY.Compute();                                    // use PID loop to calculate output
  Ch2PWM = abs(OutputY);                               // make output always positive to drive PWM 
  Ch2PWM = Ch2PWM*motorGain;                           // use motorGain variable to scale output
  Ch2PWM = map(Ch2PWM, 0, 255, motorStart, 255);       // scale value so that it uses the motorStart value to start the motor higher than zero PWM if set
  Ch2PWM = constrain(Ch2PWM, 0, 255);                  // constrain PWM value to 0-255 since that's the max value of PWM
  Ch3PWM = Ch2PWM;                                     // make opposing wheels the same value

  InputX = mixX;                                       // do all of the above for the other axis
  myPIDX.Compute();
  Ch1PWM = abs(OutputX);  
  Ch1PWM = Ch1PWM*motorGain;
  Ch1PWM = map(Ch1PWM, 0, 255, motorStart, 255);
  Ch1PWM = constrain(Ch1PWM, 0, 255);
  Ch4PWM = Ch1PWM; 
 
 
  // X Wheel Axis - Channels 2 & 3
  // Ch2: PWM - 5 | DIR - 4
  // Ch3: PWM - 9 | DIR - 8
  // LOW direction - drives towards negative Y Gyro/Accel 
  
  if (OutputY <= deadSpot*-1)                            // decide which way to turn the wheels based on deadSpot variable
  {
  digitalWrite(4, HIGH);                                 // set direction pins
  digitalWrite(8, HIGH);
  analogWrite(5, Ch2PWM);                                // set PWM pins 
  analogWrite(9, Ch3PWM);
  }
  else if (OutputY >= deadSpot)                          // decide which way to turn the wheels based on deadSpot variable
  { 
  digitalWrite(4, LOW);
  digitalWrite(8, LOW);
  analogWrite(5, Ch2PWM);  
  analogWrite(9, Ch3PWM);
  }
  else
  {
  analogWrite(5, 0);                                      // if we are within the deadspot turn off both wheels
  analogWrite(9, 0); 
  }  
  
  // Y Wheel Axis - Channels 1 & 4
  // Ch1: PWM - 3 | DIR - 2
  // Ch4: PWM - 11 | DIR - 12
  // LOW direction - drives towards negative X Gyro/Accel   
  
  if (OutputX >= deadSpot)                                // do all of the above for the other axis
  {
  digitalWrite(2, LOW);
  digitalWrite(12, LOW);
  analogWrite(3, Ch1PWM);  
  analogWrite(11, Ch4PWM);
  }
  else if (OutputX <= deadSpot*-1)
  { 
  digitalWrite(2, HIGH);
  digitalWrite(12, HIGH);
  analogWrite(3, Ch1PWM);  
  analogWrite(11, Ch4PWM);
  }
  else
  {
  analogWrite(3, 0);  
  analogWrite(11, 0); 
  }
  }
}



