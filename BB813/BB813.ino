
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

int back;
int forward;
int left;
int right;

double bias = 4.5;

float Ch1PWM = 0;
float Ch2PWM = 0;
float Ch3PWM = 0;
float Ch4PWM = 0;

double deadSpot = 0;        // variable for deadspot
int motorStart = 0;         // variable to make the motor start at something else other than zero
double motorGain = 6;       // post PID motor gain

double SetpointY, InputY, OutputY;    // PID variables
double SetpointX, InputX, OutputX;    // PID variables

double Pk = 12;
double Ik = 0.45;
double Dk = 0.35;

PID myPIDY(&InputY, &OutputY, &SetpointY, Pk, Ik , Dk, DIRECT);    // Y axis PID
PID myPIDX(&InputX, &OutputX, &SetpointX, Pk, Ik,  Dk, DIRECT);    // X Axis PID

void setup()
{
    pinMode(3, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
    
    pinMode(10, INPUT);
    pinMode(11, INPUT);     
    pinMode(12, INPUT);
    pinMode(13, INPUT);   

    SetpointY =0;      // Desired value for PID loops
    SetpointX =0; 

    myPIDY.SetMode(AUTOMATIC);
    myPIDY.SetOutputLimits(-255, 255);    // limits for PID loops
    myPIDX.SetMode(AUTOMATIC);
    myPIDX.SetOutputLimits(-255, 255);
    
    myPIDY.SetSampleTime(10);
    myPIDX.SetSampleTime(10);
      
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
  GyroX = GyroX+0.3;
  GyroY = GyroY+0.32;
  /*
  Serial.print("AccelX ");
  Serial.print(AccelX );
  Serial.print(", ");
  Serial.print("AccelY ");
  Serial.print(AccelY);
  Serial.println(", ");
  
  Serial.print("GyroX ");
  Serial.print(GyroX);
  Serial.print(", ");
  Serial.print("GyroY ");
  Serial.print(GyroY);
  Serial.println(", ");  
  */
  forward = digitalRead(12);                            // Read pins from R/C Arduino
  back = digitalRead(13);
  left = digitalRead(10);
  right = digitalRead(11);
  
      /*
      if (back == HIGH){                              
      AccelY = AccelY+bias;
      }

      else if (forward == HIGH){                              
      AccelY = AccelY-bias;
      }

      if (left == HIGH){                              
      AccelX = AccelX+bias;
      }

      else if (right == HIGH){                              
      AccelX = AccelX-bias;
      }
      */
      
      
      
      if (back == HIGH){                              
      GyroY = GyroY+bias;
      }

      else if (forward == HIGH){                              
      GyroY = GyroY-bias;
      }

      if (left == HIGH){                              
      GyroX = GyroX+bias;
      }

      else if (right == HIGH){                              
      GyroX = GyroX-bias;
      }
  
  
  x=AccelX;                          // extra variables to make the next section look cleaner
  y=AccelY;
  z=AccelZ;
  
  pitchAccel = atan2(x, sqrt(y * y) + (z * z));
  rollAccel = atan2(y, sqrt(x * x) + (z * z));
  pitchAccel *= 180.0 / PI;
  rollAccel *= 180.0 / PI;
  
  pitchAccel = pitchAccel +0.08;    // zero out pitch and roll based on IMU mounting
  rollAccel = rollAccel -0.3;
  
  
  Serial.print("Pitch ");
  Serial.print(pitchAccel);
  Serial.print(", ");
  Serial.print("roll ");
  Serial.print(rollAccel);
  Serial.println();
  
  
  currentMillis = millis();          // bookmark current time
  
  if (currentMillis - previousMillis >= interval)      // if the right amount of time has passed then...
  {
  previousMillis = currentMillis;                      // boomark the time again
 
  mixY = 0.98 *(mixY+GyroY*dt) + 0.02*rollAccel;       // Complimentary filter for Y, mix Gyro and Accel with average over time dt
  mixX = 0.98 *(mixX+GyroX*dt) + 0.02*pitchAccel;      // Complimentary filter for X
    
  /*
  Serial.print("MixX ");
  Serial.print(mixX);
  Serial.print(", ");
  Serial.print("MixY");
  Serial.print(mixY);
  Serial.println();
  */

  InputY = mixY;                                       // take output of compimentary filter and put it into the PID inut variable
  myPIDY.Compute();                                    // use PID loop to calculate output
  Ch2PWM = abs(OutputY);                               // make output always positive to drive PWM 
  Ch2PWM = Ch2PWM*motorGain;                           // use motorGain variable to scale output
  Ch2PWM = map(Ch2PWM, 0, 255, motorStart, 255);       // scale value so that it uses the motorStart value to start the motor higher than zero PWM if set
  Ch2PWM = constrain(Ch2PWM, 0, 255);                  // constrain PWM value to 0-255 since that's the max value of PWM
  //Ch3PWM = Ch2PWM;                                     // make opposing wheels the same value

  InputX = mixX;                                       // do all of the above for the other axis
  myPIDX.Compute();
  Ch1PWM = abs(OutputX);  
  Ch1PWM = Ch1PWM*motorGain;
  Ch1PWM = map(Ch1PWM, 0, 255, motorStart, 255);
  Ch1PWM = constrain(Ch1PWM, 0, 255);
  //Ch4PWM = Ch1PWM;
  
  
  // Y axis
 
  if (OutputY <= deadSpot*-1)                            // decide which way to turn the wheels based on deadSpot variable
  {

  analogWrite(3, Ch2PWM);                                // set PWM pins 
  analogWrite(5, 0);

  }
  else if (OutputY >= deadSpot)                          // decide which way to turn the wheels based on deadSpot variable
  { 

  analogWrite(5, Ch2PWM);  
  analogWrite(3, 0);
  }
  else
  {
  analogWrite(5, 0);                                      // if we are within the deadspot turn off both wheels
  analogWrite(3, 0); 
  }  
  
  // X axis
  
  if (OutputX >= deadSpot)                                // do all of the above for the other axis
  {

  analogWrite(9, Ch1PWM);  
  analogWrite(6, 0);
  }
  else if (OutputX <= deadSpot*-1)
  { 

  analogWrite(6, Ch1PWM);  
  analogWrite(9, 0);
  }
  else
  {
  analogWrite(6, 0);  
  analogWrite(9, 0); 
  }
  }
  
  //delay(200);
}



