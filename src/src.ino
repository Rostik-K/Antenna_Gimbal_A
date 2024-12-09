#include <Wire.h>
#include <SCServo.h>
#include <ESL_LIS3MDL.h>
#include <LSM6.h>

#include <math.h>
//#include "test_print.h"

//#define DEBUG            
// #define YAW_SERVO_TEST    1
// #define PITCH_SERVO_TEST  1
//#define PRINT_MAG        1
//#define PRINT_ACC    1
//#define PRINT_GYRO   1
#define AIMED_PITCH 140

// I2C Config
#define I2C_0 Wire
#define I2C_1 Wire1
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

// UART Driver communication
#define SERVO_SERIAL Serial2 // Use Serial2 for UART
#define TXD_PIN 4           // Set TX pin (change to your wiring)
#define RXD_PIN 5           // Set RX pin (change to your wiring)

#define YAW_SERVO_ID    2
#define PITCH_SERVO_ID  1

#define DEMPHER_VALUE_DEGREES 1
#define DEMPHER_PITCH_VALUE_DEGREES 1


// Servo 
#define YAW_SEROV_ID 1 //-- YAW
// ID 2 -- PITCH
/* byte ID[2];
u16 Position[2];
u16 Speed[2]; */

float aimedHeading;
float pitch;
float roll;
const int startingHeadServoPos = 250;
int previousHeadingFix = startingHeadServoPos;


SCSCL sc;
ESL_LIS3MDL mag(&I2C_1);
LSM6 imu;

void setup() {
  // USB Serial
  Serial.begin(9600);
  delay(500);
  Serial.println("USB Serial Init!");

  I2C_1.setSDA(I2C_SDA_PIN);
  I2C_1.setSCL(I2C_SCL_PIN);
  I2C_1.begin();
  imu.setBus(&I2C_1);

  if(!mag.init()) {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while(1);
  }
  mag.enableDefault();

  if (!imu.init()) {
    Serial.println("Failed to initialize the LSM6 imu!");
  } else {
    Serial.println("LSM6 imu initialized successfully!");
  }
  imu.enableDefault();

  // Servo 
  SERVO_SERIAL.setTX(TXD_PIN);
  SERVO_SERIAL.setRX(RXD_PIN);
  SERVO_SERIAL.begin(1000000);
  sc.pSerial = &SERVO_SERIAL;

  delay(1000);

  Serial.println("Initializing servos...");

  delay(1000); 
}

void loop() {
  float currentHeading;
  float currentPitch;
  float currentRoll;

  currentHeading = heading();
  currentPitch = 0;
  currentRoll = 0;

  int headingError = aimedHeading - currentHeading;
  if (headingError < -180) {
    headingError += 360;
  } else if (headingError > 180) {
    headingError -= 360;
  }
  
  // Update servo position if needed
  int headingFix = startingHeadServoPos + headingError;
  if (abs(headingFix - previousHeadingFix) > DEMPHER_VALUE_DEGREES)
  {
    if ((headingFix < 180) && (headingFix > 0))
    {
      if (headingFix < previousHeadingFix)
        previousHeadingFix--;
      else
        previousHeadingFix++;
        sc.WritePos(YAW_SERVO_ID, previousHeadingFix, 0, 1500);
    }
  }

  /* // Update roll servo position
  int rollFix = startingRollServoPos + (int)filteredRoll; // Adjust based on your application
  if (abs(rollFix - rollServo.read()) > DEMPHER_VALUE_DEGREES) {
      rollServo.write(rollFix);
  } */



  delay(20);
}

/*
 * Normalization of accelerometer raw data
 */
void normalize(float *x, float *y, float *z)
{
    float length = sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (length != 0)
    {
        *x /= length;
        *y /= length;
        *z /= length;
    }
}

float computeHeading(float mx, float my, float mz, float ax, float ay, float az)
{
    // Normalize accelerometer data
    normalize(&ax, &ay, &az);
    // Compute pitch and roll
    pitch = asin(-ax);
    roll = asin(ay / cos(pitch));

    // Compensate magnetometer data for pitch and roll
    float mx_comp = mx * cos(pitch) + mz * sin(pitch);
    float my_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    // Compute heading
    float heading = atan2(my_comp, mx_comp);
    // Convert heading to degrees
    heading *= 180.0 / M_PI;

    // Normalize heading to 0-360 degrees
    if (heading < 0)
    {
        heading += 360.0;
    }
    return heading;
}

float heading(void) {
  mag.read();
  imu.read();
  
  float heading = computeHeading(mag.m.x, mag.m.y, mag.m.z, imu.a.x, imu.a.y, imu.a.z);

  return heading;
}

/* 

  // Create servo objects
  SCSCL sc;     // SC series servo object

  void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    // Initialize the servo serial port
   

    

    // Debug output
  }
    
  void loop() {
    Position[0] = 600;
    Position[1] = 600;
    Speed[0] = 200;
    Speed[1] = 200;

    sc.SyncWritePos(ID, 2, Position, 0, Speed);
    delay(1000); // Wax1it for motion to complete

    Position[0] = 200;
    Position[1] = 200;
    Speed[0] = 200;
    Speed[1] = 200;
    // Example: Rotate both motors back to 0 degrees in parallel
    sc.SyncWritePos(ID, 2, Position, 0, Speed);//Servo((ID1/ID2)) moves at max speed=1500, moves to position=20.

    delay(1000); // Wait for motion to complete
  }
 */