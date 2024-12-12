#include <Wire.h>
#include <SCServo.h>
#include <ESL_LIS3MDL.h>
#include <LSM6.h>
#include <math.h>
#include <stdint.h>
#include "test_print.h"
//#include "utils.h"

/* CONFIGS  */

#define DEBUG   // Adds UART output to funcitons
//  #define TEST_GIMBAL_YAW
//  #define TEST_GIMBAL_PITCH    
//  #define TEST_GIMBAL_SYNC

//  #define TEST_GIMBAL_IMU
//  #define TEST_GIMBAL_HEADING
//   #define PRINT_MAG   1
//   #define PRINT_ACC   1
//    #define PRINT_GYRO  1

#define LED_PIN 25
// I2C Config
#define I2C_0 Wire
#define I2C_1 Wire1
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

// UART Config 
#define DEBUG_SERIAL_SPEED  9600
#define SERVO_SERIAL_SPEED  1000000 // 1.000.000
#define DEBUG_SERIAL        Serial 
#define SERVO_SERIAL        Serial2  // Use Serial2 for UART
#define TXD_PIN 4             // Set TX pin (change to your wiring)
#define RXD_PIN 5             // Set RX pin (change to your wiring)

// Servo Config
#define YAW_SERVO_ID      1 //-- YAW     ?????
#define PITCH_SERVO_ID    2 //-- PITCH
#define PITCH_MAX_LIMIT   600            // CS09 Servos by Waveshare initialy have 0-300 degrees controlled rotation angle. And due it`s design we should use 0-1023 range int values respectively.
#define PITCH_MIN_LIMIT   250
#define YAW_MAX_LIMIT     1023
#define YAW_MIN_LIMIT     0

// Gayball Config
#define DEMPHER_VALUE_DEGREES         1
#define DEMPHER_PITCH_VALUE_DEGREES   1
#define AIMED_PITCH                   200

// Filtering
// Alpha low pass filter
#define ALPHA 0.1

// Defines
#ifdef DEBUG
  #define LOG(x) DEBUG_SERIAL.println(x)
#else
  #define LOG(x) // No-op
#endif

//#define PITCH_WRITE(pos) sc.WritePos(PITCH_SERVO_ID, pos, 0, 1500)
float filteredPitch = 0.0;

// Gayball Variables
float aimedHeading;
float pitch;
float roll;
const int startingHeadServoPos = 250;
int previousHeadingFix = startingHeadServoPos;
int previousPitch = 0;
const int targetAngle = -40; 

float filteredYaw = 0.0;              // Filtered yaw angle
float targetAzimuth = 90.0;           // Target azimuth in degrees


// Base instances
ESL_LIS3MDL mag(&I2C_1);
LSM6 imu;
SCSCL sc;

float getPitch(int16_t x, int16_t z) {
  return atan2(x, z) * 180.0 / M_PI; // Convert from radians to degrees
}

float getAzimuth(float mx, float my) {
  float azimuth = atan2(my, mx) * (180.0 / M_PI); // Convert radians to degrees
  if (azimuth < 0) {
    azimuth += 360.0; // Normalize to 0-360 degrees
  }
  return azimuth;
}

void setup() {
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  // LED

  // USB Serial Init
  DEBUG_SERIAL.begin(DEBUG_SERIAL_SPEED);
  //while(!DEBUG_SERIAL) {delay(10);}           // BUG
  delay(100);
  LOG("USB Serial Enabled!");

  // I2C Init
  /*
   *  TODO: 
   *     Validate if particularx Sensor propertly works.
   */
  I2C_1.setSDA(I2C_SDA_PIN);
  I2C_1.setSCL(I2C_SCL_PIN);
  I2C_1.begin();
  imu.setBus(&I2C_1); // Set specific I2C to IMU unit.


  // Magnetometer Init
  if(!mag.init()) {
    LOG("Failed to detect and initialize LIS3MDL magnetometer!");
    while(1);
  } else {
    LOG("LIS3MDL initialized successfully!");
  }
  mag.enableDefault();  // Enable Default setting for Mag.

  // IMU Init
  if(!imu.init()) {
    LOG("Failed to initialize the LSM6 imu!");
    while(1);
  } else {
    LOG("LSM6 imu initialized successfully!");
  }
  imu.enableDefault();  // Enable Default settings for IMU

  // Servo Serial Init
  SERVO_SERIAL.setTX(TXD_PIN);
  SERVO_SERIAL.setRX(RXD_PIN);
  SERVO_SERIAL.begin(1000000);
  if (!SERVO_SERIAL) {
    LOG("Failed to initialize UART for Servos!");
    while(1);
  } 
  sc.pSerial = &SERVO_SERIAL; // Give specific Serial to Servo
  LOG("UART Serial for servo Enabled!");

  // Test gimbal servos
  #ifdef TEST_GIMBAL_PITCH
    LOG("Gimbal PITCH test started!");
    while(1) {testServo(PITCH_SERVO_ID, &sc,  &DEBUG_SERIAL, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);}
  #endif

  #ifdef TEST_GIMBAL_YAW
   // LOG("Gimbal YAW test started!");
    while(1) {testServo(YAW_SERVO_ID, &sc, &DEBUG_SERIAL, YAW_MIN_LIMIT, YAW_MAX_LIMIT);}
  #endif

  #if defined(DEBUG) && defined(TEST_GIMBAL_SYNC)
    u8 id_array[] = {YAW_SERVO_ID, PITCH_SERVO_ID};
    u8 servoCount = sizeof(id_array);        // Number of servos

    u16 positions1[] = {YAW_MIN_LIMIT, PITCH_MIN_LIMIT};      // First set of positions LOW -- YAW, PITCH
    u16 positions2[] = {YAW_MAX_LIMIT, PITCH_MAX_LIMIT};      // Second set of positions HIGH

    //u16 time[] = {1000, 1000};         // Time to reach each position (1 second)
    u16 speed[] = {1500, 1500};           // Speed of movement for each servo

    // Test loop
    while(1) {            
        // Move to first position set
        sc.SyncWritePos(id_array, servoCount, positions1, 0, speed);
        delay(3000);                         // Wait for movement to complete

        // Move to second position set
        sc.SyncWritePos(id_array, servoCount, positions2, 0, speed);
        delay(3000);                         // Wait for movement to complete
    }
  #endif

  // Test IMU 
  #if defined(DEBUG) && defined(TEST_GIMBAL_IMU)
    while(1) {
      mag.read();
      imu.read();
      printImuRawData(&I2C_1, &DEBUG_SERIAL);
    }
  #endif

  #if defined(DEBUG) && defined(TEST_GIMBAL_HEADING)
    while(1) {LOG(heading());}
  #endif
  
  pinMode(LED_PIN, OUTPUT);

  // Setup delay

  delay(1000); 
}

void loop() {

  // COMBINE (NO SYNC)
  // Read IMU data for pitch compensation
  imu.read();
  mag.read();

  // Calculate azimuth (yaw) using the magnetometer data
  float yaw = getAzimuth(mag.m.x, mag.m.y);
  float pitch = getPitch(imu.a.x, imu.a.z); // Get raw pitch from IMU


  // Apply low-pass filter for stability
  filteredYaw = ALPHA * yaw + (1.0 - ALPHA) * filteredYaw;
  filteredPitch = ALPHA * pitch + (1.0 - ALPHA) * filteredPitch;

  // Calculate the compensation angle
  float compensationAngleYaw = targetAzimuth - filteredYaw;
  float compensationAnglePitch = targetAngle - filteredPitch;

  // Map the compensation angle to the servo's position range
  int positionYaw = map(compensationAngleYaw, -180, 180, YAW_MIN_LIMIT, YAW_MAX_LIMIT);
  positionYaw = constrain(positionYaw, YAW_MIN_LIMIT, YAW_MAX_LIMIT);

  int positionPitch = map(compensationAnglePitch, -60, 60, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);
  positionPitch = constrain(positionPitch, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);

  // Send the position to the servo
  sc.WritePos(YAW_SERVO_ID, positionYaw, 0, 1500);
  sc.WritePos(PITCH_SERVO_ID, positionPitch, 0, 1500);   


  // Delay for stability
  delay(20);

  // PITCH TEST

  // YAW TEST  (BAD)
  /*    
        mag.read();
        // Calculate azimuth (yaw) using the magnetometer
        float yaw = getAzimuth(mag.m.x, mag.m.y);

        // Low-pass filter to smooth the yaw value
        filteredYaw = ALPHA * yaw + (1.0 - ALPHA) * filteredYaw;

        // Calculate the compensation angle
        float compensationAngle = targetAzimuth - filteredYaw;

        // Map the compensation angle to the servo's position range
        int position = map(compensationAngle, -180, 180, YAW_MIN_LIMIT, YAW_MAX_LIMIT);
        position = constrain(position, YAW_MIN_LIMIT, YAW_MAX_LIMIT);

        // Send the position to the servo
        sc.WritePos(YAW_SERVO_ID, position, 0, 1500);
        LOG(position);

        delay(20); // Main loop delay  */  
      
    
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


// TEST-/-DEBUG Functions
void testServo(byte servo_id, SCSCL* servo, HardwareSerial* serial, int min_limit, int max_limit) {
  // Validate limits
  if (max_limit < 0 || min_limit < 0) {
    //serial->println("Error: Invalid servo limits!");
    return;
  }
  // Send commands to the servo
  // Move to max limit
  servo->WritePos(servo_id, max_limit, 0, 1000);
  #ifdef DEBUG
    serial->print("Servo Pos: ");
    serial->println(max_limit);
  #endif
  delay(3000);
  // Move to min limit
  servo->WritePos(servo_id, min_limit, 0, 1000); 
  #ifdef DEBUG
    serial->print("Servo Pos: ");
    serial->println(min_limit);
  #endif
  delay(3000);
}

void printImuRawData(TwoWire* imu_addres, HardwareSerial* output_stream) {

  char raw_data_array[64] = {0};

  #if defined(DEBUG) && defined(PRINT_MAG) && (PRINT_MAG == 1)
    // PRINT MAG DATA
    snprintf(raw_data_array, sizeof(raw_data_array), "MAG: %6d, %6d, %6d", mag.m.x, mag.m.y, mag.m.z);
    output_stream->println(raw_data_array); 
  #endif
  #if defined(DEBUG) && defined(PRINT_ACC) && (PRINT_ACC == 1)
    // PRINT ACC DATA
    snprintf(raw_data_array, sizeof(raw_data_array), "ACC: %6d, %6d, %6d", imu.a.x, imu.a.y, imu.a.z);
    output_stream->println(raw_data_array); 
  #endif 
  #if defined(DEBUG) && defined(PRINT_GYRO) && (PRINT_GYRO == 1)
    // PRINT GYRO DATA
    snprintf(raw_data_array, sizeof(raw_data_array), "GYRO: %6d, %6d, %6d", imu.g.x, imu.g.y, imu.g.z);
    output_stream->println(raw_data_array); 
  #endif
}

/*
 float currentHeading;
  float currentPitch;
  float currentRoll;

  // Compute heading
  currentHeading = heading();
  currentPitch = 0;
  currentRoll = 0;

  int headingError = aimedHeading - currentHeading;
  if (headingError < -180) {
    headingError += 360;
  } else if (headingError > 180) {
    headingError -= 360;
  }
  
  /*   // Update servo position if needed
    int headingFix = startingHeadServoPos + headingError;
    if (abs(headingFix - previousHeadingFix) > DEMPHER_VALUE_DEGREES)
    {
      if ((headingFix < 180) && (headingFix > 0))
      {
        if (headingFix < previousHeadingFix)
          previousHeadingFix--;
        else {
          previousHeadingFix++;
          sc.WritePos(YAW_SERVO_ID, map(previousHeadingFix, 0, 360, ), 1500, 50); 
        }
      }
    } */

  // Update roll servo position
 /*  int rollFix = startingRollServoPos + (int)filteredRoll;         // Adjust based on your application
  if (abs(rollFix - rollServo.read()) > DEMPHER_VALUE_DEGREES) {
      rollServo.write(rollFix);
  } 

  if (abs(currentPitch - previousPitch) > DEMPHER_PITCH_VALUE_DEGREES)
  {
    if (((AIMED_PITCH + currentPitch) > 0) && ((AIMED_PITCH + currentPitch) < 180))
    {
      previousPitch = currentPitch;
      sc.WritePos(PITCH_SERVO_ID, map(AIMED_PITCH + currentPitch, 0, 360, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT), 0, 50);
    }
  }
*/
