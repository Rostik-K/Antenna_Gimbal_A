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

// #define TEST_GIMBAL_UART

//  #define TEST_GIMBAL_YAW
//  #define TEST_GIMBAL_PITCH    
//  #define TEST_GIMBAL_SYNC

  #define TEST_SERVO_FEEDBACK
//  #define TEST_GIMBAL_IMU
//  #define TEST_GIMBAL_HEADING
//    #define PRINT_MAG   1
//    #define PRINT_ACC   1
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
#define PITCH_MAX_LIMIT   640            // CS09 Servos by Waveshare initialy have 0-300 degrees controlled rotation angle. And due it`s design we should use 0-1023 range int values respectively.
#define PITCH_MIN_LIMIT   280
#define YAW_MAX_LIMIT     1000    // As 0 
#define YAW_MIN_LIMIT     0       // As 300

// Gayball Config
#define DEMPHER_VALUE_DEGREES         1
#define DEMPHER_PITCH_VALUE_DEGREES   1
#define AIMED_PITCH                   200
#define SMOOTHING_STEPS  8

// Filtering
// Alpha low pass filter
#define ALPHA 0.1

// Defines
#ifdef DEBUG
  #define LOG(x) DEBUG_SERIAL.println(x)
#else
  #define LOG(x) // No-op
#endif

// Gayball Variables
int base_yaw = 0;
int base_pitch = 0;

float aimed_heading = 0;
float pitchHistory[SMOOTHING_STEPS];
int magHistoryPosition = 0;

int previousPitch = 0;
const int targetAngle = 0; 

float filteredYaw = 0.0;              // Filtered yaw angle
float targetAzimuth = 90.0;           // Target azimuth in degrees

// KALMAN
double pitch = 0, roll = 0, yaw = 0;  // Orientation angles (degrees)
double dt = 0.02;    

double Q_angle = 0.1;  // Process noise variance for the accelerometer
double Q_bias = 0.003;   // Process noise variance for the gyroscope bias
double R_measure = 0.03; // Measurement noise variance
double angle = 0, bias = 0, rate = 0; // Kalman filter state variables
double P[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix
 
unsigned long lastTime; // For calculating loop time

// Base instances
ESL_LIS3MDL mag(&I2C_1);
LSM6 imu;
SCSCL sc;

void setup() {
  // Indication LED init 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // USB Serial Init
  DEBUG_SERIAL.begin(DEBUG_SERIAL_SPEED);
  //while(!DEBUG_SERIAL) {delay(10);}           // BUG
  delay(100);
  LOG("USB Serial Enabled!");

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
    //LOG("Gimbal PITCH test started!");
    while(1) {testServo(PITCH_SERVO_ID, &sc,  &DEBUG_SERIAL, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);}
  #endif

  #ifdef TEST_GIMBAL_YAW
   // LOG("Gimbal YAW test started!");
    while(1) {testServo(YAW_SERVO_ID, &sc, &DEBUG_SERIAL, YAW_MIN_LIMIT, YAW_MAX_LIMIT);}
  #endif

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

  // Test different UART output
  #if defined(DEBUG) && defined(TEST_GIMBAL_UART)
    while(1) {
      // USB
      DEBUG_SERIAL.println("HELLO FROM USB!");
      // UART0
      SDCARD_SERIAL.println("HELLO FROM UART0!");
      // UART1
      delay(500);
    }
  #endif
  // Test 2 Servo rotation in sync
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
  // Just calculate heading 
  #if defined(DEBUG) && defined(TEST_GIMBAL_HEADING)
    while(1) {LOG(heading());}
  #endif

  #if defined(DEBUG) && defined(TEST_SERVO_FEEDBACK)
    int pitch_pos = 0;
    int yaw_pos = 0; 
    while(1) {
      yaw_pos = sc.ReadPos(YAW_SERVO_ID);
      pitch_pos = sc.ReadPos(PITCH_SERVO_ID);

      DEBUG_SERIAL.print("YAW Pos: ");
      DEBUG_SERIAL.print(yaw_pos);
      DEBUG_SERIAL.print(". ");

      DEBUG_SERIAL.print("PITCH Pos: ");
      DEBUG_SERIAL.print(pitch_pos);
      DEBUG_SERIAL.println("."); 
  #endif

  // Setup delay
  delay(1000); 

  // Initial Gimbal Setups
    // Calibrate MAG
  setImuBasePosition(); // mb take north as reference point 
  setBasePosition(&base_yaw, &base_pitch);

  DEBUG_SERIAL.print("BASE YAW / PITCH: ");
  DEBUG_SERIAL.print(base_yaw);
  DEBUG_SERIAL.print(" | ");
  DEBUG_SERIAL.println(base_pitch);

}

void loop() {

  // COMBINE (NO SYNC)
  // Read IMU data for pitch compensation
  /*   
    imu.read();
    mag.read();

    // Calculate azimuth (yaw) using the magnetometer data
    //float yaw = getAzimuth(mag.m.x, mag.m.y);
    float yaw = heading();
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
    delay(20); */

  // PITCH TEST

  /*   unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0; // Calculate time in seconds
    if (dt == 0) dt = 0.001;               // Prevent division by zero
    lastTime = currentTime;

    pitch = Kalman_filter(pitch, mag.m.x, atan2(-imu.a.x, sqrt(imu.a.y * imu.a.y + imu.a.z * imu.a.z)) * 180.0 / M_PI);
      
    float compensationAnglePitch = targetAngle - pitch;

    int positionPitch = map(compensationAnglePitch, -60, 60, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);
    positionPitch = constrain(positionPitch, PITCH_MIN_LIMIT, PITCH_MAX_LIMIT);
  */

  /*   
    yaw = atan2(mag.m.x, mag.m.y); // Calculate yaw from magnetometer readings
    float declinationAngle = -0.1783; // Declination in radians for -10° 13'
    yaw += declinationAngle;          // Adjust for magnetic declination
  
    // Normalize yaw to 0-360 degrees
    if (yaw < 0) yaw += 2 * PI;
    if (yaw > 2 * PI) yaw -= 2 * PI;
    yaw = yaw * 180.0 / PI; // Convert yaw to degrees
  
    float compensationAngleYaw = targetAngle - yaw;
    int positionYaw = map(compensationAngleYaw, -180, 180, YAW_MIN_LIMIT, YAW_MAX_LIMIT);
    positionYaw = constrain(positionYaw, YAW_MIN_LIMIT, YAW_MAX_LIMIT);

    sc.WritePos(PITCH_SERVO_ID, positionPitch, 0, 2000);
    sc.WritePos(YAW_SERVO_ID, positionYaw, 0, 2000);
  */

  delay(20);
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

double Kalman_filter(double angle, double gyroRate, double accelAngle) {
  // Predict
  rate = gyroRate - bias;
  angle += dt * rate;
 
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
 
  // Update
  double S = P[0][0] + R_measure; // Estimate error
  double K[2];                    // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
 
  double y = accelAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;
 
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];
 
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
 
  return angle;
}

void setBasePosition(int* base_yaw_pos, int* base_pitch_pos) {
  // Take 40 samples of position feedback of each axis
  int sample_num = 40;
  int yaw_pos_sum = 0;
  int pitch_pos_sum = 0;
  int yaw_pos_data[sample_num] = {0};
  int pitch_pos_data[sample_num] = {0};

  for(int i = 0; i < sample_num; i++) {
    yaw_pos_data[i] = sc.ReadPos(YAW_SERVO_ID);
    pitch_pos_data[i] = sc.ReadPos(PITCH_SERVO_ID);
    delay(10); // delay for stability
  }

  // get average and set BASE
  for(int i = 0; i < sample_num; i++) {
    yaw_pos_sum += yaw_pos_data[i];
    pitch_pos_sum += pitch_pos_data[i];
  }

  *base_yaw_pos = yaw_pos_sum / sample_num;
  *base_pitch_pos = pitch_pos_sum / sample_num;

  #ifdef DEBUG
    DEBUG_SERIAL.print("YAW BASE = ");
    DEBUG_SERIAL.print(*base_yaw_pos);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print("PITCH BASE = ");
    DEBUG_SERIAL.print(*base_pitch_pos);
    DEBUG_SERIAL.println(".");
  #endif
}

void setImuBasePosition(void) {
  float controlHeading;
  
  // 8 times blink 0.5 seconds
  /*   for (int i = 0; i < 9; i++)
  {
    digitalWrite(LED_ONBRD_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_ONBRD_PIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
  } */

  // Fix the required direction
  aimed_heading = 0;
  magHistoryPosition = 0;
  while (magHistoryPosition < SMOOTHING_STEPS)
  {
    controlHeading = heading();
    pitchHistory[magHistoryPosition] = (pitch * 180) / PI;
    aimed_heading += controlHeading;
    delay(50);
    magHistoryPosition++;
  }
  magHistoryPosition = 0;
  aimed_heading = controlHeading;
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
