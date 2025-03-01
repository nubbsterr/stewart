/* TO-DO:
 * calibrate gy521
 * patch error_read for getAngle, returning -1
 * get pid equation/formula stuff going, conplementaru filter to filter out noise from accelerometer
 * remove I from pid maybe?
 * get angle and acceleration from sensor, angle from I2C lib
 * pitch deck for project can be ultra shrimple; show project purpose, design, issues in development, etc.
*/

// Lib include
#include <Wire.h> // I2C interface for Accelerometer
#include <math.h> // aTan function for degeree calculation


// PID 
const int Kp { 0 };
const int Ki { 0 };
const int Kd { 0 };
volatile int pid_I { 0 };          // Integral value for PID, global to get integral total
volatile int error { 0 };          // Error = SP - PV, manipulate output to get closer to SP
volatile int previous_error { 0 }; // Used for Derivative calculation
const float targetAngle { 0 };          // Set Point (SP), aligned upwards against the normal
volatile float pitchAngle { 0 };        // Process Value (VP), Y-axis angle 
volatile float yawAngle { 0 };          // X-axis angle
const float radsToDegrees { 180/M_PI }; // Convert radian measure of accelerometer to degrees

// Accelerometer
/* INT = interrupt, notifies data is available to read
 * SCL = serial clock, similar to baud rate, synchronize transmission of data
 * SDA = serial data, used for transferring data w/ I2C
*/ 
#define MPUADDR 0x68 
#define SCL 34
#define SDA 35
int16_t accel_X { 0 }; // horizonatal acceleration
int16_t accel_Y { 0 }; // vertical acceleration
int16_t accel_Z { 0 }; // vertical acceleration


// LN298
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENA 33
#define ENB 25
volatile int ena_speed { 0 }; // Speed of left-most motor
volatile int enb_speed { 0 }; // Speed of right-most motor

// Timer
volatile float current_time { 0 };  // Ellapsed time in millisceonds
volatile float previous_time { 0 };

void driveStop()
{
  // all OUT set to Low
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void driveBackward()
{
  // drive both motors counterclockwise
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void driveForward()
{
  // drive both motors clockwise
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void setup() {
  Serial.begin(115200); // 115200 baud rate

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, ena_speed);
  digitalWrite(ENB, enb_speed);

  // Begin I2C communication and wake accelerometer
  Wire.begin();
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x6B); // Wake MPU6050, write to pwr_mgmt register
  Wire.write(0);    // Send wake signal
  Wire.endTransmission(true); 
}

constexpr int calculatePID()
{
  // Linear models since exponential is too hard/process demanding

  // P controlled by error, error is proportional to correction, provide quick correction to SP
  int pid_P = Kp*error;

  /* I controlled by total error over time, improves stability but must be fine tuned more than P/D. Has greater effect on PID over time,
  Calculated by multiplying Kp by error and cycle time/time of how often PID is calculated, then take summation of integrals to get integral total */

  pid_I = (Ki*error) + pid_I;

  /* D controlled by rate of change of error, aims to predict change of process value to correct movement, prevent overshooting.
  Biases output to prevent PV from overshooting SP */
  int pid_D = Kd*((error-previous_error)/time);

  int pid = pid_P + pid_I + pid_D;
  return pid;
}

void getAngle()
{
  Wire.write(0x3B); // Write to Accel_X high-byte register
  Wire.endTransmission(true);

  /* Accelerometer is 16 bits but read only returns 8 bits, so run 2 reads.
   * First read is shifted 8 units/multiplied by 256, which isolates the top/high 8 bits.
   * Second read is not shifted and will get the bottom 8 bits.
   * The two reads are then logically OR'd; added together to get the total acceleration data. 
  */

  accel_X = Wire.read()<<8 | Wire.read(); 
  accel_Y = Wire.read()<<8 | Wire.read();
  accel_Z = Wire.read()<<8 | Wire.read();
}

void loop() 
{
  previous_time = current_time; // last recorded time
  current_time = millis();      // millis records time since program start
  error = targetAngle - 
  previous_error = error;       
  getAngle();

  Serial.print("Accel X reading | "); Serial.println(accel_X);
  Serial.print("Accel Y reading | "); Serial.println(accel_Y);
  Serial.print("Accel Z reading | "); Serial.println(accel_Z);

  Serial.print("Ellapsed Time (s): "); Serial.println(current_time/1000);
  delay(1000);
}
