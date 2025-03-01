/* TO-DO:
 * calibrate gy521, also set logic for PID # to go back/forwards in loop, get pitch value to find where negative and positive
 * pitch deck for project can be ultra shrimple; show project purpose, design, issues in development, etc.
*/

// Lib include
#include <GY521.h> // Read MPU6050 data using Wire.h, return angle values for PID calculation

/* PID and Accelerometer
 * INT, SCL, SDA are all handled by GY521.h lib. Pinout is predefined w/ macros in lib header. */
GY521 mpu(0x68);                        // Create sensor object for GY521 lib interface
const int Kp { 0 };                     // Proportional-Gain constant for PID
const int Ki { 0 };                     // Integral-Gain constant for PID
const int Kd { 0 };                     // Derivative-Gain constant for PID
int pid_I { 0 };                        // Integral value for PID, global to get integral total
int error { 0 };                        // Error = SP - PV, manipulate output to get closer to SP
int previous_error { 0 };               // Used for Derivative calculation
const float targetAngle { 0 };          // Set Point (SP), aligned upwards against the normal
float pitchAngle { 0 };                 // Process Value (VP), Y-axis angle 
const float radsToDegrees { 180/M_PI }; // Convert radian measure of accelerometer to degrees

float yawAngle { 0 };   // X-axis angle

// LN298
#define IN1 26
#define IN2 27
#define ENA 25
int ena_speed { 0 }; // Motor speed

// Timer
float program_time { 0 };  // Ellapsed time in millisceonds from program start
float previous_time { 0 }; 
float cycle_time { 0 };    // Cycle time taken to complete PID calculation

void driveStop()
{
  // all OUT set to Low
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
}

void driveBackward()
{
  // drive both motors counterclockwise
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void driveForward()
{
  // drive both motors clockwise
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void setup() {
  Serial.begin(115200); // 115200 baud rate

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(ENA, ena_speed);

  // Start I2C Wire connection to wake MPU then check if MPU is awake before start
  Wire.begin();
  if (mpu.wakeup() == false)
  {
    Serial.println("MPU is asleep after I2C Start--Angles will not be read from sensor. (0x68 ADDR)");
  }

  // Set sensor sensitivity and calibrate
  mpu.setAccelSensitivity(2);    // 8g of force max
  mpu.setGyroSensitivity(1);     // 500 degreees/second max
  mpu.setNormalize(true);        // Normalize angle measurements
  mpu.calibrate(20);             // 20 reads on MPU to calibrate angles to 0
}

int calculatePID()
{
  // Linear models since exponential is too hard/process demanding

  // P controlled by error, error is proportional to correction, provide quick correction to SP
  int pid_P = Kp*error;

  /* I controlled by total error over time, improves stability but must be fine tuned more than P/D. Has greater effect on PID over time,
  Calculated by multiplying Kp by error and cycle time/time of how often PID is calculated, then take summation of integrals to get integral total */

  pid_I = (Ki*error) + pid_I;

  /* D controlled by rate of change of error, aims to predict change of process value to correct movement, prevent overshooting.
  Biases output to prevent PV from overshooting SP */
  int pid_D = Kd*((error-previous_error) / cycle_time);

  int pid = pid_P + pid_I + pid_D;
  return pid;
}

void getAngle()
{
  // Read data from sensor w/ read() then get XYZ-angles, only pitch angle is needed for PID
  mpu.read();
  pitchAngle = mpu.getAngleY(); 
  yawAngle = mpu.getAngleX();
}

void loop() 
{
  previous_time = program_time; // last recorded time
  program_time = millis();      // millis records time since program start
  cycle_time = program_time - previous_time; 

  getAngle();
  error = targetAngle - pitchAngle; 
  previous_error = error;     


  Serial.print("Angle Y reading | "); Serial.println(pitchAngle);
  Serial.print("Angle X reading | "); Serial.println(yawAngle);

  Serial.print("Ellapsed Time (s): "); Serial.println(program_time/1000);
  delay(1000);
}
