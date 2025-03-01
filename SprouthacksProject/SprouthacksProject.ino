/* TO-DO:
 * tune PID to lessen overcorrection by robot
 * speed map w/ PID: -350 to +350
 * pitch deck for project can be ultra shrimple; show project purpose, design, issues in development, etc.
*/

// Lib include
#include <GY521.h> // Read MPU6050 data using Wire.h, return angle values for PID calculation

/* PID and Accelerometer
 * INT, SCL, SDA are all handled by GY521.h lib. Pinout is predefined w/ macros in lib header. */
GY521 mpu(0x68);                        // Create sensor object for GY521 lib interface
const float Kp { 5000 };                 // Proportional-Gain constant for PID
const float Ki { 0.001 };                   // Integral-Gain constant for PID
const float Kd { 0.005 };              // Derivative-Gain constant for PID
int pid_I { 0 };                        // Integral value for PID, global to get integral total
int error { 0 };                        // Error = SP - PV, manipulate output to get closer to SP
int previous_error { 0 };               // Used for Derivative calculation
const float targetAngle { 0 };          // Set Point (SP), aligned upwards against the normal
float pitchAngle { 0 };                 // Process Value (VP), Y-axis angle 
float yawAngle { 0 };                   // X-axis angle

// LN298
#define IN1 26
#define IN2 27
#define ENA 25
int ena_speed { 0 }; // Motor speed

// Timer
float program_time { 0 };  // Ellapsed time in millisceonds from program start
float previous_time { 0 }; 
float cycle_time { 0 };    // Cycle time taken to complete PID calculation

// all OUT set to Low
void halt()
{
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
}

// Drive both motors counterclockwise
void driveBackwards(int motorSpeed)
{
  analogWrite(ENA, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

// Drive both motors clockwise
void driveForward(int motorSpeed)
{
  analogWrite(ENA, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

// Start I2C connection and calibrate MPU
void setup() {
  Serial.begin(115200); // 115200 baud rate

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  Wire.begin();
  if (mpu.wakeup() == false)
  {
    Serial.println("MPU is asleep after I2C Start--Angles will not be read from sensor. (0x68 ADDR)");
  }

  // Set sensor sensitivity, calibrate MPU and normalize angle return
  mpu.setAccelSensitivity(3);    // 16g of force max
  mpu.setGyroSensitivity(2);     // 1000 degreees/second max
  mpu.setNormalize(true);        
  mpu.calibrate(20);             
}

// Calculate PID using linear models since exponential is too hard/process demanding
float calculatePID()
{
  // P controlled by error, error is proportional to correction, provide quick correction to SP
  float pid_P = Kp*error;

  /* I controlled by total error over time, improves stability but must be fine tuned more than P/D. 
   * Has greater effect on PID over time, */
  pid_I = (Ki*error) + pid_I;

  /* D controlled by rate of change of error, aims to predict change of process value to correct movement, 
   * biases output to prevent overcorrection */
  float pid_D = Kd*((error-previous_error) / 20);

  float pid = pid_P + pid_I + pid_D;
  return pid;
}

// Get angles from GY521 using lib
void getAngle()
{
  // Read data from sensor w/ read() then get XYZ-angles, only pitch angle is needed for PID
  mpu.read();
  pitchAngle = mpu.getAngleY(); 
  yawAngle = mpu.getAngleX();
}

// Calculate cycle time and continously get angles, calculate error and PID, and manage movement
void loop() 
{
  previous_time = program_time; // last recorded time
  program_time = millis();      // millis records time since program start
  cycle_time = program_time - previous_time; 
  
  if (cycle_time <= 20)
  {
    cycle_time = 20 - cycle_time;
    delay(cycle_time);
  }

  getAngle();
  error = targetAngle - pitchAngle; 
  previous_error = error;
  float pid { calculatePID() };

  // Map ena_speed based on PID threshold from -350 to +350 to 0-255 range for analog, cannot exceed bounds
  ena_speed = map(pid, -150, 150, 0, 255); 
  if (ena_speed >= 255) ena_speed = 255;
  else if (ena_speed < 0)
  {
    ena_speed = abs(ena_speed);
    if (ena_speed >=255) ena_speed = 255;
  } 

  // Debugging yuh
  //Serial.print("Angle Y reading \t"); Serial.println(pitchAngle);
  //Serial.print("Angle X reading \t"); Serial.println(yawAngle);
  Serial.print("Speed \t"); Serial.println(ena_speed);
  Serial.print("PID \t"); Serial.println(pid);

  Serial.print("Ellapsed Time (s) \t"); Serial.println(program_time/1000);
  Serial.print("Cycle Time (ms) \t"); Serial.println(cycle_time);

  // Control robot movement based on PID
  if (pid < -5) driveForward(ena_speed); 
  else if (pid > 5) driveBackwards(ena_speed);
  else halt();
}
