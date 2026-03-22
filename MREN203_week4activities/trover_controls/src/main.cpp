#include <Arduino.h>
// Need this library installed to access the UNO Wifi Rev2 board's IMU
// For more details, look here:
// https://www.arduino.cc/reference/en/libraries/arduino_lsm6ds3/,→
#include <Arduino_LSM6DS3.h>

int EA = 5;
int I1 = 8;
int I2 = 11;
int EB = 6;
int I3 = 12;
int I4 = 13;

const byte SIGNAL_A = 10;
const byte SIGNAL_B = 9;
const byte SIGNAL_C = 2;
const byte SIGNAL_D = 3;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle track [m]
const double ELL = 0.2775;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Controller gains (use the same values for both wheels)
const double KP = 200.0; // Proportional gain
const double KI = 100.0; // Integral gain

/* VARIABLE DECLARATIONS */

// Motor PWM command variables [0-255]
short u_L = 0;
short u_R = 0;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variables to store estimated angular rates of wheels [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Variables to store estimated wheel speeds [m/s]
double v_L = 0.0;
double v_R = 0.0;

// Variables to store vehicle speed and turning rate
double v = 0.0;     // [m/s]
double omega = 0.0; // [rad/s]

// Variables to store desired vehicle speed and turning rate
double v_d = 0.0;     // [m/s]
double omega_d = 0.0; // [rad/s]

// Variable to store desired wheel speeds [m/s]
double v_Ld = 0.0;
double v_Rd = 0.0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Variables to store errors for controller
double e_L = 0.0;
double e_R = 0.0;
double e_Lint = 0.0;
double e_Rint = 0.0;

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store accelerations [g's]
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;

const float GYRO_Z_BIAS = 0; // CALIBRATE THIS!!!

// Struct definitions
struct VelocityCommand
{
  float lin_vel_x;
  float ang_vel_z;
};

struct WheelSpeeds
{
  float left;
  float right;
};

// Function prototypes
void decodeEncoderTicksL();
void decodeEncoderTicksR();
void forward(double vL, double vR, double t, VelocityCommand desired_speed);
short PI_controller(double e_now, double e_int, double k_P, double k_I);
WheelSpeeds ReadWheelSpeeds(VelocityCommand cmd);
WheelSpeeds Drive(VelocityCommand cmd);
void driveVehicle(short u_L, short u_R);
void SendSensorData(int left_ticks, int right_ticks, float gyro_z);
VelocityCommand ReceiveVelocityCommand();
void ReadImu();

// This function applies PWM inputs (u_L and u_R) to the right and left wheels
void driveVehicle(short u_L, short u_R)
{
  // LEFT WHEEL
  if (u_L < 0) // If the controller calculated a negative input...
  {
    digitalWrite(I3, HIGH); // Drive backward (left wheels)
    digitalWrite(I4, LOW);  // Drive backward (left wheels)

    analogWrite(EB, -u_L); // Write left motors command
  }
  else // the controller calculated a positive input
  {
    digitalWrite(I3, LOW);  // Drive forward (left wheels)
    digitalWrite(I4, HIGH); // Drive forward (left wheels)

    analogWrite(EB, u_L); // Write left motors command
  }

  // RIGHT WHEEL
  if (u_R < 0) // If the controller calculated a negative input...
  {
    digitalWrite(I1, LOW);  // Drive backward (right wheels)
    digitalWrite(I2, HIGH); // Drive backward (right wheels)

    analogWrite(EA, -u_R); // Write right motors command
  }
  else // the controller calculated a positive input
  {
    digitalWrite(I1, HIGH); // Drive forward (right wheels)
    digitalWrite(I2, LOW);  // Drive forward (right wheels)

    analogWrite(EA, u_R); // Write right motors command
  }
}

/*
  Serial communicaiton
*/
void SendSensorData(int left_ticks, int right_ticks, float gyro_z)
{
  // Send: "leftTicks,rightTicks,gyroZ\n"
  Serial.print(left_ticks);
  Serial.print(",");
  Serial.print(right_ticks);
  Serial.print(",");
  Serial.println(gyro_z); // println adds \n
}

VelocityCommand ReceiveVelocityCommand()
{
  VelocityCommand cmd = {0.0, 0.0};

  if (Serial.available() > 0)
  {
    Serial.println("serial available");

    String data = Serial.readStringUntil('\n');

    // Parse "0.5,0.3\n"
    int commaIndex = data.indexOf(',');

    float lin_vel_x = data.substring(0, commaIndex).toFloat();
    float ang_vel_z = data.substring(commaIndex + 1).toFloat();

    cmd.lin_vel_x = lin_vel_x;
    cmd.ang_vel_z = ang_vel_z;

    // Serial.print("lin: ");
    // Serial.print(lin_vel_x);
    // Serial.print(", Ang: ");
    // Serial.println(ang_vel_z);
  }

  return cmd;
}

void decodeEncoderTicksL()
{
  if (digitalRead(SIGNAL_B) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_L--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_L++;
  }
}

void decodeEncoderTicksR()
{
  if (digitalRead(SIGNAL_D) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_R--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_R++;
  }
}

// Compute the wheel rate from elapsed time and encoder ticks [rad/s]
double compute_wheel_rate(long encoder_ticks, double delta_t)
{
  double omega;
  omega = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / delta_t;
  return omega;
}

// Compute wheel speed [m/s]
double compute_wheel_speed(double omega_wheel)
{
  double v_wheel;
  v_wheel = omega_wheel * RHO;
  return v_wheel;
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R)
{
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R)
{
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}

// Compute v_L from v and omega
double compute_L_wheel_speed(double v, double omega)
{
  double v_wheel = 0.0;
  v_wheel = v - ELL / 2.0 * omega;
  return v_wheel;
}

// Compute v_R from v and omega
double compute_R_wheel_speed(double v, double omega)
{
  double v_wheel = 0.0;
  v_wheel = v + ELL / 2.0 * omega;
  return v_wheel;
}

// Wheel speed PI controller function
short PI_controller(double e_now, double e_int, double k_P, double k_I)
{
  short u;
  u = (short)(k_P * e_now + k_I * e_int);

  // Saturation (i.e., maximum input) detection
  if (u > 255)
  {
    u = 255;
  }
  else if (u < -255)
  {
    u = -255;
  }
  return u;
}

void ReadImu()
{
  // Read from the accelerometer
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(a_x, a_y, a_z);

    // Print the accelerometer measurements to the Serial Monitor
    // Serial.print(a_x);
    // Serial.print("\t");
    // Serial.print(a_y);
    // Serial.print("\t");
    // Serial.print(a_z);
    // Serial.print(" g\t\t");
  }

  // Read from the gyroscope
  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(omega_x, omega_y, omega_z);
    omega_z = omega_z - GYRO_Z_BIAS;

    // Print the gyroscope measurements to the Serial Monitor
    // Serial.print(omega_x);
    // Serial.print("\t");
    // Serial.print(omega_y);
    // Serial.print("\t");
    // Serial.print(omega_z);
    // Serial.print(" deg/s\n");
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Wait for serial connection before starting
  while (!Serial)
    delay(10);

  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicksL, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksR, RISING);

  // Check that the board is initialized
  if (!IMU.begin())
  {
    // Print an error message if the IMU is not ready
    Serial.print("Failed to initialize IMU :(");
    Serial.print("\n");
    while (1)
    {
      delay(10);
    }
  }

  // Read the sample rate of the accelerometer and gyroscope
  a_f = IMU.accelerationSampleRate();
  g_f = IMU.gyroscopeSampleRate();

  // Print these values to the serial window
  // Serial.print("Accelerometer sample rate: ");
  // Serial.println(a_f);
  // Serial.print("Gyroscope sample rate: ");
  // Serial.println(g_f);

  delay(1000);
}

void loop()
{
  t_now = millis();

  // put your main code here, to run repeatedly:
  // Perform control update every T milliseconds
  if (t_now - t_last >= T)
  {
    VelocityCommand vel_desired = ReceiveVelocityCommand();
    v_d = vel_desired.lin_vel_x;
    omega_d = vel_desired.ang_vel_z;

    ReadImu();

    // Estimate the rotational speed of each wheel [rad/s]
    omega_L = compute_wheel_rate(encoder_ticks_L, (double)(t_now - t_last));
    omega_R = compute_wheel_rate(encoder_ticks_R, (double)(t_now - t_last));

    // Compute the speed of each wheel [m/s]
    v_L = compute_wheel_speed(omega_L);
    v_R = compute_wheel_speed(omega_R);

    // Compute the speed of the vehicle [m/s]
    v = compute_vehicle_speed(v_L, v_R);

    // Compute the turning rate of the vehicle [rad/s]
    omega = compute_vehicle_rate(v_L, v_R);

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;

    // Compute the desired wheel speeds from v_d and omega_d
    v_Ld = compute_L_wheel_speed(v_d, omega_d);
    v_Rd = compute_R_wheel_speed(v_d, omega_d);

    // Compute errors
    e_L = v_Ld - v_L;
    e_R = v_Rd - v_R;

    // Integrate errors with anti-windup
    if (abs(u_L) < 255)
    {
      e_Lint += e_L;
    }
    if (abs(u_R) < 255)
    {
      e_Rint += e_R;
    }

    // Compute control signals using PI controller
    u_L = PI_controller(e_L, e_Lint, KP, KI);
    u_R = PI_controller(e_R, e_Rint, KP, KI);

    // Drive the vehicle
    driveVehicle(u_L, u_R);

    // Print some stuff to the serial monitor (or plotter)
    // Serial.print("Vehicle_speed_[m/s]:");
    // Serial.print(v);
    // Serial.print(",");
    // Serial.print("Turning_rate_[rad/s]:");
    // Serial.print(omega);
    // Serial.print(",");
    // Serial.print("u_L:");
    // Serial.print(u_L);
    // Serial.print(",");
    // Serial.print("u_R:");
    // Serial.print(u_R);
    // Serial.print("\n");

    // Test this
    // SendSensorData(100, 300, 0.487);

    t_now = t_last;
  }
}
