#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

// Number of samples to average for bias estimation
const int NUM_SAMPLES = 500;

float gyro_z_bias = 0.0;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      delay(10);
  }

  Serial.println("Keep the robot completely still...");
  delay(2000);

  // Collect samples to estimate bias
  float sum = 0.0;
  int count = 0;
  while (count < NUM_SAMPLES)
  {
    if (IMU.gyroscopeAvailable())
    {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sum += z;
      count++;
    }
  }

  gyro_z_bias = sum / NUM_SAMPLES;

  Serial.print("Gyro Z bias (deg/s): ");
  Serial.println(gyro_z_bias, 6);
  Serial.println("Starting corrected readings...");
  delay(1000);
}

void loop()
{
  if (IMU.gyroscopeAvailable())
  {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    float corrected_z = z - gyro_z_bias;

    Serial.print("Raw Z: ");
    Serial.print(z, 4);
    Serial.print(" deg/s  |  Corrected Z: ");
    Serial.print(corrected_z, 4);
    Serial.println(" deg/s");
  }
}
