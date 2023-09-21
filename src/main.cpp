#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <math.h>

typedef SimpleKalmanFilter KalmanFilter;

enum Motions
{
  ACC_X,
  ACC_Y,
  ACC_Z,
  GYRO_X,
  GYRO_Y,
  GYRO_Z,
  LENGTH
};

KalmanFilter *filters = (KalmanFilter *)malloc(sizeof(KalmanFilter) * Motions::LENGTH);
MPU6050 gyro;

void setup()
{
  for (int i = 0; i < Motions::LENGTH; i++)
    filters[i] = KalmanFilter(1000, 1000, 0.01);

  Serial.begin(115200);

  Serial.println("Initializing gyro sensor...");
  Wire.begin();
  gyro.initialize();
  Serial.println("Initialized!");

  bool status = gyro.testConnection();

  Serial.println("Testing sensor connection...");

  if (!status)
  {
    Serial.println("Sensor connection failed! Waiting...");

    while (!gyro.testConnection())
      delay(50);
  }

  Serial.println("Sensor connection successful!");
}

void loop()
{
  int16_t data[Motions::LENGTH];
  gyro.getMotion6(&data[ACC_X], &data[ACC_Y], &data[ACC_Z], &data[GYRO_X], &data[GYRO_Y], &data[GYRO_Z]);

  for (int16_t value : data)
    Serial.printf("%d\t", value);

  for (int i = 0; i < Motions::LENGTH; i++)
    Serial.printf("%d\t", (int16_t)round(filters[i].updateEstimate(data[i])));

  Serial.print("\n");
}