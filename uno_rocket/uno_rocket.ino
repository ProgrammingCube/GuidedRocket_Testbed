#include <PID_v1.h>

#include <ArduinoSort.h>
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

double Setpoint;
double Input;
double Output;

double Kp=0, Ki=10, Kd=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

Servo canard[4];

uint8_t canard_pin[] = {3, 6, 5, 9};

float offset_x, offset_y;
float angle_x, angle_y;

void configureSensor(void)
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < 4; i++)
  {
    canard[i].attach(canard_pin[i]);
    canard[i].write(90);
  }

  if(!lsm.begin())
  {
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  configureSensor();
  offset_x = getDriftX();
  offset_y = getDriftY();

  Setpoint = 90;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  angle_x = atan(accel.acceleration.y/sqrt((accel.acceleration.x*accel.acceleration.x) + (accel.acceleration.z*accel.acceleration.z)));
  angle_y = atan(accel.acceleration.x/sqrt((accel.acceleration.y*accel.acceleration.y) + (accel.acceleration.z*accel.acceleration.z)));
  float x_angle = ((angle_x - offset_x) * (180 / PI) + 90);
  float y_angle = ((angle_y - offset_y) * (180 / PI) + 90);
  //Serial.print("X Angle: ");
  //Serial.println(angle_x - offset_x);

  Input = x_angle;

  myPID.Compute();

  canard[3].write(180 - Output);

  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);
  Serial.print(" ");
  Serial.println(Setpoint);
  /*
  canard[1].write(180 - x_angle);
  canard[3].write(x_angle);
  canard[0].write(y_angle);
  canard[2].write(180 - y_angle);
  */
  //Serial.print("Y Angle: ");
  //Serial.println(angle_y - offset_y);
  //Serial.println(((angle_x - offset_x) * (180 / PI) + 90));
  //Serial.println();
  //Serial.println();
  //delay(500);
}

float getDriftX()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  float x_array[200];
  for (uint8_t i = 0; i < 200; i++)
  {
    x_array[i] = atan(accel.acceleration.y/sqrt((accel.acceleration.x*accel.acceleration.x) + (accel.acceleration.z*accel.acceleration.z)));
  }
  sortArray(x_array, 200);
  return x_array[100];
}

float getDriftY()
{
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  //double x_array[200];
  float y_array[200];
  for (uint8_t i = 0; i < 200; i++)
  {
    y_array[i] = atan(accel.acceleration.x/sqrt((accel.acceleration.y*accel.acceleration.y) + (accel.acceleration.z*accel.acceleration.z)));
  }
  sortArray(y_array, 200);
  return y_array[100];
}
