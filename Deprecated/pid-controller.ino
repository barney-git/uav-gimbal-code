// Gimbal feedback system

// Live -> 5V
// Ground -> GND
// Signal (SDA) -> A4
// Clock (SDL) -> A5 

// Accelerometer range set to: +-4G
// Gyro range set to: 2000 degrees/s
// Accelerometer data rate set to: 104 Hz
// Gyro data rate set to: 104 Hz


// Define PIDs

struct PID {

  float K_p; // Proportional gain
  float K_i; // Integral gain
  float K_d; // Differential gain

  float integral;
  float prev_error;
  float prev_output;

  PID(float kp, float ki, float kd) {
    K_p = kp;
    K_i = ki;
    K_d = kd;
    integral = 0;
    prev_error = 0;
    prev_output = 0;
  }

  float reaction(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    Serial.print("setpoint: ");
    Serial.print(setpoint);
    Serial.print(" measurement: ");
    Serial.print(measurement);
    Serial.print(" error: ");
    Serial.print(error);

    integral += (error * dt);
    float integral_limit = 50;
    integral = constrain(integral, -integral_limit, integral_limit);

    float derivative = (error - prev_error) / dt; // Would be cleaner to get derivative (angular velocity) directly from accelerometer

    prev_error = error;

    float output = prev_output + K_p * error + K_i*integral - K_d * derivative;
    prev_output = output;

    Serial.print(" output: ");
    Serial.println(output);

    return output;

  }
};

PID pitchPID(0.35,0,0);
PID rollPID(0.1,0,0);

// IMU to Servos

#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX sox;

#include <Servo.h>
Servo servo_pitch;
Servo servo_roll;

unsigned long prev_time;
float dt;
unsigned long prev_servo_update = 0;

float pitch;
float roll;

float tau = 0.5; // Time constant over which drift is corrected

int iteration;

void accelerometer_angle(const sensors_event_t &accel,float &pitch, float &roll) {

  float X, Y, Z;

  float aX = accel.acceleration.x;
  float aY = accel.acceleration.y;
  float aZ = accel.acceleration.z;

  roll = atan2(aY,aZ);
  pitch = atan2(aX,sqrt(aY*aY+aZ*aZ));

}

void gyro_angle(const sensors_event_t &gyro, float &pitch, float &roll) {

  float gX = gyro.gyro.x;
  float gY = gyro.gyro.y;
  float gZ = gyro.gyro.z;

  roll = gX * dt;
  pitch = gY * dt;

}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  servo_pitch.attach(10);
  servo_roll.attach(11);

  prev_time = micros();
  iteration = 0;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sox.getEvent(&accel, &gyro, &temp);

  accelerometer_angle(accel,pitch, roll);

}

void loop() {

  unsigned long now = micros();
  dt = (now - prev_time) / float(1000000);
  prev_time = now;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sox.getEvent(&accel, &gyro, &temp);

  float g_pitch, g_roll;
  gyro_angle(gyro,g_pitch, g_roll);

  float a_pitch, a_roll;
  accelerometer_angle(accel,a_pitch, a_roll);

  // Serial.println(tau);
  float alpha = tau/(tau+dt);
  alpha = 0.95;
  // Serial.println(alpha);

  pitch = alpha * (pitch + g_pitch) + (1 - alpha) * a_pitch;
  roll = alpha * (roll + g_roll) + (1 - alpha) * a_roll;

  float pitch_deg = pitch * 180 / PI;
  float roll_deg = roll * 180 / PI;

  // float pitch_output = pitchPID.reaction(0, pitch_deg, dt);
  float pitch_output = 0;
  float roll_output = rollPID.reaction(0, roll_deg, dt);


  // Test roll vs output
  // Serial.print(roll_deg);
  // Serial.print(",");
  // Serial.print(roll_output);
  // Serial.println(",");

  // Serial.print(pitch_output);
  // Serial.print(",");

  // Serial.print(pitch_deg);
  // Serial.print(",");
  // Serial.println(0);

  iteration += 1;

  if (millis() - prev_servo_update >= 20) {

    float servo_pitch_write = 90 + pitch_output;

    float roll_output = rollPID.reaction(0, roll_deg, dt);

    float servo_roll_write = roll_output;
    servo_pitch.write(servo_pitch_write);
    servo_roll.write(servo_roll_write);
    prev_servo_update = millis();
  }
}