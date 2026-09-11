// Gimbal feedback system

// Live -> 5V
// Ground -> GND
// Signal (SDA / Blue) -> A4
// Clock (SDL / Yellow) -> A5 

// Accelerometer range set to: +-4G
// Gyro range set to: 2000 degrees/s
// Accelerometer data rate set to: 104 Hz
// Gyro data rate set to: 104 Hz

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

float pitch_output = 0;
float roll_output = 0;

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

  servo_pitch.attach(11); //\Botton pins
  servo_roll.attach(10); // Top pins

  servo_pitch.write(90);
  servo_roll.write(90);

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
  alpha = 0.96;
  // Serial.println(alpha);
  
  float a_pitch_wrapped = fmod((a_pitch - pitch) + PI, 2*PI) - PI + pitch;
  float a_roll_wrapped  = fmod((a_roll  - roll)  + PI, 2*PI) - PI + roll;

  pitch = alpha * (pitch + g_pitch) + (1 - alpha) * a_pitch_wrapped;
  roll  = alpha * (roll  + g_roll)  + (1 - alpha) * a_roll_wrapped;

  float pitch_deg = (pitch * 180 / PI);
  float roll_deg = (roll * 180 / PI) + 180;

  pitch_deg = constrain(pitch_deg, -90, 90);
  roll_deg = constrain(roll_deg, -90, 90);

  iteration += 1;

  if (millis() - prev_servo_update >= 20 && millis() > 2000) {

    // Control system

    float pitch_error = 0 - pitch_deg;
    pitch_output += 0.15 * pitch_error;

    float roll_error = 0 - roll_deg;
    roll_output += 0.3 * roll_error;

    pitch_output = constrain(pitch_output, -90, 90);
    roll_output = constrain(roll_output, -90, 90);

    // Serial.print(pitch_output);
    // Serial.print(" , ");
    Serial.print("Pitch: ");
    Serial.print(pitch_deg);
    Serial.print(" - ");
    Serial.print(pitch_output);
    Serial.print(" Error: ");
    Serial.print(pitch_error);
    Serial.print(" , ");
    // Serial.print(0);
    // Serial.print(" , ");
    Serial.print("Roll: ");
    Serial.print(roll_deg);
    Serial.print(" - ");
    Serial.print(roll_output);
    Serial.print(" Error: ");
    Serial.println(roll_error);

    // Output

    float servo_pitch_write = pitch_output + 90;
    float servo_roll_write = roll_output + 90;

    servo_pitch_write = constrain(servo_pitch_write, 0, 180);
    servo_roll_write = constrain(servo_roll_write, 0, 180);

    servo_pitch.write(servo_pitch_write);
    servo_roll.write(servo_roll_write);

    prev_servo_update = millis();

  }
}