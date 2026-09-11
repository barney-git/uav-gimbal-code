// Gimbal feedback system

// Live -> 5V
// Ground -> GND
// Signal (SDA) -> A4
// Clock (SDL) -> A5 

// Accelerometer range set to: +-4G
// Gyro range set to: 2000 degrees/s
// Accelerometer data rate set to: 104 Hz
// Gyro data rate set to: 104 Hz

// IMU to Servos

#include <Servo.h>
Servo servo_pitch;
Servo servo_roll;

unsigned long prev_time;
float dt;
unsigned long prev_servo_update = 0;

int iteration;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  servo_pitch.attach(10); // Roll
  servo_roll.attach(11); // Pitch

}

void loop() {

  unsigned long now = micros();
  dt = (now - prev_time) / float(1000000);
  prev_time = now;

  iteration += 1;

  if (millis() - prev_servo_update >= 20) {

    float pitch_angle = 0;
    float roll_angle = 0;

    float servo_pitch_write = pitch_angle + 90;
    float servo_roll_write = roll_angle + 90;

    servo_pitch_write = constrain(servo_pitch_write, 0, 180);
    servo_roll_write = constrain(servo_roll_write, 0, 180);

    servo_pitch.write(servo_pitch_write);
    servo_roll.write(servo_roll_write);
    prev_servo_update = millis();

  }
}