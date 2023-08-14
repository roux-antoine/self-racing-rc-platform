#include <Servo.h>

// CHANNEL 1 = STEERING
volatile int pwm_value_1_change = 0; // valeur intermediaire
volatile int pwm_value_1 = 0;
volatile int prev_time_1 = 0;

// CHANNEL 2 = THROTTLE
volatile int pwm_value_2_change = 0; // valeur intermediaire
volatile int pwm_value_2 = 0;
volatile int prev_time_2 = 0;

// CHANNEL 3 = SWITCH MANUAL / AUTONOMOUS
volatile int pwm_value_3_change = 0; // valeur intermediaire
volatile int pwm_value_3 = 0;
volatile int prev_time_3 = 0;

// Interrupt pins
// PIN MEGA   Interrupt pim
//  2             0
//  3             1
//  21            2
//  20            3
//  19            4
int int_id_chan_1 = 1;
int int_id_chan_2 = 0;
int int_id_chan_3 = 2;

// INITIAL VALUES SERVO.WRITE()
int throttle_angle = 90;
int steering_angle = 90;

// servo object
Servo throttle_servo;
Servo steering_servo;

void setup() {

  Serial.begin(57600);
  // attachInterrupt(int_id_chan_2, throttle_callback, CHANGE);
  attachInterrupt(int_id_chan_1, steering_callback, CHANGE);
  // attachInterrupt(int_id_chan_3, change_3, CHANGE);

  throttle_servo.attach(6);
  throttle_servo.write(90);

  steering_servo.attach(7);
  steering_servo.write(90);

  delay(5000);
  Serial.println("End setup");
}

void loop() {

  // Throttle
  // throttle_angle = 90 + int((pwm_value_2 - 1500) / 33);
  // throttle_servo.write(throttle_angle);
  // Serial.println(throttle_angle);

  // Steering
  // steering_servo.write(97);
  if (pwm_value_1 < 1520 && pwm_value_1 > 1500) {
    steering_angle = 83.5;
  } else {
    steering_angle = int(83.5 + (pwm_value_1 - 1500) / 30.3);
  }

  steering_servo.write(steering_angle);
  Serial.println(steering_angle);

  delay(1);
}

void throttle_callback() {
  pwm_value_2_change = micros() - prev_time_2;
  prev_time_2 = micros();

  if (pwm_value_2_change < 3500) {
    pwm_value_2 = pwm_value_2_change;
  }
}

void steering_callback() {
  pwm_value_1_change = micros() - prev_time_1;
  prev_time_1 = micros();

  if (pwm_value_1_change < 3500) {
    pwm_value_1 = pwm_value_1_change;
    // Serial.println(pwm_value_1);
  }
}
