#include <Arduino.h>
#include <Servo.h>
#include <ros.h>

// ------ CONSTANTS ------

const int CHANNEL_2_PIN = 18; // steering
const int CHANNEL_3_PIN = 19; // throttle

const int STEERING_OUTPUT_PIN = 7;
const int THROTTLE_OUTPUT_PIN = 8;

const int LED_PIN = 13;

const float STEERING_IDLE = 98;
const float STEERING_MAX = 123;
const float STEERING_MIN = 68;

const float THROTTLE_IDLE = 90;
const float THROTTLE_MAX_MANUAL = 102;
const float THROTTLE_MIN_MANUAL = 70;

const unsigned long PULSE_WIDTH_THRESHOLD = 2000;

const unsigned long CHANNEL_2_IDLE_MIN = 1470; // = steering
const unsigned long CHANNEL_2_IDLE_MAX = 1530;
const unsigned long CHANNEL_2_MAX = 1880;
const unsigned long CHANNEL_2_MIN = 1180;

const unsigned long CHANNEL_3_IDLE_MIN = 1455; // throttle
const unsigned long CHANNEL_3_IDLE_MAX = 1515;
const unsigned long CHANNEL_3_MAX = 1716;
const unsigned long CHANNEL_3_MIN = 1092;

// ------ VARIABLES ------

// commands that will be sent by PWM
float steering_angle_rx = STEERING_IDLE;
float throttle_angle_rx = THROTTLE_IDLE;
volatile float steering_angle_pi = STEERING_IDLE;
volatile float throttle_angle_pi = THROTTLE_IDLE;
float steering_angle_final = STEERING_IDLE;
float throttle_angle_final = THROTTLE_IDLE;

// temporary storage values
volatile float flipped_steering_value_rad = 0;

// channel 1 = steering
volatile unsigned long tmp_pulse_width_1 = 0;
volatile unsigned long pulse_width_1 = 0;
volatile unsigned long prev_time_1 = 0;
// channel 2 = throttle
volatile unsigned long tmp_pulse_width_2 = 0;
volatile unsigned long pulse_width_2 = 0;
volatile unsigned long prev_time_2 = 0;

// Servo objects
Servo throttle_servo;
Servo steering_servo;

// ------ FUNCTIONS ------

void steering_callback() {
  tmp_pulse_width_1 = micros() - prev_time_1;
  prev_time_1 = micros();

  if (tmp_pulse_width_1 < PULSE_WIDTH_THRESHOLD) {
    pulse_width_1 = tmp_pulse_width_1;
  }
}

void throttle_callback() {
  tmp_pulse_width_2 = micros() - prev_time_2;
  prev_time_2 = micros();

  if (tmp_pulse_width_2 < PULSE_WIDTH_THRESHOLD) {
    pulse_width_2 = tmp_pulse_width_2;
  }
}

void display_value(const char description[], float value) {
  Serial.print(description);
  Serial.println(value);
}

// ------ SETUP ------

void setup() {

  // Setting up connection etc

  Serial.begin(57600);
  Serial.println("In the setup");

  // Setting up pins
  attachInterrupt(digitalPinToInterrupt(CHANNEL_2_PIN), steering_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_3_PIN), throttle_callback, CHANGE);
  pinMode(LED_PIN, OUTPUT);
  steering_servo.attach(STEERING_OUTPUT_PIN);
  throttle_servo.attach(THROTTLE_OUTPUT_PIN);

  // Doing the ESC calibration
  steering_servo.write(STEERING_IDLE);
  throttle_servo.write(THROTTLE_IDLE);
  delay(5000);

  Serial.println("End setup");
}

// ------ LOOP ------

void loop() {

  // Compute steering from the Rx
  if (pulse_width_1 < CHANNEL_2_IDLE_MAX && pulse_width_1 > CHANNEL_2_IDLE_MIN) {
    steering_angle_rx = STEERING_IDLE;
  } else if (pulse_width_1 >= CHANNEL_2_IDLE_MAX) {
    steering_angle_rx = int(STEERING_IDLE + (pulse_width_1 - CHANNEL_2_IDLE_MAX) * ((STEERING_MAX - STEERING_IDLE) / (CHANNEL_2_MAX - CHANNEL_2_IDLE_MAX)));
  } else if (pulse_width_1 <= CHANNEL_2_IDLE_MIN) {
    steering_angle_rx = int(STEERING_IDLE - (CHANNEL_2_IDLE_MIN - pulse_width_1) * ((STEERING_IDLE - STEERING_MIN) / (CHANNEL_2_IDLE_MIN - CHANNEL_2_MIN)));
  } else {
  }

  // Compute throttle from the Rx
  if (pulse_width_2 < CHANNEL_3_IDLE_MAX && pulse_width_2 > CHANNEL_3_IDLE_MIN) {
    throttle_angle_rx = THROTTLE_IDLE;
  } else if (pulse_width_2 >= CHANNEL_3_IDLE_MAX) {
    throttle_angle_rx = int(THROTTLE_IDLE + (pulse_width_2 - CHANNEL_3_IDLE_MAX) * ((THROTTLE_MAX_MANUAL - THROTTLE_IDLE) / (CHANNEL_3_MAX - CHANNEL_3_IDLE_MAX)));
  } else if (pulse_width_2 <= CHANNEL_3_IDLE_MIN) {
    throttle_angle_rx = int(THROTTLE_IDLE - (CHANNEL_3_IDLE_MIN - pulse_width_2) * ((THROTTLE_IDLE - THROTTLE_MIN_MANUAL) / (CHANNEL_3_IDLE_MIN - CHANNEL_3_MIN)));
  } else {
  }

  steering_angle_final = steering_angle_rx;
  throttle_angle_final = throttle_angle_rx;

  // Making sure the commands are in bounds
  if (steering_angle_final > STEERING_MAX) {
    steering_angle_final = STEERING_MAX;
  } else if (steering_angle_final < STEERING_MIN) {
    steering_angle_final = STEERING_MIN;
  }

  // Sending the commands
  steering_servo.write(steering_angle_final);
  throttle_servo.write(throttle_angle_final);

  Serial.print("steering ");
  Serial.println(steering_angle_final);
  Serial.print("throttle ");
  Serial.println(throttle_angle_final);

  delay(100);
}
