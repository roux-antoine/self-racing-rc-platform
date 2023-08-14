#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <self_racing_car_msgs/ArduinoLogging.h>
#include <self_racing_car_msgs/VehicleCommand.h>

// ------ CONSTANTS ------

const int CHANNEL_2_PIN = 18; // steering
const int CHANNEL_3_PIN = 19; // throttle
const int CHANNEL_5_PIN = 3;

const int STEERING_OUTPUT_PIN = 7;
const int THROTTLE_OUTPUT_PIN = 8;

const int LED_PIN = 13;

const float PHYSICAL_MAX_ANGLE_STEERING = 0.4;  // rad (measuring the max wheel angle)
const float EFFECTIVE_MAX_ANGLE_STEERING = 0.3; // rad (assuming 2.5m turning circle and 40.6cm wheelbase)
const float MAX_THROTTLE_UNIT = 100;            // arbitrary unit of throttle cmd sent by the Pi

const float STEERING_IDLE = 98;
const float STEERING_MAX = 123;
const float STEERING_MIN = 68;

const float THROTTLE_IDLE = 90;
const float THROTTLE_MAX_MANUAL = 102; // without load, the wheels start turning around 96
const float THROTTLE_MIN_MANUAL = 70;
const float THROTTLE_MAX_AUTONOMOUS = 102;
const float THROTTLE_MIN_AUTONOMOUS = 70;

const unsigned long PULSE_WIDTH_THRESHOLD = 2000;

const unsigned int DISENGAGEMENT_HYSTERESIS_THRESHOLD = 3;

const unsigned long CHANNEL_2_IDLE_MIN = 1470; // = steering
const unsigned long CHANNEL_2_IDLE_MAX = 1530;
const unsigned long CHANNEL_2_MAX = 1880;
const unsigned long CHANNEL_2_MIN = 1180;
const unsigned long CHANNEL_2_OVERRIDE_MIN = 1353;
const unsigned long CHANNEL_2_OVERRIDE_MAX = 1647;

const unsigned long CHANNEL_3_IDLE_MIN = 1455; // throttle
const unsigned long CHANNEL_3_IDLE_MAX = 1515;
const unsigned long CHANNEL_3_MAX = 1716;
const unsigned long CHANNEL_3_MIN = 1092;
const unsigned long CHANNEL_3_OVERRIDE_MIN = 1388;
const unsigned long CHANNEL_3_OVERRIDE_MAX = 1582;

const unsigned long CHANNEL_5_THRESHOLD = 1700;

const bool ROS_MODE = true;

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

// channel 2 = steering
volatile unsigned long tmp_pulse_width_steering = 0;
volatile unsigned long pulse_width_steering = 0;
volatile unsigned long prev_time_steering = 0;
// channel 3 = throttle
volatile unsigned long tmp_pulse_width_throttle = 0;
volatile unsigned long pulse_width_throttle = 0;
volatile unsigned long prev_time_throttle = 0;
// channel 5 = engagement switch
volatile unsigned long tmp_pulse_width_engaged = 0;
volatile bool engaged_mode = false;
volatile unsigned long prev_time_engaged = 0;

volatile unsigned int engagement_hysteresis_counter = 0;
volatile unsigned int disengagement_hysteresis_counter = 0;

// override flags
bool override_steering = false;
bool override_throttle = false;

// Servo objects
Servo throttle_servo;
Servo steering_servo;

// ROS stuff
ros::NodeHandle nh;

void vehicle_command_callback(const self_racing_car_msgs::VehicleCommand &msg) {
  digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN)); // blink the led

  // flipping the sign because a positive servo value turns towards the negative direction
  flipped_steering_value_rad = -msg.steering_value_rad;

  // Convert the steering angle value
  if (flipped_steering_value_rad >= EFFECTIVE_MAX_ANGLE_STEERING) {
    steering_angle_pi = STEERING_MAX;
  } else if (flipped_steering_value_rad <= -EFFECTIVE_MAX_ANGLE_STEERING) {
    steering_angle_pi = STEERING_MIN;
  } else if (flipped_steering_value_rad >= 0) {
    steering_angle_pi = STEERING_IDLE + flipped_steering_value_rad * ((STEERING_MAX - STEERING_IDLE) / EFFECTIVE_MAX_ANGLE_STEERING);
  } else {
    steering_angle_pi = STEERING_IDLE + flipped_steering_value_rad * ((STEERING_IDLE - STEERING_MIN) / EFFECTIVE_MAX_ANGLE_STEERING);
  }

  // Converting the throttle value
  if (msg.throttle_value >= MAX_THROTTLE_UNIT) {
    throttle_angle_pi = THROTTLE_MAX_AUTONOMOUS;
  } else if (msg.throttle_value <= -MAX_THROTTLE_UNIT) {
    throttle_angle_pi = THROTTLE_MIN_AUTONOMOUS;
  } else if (msg.throttle_value >= 0) {
    throttle_angle_pi = THROTTLE_IDLE + msg.throttle_value * ((THROTTLE_MAX_AUTONOMOUS - THROTTLE_IDLE) / MAX_THROTTLE_UNIT);
  } else {
    throttle_angle_pi = THROTTLE_IDLE + msg.throttle_value * ((THROTTLE_IDLE - THROTTLE_MIN_AUTONOMOUS) / MAX_THROTTLE_UNIT);
  }
}
ros::Subscriber<self_racing_car_msgs::VehicleCommand> vehicle_command_sub("vehicle_command", &vehicle_command_callback);
self_racing_car_msgs::ArduinoLogging arduino_logging_msg;
ros::Publisher arduino_logging_pub("arduino_logging", &arduino_logging_msg);

// ------ FUNCTIONS ------

void steering_callback() {
  tmp_pulse_width_steering = micros() - prev_time_steering;
  prev_time_steering = micros();

  if (tmp_pulse_width_steering < PULSE_WIDTH_THRESHOLD) {
    pulse_width_steering = tmp_pulse_width_steering;
  }
}

void throttle_callback() {
  tmp_pulse_width_throttle = micros() - prev_time_throttle;
  prev_time_throttle = micros();

  if (tmp_pulse_width_throttle < PULSE_WIDTH_THRESHOLD) {
    pulse_width_throttle = tmp_pulse_width_throttle;
  }
}

void engaged_mode_callback() {
  tmp_pulse_width_engaged = micros() - prev_time_engaged;
  prev_time_engaged = micros();

  // NOTE: logic without hysteresis
  if (tmp_pulse_width_engaged < PULSE_WIDTH_THRESHOLD) {
    if (tmp_pulse_width_engaged > CHANNEL_5_THRESHOLD) {
      engaged_mode = true;
    } else {
      engaged_mode = false;
    }
  }
  // end NOTE

  // logic with hysteresis
  /*
  if (tmp_pulse_width_engaged < PULSE_WIDTH_THRESHOLD) {
    if (tmp_pulse_width_engaged > CHANNEL_5_THRESHOLD) {
      engagement_hysteresis_counter += 1;
      disengagement_hysteresis_counter = 0;
    } else {
      engagement_hysteresis_counter = 0;
      disengagement_hysteresis_counter += 1;
    }

    if (disengagement_hysteresis_counter >= DISENGAGEMENT_HYSTERESIS_THRESHOLD) {
      engaged_mode = false;
    } else if (engagement_hysteresis_counter >= DISENGAGEMENT_HYSTERESIS_THRESHOLD) {
      engaged_mode = true;
    }
  }
  */
}

void display_value(const char description[], float value) {
  if (ROS_MODE) {
    nh.loginfo(description);
    char result[8];
    dtostrf(value, 6, 2, result);
    nh.loginfo(result);
  } else {
    Serial.print(description);
    Serial.println(value);
  }
}

// ------ SETUP ------

void setup() {

  // Setting up connection etc
  if (ROS_MODE) {
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(vehicle_command_sub);
    nh.advertise(arduino_logging_pub);
    nh.loginfo("In the setup");
  } else {
    Serial.begin(57600);
    Serial.println("In the setup");
  }

  // Setting up pins
  // TODO for some reason, digitalPinToInterrupt is not found when building with catkin
  // interrupt_number_channel_2 = digitalPinToInterrupt(CHANNEL_2_PIN);
  // interrupt_number_channel_3 = digitalPinToInterrupt(CHANNEL_3_PIN);
  // interrupt_number_channel_5 = digitalPinToInterrupt(CHANNEL_5_PIN);
  int interrupt_number_channel_2 = 5;
  int interrupt_number_channel_3 = 4;
  int interrupt_number_channel_5 = 1;
  attachInterrupt(interrupt_number_channel_2, steering_callback, CHANGE);
  attachInterrupt(interrupt_number_channel_3, throttle_callback, CHANGE);
  attachInterrupt(interrupt_number_channel_5, engaged_mode_callback, CHANGE);

  pinMode(LED_PIN, OUTPUT);
  steering_servo.attach(STEERING_OUTPUT_PIN);
  throttle_servo.attach(THROTTLE_OUTPUT_PIN);

  // Doing the ESC calibration (unnecessary with Infraction)
  steering_servo.write(STEERING_IDLE);
  throttle_servo.write(THROTTLE_IDLE);
  delay(5000);

  if (ROS_MODE) {
    nh.loginfo("End setup");
  } else {
    Serial.println("End setup");
  }
}

// ------ LOOP ------

void loop() {

  // Compute steering from the Rx
  if (pulse_width_steering < CHANNEL_2_IDLE_MAX && pulse_width_steering > CHANNEL_2_IDLE_MIN) {
    steering_angle_rx = STEERING_IDLE;
  } else if (pulse_width_steering >= CHANNEL_2_IDLE_MAX) {
    steering_angle_rx = int(STEERING_IDLE + (pulse_width_steering - CHANNEL_2_IDLE_MAX) * ((STEERING_MAX - STEERING_IDLE) / (CHANNEL_2_MAX - CHANNEL_2_IDLE_MAX)));
  } else if (pulse_width_steering <= CHANNEL_2_IDLE_MIN) {
    steering_angle_rx = int(STEERING_IDLE - (CHANNEL_2_IDLE_MIN - pulse_width_steering) * ((STEERING_IDLE - STEERING_MIN) / (CHANNEL_2_IDLE_MIN - CHANNEL_2_MIN)));
  } else {
  }

  // Compute throttle from the Rx
  if (pulse_width_throttle < CHANNEL_3_IDLE_MAX && pulse_width_throttle > CHANNEL_3_IDLE_MIN) {
    throttle_angle_rx = THROTTLE_IDLE;
  } else if (pulse_width_throttle >= CHANNEL_3_IDLE_MAX) {
    throttle_angle_rx = int(THROTTLE_IDLE + (pulse_width_throttle - CHANNEL_3_IDLE_MAX) * ((THROTTLE_MAX_MANUAL - THROTTLE_IDLE) / (CHANNEL_3_MAX - CHANNEL_3_IDLE_MAX)));
  } else if (pulse_width_throttle <= CHANNEL_3_IDLE_MIN) {
    throttle_angle_rx = int(THROTTLE_IDLE - (CHANNEL_3_IDLE_MIN - pulse_width_throttle) * ((THROTTLE_IDLE - THROTTLE_MIN_MANUAL) / (CHANNEL_3_IDLE_MIN - CHANNEL_3_MIN)));
  } else {
  }

  // checking if we are in override mode
  if (pulse_width_steering < CHANNEL_2_OVERRIDE_MIN || pulse_width_steering > CHANNEL_2_OVERRIDE_MAX) {
    override_steering = true;
  } else {
    override_steering = false;
  }
  if (pulse_width_throttle < CHANNEL_3_OVERRIDE_MIN || pulse_width_throttle > CHANNEL_3_OVERRIDE_MAX) {
    override_throttle = true;
  } else {
    override_throttle = false;
  }

  // Deciding which command to send // TODO: it'd be nice to have some smoothing when switching between the 2 modes
  if (!engaged_mode) {
    steering_angle_final = steering_angle_rx;
    throttle_angle_final = throttle_angle_rx;
  } else if (override_steering) {
    steering_angle_final = steering_angle_rx;
    throttle_angle_final = throttle_angle_pi;
  } else if (override_throttle) {
    steering_angle_final = steering_angle_pi;
    throttle_angle_final = throttle_angle_rx;
  } else {
    steering_angle_final = steering_angle_pi;
    throttle_angle_final = throttle_angle_pi;
  }

  // Making sure the commands are in bounds
  if (steering_angle_final > STEERING_MAX) {
    steering_angle_final = STEERING_MAX;
  } else if (steering_angle_final < STEERING_MIN) {
    steering_angle_final = STEERING_MIN;
  }

  // Making sure the commands are in bounds
  if (steering_angle_final > STEERING_MAX) {
    steering_angle_final = STEERING_MAX;
  } else if (steering_angle_final < STEERING_MIN) {
    steering_angle_final = STEERING_MIN;
  }

  // Sending the commands
  steering_servo.write(steering_angle_final);
  throttle_servo.write(throttle_angle_final);

  // Publishing the logging info
  if (ROS_MODE) {
    arduino_logging_msg.steering_angle_rx = steering_angle_rx;
    arduino_logging_msg.throttle_angle_rx = throttle_angle_rx;
    arduino_logging_msg.steering_angle_pi = steering_angle_pi;
    arduino_logging_msg.throttle_angle_pi = throttle_angle_pi;
    arduino_logging_msg.steering_angle_final = steering_angle_final;
    arduino_logging_msg.throttle_angle_final = throttle_angle_final;
    arduino_logging_msg.tmp_pulse_width_1 = tmp_pulse_width_steering;
    arduino_logging_msg.tmp_pulse_width_2 = tmp_pulse_width_throttle;
    arduino_logging_msg.tmp_pulse_width_3 = tmp_pulse_width_engaged;
    arduino_logging_msg.engaged_mode = engaged_mode;
    arduino_logging_msg.override_steering = override_steering;
    arduino_logging_msg.override_throttle = override_throttle;

    arduino_logging_pub.publish(&arduino_logging_msg);
  }

  if (ROS_MODE) {
    nh.spinOnce();
  }

  // display_value("steering_angle_final ", steering_angle_final);
  // display_value("throttle_angle_final ", throttle_angle_final);
  // display_value("engaged_mode", engaged_mode);
  // display_value("override throttle ", override_throttle);
  // display_value("override steering ", override_steering);

  delay(100);
}
