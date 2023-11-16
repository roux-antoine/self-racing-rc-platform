#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <self_racing_car_msgs/ArduinoLogging.h>
#include <std_msgs/Float32.h>

// ------ CONSTANTS ------

const int CHANNEL_2_PIN = 18; // steering
const int CHANNEL_3_PIN = 19; // throttle
const int CHANNEL_5_PIN = 3;

const int STEERING_OUTPUT_PIN = 7;
const int THROTTLE_OUTPUT_PIN = 8;

const int LED_PIN = 13;

const float STEERING_IDLE_PWM = 98;
const float STEERING_MAX_PWM = 123;
const float STEERING_MIN_PWM = 68;

const float THROTTLE_IDLE_PWM = 90;
const float THROTTLE_MAX_MANUAL_PWM = 102; // without load, the wheels start turning around 96
const float THROTTLE_MIN_MANUAL_PWM = 70;
const float THROTTLE_MAX_AUTONOMOUS_PWM = 115;
const float THROTTLE_MIN_AUTONOMOUS_PWM = 70;

const unsigned long PULSE_WIDTH_THRESHOLD = 2000;

const unsigned int DISENGAGEMENT_HYSTERESIS_THRESHOLD = 3;
const unsigned int STEERING_OVERRIDE_HYSTERESIS_THRESHOLD = 3;
const unsigned int THROTTLE_OVERRIDE_HYSTERESIS_THRESHOLD = 3;

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
float steering_cmd_rx = STEERING_IDLE_PWM;
float throttle_cmd_rx = THROTTLE_IDLE_PWM;
volatile float steering_cmd_autonomous = STEERING_IDLE_PWM;
volatile float throttle_cmd_autonomous = THROTTLE_IDLE_PWM;
float steering_cmd_final = STEERING_IDLE_PWM;
float throttle_cmd_final = THROTTLE_IDLE_PWM;
int throttle_max_pwm = THROTTLE_MAX_MANUAL_PWM;
int throttle_min_pwm = THROTTLE_MIN_MANUAL_PWM;

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

// override stuff
volatile unsigned int engagement_hysteresis_counter = 0;
volatile unsigned int disengagement_hysteresis_counter = 0;
unsigned int steering_override_hysteresis_counter = 0;
unsigned int throttle_override_hysteresis_counter = 0;
bool override_steering = false;
bool override_throttle = false;

// Servo objects
Servo throttle_servo;
Servo steering_servo;

// ROS stuff
ros::NodeHandle nh;

void steering_pwm_cmd_callback(const std_msgs::Float32 &msg) {

  if (msg.data > STEERING_MAX_PWM) {
    steering_cmd_autonomous = STEERING_MAX_PWM;
  } else if (msg.data < STEERING_MIN_PWM) {
    steering_cmd_autonomous = STEERING_MIN_PWM;
  } else {
    steering_cmd_autonomous = msg.data;
  }
}
void throttle_pwm_cmd_callback(const std_msgs::Float32 &msg) {

  if (msg.data > THROTTLE_MAX_AUTONOMOUS_PWM) {
    throttle_cmd_autonomous = THROTTLE_MAX_AUTONOMOUS_PWM;
  } else if (msg.data < THROTTLE_MIN_AUTONOMOUS_PWM) {
    throttle_cmd_autonomous = THROTTLE_MIN_AUTONOMOUS_PWM;
  } else {
    throttle_cmd_autonomous = msg.data;
  }
}
ros::Subscriber<std_msgs::Float32> steering_pwm_cmd_sub("steering_pwm_cmd", steering_pwm_cmd_callback);
ros::Subscriber<std_msgs::Float32> throttle_pwm_cmd_sub("throttle_pwm_cmd", throttle_pwm_cmd_callback);
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
  // if (tmp_pulse_width_engaged < PULSE_WIDTH_THRESHOLD) {
  //   if (tmp_pulse_width_engaged > CHANNEL_5_THRESHOLD) {
  //     engaged_mode = true;
  //   } else {
  //     engaged_mode = false;
  //   }
  // }
  // end NOTE

  // logic with hysteresis
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
    nh.subscribe(steering_pwm_cmd_sub);
    nh.subscribe(throttle_pwm_cmd_sub);
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
  steering_servo.write(STEERING_IDLE_PWM);
  throttle_servo.write(THROTTLE_IDLE_PWM);
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
    steering_cmd_rx = STEERING_IDLE_PWM;
  } else if (pulse_width_steering >= CHANNEL_2_IDLE_MAX) {
    steering_cmd_rx = int(STEERING_IDLE_PWM + (pulse_width_steering - CHANNEL_2_IDLE_MAX) * ((STEERING_MAX_PWM - STEERING_IDLE_PWM) / (CHANNEL_2_MAX - CHANNEL_2_IDLE_MAX)));
  } else if (pulse_width_steering <= CHANNEL_2_IDLE_MIN) {
    steering_cmd_rx = int(STEERING_IDLE_PWM - (CHANNEL_2_IDLE_MIN - pulse_width_steering) * ((STEERING_IDLE_PWM - STEERING_MIN_PWM) / (CHANNEL_2_IDLE_MIN - CHANNEL_2_MIN)));
  } else {
  }

  // Compute throttle from the Rx
  if (pulse_width_throttle < CHANNEL_3_IDLE_MAX && pulse_width_throttle > CHANNEL_3_IDLE_MIN) {
    throttle_cmd_rx = THROTTLE_IDLE_PWM;
  } else if (pulse_width_throttle >= CHANNEL_3_IDLE_MAX) {
    throttle_cmd_rx = int(THROTTLE_IDLE_PWM + (pulse_width_throttle - CHANNEL_3_IDLE_MAX) * ((THROTTLE_MAX_MANUAL_PWM - THROTTLE_IDLE_PWM) / (CHANNEL_3_MAX - CHANNEL_3_IDLE_MAX)));
  } else if (pulse_width_throttle <= CHANNEL_3_IDLE_MIN) {
    throttle_cmd_rx = int(THROTTLE_IDLE_PWM - (CHANNEL_3_IDLE_MIN - pulse_width_throttle) * ((THROTTLE_IDLE_PWM - THROTTLE_MIN_MANUAL_PWM) / (CHANNEL_3_IDLE_MIN - CHANNEL_3_MIN)));
  } else {
  }

  // checking if we are in override mode
  if (pulse_width_steering < CHANNEL_2_OVERRIDE_MIN || pulse_width_steering > CHANNEL_2_OVERRIDE_MAX) {
    steering_override_hysteresis_counter += 1;
    if (steering_override_hysteresis_counter >= STEERING_OVERRIDE_HYSTERESIS_THRESHOLD) {
      override_steering = true;
    }
  } else {
    steering_override_hysteresis_counter = 0;
    override_steering = false;
  }
  if (pulse_width_throttle < CHANNEL_3_OVERRIDE_MIN || pulse_width_throttle > CHANNEL_3_OVERRIDE_MAX) {
    throttle_override_hysteresis_counter += 1;
    if (throttle_override_hysteresis_counter >= THROTTLE_OVERRIDE_HYSTERESIS_THRESHOLD) {
      override_throttle = true;
    }
  } else {
    throttle_override_hysteresis_counter = 0;
    override_throttle = false;
  }

  // Deciding which command to send
  // IDEA: it'd be nice to have some smoothing when switching between the 2 modes
  if (!engaged_mode) {
    steering_cmd_final = steering_cmd_rx;
    throttle_cmd_final = throttle_cmd_rx;
    throttle_max_pwm = THROTTLE_MAX_MANUAL_PWM;
    throttle_min_pwm = THROTTLE_MIN_MANUAL_PWM;
  } else if (override_steering) {
    steering_cmd_final = steering_cmd_rx;
    throttle_cmd_final = throttle_cmd_autonomous;
    throttle_max_pwm = THROTTLE_MAX_AUTONOMOUS_PWM;
    throttle_min_pwm = THROTTLE_MIN_AUTONOMOUS_PWM;
  } else if (override_throttle) {
    steering_cmd_final = steering_cmd_autonomous;
    throttle_cmd_final = throttle_cmd_rx;
    throttle_max_pwm = THROTTLE_MAX_MANUAL_PWM;
    throttle_min_pwm = THROTTLE_MIN_MANUAL_PWM;
  } else {
    steering_cmd_final = steering_cmd_autonomous;
    throttle_cmd_final = throttle_cmd_autonomous;
    throttle_max_pwm = THROTTLE_MAX_AUTONOMOUS_PWM;
    throttle_min_pwm = THROTTLE_MIN_AUTONOMOUS_PWM;
  }

  // Making sure the commands are in bounds
  if (steering_cmd_final > STEERING_MAX_PWM) {
    steering_cmd_final = STEERING_MAX_PWM;
  } else if (steering_cmd_final < STEERING_MIN_PWM) {
    steering_cmd_final = STEERING_MIN_PWM;
  }

  // Making sure the commands are in bounds
  if (throttle_cmd_final > throttle_max_pwm) {
    throttle_cmd_final = throttle_max_pwm;
  } else if (throttle_cmd_final < throttle_min_pwm) {
    throttle_cmd_final = throttle_min_pwm;
  }

  // Sending the commands
  steering_servo.write(steering_cmd_final);
  throttle_servo.write(throttle_cmd_final);

  // Publishing the logging info
  if (ROS_MODE) {
    arduino_logging_msg.steering_cmd_rx = steering_cmd_rx;
    arduino_logging_msg.throttle_cmd_rx = throttle_cmd_rx;
    arduino_logging_msg.steering_cmd_autonomous = steering_cmd_autonomous;
    arduino_logging_msg.throttle_cmd_autonomous = throttle_cmd_autonomous;
    arduino_logging_msg.steering_cmd_final = steering_cmd_final;
    arduino_logging_msg.throttle_cmd_final = throttle_cmd_final;
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

  // display_value("steering_cmd_final ", steering_cmd_final);
  // display_value("throttle_cmd_final ", throttle_cmd_final);
  // display_value("engaged_mode", engaged_mode);
  // display_value("override throttle ", override_throttle);
  // display_value("override steering ", override_steering);

  delay(5);
}
