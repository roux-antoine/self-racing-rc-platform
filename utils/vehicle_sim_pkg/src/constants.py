import numpy as np

# Vehicle parameters
NEUTRAL_STEERING_PWM_CMD = 90 
NEUTRAL_THROTTLE_PWM_CMD = 90
WHEELBASE_M = 0.406
MAX_STEERING_ANGLE_RAD = np.pi / 4

# Simulation settings
SIM_FREQUENCY_HZ = 10.0
SIM_TIME_STEP_SECS = 0.1

# Vehicle sim parameters
DESIRED_ACC_GAIN_P = 0.5

# Steering parameters
EFFECTIVE_MAX_STEERING_ANGLE_RAD = 0.3 # Max steering angle achievable by wheels (average left and right)
STEERING_PWM_IDLE = 90
STEERING_PWM_MIN = 65

# Throttle parameters
THROTTLE_PWM_TO_VELOCITY_MPS_SLOPE = 0.54
THROTTLE_PWM_TO_VELOCITY_MPS_OFFSET = -51.1