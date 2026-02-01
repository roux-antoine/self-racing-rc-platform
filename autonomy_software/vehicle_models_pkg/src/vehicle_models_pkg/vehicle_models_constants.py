# Steering parameters
PHYSICAL_MAX_STEERING_ANGLE_RAD = (
    0.52  # Max steering angle achievable by wheels (average left and right)
)
EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1 = (
    0.30  # _Effective_ steering angle achievable by wheels (average left and right)
)
STEERING_IDLE_PWM = 90
STEERING_MIN_PWM = 64  # 62 + 2
STEERING_MAX_PWM = 116  # 118 - 2
STEERING_DIRECTION_FACTOR = (
    -1
)  # Used because a positive change in steering command leads to a negative change in steering angle

WHEELBASE = 0.406
PWM_DIFF_AT_MAX_STEER_ANGLE = (
    STEERING_MAX_PWM - STEERING_IDLE_PWM
)  # Assuming symmetric steering

# Measured by eye from 2025-08-24 testing session
MIN_STEERING_FBK = 215
MAX_STEERING_FBK = 450
STEERING_FBK_IDLE = 335
