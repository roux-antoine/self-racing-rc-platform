# Steering param identification

Set of scripts to estimate the parameters of the steering of the RC car

permanent_regime_circle_fitter.py
- finds parts of the rosbag where the GPS fix is good
- fits circles to the trajectory of the car
- for 'good' circles, extracts vehicle dynamics information
- saves to a CSV

permanent_regime_steering_params_identification.py
- parses the CSV to help build a steering model

How-to:
- run permanent_regime_circle_fitter.py on a set of bags, tweak the parameters to find only the circular motions that you think are 'clean' enough. For instance:
    - length of the circular motion
    - threshold on the computed radius
    - threshold on the mean vehicle speed during the circular motion
    - threshold on the variation of the vehicle speed during the circular motion
    - threshold on the variation of the steering command during the circular motion
- save the circular motions to csv
- use permanent_regime_steering_params_identification.py to fit different models based on the recorded data
- create new vehicle models classes in vehicle_models_pkg and try them out in sim and in real
