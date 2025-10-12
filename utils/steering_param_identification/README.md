# Steering param identification

TODO

Script 1: permanent_regime_circle_fitter.py
- finds parts of the rosbag where the GPS fix is good
- fits circles to the trajectory of the car
- for 'good' circles, extracts vehicle dynamics information
- saves to a CSV

Script 2: permanent_regime_steering_params_identification.py
- parses the CSV to help build a steering model
