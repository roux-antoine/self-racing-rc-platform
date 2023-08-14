# Setup

The package has been setup using the instructions in http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

Some notes:
* since we use Noetic, add `add_compile_options(-std=c++11)` in all CMakeLists.txt

# How to build and upload with catkin

## How to build

In `firmware/CMakeLists.txt`:
* Make sure that the board is correct (uno vs mega)
* Make sure that the port where the board is connected is correct

Then run: `catkin build arduino_pkg`

## How to upload

`catkin build --no-deps  arduino_pkg --make-args arduino_pkg_firmware_aim-upload`

# How to build and upload with the Arduino IDE

* Build the package with the custom messsages: `catkin build self_racing_car_msgs`
* Source newly built files: `source devel/setup.bash`
* Deploy the built libraries to Arduino IDE's library folder: `rosrun rosserial_arduino make_libraries.py ~/Downloads/arduino-1.8.18/libraries self_racing_car_msgs`
* Make sure that the libraries have been deployed by verifying that `~/Downloads/arduino-1.8.18/libraries/ros_lib/self_racing_car_msgs` exists
* Copy-paste the script in the Arduino IDE
* Make sure that the "bridge" below is not running
* Build and upload from the Arduino IDE

# Running the code

* Start roscore if needed
* Initiate the "bridge": `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600` (or whatever the port and baudrate are)
