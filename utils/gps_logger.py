import os
import sys
import time

import adafruit_gps
import serial

PI_PORT = "/dev/ttyUSB0"
MAC_PORT = "/dev/cu.usbserial-110"
uart = serial.Serial(MAC_PORT, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)

# Tell to only send RMC (required to get 10 Hz)
gps.send_command(b"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

# Query current NMEA sentence output settings
gps.send_command(b"PMTK414")
while True:
    print("Waiting for response from GPS")
    if not gps.update():
        time.sleep(1)  # sleeping 1s to avoid looping every few microseconds
        continue
    else:
        if "PMTK514" in gps._raw_sentence:
            if (
                gps._raw_sentence
                == "$PMTK514,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*33"
            ):
                print("Command PMTK314 did run successfully")
                break
            else:
                print("Command PMTK314 did not run successfully, exiting")
                sys.exit()

# Set the output frequency at 10 Hz
gps.send_command(b"PMTK220,100")

# NOTE it seems there is no way to make sure the PMTK220 command was applied successfully

filename = input("Give file name: ")
filepath = f"{filename}.nmea"
if os.path.exists(filepath):
    print("File already exists, exiting")
    sys.exit(-1)

output_file = open(filepath, "w")

while True:

    if not gps.update():
        time.sleep(0.001)  # sleeping 1ms to avoid looping every few microseconds
        continue

    if not gps.has_fix:
        # TODO maybe we can use this to trigger a fallback behavior later on
        continue
    print(gps._raw_sentence)
    output_file.write(f"{gps._raw_sentence}\n")
