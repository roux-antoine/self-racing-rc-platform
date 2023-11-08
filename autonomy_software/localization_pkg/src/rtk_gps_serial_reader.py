#!/usr/bin/python3

import datetime as dt
import rospy
import serial
from pynmeagps import NMEAReader

from nmea_msgs.msg import Gprmc

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

"""
TODO:
- Good way to have try / while True within ros looop?
- improve logging
- improve error handling
"""


class RtkGpsSerialReader:
    def __init__(self):
        self.serial_port = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)

        self.pub = rospy.Publisher("gps_info", Gprmc, queue_size=10)

        rospy.init_node("rtk_gps_serial_reader", anonymous=True)

        self.rate = rospy.Rate(100)  # 100Hz

    def loop(self):

        while not rospy.is_shutdown():

            # Read data from the serial port
            serial_data = self.serial_port.readline().decode("utf-8").strip()

            # If data received, print it
            if serial_data:
                # uncomment to handle RMC
                if "GNRMC" in serial_data:
                    parsed_msg = NMEAReader.parse(serial_data)
                    print(parsed_msg)
                    self.publish_nmea_sentence_rmc(parsed_msg)

            self.rate.sleep()

    def publish_nmea_sentence_rmc(self, parsed_msg):
        """
        Formats the parsed line from the serial into a nmea_msgs/Gprmc message and publishes it
        """
        # we need to construct the date and time by hand
        date_time_string = f"{parsed_msg.date}_{parsed_msg.time}"
        if "." in date_time_string:
            dt_object = dt.datetime.strptime(date_time_string, "%Y-%m-%d_%H:%M:%S.%f")
        else:
            dt_object = dt.datetime.strptime(date_time_string, "%Y-%m-%d_%H:%M:%S")

        msg = Gprmc()
        msg.header.stamp = rospy.Time.now()
        msg.utc_seconds = dt_object.replace(tzinfo=dt.timezone.utc).timestamp()
        msg.lat = parsed_msg.lat
        msg.lon = parsed_msg.lon
        msg.lat_dir = parsed_msg.NS
        msg.lon_dir = parsed_msg.EW
        msg.speed = parsed_msg.spd
        msg.track = parsed_msg.cog
        msg.date = parsed_msg.date.strftime("%Y-%m-%d")
        try:
            msg.mag_var = float(parsed_msg.mv)
        except Exception:
            msg.mag_var = -1
        msg.mag_var_direction = parsed_msg.mvEW
        msg.mode_indicator = parsed_msg.posMode

        self.pub.publish(msg)


if __name__ == "__main__":
    rtk_gps_serial_reader = RtkGpsSerialReader()
    rtk_gps_serial_reader.loop()
