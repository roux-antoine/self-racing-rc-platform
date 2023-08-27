#!/usr/bin/python3

import rospy
import serial
from pynmeagps import NMEAReader

from self_racing_car_msgs.msg import RmcNmea

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

        self.pub = rospy.Publisher("gps_info", RmcNmea, queue_size=10)

        rospy.init_node("rtk_gps_serial_reader", anonymous=True)

        self.rate = rospy.Rate(100)  # 100Hz

    def loop(self):

        while not rospy.is_shutdown():

            # Read data from the serial port
            serial_data = self.serial_port.readline().decode("utf-8").strip()

            # If data received, print it
            if serial_data:
                print("Received data from serial port: ", serial_data)

                # uncomment to handle RMC
                if "GNRMC" in serial_data:
                    print(serial_data)
                    parsed_msg = NMEAReader.parse(serial_data)
                    print(parsed_msg)
                    # print("Lat: {}, Lon: {}".format(parsed_msg.lat, parsed_msg.lon))
                    # print(parsed_msg)
                    self.publish_nmea_sentence_rmc(parsed_msg)

                # uncomment to handle GGA
                # NOTE interestingly there is no velocity info in this message
                # if('GNGGA' in serial_data):
                #     print(serial_data)
                #     parsed_msg = NMEAReader.parse(serial_data)
                #     print(parsed_msg)
                #     # print("Lat: {}, Lon: {}".format(parsed_msg.lat, parsed_msg.lon))
                #     # print(parsed_msg)
                #     self.publish_nmea_sentence_gga(parsed_msg)

            self.rate.sleep()

    def publish_nmea_sentence_rmc(self, parsed_msg):
        """
        TODO
        """
        msg = RmcNmea()
        msg.header.stamp = rospy.Time.now()
        msg.timestamp_utc = 0  # TODO replace with self.gps.timestamp_utc but needs to be converted from time.struct_time
        msg.latitude = parsed_msg.lat
        msg.longitude = parsed_msg.lon
        msg.speed_knots = parsed_msg.spd
        msg.track_angle_deg = (
            parsed_msg.cog
        )  # NOTE: what is cog and how to get track angle?

        self.pub.publish(msg)

    def publish_nmea_sentence_gga(self, parsed_msg):
        """
        TODO
        """
        # TODO
        pass


if __name__ == "__main__":
    rtk_gps_serial_reader = RtkGpsSerialReader()
    rtk_gps_serial_reader.loop()
