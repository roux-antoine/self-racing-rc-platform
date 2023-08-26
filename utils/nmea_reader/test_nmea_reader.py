"""
Script to test the pynmeagps library
To understand RMC format: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual1.pdf
"""

from pynmeagps import NMEAReader

# The example sentences won't work if you change the data without changing the checksum. 
USE_CHECKSUM_VALIDATOR = False

# INPUT NMEA SENTENCES

# Sentence from our rtk gps - manually added course over ground = 12
input_sentence_rmc = '$GNRMC,031406.60,A,3744.61398,N,12226.29502,W,0.020,12,100823,,,R,V*11\r\n'

# Same. No speed. In parsed result, cog = None
# input_sentence_rmc = '$GNRMC,031406.60,A,3744.61398,N,12226.29502,W,0.020,,100823,,,R,V*11\r\n'

# PARSE SENTENCE
parsed_msg = NMEAReader.parse(input_sentence_rmc, validate=USE_CHECKSUM_VALIDATOR)

# RESULT
print(" ")
print('Input sentence: ', input_sentence_rmc)
print('Parsed message: ', parsed_msg)
print("Lat: {}, Lon: {}, Speed: {} kts, Orientation: {} degrees".format(parsed_msg.lat, parsed_msg.lon, parsed_msg.spd, parsed_msg.cog))