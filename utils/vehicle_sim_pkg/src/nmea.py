# from datetime import datetime, timezone
from geometry_utils.geometry_utils import State

# import utm
# import numpy as np


class Nmea:
    def __init__():
        # self.mps_to_knots = 1.94384
        pass

    def create_sentence_string_from_state(self, state: State) -> str:
        """
        Function to generate a NMEA sentence from the vehicle sim's current state

        Arguments:
            - state [State]: Current state of vehicle sim
        Outputs:
            - sentence [str]: Nmea sentence
        """

        # # Compute latitude and longitude from UTM coordinates
        # zone_number = 10
        # zone_letter = "S"
        # latitude, longitude = utm.to_latlon(state.x, state.y, zone_number, zone_letter)

        # # Field 1
        # rmc_string = "$GNRMC"

        # # Field 2: Time fix was taken [HHMMSS]
        # now = datetime.now(timezone.utc)
        # utc = "%s%s%s" % (now.hour, now.minute, now.second)

        # # Field 3: Status. A = active. V = void
        # status = "A"

        # # Field 4: Latitude and latitude direction
        # lat = latitude
        # latitude_direction = "N"

        # # Field 5: Longitude and longitude direction
        # lon = longitude
        # longitude_direction = "W"

        # # Field 6: Speed (knots)
        # speed_kts = state.vx * self.mps_to_knots

        # # Field 7: Track angle in degrees (origin is true north. Clockwise direction)
        # track = state.angle * 180 / np.pi  # TODO: Convert this

        # # Field 8: Date [DDMMYYYY]
        # date = "%s%s%s" % (now.day, now.month, now.year)

        # # Field 9: Mode indicator. A = autonomous, E = estimated, S = simulator
        # mode = "A"

        # # Field 10: Navigational status. S = safe, C = caution, U = unsafe, V = void
        # nav_status = "S"

        # # Field 11: Checksum data. Begins with *
        # # checksum =

        # sentence = rmc_string + "," + longitude_direction

        # return sentence
        pass
