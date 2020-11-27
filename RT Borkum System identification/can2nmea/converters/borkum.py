"""
See Borkum spec (link)

TRC azimuth goes from 0-360
RPM goes from 0 - 100
"""
import re
import struct

from cap_connections.connection.nmea.checksum import checksum_from_nmea, checksum_nmea
from pyvit import can

from core.can_parser import CANParser

# This is the same for both throttle and azimuth. The only difference is the range but that is handled elsewhere
CAN_BYTE_FMT = "Hbb"

CAN_AZ_ID = 643
CAN_THROTTLE_ID = 387

CAN_THROTTLE_ATTRS = ["throttle", "alarm_info", "status_info"]
CAN_AZIMUTH_ATTRS = ["azimuth", "alarm_info", "status_info"]

# TODO: Make (abstract) base converter that defines an interface for each converter to adhere to
class CAN2TRCConverter:

    def __init__(self, thruster_id):
        self.throttle_parser = CANParser(CAN_BYTE_FMT, CAN_THROTTLE_ATTRS, can_ids=CAN_THROTTLE_ID)
        self.az_parser = CANParser(CAN_BYTE_FMT, CAN_AZIMUTH_ATTRS, can_ids=CAN_AZ_ID)

        self.thruster_id = thruster_id
        self.az = None
        self.throttle = None

    def update_azimuth(self, can_frame):
        self.az = self.az_parser.to_dict(can_frame)

    def update_throttle(self, can_frame):
        self.throttle = self.throttle_parser.to_dict(can_frame)

    def update(self, can_frame):
        if can_frame.arb_id == CAN_AZ_ID:
            self.update_azimuth(can_frame)
        elif can_frame.arb_id == CAN_THROTTLE_ID:
            self.update_throttle(can_frame)

    def __str__(self):
        return f"Borkum CAN2TRCConverter az={self.az}, throttle={self.throttle}"

    @property
    def nmea(self):
        if self.az and self.throttle:
            throttle_value = self.throttle["throttle"] / 10 # Not sure about this one
            az_value = int(round(self.az["azimuth"] / 10))

            # Borkum has no pitch control and is set to 0.
            raw = f"$PTRC,{self.thruster_id},{throttle_value},P,0,P,{az_value},B"
            return checksum_nmea(raw) + "\r\n"


class TRC2CANConverter:

    def __init__(self, thruster_id=None):
        self.thruster_id = thruster_id

    def convert(self, nmea):
        try:
            if not isinstance(nmea, str):
                return None

            nmea = nmea.rstrip()
            nmea, checksum = nmea.split("*")
            exp_checksum = checksum_from_nmea(nmea)
            #if not re.match("\$\w+,\d+\.?\d*,\d+,\w,\d,\w,\d+\.?\d*,\w\*[\d\w]{2}", nmea):
            if not re.match("\$[A-Z]{1,2}TRC", nmea):
                print("INVALID NMEA")
                return None

            if checksum != exp_checksum:
                return None

            parts = nmea.split(",") # This is just a lot easier than using that regex thing
            if self.thruster_id is not None:
                thruster_id = int(parts[1])
                if thruster_id != self.thruster_id:
                    print("Thruster ID ignored ", thruster_id)
                    return None

            throttle = int(round(float(parts[2]) * 10))
            az = int(round(float(parts[6]) % 360 * 10))

            print("az=", az)
            print("throttle=", throttle)

            # We do not support these currently
            status_info = 0
            alarm_info = 0

            throttle_data = [int(x) for x in struct.pack(CAN_BYTE_FMT, throttle, status_info, alarm_info)]
            az_data = [int(x) for x in struct.pack(CAN_BYTE_FMT, az, status_info, alarm_info)]

            return [can.Frame(arb_id=CAN_THROTTLE_ID, data=throttle_data),
                    can.Frame(arb_id=CAN_AZ_ID, data=az_data)]
        except Exception as e:
            print("CAN conversion encountered error", e)
            return None

