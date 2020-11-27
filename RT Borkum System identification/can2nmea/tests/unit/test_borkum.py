import struct
import unittest
import random

from cap_connections.nmea.checksum import checksum_nmea

from converters.borkum import TRC2CANConverter

"""
Making sure the CAN signals are as you expect is very important as this controls the actual thrusters!
"""
class TestBorkumNMEA2CANConversion(unittest.TestCase):

    """
    $TRC,x,x.x,a,x.x,a,x.x,a,a*ff
    1. NumberOfThruster : Number of thruster, bow or stern
    2. RPMDemandValue : RPM demand value
    3. RPMModeIndicator : RPM mode indicator
    4. PitchDemandValue : Pitch demand value
    5. PitchModeIndicator : Pitch mode indicator
    6. AzimuthDemand : Azimuth demand
    7. OperatingLocationIndicator : Operating location indicator
    8. SentenceStatusFlag : Sentence Status Flag
    """

    def setUp(self) -> None:
        self.conv = TRC2CANConverter()

    def test_invalid_nmea(self):
        self.assertIsNone(self.conv.convert("THISISNOTEVENNMEAWHATAREYOUDOING"))
        self.assertIsNone(self.conv.convert(b"THISISNOTEVENASTRINGWHATAREYOUDOING"))
        self.assertIsNone(self.conv.convert(1))
        self.assertIsNone(self.conv.convert(None))

    def test_invalid_checksum(self):
        self.assertIsNone(self.conv.convert("$PTRC,0,0,P,0,P,75,B*00"))

    def test_throttle_conversion(self):

        for rpm in range(101):
            s = f"$PTRC,0,{rpm},P,0,P,0,B"
            s = checksum_nmea(s)

            frame, _ = self.conv.convert(s)

            data = bytes(bytearray(frame.data))
            throttle, _, _ = struct.unpack("Hbb", data)

            print("NMEA Sentence=", s)
            print("CAN throttle =", throttle)

            self.assertEqual(throttle, rpm*10)

    def test_azimuth_conversion(self):

        for az in range(0, 360):
            s = f"$PTRC,0,0,P,0,P,{az},B"
            s = checksum_nmea(s)

            _, frame = self.conv.convert(s)

            data = bytes(bytearray(frame.data))
            can_az, _, _ = struct.unpack("Hbb", data)

            print("NMEA Sentence=", s)
            print("CAN Azimuth  =", can_az)

            self.assertEqual(can_az, az * 10)

    def test_specific_thruster_id(self):
        conv = TRC2CANConverter(thruster_id=1)

        # Should not do anything since the thruster ID does not match
        self.assertIsNone(conv.convert("$PTRC,67,0,P,0,P,1,B*4B"))



