import struct

# I have issues with this one as it pads out the data despite using a lower DLC.
# from canard import can

from pyvit import can


class CANParser:

    def __init__(self, byte_fmt, attribute_names=None, can_ids=None):
        self.byte_fmt = byte_fmt
        self.attribute_names = attribute_names

        if type(can_ids) is int:
            self.can_ids = [can_ids]
        else:
            self.can_ids = can_ids

    def to_list(self, can_frame):
        if self.can_ids is None or can_frame.arb_id in self.can_ids:
            data = bytes(bytearray(can_frame.data))
            return struct.unpack(self.byte_fmt, data)
        else:
            raise ValueError("Unexpected frame ID ", can_frame.id)

    def to_dict(self, can_frame):
        if self.attribute_names:
            values = self.to_list(can_frame)
            return dict(zip(self.attribute_names, values))
        else:
            raise ValueError("CANParser does not have any attributes set to use as dictionary keys")


if __name__ == '__main__':

    """
    ID 643 {0x00 0x00 0x00 0x00} 0grade {0x00, 0x07, 0x00, 0x00} 180 degrees
    """

    # Sample thruster azimuth CAN frame
    data = bytes([0x00, 0x07, 0x00, 0x00])
    ints = [int(x) for x in data]

    frame = can.Frame(643, data=ints)

    # CAN bus messages parser of azimuth
    az_parser = CANParser("Hbb", ["azimuth", "alarm_info", "status_info"])
    print(az_parser.to_dict(frame))

    data = bytes([0x82, 0x00, 0x00, 0x01])
    ints = [int(x) for x in data]
    frame = can.Frame(387, data=ints)
    speed_parser = CANParser("Hbb", ["speed", "alarm_info", "status_info"])
    print(speed_parser.to_dict(frame))

