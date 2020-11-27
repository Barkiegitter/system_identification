import importlib
import sys
import time

import fire
from cap_connections.connection.nmea import NMEAReceiver, NMEASender
from cap_connections.connection.udp import UDPReceiver, UDPSender
from systems_monitor.client import SystemMonitorClient

from util import pretty_hex

from pyvit.hw import cantact

SYSTEM_MONITOR_URL = "http://localhost:3000/systems"
SYSTEM_MONITOR_ID = "N2C"


def can_to_trc(converter_name, thruster_id, serial_port, can_bitrate, output=None, verbose=False, speed=10):

    # Try and load the converter script with that name from converters module
    mod_path = f'converters.{converter_name}'
    lib = importlib.import_module(mod_path)

    # potentials = [x for x in dir(lib) if x.startswith("Captain")]
    CAN2TRCConverter = getattr(lib, "CAN2TRCConverter")
    converter = CAN2TRCConverter(thruster_id)

    dev = cantact.CantactDev(serial_port)
    dev.set_bitrate(can_bitrate)
    dev.start()  # Go on the bus

    # Set up how we want to process outgoing nmea sentences
    if output is not None:
        target_type, addr, port = output.split(':')
        if target_type == 'udp': # NMEASender supports this
            sender = NMEASender(addr, int(port), verbose=verbose, checksum=True)
            out_fn = sender.send_one
        else:
            raise ValueError("Invalid output string", output)
    else:
        out_fn = print

    sleep_time = 1/speed
    try:
        while True:
            frame = dev.recv()  # Receive a CAN frame
            converter.update(frame)

            nmea = converter.nmea
            if nmea is not None:
                out_fn(nmea)

            if verbose:
                print(converter)
                print(frame)
                print(nmea)

            time.sleep(sleep_time)

    except KeyboardInterrupt as e:
        print("Interrupt!")


def trc_to_can(converter_name, serial_port, can_bitrate, source=None, verbose=False, thruster_id=None):
    # Try and load the converter script with that name from converters module
    mod_path = f'converters.{converter_name}'
    lib = importlib.import_module(mod_path)

    # potentials = [x for x in dir(lib) if x.startswith("Captain")]
    TRC2CANConverter = getattr(lib, "TRC2CANConverter")
    converter = TRC2CANConverter(thruster_id=thruster_id)

    dev = cantact.CantactDev(serial_port)
    dev.set_bitrate(can_bitrate)
    dev.start()  # Go on the bus

    valid_nmea_received = 0
    invalid_nmea_received = 0

    def collect_metrics():
        return {
            "valid_nmea_received" : valid_nmea_received,
            "invalid_nmea_received": invalid_nmea_received,
        }


    # def on_nmea(nmea_list):
    #

    if source is None:
        print("reading from stdin not yet supported. Try a udp string (udp:localhost:7777)")
        return
    else:
        target_type, addr, port = source.split(':')
        if target_type == 'udp':  # NMEAReceiver supports this
            nr = NMEAReceiver(UDPReceiver(int(port)))
            nr.open()
            #nonlocal valid_nmea_received, invalid_nmea_received

            while True:
                try:
                    print("reading...")
                    nmea_list = nr.read()

                    for nmea in nmea_list:
                        frames = converter.convert(nmea)

                        if frames:
                            valid_nmea_received += 1
                            if verbose:
                                print("NMEA Received : ", nmea)
                                print("Frames = ", [str(x) for x in frames])

                            for f in frames:
                                dev.send(f)
                        else:

                            invalid_nmea_received += 1
                except KeyboardInterrupt as e:
                    print("done.")
                    sys.exit(0)


# def duplex(converter_name, serial_port, can_bitrate, nmea_source=None, nmea_target=None, verbose=False, speed=10):
#     """
#     Bi-directional NMEA 2 CAN conversion
#     :param converter_name: Name of the module lib in the converters module to be imported.
#     :param serial_port: The serial device port of this machine the CAN bus is hooked up to
#     :param can_bitrate: The bitrate the CAN bus operates on
#     :param nmea_source: A source string indicating where to get NMEA from (e.g. udp:...:... )
#     :param nmea_target: A target string indicating where to send NMEA to (e.g. udp:...:... )
#     :param verbose:
#     :return:
#     """
#
#     # TODO: Lot of code duplication with the uni-directional versions
#     mod_path = f'converters.{converter_name}'
#     lib = importlib.import_module(mod_path)
#
#     # potentials = [x for x in dir(lib) if x.startswith("Captain")]
#     TRC2CANConverter = getattr(lib, "TRC2CANConverter")
#     converter = TRC2CANConverter()
#
#     dev = cantact.CantactDev(serial_port)
#     dev.set_bitrate(can_bitrate)
#     dev.start()  # Go on the bus
#
#     # Set up how we want to process outgoing nmea sentences
#     out_fn = None
#     if nmea_target is not None:
#         target_type, addr, port = nmea_target.split(':')
#         if target_type == 'udp':  # NMEASender supports this
#             sender = NMEASender(UDPSender(addr, int(port)))
#             out_fn = sender.write
#     else:
#         out_fn = print
#
#     def on_nmea(nmea_list):
#
#         for nmea in nmea_list:
#             frames = converter.convert(nmea)
#
#             if verbose:
#                 print("NMEA Received : ", nmea)
#                 print("Frames = ", [str(x) for x in frames])
#
#             for f in frames:
#                 dev.send(f)
#
#     if nmea_source is None:
#         print("reading from stdin not yet supported. Try a UDP string (udp:localhost:7777)")
#         return
#     else:
#
#         # Set up the NMEA receiving end
#         src_type, addr, port = nmea_source.split(':')
#         sleep_time = 1 / speed
#
#         if src_type == 'udp':  # NMEAReceiver supports this
#             nr = NMEAReceiver(int(port), on_nmea, rate=1000)
#             nr.listen()
#             try:
#                 while nr.active:
#                     frame = dev.recv()  # Receive a CAN frame
#                     converter.update(frame)
#                     nmea = converter.nmea
#                     if nmea is not None:
#                         out_fn(nmea)
#                     time.sleep(sleep_time)
#
#             except KeyboardInterrupt as e:
#                 print("done.")
#                 sys.exit(0)
#
#
# def can_dump(serial_port, can_bitrate):
#
#     dev = cantact.CantactDev(serial_port)
#     dev.set_bitrate(can_bitrate)
#     dev.start()  # Go on the bus
#
#     i = 0
#     while True:
#         frame = dev.recv()  # Receive a CAN frame
#
#         print(f"Frame #{i}", frame)
#         i += 1

def decode(serial_port, can_ids, unpack_fmt):
    """ Print a CAN message in hex and the unpacked message according to some unpack """

    dev = cantact.CantactDev(serial_port)
    dev.set_bitrate(500000)  # Set the bitrate to a 1Mbaud
    dev.start()  # Go on the bus

    try:
        while True:
            frame = dev.recv()  # Receive a CAN frame
            if frame.id in can_ids:
                # unpacked = struct.unpack(unpack_fmt)
                hex_str = pretty_hex(frame.data)

    except KeyboardInterrupt as e:
        print("Interrupt!")
    
if __name__ == '__main__':
    fire.Fire()
