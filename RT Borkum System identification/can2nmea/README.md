# CAN2NMEA

This Python package allows you to convert CAN frames received over Serial and send them out over UDP in TRC NMEA 0183 format and vice versa.

The code is structured in a modular way so that specific converters for different vendors (like VETH propulsion on the Borkum) can be coded up and used interchangably. Converter code should be stored in the converters directory.

## CAN2NMEA

CAN to NMEA conversion takes in CAN frames and returns an NMEA string. The structure of the CAN frames depend on the vendor of the propulsion system. The NMEA output depends on what makes sense for what you are outputting. For a vessel with three rotatable thrusters (like the RT Borkum) and no rudder it makes sense that it outputs a TRC NMEA string only that identifies the thruster its azimuth and the RPM. A converter such as this exists for the [Borkum](converters/borkum.py) though it is not in use today because we do not read out the CAN frames we only send CAN frames to the thrusters. When this changes you may want to look at this code and modify where needed.


## NMEA2CAN

NMEA to CAN conversion is useful if you are attempting to control a vessel in a way similar to how NAUTIS controls a boat. In fact, I have used this as the basis for conversion so that any algorithm developed in the simulator will work in a plug-and-play fashion on the RT Borkum. It is advised that any future vessels work in a similar fashion. 

## CLI Tool

The CLI tool at `main.py` currently offers two flavors trc2can and can2trc. Both are explained here. In the future you may want to expand this to other NMEA sentences or make them more generic as discussed in the future work section

### CAN2TRC

Running CAN to TRC conversion from command line can be done as follows:

`can_to_trc <converter_name> <thruster_id> <serial_port> <can_bitrate> --output=<string> --verbose, --speed=<n>`

* `converter_name`: required parameter that indicates a Python file in the converters module. This lib is then dynamically loaded and the CAN2TRCConverter class is located and imported.
* `thruster_id`: A required parameter that sets the TRC messages to a specific ID which identifies which thruster this data is coming from
* `serial_port`: The path to the serial device e.g. `/dev/ttyUSB0` 
* `can_bitrate` : The bit rate to be used to write to the device
* `--verbose`: An optional flag to print verbose message not otherwise displayed like the CAN Frames received and the generated TRC strings
* `--speed`: An optional integer parameter that defaults to 10 indicating the speed at which the CAN frames are read. **NOTE: that setting this number to high may exhaust the CANbus bandwidth and severely impede the performance of the CANbus!** On the RT Borkum around 10 is fine, going higher than 20-30 not so much.

### TRC2CAN

Running the NMEA2CAN conversion can be done by issuing a command like so:

`python main.py trc_to_can <converter> <serial_port> <can_bitrate> --source=<source_identifier> --verbose --thruster_id=<n>`

Where trc_to_can indicates that we are converting TRC NMEA to CAN frames. Note the specific TRC identifier in this case. In the future this should probably be changed to a more generic `nmea_to_can` converter type (see Known Issues / Future work).

The parameters can be expalained as such:

* `converter`: required parameter that indicates a Python file in the converters module. This lib is then dynamically loaded and the TRC2CanConverter class is located and imported.
* `serial_port`: The path to the serial device e.g. `/dev/ttyUSB0` 
* `can_bitrate` : The bit rate to be used to write to the device
* `--verbose`: An optional flag to print verbose message not otherwise displayed like the NMEA messages received and the created CAN frames
* `--thruster-id`: An optional parameter that filters out and ignores any TRC message that does not have the given thruster ID (the first param in a TRC message). If this parameter is not given no TRC messages are filtered out based on their index.

`trc_to_can borkum /dev/ttyACM0 125000 --source=udp:localhost:5556 --verbose=True`

### Known Issues / Future Work

* The `trc_to_can` CLI method should possibly be generalised to `nmea_to_can` and the converters should simply be called `NMEA2CANConverter` rather than be related to specific NMEA sentences like `TRC`. The down-side is that it becomes more complicated to take into accout specific NMEA details like thruster_id. What do you do for rudder in that case?
* Duplex conversion: Should not be too hard but at some point you would want to be able to read and write from a CAN bus in a full duplex fashion. 
* The repo is a bit messy and should be cleaned and released as a package