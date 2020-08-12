# Function Imports
import collections
from pathlib import Path
import os
import socket
import json
import logging
import src.logger as logger

# Todo: generate a function to compare dictionaries and append missing entries.


def dict_merge(dct, merge_dct):
    """ Recursive dict merge. Inspired by :meth:``dict.update()``, instead of
    updating only top-level keys, dict_merge recurses down into dicts nested
    to an arbitrary depth, updating keys. The ``merge_dct`` is merged into
    ``dct``.
    :param dct: dict onto which the merge is executed
    :param merge_dct: dct merged into dct
    :return: None
    """
    for k, v in merge_dct.items():
        if (k in dct and isinstance(dct[k], dict)
                and isinstance(merge_dct[k], collections.Mapping)):
            dict_merge(dct[k], merge_dct[k])
        else:
            dct[k] = merge_dct[k]


class ConfigHandler:
    def __init__(self, config_path="../", log=logging.getLogger()):
        """
        The `class:ConfigHandler` handles the creation and reading of the autopilot configuration file.

        :param config_path: The path to the configuration directory.
        :type config_path: str
        """
        self.config_v = '0.5'
        self.config_f = "config.json"
        self.config = {}
        self.log = log
        if isinstance(config_path, str):
            self.config_path = Path(config_path)  # Turn path to platform independent path.
        else:
            quit()
        config_path = "%s/%s" % (self.config_path, self.config_f)
        if os.path.exists(config_path):
            print(config_path)
            self.config_present = True
        else:
            self.config_present = False

    def get_config(self):
        """
        Basic function that should always return a configuration file.

        :return config: A dictionary with configuration parameters
        :rtype: dict
        """
        if self.config_present:
            self.read_config()
        else:
            self.generate_config()
            self.read_config()
            print("Config File Generated, please edit topics!")
        return self.config

    def read_config(self):
        """
        This function reads the config.json on the location defined by `self.config_path`. File should be named
        `config.json`. During read the version of the file is checked against the hardcoded version in the class and
        appended if the version is different.

        :return:
        """
        if not self.config_present:
            return False

        with open('%s/%s' % (self.config_path, self.config_f), 'r') as file:
            self.config = json.load(file)

        if not self.config_v == self.config.get('Version'):
            self.log.warning('Configuration file version not equal to current code version')

    def generate_config(self, user_defined={}):
        """
        This function generates a template configuration file, most values are unintelligently filled.
        It does fill in your IP as well as PC name.

        :return: Returns success or failure
        :rtype: bool
        """
        if self.config_present:
            logging.info("Config already present. Reusing it")
            dict_merge(self.config, user_defined)
            return False

        pc_name = socket.gethostname()
        # Get local IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            self_ip = s.getsockname()[0]
            s.close()
        except ConnectionError as e:
            self_ip = "0.0.0.0"
            print("Connection could not be established:  %s" % e)
            print('reverting to localhost')
        # generate config file
        config = {"Version": self.config_v, "PC_Name": pc_name, "pc_ip": self_ip}

        config['GENERAL'] = {
            "my_mmsi": 254718240,
        }
        config['MODI'] = {
            "threaded_mode": True,
            "cambpix": False,
            "udp_mode": True,
            "start_command": False,
            "spoofsonar": True,
            "serial_control": False,
            "start-command": False,
            "debugging_mode": False,
            "topic_mode": True,
        }
        config['IP_ADDRESSES'] = {
            "self_ip": self_ip,
            "ship_comm_address": "192.168.10.4",
            "ip_rec_telemetry": self_ip,
            "ip_send_atlantis": self_ip,
            "ip_send_CapApp": "192.168.10.4",
            "ip_send_SendRPA3": "192.168.10.5",
        }
        config['LOGGER'] = logger.default()
        config['UDP_PORTS'] = {
            "port_rec_NMEA": 7777,
            "port_rec_AIS": 7778,
            "port_rec_Track": 5079,
            "port_rec_Asterix": 4377,
            "port_rec_ATL": 8008,
            "port_send_atlantis": 3322,
            "port_send_CapApp": 3322,
            "port_send_SendRPA3": 1234,
        }
        config['ARDUINO'] = {
            "com_send_Arduino": "/dev/ttyS1",
            "baud_send_Arduino": 115200,
        }
        config['SHIP_MAT'] = {
            "ships_mat_samples": 8,
            "ships_mat_vessels": 50,
            "ship_mat_parameters": 21,
            "ROI_ships": 0.08,
        }
        config['NAV_SHAPES'] = {
            "ships_data": "../data/ShipDimensions"
        }
        config['NAV_PATH'] = {
            "ships_mass": 10,
            "path_seed": "../data/PathSeeds/Boompjes2Waalhaven_pier7_8.csv",
            "path_decimation": 10,
            "force_static_path": False
        }
        config['NMEA'] = {
            "message_types": ["$GPRMC", "$HCHDT", "$HEHDT", "$SDDPT"]
        }
        config['TOPICS'] = {
            "nav_path": "paths_el",
            "nav_forecast": "forecast_el",
            "metrics_topic": "metrics_el",
            "nav_control": "controls_el",
            "ais_topic": "ais_el",
            "nmea_topics": ["nmea_el"]
        }

        dict_merge(config, user_defined)

        if not os.path.exists(self.config_path):
            os.makedirs(self.config_path)

        with open('%s/%s' % (self.config_path, self.config_f), 'w') as outfile:
            json.dump(config, outfile, indent=4)
        self.config_present = True
        logging.info(f"Configuration File generated. Saved to {self.config_path}")
        return True


if __name__ == "__main__":
    a = ConfigHandler(config_path='')
    a.generate_config()
    a.read_config()
    print(a.config)
