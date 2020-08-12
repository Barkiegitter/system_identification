import numpy  as np
import json
import logging
import src.logger as logger
# Communication
from cap_comm.ship_comm_client.client import ShipCommClient, write_topic
from cap_comm.ship_comm_client.constants import DataType

# Multithreading - Will change to numba in future versions
from multiprocessing.pool import ThreadPool
from multiprocessing import Process, Queue

from src.communications.nmea_receive import UDPProcessor as UDPP
from src.nav_control import SimpleControls
from src.nav_path import NavPath
from src.utilities.confighandler import ConfigHandler



class Autopilot:

    def __init__(self, config_path):
        self.pool0 = ThreadPool(processes=1)
        self.pool1 = ThreadPool(processes=1)
        self.pool2 = ThreadPool(processes=1)
        self.pool3 = ThreadPool(processes=1)
        self.pool4 = ThreadPool(processes=1)
        self.pool5 = ThreadPool(processes=1)
        self.pool6 = ThreadPool(processes=1)

        # Load configuration file
        cnf = ConfigHandler(config_path=config_path)
        self.settings = cnf.get_config()    # will integrate the complete config file later, now just this placeholder
        # self.settings = json.load(open(config_path, 'r'))


        # temporary cheat so I don't have to fix all the original spots which assume 18 cols
        # ships_mat_radar_ext = np.zeros((150,27)) # 9 columns added for radar data - trackid, (lat,lng) x 4 corners

        self.ships_mat = np.zeros((self.settings['SHIP_MAT'] ['ships_mat_samples'],
                                   self.settings['SHIP_MAT']['ships_mat_vessels'],
                                   self.settings['SHIP_MAT']['ship_mat_parameters']))
        self.ships_mat[:, 0, 1] = self.settings['GENERAL']['my_mmsi']  # 244070156.0 #RPA3 244596336.0 #sim
        self.ships_mat[:, :, 14:18] = 15.0  # default ship dimensions

        # port_rec_ATL  = self.settings['port_rec_ATL' ]; ip_rec_ATL  = self.settings['ip_rec_ATL' ]

        # Atlantis commands
        self.start_command = self.settings['MODI']['start_command']
        self.debugging_mode = self.settings['MODI']['debugging_mode']
        self.threaded_mode = self.settings['MODI']['threaded_mode']
        self.cambpix = self.settings['MODI']['cambpix']
        self.udp_mode = self.settings['MODI']['udp_mode']
        self.topic_mode = self.settings['MODI']['topic_mode']
        self.for_send = False
        
        self.logging = True

        if self.logging == True:
            # Processes
            self.processes = []
            self.log_q = Queue(maxsize=30)  # making this 30 will output errors if the queue is full
            self.settings['LOGGER']['log_q'] = self.log_q
            # Initiate the logger, must (probably) be done before the other classes initialise.
            logging_proc = Process(name="Logger_Listener", target=logger.LoggerListener,
                                args=(self.log_q, self.settings['LOGGER']))
            logging_proc.deamon = True
            logging_proc.start()
            self.processes.append(logging_proc)     # Its always a good idea to track your processes.
            logger.worker_configurer(self.settings["LOGGER"])    # configure local logger with the correct settings
            self.log = logging.getLogger(name="Autopilot")

        # Initializing classes
        self.sc_client = ShipCommClient(self.settings['IP_ADDRESSES']["ship_comm_address"],
                                        local_address=self.settings['IP_ADDRESSES']['self_ip'],
                                        process_name="autopilot")
        # self.SendAt = UDPSender(self.settings['nav_control']['ip_send_atlantis'], self.settings['nav_control']['port_send_atlantis'])
        self.UDPPEM = UDPP(self.settings)  # Process UDP streams
        self.NvPath = NavPath(self.settings)  # Calculate Paths
        self.NvCtrl = SimpleControls(self.settings)  # For sending control commands to ship
        # CodeAn  = CodeAnalytics()#For Debugging code

        self.engaged = False

    def engage(self):
        self.log.info("Engaging Autopilot")

        # self.pool0 = ThreadPool(processes=1)
        # self.pool1 = ThreadPool(processes=1)
        self.pool2 = ThreadPool(processes=1)
        # self.pool3 = ThreadPool(processes=1)
        self.pool6 = ThreadPool(processes=1)
        self.pool5 = ThreadPool(processes=1)
        self.engaged = True
        if self.topic_mode:
            self.run_topic_mode()

    def disengage(self):
        """
        Stop all the threads and the ship-comm-client threads
        """
        self.log.info("Disengaging Autopilot")
        self.sc_client.stop()
        # self.pool1.terminate()
        # self.pool0.terminate()
        self.pool2.terminate()
        # self.pool3.terminate()
        self.pool6.terminate()
        self.pool5.terminate()
        self.log_q.put(None)
        self.engaged = False

    def run_topic_mode(self):
        # ais_topic = self.settings['TOPICS']["ais_topic"]
        nmea_topics = tuple(self.settings['TOPICS']["nmea_topics"])  # There can be multiple NMEA topics (XSens, etc.). I might change this in the future

        self.log.info("Topic Mode!")

        # Initialise the topics for writing and reading.
        # Using DataType NMEA ensures only proper formatted and checksummed NMEA are read and written!
        self.sc_client.init_write_topic(self.settings["TOPICS"]["nav_control"], DataType.NMEA)
        self.sc_client.init_write_topic(self.settings["TOPICS"]["metrics_topic"])
        # self.sc_client.init_read_topic(ais_topic, DataType.NMEA)
        self.sc_client.init_read_topic(nmea_topics, DataType.NMEA)

        def update(received):
            self.pool5.apply_async(self.NvPath.GeneratePaths, (received, self.for_send, self.ships_mat))
            self.pool6.apply_async(self.NvCtrl.BORKUM, (received, self.for_send, self.ships_mat))
            # write_topic(self.settings["TOPICS"]["metrics_topic"], "dist2track:3.14")

        def handle_nmea(nmea):
            # print("NMEA Received:", nmea.strip())
            self.pool2.apply_async(self.UDPPEM.process_NMEA, (nmea, self.for_send, self.ships_mat))
            update(nmea)

        self.sc_client.read_topics_forever({
            nmea_topics: handle_nmea
        }, block=False)


