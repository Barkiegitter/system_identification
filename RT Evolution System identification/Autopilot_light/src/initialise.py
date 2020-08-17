"""
The sole purpose of this file is to get rid of the near infinite list of
imports in the main file
"""
import sys

import numpy  as np
import pickle as pkl
import json

#Communication
from communications.udp_listener   import UDPListener  as UDPL
from communications.nmea_receive   import UDPProcessor as UDPP
from communications.udp_manager    import NMEASender
from communications.sender         import UDPSender

#Radar
from spx_track_handler import SPxHandler
import socket

#Local Classes
from nav_control    import SimpleControls
from nav_path       import NavPath

#Multithreading - Will change to numba in future versions
import multiprocessing      as mp
from   multiprocessing.pool import ThreadPool
from   multiprocessing      import RawArray

#Logging
import logger
from multiprocessing import Queue
from multiprocessing import Process