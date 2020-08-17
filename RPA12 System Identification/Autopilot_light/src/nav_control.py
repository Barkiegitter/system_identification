#Rewrite code for PID tunning
# Test terminal command
# echo "\$PROR,139,A,139,A*1F" | socat - udp-sendto:192.168.10.177:4444

# from profiler              import profile

# communications
from cap_comm.ship_comm_client.client import write_topic, init_write_topic

import numpy as np
import csv
import time
import datetime
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import json

import src.nav_tools as nt
from src.communications.nmea_creator import NMEAStrings
from src.communications.sender import UDPSender, SerialDummy, SerialSender
from src.utilities.pid import PID
from src.visualize import live_plotter
import src.logger as logger
import logging
import random

class SimpleControls:
    def __init__(self, settings):

        self.topic_name = settings["TOPICS"]["nav_control"]
        self.metrics_topic = settings["TOPICS"]["metrics_topic"]

        serial_control = settings['MODI']['serial_control']

        self.iter_time = time.time()
        self.nmea_make = NMEAStrings()
        self.logging = False
        if self.logging == True:
            logger.worker_configurer(settings['LOGGER']['log_q'])
        self.log = logging.getLogger('CONTROL')

        # TODO: We would likely want to have a single topic for sending controls, but there is
        # self.SendAtln = UDPSender(settings['IP_ADDRESSES']['ip_send_atlantis'],
        #                           settings['UDP_PORTS']['port_send_atlantis'])
        self.SendRPA3 = UDPSender(settings['IP_ADDRESSES']['ip_send_SendRPA3'],
                                  settings['UDP_PORTS']['port_send_SendRPA3'])

        # self.SendArduino = SerialSender(settings['nav_control']['com_send_Arduino'],
        # settings['nav_control']['baud_send_Arduino'])

        if serial_control == False:
            self.SendArduino = SerialDummy('/dev/ttyS1', 115200)

        if serial_control == True:
            self.SendArduino = SerialSender('/dev/ctrl', 500000)

        # StaticPath = False
        self.SendTRC = True  # False

        # PID
        self.turning_right = True

        recieved = 0
        ships_mat = 0

        for_send = self.nmea_make.ROR(0, 0)
        self.SendArduino.send_string(recieved, for_send, ships_mat)
        for_send = self.nmea_make.RSA(0, 0)
        # self.SendAtln.send_string(recieved, for_send, ships_mat)
        self.t_reg = time.time()
        self.t_reg_plot = time.time()
        # plotting properties
        self.x_vec = np.linspace(0, 1, 1000 + 1)[0:-1]
        self.y_vec = np.zeros(len(self.x_vec))
        self.hdg_input = np.zeros(len(self.x_vec))
        self.line1 = []
        self.line2 = []

        self.manoeuvre = None
        self.manoeuvre_check = None

        #circle right
        self.heading_difference_circle_right = 0.0
        self.check_circle_manoeuvre_right = 0
        self.t_start_manoeuvre_circle_right = 0.0
        self.t_end_manoeuvre_circle_right = time.time()


        #circle left
        self.t_start_manoeuvre_circle_left = 0.0
        self.check_circle_manoeuvre_left = 0
        self.heading_difference_circle_left = 0.0
        self.t_end_manoeuvre_circle_left = time.time()

        #zigzag 10
        self.t_end_manoeuvre_zigzag_10 = time.time()
        self.manoeuvre_zigzag_10_phase = 0

        # zigzag 20
        self.t_end_manoeuvre_zigzag_20 = time.time()
        self.manoeuvre_zigzag_20_phase = 0

        #astern

        self.t_start_manoeuvre_circle_right = 0
        self.t_start_manoeuvre_circle_left = 0
        self.t_start_manoeuvre_zigzag_10 = 0
        self.t_start_manoeuvre_zigzag_20 = 0
        self.t_start_manoeuvre_astern = 0

    def write_csv(self, data, name):
        headers = ['time', 'lat', 'lon', 'hdg', 'rpm', 'rsa']
        with open(name+'.csv', 'a') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(data)
            writer.writerow(headers)

    def read_csv(self, file_name):
        with open('example.csv', 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            return csv_reader[-1]

    #create def function sending commands to ship
    

    def RPA12_command_propeller(self, ):



    def RPA12_command_rudder(self):




    def survey_throttle(self, ships_mat):
        #Sea Hunter
        #SOG = -(8.6/(4^d))+8.6

        #Zenit
        #SOG = -(10.1/(2.8^d))+10.1

        #Zenit
        Depth = ships_mat[0, 0, 6]
        denominator = np.power(2.8, Depth)
        numerator   = np.divide(10.1, denominator)
        SOG = 10.1 - numerator 
        self.default_throttled = SOG.astype('int16')
        # rerturn(self.default_throttled)

    def throttle_avenger(self, throttle_input):
        self.throttle_array = np.roll(self.throttle_array, 1)
        self.throttle_array[0] = throttle_input
        return np.average(self.throttle_array)
    def rudder_avenger(self, rudder_input):
        self.rudder_array = np.roll(self.rudder_array, 1)
        self.rudder_array[0] = rudder_input
        return np.average(self.rudder_array)

    def plot_live(self, x_vec, y1_data, y2_data):
        self.ax1.clear()
        self.ax1.plot(x_vec, y1_data, y2_data)


    def manoeuvre_circle_right(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_circle_right = self.t_start_manoeuvre_circle_right + (time.time() - self.t_reg)

            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference_circle_right
            print(covered_turn)
            if self.t_start_manoeuvre_circle_right<2.0:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
            elif covered_turn>=-10 and covered_turn<=-5 or abs(covered_turn)>350 or self.check_circle_manoeuvre_right==1:

                self.check_circle_manoeuvre_right = 1
                print(time.time() - self.t_end_manoeuvre_circle_right)
                if time.time() - self.t_end_manoeuvre_circle_right > 5.0:
                    self.manoeuvre = None

                else:
                    # MAIN STARBOARD THRUSTER
                    for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # MAIN PORT THRUSTER
                    for_send = self.nmea_make.TRC(1, 85, 100, 0)
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # BOW THRUSTER
                    for_send = self.nmea_make.TRC(0, 0, 100, 0)
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # RUDDERS
                    for_send = self.nmea_make.ROR(int(0), int(0))
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)
            else:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0 )
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #RUDDERS
                for_send = self.nmea_make.ROR(int(35), int(35))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                self.t_end_manoeuvre_circle_right = time.time()


            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            -float(ships_mat[2, 0, 3])], environment_variable)

    def manoeuvre_circle_left(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_circle_left = self.t_start_manoeuvre_circle_left + (time.time() - self.t_reg)
            # print(ships_mat[2,0,3])
            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference_circle_left
            print(covered_turn)
            if self.t_start_manoeuvre_circle_left<2.0:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
            elif covered_turn<=10 and covered_turn>=5 or abs(covered_turn)>350 or self.check_circle_manoeuvre_left==1:
                self.check_circle_manoeuvre_left = 1
                if time.time() - self.t_end_manoeuvre_circle_left >5.0:
                    self.manoeuvre = None

                else:
                    # MAIN STARBOARD THRUSTER
                    for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # MAIN PORT THRUSTER
                    for_send = self.nmea_make.TRC(1, 85, 100, 0)
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # BOW THRUSTER
                    for_send = self.nmea_make.TRC(0, 0, 100, 0)
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    # RUDDERS
                    for_send = self.nmea_make.ROR(int(0), int(0))
                    self.SendRPA3.send_string("recieved", for_send, ships_mat)
                    write_topic(self.topic_name, for_send)

                    #start another timer add another iff statement


            else:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0 )
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                #RUDDERS
                for_send = self.nmea_make.ROR(int(-35), int(-35))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                self.t_end_manoeuvre_circle_left = time.time()


            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            -float(ships_mat[2, 0, 3])], environment_variable)




    def manoeuvre_zigzag_10(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_zigzag_10 = self.t_start_manoeuvre_zigzag_10 + (time.time() - self.t_reg)

            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference_zigzag_10
            print(covered_turn)
            if self.t_start_manoeuvre_zigzag_10<2.0:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
            elif self.manoeuvre_zigzag_10_phase == 0:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(10), int(10))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                if covered_turn>10.0 or covered_turn>-350 and covered_turn<-345:
                    self.manoeuvre_zigzag_10_phase = 1
            elif self.manoeuvre_zigzag_10_phase==1:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(-10), int(-10))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                self.t_end_manoeuvre_zigzag_10 = time.time()
                if covered_turn<-10.0 or covered_turn<350 and covered_turn>345:
                    self.manoeuvre_zigzag_10_phase = 2
            elif self.manoeuvre_zigzag_10_phase == 2:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                if time.time() - self.t_end_manoeuvre_zigzag_10 > 5.0:
                    self.manoeuvre = None




            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            -float(ships_mat[2, 0, 3])], environment_variable)
    def manoeuvre_zigzag_20(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_zigzag_20 = self.t_start_manoeuvre_zigzag_20 + (time.time() - self.t_reg)

            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference_zigzag_20
            print(covered_turn)
            if self.t_start_manoeuvre_zigzag_20<2.0:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
            elif self.manoeuvre_zigzag_20_phase == 0:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(20), int(20))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                if covered_turn>20.0 or covered_turn>-340 and covered_turn<-335:
                    self.manoeuvre_zigzag_20_phase = 1
            elif self.manoeuvre_zigzag_20_phase==1:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(-20), int(-20))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                self.t_end_manoeuvre_zigzag_10 = time.time()
                if covered_turn<-20.0 or covered_turn<340 and covered_turn>335:
                    self.manoeuvre_zigzag_20_phase = 2
            elif self.manoeuvre_zigzag_20_phase == 2:
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                if time.time() - self.t_end_manoeuvre_zigzag_20 > 5.0:
                    self.manoeuvre = None

            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            -float(ships_mat[2, 0, 3])], environment_variable)

    def manoeuvre_astern(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_astern = self.t_start_manoeuvre_astern + (time.time() - self.t_reg)

            self.t_reg = time.time()
            if self.t_start_manoeuvre_astern<2.0:
                #MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, 85, 100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, 85, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
            else:
                # MAIN STARBOARD THRUSTER
                for_send = self.nmea_make.TRC(2, -85, -100, int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # MAIN PORT THRUSTER
                for_send = self.nmea_make.TRC(1, -85, -100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # BOW THRUSTER
                for_send = self.nmea_make.TRC(0, 0, 100, 0)
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)

                # RUDDERS
                for_send = self.nmea_make.ROR(int(0), int(0))
                self.SendRPA3.send_string("recieved", for_send, ships_mat)
                write_topic(self.topic_name, for_send)
                if ships_mat[0,0,5] <2.0:
                    self.manoeuvre = None

            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            -float(ships_mat[2, 0, 3])], environment_variable)


    def BORKUM_tuner(self, commands, for_send, ships_mat, thrust_constant, heading_constant, att_heading_constant, manoeuvre):

        if manoeuvre!=None:
            self.manoeuvre = manoeuvre
            print(manoeuvre)

        if self.manoeuvre:
            environment_variable = 'RPA3' + '_' + self.manoeuvre + '_' + datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d')

        if self.manoeuvre==None:

            self.t_reg = time.time()
            #MAIN STARBOARD THRUSTER
            for_send = self.nmea_make.TRC(2, 85, 100, int(10))
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)

            # MAIN PORT THRUSTER
            for_send = self.nmea_make.TRC(1, 85, 100, 10)
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)

            # BOW THRUSTER
            for_send = self.nmea_make.TRC(0, 0, 100, 10)
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)

            # RUDDERS
            for_send = self.nmea_make.ROR(int(0), int(0))
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)
            self.heading_difference_circle_right = ships_mat[0, 0, 7]
            self.heading_difference_circle_left = ships_mat[0, 0, 7]
            self.heading_difference_zigzag_10 = ships_mat[0, 0, 7]
            self.heading_difference_zigzag_20 = ships_mat[0, 0, 7]


        if self.manoeuvre=='manoeuvre_circle_right':
            self.manoeuvre_circle_right(ships_mat, environment_variable)
        if self.manoeuvre=='manoeuvre_circle_left':
            self.manoeuvre_circle_left(ships_mat, environment_variable)
        if self.manoeuvre=='manoeuvre_zigzag_10':
            self.manoeuvre_zigzag_10(ships_mat, environment_variable)
        if self.manoeuvre=='manoeuvre_zigzag_20':
            self.manoeuvre_zigzag_20(ships_mat, environment_variable)
        if self.manoeuvre=='manoeuvre_astern':
            self.manoeuvre_astern(ships_mat, environment_variable)



        return (for_send, ships_mat)