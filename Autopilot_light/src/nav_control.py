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
        self.rudder_pid = PID(0.8,0.000001,100.0) #RPA Sim Settings  only rudder: PID(1.,0.000001,15.0)
        self.rudder_array = np.zeros(20)
        self.throttle_pid = PID(25.0,0.5,0.01)       #2.0 0.01 1.0 #PID(5.0,0.2,0.02)
        self.throttle_array = np.zeros(20)


        self.rudder_front_pid = PID(0.2,0.000001,20.0)



        # self.rudder_pid = PID( 4.5, 4.3, 2.1)
        # self.rudder_pid = PID( 4.5, 4.3, 2.05) # try this
        # self.Borkum_type2_pid = PID( 11.0, 6.74, 9.55)
        # self.Borkum_type2_pid = PID( 11.1, 12.5, 11.5)

        # self.Borkum_type2_pid = PID( 3.75, 3.65, 2.8)
        self.Borkum_type2_pid = PID(3.75, 3.65, 2.95)

        # self.Borkum_type2_pid_flip = PID( 3.75, 3.65, 2.8)
        # self.Borkum_type2_pid = PID( 3.9, 0.8, 1)
        # self.Borkum_type2_pid = PID( 3.9, 0.8, 1)

        # self.Borkum_type2_pid = PID(29.4, 28.3, 21.0) #Not sure

        self.NoPts = True  # this is to deal with the case that the vessel is farther than the waypoint detection range

        # Waypoint radius detection
        self.radius = 0.001
        self.rightProp = 100
        self.leftProp = 100
        # rudder_servo = 90.0
        # rudder_nautis = 0.0
        recieved = 0
        ships_mat = 0
        self.default_throttled =  70

        self.old_heading = 0.0
        # self.count = 0

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




        self.travelled_heading_degrees = 0.0
        self.heading_difference = 0.0

        self.check_circle_manouevre_right = 0

        self.t_start_manoeuvre = time.time()

        self.initial_manoeuvre = 'none'
        self.t_start_manoeuvre_circle_right = 0.0
    # self.throttle_test()
    # self.SimpRudder_test()
    # time.sleep(1)

    def SimpRudderPID_RPA(self, recieved, for_send, ships_mat):  # Functions in a class start with lower case
        if time.time() < self.iter_time + 0.2:
            # self.log.debug(f'skipping controle {time.time()}')
            return (for_send, ships_mat)

        self.iter_time = time.time()

        lat = ships_mat[0, 0, 2]
        lng = ships_mat[0, 0, 3]
        pathLats = ships_mat[1:, 0, 2]
        pathLngs = ships_mat[1:, 0, 3]
        heading  = ships_mat[0, 0, 4]

        zerolats = np.where(pathLats == 0)[0]
        if len(zerolats) > 0:
            # self.log.debug(f"NavControl: No waypoints found {ships_mat[1:, 0, 2]}")
            return (for_send, ships_mat)

        # # self.log.debug(f'nav_control: lat {lat} lng {lng} next_waypoint_x {next_waypoint_y} next_waypoint_y {next_waypoint_x}')
        angle2waypoint = nt.heading_angle_between_points(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])
        # distance2waypoint = nt.distance(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])

        self.rudder_pid.setPoint(angle2waypoint)

        pid_sp = self.rudder_pid.update(heading)

        if angle2waypoint < 0:
            angle2waypoint = (360 + angle2waypoint)

        map_setpoint = angle2waypoint - heading

        if map_setpoint < 0:
            map_setpoint = 360 + map_setpoint

        if map_setpoint > 180:
            map_setpoint = - 360 + map_setpoint

        # self.simple_throttle(map_setpoint)
        self.static_throttle()

        # self.log.debug(f'nav_control: heading {heading} map_setpoint {map_setpoint} angle2waypoint {angle2waypoint} distance2waypoint {distance2waypoint}')

        if pid_sp < 180 and pid_sp > -180:
            rudder_servo = nt.map_gk(pid_sp, -180, 180, 40, 140)
            rudder_nautis = nt.map_gk(pid_sp, -180, 180, -35, 35)

            for_send = self.nmea_make.ROR(round(rudder_nautis, 1), round(rudder_nautis, 1))
            self.SendRPA3.send_string(recieved, for_send, ships_mat)
            for_send = self.nmea_make.RSA(round(rudder_nautis, 1), round(rudder_nautis, 1))
            self.SendAtln.send_string(recieved, for_send, ships_mat)

            for_send = self.nmea_make.ROR(round(rudder_servo, 2), round(rudder_servo, 2))  # +ranint)
            self.SendArduino.send_string(recieved, for_send, ships_mat)
        # self.log.debug(f"NavControl: hdg {heading} setp {pid_sp} angle2wp {angle2waypoint} distance2wp {distance2waypoint} rudder_servo {rudder_servo}")
        # self.log.debug(f"NavControl: Rudder casual {for_send}")

        if pid_sp < -180 or pid_sp > 180:

            rudder_nautis = np.sign(pid_sp) * 35
            if np.sign(pid_sp) == -1:
                rudder_servo = 40
            if np.sign(pid_sp) == 1:
                rudder_servo = 140

            # self.log.debug(f"black magic {rudder_servo}")

            for_send = self.nmea_make.ROR(round(rudder_nautis, 1), round(rudder_nautis, 1))
            self.SendRPA3.send_string(recieved, for_send, ships_mat)
            for_send = self.nmea_make.RSA(round(rudder_nautis, 1), round(rudder_nautis, 1))
            self.SendAtln.send_string(recieved, for_send, ships_mat)
            self.SendAtln.send_string(recieved, for_send, ships_mat)

            for_send = self.nmea_make.ROR(round(rudder_servo, 1), round(rudder_servo, 1))
            self.SendArduino.send_string(recieved, for_send, ships_mat)
            # self.log.debug(f"nav_control: hdg {heading} setp {pid_sp} angle2wp {angle2waypoint} distance2wp {distance2waypoint} rudder_servo {rudder_servo}")
            # self.log.debug(f"nav_control: Rudder casual {for_send}")
            return (for_send, ships_mat)

        return (for_send, ships_mat)

    def SimpleThrottlePID(self, recieved, for_send, ships_mat):
        if time.time() < self.iter_time + 0.2:
            # self.log.debug(f'skipping controle {time.time()}')
            return (for_send, ships_mat)

        return (for_send, ships_mat)
    

    def SimpRudderPID_BORKUM(self, recieved, for_send, ships_mat):  # Functions in a class start with lower case
        self.log.debug("SimpRudderPID_BORKUM")
        if time.time() < self.iter_time + 0.2:
            # self.log.debug(f'skipping controle {time.time()}')
            return (for_send, ships_mat)

        # self.iter_time = time.time()

        lat = ships_mat[0, 0, 2]
        lng = ships_mat[0, 0, 3]
        heading = ships_mat[0, 0, 4]

        angle2waypoint = nt.heading_angle_between_points(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])

        self.rudder_pid.setPoint(angle2waypoint)
        pid_sp = self.rudder_pid.update(heading)

        if angle2waypoint < 0:
            angle2waypoint = (360 + angle2waypoint)

        map_setpoint = angle2waypoint - heading

        if map_setpoint < 0:
            map_setpoint = 360 + map_setpoint

        if map_setpoint > 180:
            map_setpoint = - 360 + map_setpoint

        self.log.debug(f'SIMPLE nav_control: heading {heading} map_setpoint {map_setpoint} angle2waypoint {angle2waypoint} ')
        if pid_sp < 180 and pid_sp > -180:
            rudder_nautis = nt.map_gk(pid_sp, 180, -180, -35, 35)

            for_send = self.nmea_make.TRC(0, self.default_throttled, 100, rudder_nautis.astype('int'))
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(1, self.default_throttled, 100, 0)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(2, self.default_throttled, 100, 0)
            write_topic(self.topic_name, for_send)
            return (for_send, ships_mat)

        if pid_sp < -180 or pid_sp > 180:
            rudder_nautis = np.sign(pid_sp) * 35 * (-1)

            for_send = self.nmea_make.TRC(0, self.default_throttled, 50, rudder_nautis.astype('int'))
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(1, self.default_throttled, 30, 0)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(2, self.default_throttled, 10, 0)
            write_topic(self.topic_name, for_send)

            return (for_send, ships_mat)

        return (for_send, ships_mat)
    
    def fx_test(self, recieved, for_send, ships_mat):
        self.log.debug(f'nav_control: depth {ships_mat[0, 0, 0]}')
        return (for_send, ships_mat)


    def type2_BORKUM(self, recieved, for_send, ships_mat):  # Functions in a class start with lower case
        # self.Borkum_type2_pid = PID( 3.2, 2.6, 2.3) # Front range is 50 and back range is
        if time.time() < self.iter_time + 0.2:
            # self.log.debug(f'skipping controle {time.time()}')
            return (for_send, ships_mat)

        self.iter_time = time.time()

        lat = ships_mat[0, 0, 2]
        lng = ships_mat[0, 0, 3]
        pathLats = ships_mat[1:, 0, 2]
        pathLngs = ships_mat[1:, 0, 3]
        heading = ships_mat[0, 0, 4]

        zerolats = np.where(pathLats == 0)[0]
        # if len(zerolats) > 0:
        #     self.log.debug("NavControl: No waypoints found")
        #     return (for_send, ships_mat)

        angle2waypoint = nt.heading_angle_between_points(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])
        # distance2waypoint = nt.distance(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])

        self.Borkum_type2_pid.setPoint(angle2waypoint)
        pid_sp = self.Borkum_type2_pid.update(heading)

        if angle2waypoint < 0:
            angle2waypoint = (360 + angle2waypoint)

        map_setpoint = angle2waypoint - heading

        if map_setpoint < 0:
            map_setpoint = 360 + map_setpoint

        if map_setpoint > 180:
            map_setpoint = - 360 + map_setpoint

        self.log.debug(f'nav_control: heading {heading} map_setpoint {map_setpoint} angle2waypoint {angle2waypoint}, pid_sp={pid_sp}')

        if pid_sp < 180 and pid_sp > -180:
            # rudder_nautis= nt.map_gk(pid_sp, -180, 180, 145, 210)
            # rudder_nautis= nt.map_gk(pid_sp, 180, -180, -35, 35)
            rudder_back = nt.map_gk(pid_sp, -180, 180, -50, 50)
            rudder_front = nt.map_gk(pid_sp, 180, -180, -33, 33)

            for_send = self.nmea_make.TRC(0, 100, 100, rudder_back)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(1, 100, 100, rudder_front)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(2, 100, 100, rudder_front)
            write_topic(self.topic_name, for_send)

        if pid_sp < -180 or pid_sp > 180:
            # rudder_back = np.sign(pid_sp) * 50 * (-1)
            rudder_back = np.sign(pid_sp) * 50
            rudder_front = np.sign(pid_sp) * 33
            # rudder_nautis = rudder_nautis

            for_send = self.nmea_make.TRC(0, 100, 50, rudder_back)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(1, 100, 50, rudder_front)
            write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(2, 100, 50, rudder_front)
            write_topic(self.topic_name, for_send)

        self.log.debug("For SEND: ", for_send)
        # self.log.debug(for_send)
        return (for_send, ships_mat)

    def type2_BORKUM_flip(self, recieved, for_send, ships_mat):  # Functions in a class start with lower case
        # self.Borkum_type2_pid = PID( 3.2, 2.6, 2.3) #Front range is 50 and back range is
        if time.time() < self.iter_time + 0.2:
            # self.log.debug(f'skipping controle {time.time()}')
            return (for_send, ships_mat)

        self.iter_time = time.time()

        lat = ships_mat[0, 0, 2]
        lng = ships_mat[0, 0, 3]
        pathLats = ships_mat[1:, 0, 2]
        # self.log.debug(pathLats)
        pathLngs = ships_mat[1:, 0, 3]
        heading = ships_mat[0, 0, 4] + 180

        zerolats = np.where(pathLats == 0)[0]
        # if len(zerolats) > 0:
        #     self.log.debug("NavControl: No waypoints found")
        #     return (for_send, ships_mat)

        angle2waypoint = nt.heading_angle_between_points(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])
        # distance2waypoint = nt.distance(lat, lng, ships_mat[1, 0, 2], ships_mat[1, 0, 3])

        self.Borkum_type2_pid.setPoint(angle2waypoint)
        pid_sp = self.Borkum_type2_pid.update(heading)

        if angle2waypoint < 0:
            angle2waypoint = (360 + angle2waypoint)

        map_setpoint = angle2waypoint - heading

        if map_setpoint < 0:
            map_setpoint = 360 + map_setpoint

        if map_setpoint > 180:
            map_setpoint = - 360 + map_setpoint

        # self.log.debug(f'nav_control: heading {heading} map_setpoint {map_setpoint} angle2waypoint {angle2waypoint}')

        if pid_sp < 180 and pid_sp > -180:
            # rudder_nautis= nt.map_gk(pid_sp, -180, 180, 145, 210)
            # rudder_nautis= nt.map_gk(pid_sp, 180, -180, -35, 35)
            rudder_back = nt.map_gk(pid_sp, -180, 180, -50, 50)
            rudder_front = nt.map_gk(pid_sp, 180, -180, -33, 33)

            for_send = self.nmea_make.TRC(0, 100, 100, rudder_back + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)

            for_send = self.nmea_make.TRC(1, 100, 100, rudder_front + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)

            for_send = self.nmea_make.TRC(2, 100, 100, rudder_front + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)
            return (for_send, ships_mat)

        if pid_sp < -180 or pid_sp > 180:
            rudder_back = np.sign(pid_sp) * 50
            rudder_front = np.sign(pid_sp) * 33 * (-1)
            # rudder_nautis = rudder_nautis

            for_send = self.nmea_make.TRC(0, 100, 50, rudder_back + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)
            # write_topic(self.topic_name, for_send)

            for_send = self.nmea_make.TRC(1, 100, 50, rudder_front + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)

            for_send = self.nmea_make.TRC(2, 100, 50, rudder_front + 180)
            try:
                write_topic(self.topic_name, for_send)
            except Exception as e:
                print(e)

            return (for_send, ships_mat)

        return (for_send, ships_mat)

    def simple_throttle(self, map_setpoint):
        recieved = 0
        ships_mat = 0

        if map_setpoint < 0:
            self.leftProp = 100
            self.rightProp = nt.map_gk(map_setpoint, 0, -180, self.leftProp.astype('int'), 100)
            if self.SendTRC == True:
                for_send = self.nmea_make.TRC(2, self.rightProp, 100, 0)

                for_send = self.nmea_make.TRC(1, self.leftProp, 100, 0)

        if map_setpoint > 0:
            self.rightProp = 100
            self.leftProp = nt.map_gk(map_setpoint, 0, 180, self.rightProp.astype('int'), 100)

            if self.SendTRC == True:
                for_send = self.nmea_make.TRC(1, self.leftProp, 100, 0)
                self.SendRPA3.send_string(recieved, for_send, ships_mat)

                for_send = self.nmea_make.TRC(2, self.rightProp, 100, 0)
                self.SendRPA3.send_string(recieved, for_send, ships_mat)

    def static_throttle(self):
        recieved = 0
        ships_mat = 0
        for_send = self.nmea_make.TRC(2, self.rightProp, 100, 0)
        self.SendRPA3.send_string(recieved, for_send, ships_mat)

        for_send = self.nmea_make.TRC(1, self.leftProp, 100, 0)
        self.SendRPA3.send_string(recieved, for_send, ships_mat)

    def SimpRudder_test(self):
        recieved = 0
        ships_mat = 0
        for i in range(179):
            self.log.debug(f'random_int {i}')
            for_send = self.nmea_make.ROR(i, i + 1)
            self.log.debug(f"nav_control: Rudder casual {for_send}")
            self.SendRPA3.send_string(recieved, for_send, ships_mat)
            self.SendArduino.send_string(recieved, for_send, ships_mat)
            time.sleep(0.25)

    # return(for_send, ships_mat)

    def throttle_test(self):
        recieved = 0
        ships_mat = 0

        for i in range(30, 80):
            self.log.debug(f'Throttle test value = {i}')
            for_send = self.nmea_make.TRC(0, i, 100, 0)
            self.SendRPA3.send_string(recieved, for_send, ships_mat)

            for_send = self.nmea_make.TRC(1, i, 100, 0)
            self.SendRPA3.send_string(recieved, for_send, ships_mat)

            time.sleep(0.1)






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
            print(ships_mat[2,0,3])
            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference
            # print(ships_mat[0,0,7], self.heading_difference)
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
            elif covered_turn>=-10 and covered_turn<=-5 or abs(covered_turn)>350 or self.check_circle_manouevre_right==1 :
                self.check_circle_manouevre_right = 1
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


            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            float(ships_mat[2, 0, 3])], environment_variable)

            # self.heading_difference = ships_mat[0, 0, 7]
    def manoeuvre_circle_right(self, ships_mat, environment_variable):   #environment_variable consists out of ship manoeuvre and data

        #initialize and capture current attitude
        if time.time() - self.t_reg >0.2:
            self.t_start_manoeuvre_circle_right = self.t_start_manoeuvre_circle_right + (time.time() - self.t_reg)
            print(ships_mat[2,0,3])
            self.t_reg = time.time()
            covered_turn = ships_mat[0,0,7] - self.heading_difference
            # print(ships_mat[0,0,7], self.heading_difference)
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
            elif covered_turn>=-10 and covered_turn<=-5 or abs(covered_turn)>350 or self.check_circle_manouevre_right==1 :
                self.check_circle_manouevre_right = 1
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

                #start another timer


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


            self.write_csv([time.time(), ships_mat[0, 0, 2], ships_mat[0, 0, 3], ships_mat[0, 0, 4], ships_mat[2, 0, 2],
                            float(ships_mat[2, 0, 3])], environment_variable)

    def only_throttle_rudder(self, ships_mat, thrust_constant, heading_constant, att_heading_constant):
        heading = ships_mat[0, 0, 7]
        att_heading = ships_mat[0, 0, 4]
        att_heading = -1*(heading - att_heading)

        # print(heading, att_heading)
        #time, lat, lon, heading, rpm, rsa
        # print(1, ships_mat[2, 0, 3])
        self.write_csv([time.time(), ships_mat[0,0,2], ships_mat[0,0,3],  ships_mat[0,0,7], ships_mat[2,0,2],  float(ships_mat[2,0,3])])      #lat lon 2, 3 rpm rudder angle
        att_heading = np.sign(att_heading) * (360%att_heading)
        pid_sp = self.rudder_pid.update_rudder(heading)
        pid_sp_front = self.rudder_front_pid.update_rudder(att_heading)

        # print(att_heading, pid_sp_front)
        # if pid_sp < 180 and pid_sp > -180:
        #     rudder_nautis = nt.map_gk(pid_sp, 180, -180, -35, 35)

        if pid_sp>75.0 :
            pid_sp=75.0
        if pid_sp<-75.0:
            pid_sp = -75.0
        if pid_sp < -180 or pid_sp > 180:
            pid_sp = np.sign(pid_sp) * 75.0 #* (-1)

        if pid_sp_front>75.0 :
            pid_sp_front=75.0
        if pid_sp_front<-75.0:
            pid_sp_front = -75.0
        if pid_sp_front < -180 or pid_sp_front > 180:
            pid_sp_front = np.sign(pid_sp_front) * 75.0 #* (-1)

        rudder_nautis_front = pid_sp_front
        rudder_nautis = pid_sp
        new_throttle = self.throttle_pid.update_speed(ships_mat[0, 0, 5])
        # print(ships_mat[0, 0, 5])
        new_throttle = self.throttle_avenger(new_throttle)
        rudder_nautis = self.rudder_avenger(rudder_nautis)

        # print(rudder_nautis, ships_mat[0,0,4])
        # new_throttle = 60
        if new_throttle>100.0:
            new_throttle = 100.0


        self.rightProp = 100


        ################ change here
        rudder_nautis_front = 0

        # self.count  += 1
        if time.time() - self.t_reg >0.2:
            # self.write_csv([heading, att_heading])
            # print(self.count)
            # self.count = 0
            self.t_reg = time.time()
            for_send = self.nmea_make.TRC(2, 0, -100, int(10))  # changed here
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)

            self.leftProp = 100
            for_send = self.nmea_make.TRC(1, 80, 100, 10) # changed here  + 0.5*rudder_nautis
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)
            self.backProp = 100
            for_send = self.nmea_make.TRC(0, 0, 100,10 ) #.astype('int')  -1*(int(rudder_nautis-rudder_nautis_front))
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)


            for_send = self.nmea_make.ROR(int(10), int(10))
            self.SendRPA3.send_string("recieved", for_send, ships_mat)
            write_topic(self.topic_name, for_send)



    def BORKUM_tuner(self, commands, for_send, ships_mat, thrust_constant, heading_constant, att_heading_constant, manoeuvre):

        if manoeuvre:
            environment_variable = 'RPA3' + '_' + manoeuvre + '_' + datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d')

        # print(self.t_start_manoeuvre_circle_right)
        if manoeuvre=='manoeuvre_circle_right':
            self.manoeuvre_circle_right(ships_mat, environment_variable)

        elif manoeuvre==None:
            self.t_start_manoeuvre_circle_right = 0
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
            self.heading_difference = ships_mat[0, 0, 7]

        return (for_send, ships_mat)