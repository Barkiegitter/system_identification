#To Do: Try opencv dialation and then binary threshold
# https://towardsdatascience.com/computer-vision-for-beginners-part-2-29b3f9151874

#NavPath
# import utilities.astar
import os

import numpy                as np
import src.nav_tools        as nt


import matplotlib.pyplot    as plt
import logging
import src.logger as logger
import sys
import time
from datetime           import datetime
from numpy              import genfromtxt
from matplotlib         import path

import random
import json
# import cv2

# import cap_comm
from src.utilities import astar
# from src.communications.sender        import UDPSender as Sender
from src.communications.nmea_creator  import NMEAStrings
#I believe this one is needed to avoid the a_star.astar undefined error ?
from scipy                  import interpolate
from scipy.spatial          import KDTree
from scipy.spatial          import cKDTree
#from profiler               import profile

class NavPath(object):
    def __init__(self, settings):
        
        self.logging = False
        if self.logging == True:
            logger.worker_configurer(settings["LOGGER"]['log_q'])
            self.log = logging.getLogger("NavPath")
            self.log.info(f'NavPath: Initialized NavPath and sending to 7775')
        # cap_comm.write("Paths", "1,3,4 56")

        self.num_time_samples = int(settings['SHIP_MAT']["ships_mat_samples"])
        # self.SendEX = Sender(settings["IP_ADDRESSES"]['ip_send_atlantis'], settings["UDP_PORTS"]['port_send_atlantis'])
        self.nmea_send = NMEAStrings()
        filename = os.path.join(os.path.dirname(__file__), (settings["NAV_PATH"]['path_seed']))
        nav_map = genfromtxt(filename, delimiter=',')
        self.pts = nav_map[:, :4]
        #self.desired_resolution = 0.05 with self.distance2wpt = 3 works nice on PID 
        self.desired_resolution = 0.01
        self.distance2wpt = 20
        self.static_path = settings["NAV_PATH"]['force_static_path']
        # self.distance2wpt = 0.0001
        self.all_waypoints_alpha, actual_resolution_row, actual_resolution_col = self.auto_thicken_paths(self.pts, self.desired_resolution) # self.thicken_paths(density) 
        self.row_distance_m = actual_resolution_row * 1000 #Resolution of points in perpendicular to boat
        self.col_distance_gl = actual_resolution_col #low presision km to global equivelance
        self.response_time = 0.3 #This is how long the path generator should take. It traids off distance for speed
        self.lookback = 0
        self.iterpolation_space = 11 # 5 #This is best if its an odd number (It will choose to cross beween two boats in the middle)
        self.RIS = 40 # Row interpolation space

        # Initiate Local Logger
        if self.logging == True:
            logger.worker_configurer(settings['LOGGER']['log_q'])    # configure local logger with the correct settings
            self.log = logging.getLogger(name="Autopilot")
        #Things to change based on density
        
        self.path_decimation = settings["NAV_PATH"]['path_decimation']
        self.safty_distance = int(actual_resolution_row/nt.geo_to_meters(20, 20)) # 500 pts
        self.mass =  settings["NAV_PATH"]['ships_mass']
        if self.logging == True:
            self.log.info(f'NavPath = safety distance {self.safty_distance}')

        self.starting_value = 0 

        self.length = self.ending_value = self.all_waypoints_alpha.shape[0]
        self.total_path_length = self.all_waypoints_alpha.shape[0]
        self.half_width_alpha =  int(self.all_waypoints_alpha.shape[1]/2)

        #For changing the path length
        self.path_resizing_parameter = int(0.001*self.length)  #50 

        self.center_track = int(self.half_width_alpha/2)
        # self.total_path_length = self.ending_value
        self.create_waypoint_variables(self.starting_value, self.total_path_length)
        self.search_path = np.arange(0, self.half_width_alpha)
        self.my_path = self.center_track
        self.trigger = True #Center Path biase trigger
        self.old_my_path = self.my_path
        self.same_path_trigger_tick = nt.get_time()
        self.path4biase = 0

        self.tree = self.create_search_trees(self.all_waypoints_alpha)
        if self.logging == True:
            self.log.info("Created Tree")
        self.number_of_loops = 0
        self.box_lats_stacked = [[]]
        self.box_lngs_stacked = [[]]
        self.momentum = False

        #Number of points in a ship. Initialise variable

        # self.avg_distans_points = nt.distance_spherical()
        self.longest_ship = 0 
        self.mmsi_keep = 0 #This is a place to  keep the ships that are in the track

    def print_grid(self, objectgrid):
        if objectgrid.shape[0] == 0:
            for i in range(self.half_width):
                self.log.debug("##############################")
        else:
            self.log.debug(f'NavPath: This is search xy --- Start')
            for i in range(search_xy.shape[0]):
                self.log.debug(f'  {search_xy[i, ::4]}{i} ')
                self.log.debug(f'NavPath: This is search xy --- End')

    def show_search_space(self, search_xy, search_xy_map):
        search_xy_cv = np.zeros([self.half_width, search_xy.shape[1], 3], np.uint8)
        fraction = np.max(search_xy_map)

        search_xy_cv[:, :, 0] = search_xy*255
        search_xy_cv[:, :, 1] = (search_xy_map/fraction)*255
        search_xy_cv[:, :, 2] = 0

        toc = time.time()

        img = np.clip(search_xy_cv , 0, 255)
        img = cv2.resize(img, (500, 500))
        cv2.imwrite(f'searchrec/test{toc}.jpg', search_xy_cv)
        # cv2.imshow("image", img)
        self.log.info(f'NavPath: This is search xy --- End')

    def checkout_interpolations(self, system_wait):
        return(self.all_waypoints_alpha, self.half_width_alpha)

    def create_search_trees(self, all_waypoints):    
        trees = []
        for thread in range(self.half_width_alpha):
            x = all_waypoints[:, thread]
            y = all_waypoints[:, thread+self.half_width_alpha]
            tree_object = cKDTree(list(zip(x, y)), balanced_tree=False)
            # tree_object = KDTree(list(zip(x, y)))
            trees.append(tree_object)
        return(trees)

    def create_waypoint_variables(self, starting_value, ending_value):
        self.starting_value = starting_value
        self.ending_value = ending_value
        # self.waypoints = self.all_waypoints_alpha[starting_value:ending_value, :]
        # self.ending_value = starting_value + ending_value
        self.waypoints = self.all_waypoints_alpha[self.starting_value:self.ending_value, :]
        self.length = len(self.waypoints[:, 0])
        self.width  = int(self.waypoints.shape[1]) 
        self.half_width = int(self.waypoints.shape[1]/2)
        self.path_reduction_rate = self.length//100
        # self.log.info(f'NavPath: In create_waypoint_variables starting value is {self.starting_value} ending_value is {self.ending_value} length is {self.length}')
        self.lat_mat = np.zeros((self.width, self.length))
        self.lng_mat = np.zeros((self.width, self.length))

    def thicken_rows(self, pts):  #Interpolates all the values inbetween rows
        ptsx2 = np.zeros((pts.shape[0]*2, pts.shape[1]))
        ptsx2[1:-1:2, :] = np.divide((pts[0:-1:1, :]+ pts[1::1, :]), 2)
        ptsx2[::2, :] = pts
        return(ptsx2[:-1][:])

    def thicken_columns(self, pts): #Interpolates all the values inbetween columns with form |lat|lat*|lat|lon|lon*|lon|
        ptsy2      = np.zeros((pts.shape[0], pts.shape[1]*2-1))
        lats_end   = int(pts.shape[1])
        pts_middle = int(pts.shape[1]/2)
        ptsy2[:,    :lats_end:2] = pts[:, :pts_middle]
        ptsy2[:,  lats_end-1::2] = pts[:, pts_middle:]
        ptsy2[:, 1:lats_end-1:2] = np.divide((pts[:, 0:pts_middle-1:1]+ pts[:,  1:pts_middle:1]), 2)
        ptsy2[:,  lats_end:-1:2] = np.divide((pts[:,  pts_middle:-1:1]+ pts[:, pts_middle+1::1]), 2)
        return(ptsy2[:, :-1])

    def thicken_paths(self, density):
        for i in range(density):
            self.pts = self.thicken_columns(self.thicken_rows(self.pts))
        return(self.pts)

    def auto_thicken_paths(self, pts, desired_resolution):
        if self.logging == True:
            self.log.info(f'NavPath pts shape {pts.shape}')
        distance_col = nt.distance_spherical(pts[:, 0], pts[:, 2], pts[:, 1], pts[:, 2])
        distance_row = nt.distance_spherical(pts[1::2, 1], pts[1::2, 3], pts[:-1:2, 1], pts[:-1:2, 3])
        if self.logging == True:
            self.log.info(f"NavPath: The aproximate distance to destination = numpy.sum distance_row {np.sum(distance_row)} km ")

        while np.max(distance_col) > desired_resolution:
            if self.logging == True:
                self.log.info("Generating cols")
            self.pts = self.thicken_columns(self.pts)
            distance_col = nt.distance_spherical(self.pts[:, 0], self.pts[:, 2], self.pts[:, 1], self.pts[:, 3])
            if self.logging == True:
                self.log.info(f'max cols = {np.max(distance_col)} mean cols = {np.mean(distance_col)}')
        if self.logging == True:
            self.log.info("Done Generating cols")

        while np.max(distance_row) > desired_resolution:
            if self.logging == True:
                self.log.info("Generating rows")
            self.pts = self.thicken_rows(self.pts)
            distance_row = nt.distance_spherical(self.pts[1::2, 0], self.pts[1::2, 2], self.pts[:-1:2, 0], self.pts[:-1:2, 2])
            if self.logging == True:
                self.log.info(f'max rows = {np.max(distance_row)} mean rows = {np.mean(distance_row)}')
        
        if self.logging == True:
            self.log.info(f'NavPath pts shape {self.pts.shape}')
        
        return(self.pts, np.mean(distance_row), np.mean(distance_col))

    def skip_astar(self, my_path):
        # self.log.info("No objects in path")
        i = int(self.my_path)
        lat_mat = self.waypoints[:, i] #
        lng_mat = self.waypoints[:, i+self.half_width] #
        the_final_path =  np.array([lng_mat, lat_mat])
        return(the_final_path, 1) #my_path)# self.length)
 

    def Shorten_Path(self): #Functions in a class start with lower case
        if self.length-self.path_resizing_parameter > self.path_resizing_parameter:
            # self.log.info("Two")
            # self.log.info(f'NavPath: Making path shorter with ')
            # self.length = int(self.length/2)
            self.length = self.length - self.path_resizing_parameter 
            # self.log.info("Three")
            #self.log.info(f'NavPath: Making path shorter with {self.length}')
            self.create_waypoint_variables(self.starting_value, self.starting_value+self.length)
            # self.log.info("Four")


    def Lengthen_Path(self): #Functions in a class start with lower case
        if self.length+self.path_resizing_parameter < self.total_path_length:
            #self.log.info(f'NavPath: Making path longer with {self.length}')
            self.length = self.length +  self.path_resizing_parameter 
            self.create_waypoint_variables(self.starting_value, self.starting_value+self.length)
    
    #Path length is directly related to path generation time. This function modulates the lenght 
    # of the path to meet performance requirments. 

    def path_preformance_modulation(self, toc, tic):
        delta_time = toc-tic
        # self.log.info(f"nav_path : perfomance modulation this is the delta_time {delta_time}")

        if delta_time > self.response_time:
            self.Shorten_Path()
        elif delta_time < self.response_time:
            self.Lengthen_Path()


    def Try_Again(self, box_lats_stacked, box_lngs_stacked, my_path): #Functions in a class start with lower case
        if self.length > 200:
            self.Shorten_Path()
            return(self.path_generator(box_lats_stacked, box_lngs_stacked, self.my_path))
        else:
            # self.log.info(f'NavPath: Sending a no path signal  ---------------- Dead Path')
            lat_mat = self.waypoints[:20, self.my_path] #pnt_grid
            lng_mat = self.waypoints[:20, self.my_path+self.half_width] #
            the_final_path =  np.array([lng_mat, lat_mat])
            return(the_final_path, 1)


    def No_Path(self, box_lats_stacked, box_lngs_stacked):  #Functions in a class start with lower case
        lat_mat = self.waypoints[:self.distance2wpt, self.my_path] #
        lng_mat = self.waypoints[:self.distance2wpt, self.my_path+self.half_width] #
        the_final_path =  np.array([lng_mat, lat_mat])
        if self.logging == True:
            self.log.info("nav_path : #### Warning! Warning! Warning! Warning! Warning! ####")
        return(the_final_path, 1)


    def gen_path_search_array(self):
        if self.my_path == 0:
            self.search_path = np.arange(0, self.half_width_alpha)
            return(self.search_path)
            
        elif self.my_path == len(self.search_path)-1:
            self.search_path = np.arange(0, self.half_width_alpha)
            self.search_path = self.search_path[::-1]
            return(self.search_path)

        #Randomise search sequence start direction
        # coin_flip = bool(random.getrandbits(1))
        # if coin_flip == True:
        if self.my_path < self.half_width_alpha:
            on_left = True
        else:
            on_left = False
    
        if on_left == True:
            inc = 0
            while self.my_path - inc/2 >= 0 and self.my_path + inc/2 < (self.half_width_alpha):
                if inc % 2 == 0:
                    self.search_path[inc] = self.my_path + inc/2
                else:
                    self.search_path[inc] = self.my_path - inc/2
                # self.log.info(f'inc {inc} {inc % 2} {inc/2} {self.search_path[inc]} {coin_flip}')
                inc = inc+1
            return(self.search_path)
        else:
            inc = 0
            inc_new = 0
            while self.my_path - (inc/2 - 1) >= 0 and self.my_path + (inc/2 + 1) < (self.half_width_alpha):
                if inc % 2 == 0:
                    self.search_path[inc] = self.my_path + inc_new//2
                else:
                    self.search_path[inc] = self.my_path - inc_new//2
                # self.log.info(f'inc {inc} {inc % 2} {inc_new/2} {self.search_path[inc]} {coin_flip}')
                inc = inc+1
                inc_new = inc_new - 1

            return(self.search_path)


    #ToDo: encurage center of path
    def Position2Path2(self, lat, lng, center = False): #Functions in a class start with lower case
        for size in range(1, 3):  #1):
            r = self.gen_path_search_array()
            # self.log.info(f"nav_path:/----- This is the Search Path {r} -------/")
            # self.log.info(f"nav_path : {r}")

            for my_path in r:
                # itteration = self.tree[my_path].query_ball_point([lat, lng], r = (self.distance2track * size))
                # self.log.info(f"nav_path: Radius for ego detection {self.col_distance_gl}")
                itteration = self.tree[my_path].query_ball_point([lat, lng], r = ((self.col_distance_gl/100)*size))
                # self.log.info(f"NavPath : this is my_path {my_path} itteration {itteration} {size}")

                if len(itteration) > 0:
                    if len(itteration) > 2:
                        median_value = int(np.median(itteration))
                        
                    else:
                        median_value = itteration[0]

                    if  median_value-self.lookback > 0:
                        # self.log.info(f"nav_path : Updated position ###################################### {r}")
                        # self.create_variables(self.pts, itteration[0]-self.lookback)
                        self.starting_value = median_value-self.lookback

                        # self.create_variables(self.all_waypoints_alpha, itteration[0]-self.lookback, self.total_path_length)
                        # biasing the center of the path
                        #check to see if the path changed
                        if self.old_my_path != my_path  and self.old_my_path != self.path4biase:
                            self.same_path_trigger_tick = nt.get_time()
                            
                            # self.trigger = True
                            # self.log.info(f"navpath ln 303: moved path with path value {my_path} and time {self.same_path_trigger_tick}")
                            # self.old_my_path = my_path

                        #if the path is the same check to see how long it has been the same
                        elif self.old_my_path == my_path:
                            if my_path != self.path4biase:
                                time_to_move2center = nt.get_time() - self.same_path_trigger_tick 
                                if time_to_move2center > 2:
                                    self.biase_path = True
                                else:
                                    self.biase_path = False
                            else:
                                self.biase_path = False
                            # self.log.info(f"nav_path = time to move to center? -----------{my_path}-----------{time_to_move2center}----------")
                            if self.biase_path == True:
                                self.path4biase = my_path
                                if my_path < self.center_track:  
                                    my_path = my_path + 1
                                    # self.log.info(f"nav_path = time to move to center {my_path + 1} from {my_path} to get to {self.center_track}, Biasing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                                elif my_path > self.center_track:
                                    my_path = my_path - 1
                                    # self.log.info(f"nav_path = time to move to center {my_path - 1} from {my_path} to get to {self.center_track}, Biasing!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        
                        self.old_my_path = my_path

                        if center == True:
                            self.path = self.half_width_alpha//2
                            # self.log.info(f"nav_path: center of path {self.path}")
                        else:
                            self.my_path = my_path

                        self.create_waypoint_variables(self.starting_value, self.starting_value+self.length)
                        # return(itteration[0]-self.lookback, i)
                        # self.gen_path_search_array()
                        return(my_path)

                    self.starting_value =  median_value

                    if center == True:
                        self.path = self.half_width_alpha//2
                        # self.log.info(f"nav_path: center of path {self.path} ")
                    else:
                        self.my_path = my_path

                    self.create_waypoint_variables(self.starting_value , self.starting_value+self.length)

                    
                    # self.gen_path_search_array()
                    # return(my_path)
        # self.log.info(f"nav_path : too far from path")
        return(self.my_path)


    # def boxes_in_track(self, box_lats, box_lngs, path_pts): 
    #     box = np.array((box_lats, box_lngs))
    #     path_space = path.Path(box.T, closed=True)
    #     inpath = path_space.contains_points(path_pts.T, radius = 0.00005) #0.00005
    #     itteration = np.where(inpath[:] != False)
    #     return(itteration)
    
    def update_pts_mat(self, the_final_path, ships_mat):
        for time_sample in range(1, self.num_time_samples):
            if the_final_path.shape[1] > 80: 
                # self.log.debug(f"time_sample {time_sample}")
                if time_sample+1 * self.distance2wpt < the_final_path.shape[1]:
                # if time_sample+1 * self.distance2wpt <= 23:
                    # self.log.debug(f"Passed")
                    ships_mat[time_sample, 0, [2, 3]] = the_final_path[[1, 0], self.distance2wpt*time_sample] # * int(waypoint_of_interest)]
                # else:
                #     self.log.debug(f"Fail the_final_path.shape[1] {the_final_path.shape[1]}")

        return(ships_mat)


    # @profile
    def GeneratePaths(self, recieved, for_send, ships_mat): #Functions in a class start with lower case

        tic = time.time()
        self.number_of_loops = 0
        #Check to see if vessel is within bounds of search space to search location on path
        self.path  = self.Position2Path2(ships_mat[0, 0, 2], ships_mat[0, 0, 3], self.static_path) #swapped lat and long

        #Get all the ships last recieved input and thier predicted position
        # num_ships =  np.where(ships_mat[-2:, 1:, 1] != 0)


        the_final_path, go_or_no = self.skip_astar(self.my_path)
        ships_mat2 = self.update_pts_mat(the_final_path, ships_mat)
        ships_mat = ships_mat2
        # return(for_send, ships_mat)

        #Save previous waypoint
        ships_mat[0, 0, [14, 15]] = ships_mat[1, 0, [2, 3]] #Backup previous waypoint for cross track error calc
        ships_mat[0, 0, 16      ] = ships_mat[0, 0, 5     ] #Backup previous speed
        ships_mat[0, 0, 17      ] = ships_mat[0, 0, 0     ] #Backup previous time

        #Decimate path and send it
        delta_path = np.where(the_final_path[0] != 0)[0]
        ships_mat = self.update_pts_mat(the_final_path, ships_mat)

        if delta_path.size != 0:
    
            dpath = delta_path[:-self.path_decimation:self.path_decimation]

            # waypoint_number = 0

            toc = time.time()

            #Simple Momentum Consideration
            if self.momentum == True:

                angle2waypoint = nt.heading_angle_between_points(ships_mat[1, 0, 2], ships_mat[1, 0, 3] ,the_final_path[1, 1], the_final_path[0, 1])
                # waypoint_of_interest = ships_mat[0, 0, 5] * np.abs(np.cos(angle2waypoint))

            for_send = self.nmea_send.WPT_ARRAY(the_final_path[0, dpath],  the_final_path[1, dpath])
            # for_send = self.nmea_send.WPT_ARRAY(ships_mat[:, 0, 3] ,  ships_mat[:, 0, 2])
            # self.log.info(the_final_path[0, dpath])
            # self.log.info("!!!!!!!!!!!!!!!!", for_send)

            # self.SendEX.send_string(recieved, for_send, ships_mat)
        else:
            if self.logging == True:
                self.log.error(f"NavPath ln 763: error unexpected else")
            else:
                pass

        toc = time.time()
        self.path_preformance_modulation(tic, toc)

        return(for_send, ships_mat)
