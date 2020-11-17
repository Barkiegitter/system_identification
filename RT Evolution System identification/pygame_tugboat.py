#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 10 13:39:01 2020

@author: erwinlodder
"""
import pygame
import manoeuvre_model_evo
import time
import pandas as pd
import numpy as np
from manoeuvre_model_evo import ship_model
from ship_class import ship
# ship = ship()

df_main = pd.read_csv('test_1_large.csv', sep=',')
coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')
acc_lim = np.genfromtxt('acc_limits.csv', delimiter=',')

df_main = df_main[20:-500]
u, v, r, hdg = df_main.loc[df_main.index[0], 'u'],df_main.loc[df_main.index[0], 'v'], df_main.loc[df_main.index[0], 'r'], df_main.loc[df_main.index[0], 'hdg']
# ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'], ship, coef_)

df_input = df_main[['rsa_0', 'rsa_1', 'rsa_2', 'rpm_0', 'rpm_1', 'rpm_2']]

class Kernel:
    def __init__(self, settings, ship, ship_model, coef_, acc_lim):
        pygame.init()
        self.screen_size = (1000, 1000) #width height
        self.screen = pygame.display.set_mode(self.screen_size)
        self.bg_color = (140, 153, 173)
        pygame.display.set_caption('tugboat control')
        
        self.scale_image = (114,260)
        self.pixel_meter_ratio = 3.0

        self.spawn_location = (500,500)

        self.tugboat_img = pygame.image.load('tugboat.png')
        self.tugboat_img = pygame.transform.scale(self.tugboat_img, self.scale_image)

        self.rpm_const = 0.0
        self.az_angle = 0.
        print(self.tugboat_img.get_rect())

        self.t_reg = time.time()
        self.x = 0.0
        self.y = 0.0
        self.heading = 10.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0
        self.u_dot = 0.0
        self.v_dot = 0.0
        self.r_dot = 0.0
        self.coef_ = coef_
        self.ship = ship()
        self.acc_lim = acc_lim
        self.ship_model = ship_model(0,0,0,self.ship,self.coef_, self.acc_lim)



    def update_tugboat_img(self, x, y):
        tug_boat_img = pygame.transform.rotate(self.tugboat_img, -self.heading)
        self.screen.blit(tug_boat_img, (x-(int(self.scale_image[0]/2)),y-(int(self.scale_image[1]/2))))
    
    def button(self):
        mouse = pygame.mouse.get_pos()
        click = pygame.mouse.get_pressed()
        if click[0] == 1:
            print(mouse[0],mouse[1])
            return mouse[0], mouse[1]
        else:
            return None

    def main_loop(self):
        dt = time.time() - self.t_reg
        
        if dt>0.3:
            self.t_reg = time.time()
            self.u, self.v, self.r, self.heading, delta_x_0, delta_y_0, delta_r_0, self.u_dot, self.v_dot, self.r_dot = self.ship_model.manoeuvre_model_rt_evolution(
                self.u, self.v, self.r, self.heading,
                self.rpm_const, self.rpm_const, self.rpm_const,
                self.az_angle, 180., 180., dt)
            self.x = self.x + delta_x_0
            self.y = self.y + delta_y_0
            # self.heading = self.heading + delta_r_0
            self.spawn_location = (500 + int(self.x * self.pixel_meter_ratio),500 - int(self.y * self.pixel_meter_ratio))
            # self.spawn_location[1] = int(self.y * self.pixel_meter_ratio)
            print(self.rpm_const, self.az_angle)
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
            if self.button():
                None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.rpm_const = self.rpm_const + 1.0

                if event.key == pygame.K_DOWN:
                    self.rpm_const = self.rpm_const - 1.0

                if event.key == pygame.K_LEFT:
                    self.az_angle = self.az_angle + 10.0

                if event.key == pygame.K_RIGHT:
                    self.az_angle = self.az_angle - 10.0


            # if dt>0.1:


            if self.heading>360.0:
                self.heading = self.heading - 360
            if self.heading<0.0:
                self.heading = self.heading + 360

        # if dt>0.5:
        
            # self.t_reg = time.time()
        self.screen.fill(self.bg_color)
        
        self.update_tugboat_img(self.spawn_location[0], self.spawn_location[1])
        # orientation and line
        pygame.display.update()

    
settings = None
if __name__ == '__main__':
    a = Kernel(settings, ship, ship_model, coef_, acc_lim)
    while True:
        a.main_loop()
   
    
   

    
   
    
   
#take u_dot out of equation of vorige u_dot, mse based on x and y    
   
    
   
# def haversine(lon1, lat1, lon2, lat2):
#     """
#     Calculate the great circle distance between two points
#     on the earth (specified in decimal degrees)
#     """
#     # convert decimal degrees to radians
#     lon1, lat1, lon2, lat2 = map(np.deg2rad, [lon1, lat1, lon2, lat2])

#     # haversine formula
#     dlon = lon2 - lon1
#     dlat = lat2 - lat1
#     a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
#     c = 2 * np.arcsin(np.sqrt(a))
#     r = 6371 # Radius of earth in kilometers. Use 3956 for miles
#     return c * r * 1000
  
# def haversine_invert(lon1, lat1, d, brng):
#     d = d/1000.
#     R = 6378.1 #Radius of the Earth in km
#     brng = 1.57 #in rad

#     lat2 = np.arcsin( np.sin(lat1)*np.cos(d/R) +
#          np.cos(lat1)*np.sin(d/R)*np.cos(brng))
    
#     lon2 = lon1 + np.arctan2(np.sin(brng)*np.sin(d/R)*np.cos(lat1),
#                  np.cos(d/R)-np.sin(lat1)*np.sin(lat2))
    
#     lat2 = np.rad2deg(lat2)
#     lon2 = np.rad2deg(lon2)
  
    