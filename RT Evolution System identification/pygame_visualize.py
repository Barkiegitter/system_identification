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


class Kernel:
    def __init__(self, settings):
        pygame.init()
        self.screen_size = (900, 800) #width height
        self.screen = pygame.display.set_mode(self.screen_size)
        self.bg_color = (140, 153, 173)
        pygame.display.set_caption('tugboat control')
        
        self.scale_image = (228,520)
        self.pixel_meter_ratio = 5

        self.spawn_location = (350,300)

        self.tugboat_img = pygame.image.load('tugboat.png')
        self.tugboat_img = pygame.transform.scale(self.tugboat_img, self.scale_image)
        self.pivot = pygame.math.Vector2(self.scale_image[0]/2, -self.scale_image[1]/2)
        self.w, self.h = self.tugboat_img.get_size()
        self.box = [pygame.math.Vector2(p) for p in [(0, 0), (self.w, 0), (self.w, -self.h), (0, -self.h)]]
        self.t_reg = time.time()
        
        self.x_reg_px = 0.0
        self.y_reg_px = 0.0


    def update_tugboat_img(self, x, y, hdg):
        # tug_boat_img = pygame.transform.rotate(self.tugboat_img, -hdg+10)
        pivot_rotate = self.pivot.rotate(-hdg)
        pivot_move   = pivot_rotate - self.pivot
        box_rotate = [p.rotate(-hdg) for p in self.box]
        min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
        max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])
        origin = (x + min_box[0] - pivot_move[0], y - max_box[1] + pivot_move[1])

        rotated_image = pygame.transform.rotate(self.tugboat_img, -hdg)
        self.screen.blit(rotated_image, origin)
        
        
        
        # self.screen.blit(tug_boat_img, (x-(int(self.scale_image[0]/2)),y-(int(self.scale_image[1]/2))))
    
    def button(self):
        mouse = pygame.mouse.get_pos()
        click = pygame.mouse.get_pressed()
        if click[0] == 1:
            print(mouse[0],mouse[1])
            return mouse[0], mouse[1]
        else:
            return None

    def main_loop(self, x, y, hdg):
        # dt = time.time() - self.t_reg
        
        
        # for event in pygame.event.get():

        #     if event.type == pygame.QUIT:
        #         pygame.display.quit()
        #         pygame.quit()
        #     if self.button():
        #         None

        #     if event.type == pygame.KEYDOWN:
        #         if event.key == pygame.K_UP:
        #             self.rpm_const = self.rpm_const + 1.0

        #         if event.key == pygame.K_DOWN:
        #             self.rpm_const = self.rpm_const - 1.0

        #         if event.key == pygame.K_LEFT:
        #             self.az_angle = self.az_angle + 10.0

        #         if event.key == pygame.K_RIGHT:
        #             self.az_angle = self.az_angle - 10.0


        #     # if dt>0.1:


        #     if self.heading>360.0:
        #         self.heading = self.heading - 360
        #     if self.heading<0.0:
        #         self.heading = self.heading + 360

        self.screen.fill(self.bg_color)
        
        self.x_reg_px =  x * self.pixel_meter_ratio
        self.y_reg_px =  y * self.pixel_meter_ratio
        
        
        # self.spawn_location = (self.spawn_location[0] + int(self.x_reg_px),self.spawn_location[1] - int(self.y_reg_px))
        
        self.update_tugboat_img(self.spawn_location[0] + int(self.x_reg_px), self.spawn_location[1]- int(self.y_reg_px), hdg)
        # print(self.spawn_location[0],self.spawn_location[1])
        # orientation and line
        pygame.display.update()

    
# settings = None
# if __name__ == '__main__':
#     a = Kernel(settings)
#     while True:
#         a.main_loop()
   
    
   

    
   
    
   
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
  
    