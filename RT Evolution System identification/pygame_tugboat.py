#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 10 13:39:01 2020

@author: erwinlodder
"""
import pygame
import sys
import numpy as np

class Kernel:
    def __init__(self, settings):
        pygame.init()
        self.screen_size = (1000, 1000) #width height
        self.screen = pygame.display.set_mode(self.screen_size)
        self.bg_color = (140, 153, 173)
        pygame.display.set_caption('tugboat control')
        
        self.scale_image = (114,260)
        self.pixel_meter_ratio = 0.1
        
        self.spawn_location = (500,500)
        
        self.tugboat_img = pygame.image.load('tugboat.png')
        self.tugboat_img = pygame.transform.scale(self.tugboat_img, self.scale_image)
        print(self.tugboat_img.get_rect())
        
        
        # self.lat_begin = 0.0
        # self.lon_begin = 0.0
        # self.lat_end = 0.0005
        # self.lon_end = 0.0005
        # self.spawn_location = (0.00025, 0.00025)  #lon, lat
    
    def update_tugboat_img(self, x, y):
        self.screen.blit(self.tugboat_img, (x-(int(self.scale_image[0]/2)),y-(int(self.scale_image[1]/2))))
    
    def button(self):
        mouse = pygame.mouse.get_pos()
        click = pygame.mouse.get_pressed()
        if click[0] == 1:
            print(mouse[0],mouse[1])
            return mouse[0], mouse[1]
        else:
            return None
    def main_loop(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.display.quit()
                    pygame.quit()                
                if self.button():
                    None
                self.screen.fill(self.bg_color)
                self.update_tugboat_img(self.spawn_location[0], self.spawn_location[1])
                pygame.display.update()
    
settings = None
if __name__ == '__main__':
    a = Kernel(settings)
    a.main_loop()
   
    
   

    
   
    
   
    
   
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
  
    