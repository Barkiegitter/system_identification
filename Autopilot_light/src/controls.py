# from Objects import Polygon
import pygame
import sys

class Kernel:
    def __init__(self, settings):
        pygame.init()
        self.screen_size = (1200,  800)
        self.bg_color = (255, 255, 255)
        # self.fps = 60
        self.screen = pygame.display.set_mode(self.screen_size)
        self.constant = 0
        self.heading_constant = 0
        self.X = 400
        self.Y = 400
        self.yellow = (255, 255, 0)
        self.att_hdg = 0

        self.broadcast_heading = 0.0
        self.broadcast_speed = 0.0

        self.BLACK = (0,0,0)
        self.broadcast = False

        pygame.display.set_caption('Show Text')

        self.font = pygame.font.SysFont('monospace', 32)

        self.text = self.font.render('GeeksForGeeks', True, self.yellow)

        self.textRect = self.text.get_rect()

        self.textRect.center = (self.X / 2, self.Y / 2)

        self.manoeuvre = None

        self.largeText2 = pygame.font.Font('freesansbold.ttf', 15)

        self.manoeuvre_circle_right = False
        self.manoeuvre_circle_left = False
        self.manoeuvre_zigzag_10 = False
        self.manoeuvre_zigzag_20 = False
        self.manoeuvre_astern = False


    def button(self, msg, x, y, w, h, ic, ac, action=None):
        mouse = pygame.mouse.get_pos()
        click = pygame.mouse.get_pressed()
        TextSurf, TextRect = self.text_objects(
            str(msg), self.largeText2)
        TextRect.center = ((int(x+w/2)), (int(y+h/2)))
        self.screen.blit(TextSurf, TextRect)
        if x + w > mouse[0] > x and y + h > mouse[1] > y:
            pygame.draw.rect(self.screen, ac, (x, y, w, h))
            TextSurf, TextRect = self.text_objects(
                msg, self.largeText2)
            TextRect.center = ((int(x + w / 2)), (int(y + h / 2)))
            if click[0] == 1:
                self.manoeuvre = msg  # Call the callback function.

    def text_objects(self, text, font):
        textSurface = font.render(text, True, self.BLACK)
        return textSurface, textSurface.get_rect()

    def main_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    self.constant = self.constant + 1.0

                if event.key == pygame.K_DOWN:
                    self.constant = self.constant - 1.0

                if event.key == pygame.K_LEFT:
                    self.heading_constant = self.heading_constant - 1.0
                    if self.heading_constant>360 :
                        self.heading_constant = self.heading_constant % 360
                    # if self.heading_constant<0 :
                    #     self.heading_constant = 360 + self.heading_constant

                if event.key == pygame.K_RIGHT:
                    self.heading_constant = self.heading_constant + 1.0
                    if self.heading_constant>360 :
                        self.heading_constant = self.heading_constant % 360
                    # if self.heading_constant<0 :
                    #     self.heading_constant = 360 + self.heading_constant

                if event.key == pygame.K_a:
                    self.att_hdg = self.att_hdg - 10.0
                    if self.att_hdg > 360:
                        self.att_hdg = self.att_hdg % 360
                    if self.att_hdg < 0:
                        self.att_hdg = 360 + self.att_hdg

                if event.key == pygame.K_d:
                    self.att_hdg = self.att_hdg + 10
                    if self.att_hdg > 360:
                        self.att_hdg = self.att_hdg % 360
                    if self.att_hdg < 0:
                        self.att_hdg = 360 + self.att_hdg

                if event.key == pygame.K_RETURN :
                    self.broadcast == True
                    self.broadcast_heading = self.heading_constant
                    self.broadcast_speed = self.constant


            # self.text = self.font.render(str(self.constant) + ' ' + str(self.heading_constant) + ' ' + str(self.att_hdg), True, self.yellow)
            # self.screen.blit(self.text, self.textRect)

            self.screen.fill(self.bg_color)

            TextSurf, TextRect = self.text_objects(str(self.constant) + '    ' + str(self.heading_constant) + '    ' + str(self.att_hdg), self.largeText2)
            TextRect.center = ((100), (100))
            self.screen.blit(TextSurf, TextRect)
            self.button("manoeuvre_circle_right", 200, 100, 150, 60, (100, 100, 100), (168, 168, 168))
            self.button("manoeuvre_circle_left", 200, 200, 150, 60, (100, 100, 100), (168, 168, 168))
            self.button("manoeuvre_zigzag_10", 200, 300, 150, 60, (100, 100, 100), (168, 168, 168))
            self.button("manoeuvre_zigzag_20", 200, 400, 150, 60, (100, 100, 100), (168, 168, 168))
            self.button("manoeuvre_astern", 200, 500, 150, 60, (100, 100, 100), (168, 168, 168))

            pygame.display.update()
        if self.broadcast:
            return self.constant, self.heading_constant ,self.att_hdg, self.manoeuvre
        else:
            self.broadcast = False
            return self.broadcast_speed, self.broadcast_heading, self.att_hdg, self.manoeuvre




