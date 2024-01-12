import math
import sys
import numpy as np

import pygame
import pygame.locals


# ruff: noqa
# pylint: skip-file
class KeyboardCmd:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.rot = 0.0
        self.pick = None
        self.release = None
        self.state = -1
        self.arm1 = 0.0
        self.arm2 = 0.0
        self.arm3 = 0.0

    def initialize(self):
        pygame.init()
        BLACK = (0, 0, 0)
        WIDTH = 400
        HEIGHT = 400
        windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        windowSurface.fill(BLACK)

    def reset(self):
        self.vx = 0.0
        self.vy = 0.0
        self.rot = 0.0
        self.pick = None
        self.release = None
        self.state = -1
        self.arm1 = 0.0
        self.arm2 = 0.0
        self.arm3 = 0.0

    def get_keyboard_cmd(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # checking if keydown event happened or not
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    self.vy += 0.02
                if event.key == pygame.K_d:
                    self.vy -= 0.02

                if event.key == pygame.K_w:
                    self.vx += 0.02
                if event.key == pygame.K_s:
                    self.vx -= 0.02

                if event.key == pygame.K_q:
                    self.rot += math.radians(5)
                if event.key == pygame.K_e:
                    self.rot -= math.radians(5)

                if event.key == pygame.K_o:
                    self.pick = "left_hand"
                    self.release = False
                if event.key == pygame.K_p:
                    self.pick = "right_hand"
                    self.release = False
                if event.key == pygame.K_r:
                    self.release = True
                    self.pick = None
                elif event.key == pygame.K_0:
                    self.state = 0
                elif event.key == pygame.K_1:
                    self.state = 1
                elif event.key == pygame.K_2:
                    self.state = 2
                elif event.key == pygame.K_3:
                    self.state = 3

                elif event.key == pygame.K_z:
                    self.arm1 += 5 * np.pi / 180
                elif event.key == pygame.K_x:
                    self.arm1 -= 5 * np.pi / 180
                elif event.key == pygame.K_c:
                    self.arm2 += 5 * np.pi / 180
                elif event.key == pygame.K_v:
                    self.arm2 -= 5 * np.pi / 180
                elif event.key == pygame.K_b:
                    self.arm3 += 5 * np.pi / 180
                elif event.key == pygame.K_n:
                    self.arm3 -= 5 * np.pi / 180

        print(f"desired velocity cmd: {self.vx, self.vy, self.rot,self.state}")
        # TODO: check velocity limits
