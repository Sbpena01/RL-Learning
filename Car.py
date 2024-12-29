import pygame
import numpy as np

import Utils
from Obstacle import Obstacle

WHEELBASE = 0.5  # meters
LENGTH = 1  # meters
COLOR = (0, 0, 0)

class Car():
    def __init__(self, position: tuple):
        self.m2p = 50  # 10 pixels is 1 meter.

        self.position = position
        self.theta = 0.0  # Radians
        self.v = 0.0  # Velocity
        self.phi = 0.0  # Steering Angle
        self.l = LENGTH
        self.w = WHEELBASE

        visual_size = (self.l*self.m2p, self.w*self.m2p)
        self.visual = pygame.Surface(visual_size, pygame.SRCALPHA)
        self.visual.fill(COLOR)
        self.collision_box_corners = [
            (self.position[0]-visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]+visual_size[1]/2),
            (self.position[0]-visual_size[0]/2, self.position[1]+visual_size[1]/2)
        ]
        self.collision_box_edges = [
            (self.collision_box_corners[0], self.collision_box_corners[1]),
            (self.collision_box_corners[1], self.collision_box_corners[2]),
            (self.collision_box_corners[2], self.collision_box_corners[3]),
            (self.collision_box_corners[3], self.collision_box_corners[0])
        ]

        self.original_corners = [
            (self.position[0]-visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]+visual_size[1]/2),
            (self.position[0]-visual_size[0]/2, self.position[1]+visual_size[1]/2)
        ]

    def draw(self, source: pygame.Surface, draw_hitbox=False):
        rotated_surface = pygame.transform.rotate(self.visual, -np.rad2deg(self.theta))
        rotated_rect = rotated_surface.get_rect(center=self.position)
        self.updateCollisionBox()
        if draw_hitbox:
            pygame.draw.line(source, (255, 0, 0), self.collision_box_corners[0], self.collision_box_corners[1])
            pygame.draw.line(source, (255, 0, 0), self.collision_box_corners[1], self.collision_box_corners[2])
            pygame.draw.line(source, (255, 0, 0), self.collision_box_corners[2], self.collision_box_corners[3])
            pygame.draw.line(source, (255, 0, 0), self.collision_box_corners[3], self.collision_box_corners[0])
        source.blit(rotated_surface, rotated_rect.topleft)
    
    def updateCollisionBox(self):
        idx = 0
        for corner in self.original_corners:
            new_px = self.position[0] + (corner[0] - self.position[0]) * np.cos(self.theta) - (corner[1] - self.position[1]) * np.sin(self.theta)
            new_py = self.position[1] + (corner[0] - self.position[0]) * np.sin(self.theta) + (corner[1] - self.position[1]) * np.cos(self.theta)
            self.collision_box_corners[idx] = (new_px, new_py)
            idx += 1
        visual_size = (self.l*self.m2p, self.w*self.m2p)
        self.original_corners = [
            (self.position[0]-visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]-visual_size[1]/2),
            (self.position[0]+visual_size[0]/2, self.position[1]+visual_size[1]/2),
            (self.position[0]-visual_size[0]/2, self.position[1]+visual_size[1]/2)
        ]
        self.collision_box_edges = [
            (self.collision_box_corners[0], self.collision_box_corners[1]),
            (self.collision_box_corners[1], self.collision_box_corners[2]),
            (self.collision_box_corners[2], self.collision_box_corners[3]),
            (self.collision_box_corners[3], self.collision_box_corners[0])
        ]
    
    def setPosition(self, position: tuple):
        self.position = position

    def setVelocity(self, velocity: float):
        self.v = velocity * self.m2p
    
    def setSteeringAngle(self, phi: float):
        self.phi = phi

    def step(self, dt):
        x_dot = self.v * np.cos(self.theta)
        y_dot = self.v * np.sin(self.theta)
        theta_dot = (self.v/self.l)*np.tan(self.phi)
        d_position = (x_dot * dt, y_dot * dt)
        d_theta = theta_dot * dt
        self.position = (self.position[0] + d_position[0], self.position[1] + d_position[1])
        self.theta += d_theta

    def translate(self, displacement: float, axis: int):
        displacement_pixels = displacement / self.m2p
        if axis != 0 and axis != 1:
            return
        if axis:
            self.position = (self.position[0], self.position[1]+displacement_pixels)
        else:
            self.position = (self.position[0]+displacement_pixels, self.position[1])

    def rotateRadians(self, rotation):
        self.theta += rotation
    
    def rotateDegrees(self, rotation):
        self.rotateRadians(np.deg2rad(rotation))

    def checkObstacleCollision(self, obstacle: Obstacle):
        for edge in obstacle.edges:
            for car_edge in self.collision_box_edges:
                if Utils.checkLineCollision(edge, car_edge):
                    return True
        return False

