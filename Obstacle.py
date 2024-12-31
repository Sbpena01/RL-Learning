import pygame

from Car import Car
import Utils

class Obstacle():
    def __init__(self, center: tuple, width: int, height: int):
        self.corners = [
            (center[0] - width/2, center[1]-height/2),
            (center[0] + width/2, center[1]-height/2),
            (center[0] + width/2, center[1]+height/2),
            (center[0] - width/2, center[1]+height/2),
        ]
        self.edges = [
            (self.corners[0], self.corners[1]),
            (self.corners[1], self.corners[2]),
            (self.corners[2], self.corners[3]),
            (self.corners[3], self.corners[0])
        ]
        self.center = center
        self.top_left = self.corners[0]
        self.width = width
        self.height = height
        self.surface = pygame.Surface((width, height))
    
    def draw(self, source: pygame.Surface):
        source.blit(self.surface, self.top_left)

class Environment():
    def __init__(self, height, width):
        self.obstacles: list[Obstacle] = []
        self.height = height
        self.width = width
        self.m2p = 50
    
    def addObstacle(self, obstacle: Obstacle):
        self.obstacles.append(obstacle)
    
    def createAndAddObstacle(self, center: tuple, width: int, height: int):
        obstacle_to_add = Obstacle(center, width, height)
        self.obstacles.append(obstacle_to_add)
    
    def draw(self, source: pygame.Surface):
        for obstacle in self.obstacles:
            source.blit(obstacle.surface, obstacle.top_left)
    
    def checkCarCollision(self, car:Car):
        for obstacle in self.obstacles:
            for edge in obstacle.edges:
                for car_edge in car.collision_box_edges:
                    if Utils.checkLineCollision(edge, car_edge):
                        return True
        return False

    def checkRectCollision(self, input: pygame.rect):
        for obstacle in self.obstacles:
            rect = obstacle.surface.get_rect()
            rect.update(obstacle.top_left[0], obstacle.top_left[1], obstacle.width, obstacle.height)
            if rect.colliderect(input):
                return True
        return False