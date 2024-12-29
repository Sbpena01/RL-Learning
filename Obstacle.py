import pygame

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
        