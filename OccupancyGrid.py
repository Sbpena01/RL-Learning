import pygame
import numpy as np
import copy

from Obstacle import Environment

class State():
    def __init__(self, position, theta, v, phi, cost=0.0, parent=None):
        self.position = position
        self.theta = theta
        self.v = v
        self.phi = phi
        # self.cost = cost
        # self.parent = parent

    # def setCost(self, cost):
    #     self.cost = cost
    
    def __str__(self):
        return f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}), Theta: {np.rad2deg(self.theta):.2f}, Velocity: {self.v}"

    # def __lt__(self, other: 'State'):
    #     return self.cost < other.cost

class Cell():
    def __init__(self, center, width, height, grid_coord):
        self.corners = [
            (center[0] - width/2, center[1]-height/2),
            (center[0] + width/2, center[1]-height/2),
            (center[0] + width/2, center[1]+height/2),
            (center[0] - width/2, center[1]+height/2),
        ]
        self.top_left = self.corners[0]
        self.width = width
        self.height = height
        self.world_coord = center
        self.grid_coord = grid_coord
        self.rect = pygame.Rect(self.top_left[0], self.top_left[1], self.width, self.height)
        self.color = (100, 100, 100)
        self.value = -1  # Unknown
        self.state = State((self.world_coord), 0.0, 0.0, 0.0)
    
    def __eq__(self, other: 'Cell'):
        return self.grid_coord == other.grid_coord

    def worldToGrid(self, world_coord):
        return (int(world_coord[0]/self.width), int(world_coord[1]/self.height))

    def updateColor(self):
        self.color = (0, 0, 0) if self.value == 1 else (255, 255, 255)
    
    def setState(self, state:State):
        self.state = state

class OccupancyGrid():
    def __init__(self, resolution: int, environment: Environment):
        self.data = []
        self.cells = {}
        self.resolution = resolution  # Meters per cell
        self.environment = environment
        self.m2p = environment.m2p
        self.p2m = 1/self.m2p
        self.width = environment.width
        self.height = environment.height
    
    def worldToGrid(self, world_coord):
        side_length = self.resolution
        return (int(world_coord[0]/side_length), int(world_coord[1]/side_length))
    
    def gridToWorld(self, grid_coord):
        return (grid_coord[0]*self.resolution*self.m2p+5, grid_coord[1]*self.resolution*self.m2p+5)

    def createFromEnvironment(self):
        cell_side_length = self.resolution
        width_coords = np.arange(cell_side_length/2, self.width+cell_side_length, cell_side_length)
        height_coords = np.arange(cell_side_length/2, self.height+cell_side_length, cell_side_length)
        for w in width_coords:
            for h in height_coords:
                cell = Cell((w,h), cell_side_length, cell_side_length, self.worldToGrid((w,h)))
                cell.value = 1 if self.environment.checkRectCollision(cell) else 0
                cell.updateColor()
                self.cells[cell.grid_coord] = cell

    def drawGrid(self, source: pygame.Surface, m2p = 50):
        for cell in self.cells.values():
            cell_rect = pygame.Rect(cell.top_left[0]*m2p, cell.top_left[1]*m2p, cell.width*m2p, cell.height*m2p)
            pygame.draw.rect(source, cell.color, cell_rect)
            pygame.draw.rect(source, (100, 100, 100), cell_rect, width=1)

    def getNeighbors(self, cell: Cell):
        neighbors = []
        cell_coord = cell.grid_coord
        neighbors.append((cell_coord[0]+1, cell_coord[1]))
        neighbors.append((cell_coord[0]-1, cell_coord[1]))
        neighbors.append((cell_coord[0], cell_coord[1]+1))
        neighbors.append((cell_coord[0], cell_coord[1]-1))
        neighbors.append((cell_coord[0]+1, cell_coord[1]+1))
        neighbors.append((cell_coord[0]-1, cell_coord[1]-1))
        neighbors.append((cell_coord[0]-1, cell_coord[1]+1))
        neighbors.append((cell_coord[0]+1, cell_coord[1]-1))

        free_neighbors: list[Cell] = []
        for neighbor in neighbors:
            cell = self.cells.get(neighbor)
            if cell is None:
                continue
            if cell.value == 0:
                free_neighbors.append(cell)
        return free_neighbors
    
    def createConfigurationSpace(self, layers=1):
        for _ in range(0, layers+1):
            added_c_space: list[Cell] = []
            for cell in self.cells.values():
                if cell.value != 1:
                    continue
                neighbors = self.getNeighbors(cell)
                for neighbor_cell in neighbors:
                    added_c_space.append(neighbor_cell)
            for cell in added_c_space:
                cell.value = 1
                cell.updateColor()
            
        