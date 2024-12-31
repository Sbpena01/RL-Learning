import numpy as np
import pygame

from Car import Car
from Obstacle import Environment
from OccupancyGrid import *
from MotionPlanner import MotionPlanner

SCREEN_HEIGHT = 950
SCREEN_WIDTH = 1600
BACKGROUND_COLOR = (255, 255, 255)  # White
METERS_TO_PIXELS = 50
PIXELS_TO_METERS = 1 / METERS_TO_PIXELS
GRID_RESOLUTION = 0.5  # Meters per gridcell
FPS = 30

CAR_START = (1.75, 2.25)

def main(args=None):
    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(BACKGROUND_COLOR)
    font = pygame.font.Font(None, size=16)
    clock = pygame.time.Clock()
    car = Car(CAR_START, m2p=METERS_TO_PIXELS)
    env = Environment(SCREEN_HEIGHT*PIXELS_TO_METERS, SCREEN_WIDTH*PIXELS_TO_METERS)

    # Add obstacles
    env.createAndAddObstacle((300, 300), 50, 200)
    env.createAndAddObstacle((1000, 600), 500, 100)

    grid = OccupancyGrid(GRID_RESOLUTION, env)
    grid.createFromEnvironment()
    grid.createConfigurationSpace(layers=1)

    start = State(car.position, car.theta, car.v, car.phi)
    goal = State((30, 15), 0.0, 0.0, 0.0)
    planner = MotionPlanner(start, goal, car, grid)
    path = planner.plan(screen)

    # Game Loop
    running = True
    while running:
        clock.tick(FPS)
            
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    car.setVelocity(5.0)
                elif event.key == pygame.K_s:
                    car.setVelocity(-5.0)
                if event.key == pygame.K_d:
                    car.setSteeringAngle(np.deg2rad(30))
                elif event.key == pygame.K_a:
                    car.setSteeringAngle(np.deg2rad(-30))
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w or event.key == pygame.K_s:
                    car.setVelocity(0.0)
                if event.key == pygame.K_d or event.key == pygame.K_a:
                    car.setSteeringAngle(0.0)

        # Kinematics step
        dt = clock.get_time() / 1000  # milliseconds to seconds
        car.step(dt)

        # Collision Check
        collision = env.checkCarCollision(car)
        collision_text = font.render(f"CRASHED", True, (255, 0, 0)) if collision else font.render(f"Safe", True, (0, 255, 0))

        # Draw the environment on the screen
        screen.fill(BACKGROUND_COLOR)


        grid.drawGrid(screen, m2p=METERS_TO_PIXELS)
        env.draw(screen)
        car.draw(screen, draw_hitbox=False)

        if path is not None:
            planner.drawPath(path, screen)
        planner.draw(screen)

        # Text in the top left corner
        position_text = font.render(f"Car Position: {car.position[0]:.2f}, {car.position[1]:.2f}", True, (0,0,0))
        cell_text = font.render(f"Grid Position: {grid.worldToGrid(car.position)}", True, (0,0,0))
        angle_text = font.render(f"Car Angle: {np.rad2deg(car.theta):.2f}", True, (0,0,0))
        pygame.draw.rect(screen, (200,200,200), pygame.Rect(5.0, 5.0, 145, 45))
        screen.blit(position_text, (10, 10))
        screen.blit(cell_text, (10, 10+font.get_linesize()))
        screen.blit(angle_text, (10, 10+2*font.get_linesize()))
        screen.blit(collision_text, (500, 10))

        pygame.display.flip()
    pygame.quit()

if __name__ == '__main__':
    main()