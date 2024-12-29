import numpy as np
import pygame

from Car import Car

SCREEN_HEIGHT = 950
SCREEN_WIDTH = 1600
BACKGROUND_COLOR = (255, 255, 255)  # White

def main(args=None):
    pygame.init()
    pygame.font.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(BACKGROUND_COLOR)
    font = pygame.font.Font(None, size=16)
    clock = pygame.time.Clock()

    # Temp car to experiment with drawing a surface onto pygame.
    car = Car((100, 100))

    # Game Loop
    running = True
    while running:
        clock.tick(60)  # 60 fps
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    car.setVelocity(3.0)
                elif event.key == pygame.K_s:
                    car.setVelocity(-3.0)
                if event.key == pygame.K_d:
                    car.setSteeringAngle(np.deg2rad(1))
                elif event.key == pygame.K_a:
                    car.setSteeringAngle(np.deg2rad(-1))
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_w or event.key == pygame.K_s:
                    car.setVelocity(0.0)
                if event.key == pygame.K_d or event.key == pygame.K_a:
                    car.setSteeringAngle(0.0)

        # Kinematics step
        dt = clock.get_time() / 1000  # milliseconds to seconds
        car.step(dt)

        # Draw the environment on the screen
        screen.fill(BACKGROUND_COLOR)
        position_text = font.render(f"Car Position: {car.position[0]:.2f}, {car.position[1]:.2f}", True, (0,0,0))
        angle_text = font.render(f"Car Angle: {np.rad2deg(car.theta):.2f}", True, (0,0,0))
        screen.blit(position_text, (10, 10))
        screen.blit(angle_text, (10, 10+font.get_linesize()))
        car.draw(screen)
        pygame.display.flip()
    pygame.quit()

if __name__ == '__main__':
    main()