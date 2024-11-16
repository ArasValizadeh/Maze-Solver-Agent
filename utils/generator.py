import pygame
import env
import json
import os

maps_path = os.path.join((os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))), "Maps")

class Generator:
    def __init__(self, map_name, FPS):
        self.env = env.Env(map_name)
        self.FPS = FPS
        pygame.init()
        self.screen = pygame.display.set_mode((self.env.x_range * 20, self.env.y_range * 20))
        pygame.display.set_caption("Obstacle Generator")
        self.clock = pygame.time.Clock()

    def draw_grid(self):
        """Draw the grid and obstacles"""

        self.screen.fill((255, 255, 255))  # White background
        
        for x in range(self.env.x_range):
            for y in range(self.env.y_range):
                rect = pygame.Rect(x * 20, y * 20, 20, 20)

                if (x, y) in self.env.obs:
                    pygame.draw.rect(self.screen, (0, 0, 0), rect)  # Draw obstacles in black
                
                else:
                    pygame.draw.rect(self.screen, (200, 200, 200), rect, 1)  # Draw grid in light grey

        pygame.display.update()

    def toggle_obstacle(self, pos):
        """Toggle the obstacle at the given position"""

        grid_pos = (pos[0] // 20, pos[1] // 20)

        if grid_pos in self.env.obs:
            self.env.obs.remove(grid_pos)  # Remove obstacle if it's already there
        
        else:
            self.env.obs.add(grid_pos)  # Add obstacle if it's not there

    def input_obstacles(self):
        """Allow students to add obstacles using mouse clicks"""

        running = True
        while running:
            self.draw_grid()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.toggle_obstacle(pygame.mouse.get_pos())  # Add/remove obstacle on click

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:  # Press Enter to finish
                        running = False

            self.clock.tick(self.FPS)

        pygame.quit()

        return self.env.obs

    def save_obstacles(self, file_path):
        """Save obstacles to a file"""

        with open(file_path, 'w') as f:
            json.dump(list(self.env.obs), f)
        
        print(f"Obstacles saved to {file_path}.")

def main(map_name='default', FPS=30):
    file_path = os.path.join(maps_path, map_name + ".json")

    generator = Generator(map_name, FPS)
    generator.input_obstacles()
    generator.save_obstacles(file_path)

if __name__ == "__main__":
    main()