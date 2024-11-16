import pygame
import env
import math

class Plotting:
    def __init__(self, xI, xG, map_name, FPS=60):
        self.FPS = FPS
        self.xI, self.xG = xI, xG
        self.env = env.Env(map_name)
        self.obs = self.env.obs

        # Define the height for the banner
        self.banner_height = 40

        # Calculate Euclidean distance between start and goal for color interpolation
        self.min_path_length = math.hypot(xG[0] - xI[0], xG[1] - xI[1])

        # Pygame initialization
        pygame.init()
        self.screen = pygame.display.set_mode((self.env.x_range * 20, self.env.y_range * 20 + self.banner_height))
        pygame.display.set_caption("Robot Path Planning")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 24)  # Font for displaying text

    def draw_grid(self):
        # Draw the white banner at the top
        pygame.draw.rect(self.screen, (255, 255, 255), (0, 0, self.env.x_range * 20, self.banner_height))

        # Draw the grid area below the banner
        self.screen.fill((245, 245, 245), (0, self.banner_height, self.env.x_range * 20, self.env.y_range * 20))

        # Draw obstacles
        self.draw_obstacles()

        # Draw start and goal with a glowing effect
        self.draw_glow(self.xI, (0, 0, 255))  # Blue start point with glow
        self.draw_glow(self.xG, (0, 255, 0))  # Green goal point with glow
    
    def draw_obstacles(self):
        """Draw obstacles in black color"""

        for obs in self.obs:
            pygame.draw.rect(self.screen, (0, 0, 0), (obs[0] * 20, obs[1] * 20 + self.banner_height, 20, 20))
    
    def draw_glow(self, pos, color):
        """Draw a glowing effect around a point"""

        for radius in range(15, 0, -5):  # Layered circles to simulate glow
            alpha = 50 if radius == 15 else 150  # Adjust transparency
            s = pygame.Surface((40, 40), pygame.SRCALPHA)
            pygame.draw.circle(s, (*color, alpha), (20, 20), radius)
            self.screen.blit(s, (pos[0] * 20 - 10, pos[1] * 20 + self.banner_height - 10))

    def draw_path(self, path, END=False):
        """Draw path with dynamic size and gradient"""

        length = len(path)
        if END:
            # Draw the final path in solid red
            for pos in path:
                pygame.draw.circle(self.screen, (255, 50, 50), (pos[0] * 20 + 10, pos[1] * 20 + 10 + self.banner_height), 10)
            
        else:
            # Draw the path with a color gradient
            for index, pos in enumerate(path):
                size = 10 + (index * 5 // length)  # Dynamically grow the path circles
                color = (255 - (index * 200 // length), 50, 50)  # Gradient from red to light red
                pygame.draw.circle(self.screen, color, (pos[0] * 20 + 10, pos[1] * 20 + 10 + self.banner_height), size)

    def draw_visited(self, visited):
        """Draw visited nodes with a fading color gradient"""

        length = len(visited)
        for index, pos in enumerate(visited):
            size = 8  # Fixed size for visited nodes
            gradient = (200 - int(200 * index / length), 200, 200)  # Fade the grey color
            pygame.draw.circle(self.screen, gradient, (pos[0] * 20 + 10, pos[1] * 20 + 10 + self.banner_height), size)

    def draw_particle_trail(self, path):
        """Add a subtle particle effect along the path"""
        
        particles = [Particle(pos) for pos in path]
        for particle in particles:
            particle.update()
            particle.draw(self.screen, self.banner_height)
    
    def interpolate_color(self, value, max_value):
        """Interpolate from green to red based on value."""

        ratio = min(value / max_value, 1)  # Ensure the ratio does not exceed 1
        red = int(255 * ratio)
        green = int(255 * (1 - ratio))

        return red, green, 0
    
    def update_info_display(self, visited_count, path_length):
        """Display visited nodes count and path length with color interpolation."""

        max_visited = max((self.env.x_range * self.env.y_range) - (len(self.obs) * 4), 1) # Adjust the ratio for visited nodes
        max_path = max(self.min_path_length * 4, 1) # Adjust the ratio for path length

        # Calculate dynamic colors for each number
        visited_color = self.interpolate_color(visited_count, max_visited)
        path_color = self.interpolate_color(path_length, max_path)

        # Render labels in black and numbers in dynamic colors
        visited_label = self.font.render("Visited:", True, (0, 0, 0))
        visited_number = self.font.render(str(visited_count), True, visited_color)
        path_label = self.font.render("Path Length:", True, (0, 0, 0))
        path_number = self.font.render(str(path_length), True, path_color)

        # Left side of the banner
        self.screen.blit(visited_label, (10, 10))
        self.screen.blit(visited_number, (10 + visited_label.get_width() + 5, 10))

        # Right side of the banner
        self.screen.blit(path_label, (self.env.x_range * 20 - path_label.get_width() - path_number.get_width() - 15, 10))
        self.screen.blit(path_number, (self.env.x_range * 20 - path_number.get_width() - 10, 10))

    def update(self):
        pygame.display.update()
        self.clock.tick(self.FPS)

    def animation(self, path, visited):
        running = True
        path_index = 0
        visited_index = 0

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.draw_grid()

            # Draw visited nodes gradually
            if visited_index < len(visited):
                self.draw_visited(visited[:visited_index])
                visited_index += 1  # Increment the visited index to draw more nodes
                pygame.time.delay(5)

            # Draw the path gradually
            elif path_index < len(path):
                self.draw_visited(visited)  # Draw all visited nodes
                self.draw_path(path[:path_index])
                self.draw_particle_trail(path[:path_index])  # Draw the particle trail
                path_index += 1  # Increment the path index to draw more path points
                pygame.time.delay(50)

            else:
                self.draw_visited(visited[:visited_index])
                self.draw_path(path[:path_index], True)  # Final path in solid red
                self.draw_particle_trail(path[:path_index])  # Final particle trail
                pygame.time.delay(50)
            
            # Update the info display with dynamic colors for the current visited count and path length
            self.update_info_display(visited_count=visited_index, path_length=path_index)

            self.update()

        pygame.quit()

class Particle:
    def __init__(self, pos):
        self.x, self.y = pos
        self.lifetime = 100  # Short lifetime
        self.size = 5
        self.color = (255, 150, 150)
    
    def update(self):
        self.size -= 0.1  # Shrink over time
        self.lifetime -= 1
    
    def draw(self, screen, banner_height):
        if self.lifetime > 0:
            pygame.draw.circle(screen, self.color, (self.x * 20 + 10, self.y * 20 + 10 + banner_height), int(self.size))