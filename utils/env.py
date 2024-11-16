import json
import os

maps_path = os.path.join((os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))), "Maps")

class Env:
    def __init__(self, map_name):
        self.x_range = 51
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.load_obstacles(map_name)

    def load_obstacles(self, map_name="default"):
        """Load obstacles from a file"""

        file_path = os.path.join(maps_path, map_name + ".json")

        try:
            with open(file_path, 'r') as f:
                obs_list = json.load(f)
                obs = set(tuple(ob) for ob in obs_list)
            
            print(f"Obstacles loaded from {file_path}.")

        except FileNotFoundError:
            print(f"File {file_path} not found.")
        
        return obs