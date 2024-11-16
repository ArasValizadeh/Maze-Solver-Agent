from agent import AbstractSearchAgent
from plotting import Plotting
import generator as gn
import sys
import os
from collections import deque
import heapq
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/utils")
class BFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_goal, self.s_start
        queue = deque([(start, [start])])
        visited = set([start])
        viewed = []
        while queue:
            current, path = queue.popleft()
            viewed.append(current)
            if current == goal:
                return path, viewed
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return [], viewed

class DFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_start, self.s_goal
        stack = [(start, [start])]
        visited = set([start])
        viewed = []
        while stack:
            current, path = stack.pop()
            viewed.append(current)
            if current == goal:
                return path, viewed
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append((neighbor, path + [neighbor]))
        return [], viewed

class AStar_Agent(AbstractSearchAgent):
    def heuristic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
    def heuristic2(self, node, goal):
        visibility_bonus = 5  
        distance = abs(node[0] - goal[0]) + abs(node[1] - goal[1])  
        # none represent wall
        free_neighbors = sum(1 for neighbor in self.get_neighbor(node) if neighbor is not None)
        return distance - visibility_bonus * free_neighbors
    def searching(self):
        start, goal = self.s_start, self.s_goal
        open_set = [(0, start, [start])]
        visited = set([start])
        viewed = []
        while open_set:
            _, current, path = heapq.heappop(open_set)
            viewed.append(current)
            if current == goal:
                return path, viewed
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    priority = len(path) + 1 + self.heuristic2(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor, path + [neighbor]))
        return [], viewed

if __name__ == "__main__":
    s_start = (5, 5) 
    s_goal = (45, 25) 
    FPS = 60
    generate_mode = False 
    map_name = 'default'
    if generate_mode:
        gn.main(map_name)
    else:
        agent = AStar_Agent(s_start, s_goal, map_name) 
        path, visited = agent.searching()
        plot = Plotting(s_start, s_goal, map_name, FPS)
        plot.animation(path, visited)