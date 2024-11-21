import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/utils")

from agent import AbstractSearchAgent
from plotting import Plotting
import generator as gn
from collections import deque
import heapq

class BFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_start, self.s_goal
        # open_list
        queue = deque([(start, [start])])
        # close_list        
        visited = set()
        # The nodes which has been viewed        
        viewed = []
        if (start == goal):
            return [] , [start]
        while queue:
            current, path = queue.popleft()
            viewed.append(current)
            visited.add(current)
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    if neighbor == goal:
                        return path + [neighbor], viewed
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return [], viewed


class DFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_start, self.s_goal
        # open_list
        stack = [(start, [start])]
        # close_list
        visited = set()
        # The nodes which has been viewed
        viewed = []
        if (start == goal):
            return [] , [start]
        while stack:
            current, path = stack.pop()
            viewed.append(current)
            visited.add(current)
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    if neighbor == goal:
                        return path + [neighbor] , viewed
                    visited.add(neighbor)
                    stack.append((neighbor, path + [neighbor]))
        return [], viewed
 
class AStar_Agent(AbstractSearchAgent):
    def heuristic(self, node, goal):
        return max(abs(node[0] - goal[0]), abs(node[1] - goal[1]))
    def heuristic3(self, node ,goal):
        return abs(node[0] - goal[0]) +  abs(node[1] - goal[1])
    def heuristic2(self, node, goal):
        rate = 5  
        distance = abs(node[0] - goal[0]) + abs(node[1] - goal[1])  
        free_neighbors = sum(1 for neighbor in self.get_neighbor(node))
        return distance - rate * free_neighbors
    def searching(self):
        start, goal = self.s_start, self.s_goal
        open_set = [(0, start, [start])]
        visited = set()
        viewed = []
        while open_set:
            _, current, path = heapq.heappop(open_set)
            viewed.append(current)
            visited.add(current)
            if current == goal:
                return path, viewed
            for neighbor in self.get_neighbor(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    priority = len(path) + 1 + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor, path + [neighbor]))
        return [], viewed

if __name__ == "__main__":
    s_start = (5, 5) 
    s_goal = (45, 25) 
    FPS = 100
    generate_mode = False 
    map_name = 'default'
    if generate_mode:
        gn.main(map_name)
    else:
        agent = AStar_Agent(s_start, s_goal, map_name) 
        path, visited = agent.searching()
        plot = Plotting(s_start, s_goal, map_name, FPS)
        plot.animation(path, visited)