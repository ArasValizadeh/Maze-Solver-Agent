from abc import ABC, abstractmethod
import env

class AbstractSearchAgent(ABC):
    """
    Abstract class for Search Agents.
    """

    def __init__(self, s_start, s_goal, map_name):
        self.s_start = s_start
        self.s_goal = s_goal

        self.Env = env.Env(map_name)
        self.u_set = self.Env.motions  
        self.obs = self.Env.obs 

        self.OPEN = []  
        self.CLOSED = [] 
        self.PARENT = dict()
        self.g = dict()  

    @abstractmethod
    def searching(self):
        """
        Abstract method for the searching algorithm.
        :return: path, visited nodes
        """
        pass

    def get_neighbor(self, s):
        """
        Find neighbors of the state that are not obstacles.
        :param s: current state
        :return: list of neighbors
        """
        
        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set if (s[0] + u[0], s[1] + u[1]) not in self.obs]

    def extract_path(self, PARENT):
        """
        Extract the path from the start to the goal based on the parent dictionary.
        :return: the planned path
        """

        path = [self.s_goal]
        s = self.s_goal

        while s != self.s_start:
            s = PARENT[s]
            path.append(s)

        return list(reversed(path))