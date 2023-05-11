"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self, cost_map):
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.cost_map = cost_map
        y, x = self.cost_map.shape
        self.x_range = (0, x)
        self.y_range = (0, y)
        self.obs_rectangle = self.obs_rectangle(self)
        self.obs_circle = self.obs_circle(self)

    def update_obs(self, obs_rectangle, obs_circle):
        self.obs_rectangle = obs_rectangle
        self.obs_circle = obs_circle

    def update_obs(self, obs_rectangle):
        self.obs_rectangle = obs_rectangle

    @staticmethod
    def obs_rectangle(self):
        obs_rectangle = []
        for i in range(self.y_range[1]):
            for j in range(self.x_range[1]):
                if self.cost_map[i][j] == 255: 
                    obs_rectangle.append([j, i, 1, 1])
        return obs_rectangle

    @staticmethod
    def obs_circle(self):
        obs_cir = [
        ]
        return obs_cir

    '''
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1]
            [50, 1, 1, 30]
        ]
        return obs_boundary

    
    '''
