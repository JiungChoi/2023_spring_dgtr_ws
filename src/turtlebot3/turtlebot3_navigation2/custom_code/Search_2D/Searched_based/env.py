class Env:
    def __init__(self, cost_map):
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.cost_map = cost_map
        self.y_range, self.x_range = self.cost_map.shape
        self.obs = self.obs_map()
        
        

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        obs = set()
        for i in range(self.y_range):
            for j in range(self.x_range):
                if self.cost_map[i][j] == 255: 
                    obs.add((j, i))

        return obs