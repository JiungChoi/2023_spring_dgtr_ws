class Env:
    def __init__(self, cost_map):
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.cost_map = cost_map
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        obs = set()
        size_y, size_x = self.cost_map.shape

        for i in range(size_y):
            for j in range(size_x):
                if self.cost_map[i][j] == 255: 
                    obs.add((j, i))

        return obs