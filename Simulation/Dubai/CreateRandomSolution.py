import numpy as np


class RandomSolution:
    def __init__(self, var_size, var_min, var_max):
        """

        """
        self.r = np.random.uniform(var_min['r'], var_max['r'], var_size)
        self.psi = np.random.uniform(var_min['psi'], var_max['psi'], var_size)
        self.phi = np.random.uniform(var_min['phi'], var_max['phi'], var_size)

    def get_solution(self):
        """Retourne la solution sous forme de dictionnaire."""
        return {'r': self.r, 'psi': self.psi, 'phi': self.phi}