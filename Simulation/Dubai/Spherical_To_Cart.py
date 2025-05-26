import numpy as np

class SphericalToCartesian:
    def __init__(self, model):
        self.model = model

    def convert(self, sol):
        # Start location
        xs, ys, zs = self.model['start']
        
        # Solution in Spherical space
        r = np.asarray(sol['r']).flatten()
        psi = np.asarray(sol['psi']).flatten()
        phi = np.asarray(sol['phi']).flatten()
        
        # Initialize arrays
        x = np.zeros(self.model['n'])
        y = np.zeros(self.model['n'])
        z = np.zeros(self.model['n'])
        
        # First point
        x[0] = xs + r[0] * np.cos(psi[0]) * np.cos(phi[0])
        y[0] = ys + r[0] * np.cos(psi[0]) * np.sin(phi[0])
        z[0] = zs + r[0] * np.sin(psi[0])
        
        # Subsequent points
        for i in range(1, self.model['n']):
            x[i] = x[i-1] + r[i] * np.cos(psi[i]) * np.cos(phi[i])
            y[i] = y[i-1] + r[i] * np.cos(psi[i]) * np.sin(phi[i])
            z[i] = z[i-1] + r[i] * np.sin(psi[i])
        
        # Apply bounds if they exist
        if 'xmin' in self.model and 'xmax' in self.model:
            x = np.clip(x, self.model['xmin'], self.model['xmax'])
        if 'ymin' in self.model and 'ymax' in self.model:
            y = np.clip(y, self.model['ymin'], self.model['ymax'])
        if 'zmin' in self.model and 'zmax' in self.model:
            z = np.clip(z, self.model['zmin'], self.model['zmax'])
        
        return {
            'x': x,
            'y': y,
            'z': z
        }