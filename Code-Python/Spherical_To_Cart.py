import numpy as np


class SphericalToCartesian:
    def __init__(self, model):
        self.model = model

    def convert(self, sol):
        # Start location
        xs, ys, zs = self.model['start']

        # Solution in Spherical space
        r = sol['r']
        psi = sol['psi']
        phi = sol['phi']

        # First Cartesian coordinate
        x = [xs + r[0] * np.cos(psi[0]) * np.sin(phi[0])]
        y = [ys + r[0] * np.cos(psi[0]) * np.cos(phi[0])]
        z = [zs + r[0] * np.sin(psi[0])]

        # Check limits
        x[0] = min(max(x[0], self.model['xmin']), self.model['xmax'])
        y[0] = min(max(y[0], self.model['ymin']), self.model['ymax'])
        z[0] = min(max(z[0], self.model['zmin']), self.model['zmax'])

        # Next Cartesian coordinates
        for i in range(1, self.model['n']):
            x_i = x[i - 1] + r[i] * np.cos(psi[i]) * np.sin(phi[i])
            y_i = y[i - 1] + r[i] * np.cos(psi[i]) * np.cos(phi[i])
            z_i = z[i - 1] + r[i] * np.sin(psi[i])

            x.append(min(max(x_i, self.model['xmin']), self.model['xmax']))
            y.append(min(max(y_i, self.model['ymin']), self.model['ymax']))
            z.append(min(max(z_i, self.model['zmin']), self.model['zmax']))

        return {'x': x, 'y': y, 'z': z}