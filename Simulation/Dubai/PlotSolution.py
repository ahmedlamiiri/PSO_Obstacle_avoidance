import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D
import rasterio

class PlotSolution:
    def __init__(self, sol, model, smooth_factor=0.95):
        self.sol = sol
        self.model = model
        self.smooth_factor = smooth_factor

    def lonlat_to_rowcol(self, lon, lat):
        # Convertit coordonnées GPS (lon, lat) en indices (row, col) dans la matrice H
        # Utilise l'inverse de la transformation rasterio
        row, col = rasterio.transform.rowcol(self.model.transform, lon, lat)
        # Clamp indices dans les bornes
        row = np.clip(row, 0, self.model.H.shape[0] - 1)
        col = np.clip(col, 0, self.model.H.shape[1] - 1)
        return row, col

    def get_terrain_height(self, lon, lat):
        row, col = self.lonlat_to_rowcol(lon, lat)
        return self.model.H[row, col]

    def smooth_path(self, points, num_points=50):
        npts = points.shape[1]
        if npts < 4:
            return points
        t = np.linspace(0, 1, npts)
        t_smooth = np.linspace(0, 1, num_points)
        smoothed_points = np.zeros((points.shape[0], num_points))
        for i in range(points.shape[0]):
            try:
                spl = make_interp_spline(t, points[i, :], k=3)
                smoothed_points[i, :] = spl(t_smooth)
            except ValueError:
                smoothed_points[i, :] = np.interp(t_smooth, t, points[i, :])
        return smoothed_points

    def add_takeoff_landing(self, points):
        x, y, z = points
        xs, ys, zs = self.model.start_location
        xf, yf, zf = self.model.end_location

        terrain_height_start = self.get_terrain_height(xs, ys)
        terrain_height_end = self.get_terrain_height(xf, yf)

        t_takeoff = np.linspace(0, 1, 10)
        x_takeoff = np.full_like(t_takeoff, xs)
        y_takeoff = np.full_like(t_takeoff, ys)
        z_takeoff = terrain_height_start + t_takeoff * (z[0] - terrain_height_start)

        t_landing = np.linspace(0, 1, 10)
        x_landing = np.full_like(t_landing, xf)
        y_landing = np.full_like(t_landing, yf)
        z_landing = z[-1] + (1 - t_landing) * (terrain_height_end - z[-1])

        x_full = np.concatenate([x_takeoff, x, x_landing])
        y_full = np.concatenate([y_takeoff, y, y_landing])
        z_full = np.concatenate([z_takeoff, z, z_landing])

        return np.array([x_full, y_full, z_full])

    def plot_solution(self):
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(131, projection='3d')

        X, Y, H = self.model.X, self.model.Y, self.model.H
        x, y, z = self.sol['x'], self.sol['y'], self.sol['z']
        xs, ys, zs = self.model.start_location
        xf, yf, zf = self.model.end_location
        threats = self.model.threats

        # Concat start/end points GPS
        x_all = np.concatenate(([xs], x, [xf]))
        y_all = np.concatenate(([ys], y, [yf]))
        z_all = np.concatenate(([zs], z, [zf]))

        # Calcul altitude absolue = altitude relative + altitude terrain
        z_abs = np.array([self.get_terrain_height(x_all[i], y_all[i]) + z_all[i] for i in range(len(x_all))])

        points = np.array([x_all, y_all, z_abs])
        smoothed_points = self.smooth_path(points)
        final_points = self.add_takeoff_landing(smoothed_points)

        # Plot terrain
        ax.plot_surface(X, Y, H, cmap='summer', alpha=0.5)

        n_points = len(final_points[0])
        takeoff_end = 10
        cruise_end = n_points - 10

        ax.plot(final_points[0, :takeoff_end], final_points[1, :takeoff_end], final_points[2, :takeoff_end], 'g-', linewidth=2, label='Takeoff')
        ax.plot(final_points[0, takeoff_end:cruise_end], final_points[1, takeoff_end:cruise_end], final_points[2, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')
        ax.plot(final_points[0, cruise_end:], final_points[1, cruise_end:], final_points[2, cruise_end:], 'r-', linewidth=2, label='Landing')

        ax.scatter([xs, xf], [ys, yf], [self.get_terrain_height(xs, ys), self.get_terrain_height(xf, yf)], color='black', s=100, label='Start/End')

        if threats:
            for threat in threats:
                tx, ty, tz, radius = threat
                height = 250
                # Adapt coordinates en GPS, rayon aussi (approximation)
                u = np.linspace(0, 2 * np.pi, 20)
                v = np.linspace(0, np.pi, 20)
                x_sphere = tx + radius * np.outer(np.cos(u), np.sin(v))
                y_sphere = ty + radius * np.outer(np.sin(u), np.sin(v))
                z_sphere = tz + height * np.outer(np.ones(np.size(u)), np.cos(v))
                ax.plot_surface(x_sphere, y_sphere, z_sphere, color='red', alpha=0.3)

        ax.legend()
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_zlabel('Altitude [m]')
        ax.set_title('3D View')

        # Vue du dessus (2D)
        ax_top = fig.add_subplot(132)
        ax_top.contour(X, Y, H, levels=20, cmap='summer')

        ax_top.plot(final_points[0, :takeoff_end], final_points[1, :takeoff_end], 'g-', linewidth=2, label='Takeoff')
        ax_top.plot(final_points[0, takeoff_end:cruise_end], final_points[1, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')
        ax_top.plot(final_points[0, cruise_end:], final_points[1, cruise_end:], 'r-', linewidth=2, label='Landing')
        ax_top.scatter([xs, xf], [ys, yf], color='black', s=100, label='Start/End')

        if threats:
            for threat in threats:
                tx, ty, tz, radius = threat
                circle = plt.Circle((tx, ty), radius, color='red', alpha=0.3)
                ax_top.add_artist(circle)

        ax_top.legend()
        ax_top.set_xlabel('Longitude')
        ax_top.set_ylabel('Latitude')
        ax_top.set_title('Top View')

        # Vue latérale
        ax_side = fig.add_subplot(133)
        ax_side.plot(final_points[0, :takeoff_end], final_points[2, :takeoff_end], 'g-', linewidth=2, label='Takeoff')
        ax_side.plot(final_points[0, takeoff_end:cruise_end], final_points[2, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')
        ax_side.plot(final_points[0, cruise_end:], final_points[2, cruise_end:], 'r-', linewidth=2, label='Landing')

        ax_side.scatter([xs, xf], [self.get_terrain_height(xs, ys), self.get_terrain_height(xf, yf)], color='black', s=100, label='Start/End')

        ax_side.legend()
        ax_side.set_xlabel('Longitude')
        ax_side.set_ylabel('Altitude [m]')
        ax_side.set_title('Side View')

        plt.tight_layout()
        plt.show()
