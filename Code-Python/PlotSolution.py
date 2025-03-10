import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
from CreateModel import TerrainModel
# Assuming TerrainModel is defined above as given

class PlotSolution:
    def __init__(self, sol, model, smooth_factor=0.95):
        self.sol = sol
        self.model = model
        self.smooth_factor = smooth_factor

    def smooth_path(self, points, num_points=22):
        # Ensure there are enough points for smoothing
        npts = points.shape[1]
        if npts < 4:
            print("Not enough points for B-spline smoothing, returning original points.")
            return points

        # Create parameter array for the spline
        t = np.linspace(0, 1, npts)
        t_smooth = np.linspace(0, 1, num_points)

        # Initialize smoothed points array
        smoothed_points = np.zeros((points.shape[0], num_points))

        # Smooth each dimension separately
        for i in range(points.shape[0]):
            try:
                spl = make_interp_spline(t, points[i, :], k=3)
                smoothed_points[i, :] = spl(t_smooth)
            except ValueError:
                print(f"Warning: B-spline interpolation failed for dimension {i}. Using linear interpolation.")
                smoothed_points[i, :] = np.interp(t_smooth, t, points[i, :])

        return smoothed_points
        # Smooth each dimension separately
        for i in range(points.shape[0]):
            # Create B-spline
            spl = make_interp_spline(t, points[i, :], k=3)
            # Evaluate B-spline
            smoothed_points[i, :] = spl(t_smooth)

        return smoothed_points

    def add_takeoff_landing(self, points):
        """Add vertical takeoff and landing phases to the path"""
        x, y, z = points
        xs, ys, zs = self.model.start_location
        xf, yf, zf = self.model.end_location

        # Get terrain height at start and end points
        H = self.model.H
        terrain_height_start = H[int(np.clip(round(ys), 0, H.shape[0] - 1)),
        int(np.clip(round(xs), 0, H.shape[1] - 1))]
        terrain_height_end = H[int(np.clip(round(yf), 0, H.shape[0] - 1)),
        int(np.clip(round(xf), 0, H.shape[1] - 1))]

        # Create vertical takeoff points (10 points)
        t_takeoff = np.linspace(0, 1, 10)
        x_takeoff = np.full_like(t_takeoff, xs)
        y_takeoff = np.full_like(t_takeoff, ys)
        z_takeoff = terrain_height_start + t_takeoff * (z[0] - terrain_height_start)

        # Create vertical landing points (10 points)
        t_landing = np.linspace(0, 1, 10)
        x_landing = np.full_like(t_landing, xf)
        y_landing = np.full_like(t_landing, yf)
        z_landing = z[-1] + (1 - t_landing) * (terrain_height_end - z[-1])

        # Combine all points
        x_full = np.concatenate([x_takeoff, x, x_landing])
        y_full = np.concatenate([y_takeoff, y, y_landing])
        z_full = np.concatenate([z_takeoff, z, z_landing])

        return np.array([x_full, y_full, z_full])

    def plot_solution(self):
        fig = plt.figure(figsize=(15, 10))

        # 3D View
        ax = fig.add_subplot(131, projection='3d')
        X, Y, H = self.model.X, self.model.Y, self.model.H
        x, y, z = self.sol['x'], self.sol['y'], self.sol['z']
        xs, ys, zs = self.model.start_location
        xf, yf, zf = self.model.end_location
        threats = self.model.threats

        # Concatenate start and end points
        x_all = np.concatenate(([xs], x, [xf]))
        y_all = np.concatenate(([ys], y, [yf]))
        z_all = np.concatenate(([zs], z, [zf]))

        # Calculate absolute altitude including terrain height
        z_abs = np.array([z_all[i] + H[int(np.clip(round(y_all[i]), 0, H.shape[0] - 1)),
        int(np.clip(round(x_all[i]), 0, H.shape[1] - 1))]
                          for i in range(len(x_all))])

        # Smooth the path
        points = np.array([x_all, y_all, z_abs])
        smoothed_points = self.smooth_path(points)

        # Add takeoff and landing phases
        final_points = self.add_takeoff_landing(smoothed_points)

        # Plot terrain surface
        ax.plot_surface(X, Y, H, cmap='summer', alpha=0.5)

        # Plot complete path with different colors for takeoff, cruise, and landing
        n_points = len(final_points[0])
        takeoff_end = 10
        cruise_end = n_points - 10

        # Plot takeoff phase in green
        ax.plot(final_points[0, :takeoff_end],
                final_points[1, :takeoff_end],
                final_points[2, :takeoff_end], 'g-', linewidth=2, label='Takeoff')

        # Plot cruise phase in blue
        ax.plot(final_points[0, takeoff_end:cruise_end],
                final_points[1, takeoff_end:cruise_end],
                final_points[2, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')

        # Plot landing phase in red
        ax.plot(final_points[0, cruise_end:],
                final_points[1, cruise_end:],
                final_points[2, cruise_end:], 'r-', linewidth=2, label='Landing')

        ax.scatter([xs, xf], [ys, yf], [H[int(round(ys)), int(round(xs))],
                                        H[int(round(yf)), int(round(xf))]],
                   color='black', s=100, label='Start/End')

        # Plot threats/obstacles
        if threats:
            for threat in threats:
                x, y, z, radius = threat
                height = 250  # Fixed height of threats (cylindrical shape)

                # Create cylinder for threat visualization
                u = np.linspace(0, 2 * np.pi, 20)
                v = np.linspace(0, np.pi, 20)
                x_sphere = x + radius * np.outer(np.cos(u), np.sin(v))
                y_sphere = y + radius * np.outer(np.sin(u), np.sin(v))
                z_sphere = z + height * np.outer(np.ones(np.size(u)), np.cos(v))


                ax.plot_surface(x_sphere, y_sphere, z_sphere, color='red', alpha=0.3)

        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.set_title('3D View')

        # Top View (2D)
        ax_top = fig.add_subplot(132)
        ax_top.contour(X, Y, H, levels=20, cmap='summer')

        # Plot complete path in top view
        ax_top.plot(final_points[0, :takeoff_end],
                    final_points[1, :takeoff_end], 'g-', linewidth=2, label='Takeoff')
        ax_top.plot(final_points[0, takeoff_end:cruise_end],
                    final_points[1, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')
        ax_top.plot(final_points[0, cruise_end:],
                    final_points[1, cruise_end:], 'r-', linewidth=2, label='Landing')

        ax_top.scatter([xs, xf], [ys, yf], color='black', s=100, label='Start/End')

        # Plot threats in top view
        if threats:
            for threat in threats:
                x, y, z, radius = threat
                circle = plt.Circle((x, y), radius, color='red', alpha=0.3)
                ax_top.add_artist(circle)

        ax_top.legend()
        ax_top.set_xlabel('x [m]')
        ax_top.set_ylabel('y [m]')
        ax_top.set_title('Top View')

        # Side View
        ax_side = fig.add_subplot(133)

        # Plot complete path in side view
        ax_side.plot(final_points[0, :takeoff_end],
                     final_points[2, :takeoff_end], 'g-', linewidth=2, label='Takeoff')
        ax_side.plot(final_points[0, takeoff_end:cruise_end],
                     final_points[2, takeoff_end:cruise_end], 'b-', linewidth=2, label='Cruise')
        ax_side.plot(final_points[0, cruise_end:],
                     final_points[2, cruise_end:], 'r-', linewidth=2, label='Landing')

        ax_side.scatter([xs, xf],
                        [H[int(round(ys)), int(round(xs))],
                         H[int(round(yf)), int(round(xf))]],
                        color='black', s=100, label='Start/End')


        ax_side.legend()
        ax_side.set_xlabel('x [m]')
        ax_side.set_ylabel('z [m]')
        ax_side.set_title('Side View')

        plt.tight_layout()
        plt.show()


# Test code (only runs when script is run directly)
if __name__ == "__main__":
    # Create the TerrainModel instance
    terrain_file = 'ChrismasTerrain.tif'
    threats = [
        (400, 500, 100, 80),
        (600, 200, 150, 70),
        (500, 350, 150, 80),
        (300, 200, 150, 80),
        (700, 550, 150, 70),
        (650, 750, 150, 80)
    ]
    bounds = {'xmin': 1, 'xmax': 1000, 'ymin': 1, 'ymax': 1000, 'zmin': 100, 'zmax': 200}  # Limits
    start_location = np.array([200, 100, 150])
    end_location = np.array([800, 800, 150])
    n = 22

    # Create TerrainModel
    model = TerrainModel(terrain_file, threats, bounds, start_location, end_location, n)

    # Test solution (simple path)
    t = np.linspace(0, 1, 10)
    sol = {
        'x': 100 + 601 * t,
        'y': 100 + 600 * t,
        'z': 150 * np.ones_like(t)
    }

    # Plot solution
    plotter = PlotSolution(sol, model)
    plotter.plot_solution()