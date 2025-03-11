import numpy as np
import matplotlib.pyplot as plt
from CreateRandomSolution import RandomSolution
from PlotSolution import PlotSolution
from CreateModel import TerrainModel
from MyCost import PathCostCalculator
from Spherical_To_Cart import SphericalToCartesian


class PSO:
    def __init__(self, TerrainModel, max_it=200, n_pop=500):
        self.TerrainModel = TerrainModel
        self.max_it = max_it
        self.n_pop = n_pop
        self.w = 1.0
        self.w_damp = 0.98
        self.c1 = 1.5
        self.c2 = 1.5
        self.best_cost = np.zeros(max_it)

        # Create model dict for PathCostCalculator and SphericalToCartesian
        self.model_dict = {
            'H': TerrainModel.H,
            'n': TerrainModel.n,
            'start': TerrainModel.start_location,
            'end': TerrainModel.end_location,
            'threats': TerrainModel.threats,
            'xmin': 0,
            'xmax': TerrainModel.MAPSIZE_X - 1,
            'ymin': 0,
            'ymax': TerrainModel.MAPSIZE_Y - 1,
            'zmin': TerrainModel.bounds['zmin'],
            'zmax': TerrainModel.bounds['zmax']
        }

        self.cost_calculator = PathCostCalculator(self.model_dict)
        self.coordinate_converter = SphericalToCartesian(self.model_dict)

        # Variables bounds
        angle_range = np.pi / 4  # Limit angle range for better solutions
        vertical_angle_range = np.pi / 4  # Very small range for almost fixed altitude
        dir_vector = TerrainModel.end_location - TerrainModel.start_location
        phi0 = np.arctan2(dir_vector[1], dir_vector[0])

        self.var_min = {
            'r': 0,
            'psi': -vertical_angle_range,  # Reduced vertical angle range
            'phi': phi0 - angle_range
        }
        self.var_max = {
            'r': 2 * np.linalg.norm(TerrainModel.start_location - TerrainModel.end_location) / TerrainModel.n,
            'psi': vertical_angle_range,  # Reduced vertical angle range
            'phi': phi0 + angle_range
        }

        # Velocity bounds
        alpha = 0.5
        self.vel_min = {}
        self.vel_max = {}
        for key in self.var_min:
            self.vel_max[key] = alpha * (self.var_max[key] - self.var_min[key])
            self.vel_min[key] = -self.vel_max[key]

        # Initialize particles
        self.particles = []
        self.global_best = {'position': None, 'cost': np.inf}

        # Initialize particles with valid solution
        is_init = False
        print("Initialising...")
        while not is_init:
            self.particles = []
            for _ in range(n_pop):
                particle = self.create_particle()
                self.particles.append(particle)

                if particle['cost'] < self.global_best['cost']:
                    self.global_best = {
                        'position': particle['position'].copy(),
                        'cost': particle['cost']
                    }
                    is_init = True

    def create_particle(self):
        solution = RandomSolution(self.TerrainModel.n, self.var_min, self.var_max)
        position = solution.get_solution()

        # Initialize velocity components separately
        velocity = {
            'r': np.zeros(self.TerrainModel.n),
            'psi': np.zeros(self.TerrainModel.n),
            'phi': np.zeros(self.TerrainModel.n)
        }

        # Convert spherical to cartesian for cost calculation
        cart_position = self.coordinate_converter.convert(position)
        cost = self.cost_calculator.calculate_cost(cart_position)

        return {
            'position': position,
            'velocity': velocity,
            'cost': cost,
            'best': {
                'position': position.copy(),
                'cost': cost
            }
        }

    def optimize(self):
        # Initialize best_cost array
        self.best_cost = np.full(self.max_it, np.inf)

        for it in range(self.max_it):
            # Store the current best cost
            self.best_cost[it] = self.global_best['cost']

            for particle in self.particles:
                # Update each component separately
                for key in ['r', 'psi', 'phi']:
                    # Update Velocity
                    r1, r2 = np.random.rand(self.TerrainModel.n), np.random.rand(self.TerrainModel.n)
                    particle['velocity'][key] = (
                            self.w * particle['velocity'][key] +
                            self.c1 * r1 * (particle['best']['position'][key] - particle['position'][key]) +
                            self.c2 * r2 * (self.global_best['position'][key] - particle['position'][key])
                    )

                    # Update Velocity Bounds
                    particle['velocity'][key] = np.clip(particle['velocity'][key], self.vel_min[key], self.vel_max[key])

                    # Update Position
                    particle['position'][key] += particle['velocity'][key]

                    # Velocity Mirroring
                    out_of_range = (particle['position'][key] < self.var_min[key]) | (
                                particle['position'][key] > self.var_max[key])
                    particle['velocity'][key][out_of_range] = -particle['velocity'][key][out_of_range]

                    # Update Position Bounds
                    particle['position'][key] = np.clip(particle['position'][key], self.var_min[key], self.var_max[key])

                # Evaluate new position
                cart_position = self.coordinate_converter.convert(particle['position'])
                particle['cost'] = self.cost_calculator.calculate_cost(cart_position)

                # Update Personal Best
                if particle['cost'] < particle['best']['cost']:
                    particle['best']['position'] = particle['position'].copy()
                    particle['best']['cost'] = particle['cost']

                    # Update Global Best
                    if particle['best']['cost'] < self.global_best['cost']:
                        self.global_best = {
                            'position': particle['position'].copy(),
                            'cost': particle['cost']
                        }

            # Inertia Weight Damping
            self.w *= self.w_damp
            print(f'Iteration {it + 1}: Best Cost = {self.best_cost[it]}')

    def plot_results(self):
        best_position = self.global_best['position']
        print("Best solution:", best_position)

        # Convert best position from spherical to cartesian
        cart_position = self.coordinate_converter.convert(best_position)

        # Plot the terrain and path
        plot_model = self.TerrainModel  # Directly use the TerrainModel object
        plotter = PlotSolution(cart_position, plot_model)
        plotter.plot_solution()

        # Superpose le chemin optimal sur le terrain
        x, y, z = zip(*cart_position)
        plt.plot(x, y, z, color='red', marker='o', linestyle='-', linewidth=2, label='Optimal Path')
        plt.legend()

        # Plot the convergence curve
        plt.figure()
        plt.plot(range(1, self.max_it + 1), self.best_cost, 'b-', linewidth=2)
        plt.xlabel('Iteration')
        plt.ylabel('Best Cost')
        plt.title('PSO Convergence Curve')
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    # Import the configuration from your model or any other necessary file
    from CreateModel import terrain_file, threats, bounds, start_location, end_location, n

    terrain_model = TerrainModel(terrain_file, threats, bounds, start_location, end_location, n)
    pso = PSO(terrain_model)
    pso.optimize()
    pso.plot_results()