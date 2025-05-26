import numpy as np
import matplotlib.pyplot as plt
import geopandas as gpd
from shapely.geometry import Point
import json
import airsim  # Ajout de l'import pour communication avec AirSim
from CreateRandomSolution import RandomSolution
from PlotSolution import PlotSolution
from CreateModel import TerrainModel
from MyCost import PathCostCalculator
from Spherical_To_Cart import SphericalToCartesian

class PSO:
    def __init__(self, TerrainModel, max_it=5, n_pop=500):
        self.TerrainModel = TerrainModel
        self.max_it = max_it
        self.n_pop = n_pop
        self.w = 1.0
        self.w_damp = 0.98
        self.c1 = 1.5
        self.c2 = 1.5
        self.best_cost = np.zeros(max_it)

        self.model_dict = {
            'H': TerrainModel.H,
            'n': TerrainModel.n,
            'start': np.asarray(TerrainModel.start_location).flatten(),
            'end': np.asarray(TerrainModel.end_location).flatten(),
            'threats': TerrainModel.threats,
            'xmin': TerrainModel.bounds['xmin'],
            'xmax': TerrainModel.bounds['xmax'],
            'ymin': TerrainModel.bounds['ymin'],
            'ymax': TerrainModel.bounds['ymax'],
            'zmin': TerrainModel.bounds['zmin'],
            'zmax': TerrainModel.bounds['zmax'],
            'transform': TerrainModel.transform
        }
        self.cost_calculator = PathCostCalculator(self.model_dict)
        self.coordinate_converter = SphericalToCartesian(self.model_dict)

        angle_range = np.pi/4
        vertical_angle_range = np.pi/4
        dir_vector = TerrainModel.end_location - TerrainModel.start_location
        phi0 = np.arctan2(dir_vector[1], dir_vector[0])

        self.var_min = {
            'r': 0.0,
            'psi': -vertical_angle_range,
            'phi': phi0 - angle_range
        }
        self.var_max = {
            'r': 2 * np.linalg.norm(dir_vector) / TerrainModel.n,
            'psi': vertical_angle_range,
            'phi': phi0 + angle_range
        }

        alpha = 0.5
        self.vel_min = {k: -alpha * (self.var_max[k] - self.var_min[k]) for k in self.var_min}
        self.vel_max = {k: alpha * (self.var_max[k] - self.var_min[k]) for k in self.var_max}

        self.particles = []
        self.global_best = {'position': None, 'cost': np.inf}
        self.initialize_particles()

    def initialize_particles(self):
        print("Initializing particles...")
        valid_particles = 0
        while valid_particles < self.n_pop:
            particle = self.create_particle()
            # On considère valide si coût fini (non inf)
            if particle['cost'] < np.inf:
                self.particles.append(particle)
                if particle['cost'] < self.global_best['cost']:
                    self.global_best = {
                        'position': {k: v.copy() for k, v in particle['position'].items()},
                        'cost': particle['cost']
                    }
                valid_particles += 1

    def create_particle(self):
        solution = RandomSolution(self.TerrainModel.n, self.var_min, self.var_max)
        position = solution.get_solution()  # dictionnaire de vecteurs np.array

        velocity = {
            'r': np.random.uniform(self.vel_min['r'], self.vel_max['r'], self.TerrainModel.n),
            'psi': np.random.uniform(self.vel_min['psi'], self.vel_max['psi'], self.TerrainModel.n),
            'phi': np.random.uniform(self.vel_min['phi'], self.vel_max['phi'], self.TerrainModel.n)
        }

        cart_position = self.coordinate_converter.convert(position)
        cost = self.cost_calculator.calculate_cost(cart_position)

        return {
            'position': {k: v.copy() for k, v in position.items()},
            'velocity': velocity,
            'cost': cost,
            'best': {
                'position': {k: v.copy() for k, v in position.items()},
                'cost': cost
            }
        }

    def optimize(self):
        print("Starting optimization...")
        for it in range(self.max_it):
            for particle in self.particles:
                for key in ['r', 'psi', 'phi']:
                    r1 = np.random.rand(self.TerrainModel.n)
                    r2 = np.random.rand(self.TerrainModel.n)

                    # Mettre à jour la vitesse
                    cognitive = self.c1 * r1 * (particle['best']['position'][key] - particle['position'][key])
                    social = self.c2 * r2 * (self.global_best['position'][key] - particle['position'][key])
                    particle['velocity'][key] = (
                        self.w * particle['velocity'][key]
                        + cognitive
                        + social
                    )

                    # Limiter la vitesse
                    particle['velocity'][key] = np.clip(particle['velocity'][key], self.vel_min[key], self.vel_max[key])

                    # Mettre à jour la position
                    particle['position'][key] += particle['velocity'][key]

                    # Gérer les positions hors limites
                    out_of_bounds_low = particle['position'][key] < self.var_min[key]
                    out_of_bounds_high = particle['position'][key] > self.var_max[key]

                    # Inverser la vitesse pour particules hors limites
                    particle['velocity'][key][out_of_bounds_low | out_of_bounds_high] *= -1

                    # Recaler la position dans les bornes
                    particle['position'][key] = np.clip(particle['position'][key], self.var_min[key], self.var_max[key])

                cart_position = self.coordinate_converter.convert(particle['position'])
                particle['cost'] = self.cost_calculator.calculate_cost(cart_position)

                if particle['cost'] < particle['best']['cost']:
                    particle['best']['position'] = {k: v.copy() for k, v in particle['position'].items()}
                    particle['best']['cost'] = particle['cost']

                    if particle['best']['cost'] < self.global_best['cost']:
                        self.global_best = {
                            'position': {k: v.copy() for k, v in particle['position'].items()},
                            'cost': particle['cost']
                        }

            self.w *= self.w_damp
            self.best_cost[it] = self.global_best['cost']
            print(f"Iteration {it+1}/{self.max_it} - Best Cost: {self.global_best['cost']:.4f}")

    def plot_results(self):
        best_position = self.global_best['position']
        cart_position = self.coordinate_converter.convert(best_position)

        plotter = PlotSolution(cart_position, self.TerrainModel)
        plotter.plot_solution()

        plt.figure()
        plt.plot(range(1, self.max_it+1), self.best_cost[:self.max_it], 'b-', linewidth=2)
        plt.xlabel('Iteration')
        plt.ylabel('Best Cost')
        plt.title('PSO Convergence')
        plt.grid(True)
        plt.show()

    def convert_gps_to_ned(self, lon, lat, alt, origin_lon, origin_lat, origin_alt):
        # Déterminer automatiquement la bonne zone UTM
        utm_zone = int((origin_lon + 180) / 6) + 1
        epsg_code = 32600 + utm_zone  # Hémisphère nord ; pour sud, utilise 32700 + zone

        gdf = gpd.GeoDataFrame(geometry=[
            Point(origin_lon, origin_lat),
            Point(lon, lat)
        ], crs="EPSG:4326")

        gdf_utm = gdf.to_crs(f"EPSG:{epsg_code}")

        origin_xy = gdf_utm.iloc[0].geometry
        target_xy = gdf_utm.iloc[1].geometry

        x = target_xy.x - origin_xy.x     # vers le nord
        y = -(target_xy.y - origin_xy.y)  # vers l’est (inversé car NED)
        z = -(alt - origin_alt)      # vers le bas

        return x, y, z

    def export_to_airsim(self, filename="airsim_path.json", coord_type="NED"):
        best_position = self.global_best['position']
        cart_position = self.coordinate_converter.convert(best_position)

        x = np.concatenate([[self.model_dict['start'][0]], cart_position['x'], [self.model_dict['end'][0]]])
        y = np.concatenate([[self.model_dict['start'][1]], cart_position['y'], [self.model_dict['end'][1]]])
        z = np.concatenate([[self.model_dict['start'][2]], cart_position['z'], [self.model_dict['end'][2]]])

        origin_lon = self.model_dict['start'][0]
        origin_lat = self.model_dict['start'][1]
        origin_alt = self.model_dict['start'][2]

        waypoints = []
        for i in range(len(x)):
            if coord_type == "NED":
                ned_x, ned_y, ned_z = self.convert_gps_to_ned(x[i], y[i], z[i], origin_lon, origin_lat, origin_alt)
                waypoints.append({
                    "x": float(ned_x),
                    "y": float(ned_y),
                    "z": float(ned_z),
                    "speed": 3.0
                })
            else:
                waypoints.append({
                    "lon": float(x[i]),
                    "lat": float(y[i]),
                    "alt": float(z[i]),
                    "speed": 3.0
                })

        trajectory = {
            "version": 1.0,
            "coord_type": coord_type,
            "waypoints": waypoints
        }

        with open(filename, 'w') as f:
            json.dump(trajectory, f, indent=2)

        print(f"Trajectory exported to {filename} ({coord_type} coordinates)")

    def send_trajectory_to_airsim(self, coord_type="NED", vehicle_name=""):
        """
        Envoie la trajectoire calculée directement à AirSim via client Python,
        en faisant suivre au drone les waypoints générés.
       
        :param coord_type: "NED" ou "GPS" pour le type de coordonnées
        :param vehicle_name: Nom du véhicule dans AirSim si plusieurs drones (optionnel)
        """
        best_position = self.global_best['position']
        cart_position = self.coordinate_converter.convert(best_position)

        x = np.concatenate([[self.model_dict['start'][0]], cart_position['x'], [self.model_dict['end'][0]]])
        y = np.concatenate([[self.model_dict['start'][1]], cart_position['y'], [self.model_dict['end'][1]]])
        z = np.concatenate([[self.model_dict['start'][2]], cart_position['z'], [self.model_dict['end'][2]]])

        origin_lon = self.model_dict['start'][0]
        origin_lat = self.model_dict['start'][1]
        origin_alt = self.model_dict['start'][2]

        # Connexion au client AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        if vehicle_name:
            client.enableApiControl(True, vehicle_name)
            client.armDisarm(True, vehicle_name)
        else:
            client.enableApiControl(True)
            client.armDisarm(True)

        print("Sending waypoints to AirSim...")

        for i in range(len(x)):
            if coord_type == "NED":
                ned_x, ned_y, ned_z = self.convert_gps_to_ned(x[i], y[i], z[i], origin_lon, origin_lat, origin_alt)
                # Créer position de type Vector3r pour AirSim
                position = airsim.Vector3r(ned_x, ned_y, ned_z)
            else:
                # Note : AirSim n'utilise pas nativement le GPS dans ce mode, conversion nécessaire.
                # Pour simplicité, on envoie les coordonnées GPS brutes ici (non recommandé)
                position = airsim.Vector3r(x[i], y[i], z[i])

            speed = 3.0  # vitesse en m/s
            # Envoyer le drone à chaque waypoint séquentiellement avec moveToPositionAsync
            if vehicle_name:
                client.moveToPositionAsync(position.x_val, position.y_val, position.z_val, speed, vehicle_name=vehicle_name).join()
            else:
                client.moveToPositionAsync(position.x_val, position.y_val, position.z_val, speed).join()

            print(f"Waypoint {i+1}/{len(x)} sent: x={position.x_val:.2f}, y={position.y_val:.2f}, z={position.z_val:.2f}")

        print("All waypoints sent.")

        # Libération du contrôle si nécessaire
        if vehicle_name:
            client.armDisarm(False, vehicle_name)
            client.enableApiControl(False, vehicle_name)
        else:
            client.armDisarm(False)
            client.enableApiControl(False)

if __name__ == "__main__":
    from CreateModel import terrain_file, threats, bounds, start_location, end_location, n

    terrain_model = TerrainModel(terrain_file, threats, bounds, start_location, end_location, n)

    pso = PSO(terrain_model)
    pso.optimize()
    pso.plot_results()
    pso.export_to_airsim("trajectory_gps.json", coord_type="GPS")
    pso.export_to_airsim("trajectory_ned.json", coord_type="NED")

    # Envoi direct de la trajectoire à AirSim (ici en NED)
    pso.send_trajectory_to_airsim(coord_type="NED")