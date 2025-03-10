import numpy as np #bibliothéque pour effectuer les calculs numérique
import matplotlib.pyplot as plt #bibliotheque permet de créer des graphiques et visualisations
from PIL import Image  #bibliothéque permet de charger l'image


class TerrainModel:
    def __init__(self, terrain_file, threats, bounds, start_location, end_location, n):
        self.H = np.array(Image.open(terrain_file))
        self.H[self.H < 0] = 0

        self.MAPSIZE_Y, self.MAPSIZE_X = self.H.shape
        self.X, self.Y = np.meshgrid(np.arange(1, self.MAPSIZE_X+1 ), np.arange(1, self.MAPSIZE_Y+1 ))

        self.threats = threats
        self.bounds = bounds
        self.start_location = start_location
        self.end_location = end_location
        self.n = n

    def plot(self):
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')

        # Tracé du terrain
        ax.plot_surface(self.X, self.Y, self.H, cmap='summer', edgecolor='none', alpha=0.7)

        # Ajout des menaces sous forme de cylindres
        h = 250  # Hauteur des cylindres
        for threat in self.threats:
            threat_x, threat_y, threat_z, threat_radius = threat
            theta = np.linspace(0, 2 * np.pi, 100)
            z = np.linspace(threat_z, threat_z + h, 2)
            theta, z = np.meshgrid(theta, z)
            x = threat_radius * np.cos(theta) + threat_x
            y = threat_radius * np.sin(theta) + threat_y
            ax.plot_surface(x, y, z, color='red', alpha=0.3, edgecolor='none')

        # Ajustements des axes
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        plt.show()


# Initialisation des données
terrain_file = 'ChrismasTerrain.tif'
threats = [
    (400, 600, 100, 80),
    (600, 500, 150, 70),
    (500, 350, 150, 80),
    (350, 200, 150, 80),
    (700, 550, 150, 70),
    (650, 750, 150, 80)
]
bounds = {'xmin': 1, 'xmax': 1000, 'ymin': 1, 'ymax': 1000, 'zmin': 100, 'zmax': 200}  #limite de la carte
start_location = np.array([200, 100, 150])
end_location = np.array([800, 800, 150])
n = 22

# Création et affichage du modèle
model = TerrainModel(terrain_file, threats, bounds, start_location, end_location, n)
model.plot()