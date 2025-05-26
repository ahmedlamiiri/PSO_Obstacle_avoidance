import numpy as np
import matplotlib.pyplot as plt
import rasterio
import osmnx as ox
import geopandas as gpd


class TerrainModel:
    def __init__(self, terrain_file, threats, bounds, start_location, end_location, n):
        with rasterio.open(terrain_file) as src:
            self.H = src.read(1)
            self.transform = src.transform
            self.crs = src.crs
            height, width = self.H.shape
            x = np.arange(0, width) * self.transform.a + self.transform.c
            y = np.arange(0, height) * self.transform.e + self.transform.f
            self.X, self.Y = np.meshgrid(x, y)

        self.H[self.H < 0] = 0

        self.threats = threats
        self.bounds = bounds
        self.start_location = start_location
        self.end_location = end_location
        self.n = n
        self.MAPSIZE_X = width
        self.MAPSIZE_Y = height

    def plot(self):
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot_surface(self.X, self.Y, self.H, cmap='summer', edgecolor='none', alpha=0.7)

        h = 100  # hauteur arbitraire des menaces
        for threat in self.threats:
            threat_x, threat_y, threat_z, threat_radius = threat
            theta = np.linspace(0, 2 * np.pi, 100)
            z = np.linspace(threat_z, threat_z + h, 2)
            theta, z = np.meshgrid(theta, z)
            x = threat_radius  * np.cos(theta) + threat_x 
            y = threat_radius * np.sin(theta) + threat_y 
            ax.plot_surface(x, y, z, color='red', alpha=0.3, edgecolor='none')

        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_zlabel('Altitude [m]')
        plt.title("Modèle de terrain avec menaces (OSM)")
        plt.show()


# === Chargement du fichier TIF ===
terrain_file = 'Dubai.tif'  # Assure-toi que ce fichier est bien en EPSG:4326

# === Extraction des bâtiments OpenStreetMap ===
latitude = 25.096695
longitude = 55.174876
radius_m = 300
tags = {'building': True}

# Récupérer les bâtiments dans un rayon donné
buildings = ox.features_from_point((latitude, longitude), tags=tags, dist=radius_m)

# Filtrer les polygones uniquement
buildings = buildings[buildings.geometry.type == 'Polygon']

# Les géométries sont déjà en EPSG:4326 (GPS), donc pas besoin de reprojection
# Calculer le centre et le rayon équivalent de chaque bâtiment
threats = []
for geom in buildings.geometry:
    if geom.is_valid and not geom.is_empty:
        centroid = geom.centroid
        area = geom.area  # en degrés² car CRS = EPSG:4326
        # On suppose que la géométrie est en degrés ; on convertit approximativement l'aire en mètres²
        area_m2 = area * (111_320 ** 2)  # approx : 1 deg ≈ 111.32 km
        radius_m = np.sqrt(area_m2 / np.pi)  # rayon en mètres
        radius_deg = radius_m / 111_320  # conversion inverse pour rester en degrés
        threats.append((centroid.x, centroid.y, 0, radius_deg))

print(f"{len(threats)} menaces détectées.")
for i, t in enumerate(threats[:1000], 1):
    print(f"Menace {i} → Longitude: {t[0]:.6f}, Latitude: {t[1]:.6f}")

# === Définir les bornes et points de départ/arrivée (en GPS aussi) ===
bounds = {
    'xmin': 55.172, 'xmax': 55.178,  # longitude
    'ymin': 25.095, 'ymax': 25.098,  # latitude
    'zmin': 150, 'zmax': 250      # altitude
}

start_location = np.array([55.173017, 25.097035,200])  # Longitude, Latitude, Alt
end_location = np.array([55.177901, 25.096269, 200])
n = 100

# === Création du modèle ===
model = TerrainModel(terrain_file, threats, bounds, start_location, end_location, n)
model.plot()