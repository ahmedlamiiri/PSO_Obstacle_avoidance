import osmnx as ox
import matplotlib.pyplot as plt

# Coordonnées du centre
latitude = 25.096695
longitude = 55.174876
radius_m = 300


# Tags pour les bâtiments
tags = {'building': True}

# Nouvelle fonction recommandée (features_from_point au lieu de geometries_from_point)
buildings = ox.features_from_point((latitude, longitude), tags=tags, dist=radius_m)

# Extraire les centres des polygones comme menaces
threats = []
for idx, row in buildings.iterrows():
    if row.geometry.geom_type == 'Polygon':
        centroid = row.geometry.centroid
        lon, lat = centroid.x, centroid.y
        threats.append((lon, lat, 150, 20))  # altitude arbitraire + rayon

print(f"{len(threats)} menaces détectées.")
for i, t in enumerate(threats[:1000], 1):
    print(f"Menace {i} → Longitude: {t[0]:.6f}, Latitude: {t[1]:.6f}")

# Visualisation optionnelle
buildings.plot()
plt.title("Bâtiments détectés (OSM)")
plt.show()
