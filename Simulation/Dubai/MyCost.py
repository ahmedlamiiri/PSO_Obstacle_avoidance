import numpy as np
import rasterio

class PathCostCalculator:
    def __init__(self, model):
        self.model = model
        self.J_inf = 1e10  # Large value instead of infinity for better handling
        self.n = model['n']
        self.H = model['H']
        self.transform = model.get('transform', None)
        
        # Paramètres ajustables
        self.drone_size = 0.0001  # en degrés GPS (~10m)
        self.danger_dist = 5 * self.drone_size  # distance de sécurité
        self.turning_max = 45  # angle max de virage en degrés
        self.climb_max = 30   # angle max de montée/descente en degrés
        
        # Coefficients de pondération
        self.b1 = 5  # poids pour la distance totale
        self.b2 = 1  # poids pour les menaces
        self.b3 = 10  # poids pour l'altitude
        self.b4 = 1 # poids pour les angles

    def gps_to_matrix(self, lon, lat):
        """Convertit des coordonnées GPS (lon, lat) en indices matriciels (row, col)"""
        if self.transform:
            row, col = rasterio.transform.rowcol(self.transform, lon, lat)
            row = np.clip(row, 0, self.H.shape[0] - 1)
            col = np.clip(col, 0, self.H.shape[1] - 1)
            return row, col
        return int(round(lat)), int(round(lon))  # fallback si pas de transform

    def dist_p2s(self, x, a, b):
        """Distance point à segment (en coordonnées GPS)"""
        d_ab = np.linalg.norm(a - b)
        d_ax = np.linalg.norm(a - x)
        d_bx = np.linalg.norm(b - x)

        if d_ab != 0:
            if np.dot(a - b, x - b) * np.dot(b - a, x - a) >= 0:
                A = np.array([b - a, x - a])
                dist = abs(np.linalg.det(A)) / d_ab
            else:
                dist = min(d_ax, d_bx)
        else:
            dist = d_ax
        return dist

    def calculate_terrain_height(self, lon, lat):
        """Obtient l'altitude du terrain pour des coordonnées GPS"""
        row, col = self.gps_to_matrix(lon, lat)
        return self.H[row, col]

    def calculate_cost(self, sol):
        # Convertir toutes les entrées en numpy arrays 1D
        x = np.asarray(sol['x']).flatten()
        y = np.asarray(sol['y']).flatten()
        z = np.asarray(sol['z']).flatten()
        
        # Extraire les coordonnées de départ et arrivée
        xs, ys, zs = self.model['start']
        xf, yf, zf = self.model['end']
        
        # Construction des chemins complets
        x_all = np.concatenate([[xs], x, [xf]])
        y_all = np.concatenate([[ys], y, [yf]])
        z_all = np.concatenate([[zs], z, [zf]])
        
        # Vérification des dimensions
        if len(x_all) != len(y_all) or len(y_all) != len(z_all):
            return self.J_inf

        N = len(x_all)
        
        # Calcul des altitudes absolues
        z_abs = np.zeros(N)
        for i in range(N):
            terrain_height = self.calculate_terrain_height(x_all[i], y_all[i])
            z_abs[i] = z_all[i] + terrain_height

        # 1. Coût de la distance totale (J1)
        J1 = 0
        for i in range(N-1):
            dx = x_all[i+1] - x_all[i]
            dy = y_all[i+1] - y_all[i]
            dz = z_abs[i+1] - z_abs[i]
            J1 += np.sqrt(dx**2 + dy**2 + dz**2)

        # 2. Coût des menaces (J2)
        threats = self.model['threats']
        J2 = 0
        
        for threat in threats:
            threat_x, threat_y, _, threat_radius = threat
            threat_pos = np.array([threat_x, threat_y])
            
            for j in range(N-1):
                seg_start = np.array([x_all[j], y_all[j]])
                seg_end = np.array([x_all[j+1], y_all[j+1]])
                dist = self.dist_p2s(threat_pos, seg_start, seg_end)
                
                if dist < threat_radius:
                    J2 += self.J_inf
                elif dist < threat_radius + self.danger_dist:
                    J2 += (threat_radius + self.danger_dist - dist) / self.danger_dist

        # 3. Coût de l'altitude (J3)
        zmax = self.model['zmax']
        zmin = self.model['zmin']
        z_target = (zmax + zmin) / 2
        J3 = 0
        for i in range(len(z)):
            if z[i] < 0:
                J3 += self.J_inf
            else:
                J3 += abs(z[i] - z_target) / (zmax - zmin)

        # 4. Coût des virages et pentes (J4)
        J4 = 0
        for i in range(N-2):
            # Angle de virage horizontal
            vec1 = np.array([x_all[i+1] - x_all[i], y_all[i+1] - y_all[i]])
            vec2 = np.array([x_all[i+2] - x_all[i+1], y_all[i+2] - y_all[i+1]])
            
            norm1 = np.linalg.norm(vec1)
            norm2 = np.linalg.norm(vec2)
            
            if norm1 > 0 and norm2 > 0:
                dot_product = np.dot(vec1, vec2) / (norm1 * norm2)
                # Clamp to avoid numerical errors in arccos
                dot_product = np.clip(dot_product, -1.0, 1.0)
                angle = np.degrees(np.arccos(dot_product))
                if angle > self.turning_max:
                    J4 += (angle - self.turning_max) / 90

            # Angle de montée/descente
            dz = z_abs[i+1] - z_abs[i]
            dy = np.linalg.norm([x_all[i+1] - x_all[i], y_all[i+1] - y_all[i]])
            if dy > 0:
                climb_angle = np.degrees(np.arctan2(dz, dy))
                if abs(climb_angle) > self.climb_max:
                    J4 += (abs(climb_angle) - self.climb_max) / 90

        # Coût total pondéré
        b1, b2, b3, b4 = 5, 1, 10, 1

        cost = b1 * J1 + b2 * J2 + b3 * J3 + b4 * J4
                
        return cost