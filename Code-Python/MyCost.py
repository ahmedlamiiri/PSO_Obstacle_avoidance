import numpy as np

class PathCostCalculator:
    def __init__(self, model):
        self.model = model
        self.J_inf = float('inf')
        self.n = model['n']
        self.H = model['H']

    def dist_p2s(self, x, a, b):
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

    def calculate_cost(self, sol):
        x, y, z = sol['x'], sol['y'], sol['z']
        xs, ys, zs = self.model['start']
        xf, yf, zf = self.model['end']

        x_all = np.concatenate(([xs], x, [xf]))
        y_all = np.concatenate(([ys], y, [yf]))
        z_all = np.concatenate(([zs], z, [zf]))
        N = len(x_all)

        z_abs = np.array([z_all[i] + self.H[int(round(y_all[i])), int(round(x_all[i]))] for i in range(N)])

        J1 = sum(np.linalg.norm([x_all[i + 1] - x_all[i], y_all[i + 1] - y_all[i], z_abs[i + 1] - z_abs[i]]) for i in range(N - 1))

        threats = self.model['threats']
        drone_size = 1
        danger_dist = 10 * drone_size
        J2 = 0

        for threat in threats:
            threat_x, threat_y, _, threat_radius = threat
            for j in range(N - 1):
                dist = self.dist_p2s(np.array([threat_x, threat_y]), np.array([x_all[j], y_all[j]]), np.array([x_all[j + 1], y_all[j + 1]]))
                if dist > (threat_radius + drone_size + danger_dist):
                    threat_cost = 0
                elif dist < (threat_radius + drone_size):
                    threat_cost = self.J_inf
                else:
                    threat_cost =  ((threat_radius + drone_size + danger_dist) - dist)
                J2 += threat_cost

        zmax = self.model['zmax']
        zmin = self.model['zmin']

        J3 = sum(self.J_inf if z[i] < 0 else abs(z[i] - (zmax + zmin) / 2) for i in range(self.n))

        J4 = 0
        turning_max = 45  # Even smoother turns
        climb_max = 45   # More stable altitude

        for i in range(N - 2):
            segment1_proj = np.array([x_all[i + 1] - x_all[i], y_all[i + 1] - y_all[i], 0])
            segment2_proj = np.array([x_all[i + 2] - x_all[i + 1], y_all[i + 2] - y_all[i + 1], 0])
            climb_angle1 = np.degrees(np.arctan2(z_abs[i + 1] - z_abs[i], np.linalg.norm(segment1_proj)))
            climb_angle2 = np.degrees(np.arctan2(z_abs[i + 2] - z_abs[i + 1], np.linalg.norm(segment2_proj)))
            cross_prod = np.cross(segment1_proj, segment2_proj)
            turning_angle = np.degrees(np.arctan2(np.linalg.norm(cross_prod), np.dot(segment1_proj, segment2_proj)))

            if abs(turning_angle) > turning_max:
                J4 += abs(turning_angle)
            if abs(climb_angle2 - climb_angle1) > climb_max:
                J4 += abs(climb_angle2 - climb_angle1)

        b1, b2, b3, b4 = 5, 1, 10, 1

        cost = b1 * J1 + b2 * J2 + b3 * J3 + b4 * J4
        return cost