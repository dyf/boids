import numpy as np

class Rule:
    def compute_force(self, boids):
        raise NotImplementedError
    
    def __call__(self, boids, dt):
        self.compute_force(boids, dt)
        
class Separation(Rule):
    def __init__(self, dist, k, min_dist=0.001):
        self.dist = dist
        self.min_dist = min_dist
        self.k = k

    def compute_force(self, boids, dt):
        new_forces = np.zeros_like(boids['force'])

        for pair in boids.near_pairs(self.dist):
            i,j = pair
            dp = boids['position'][j,:] - boids['position'][i,:]
            d = np.linalg.norm(dp)
            d = sel.min_dist if d < self.min_dist else d
            scale = (self.dist - d) / d
            df = self.k * dp / d * scale
            new_forces[i] -= df
            new_forces[j] += df

        return new_forces

class Alignment(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        new_forces = np.zeros_like(boids['force'])

        cts = np.zeros(len(boids['force']))
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            cts[i] += 1
            cts[j] += 1

            new_forces[i] += boids['velocity'][j]
            new_forces[j] += boids['velocity'][i]

        nonzero = np.where(cts > 0)
        new_forces[nonzero,:] /= cts[nonzero, np.newaxis]

        return new_forces

class Cohesion(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        centroids = np.zeros_like(boids['position'])

        cts = np.zeros(len(boids['position']))
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            cts[i] += 1
            cts[j] += 1

            centroids[i] += boids['position'][j]
            centroids[j] += boids['position'][i]

        nonzero = np.where(cts > 0)
        centroids[nonzero,:] /= cts[nonzero, np.newaxis]

        new_forces = np.zeros_like(boids['force'])

        for i in nonzero[0]:
            dp = centroids[i] - boids['position'][i]
            mag = np.linalg.norm(dp)
            if mag > 0:
                new_forces[i] = dp / mag * self.k

        return new_forces

        
        
class Drag(Rule):
    def __init__(self, k):
        self.k = k

    def compute_force(self, boids, dt):
        return -self.k * boids['velocity'] / dt
