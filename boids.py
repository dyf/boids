import numpy as np 
import pandas as pd
import scipy.spatial
from collections import defaultdict

class Rule:
    def compute_force(self, boids):
        raise NotImplementedError
    
    def __call__(self, boids, dt):
        self.compute_force(boids, dt)
        
class AvoidNeighbors(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        new_forces = np.zeros_like(boids.force)
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            dp = boids.position[j,:] - boids.position[i,:]
            df = self.k * dp / np.linalg.norm(dp)
            new_forces[i] -= df
            new_forces[j] += df

        return new_forces

class Align(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        new_forces = np.zeros_like(boids.force)

        cts = np.zeros(len(boids.force))
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            cts[i] += 1
            cts[j] += 1

            new_forces[i] += boids.velocity[j]
            new_forces[j] += boids.velocity[i]

        return new_forces / cts / dt

class Cohesion(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        centroids = np.zeros_like(boids.position)

        cts = np.zeros(len(boids.position))
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            cts[i] += 1
            cts[j] += 1

            centroids[i] += boids.position[j]
            centroids[j] += boids.position[i]

        centroids /= cts

        new_forces = np.zeros_like(boids.force)

        for i, centroid in enumerate(centroids):
            dp = centroid - boids.position[i]
            new_forces[i] = dp / np.linalg.norm(dp) * self.k

        return new_forces

        
        
class Drag(Rule):
    def __init__(self, k):
        self.k = k

    def compute_force(self, boids, dt):
        return -self.k * boids.velocity / dt

    
class Boids:
    def __init__(self, N, rules=None, dims=2, extent=None):
        self.mass = np.ones((N,dims), dtype=float)
        self.position = np.zeros((N,dims), dtype=float)
        self.velocity = np.zeros((N,dims), dtype=float)
        self.force = np.zeros((N,dims), dtype=float)

        
        self._ssub = scipy.spatial.KDTree(self.position)
        self._near_pairs = {}
        
        self.rules = rules if rules is not None else []
        self.extent = extent if extent is not None else np.array([[0,1]] * dims, dtype=float)

    def update(self, dt):
        self._near_pairs = {}
        
        self.compute_force(dt)
        self.update_velocity(dt)
        self.update_position(dt)

    def near_pairs(self, max_dist):
        res = self._near_pairs.get(max_dist, None)

        if res is None:
            res = self._ssub.query_pairs(max_dist)
            self._near_pairs[max_dist] = res
            
        return res

    def compute_force(self, dt):
        self.force.fill(0)

        for rule in self.rules:
            self.force += rule.compute_force(self, dt)

    def update_velocity(self, dt):
        # f = m * a
        # f = m * dv / dt
        # dv = f * dt / m
        dv = self.force * dt / self.mass
        self.velocity += dv
        
    def update_position(self, dt):
        # v = dp / dt
        # dp = v * dt
        dp = self.velocity * dt
        self.position += dp
    
    

if __name__ == "__main__":
    boids = Boids(2, dims = 2, rules = [
        AvoidNeighbors(1,1),
        #Drag(.5),
        #Align(1, 1),
        Cohesion(1, 1),
    ])
    boids.position = np.array([[0,0],[0,1]], dtype=float)

    print(boids.position)
    for i in range(20):
        boids.update(.1)
        print(i,boids.position)
