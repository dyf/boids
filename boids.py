import numpy as np 
import pandas as pd
import scipy.spatial
from collections import defaultdict

class Rule:
    def compute_force(self, boids):
        raise NotImplementedError
    
    def __call__(self, boids, dt):
        self.compute_force(boids, dt)
        
class Separation(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids, dt):
        new_forces = np.zeros_like(boids.force)

        for pair in boids.near_pairs(self.dist):
            i,j = pair
            dp = boids.position[j,:] - boids.position[i,:]
            d = np.linalg.norm(dp)
            scale = (self.dist - d) / d
            df = self.k * dp / d * scale
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

        nonzero = np.where(cts > 0)
        new_forces[nonzero,:] /= cts[nonzero, np.newaxis]

        return new_forces

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

        nonzero = np.where(cts > 0)
        centroids[nonzero,:] /= cts[nonzero, np.newaxis]

        new_forces = np.zeros_like(boids.force)

        for i in nonzero[0]:
            dp = centroids[i] - boids.position[i]
            mag = np.linalg.norm(dp)
            if mag > 0:
                new_forces[i] = dp / mag * self.k

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

        
        self._ssub = None
        self._near_pairs = {}
        
        self.rules = rules if rules is not None else []
        self.extent = extent if extent is not None else np.array([[0,1]] * dims, dtype=float)

    def set_position(self, pos):
        np.copyto(self.position, pos)

    def update(self, dt):
        self._near_pairs = {}
        self._ssub = None
        
        self.compute_force(dt)
        self.update_velocity(dt)
        self.update_position(dt)

    def near_pairs(self, max_dist):
        res = self._near_pairs.get(max_dist, None)

        if self._ssub is None:
            self._ssub = scipy.spatial.KDTree(self.position)
            
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
    N = 400
    dims = 2
    
    boids = Boids(N, dims = dims, rules = [
        Separation(.1, 1),
        #Drag(.5),
        Align(.1, 1),
        Cohesion(.1, 2),
    ])

    xx, yy = np.meshgrid(np.linspace(0,1,20), np.linspace(0,1,20))
    
    #boids.position = np.random.random((N,dims)).astype(float)
    boids.set_position(np.array([xx.ravel(), yy.ravel()]).T)

    import matplotlib
    matplotlib.use('agg')
    import rendermpl

    rendermpl.render(steps=100, dt=0.02, boids=boids, prefix="/mnt/c/Users/davidf/workspace/boids/test_")
