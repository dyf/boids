import numpy as np 
import pandas as pd
import scipy.spatial

class Rule:
    def compute_force(self, boids):
        raise NotImplementedError
    
    def __call__(self, boids):
        self.compute_force(boids)
        
class AvoidNeighbors(Rule):
    def __init__(self, dist, k):
        self.dist = dist
        self.k = k

    def compute_force(self, boids):
        new_forces = np.zeros_like(boids.force)
        
        for pair in boids.near_pairs(self.dist):
            i,j = pair
            dp = boids.position[j,:] - boids.position[i,:]
            df = self.k * dp / np.linalg.norm(dp)
            new_forces[i] -= df
            new_forces[j] += df

        return new_forces

    
class Boids:
    def __init__(self, N, rules=None, dims=2):
        self.mass = np.ones((N,dims), dtype=float)
        self.position = np.zeros((N,dims), dtype=float)
        self.velocity = np.zeros((N,dims), dtype=float)
        self.force = np.zeros((N,dims), dtype=float)

        
        self._ssub = scipy.spatial.KDTree(self.position)
        self._near_pairs = {}
        
        self.rules = rules if rules is not None else []

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
            self.force += rule.compute_force(self)

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
    boids = Boids(2, dims = 2, rules = [AvoidNeighbors(1,1)])
    boids.position = np.array([[0,0],[0,1]], dtype=float)

    print(boids.position)
    for i in range(100):
        boids.update(.1)
        print(boids.position)
