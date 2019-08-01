import numpy as np 
import pandas as pd
import scipy.spatial


    
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
    

