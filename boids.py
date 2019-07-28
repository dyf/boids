import numpy as np 
import pandas as pd

class Rule: pass

class AvoidNeighbors:
    def __init__(self, dist):
        self.dist = dist

    def __call__(self, boids):
        d = boids.distance_matrix
    
    
class Boids:
    def __init__(self, N, dims=2):
        self.mass = np.ones((n,dims), dtype=float)
        self.position = np.zeros((n,dims), dtype=float)
        self.velocity = np.zeros((n,dims), dtype=float)
        self.force = np.zeros((n,dims), dtype=float)
        
        self.rules = [] 

    def update(self, dt):        
        self.compute_force(dt)
        self.update_velocity(dt)
        self.update_position(dt)

    def compute_force(self, dt):
        self.force.fill(0)

        for rule in self.rules:
            self.force += rule(self)

    def update_velocity(self, dt):
        # f = m * a
        # f = m * dv / dt
        # dv = f * dt / m
        dv = self.force * dt / self.mass
        self.velocity += dv
        
    def move(self, dt):
        # v = dp / dt
        # dp = v * dt
        dp = self.velocity * dt
        self.position += dp
    
    def add_rule(self, fn):
        pass

