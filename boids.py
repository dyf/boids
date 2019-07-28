import numpy as np 
import pandas as pd

class Boids:
    def __init__(self, N, dims=2):
        self.position = np.array((n,dims), dtype=float)
        self.velocity = np.array((n,dims), dtype=float)        
        self.rules = []

    def update(self, dt):        
        f = self.compute_force(dt)
        self.move(f, dt)

    def compute_forces(self, dt):
        pass
        
    def move(self, forces, dt):
        pass
    
    def add_rule(self, fn):
        pass

