import matplotlib
matplotlib.use('agg')

import numpy as np
import rendermpl

from boids import Boids
from rules import Separation, Alignment, Cohesion, Drag

N = 400
dims = 2
    
boids = Boids(N, dims = dims, rules = [
    Separation(.1, 1),
    #Drag(.5),
    Alignment(.1, 4),
    Cohesion(.1, 4),
])

#xx, yy = np.meshgrid(np.linspace(0,1,20), np.linspace(0,1,20))

#boids.velocity[(yy<.5).ravel(),0] = 1
#boids.velocity[(yy>=.5).ravel(),0] = -1
    
#boids.position = np.random.random((N,dims)).astype(float)
boids.set_position(np.random.random((N,2)).astype(float))
boids.velocity[boids.position[:,1] < 0.5,0] = 1
boids.velocity[boids.position[:,1] <= 0.5,0] = -1


rendermpl.render(steps=200, dt=0.02, boids=boids, prefix="/mnt/c/Users/davidf/workspace/boids/test_")
