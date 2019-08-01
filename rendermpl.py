import matplotlib.pyplot as plt

def render(steps, dt, boids, prefix):

    for i in range(steps):
        print("%d/%d"%(i+1,steps))
        fig, ax = plt.subplots()
        ax.scatter(boids.position[:,0], boids.position[:,1])
        plt.savefig(prefix+"%03d.png" % i)
        plt.close()

        boids.update(dt)
