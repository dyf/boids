import matplotlib.pyplot as plt

def render(boids, i, total, t, t_end, prefix):
    print("%d/%d"%(i+1,total))
    fig, ax = plt.subplots()
    ax.scatter(boids['position'][:,0], boids['position'][:,1])
    plt.savefig(prefix+"%03d.png" % i)
    plt.close()
        
