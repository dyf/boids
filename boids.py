import numpy as np 
import xarray as xr
import scipy.spatial
import functools

class Boids:
    def __init__(self, N, rules=None, dims=2, extent=None):
        self.N = N
        self.dims = dims

        self.props = xr.Dataset({
            'mass': (('boid',), np.ones(N, dtype=float)),
            'position': (('boid','location'), np.zeros((N,dims), dtype=float)),
            'velocity': (('boid','location'), np.zeros((N,dims), dtype=float)),
            'force': (('boid','location'), np.zeros((N,dims), dtype=float)),
        }, {
            'boid': np.arange(N)
        })
        
        self._ssub = None
        self._near_pairs = {}
        
        self.rules = rules if rules is not None else []

    def __getitem__(self, name):
        return self.props[name].values

    def __setitem__(self, name, value):
        np.copyto(self[name], value)

    @staticmethod
    def to_param_v(v, p):
        return np.concat(v.ravel(), p.ravel())

    @staticmethod
    def from_param_v(p):
        return

    @staticmethod
    def callback(f, *args, **kwargs):
        return functools.partial(f, *args, **kwargs)

    def simulate(self, ts, callbacks=None):
        callbacks = callbacks if callbacks else []

        for callback in callbacks:
            callback(boids=self, i=0, total=len(ts), t=ts[0], t_end=ts[-1])
        
        dts = np.diff(ts)
        for i,dt in enumerate(dts):
            self.update(dt)

            for callback in callbacks:
                callback(boids=self, i=i+1, total=len(ts), t=ts[i+1], t_end=ts[-1])
        
    def compute_derivative(self):
        dvdt = self['force'] / self['mass']
        dpdt = self['velocity']

        return self.to_param_v(dvdt, dpdt)
        
    def update(self, dt):
        self._near_pairs = {}
        self._ssub = None
        
        self.compute_force(dt)
        self.update_velocity(dt)
        self.update_position(dt)

    def near_pairs(self, max_dist):
        res = self._near_pairs.get(max_dist, None)

        if self._ssub is None:
            self._ssub = scipy.spatial.KDTree(self['position'])
            
        if res is None:
            res = self._ssub.query_pairs(max_dist)
            self._near_pairs[max_dist] = res
            
        return res

    def compute_force(self, dt):
        self['force'].fill(0)

        for rule in self.rules:
            self['force'] += rule.compute_force(self, dt)

    def update_velocity(self, dt):
        # f = m * a
        # f = m * dv / dt
        # dv = f * dt / m

        dv = self['force'] * dt / self['mass'][:,np.newaxis]
        self['velocity'] += dv
        
    def update_position(self, dt):
        # v = dp / dt
        # dp = v * dt
        dp = self['velocity'] * dt
        self['position'] += dp
    

