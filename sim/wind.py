"""Wind fields.

Every wind in this project is a callable `(t, position) -> velocity vector`.
Position matters because the surface boundary layer makes wind a function of
height, and because the kite, the payload and the tether midpoint all sit at
different heights -- a difference the digital twin has to carry even though the
indoor tests will not see it.

Fields compose: `add(shear(...), gust(...))` is a gust riding on a profile.
"""

import numpy as np

REF_HEIGHT = 10.0     # m, standard meteorological reference
# Not open flat ground: that is about 1/7 = 0.143 in the standards. 0.20 is
# terrain with scattered obstacles -- a school field with buildings and trees
# around it, which is where this will actually fly. It is the conservative
# choice for low-altitude work, because a larger exponent means less wind down
# low. See extremes of this in lowaltitude.py.
ALPHA = 0.20          # power-law exponent, terrain with scattered obstacles
Z_MIN = 0.5           # m, below this the profile stops meaning anything


def uniform(speed):
    """Height-independent wind. What a fan produces, and what every result
    before the boundary-layer work assumed."""
    vector = np.array([speed, 0.0])
    return lambda t, r: vector


def shear(reference_speed, alpha=ALPHA, reference_height=REF_HEIGHT):
    """Power-law surface layer: v(z) = v_ref * (z / z_ref) ** alpha."""
    def field(t, r):
        height = max(float(r[1]), Z_MIN)
        return np.array([reference_speed * (height / reference_height) ** alpha,
                         0.0])
    return field


def gust(base_field, peak_ratio=1.67, start=5.0, duration=1.5):
    """One raised-cosine gust scaling whatever field it wraps."""
    def field(t, r):
        v = base_field(t, r)
        if start <= t < start + duration:
            s = 0.5 * (1 - np.cos(2 * np.pi * (t - start) / duration))
            return v * (1 + (peak_ratio - 1) * s)
        return v
    return field


def ramp(base_field, final_ratio, start=10.0, duration=120.0):
    """Mean wind trending from the base field to final_ratio times it."""
    def field(t, r):
        v = base_field(t, r)
        if t < start:
            return v
        frac = min((t - start) / duration, 1.0)
        return v * (1 + (final_ratio - 1) * frac)
    return field


def lull(base_field, final_ratio, start=2.0, duration=0.5):
    return ramp(base_field, final_ratio, start, duration)


def turbulence(base_field, intensity=0.12, length_scale=60.0, speed=6.0,
               duration=600.0, step=0.05, seed=0):
    """Ornstein-Uhlenbeck gust process: the right variance and the right
    timescale, without pretending to a full von Karman spectrum.

    `intensity` is sigma_u / U, which over open ground runs 0.10-0.20 near the
    surface. `length_scale` in metres becomes a correlation time through
    Taylor's frozen-turbulence hypothesis, tau = L / U.

    The series is generated once on a fixed grid and interpolated, so the field
    is a deterministic function of t. Sampling it lazily instead would hand an
    adaptive solver a different value each time it re-evaluated the same
    instant, which quietly destroys its error estimate.
    """
    rng = np.random.default_rng(seed)
    tau = length_scale / max(speed, 1e-3)
    decay = np.exp(-step / tau)
    sigma = intensity * np.sqrt(1 - decay**2)

    n = int(duration / step) + 2
    u = np.empty(n)
    u[0] = rng.normal(0.0, intensity)   # start already stationary, no burn-in
    for i in range(1, n):
        u[i] = u[i - 1] * decay + rng.normal(0.0, sigma)
    grid = np.arange(n) * step

    def field(t, r):
        return base_field(t, r) * (1 + np.interp(t, grid, u))

    field.series = (grid, u)
    field.tau = tau
    return field


def add(*fields):
    def field(t, r):
        return sum(f(t, r) for f in fields)
    return field
