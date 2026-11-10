"""2D point-mass dynamics of a bridled kite on an extensible tether.

Plane is (x = downwind, z = up), anchor at the origin. The tether is a
spring-damper that carries tension only -- it goes slack rather than pushing,
which is the nonlinearity that matters for line-length control.

Tether mass is neglected: ~50 g of silk over 50 m against a ~450 g kite. That
is defensible for the stretch/oscillation question studied here, but it drops
line sag, so absolute elevation angles run slightly optimistic.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq, fsolve

import wind as wind_module
from params import CD_CYLINDER, G, RHO_AIR


def line_force(v, wind, r, tether, length):
    """Tether weight and crossflow drag, reduced to one force at the kite.

    Half the distributed load is applied at the kite, which matches the moment
    about the anchor for a straight line. Drag is evaluated with the apparent
    wind at the line's midpoint, and by the independence principle only the
    component normal to the line produces drag.

    Line inertia is still ignored (one third of 50 g against a 450 g kite
    shifts the bounce frequency under 2%).
    """
    rhat = r / np.linalg.norm(r)
    w_mid = wind - 0.5 * v
    w_normal = w_mid - (w_mid @ rhat) * rhat
    speed = np.linalg.norm(w_normal)

    drag = np.zeros(2)
    if speed > 1e-9:
        area = tether.diameter * length
        drag = (0.5 * 0.5 * RHO_AIR * speed**2 * area * CD_CYLINDER
                * (w_normal / speed))

    weight = np.array([0.0, -0.5 * tether.mass_per_m * length * G])
    return drag + weight


def aero_force(v, wind, kite, r):
    """Quasi-steady lift + drag on the canopy.

    The bridle fixes the kite's attitude relative to the *line*, not the
    ground, so lift is taken perpendicular to apparent wind and oriented
    outward along the tether. If that orientation cannot hold the kite up,
    the equilibrium solve reports a negative elevation instead of a fake one.
    """
    w = wind - v
    speed = np.linalg.norm(w)
    if speed < 1e-9:
        return np.zeros(2)

    what = w / speed
    q = 0.5 * RHO_AIR * speed**2 * kite.area

    perp = np.array([-what[1], what[0]])
    rhat = r / np.linalg.norm(r)
    if perp @ rhat < 0:
        perp = -perp

    return q * kite.cd * what + q * kite.cl * perp


def sag_angle(tension_n, length, elevation, tether):
    """Angle between the straight chord and the cable's tangent at either end.

    Small-sag parabolic cable: the vertical reactions are the chord-direction
    component plus and minus half the cable weight, so
    tan(theta_end) = tan(theta_chord) +/- W/(2H), which for small sag gives
    delta = W*cos(theta) / (2*T). The cable bows below the chord, so the tangent
    at the kite is this much steeper and the tangent at the anchor this much
    shallower.

    Note what this does *not* change: the force the cable exerts on the kite is
    already correct, because lumping half the cable weight at the kite is the
    same parabolic result. Sag enters as a sensing error -- a ground-mounted
    angle sensor reads the tangent, while altitude needs the chord.

    The small-sag form degrades once delta grows; callers should treat a result
    beyond ~20 deg as out of range rather than trusting it.
    """
    if tension_n <= 1e-9:
        return 0.0
    weight = tether.mass_per_m * length * G
    return 0.5 * weight * np.cos(elevation) / tension_n


def tension(radius, radius_rate, L0, tether, mass):
    """Axial tension [N]; zero when the line is slack."""
    strain = (radius - L0) / L0
    if strain <= 0:
        return 0.0
    k = tether.stiffness(L0)
    c = 2 * tether.damping_ratio * np.sqrt(k * mass)
    return max(0.0, tether.EA * strain + c * radius_rate)


def added_mass(kite, factor=1.0):
    """Air the canopy drags along when it accelerates broadside, in kg.

    Potential flow for a circular disc of radius a gives 8*rho*a^3/3, taken
    here with a from the sail area. For this kite that is 207 g against a
    450 g airframe -- 46% of the inertia, far above the parameter uncertainty
    the rest of the model lives with, so leaving it at zero is not a
    simplification but an error in a known direction (frequencies too high).

    The coefficient is the uncertain part, not the existence of the term:
      * the disc result is for a rigid solid disc, and could not be confirmed
        against a primary source in the literature check -- Lamb's
        Hydrodynamics is the reference to verify it against;
      * the vent hole removes canopy area and should reduce it;
      * membrane canopies go the other way. Parachute work reports apparent
        mass of 1.2 to 4 times the enclosed air mass, so for fabric the disc
        value is more likely a floor than a ceiling.

    `factor` exists so that band can be swept rather than asserted. What the
    model must not do is assume the term is zero.

    Potential flow gives zero added mass for motion in the plane of the plate,
    so this is applied only along the canopy normal. The resultant of lift and
    drag is normal to a flat plate, which is the direction used below.
    """
    a = np.sqrt(kite.area / np.pi)
    return factor * 8.0 / 3.0 * RHO_AIR * a**3


def effective_mass(kite, am_factor=1.0):
    """Inertia that resists broadside acceleration: airframe plus added mass.

    Anything that reports a bounce frequency has to use this, not kite.mass.
    Mixing the two is not a small error -- compare_tether drives the chatter
    scenario *at* the frequency it computes, so an airframe-mass figure would
    excite 17% off resonance while still being labelled a resonance test.
    """
    return kite.mass + added_mass(kite, am_factor)


def rhs(t, y, kite, tether, wind_fn, length_fn, am_factor=1.0):
    r, v = y[:2], y[2:]
    radius = np.linalg.norm(r)
    rhat = r / radius
    T = tension(radius, v @ rhat, length_fn(t), tether, kite.mass)

    aero = aero_force(v, wind_fn(t, r), kite, r)

    # The tether spans a range of heights, so its drag uses the wind at the
    # line's midpoint rather than the kite's.
    force = (
        aero
        + line_force(v, wind_fn(t, 0.5 * r), r, tether, length_fn(t))
        + np.array([0.0, -kite.mass * G])
        - T * rhat
    )

    # Inertia is directional: the canopy carries air with it broadside and
    # none edgewise. Split the acceleration along the canopy normal (taken as
    # the aerodynamic resultant) and in the plane, and give each its own mass.
    ma = added_mass(kite, am_factor)
    mag = np.linalg.norm(aero)
    if ma <= 0.0 or mag < 1e-9:
        return np.concatenate([v, force / kite.mass])

    nhat = aero / mag
    f_n = force @ nhat
    a_vec = (force - f_n * nhat) / kite.mass + (f_n / (kite.mass + ma)) * nhat
    return np.concatenate([v, a_vec])


def equilibrium(kite, tether, wind, L0):
    """Steady flying state: returns (state, tension, elevation_deg).

    `wind` is either a speed in m/s or a wind field from wind.py. A field is
    evaluated at the kite and at the tether midpoint, so a sheared profile
    gives a different answer from the uniform wind of the same reference speed.

    Returns NaN for all three when no flying equilibrium exists. Callers must
    still reject roots below the horizon, where the outward-lift bridle
    assumption stops describing a real kite.

    Solved as a one-dimensional bracketed root find rather than a
    two-dimensional Newton solve. The reason is measured, not stylistic: over
    the 2000-sample parameter study the 2-D solve failed to converge on 5.5%
    of draws, and every one of those sat within a few percent of lift equal to
    weight -- 148 failures all between 0.67 and 1.05 on that ratio, median
    0.98. That is the bifurcation where the flying root and the below-horizon
    root approach each other, and a Newton step from a crude guess slides off.
    Worse, fsolve returned its last iterate regardless: elevations near
    -550 deg carrying a 1.6 N residual on a balance that closes to 1e-10 when
    it works.

    The structure removes the problem. For a given elevation the tension is
    whatever the balance needs along the tether, so only the perpendicular
    component has to vanish -- one equation, one unknown, on a bounded
    interval. A sign change is then a proof that a root exists and brackets
    it; no sign change is a proof that it does not. Neither outcome can be a
    silent failure.
    """
    m = kite.mass
    field = wind if callable(wind) else wind_module.uniform(wind)

    def state_at(theta):
        """Position and tension implied by an elevation angle.

        Radius depends on tension and tension on radius. In uniform wind two
        passes are plenty, which is what this used to do. In a sheared field
        they are not: the radius sets the altitude, the altitude sets the
        wind, and the loop has to close. Two passes left a 1.5e-3 N residual
        at 10 m/s -- negligible against everything else here, but it made the
        "equilibrium" not quite one, so it iterates to tolerance instead. The
        cost falls on the final call, not on the bracketing scan.
        """
        rhat = np.array([np.cos(theta), np.sin(theta)])
        radius = L0
        for _ in range(12):
            r = radius * rhat
            total = (
                aero_force(np.zeros(2), field(0.0, r), kite, r)
                + line_force(np.zeros(2), field(0.0, 0.5 * r), r, tether, L0)
                + np.array([0.0, -m * G])
            )
            T = float(total @ rhat)
            nxt = L0 * (1.0 + max(T, 0.0) / tether.EA)
            if abs(nxt - radius) < 1e-12 * L0:
                radius = nxt
                break
            radius = nxt
        return radius * rhat, T, total

    def perpendicular(theta):
        """What has to vanish: force across the tether cannot be balanced."""
        rhat = np.array([np.cos(theta), np.sin(theta)])
        that = np.array([-rhat[1], rhat[0]])
        _, _, total = state_at(theta)
        return float(total @ that)

    # Fast path. The bracketed scan below is 18x the work of a Newton solve,
    # and on 94.5% of calls the Newton solve is right -- measured on the
    # parameter study, and measured again as an 8 s study turning into an
    # 81 min one when the scan ran unconditionally. So try Newton first and
    # *check* it: converged, residual closed, and above the horizon. Anything
    # short of that falls through to the scan, which cannot fail silently.
    # Checking is what makes the fast path safe; using it unchecked is the
    # bug this function was rewritten to remove.
    probe = field(0.0, np.array([L0 * 0.8, L0 * 0.6]))
    q = 0.5 * RHO_AIR * float(np.linalg.norm(probe)) ** 2 * kite.area
    fx, fz = q * kite.cd, q * kite.cl - m * G
    guess = [np.arctan2(fz, fx), L0 * (1 + np.hypot(fx, fz) / tether.EA)]

    def residual2d(u):
        theta_, radius_ = u
        if radius_ <= 0:
            return np.array([1e3, 1e3])
        rhat_ = np.array([np.cos(theta_), np.sin(theta_)])
        r_ = radius_ * rhat_
        T_ = tension(radius_, 0.0, L0, tether, m)
        return (
            aero_force(np.zeros(2), field(0.0, r_), kite, r_)
            + line_force(np.zeros(2), field(0.0, 0.5 * r_), r_, tether, L0)
            + np.array([0.0, -m * G])
            - T_ * rhat_
        )

    sol, _, ier, _ = fsolve(residual2d, guess, full_output=True)
    if ier == 1 and float(np.linalg.norm(residual2d(sol))) < 1e-9:
        theta_fast = float(sol[0])
        if np.radians(0.05) < theta_fast < np.radians(89.5):
            r, T, _ = state_at(theta_fast)
            return (np.concatenate([r, np.zeros(2)]), float(T),
                    float(np.degrees(theta_fast)))

    # Scan the half-plane above the horizon for a sign change. The upper limit
    # is just short of vertical, where the tether direction degenerates.
    lo, hi = np.radians(0.05), np.radians(89.5)
    thetas = np.linspace(lo, hi, 181)
    values = np.array([perpendicular(t) for t in thetas])
    if not np.all(np.isfinite(values)):
        nan2 = np.array([np.nan, np.nan])
        return np.concatenate([nan2, nan2]), float("nan"), float("nan")

    sign_change = np.nonzero(np.sign(values[:-1]) * np.sign(values[1:]) < 0)[0]
    if sign_change.size == 0:
        # No equilibrium above the horizon. This is an answer, not a failure.
        nan2 = np.array([np.nan, np.nan])
        return np.concatenate([nan2, nan2]), float("nan"), float("nan")

    # Highest root: with several crossings the physical one is the steepest
    # the kite can hold, the lower ones being unstable branches.
    i = sign_change[-1]
    theta = brentq(perpendicular, thetas[i], thetas[i + 1], xtol=1e-12)

    r, T, _ = state_at(theta)
    state = np.concatenate([r, np.zeros(2)])
    return state, float(T), float(np.degrees(theta))


def min_area_for_elevation(kite, wind_speed, elevation_deg):
    """Sail area needed to fly at a given elevation angle [m^2].

    Returns inf when the kite's L/D cannot reach that angle at any size.
    """
    denom = kite.cl - kite.cd * np.tan(np.radians(elevation_deg))
    if denom <= 0:
        return np.inf
    return 2 * kite.mass * G / (RHO_AIR * wind_speed**2 * denom)


def simulate(kite, tether, wind_fn, length_fn, state0, t_end, n_out=2000,
             method="Radau", rtol=1e-6, atol=1e-8, max_step=0.1):
    """Integrate the kite.

    Defaults chosen by measurement, not habit -- see convergence.py. rtol=1e-6
    puts the gust peak within 1e-7 of a reference run, three times faster than
    the 1e-8 used before.

    Radau rather than RK45: RK45 is about twice as fast on the silk tether, but
    slower *and* less accurate on the stiffest one, and the whole point of the
    tether comparison is to span that range. LSODA is excluded outright -- at
    rtol=1e-6 it stepped straight over the gust and returned a peak 83% low,
    without any error.

    `max_step` is the guard against that failure, and Radau needs it too. A
    gust is zero outside its window, so between gusts the right-hand side looks
    flat and an adaptive solver will happily take a step longer than the gust
    and never evaluate inside it. Error control cannot catch what it never
    samples. A soft tether makes it worse, because slow dynamics invite long
    steps: at k=19 N/m the solver skipped the whole gust and reported the
    steady tension as the peak. Capping the step below the shortest forcing
    feature costs little and removes the whole class of error.
    """
    t_eval = np.linspace(0.0, t_end, n_out)
    sol = solve_ivp(
        rhs,
        (0.0, t_end),
        state0,
        args=(kite, tether, wind_fn, length_fn),
        method=method,
        t_eval=t_eval,
        rtol=rtol,
        atol=atol,
        max_step=max_step,
    )
    if not sol.success:
        raise RuntimeError(f"integration failed: {sol.message}")

    r = sol.y[:2].T
    v = sol.y[2:].T
    radius = np.linalg.norm(r, axis=1)
    rhat = r / radius[:, None]
    radius_rate = np.einsum("ij,ij->i", v, rhat)
    commanded = np.array([length_fn(t) for t in sol.t])
    T = np.array([
        tension(radius[i], radius_rate[i], commanded[i], tether, kite.mass)
        for i in range(len(sol.t))
    ])

    return {
        "t": sol.t,
        "r": r,
        "v": v,
        "radius": radius,
        "commanded": commanded,
        "tension": T,
        "elevation": np.degrees(np.arctan2(r[:, 1], r[:, 0])),
    }
