"""Verification for the winch controller and the claims it rests on.

The first test pins the headline result -- that the winch has no authority over
steady-state tension. It is the reason the controller commands altitude rather
than tension, so if a future model change breaks it, the whole design is wrong
and should fail loudly here. Run with: pytest -q
"""

import numpy as np
import pytest

from control import (MAX_RATE, TENSION_LIMIT, AltitudeLoop, plant, run,
                     steady)
from model import equilibrium, sag_angle
from params import G, LINE_LENGTH, SILK, WIND_BASE, Kite


# ------------------------------------------------------- the headline claim

def test_line_length_barely_moves_tension():
    kite = Kite()
    _, short, _ = equilibrium(kite, SILK, WIND_BASE, 20.0)
    _, long, _ = equilibrium(kite, SILK, WIND_BASE, 80.0)
    assert abs(long / short - 1) < 0.05, "winch would have tension authority"


def test_wind_dominates_tension():
    kite = Kite()
    _, calm, _ = equilibrium(kite, SILK, 6.0, LINE_LENGTH)
    _, blowy, _ = equilibrium(kite, SILK, 9.0, LINE_LENGTH)
    assert blowy / calm > 2.0


# ------------------------------------------------------------- the loop

def controller(**kw):
    return AltitudeLoop(target_alt=30.0, **kw)


def test_rate_command_never_exceeds_the_spool():
    for target in (0.0, 10.0, 30.0, 500.0, 1e6):
        for elev in (np.radians(5.0), np.radians(40.0), np.radians(85.0)):
            cmd = AltitudeLoop(target_alt=target).update(50.0, elev, 6.0)
            assert abs(cmd) <= MAX_RATE + 1e-12


def test_too_high_reels_in_and_too_low_pays_out():
    high = AltitudeLoop(target_alt=10.0)   # kite far above target
    low = AltitudeLoop(target_alt=60.0)    # kite far below target
    assert high.update(50.0, np.radians(40.0), 6.0) < 0
    assert low.update(50.0, np.radians(40.0), 6.0) > 0


def test_tension_limit_overrides_altitude():
    """Protection must dump line even when altitude says reel in."""
    ctrl = AltitudeLoop(target_alt=1.0)    # screaming to reel in
    assert ctrl.update(50.0, np.radians(40.0), 1.0) < 0
    cmd = ctrl.update(50.0, np.radians(40.0), TENSION_LIMIT + 1.0)
    assert ctrl.protecting
    assert cmd == pytest.approx(MAX_RATE)


def test_corrections_shrink_the_altitude_error():
    """Sag and stretch corrections should mostly cancel the sensing bias.

    Residual is second order: the correction evaluates cos() at the sensed
    angle rather than the true chord angle.
    """
    kite = Kite()
    L0 = LINE_LENGTH
    state, T, elev_deg = equilibrium(kite, SILK, WIND_BASE, L0)
    elev = np.radians(elev_deg)
    truth = state[1]
    sensed = elev - sag_angle(T, L0, elev, SILK)

    naive = AltitudeLoop(truth).update(L0, sensed, T)
    fixed = AltitudeLoop(truth, ea=SILK.EA,
                         line_weight_per_m=SILK.mass_per_m).update(L0, sensed, T)

    assert abs(naive) > 1e-3, "test is vacuous if the naive loop is already right"
    assert abs(fixed) < 0.2 * abs(naive)


def test_sag_correction_inverts_the_sag():
    """Applying sag then correcting it returns the chord angle."""
    elev = np.radians(38.0)
    T, L0 = 5.5, 50.0
    sensed = elev - sag_angle(T, L0, elev, SILK)
    weight = SILK.mass_per_m * L0 * G
    recovered = sensed + 0.5 * weight * np.cos(sensed) / T
    assert recovered == pytest.approx(elev, abs=np.radians(0.1))


# ------------------------------------------------------------- the plant

def test_plant_holds_equilibrium_with_no_command():
    kite = Kite()
    base, _, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    y = np.concatenate([base, [LINE_LENGTH, 0.0]])
    d = plant(0.0, y, kite, SILK, steady(WIND_BASE), 0.0)
    assert np.linalg.norm(d[2:4]) < 1e-6   # no acceleration
    assert d[4] == 0.0                     # length static
    assert d[5] == 0.0                     # spool static


def test_spool_rate_follows_the_command():
    kite = Kite()
    base, _, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    y = np.concatenate([base, [LINE_LENGTH, 0.0]])
    d = plant(0.0, y, kite, SILK, steady(WIND_BASE), MAX_RATE)
    assert d[5] > 0


def test_closed_loop_drives_altitude_to_target():
    kite = Kite()
    base, _, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    target = base[1] - 4.0                 # ask for 4 m lower
    res = run(kite, SILK, steady(WIND_BASE),
              AltitudeLoop(target, ea=SILK.EA,
                           line_weight_per_m=SILK.mass_per_m), 300.0)
    assert res["altitude"][-1] == pytest.approx(target, abs=0.15)
