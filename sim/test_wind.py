"""Verification for the wind fields.

The turbulence process gets the most attention here: its whole purpose is to
carry a prescribed variance and correlation time, so those are checked against
the theory rather than against a recorded sample. Run with: pytest -q
"""

import numpy as np
import pytest

import wind


def at(field, t, height=30.0, downwind=40.0):
    return field(t, np.array([downwind, height]))


# ------------------------------------------------------------------ uniform

def test_uniform_ignores_time_and_place():
    field = wind.uniform(6.0)
    assert at(field, 0.0, height=1.0)[0] == 6.0
    assert at(field, 99.0, height=200.0)[0] == 6.0


# -------------------------------------------------------------------- shear

def test_shear_follows_the_power_law():
    field = wind.shear(6.0, alpha=0.2)
    low = at(field, 0.0, height=10.0)[0]
    high = at(field, 0.0, height=20.0)[0]
    assert high / low == pytest.approx(2.0 ** 0.2, rel=1e-12)


def test_shear_matches_reference_at_reference_height():
    field = wind.shear(6.0)
    assert at(field, 0.0, height=wind.REF_HEIGHT)[0] == pytest.approx(6.0)


def test_shear_is_clamped_near_the_ground():
    field = wind.shear(6.0)
    assert at(field, 0.0, height=0.01)[0] == at(field, 0.0, height=wind.Z_MIN)[0]


def test_shear_is_slower_below_the_reference_height():
    field = wind.shear(6.0)
    assert at(field, 0.0, height=3.0)[0] < 6.0


# --------------------------------------------------------------------- gust

def test_gust_leaves_the_base_field_outside_its_window():
    field = wind.gust(wind.uniform(6.0), peak_ratio=2.0, start=5.0, duration=2.0)
    assert at(field, 4.9)[0] == pytest.approx(6.0)
    assert at(field, 7.1)[0] == pytest.approx(6.0)


def test_gust_reaches_its_peak_at_the_middle():
    field = wind.gust(wind.uniform(6.0), peak_ratio=2.0, start=5.0, duration=2.0)
    assert at(field, 6.0)[0] == pytest.approx(12.0, rel=1e-9)


def test_gust_is_continuous_at_its_edges():
    field = wind.gust(wind.uniform(6.0), peak_ratio=2.0, start=5.0, duration=2.0)
    assert at(field, 5.0)[0] == pytest.approx(6.0, abs=1e-9)
    assert at(field, 6.999)[0] == pytest.approx(6.0, abs=1e-3)


def test_gust_rides_on_whatever_it_wraps():
    field = wind.gust(wind.shear(6.0), peak_ratio=2.0, start=5.0, duration=2.0)
    assert at(field, 6.0, height=10.0)[0] == pytest.approx(12.0, rel=1e-9)
    assert at(field, 6.0, height=40.0)[0] > 12.0


# --------------------------------------------------------------------- ramp

def test_ramp_holds_before_and_after():
    field = wind.ramp(wind.uniform(6.0), 1.5, start=10.0, duration=100.0)
    assert at(field, 0.0)[0] == pytest.approx(6.0)
    assert at(field, 60.0)[0] == pytest.approx(7.5)
    assert at(field, 500.0)[0] == pytest.approx(9.0)


# --------------------------------------------------------------- turbulence

def test_turbulence_has_the_requested_intensity():
    field = wind.turbulence(wind.uniform(6.0), intensity=0.15,
                            length_scale=60.0, speed=6.0,
                            duration=20000.0, seed=1)
    _, u = field.series
    assert np.std(u) == pytest.approx(0.15, rel=0.05)
    assert abs(np.mean(u)) < 0.02


def test_turbulence_has_the_requested_correlation_time():
    field = wind.turbulence(wind.uniform(6.0), intensity=0.15,
                            length_scale=60.0, speed=6.0,
                            duration=20000.0, step=0.05, seed=2)
    grid, u = field.series
    lag = int(field.tau / 0.05)          # one correlation time
    rho = np.corrcoef(u[:-lag], u[lag:])[0, 1]
    assert rho == pytest.approx(np.exp(-1.0), abs=0.08)


def test_turbulence_is_a_deterministic_function_of_time():
    """An adaptive solver re-evaluates instants out of order; the field must
    not change underneath it."""
    field = wind.turbulence(wind.uniform(6.0), seed=3)
    forward = [at(field, t)[0] for t in (1.0, 2.0, 3.0)]
    backward = [at(field, t)[0] for t in (3.0, 2.0, 1.0)][::-1]
    assert forward == backward


def test_turbulence_is_reproducible_across_instances():
    a = wind.turbulence(wind.uniform(6.0), seed=4)
    b = wind.turbulence(wind.uniform(6.0), seed=4)
    assert at(a, 7.3)[0] == at(b, 7.3)[0]


def test_turbulence_keeps_the_mean_wind():
    field = wind.turbulence(wind.uniform(6.0), intensity=0.15,
                            duration=20000.0, seed=5)
    samples = [at(field, t)[0] for t in np.linspace(0, 19000, 4000)]
    assert np.mean(samples) == pytest.approx(6.0, rel=0.02)


# ----------------------------------------------------------------- compose

def test_add_sums_the_fields():
    field = wind.add(wind.uniform(4.0), wind.uniform(2.0))
    assert at(field, 0.0)[0] == pytest.approx(6.0)
