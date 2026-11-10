"""Verification for the self-updating calibration loop.

The detector's statistics are validated by Monte Carlo inside auto.py (false
alarm rate and detection delay). These are the fast structural checks that
guard the two bugs that validation exposed: a baseline that moved when
convergence was declared, and a detector that only armed after convergence --
which let a large change suppress its own detection. Run with: pytest -q
"""

import numpy as np
import pytest

from auto import (BASELINE_FLIGHTS, CHANGE_PERSISTENCE, CHANGE_WINDOW,
                  Calibration)


def feed(cal, values):
    return [cal.add(v) for v in values]


def test_baseline_is_taken_at_a_fixed_flight_count():
    cal = Calibration()
    feed(cal, [0.75 + 0.01 * i for i in range(BASELINE_FLIGHTS)])
    assert np.isfinite(cal.baseline)
    assert cal.baseline_scatter > 0


def test_baseline_does_not_move_once_set():
    """Convergence used to overwrite it, which silently re-centred the
    detector on whatever the data had already drifted to."""
    cal = Calibration()
    feed(cal, [0.75] * BASELINE_FLIGHTS)
    first = cal.baseline
    feed(cal, [0.75] * 6)
    assert cal.baseline == first


def test_detector_arms_without_waiting_for_convergence():
    """A large change prevents convergence; the detector must still fire."""
    rng = np.random.default_rng(0)
    cal = Calibration()
    feed(cal, list(0.75 + rng.normal(0, 0.02, BASELINE_FLIGHTS)))
    notes = feed(cal, [0.40] * (CHANGE_WINDOW + CHANGE_PERSISTENCE))
    assert any("CHANGE" in n for n in notes)


def test_a_single_outlier_does_not_fire():
    cal = Calibration()
    feed(cal, [0.75] * BASELINE_FLIGHTS)
    assert "CHANGE" not in cal.add(0.20)


def test_change_fires_only_once():
    cal = Calibration()
    feed(cal, [0.75] * BASELINE_FLIGHTS)
    notes = feed(cal, [0.30] * 10)
    assert sum("CHANGE" in n for n in notes) == 1


def test_steady_input_converges_and_says_so():
    cal = Calibration()
    notes = feed(cal, [0.75] * 12)
    assert cal.converged
    assert sum("CONVERGED" in n for n in notes) == 1


def test_random_error_shrinks_with_more_flights():
    rng = np.random.default_rng(1)
    cal = Calibration()
    feed(cal, list(0.75 + rng.normal(0, 0.05, 4)))
    early = cal.random_sigma
    feed(cal, list(0.75 + rng.normal(0, 0.05, 20)))
    assert cal.random_sigma < early


def test_pooling_cannot_remove_a_shared_bias():
    """The whole reason the loop stops: every flight carries the same offset."""
    rng = np.random.default_rng(2)
    bias = 0.08
    cal = Calibration()
    feed(cal, list(0.75 * (1 + bias) + rng.normal(0, 0.03, 40)))
    assert cal.estimate == pytest.approx(0.75 * (1 + bias), rel=0.02)
