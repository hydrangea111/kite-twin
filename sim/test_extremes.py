"""Tests for the extreme-value machinery.

The flights themselves are far too slow to run here; what is testable is the
statistics sitting on top of them, and that is where a quiet error would do
the most damage. A return level that is wrong by a factor does not look
wrong -- it looks like a number.
"""

import numpy as np
import pytest
from scipy import stats

from extremes import bootstrap_return, gumbel_return_level


def test_return_level_recovers_a_known_gumbel():
    """Draw from a Gumbel with known parameters and fit it back."""
    loc, scale = 30.0, 5.0
    sample = stats.gumbel_r.rvs(loc, scale, size=4000, random_state=0)
    for period in (10, 100, 1000):
        truth = stats.gumbel_r.ppf(1 - 1 / period, loc, scale)
        assert gumbel_return_level(sample, period) == pytest.approx(truth, rel=0.03)


def test_return_level_grows_with_the_return_period():
    sample = stats.gumbel_r.rvs(30.0, 5.0, size=500, random_state=1)
    levels = [gumbel_return_level(sample, p) for p in (10, 100, 1000)]
    assert levels[0] < levels[1] < levels[2]


def test_return_level_exceeds_the_bulk_of_the_data():
    """A 100-flight level below the observed median would be nonsense."""
    sample = stats.gumbel_r.rvs(30.0, 5.0, size=300, random_state=2)
    assert gumbel_return_level(sample, 100) > np.median(sample)


def test_bootstrap_interval_brackets_the_estimate():
    sample = stats.gumbel_r.rvs(30.0, 5.0, size=200, random_state=3)
    point = gumbel_return_level(sample, 100)
    lo, hi = bootstrap_return(sample, 100, trials=400, seed=0)
    assert lo < point < hi


def test_bootstrap_interval_narrows_with_more_blocks():
    """More flights must buy a tighter number, or the study has no point."""
    rng = np.random.default_rng(4)
    big = stats.gumbel_r.rvs(30.0, 5.0, size=2000, random_state=5)
    widths = []
    for n in (20, 400):
        sample = rng.choice(big, n, replace=False)
        lo, hi = bootstrap_return(sample, 100, trials=400, seed=1)
        widths.append(hi - lo)
    assert widths[1] < widths[0] / 2


def test_bootstrap_is_deterministic_for_a_given_seed():
    """Design numbers that move between runs cannot be checked by anyone."""
    sample = stats.gumbel_r.rvs(30.0, 5.0, size=200, random_state=6)
    a = bootstrap_return(sample, 100, trials=300, seed=11)
    b = bootstrap_return(sample, 100, trials=300, seed=11)
    assert np.allclose(a, b)


def test_a_heavier_tail_is_not_silently_reported_as_gumbel():
    """Fitting Gumbel to a Frechet tail under-reports the far return levels.

    This is the failure the study's shape test exists to catch, so it has to
    be a failure the data can actually show.
    """
    heavy = stats.genextreme.rvs(-0.3, loc=30.0, scale=5.0, size=2000,
                                 random_state=7)
    shape, loc, scale = stats.genextreme.fit(heavy)
    xi = -shape
    assert xi > 0.1, "무거운 꼬리인데 형상이 0 근처로 나옵니다"

    gumbel_1000 = gumbel_return_level(heavy, 1000)
    gev_1000 = stats.genextreme.ppf(1 - 1 / 1000, shape, loc, scale)
    assert gumbel_1000 < gev_1000, "Gumbel 이 무거운 꼬리를 과소평가해야 정상"
