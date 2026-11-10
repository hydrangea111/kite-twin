"""Verification for the identification pipeline.

The load-bearing claim here is that an anemometer gain error cancels when the
fitted CL is used for prediction with that same anemometer. The whole argument
for not chasing the wind calibration rests on it, so it is tested rather than
asserted. Run with: pytest -q
"""

import numpy as np
import pytest

from model import equilibrium
from params import LINE_LENGTH, RHO_AIR, SILK, Kite
from twin import identify, invert_row, invert_rows, quasi_static, synthesize


def test_vectorised_inversion_matches_the_scalar_reference():
    """The fast path is only safe while it agrees with the readable one."""
    rows = clean_log(seed=21)[quasi_static(clean_log(seed=21))][:200]
    kite = Kite()
    fast_cl, fast_cd = invert_rows(rows[:, 1], rows[:, 2], rows[:, 3],
                                   rows[:, 4], kite, SILK)
    for i in range(0, 200, 17):
        cl, cd = invert_row(rows[i, 1], rows[i, 2], rows[i, 3], rows[i, 4],
                            kite, SILK)
        assert fast_cl[i] == pytest.approx(cl, rel=1e-9)
        assert fast_cd[i] == pytest.approx(cd, rel=1e-9)


def clean_log(seed=3, **kw):
    return synthesize(Kite(), SILK, [5.0, 6.0, 7.0, 8.0], seconds_each=30.0,
                      seed=seed, **kw)


def test_quasi_static_keeps_most_of_a_steady_log():
    """Guards the bug that started here: differentiating the raw log measures
    sensor noise, and rejected every single sample."""
    assert quasi_static(clean_log()).mean() > 0.5


def test_quasi_static_rejects_a_log_that_is_all_transient():
    log = clean_log()
    # 400 N over the two-minute log is 3.3 N/s, well past the 0.8 N/s the
    # filter calls steady. A gentler ramp is genuinely quasi-static and the
    # filter is right to accept it.
    log[:, 2] += np.linspace(0, 400, len(log))
    assert quasi_static(log).mean() < 0.2


def test_identify_recovers_known_coefficients():
    truth = Kite()
    fit = identify(clean_log(), truth, SILK)
    assert fit["cl"] == pytest.approx(truth.cl, rel=0.03)
    assert fit["cd"] == pytest.approx(truth.cd, rel=0.05)


def test_sag_and_line_load_corrections_are_worth_making():
    """Without them the fit is biased by several percent, which is the whole
    reason twin.invert_row is more than two lines."""
    truth = Kite()
    log = clean_log()
    corrected = identify(log, truth, SILK)["cl"]

    rows = log[quasi_static(log)]
    naive = np.median([
        (t * np.sin(np.radians(e)) + truth.mass * 9.81)
        / (0.5 * RHO_AIR * v**2 * truth.area)
        for _, _, t, e, v, _, _ in rows
    ])
    assert abs(naive / truth.cl - 1) > 3 * abs(corrected / truth.cl - 1)


def test_anemometer_gain_cancels_in_prediction():
    """A biased anemometer gives a biased CL, and the two errors undo each
    other as long as the same instrument feeds both."""
    truth = Kite()
    gain = 1.10                     # anemometer reads 10% high

    log = clean_log(seed=11)
    log[:, 4] *= gain               # every wind reading inflated
    fit = identify(log, truth, SILK)

    # CL should come back low by roughly the square of the gain.
    assert fit["cl"] == pytest.approx(truth.cl / gain**2, rel=0.05)

    # Predicting tension from a reading of that same anemometer is unbiased.
    for true_speed in (5.5, 6.5, 7.5):
        _, true_tension, elev = equilibrium(truth, SILK, true_speed,
                                            LINE_LENGTH)
        reading = true_speed * gain
        q = 0.5 * RHO_AIR * reading**2 * truth.area
        lift = q * fit["cl"]
        predicted = np.hypot(q * fit["cd"],
                             lift - truth.mass * 9.81)
        assert predicted == pytest.approx(true_tension, rel=0.08)


def test_a_wrong_area_biases_the_fit_inversely():
    truth = Kite()
    from dataclasses import replace

    assumed = replace(truth, area=truth.area * 1.10)
    fit = identify(clean_log(seed=13), assumed, SILK)
    assert fit["cl"] == pytest.approx(truth.cl / 1.10, rel=0.05)


def test_identify_refuses_a_log_with_nothing_steady():
    log = clean_log()
    log[:, 2] += np.linspace(0, 200, len(log))
    with pytest.raises(ValueError):
        identify(log, Kite(), SILK)

def test_quasi_static_rejects_a_slack_line():
    """Steady is not the same as valid.

    A slack line is perfectly steady and therefore passed the rate-based gate,
    but the balance it is about to be inverted through does not hold there.
    The inversion returned CD = -0.05 for one such sample -- a drag
    coefficient no object can have -- and a sustained lull would have dragged
    the median with it.
    """
    n = 200
    log = np.zeros((n, 7))
    log[:, 0] = np.arange(n) * 0.05
    log[:, 1] = 50.0
    log[:, 3] = 35.0
    log[:, 4] = 6.0

    log[:, 2] = 0.05                      # slack, but rock steady
    assert quasi_static(log).sum() == 0

    log[:, 2] = 5.0                       # flying, same steadiness
    assert quasi_static(log).sum() > 0


def test_quasi_static_rejects_samples_whose_sag_is_out_of_range():
    """delta ~ 1/T leaves the small-sag form's range before the line is slack."""
    n = 200
    log = np.zeros((n, 7))
    log[:, 0] = np.arange(n) * 0.05
    log[:, 1] = 100.0                     # long line, so sag bites sooner
    log[:, 2] = 0.4
    log[:, 3] = 35.0
    log[:, 4] = 6.0
    assert quasi_static(log).sum() == 0


def test_identification_is_unmoved_by_a_lull_in_the_log():
    """The gate has to protect the estimate, not just mark the rows."""
    kite, tether = Kite(), SILK
    log = synthesize(kite, tether, (5.0, 6.0, 7.0), seconds_each=30.0, seed=3)
    clean = identify(log, kite, tether, bootstrap=200)

    spoiled = log.copy()
    spoiled[: len(spoiled) // 5, 2] = 0.05        # a fifth of it goes slack
    dirty = identify(spoiled, kite, tether, bootstrap=200)

    assert dirty["cl"] == pytest.approx(clean["cl"], rel=0.05)
    assert dirty["cd"] == pytest.approx(clean["cd"], rel=0.10)
