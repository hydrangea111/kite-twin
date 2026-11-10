"""Tests for the dimensional analysis.

The similarity test is itself a validation tool, so it needs its own tests --
a collapse check that cannot fail proves nothing. These confirm that the
transform really holds the Pi groups, that matched systems collapse, and that
deliberately broken ones do not.
"""

import numpy as np
import pytest
from dataclasses import replace

from model import equilibrium
from params import G, LINE_LENGTH, RHO_AIR, SILK, WIND_BASE, Kite
from scaling import dimensionless_state, froude_mu, pi_groups, rescale

BASE = (Kite(), SILK, LINE_LENGTH, WIND_BASE)


def test_transform_holds_every_pi_group():
    ref = pi_groups(*BASE)
    for lam in (0.1, 0.5, 3.0, 10.0):
        got = pi_groups(*rescale(*BASE, lam))
        for name, value in ref.items():
            assert got[name] == pytest.approx(value, rel=1e-12), name


def test_matched_systems_give_identical_dimensionless_state():
    ref = dimensionless_state(*BASE)
    for lam in (0.2, 0.5, 2.0, 5.0):
        got = dimensionless_state(*rescale(*BASE, lam))
        for key, value in ref.items():
            assert got[key] == pytest.approx(value, rel=1e-9), f"{key} at λ={lam}"


def test_the_transform_really_changes_the_hardware():
    """A collapse is only meaningful if the systems are genuinely different."""
    k, t, L, v = rescale(*BASE, 0.2)
    base_kite, base_tether, base_L, base_v = BASE
    assert k.area < base_kite.area / 10
    assert k.mass < base_kite.mass / 50
    assert t.EA < base_tether.EA / 50
    assert L < base_L / 4
    assert v < base_v / 2


def test_froude_mismatch_breaks_only_the_frequency():
    """The finding that exposed the missing group.

    Four of the five outputs do not contain Fr, so they collapse even when it
    is not held. The reduced frequency does, and it is the only one that
    separates -- which is what pointed at the omission rather than at a bug
    in the force balance.
    """
    ref = dimensionless_state(*BASE)
    bad = dimensionless_state(*rescale(*BASE, 0.2, mu=0.7))   # Fr not matched
    for key, value in ref.items():
        if "진동수" in key:
            assert bad[key] != pytest.approx(value, rel=1e-3)
        else:
            assert bad[key] == pytest.approx(value, rel=1e-9), key


def test_added_mass_ratio_is_fixed_by_the_other_groups():
    """m_a/m = (8/3π^1.5)·Π_W·Π_A/Fr², so added mass adds no new group."""
    from model import added_mass
    kite, tether, L, v = BASE
    p = pi_groups(*BASE)
    predicted = (8.0 / (3.0 * np.pi**1.5)) * p["Pi_W"] * p["Pi_A"] / p["Fr"]**2
    assert added_mass(kite) / kite.mass == pytest.approx(predicted, rel=1e-12)


def test_reduced_frequency_follows_the_algebra():
    """f L/v = (1/2π)·√(Π_E·Π_W/(1+m_a/m))/Fr.

    The simple √λ/μ form held only while the model had no added mass. Once it
    did, a Froude-mismatched transform changed m_a/m too -- because that ratio
    depends on Π_A, the group the analysis had left out. This checks the full
    expression instead of the simplified one.
    """
    from model import added_mass
    for lam, mu in ((0.2, 0.7), (2.0, 1.5), (0.5, 1.0)):
        args = rescale(*BASE, lam, mu=mu)
        p = pi_groups(*args)
        kite = args[0]
        k = added_mass(kite) / kite.mass
        expected = (1.0 / (2 * np.pi)) * np.sqrt(
            p["Pi_E"] * p["Pi_W"] / (1.0 + k)) / p["Fr"]
        got = dimensionless_state(*args)["환산진동수 f·L/v"]
        assert got == pytest.approx(expected, rel=1e-9), f"λ={lam}, μ={mu}"


def test_a_wrongly_scaled_term_is_detected():
    """Scale the tether diameter by λ^1.5 instead of λ."""
    kite, tether, L, v = BASE
    ref = dimensionless_state(*BASE)
    k2, t2, L2, v2 = rescale(*BASE, 0.2)
    broken = dimensionless_state(k2, replace(t2, diameter=tether.diameter * 0.2**1.5),
                                 L2, v2)
    moved = [k for k in ref if broken[k] != pytest.approx(ref[k], rel=1e-4)]
    assert moved, "차원이 틀린 항이 상사 시험을 통과했습니다"


def test_froude_mu_is_the_square_root():
    for lam in (0.01, 0.25, 1.0, 4.0):
        assert froude_mu(lam) == pytest.approx(np.sqrt(lam))
    # and it is the value that keeps Fr fixed
    ref = pi_groups(*BASE)["Fr"]
    for lam in (0.3, 3.0):
        assert pi_groups(*rescale(*BASE, lam))["Fr"] == pytest.approx(ref, rel=1e-12)


def test_flight_is_decided_by_one_group():
    """Area, mass and wind enter only through Π_W, so the flying and the
    non-flying samples must not overlap in it."""
    flying, grounded = [], []
    for area in (0.3, 0.5, 0.8):
        for speed in (3.0, 4.5, 6.0, 9.0):
            for scale in (0.7, 1.0, 1.5):
                kite = replace(Kite(), area=area,
                               mass_canopy=Kite().mass_canopy * scale,
                               mass_payload=Kite().mass_payload * scale)
                pw = pi_groups(kite, SILK, LINE_LENGTH, speed)["Pi_W"]
                _, _, elev = equilibrium(kite, SILK, speed, LINE_LENGTH)
                (flying if elev > 5.0 else grounded).append(pw)
    assert flying and grounded
    assert min(flying) > max(grounded), (
        f"Π_W 구간이 겹칩니다: 체공 최소 {min(flying):.3f}, "
        f"비체공 최대 {max(grounded):.3f}")


def test_pi_w_matches_its_definition():
    kite, tether, L, v = BASE
    expected = RHO_AIR * kite.area * v**2 / (kite.mass * G)
    assert pi_groups(*BASE)["Pi_W"] == pytest.approx(expected, rel=1e-12)
