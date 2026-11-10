"""Verification against things we know independently of the code.

These check the dynamics against analytic results and conservation laws, not
against previously recorded output -- a regression test that only compares to
yesterday's numbers would happily lock in a bug. Run with: pytest -q
"""

from dataclasses import replace

import numpy as np
import pytest

import wind
from model import (aero_force, equilibrium, line_force, min_area_for_elevation, rhs,
                   simulate, tension)
from params import (G, LINE_LENGTH, RHO_AIR, SILK, SUSPENSION_LENGTH, TETHERS,
                    WIND_BASE, Kite, Payload)

WIND = np.array([WIND_BASE, 0.0])


def steady(speed=WIND_BASE):
    return wind.uniform(speed)


def fixed(length=LINE_LENGTH):
    return lambda t: length


# ---------------------------------------------------------------- tether law

def test_slack_line_carries_no_tension():
    assert tension(49.0, 0.0, 50.0, SILK, 1.0) == 0.0
    assert tension(50.0, 0.0, 50.0, SILK, 1.0) == 0.0


def test_slack_line_carries_no_tension_while_moving():
    """A string cannot push, however fast the kite is coming back at you.

    The strain guard looks redundant next to the max(0, ...) that follows it,
    and for a visibly slack line it is -- EA times a large negative strain
    swamps anything the damper contributes. It stops being redundant when the
    slack is small: at 1 cm loose on a 50 m line, EA*strain is -0.6 N while
    the damping term at 5 m/s is +1.56 N, so without the guard the line pulls
    with 0.96 N. Mutation testing found this: deleting the guard left all
    other tests passing, because none of them moved a slack line.
    """
    for slack_m, rate in ((0.01, 5.0), (0.005, 5.0), (0.01, 2.0), (0.002, 1.0)):
        t = tension(50.0 - slack_m, rate, 50.0, SILK, 0.45)
        assert t == 0.0, f"{slack_m*100:g} cm 느슨한 줄이 {rate} m/s 에서 {t:.3f} N"


def test_tension_follows_hookes_law():
    strain = 0.01
    t = tension(50.0 * (1 + strain), 0.0, 50.0, SILK, 1.0)
    assert t == pytest.approx(SILK.EA * strain, rel=1e-12)


def test_damping_cannot_push():
    """A fast-shortening taut line must not report negative tension."""
    assert tension(50.05, -100.0, 50.0, SILK, 1.0) == 0.0


# ---------------------------------------------------------------- aero + line

def test_line_drag_scales_with_wind_squared():
    r = np.array([30.0, 40.0])
    one = line_force(np.zeros(2), np.array([5.0, 0.0]), r, SILK, 50.0)
    two = line_force(np.zeros(2), np.array([10.0, 0.0]), r, SILK, 50.0)
    weight = line_force(np.zeros(2), np.zeros(2), r, SILK, 50.0)
    assert (two - weight)[0] == pytest.approx(4 * (one - weight)[0], rel=1e-9)


def test_line_weight_is_half_the_span():
    r = np.array([30.0, 40.0])
    f = line_force(np.zeros(2), np.zeros(2), r, SILK, 50.0)
    assert f[1] == pytest.approx(-0.5 * SILK.mass_per_m * 50.0 * G, rel=1e-12)


def test_sag_angle_splits_the_cable_weight_evenly():
    """tan(theta+d) - tan(theta-d) must equal W/H, the whole cable weight."""
    from model import sag_angle

    T, L, elev = 8.0, 50.0, np.radians(35.0)
    d = sag_angle(T, L, elev, SILK)
    H = T * np.cos(elev)
    weight = SILK.mass_per_m * L * G
    spread = np.tan(elev + d) - np.tan(elev - d)
    assert spread == pytest.approx(weight / H, rel=0.02)


def test_sag_angle_vanishes_on_a_weightless_line():
    from model import sag_angle

    weightless = replace(SILK, mass_per_m=0.0)
    assert sag_angle(8.0, 50.0, np.radians(35.0), weightless) == 0.0


def test_sag_angle_grows_as_tension_falls():
    from model import sag_angle

    slack = sag_angle(3.0, 50.0, np.radians(35.0), SILK)
    taut = sag_angle(18.0, 50.0, np.radians(35.0), SILK)
    assert slack > taut
    assert slack == pytest.approx(taut * 6.0, rel=1e-9)


def test_aero_lift_is_perpendicular_to_apparent_wind():
    r = np.array([30.0, 40.0])
    kite = Kite()
    f = aero_force(np.zeros(2), WIND, kite, r)
    drag_only = replace(kite, cl=0.0)
    d = aero_force(np.zeros(2), WIND, drag_only, r)
    lift = f - d
    assert abs(lift @ (WIND / np.linalg.norm(WIND))) < 1e-9


# ---------------------------------------------------------------- equilibrium

def test_equilibrium_has_no_residual_acceleration():
    # Blind by construction to bugs inside the force functions: the solver and
    # the dynamics call the same code, so they agree even when both are wrong.
    # The independent analytic checks above are what actually catch those.
    kite = Kite()
    state, _, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    accel = rhs(0.0, state, kite, SILK, steady(), fixed())[2:]
    assert np.linalg.norm(accel) < 1e-6


def test_equilibrium_tension_matches_stretch():
    kite = Kite()
    state, T, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    strain = (np.linalg.norm(state[:2]) - LINE_LENGTH) / LINE_LENGTH
    assert T == pytest.approx(SILK.EA * strain, rel=1e-6)


def test_stiffer_tether_stretches_less():
    kite = Kite()
    soft, _, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    hard, _, _ = equilibrium(kite, replace(SILK, EA=10 * SILK.EA),
                             WIND_BASE, LINE_LENGTH)
    assert np.linalg.norm(hard[:2]) < np.linalg.norm(soft[:2])


def test_equilibrium_stays_put_when_simulated():
    kite = Kite()
    state, _, elev = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    res = simulate(kite, SILK, steady(), fixed(), state, 10.0, n_out=50)
    assert res["elevation"] == pytest.approx(elev, abs=1e-3)


# ---------------------------------------------------------------- conservation

def test_energy_is_conserved_without_dissipation():
    """Strip every dissipative term and the spring-pendulum must hold its energy.

    This exercises the integrator and the whole force assembly at once: a sign
    error or a double-counted force shows up here as drift.
    """
    kite = replace(Kite(), cl=0.0, cd=0.0)
    tether = replace(SILK, damping_ratio=0.0, mass_per_m=0.0, diameter=0.0)
    m, L0 = kite.mass, LINE_LENGTH

    angle = np.radians(50.0)
    r0 = L0 * 1.002 * np.array([np.cos(angle), np.sin(angle)])
    state0 = np.concatenate([r0, np.zeros(2)])

    res = simulate(kite, tether, wind.uniform(0.0), fixed(L0),
                   state0, 20.0, n_out=800)

    radius = res["radius"]
    stretch = np.maximum(radius - L0, 0.0)
    k = tether.EA / L0
    energy = (0.5 * m * np.sum(res["v"] ** 2, axis=1)
              + m * G * res["r"][:, 1]
              + 0.5 * k * stretch ** 2)

    drift = np.ptp(energy) / abs(energy[0])
    assert drift < 1e-3, f"energy drifted {drift:.2e}"


# ---------------------------------------------------------------- two-body

def test_payload_hangs_at_its_drag_angle():
    from payload import equilibrium2

    kite, payload = Kite(), Payload()
    state = equilibrium2(kite, payload, SILK, SUSPENSION_LENGTH,
                         WIND_BASE, LINE_LENGTH)
    link = state[2:4] - state[0:2]
    measured = np.arctan2(link[0], -link[1])

    drag = 0.5 * RHO_AIR * WIND_BASE**2 * payload.area * payload.cd
    expected = np.arctan2(drag, payload.mass * G)
    assert measured == pytest.approx(expected, rel=1e-6)


def test_swing_frequency_matches_simple_pendulum():
    from payload import dominant_freq, gust, simulate as simulate2

    drop = 0.5
    res = simulate2(Kite(), Payload(), SILK, drop, gust(), LINE_LENGTH,
                    t_end=20.0, n_out=4000)
    window = (res["t"] >= 5.0) & (res["t"] < 15.0)
    measured = dominant_freq(res["t"][window], res["swing"][window])
    theory = np.sqrt(G / drop) / (2 * np.pi)
    assert measured == pytest.approx(theory, rel=0.20)


# ---------------------------------------------------------------------------
# added mass
# ---------------------------------------------------------------------------

def test_added_mass_matches_the_disc_result():
    from model import added_mass
    kite = Kite()
    a = np.sqrt(kite.area / np.pi)
    assert added_mass(kite) == pytest.approx(8.0 / 3.0 * RHO_AIR * a**3, rel=1e-12)


def test_added_mass_is_a_large_fraction_of_the_airframe():
    """If it were small the term would not be worth carrying; it is not."""
    from model import added_mass
    kite = Kite()
    assert added_mass(kite) / kite.mass > 0.3


def test_factor_zero_recovers_the_plain_newton_step():
    """The term has to be switchable, or its effect cannot be measured."""
    from model import rhs
    kite, tether = Kite(), SILK
    field = wind.uniform(WIND_BASE)
    state, _, _ = equilibrium(kite, tether, field, LINE_LENGTH)
    y = np.concatenate([state[:2], np.array([0.4, -0.3])])
    off = rhs(0.0, y, kite, tether, field, lambda t: LINE_LENGTH, am_factor=0.0)

    r, v = y[:2], y[2:]
    radius = np.linalg.norm(r)
    rhat = r / radius
    T = tension(radius, v @ rhat, LINE_LENGTH, tether, kite.mass)
    force = (aero_force(v, field(0.0, r), kite, r)
             + line_force(v, field(0.0, 0.5 * r), r, tether, LINE_LENGTH)
             + np.array([0.0, -kite.mass * G]) - T * rhat)
    assert off[2:] == pytest.approx(force / kite.mass, rel=1e-12)


def test_added_mass_only_resists_broadside_motion():
    """Potential flow gives zero added mass in the plane of the plate."""
    from model import added_mass, rhs
    kite, tether = Kite(), SILK
    field = wind.uniform(WIND_BASE)
    state, _, _ = equilibrium(kite, tether, field, LINE_LENGTH)
    y = np.concatenate([state[:2], np.array([0.3, 0.2])])

    on = rhs(0.0, y, kite, tether, field, lambda t: LINE_LENGTH)
    off = rhs(0.0, y, kite, tether, field, lambda t: LINE_LENGTH, am_factor=0.0)

    aero = aero_force(y[2:], field(0.0, y[:2]), kite, y[:2])
    nhat = aero / np.linalg.norm(aero)
    that = np.array([-nhat[1], nhat[0]])

    # In-plane acceleration is untouched; the normal one is reduced.
    assert (on[2:] @ that) == pytest.approx(off[2:] @ that, rel=1e-9)
    assert abs(on[2:] @ nhat) < abs(off[2:] @ nhat)
    ratio = (on[2:] @ nhat) / (off[2:] @ nhat)
    assert ratio == pytest.approx(
        kite.mass / (kite.mass + added_mass(kite)), rel=1e-9)


def test_added_mass_lowers_the_bounce_frequency_without_changing_ratios():
    """The same air rides on every tether, so comparisons are unaffected.

    This is why the uncertain coefficient does not threaten the elasticity
    conclusions -- it is a common factor across the cases being compared.
    """
    from model import added_mass
    kite = Kite()
    eff = kite.mass + added_mass(kite)
    plain = [t.natural_freq(LINE_LENGTH, kite.mass) for t in TETHERS]
    with_am = [t.natural_freq(LINE_LENGTH, eff) for t in TETHERS]

    assert all(b < a for a, b in zip(plain, with_am))
    assert with_am[0] / plain[0] == pytest.approx(0.828, abs=0.01)
    ratios_plain = [f / plain[0] for f in plain]
    ratios_am = [f / with_am[0] for f in with_am]
    assert ratios_am == pytest.approx(ratios_plain, rel=1e-12)


# ---------------------------------------------------------------------------
# the equilibrium solver reports failure instead of inventing a root
# ---------------------------------------------------------------------------

# One of the 148 parameter draws (of 2000) on which fsolve does not converge.
# Not a contrived case: it sits in the middle of the uncertainty ranges the
# report already carries, which is why 5.5% of the study hit it.
NO_SOLUTION = dict(area=0.472, mass_canopy=0.154, mass_payload=0.300,
                   cl=0.503, cd=0.273)
NO_SOLUTION_WIND = 5.67


def test_equilibrium_returns_nan_when_the_solve_does_not_converge():
    """Saying "no answer" is the only honest output of a failed solve.

    Before this check the solver handed back its last iterate -- elevations
    around -550 deg with a 1.6 N residual on a balance that closes to 1e-10
    when it converges. Callers filtered those out by sign, which worked only
    because the garbage happened to come out negative.
    """
    kite = replace(Kite(), **NO_SOLUTION)
    tether = replace(SILK, EA=1939.0)
    _, tension_out, elev = equilibrium(kite, tether, NO_SOLUTION_WIND, LINE_LENGTH)
    assert np.isnan(elev)
    assert np.isnan(tension_out)


def test_a_root_below_the_horizon_is_not_offered_as_a_solution():
    """A kite too small to lift itself has a mathematical root below the
    horizon, and the old solver returned it for callers to reject on sign.

    The bracketed solver searches only above the horizon, so those never
    appear. That is the better contract: below the horizon the outward-lift
    bridle assumption does not describe a kite, so the number there was never
    a flying state -- it was an artefact every caller had to remember to
    filter. Rejecting it at the source means nobody has to remember.
    """
    kite = replace(Kite(), area=0.05)
    _, _, elev = equilibrium(kite, SILK, 2.0, LINE_LENGTH)
    assert np.isnan(elev)


def test_the_solver_finds_the_steepest_root():
    """Several crossings can exist; the physical one is the highest."""
    state, T, elev = equilibrium(Kite(), SILK, 6.0, LINE_LENGTH)
    assert 0 < elev < 89
    # and it is the one the two-dimensional solver used to find
    assert elev == pytest.approx(38.326, abs=0.01)


def test_equilibrium_result_actually_satisfies_the_balance():
    """Whatever it returns non-NaN must close the force balance."""
    for speed in (4.0, 6.0, 9.0, 12.0):
        state, T, elev = equilibrium(Kite(), SILK, speed, LINE_LENGTH)
        if np.isnan(elev):
            continue
        r = state[:2]
        radius = np.linalg.norm(r)
        residual = (aero_force(np.zeros(2), wind.uniform(speed)(0.0, r), Kite(), r)
                    + line_force(np.zeros(2), wind.uniform(speed)(0.0, 0.5 * r),
                                 r, SILK, LINE_LENGTH)
                    + np.array([0.0, -Kite().mass * G])
                    - T * r / radius)
        assert np.linalg.norm(residual) < 1e-6, f"{speed} m/s"


def test_a_failed_solve_cannot_be_mistaken_for_a_low_angle():
    """NaN is not less than anything, so a `< 5` test cannot swallow it."""
    kite = replace(Kite(), **NO_SOLUTION)
    tether = replace(SILK, EA=1939.0)
    _, _, elev = equilibrium(kite, tether, NO_SOLUTION_WIND, LINE_LENGTH)
    assert not (elev < 5.0)
    assert not (elev > 5.0)

# ── 변이 시험이 찾아낸 구멍 ──────────────────────────────────────────
# model.py 변이 111개 중 33개가 살아남았다. 대부분은 평형 해법의 빠른 경로
# 안이라 등가였다 -- 뉴턴 결과를 망가뜨려도 검증에 걸려 괄호 탐색으로 넘어가
# 같은 답이 나오기 때문이고, 그것이 그 구조의 목적이다. 아래 넷은 등가가
# 아니라 아무도 보고 있지 않던 동작이다.

def test_min_area_for_elevation_falls_off_with_elevation_and_goes_infinite_past_LD():
    """필요 면적 식. 0.64 m² 권고가 여기서 나오므로 틀리면 보고서가 틀린다."""
    k = Kite()
    a30 = min_area_for_elevation(k, 6.0, 30.0)
    a45 = min_area_for_elevation(k, 6.0, 45.0)
    assert a30 < a45, "고도각을 높이려면 면적이 더 필요해야 한다"

    # L/D 한계각을 넘으면 어떤 면적으로도 불가능하다
    ld = np.degrees(np.arctan(k.cl / k.cd))
    assert np.isinf(min_area_for_elevation(k, 6.0, ld + 1.0))
    assert np.isfinite(min_area_for_elevation(k, 6.0, ld - 1.0))

    # 면적은 풍속 제곱에 반비례한다
    assert min_area_for_elevation(k, 12.0, 30.0) == pytest.approx(a30 / 4.0, rel=1e-12)


def test_lift_always_points_away_from_the_ground_station():
    """브라이들 가정: 양력은 줄 바깥쪽을 향한다. 안쪽이면 연이 아니다."""
    k = Kite()
    wind = np.array([6.0, 0.0])
    for deg in (5.0, 20.0, 45.0, 70.0, 85.0):
        r = 50.0 * np.array([np.cos(np.radians(deg)), np.sin(np.radians(deg))])
        f = aero_force(np.zeros(2), wind, k, r)
        what = wind / np.linalg.norm(wind)
        lift = f - (f @ what) * what
        assert lift @ (r / np.linalg.norm(r)) > 0, f"{deg}°에서 양력이 안쪽"


def test_lift_still_points_outward_when_the_kite_is_dropping():
    """뒤집기 분기는 겉보기바람이 기울 때만 돈다.

    바람이 수평이고 연이 가만히 있으면 양력은 언제나 위를 향하므로 부호를
    바로잡는 분기가 아예 실행되지 않는다. 정적인 조건만 보는 시험은 그 분기를
    지워도 통과한다 -- 변이 시험이 그렇게 잡아냈다. 연이 빠르게 내려오면
    겉보기바람이 기울고, 그때 분기가 일을 한다.
    """
    k = Kite()
    wind_vec = np.array([6.0, 0.0])
    r = 50.0 * np.array([np.cos(np.radians(45.0)), np.sin(np.radians(45.0))])
    rhat = r / np.linalg.norm(r)
    for vz in (-10.0, -20.0, -30.0):
        v = np.array([0.0, vz])
        f = aero_force(v, wind_vec, k, r)
        w = wind_vec - v
        what = w / np.linalg.norm(w)
        lift = f - (f @ what) * what
        assert lift @ rhat > 0, f"하강 {abs(vz)} m/s 에서 양력이 줄 안쪽을 향함"


def test_line_drag_uses_wind_relative_to_the_moving_kite():
    """줄 항력은 겉보기바람에 걸린다. 연이 바람을 따라가면 줄어들어야 한다."""
    still = line_force(np.zeros(2), np.array([6.0, 0.0]),
                       np.array([35.0, 35.0]), SILK, 50.0)
    chasing = line_force(np.array([4.0, 0.0]), np.array([6.0, 0.0]),
                         np.array([35.0, 35.0]), SILK, 50.0)
    assert abs(chasing[0]) < abs(still[0]), "연이 바람을 따라가도 항력이 그대로"


def test_sheared_equilibrium_actually_balances():
    """전단 바람에서도 수직 방향 힘이 0이어야 평형이다.

    늘어남 반복이 두 번으로 고정돼 있을 때는 전단 바람에서 잔차가 1e-3 N
    남았다. 균일 바람에서는 1e-12 이라 아무 테스트도 눈치채지 못했다.
    """
    k = Kite()
    for speed in (5.0, 8.0, 10.0):
        field = wind.shear(speed)
        state, _, elev = equilibrium(k, SILK, field, LINE_LENGTH)
        assert np.isfinite(elev)
        r = state[:2]
        rhat = r / np.linalg.norm(r)
        total = (aero_force(np.zeros(2), field(0.0, r), k, r)
                 + line_force(np.zeros(2), field(0.0, 0.5 * r), r, SILK, LINE_LENGTH)
                 + np.array([0.0, -k.mass * G]))
        perp = np.linalg.norm(total - (total @ rhat) * rhat)
        assert perp < 1e-9, f"{speed} m/s 전단에서 수직 잔차 {perp:.2e} N"
