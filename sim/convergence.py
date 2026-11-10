"""Are the results converged, and what do they cost?

Every integration so far ran at rtol=1e-8 with Radau, chosen once and never
checked. That is two unexamined claims at once: that the answers are converged
(precision) and that this much work is needed (cost). Both are measurable.

Reports, for each representative computation, the value against a trusted
reference and the time to get there, so the cheapest setting that still lands
inside a stated error can be chosen rather than guessed.
"""

import time

import numpy as np

import wind
from model import equilibrium, simulate
from params import LINE_LENGTH, SILK, WIND_BASE, Kite

TARGET = 1e-4          # acceptable relative error on reported quantities
REFERENCE = dict(method="Radau", rtol=1e-11, atol=1e-13)


def gust_field():
    return wind.gust(wind.uniform(WIND_BASE), peak_ratio=10.0 / 6.0,
                     start=2.0, duration=1.5)


def run_gust(kite, tether, method, rtol, atol, n_out=800):
    base, _, _ = equilibrium(kite, tether, WIND_BASE, LINE_LENGTH)
    started = time.perf_counter()
    res = simulate(kite, tether, gust_field(), lambda t: LINE_LENGTH, base,
                   20.0, n_out=n_out, method=method, rtol=rtol, atol=atol)
    return res["tension"].max(), time.perf_counter() - started


def run_energy(kite, tether, method, rtol, atol):
    """Undamped spring-pendulum; drift is a direct integrator quality measure."""
    from dataclasses import replace
    from params import G

    quiet_kite = replace(kite, cl=0.0, cd=0.0)
    quiet_tether = replace(tether, damping_ratio=0.0, mass_per_m=0.0,
                           diameter=0.0)
    angle = np.radians(50.0)
    r0 = LINE_LENGTH * 1.002 * np.array([np.cos(angle), np.sin(angle)])
    state0 = np.concatenate([r0, np.zeros(2)])

    started = time.perf_counter()
    res = simulate(quiet_kite, quiet_tether, wind.uniform(0.0),
                   lambda t: LINE_LENGTH, state0, 20.0, n_out=400,
                   method=method, rtol=rtol, atol=atol)
    stretch = np.maximum(res["radius"] - LINE_LENGTH, 0.0)
    energy = (0.5 * quiet_kite.mass * np.sum(res["v"] ** 2, axis=1)
              + quiet_kite.mass * G * res["r"][:, 1]
              + 0.5 * (quiet_tether.EA / LINE_LENGTH) * stretch ** 2)
    return np.ptp(energy) / abs(energy[0]), time.perf_counter() - started


if __name__ == "__main__":
    kite = Kite()

    print("=" * 78)
    print("1. 돌풍 최대장력 — 허용오차와 비용")
    print("=" * 78)
    ref, ref_time = run_gust(kite, SILK, **REFERENCE)
    print(f"  기준값 {ref:.6f} N (Radau rtol=1e-11, {ref_time:.2f}s)")
    print()
    print(f"  {'방법':<8}{'rtol':>8}{'최대장력':>14}{'상대오차':>12}{'시간':>9}"
          f"{'기준대비':>10}")
    best = {}
    for method in ("Radau", "LSODA", "RK45"):
        for rtol in (1e-4, 1e-6, 1e-8, 1e-10):
            try:
                value, elapsed = run_gust(kite, SILK, method, rtol, rtol * 1e-2)
            except Exception as exc:
                print(f"  {method:<8}{rtol:>8.0e}  실패: {type(exc).__name__}")
                continue
            err = abs(value / ref - 1)
            mark = " <-" if err < TARGET and method not in best else ""
            if err < TARGET and method not in best:
                best[method] = (rtol, elapsed, err)
            print(f"  {method:<8}{rtol:>8.0e}{value:>14.6f}{err:>12.1e}"
                  f"{elapsed:>8.2f}s{ref_time/elapsed:>9.1f}x{mark}")
    print()
    for method, (rtol, elapsed, err) in best.items():
        print(f"  {method}: rtol={rtol:.0e}에서 오차 {err:.1e}, "
              f"{elapsed:.2f}s")
    print()

    print("=" * 78)
    print("2. 에너지 보존 — 적분기 품질 자체")
    print("=" * 78)
    print(f"  {'방법':<8}{'rtol':>8}{'에너지 드리프트':>18}{'시간':>9}")
    for method in ("Radau", "LSODA", "RK45"):
        for rtol in (1e-4, 1e-6, 1e-8, 1e-10):
            try:
                drift, elapsed = run_energy(kite, SILK, method, rtol,
                                            rtol * 1e-2)
            except Exception as exc:
                print(f"  {method:<8}{rtol:>8.0e}  실패: {type(exc).__name__}")
                continue
            print(f"  {method:<8}{rtol:>8.0e}{drift:>18.2e}{elapsed:>8.2f}s")
    print()
    print("  => 드리프트는 적분기가 스스로 만든 오차라, 물리 결과와 달리")
    print("     기준값이 필요 없습니다. 작을수록 좋고, 1e-3 아래면 충분합니다")
    print("     (test_model.py가 그 선을 검사합니다).")
    print()
