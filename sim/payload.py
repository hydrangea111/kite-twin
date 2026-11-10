"""Does the hanging payload's swing pollute the tension signal?

The 4-line rig balances itself by comparing tension between lines, read from
the BTS7960 current sense. If the payload swinging on its suspension line puts
a ripple on that signal comparable to the real line-to-line differences, the
balancing loop will chase a phantom. This runs a two-body model (kite + swinging
payload) against three suspension lengths to size that ripple.
"""

from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve

import wind
from model import aero_force, equilibrium, line_force, tension
from params import (G, LINE_LENGTH, RHO_AIR, SILK, SUSPENSION, WIND_BASE,
                    Kite, Payload)

DROPS = (0.15, 0.50, 1.50)  # m, kite-to-payload suspension lengths to compare



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def payload_drag(v, wind, payload):
    w = wind - v
    speed = np.linalg.norm(w)
    if speed < 1e-9:
        return np.zeros(2)
    return 0.5 * RHO_AIR * speed**2 * payload.area * payload.cd * (w / speed)


def rhs(t, y, kite, payload, tether, drop, wind_fn, length_fn):
    rk, rp, vk, vp = y[0:2], y[2:4], y[4:6], y[6:8]
    mk, mp = kite.mass_canopy, payload.mass

    radius = np.linalg.norm(rk)
    rhat = rk / radius
    L0 = length_fn(t)
    T = tension(radius, vk @ rhat, L0, tether, mk + mp)

    link = rp - rk
    dist = np.linalg.norm(link)
    dhat = link / dist
    T_link = tension(dist, (vp - vk) @ dhat, drop, SUSPENSION, mp)

    f_kite = (
        aero_force(vk, wind_fn(t, rk), kite, rk)
        + line_force(vk, wind_fn(t, 0.5 * rk), rk, tether, L0)
        + np.array([0.0, -mk * G])
        - T * rhat
        + T_link * dhat
    )
    f_payload = (
        payload_drag(vp, wind_fn(t, rp), payload)
        + np.array([0.0, -mp * G])
        - T_link * dhat
    )
    return np.concatenate([vk, vp, f_kite / mk, f_payload / mp])


def equilibrium2(kite, payload, tether, drop, wind_speed, L0):
    """Solved in two stages: the payload's own balance fixes the suspension
    vector in closed form, and the kite then sees that as a known extra pull.

    Solving all four coordinates at once does not converge -- at the obvious
    starting guess the suspension sits unstretched, so it carries no tension
    and the Jacobian has no gradient to follow.
    """
    field = wind_speed if callable(wind_speed) else wind.uniform(wind_speed)
    mk, mp = kite.mass_canopy, payload.mass

    # The payload hangs a drop below the kite, so it feels the wind there.
    probe = field(0.0, np.array([L0 * 0.8, L0 * 0.6 - drop]))
    pull = payload_drag(np.zeros(2), probe, payload) + np.array([0.0, -mp * G])
    T_link = np.linalg.norm(pull)
    dist = drop * (1 + T_link / SUSPENSION.EA)

    def residual(u):
        theta, radius = u
        rk = radius * np.array([np.cos(theta), np.sin(theta)])
        T = tension(radius, 0.0, L0, tether, mk + mp)
        return (
            aero_force(np.zeros(2), field(0.0, rk), kite, rk)
            + line_force(np.zeros(2), field(0.0, 0.5 * rk), rk, tether, L0)
            + np.array([0.0, -mk * G])
            + pull
            - T * rk / radius
        )

    state1, _, _ = equilibrium(kite, tether, field, L0)
    guess = [np.arctan2(state1[1], state1[0]), np.linalg.norm(state1[:2])]
    theta, radius = fsolve(residual, guess)

    rk = radius * np.array([np.cos(theta), np.sin(theta)])
    return np.concatenate([rk, rk + dist * pull / T_link, np.zeros(4)])


def gust(base=WIND_BASE, peak=10.0, t0=2.0, dur=1.5):
    return wind.gust(wind.uniform(base), peak / base, t0, dur)


def simulate(kite, payload, tether, drop, wind_fn, L0, t_end=30.0, n_out=4000):
    state0 = equilibrium2(kite, payload, tether, drop, WIND_BASE, L0)
    t_eval = np.linspace(0.0, t_end, n_out)
    sol = solve_ivp(
        rhs, (0.0, t_end), state0,
        args=(kite, payload, tether, drop, wind_fn, lambda t: L0),
        method="Radau", t_eval=t_eval, rtol=1e-6, atol=1e-8, max_step=0.1,
    )
    if not sol.success:
        raise RuntimeError(sol.message)

    rk, rp = sol.y[0:2].T, sol.y[2:4].T
    vk = sol.y[4:6].T
    radius = np.linalg.norm(rk, axis=1)
    rhat = rk / radius[:, None]
    T = np.array([
        tension(radius[i], vk[i] @ rhat[i], L0, tether, kite.mass_canopy + payload.mass)
        for i in range(len(sol.t))
    ])

    link = rp - rk
    swing = np.degrees(np.arctan2(link[:, 0], -link[:, 1]))  # from straight down
    return {"t": sol.t, "tension": T, "swing": swing, "radius": radius}


def dominant_freq(t, signal):
    """Zero-crossing estimate, robust enough for a single lightly damped mode."""
    x = signal - signal.mean()
    crossings = np.where(np.diff(np.sign(x)) != 0)[0]
    if len(crossings) < 3:
        return float("nan")
    return 0.5 * (len(crossings) - 1) / (t[crossings[-1]] - t[crossings[0]])


if __name__ == "__main__":
    # 그림을 그릴 때만 필요하다. 모듈로 import 될 때(지상국 브리지가 쓰는 길)는
    # matplotlib 이 없어도 돌고, 번들 크기도 그만큼 줄어든다.
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    import matplotlib.pyplot as plt

    kite, payload = Kite(), Payload()
    field = gust()
    results = {}
    rows = []

    for drop in DROPS:
        res = simulate(kite, payload, SILK, drop, field, LINE_LENGTH)
        results[drop] = res

        ringing = (res["t"] >= 5.0) & (res["t"] < 12.0)   # gust past, still swinging
        settled = res["t"] >= 12.0
        ripple = np.ptp(res["tension"][settled]) / 2
        steady = res["tension"][settled].mean()
        rows.append({
            "drop": drop,
            "theory": np.sqrt(G / drop) / (2 * np.pi),
            "measured": dominant_freq(res["t"][ringing], res["swing"][ringing]),
            "swing_max": np.abs(res["swing"]).max(),
            "swing_rest": res["swing"][0],
            "peak_T": res["tension"].max(),
            "ripple": ripple,
            "ripple_pct": 100 * ripple / steady,
        })

    print("=" * 74)
    print("탑재물 진자 — 목줄 길이별 (명주실 본선, 돌풍 6->10 m/s)")
    print("=" * 74)
    print(f"  {'목줄':>6}{'진자주파수(이론)':>18}{'실측':>9}{'정지각':>9}{'최대흔들림':>12}")
    for r in rows:
        print(f"  {r['drop']:>5.2f}m{r['theory']:>16.2f}Hz{r['measured']:>7.2f}Hz"
              f"{r['swing_rest']:>8.1f}°{r['swing_max']:>11.1f}°")
    print()
    print(f"  {'목줄':>6}{'돌풍 최대장력':>16}{'잔여 장력맥동':>16}")
    for r in rows:
        print(f"  {r['drop']:>5.2f}m{r['peak_T']:>14.1f}N"
              f"{r['ripple']:>12.2f}N ({r['ripple_pct']:.0f}%)")
    print()

    worst_ripple = max(r["ripple_pct"] for r in rows)
    worst_swing = max(r["swing_max"] for r in rows)
    print("=" * 74)
    print("해석")
    print("=" * 74)
    print(f"  1. 장력 신호 오염은 작습니다. 흔들림이 본선 장력에 남기는 맥동이")
    print(f"     최대 {worst_ripple:.0f}% ({max(r['ripple'] for r in rows):.2f} N)로, "
          f"BTS7960 전류센싱 자체 오차(보통 5~10%)보다")
    print(f"     작습니다 -> 4라인 장력 밸런싱이 진자 때문에 속을 걱정은 안 해도 됩니다.")
    print()
    print(f"  2. 그런데 흔들림 자체는 큽니다. 정지 시에도 항력 때문에 "
          f"{abs(rows[0]['swing_rest']):.0f}° 뒤로 기운 채")
    print(f"     매달리고, 돌풍 때는 최대 {worst_swing:.0f}°까지 흔들립니다.")
    print(f"     -> 페로브스카이트 패널을 여기 매달면 입사각이 그만큼 바뀝니다.")
    print(f"        cos({worst_swing:.0f}°) = {np.cos(np.radians(worst_swing)):.2f} 이므로 "
          f"발전량이 최대 {100*(1-np.cos(np.radians(worst_swing))):.0f}%까지 깎입니다.")
    print(f"        발전량 측정([4])에서는 이게 바람보다 큰 교란 요인입니다.")
    print(f"        자세각([5])을 같이 기록해서 입사각 보정을 해야 의미 있는 데이터가 됩니다.")
    print()
    print(f"  3. 목줄 길이: 길수록 진자가 느려집니다 "
          f"({rows[0]['theory']:.2f} -> {rows[-1]['theory']:.2f} Hz).")
    print(f"     짧게 매달면 주파수가 올라가 제어 대역과 분리되고 필터링도 쉬워집니다.")
    print(f"     흔들림 진폭 자체는 목줄 길이와 거의 무관하니("
          f"{min(r['swing_max'] for r in rows):.0f}~{worst_swing:.0f}°),")
    print(f"     짧게 매다는 쪽이 유리합니다.")
    print()

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for drop, res in results.items():
        axes[0].plot(res["t"], res["tension"], label=f"목줄 {drop:.2f} m")
        axes[1].plot(res["t"], res["swing"], label=f"목줄 {drop:.2f} m")
    axes[0].set_ylabel("본선 장력 [N]")
    axes[1].set_ylabel("탑재물 흔들림 각 [deg]")
    axes[1].set_xlabel("시간 [s]")
    axes[0].set_title("탑재물 진자가 장력 신호에 남기는 맥동")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(out_path("payload_swing.png"), dpi=260)
    print("  saved out/payload_swing.png")
