"""Prediction in both directions.

Forward: given the state now, what will tension and altitude be in h seconds?
The answer is limited by the wind, not by the model -- turbulence decorrelates
on its own timescale and nothing can be predicted past it.

Backward: given measured tension and elevation, what wind is the kite in? This
matters because the boundary-layer work showed wind at the kite differs sharply
from wind at a ground-mounted anemometer, and there is no practical way to hang
a sensor at 30 m. The kite is already there.

Modelling choice: tension is treated as quasi-static in the wind. The tether's
bounce mode is 1.84 Hz and turbulence decorrelates over ~10 s, so on the
timescales prediction cares about the kite is always near its equilibrium.
`check_quasi_static()` tests that against a real integration rather than
assuming it.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import wind as wind_module
from model import equilibrium, line_force, sag_angle, simulate
from params import G, LINE_LENGTH, RHO_AIR, SILK, WIND_BASE, Kite

INTENSITY = 0.15
LENGTH_SCALE = 60.0
TAU = LENGTH_SCALE / WIND_BASE      # Taylor's frozen-turbulence timescale
HORIZONS = (1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0)



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def wind_spread(horizon, base=WIND_BASE, tau=TAU, intensity=INTENSITY):
    """1-sigma error of persistence forecasting for an OU wind process.

    Holding the current wind is the honest baseline: it is exactly right at
    h=0 and decays to the full turbulence spread as the process forgets.
    """
    return intensity * base * np.sqrt(1 - np.exp(-2 * horizon / tau))


def tension_sensitivity(kite, tether, speed=WIND_BASE, length=LINE_LENGTH):
    """dT/dv and dz/dv around the operating point."""
    delta = 0.05
    lo, t_lo, _ = equilibrium(kite, tether, speed - delta, length)
    hi, t_hi, _ = equilibrium(kite, tether, speed + delta, length)
    return (t_hi - t_lo) / (2 * delta), (hi[1] - lo[1]) / (2 * delta)


def check_quasi_static(kite, tether):
    """Does tension really track the wind without lag at these timescales?"""
    field = wind_module.ramp(wind_module.uniform(6.0), 1.4, start=5.0,
                             duration=TAU)
    base, _, _ = equilibrium(kite, tether, field, LINE_LENGTH)
    res = simulate(kite, tether, field, lambda t: LINE_LENGTH, base, 60.0,
                   n_out=600)
    _, settled, _ = equilibrium(kite, tether, 6.0 * 1.4, LINE_LENGTH)
    final = res["tension"][-50:].mean()
    return abs(final / settled - 1)


def infer_wind(tension, measured_elev_deg, length, kite, tether):
    """Invert the force balance for wind speed.

    Uses the vertical equation, where lift dominates; the horizontal one is
    the same information through a smaller term.
    """
    measured = np.radians(measured_elev_deg)
    chord = measured + sag_angle(tension, length, measured, tether)
    rhat = np.array([np.cos(chord), np.sin(chord)])
    r = length * rhat

    # Line drag depends on the wind we are solving for, so iterate; it is a
    # small term and this converges in a couple of passes.
    speed = 6.0
    for _ in range(12):
        wind_vec = np.array([speed, 0.0])
        lf = line_force(np.zeros(2), wind_vec, r, tether, length)
        needed = tension * rhat[1] - lf[1] + kite.mass * G
        q = needed / (kite.area * kite.cl)
        if q <= 0:
            return np.nan
        speed = np.sqrt(2 * q / RHO_AIR)
    return speed


if __name__ == "__main__":
    kite = Kite()

    print("=" * 78)
    print("0. 준정적 가정 검증")
    print("=" * 78)
    err = check_quasi_static(kite, SILK)
    print(f"  바람이 {TAU:.0f} s에 걸쳐 40% 오를 때, 장력이 새 평형값과 "
          f"{100*err:.2f}% 차이")
    print(f"  => 예측이 다루는 시간 규모에서는 장력이 바람을 지연 없이 따라갑니다.")
    print()

    print("=" * 78)
    print("1. 앞방향: 얼마나 멀리 예측할 수 있는가")
    print("=" * 78)
    dT_dv, dz_dv = tension_sensitivity(kite, SILK)
    _, t0, _ = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    print(f"  민감도: 풍속 1 m/s 당 장력 {dT_dv:.2f} N, 고도 {dz_dv:.2f} m")
    print(f"  난류 상관시간 tau = {TAU:.0f} s (적분규모 {LENGTH_SCALE:.0f} m / "
          f"{WIND_BASE:.0f} m/s)")
    print()
    print(f"  {'예측 구간':>10}{'풍속 오차':>12}{'장력 오차':>14}{'고도 오차':>12}")
    for h in HORIZONS:
        sv = wind_spread(h)
        print(f"  {h:>9.0f}s{sv:>11.2f}m/s"
              f"{sv*dT_dv:>11.2f}N ({100*sv*dT_dv/t0:.0f}%)"
              f"{sv*abs(dz_dv):>10.2f}m")
    saturation = INTENSITY * WIND_BASE
    one_second = wind_spread(1.0) / saturation
    quarter = -0.5 * TAU * np.log(1 - 0.25**2)
    print()
    print(f"  풍속 오차는 {saturation:.2f} m/s에서 포화합니다 (난류 전체 크기).")
    print(f"  문제는 올라가는 속도입니다. sqrt(1-exp(-2h/tau)) 형태라 h=0 근처에서")
    print(f"  기울기가 무한대이고, 1초 만에 이미 포화의 {100*one_second:.0f}%에 "
          f"도달합니다.")
    print(f"  포화의 25%에 머무는 구간은 {quarter:.1f} s뿐입니다.")
    print()
    print(f"  => 난류는 사실상 예측 불가능합니다. 지속성 예측으로 돌풍을")
    print(f"     미리 보는 건 안 됩니다. 윈치 재배치가 수 분 걸리는 것과")
    print(f"     합치면 선제 대응은 두 배로 불가능합니다 ([3]과 같은 결론).")
    print()
    print(f"  앞방향 예측이 쓸모 있는 곳은 따로 있습니다 — '우리가 넣는 입력'에")
    print(f"  대한 응답입니다. 라인을 5 m 감으면 연이 어디로 가는지는 바람의")
    print(f"  평균값만 알면 결정론적으로 나오고, 이건 [3]의 고도 제어가 이미")
    print(f"  쓰고 있는 예측입니다. 예측 대상을 난류가 아니라 제어 입력으로")
    print(f"  잡아야 의미가 있습니다.")
    print()

    print("=" * 78)
    print("2. 뒷방향: 연을 풍속계로 쓰기")
    print("=" * 78)
    print(f"  {'참 풍속':>9}{'역산 풍속':>12}{'오차':>10}")
    for true_speed in (5.0, 6.0, 7.0, 8.0):   # below ~4.5 the kite does not fly
        state, tension, elev = equilibrium(kite, SILK, true_speed, LINE_LENGTH)
        if not np.isfinite(elev) or elev <= 0:
            continue
        measured = np.degrees(np.radians(elev)
                              - sag_angle(tension, LINE_LENGTH,
                                          np.radians(elev), SILK))
        got = infer_wind(tension, measured, LINE_LENGTH, kite, SILK)
        print(f"  {true_speed:>8.1f}{got:>11.2f}{100*(got/true_speed-1):>9.1f}%")
    print()

    rng = np.random.default_rng(0)
    state, tension, elev = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    measured = np.degrees(np.radians(elev)
                          - sag_angle(tension, LINE_LENGTH,
                                      np.radians(elev), SILK))
    noisy = [
        infer_wind(tension * (1 + rng.normal(0, 0.08)),
                   measured + rng.normal(0, 2.0), LINE_LENGTH, kite, SILK)
        for _ in range(4000)
    ]
    noisy = np.array([v for v in noisy if np.isfinite(v)])
    lo, hi = np.percentile(noisy, [2.5, 97.5])
    print(f"  센서잡음(장력 8%, 각도 2°)만 반영한 95% 구간: "
          f"{lo:.2f}~{hi:.2f} m/s (폭 {100*(hi-lo)/WIND_BASE:.0f}%)")
    print()
    print("  하지만 절대값은 이것보다 나쁩니다. CL을 풍속계로 교정했다면,")
    print("  연으로 역산한 풍속은 그 풍속계의 게인 오차를 그대로 물려받습니다.")
    print("  순환 참조라 연이 독립적인 절대 풍속계가 되지는 못합니다.")
    print()
    print("  그래도 두 가지는 확실히 얻습니다:")
    print("   * 연이 실제로 있는 높이의 바람 — 경계층 때문에 지상 풍속계와")
    print("     크게 다르고, 30 m에 센서를 달 방법은 없습니다.")
    print("   * 상대 변화와 이중화 — CL 오차는 공통으로 곱해지므로 변화율에는")
    print("     영향이 없고, 풍속계와 어긋나면 둘 중 하나가 고장난 것입니다.")
    print()

    print("=" * 78)
    print("3. 두 방향을 잇기")
    print("=" * 78)
    print("  역산 풍속 -> 지속성 예측 -> 앞방향 상태 예측, 이 고리가 트윈의")
    print("  실시간 부분입니다. 역산이 지상 풍속계보다 나은 입력인 이유는")
    print("  정확도가 아니라 위치(연 고도)이고, 예측 한계는 모델이 아니라")
    print(f"  난류 상관시간 {TAU:.0f} s가 정합니다.")
    print()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    h = np.linspace(0, 120, 400)
    axes[0].plot(h, wind_spread(h), lw=2)
    axes[0].axhline(saturation, color="k", ls="--", lw=1, label="난류 전체 크기")
    axes[0].axvline(TAU, color="r", ls=":", lw=1, label=f"tau = {TAU:.0f} s")
    axes[0].set_xlabel("예측 구간 [s]")
    axes[0].set_ylabel("풍속 예측 오차 [m/s]")
    axes[0].set_title("예측 한계는 모델이 아니라 난류가 정함")
    axes[0].legend(fontsize=8)

    axes[1].hist(noisy, bins=60, color="#4C72B0")
    axes[1].axvline(WIND_BASE, color="k", ls="--", lw=1.5, label="참값")
    axes[1].set_xlabel("역산 풍속 [m/s]")
    axes[1].set_ylabel("표본 수")
    axes[1].set_title("장력·각도에서 역산한 풍속 (센서잡음만)")
    axes[1].legend(fontsize=8)

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path("predict.png"), dpi=260)
    print("  saved out/predict.png")
