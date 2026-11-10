"""What tension does a real flight actually produce?

Structural margin so far rests on one deterministic gust: 6 -> 10 m/s over
1.5 s, peaking near 33 N. That is a guess at a worst case, not a worst case.
A flight sits in continuous turbulence, and the number that sizes the bamboo
joints and the knots is the peak over the whole flight, not over one event.

This flies the kite through an Ornstein-Uhlenbeck gust field riding on a
sheared profile -- both of which only became possible once wind became a
function of position as well as time -- and reports the load spectrum.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import wind
from model import equilibrium, simulate
from params import LINE_LENGTH, SILK, WIND_BASE, Kite

# Several shorter flights beat one long one here: the design number comes from
# the spread of per-flight maxima, so more flights is worth more than more
# seconds. Runtime is minutes either way -- the turbulence grid puts a kink in
# the right-hand side every GRID_STEP, and the solver resolves every one.
DURATION = 300.0          # s per flight
FLIGHTS = 6
INTENSITY = 0.15          # sigma_u / U, open ground near the surface
LENGTH_SCALE = 60.0       # m, turbulence integral scale
# 0.25 s puts the Nyquist at 2 Hz, just above the tether's 1.84 Hz bounce mode,
# so the field can still excite it. Coarsening this to save time would silently
# remove exactly the content that matters.
GRID_STEP = 0.25

DETERMINISTIC_GUST_PEAK = 33.0   # N, what compare_tether.py reported



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def flight(kite, tether, field, t_end=DURATION, trim=None):
    """Integrate one flight. `trim` is the field to take the initial state from.

    Trimming on the turbulent field means trimming on whatever that series
    happens to hold at t = 0, and a deep enough lull there has no equilibrium
    at all -- the solver correctly reports none, and the integrator then
    refuses a non-finite initial state. Physically you launch in the mean
    conditions, so the mean field is what the kite is trimmed to; the gust
    series starts acting from there.
    """
    source = trim if trim is not None else field
    base, _, elev = equilibrium(kite, tether, source, LINE_LENGTH)
    if not np.isfinite(elev):
        raise ValueError("이 조건에는 평형이 없습니다 (체공 불가)")
    return simulate(kite, tether, field, lambda t: LINE_LENGTH, base,
                    t_end, n_out=int(t_end * 4))


def turbulent_field(seed, base=WIND_BASE, sheared=True):
    profile = wind.shear(base) if sheared else wind.uniform(base)
    return wind.turbulence(profile, intensity=INTENSITY,
                           length_scale=LENGTH_SCALE, speed=base,
                           duration=DURATION + 10.0, step=GRID_STEP, seed=seed)


if __name__ == "__main__":
    kite = Kite()

    print("=" * 76)
    print("1. 전단(shear)이 정상상태를 어떻게 바꾸는가")
    print("=" * 76)
    print(f"  {'바람 모델':<28}{'고도각':>9}{'고도':>9}{'장력':>9}")
    for label, field in (("균일 6 m/s", wind.uniform(WIND_BASE)),
                         ("전단 (기준 6 m/s @10 m)", wind.shear(WIND_BASE))):
        state, T, elev = equilibrium(kite, SILK, field, LINE_LENGTH)
        print(f"  {label:<28}{elev:>8.1f}°{state[1]:>8.1f}m{T:>8.2f}N")
    print()
    print("  두 줄은 같은 조건이 아닙니다. 전단 쪽은 '10 m 높이에서 6 m/s'라는")
    print("  뜻이고, 연은 그보다 높이 날아 실제로 더 센 바람을 맞습니다.")
    print("  즉 이건 모델 보수성이 아니라 '풍속을 어느 높이에서 쟀는가'의 문제입니다.")
    print("  풍속계를 지면 근처에 두고 그 값을 그대로 쓰면 하중을 과소평가합니다.")
    print()

    print("=" * 76)
    print(f"2. 난류 속 {DURATION/60:.0f}분 비행의 장력 분포 "
          f"(난류강도 {INTENSITY:.0%}, 적분규모 {LENGTH_SCALE:.0f} m)")
    print("=" * 76)
    peaks = []
    runs = []
    for seed in range(FLIGHTS):
        res = flight(kite, SILK, turbulent_field(seed))
        runs.append(res)
        peaks.append(res["tension"].max())

    all_tension = np.concatenate([r["tension"] for r in runs])
    print(f"  {'통계':<22}{'장력 [N]':>12}")
    for label, value in (("평균", all_tension.mean()),
                         ("표준편차", all_tension.std()),
                         ("95 백분위", np.percentile(all_tension, 95)),
                         ("99.9 백분위", np.percentile(all_tension, 99.9)),
                         (f"최대 ({FLIGHTS}회 비행)", all_tension.max())):
        print(f"  {label:<22}{value:>12.1f}")
    print()
    print(f"  비행별 최대장력: " + ", ".join(f"{p:.1f}" for p in peaks) + " N")
    print(f"  결정론적 돌풍 추정치: {DETERMINISTIC_GUST_PEAK:.0f} N")
    ratio = max(peaks) / DETERMINISTIC_GUST_PEAK
    print(f"  -> 실제 최대 / 돌풍 추정 = {ratio:.2f}배")
    print()

    print("=" * 76)
    print("해석")
    print("=" * 76)
    if ratio > 1.1:
        print(f"  * 돌풍 하나로 잡은 {DETERMINISTIC_GUST_PEAK:.0f} N은 "
              f"과소평가였습니다. 연속 난류에서는")
        print(f"    {max(peaks):.0f} N까지 나옵니다. 구조 여유를 다시 잡으세요.")
    elif ratio < 0.9:
        print(f"  * 돌풍 하나로 잡은 {DETERMINISTIC_GUST_PEAK:.0f} N이 "
              f"오히려 보수적이었습니다.")
        print(f"    연속 난류의 최대값은 {max(peaks):.0f} N입니다.")
    else:
        print(f"  * 두 추정이 비슷합니다 ({max(peaks):.0f} N vs "
              f"{DETERMINISTIC_GUST_PEAK:.0f} N).")
    print()
    print(f"  * 비행마다 최대값이 {min(peaks):.0f}~{max(peaks):.0f} N으로 흩어집니다.")
    print(f"    설계값은 전체 표본의 분위수가 아니라 '비행별 최대값'의 분포에서")
    print(f"    잡아야 합니다. 99.9 백분위"
          f"({np.percentile(all_tension, 99.9):.0f} N)는 관측된 최대값"
          f"({max(peaks):.0f} N)보다 낮아서, 그걸 쓰면 과소설계가 됩니다.")
    print(f"    비행 {len(peaks)}회로는 꼬리를 제대로 못 봅니다. 현 단계 권장값은")
    print(f"    관측 최대 {max(peaks):.0f} N에 안전계수를 곱한 값입니다.")
    print(f"  * 명주실 파단 {SILK.break_load:.0f} N 대비 여유는 "
          f"{SILK.break_load/max(peaks):.0f}배로 여전히 충분합니다.")
    print(f"    위험한 건 실이 아니라 대나무 이음부와 매듭입니다.")
    print()

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].plot(runs[0]["t"], runs[0]["tension"], lw=0.6)
    axes[0].axhline(DETERMINISTIC_GUST_PEAK, color="r", ls="--", lw=1,
                    label=f"돌풍 추정 {DETERMINISTIC_GUST_PEAK:.0f} N")
    axes[0].set_xlabel("시간 [s]")
    axes[0].set_ylabel("장력 [N]")
    axes[0].set_title(f"난류 속 {DURATION/60:.0f}분 비행 (1회)")
    axes[0].legend(fontsize=8)

    axes[1].hist(all_tension, bins=80, color="#4C72B0")
    for q, style, name in ((np.percentile(all_tension, 95), "--", "95%"),
                           (np.percentile(all_tension, 99.9), "-", "99.9%")):
        axes[1].axvline(q, color="k", ls=style, lw=1, label=f"{name} {q:.0f} N")
    axes[1].set_xlabel("장력 [N]")
    axes[1].set_ylabel("표본 수")
    axes[1].set_yscale("log")
    axes[1].set_title(f"장력 분포 ({FLIGHTS}회 비행 합산)")
    axes[1].legend(fontsize=8)

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path("loads.png"), dpi=260)
    print("  saved out/loads.png")
