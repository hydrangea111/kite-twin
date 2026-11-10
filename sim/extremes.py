"""What load should the structure actually be built to?

loads.py showed that per-flight maximum tension scatters nearly two to one
across six flights, and concluded with "observed maximum times a safety
factor". That is what you write down when you do not know the distribution,
not a design value. Six blocks cannot resolve a tail.

This runs the block-maxima analysis properly: many independent flights, the
maximum from each, and an extreme-value distribution fitted to those maxima.
That gives a return level -- the load exceeded once in N flights -- with an
interval on it, instead of a number that depends on how many times you
happened to run the simulation.

Three things are checked rather than assumed:

  * whether the blocks are long enough for the asymptotics, by refitting on
    half-length blocks and seeing if the fit moves;
  * whether the tail is Gumbel or something heavier, by letting the GEV shape
    float and seeing whether it is distinguishable from zero;
  * whether any of this precision survives the parameter uncertainty, which
    is the part the report keeps finding is the binding constraint.

    python extremes.py            # full run, ~25 min on 12 cores
    python extremes.py --quick    # 60 flights, for checking it works
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import argparse
import os
import time
from concurrent.futures import ProcessPoolExecutor

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

FORCE = False   # True 로 두면 캐시를 무시하고 다시 적분한다
from scipy import stats

import wind
from loads import DURATION, GRID_STEP, INTENSITY, LENGTH_SCALE, flight
from model import equilibrium, simulate
from params import LINE_LENGTH, SILK, WIND_BASE, Kite
from uncertainty import RANGES, build, draw

# 150 blocks, not 400. The convergence study below is what sets this: the
# 100-flight return level stops moving past about 50 blocks (+/-1.2 N), so
# 400 was buying precision the parameter uncertainty swamps anyway -- and it
# cost an hour with no way to see how far along it was.
FLIGHTS = 150          # block maxima for the main fit
PARAM_DRAWS = 12       # parameter samples for the sensitivity ensemble
PARAM_FLIGHTS = 12     # flights per parameter sample
RETURN_PERIODS = (10, 100, 1000)
CURRENT_DESIGN = 50.0        # N, what the report recommends today
DETERMINISTIC_GUST = 33.0    # N, the single-gust estimate it replaced



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def turbulent_field(seed, base=WIND_BASE, duration=DURATION):
    return wind.turbulence(wind.shear(base), intensity=INTENSITY,
                           length_scale=LENGTH_SCALE, speed=base,
                           duration=duration + 10.0, step=GRID_STEP, seed=seed)


def peak_of(seed):
    """One flight, one block maximum. Top level so it can be sent to a worker."""
    res = flight(Kite(), SILK, turbulent_field(seed),
                 trim=wind.shear(WIND_BASE))
    return float(res["tension"].max())


def half_peaks_of(seed):
    """The same flight split in two, for the block-size check."""
    res = flight(Kite(), SILK, turbulent_field(seed),
                 trim=wind.shear(WIND_BASE))
    t, T = res["t"], res["tension"]
    mid = t[-1] / 2
    return float(T[t <= mid].max()), float(T[t > mid].max())


def param_peak(args):
    """One flight with drawn parameters."""
    row, seed = args
    kite, tether, base = build(row)
    field = turbulent_field(seed, base=base)
    try:
        # Trim on the mean field, not on whatever the turbulence holds at
        # t = 0. NaN here means no equilibrium exists at all.
        state, _, elev = equilibrium(kite, tether, wind.shear(base), LINE_LENGTH)
        if not np.isfinite(elev) or elev < 5.0:
            return np.nan        # not flying; a load from here means nothing
        res = simulate(kite, tether, field, lambda t: LINE_LENGTH, state,
                       DURATION, n_out=int(DURATION * 4))
        return float(res["tension"].max())
    except Exception:            # noqa: BLE001
        return np.nan


def run_pool(fn, items, workers, label=""):
    """Map over workers, reporting progress as results land.

    A phase here runs for tens of minutes. Without a running count there is no
    way to tell a slow run from a wedged one, and the first version of this
    printed nothing between the section header and the finished table.
    """
    items = list(items)
    total = len(items)
    out = []
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for i, value in enumerate(pool.map(fn, items, chunksize=1), start=1):
            out.append(value)
            if i == 1 or i % max(1, total // 20) == 0 or i == total:
                rate = (time.perf_counter() - started) / i
                left = rate * (total - i)
                print(f"    {label}{i}/{total}  "
                      f"({rate:.1f}초/건, 남은 시간 약 {left/60:.0f}분)", flush=True)
    return out


def gumbel_return_level(peaks, period):
    """Load exceeded once per `period` flights, under a Gumbel fit."""
    loc, scale = stats.gumbel_r.fit(peaks)
    return float(stats.gumbel_r.ppf(1 - 1 / period, loc, scale))


def bootstrap_return(peaks, period, trials=2000, seed=0):
    rng = np.random.default_rng(seed)
    peaks = np.asarray(peaks)
    out = np.empty(trials)
    for i in range(trials):
        sample = rng.choice(peaks, size=len(peaks), replace=True)
        out[i] = gumbel_return_level(sample, period)
    return np.percentile(out, [2.5, 97.5])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--flights", type=int)
    args = ap.parse_args()

    n_flights = args.flights or (60 if args.quick else FLIGHTS)
    n_draws = 4 if args.quick else PARAM_DRAWS
    n_param_flights = 6 if args.quick else PARAM_FLIGHTS
    workers = max(1, (os.cpu_count() or 4) - 1)

    print("=" * 78)
    print(f"1. 블록 최대값 {n_flights}회 ({DURATION/60:.0f}분 비행, "
          f"난류강도 {INTENSITY:.0%}, 적분규모 {LENGTH_SCALE:.0f} m)")
    print("=" * 78)
    print(f"  워커 {workers}개로 실행 중 ...", flush=True)
    # 40분 걸려 얻은 150개 값을 메모리에만 두면, 인쇄된 숫자 하나를 다시 읽으려고
    # 40분을 또 써야 한다. 실제로 그렇게 됐으므로 디스크에 남긴다.
    cache = Path("out") / "block_maxima.npy"
    if cache.exists() and not FORCE:
        peaks = np.load(cache)
        print(f"  (캐시에서 블록 최대값 {len(peaks)}개를 읽었습니다)")
    else:
        peaks = np.array(run_pool(peak_of, range(n_flights), workers, "비행 "))
        peaks = peaks[np.isfinite(peaks)]
        np.save(cache, peaks)

    q = np.percentile(peaks, [5, 50, 95])
    print(f"  비행별 최대장력: 최소 {peaks.min():.1f}  5% {q[0]:.1f}  "
          f"중앙 {q[1]:.1f}  95% {q[2]:.1f}  최대 {peaks.max():.1f} N")
    print(f"  표준편차 {peaks.std(ddof=1):.2f} N  "
          f"(6회만 봤을 때의 범위 24.4~47.4 N 와 비교)")
    print()

    # ── 꼬리 모양: Gumbel 로 충분한가 ────────────────────────────────
    print("=" * 78)
    print("2. 꼬리 모양 — Gumbel 로 충분한가")
    print("=" * 78)
    shape, loc, scale = stats.genextreme.fit(peaks)
    # scipy의 genextreme은 c = -xi 부호 규약을 쓴다
    xi = -shape
    gev_ll = np.sum(stats.genextreme.logpdf(peaks, shape, loc, scale))
    g_loc, g_scale = stats.gumbel_r.fit(peaks)
    gum_ll = np.sum(stats.gumbel_r.logpdf(peaks, g_loc, g_scale))
    lr = 2 * (gev_ll - gum_ll)
    p_lr = stats.chi2.sf(max(lr, 0.0), df=1)

    print(f"  GEV 형상 ξ = {xi:+.3f}   (0 이면 Gumbel, >0 이면 더 무거운 꼬리)")
    print(f"  우도비 검정 vs Gumbel: LR={lr:.2f}, p={p_lr:.3f}")
    if p_lr > 0.05:
        print("  -> ξ 가 0 과 구별되지 않습니다. Gumbel 을 씁니다.")
    else:
        print("  -> ξ 가 0 과 구별됩니다. GEV 를 써야 하고, 아래 값들은")
        print("     Gumbel 기준이므로 보수적이지 않을 수 있습니다.")
    ks = stats.kstest(peaks, "gumbel_r", args=(g_loc, g_scale))
    print(f"  KS 적합도: D={ks.statistic:.3f}, p={ks.pvalue:.3f}"
          f"  ({'적합' if ks.pvalue > 0.05 else '부적합'})")
    print()

    # ── 블록 길이가 충분한가 ────────────────────────────────────────
    print("=" * 78)
    print("3. 블록 길이가 충분한가 (300초 vs 150초)")
    print("=" * 78)
    n_half = min(len(peaks), 60 if not args.quick else 30)
    halves = run_pool(half_peaks_of, range(n_half), workers, "반블록 ")
    half_peaks = np.array([v for pair in halves for v in pair])
    h_loc, h_scale = stats.gumbel_r.fit(half_peaks)
    print(f"  300초 블록: 위치 {g_loc:.2f}  척도 {g_scale:.2f} N")
    print(f"  150초 블록: 위치 {h_loc:.2f}  척도 {h_scale:.2f} N")
    print(f"  척도 변화 {100*(h_scale-g_scale)/g_scale:+.0f}%")
    print("  척도가 블록 길이에 크게 의존하면 점근 영역에 아직 못 들어온 것이고,")
    print("  그 경우 외삽한 재현수준은 블록 길이의 함수가 됩니다.")
    print()

    # ── 재현수준 ────────────────────────────────────────────────────
    print("=" * 78)
    print("4. 재현수준 — N회 비행에 한 번 넘는 하중")
    print("=" * 78)
    print(f"  {'재현주기':<16}{'추정':>10}{'95% 구간':>22}")
    levels = {}
    for period in RETURN_PERIODS:
        rl = gumbel_return_level(peaks, period)
        lo, hi = bootstrap_return(peaks, period, trials=600 if args.quick else 2000)
        levels[period] = (rl, lo, hi)
        print(f"  {period:>6}회 비행마다{rl:>10.1f} N   [{lo:>6.1f}, {hi:>6.1f}] N")
    print()
    print(f"  참고: 결정론적 단일 돌풍 {DETERMINISTIC_GUST:.0f} N, "
          f"현재 권고 설계값 {CURRENT_DESIGN:.0f} N")
    print(f"  관측 최대 {peaks.max():.1f} N 는 약 "
          f"{1/stats.gumbel_r.sf(peaks.max(), g_loc, g_scale):.0f}회 비행에 "
          f"한 번 꼴입니다.")
    print()

    # ── 몇 회를 돌려야 수렴하는가 ────────────────────────────────────
    print("=" * 78)
    print("5. 몇 회를 돌려야 하는가 (100회 재현수준의 수렴)")
    print("=" * 78)
    rng = np.random.default_rng(1)
    sizes = [n for n in (6, 12, 25, 50, 100, 200, 400) if n <= len(peaks)]
    conv = {}
    for n in sizes:
        est = [gumbel_return_level(rng.choice(peaks, n, replace=False), 100)
               for _ in range(200)]
        conv[n] = (float(np.mean(est)), float(np.std(est, ddof=1)))
        print(f"  {n:>4}회 사용 -> {conv[n][0]:>6.1f} ± {conv[n][1]:.1f} N")
    print()
    print(f"  6회(loads.py 가 쓴 수)에서의 흩어짐 {conv[sizes[0]][1]:.1f} N 는")
    print("  설계 여유를 정하는 데 쓰기에는 너무 큽니다.")
    print()

    # ── 파라미터 불확실성과 비교 ────────────────────────────────────
    print("=" * 78)
    print(f"6. 파라미터 불확실성 — 난류 산포보다 큰가 ({n_draws}개 표본)")
    print("=" * 78)
    prng = np.random.default_rng(7)
    rows = draw(prng, n_draws)
    jobs = [(rows[i], 10_000 + i * 100 + j)
            for i in range(n_draws) for j in range(n_param_flights)]
    param_peaks = np.array(run_pool(param_peak, jobs, workers, "표본 "))
    param_peaks = param_peaks.reshape(n_draws, n_param_flights)

    per_draw = []
    for i in range(n_draws):
        vals = param_peaks[i][np.isfinite(param_peaks[i])]
        if len(vals) >= 3:
            per_draw.append(float(np.mean(vals)))
    per_draw = np.array(per_draw)

    if len(per_draw) >= 3:
        print(f"  파라미터 표본별 평균 최대장력: "
              f"{per_draw.min():.1f} ~ {per_draw.max():.1f} N "
              f"(표준편차 {per_draw.std(ddof=1):.1f} N)")
        print(f"  고정 파라미터에서 난류만의 흩어짐: "
              f"표준편차 {peaks.std(ddof=1):.1f} N")
        ratio = per_draw.std(ddof=1) / peaks.std(ddof=1)
        print(f"  비율 {ratio:.1f}배")
        print()
        if ratio > 1.0:
            print("  -> 파라미터 불확실성이 난류 산포보다 큽니다. 재현수준을")
            print("     소수점까지 적는 것은 의미가 없고, 설계값은 파라미터가")
            print("     좁혀진 뒤에 다시 내야 합니다. 지금 할 수 있는 것은")
            print("     '현재 파라미터 추정 하에서' 라는 단서가 붙은 값뿐입니다.")
        else:
            print("  -> 난류 산포가 더 큽니다. 재현수준이 설계값을 지배합니다.")
    else:
        print("  표본이 부족해 비교할 수 없습니다.")
    print()

    # ── 권고 ────────────────────────────────────────────────────────
    rl100, lo100, hi100 = levels[100]
    print("=" * 78)
    print("7. 권고")
    print("=" * 78)
    print(f"  100회 비행 재현수준의 95% 상한: {hi100:.1f} N")
    print(f"  현재 권고값 {CURRENT_DESIGN:.0f} N 은 "
          f"{'충분합니다' if CURRENT_DESIGN >= hi100 else '부족합니다'}.")
    print()
    print("  다만 이 값은 전부 '현재 CL·CD 추정이 맞다면' 이라는 조건부입니다.")
    print("  그리고 구조상 먼저 끊어지는 곳 — 대나무 이음부와 매듭 — 의 강도는")
    print("  아직 전혀 측정되지 않았습니다. 설계하중을 아무리 정밀하게 내도")
    print("  비교할 상대가 없으면 쓸 수 없습니다.")
    print()

    # ── 그림 ────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))

    ax = axes[0]
    ax.hist(peaks, bins=30, density=True, color="#9ecae1",
            edgecolor="#4292c6", label=f"비행별 최대값 ({len(peaks)}회)")
    xs = np.linspace(peaks.min() * 0.9, max(peaks.max(), rl100) * 1.05, 300)
    ax.plot(xs, stats.gumbel_r.pdf(xs, g_loc, g_scale), "k", lw=2,
            label=f"Gumbel 적합 (μ={g_loc:.1f}, β={g_scale:.1f})")
    for period, color in zip(RETURN_PERIODS, ("#f0a202", "#d94801", "#a63603")):
        ax.axvline(levels[period][0], color=color, ls="--", lw=1.4,
                   label=f"{period}회 재현 {levels[period][0]:.0f} N")
    ax.axvline(CURRENT_DESIGN, color="#2171b5", lw=2,
               label=f"현재 설계값 {CURRENT_DESIGN:.0f} N")
    ax.set_xlabel("비행 최대 장력 [N]")
    ax.set_ylabel("밀도")
    ax.set_title("블록 최대값 분포")
    ax.legend(fontsize=7.5)

    ax = axes[1]
    order = np.sort(peaks)
    emp = (np.arange(1, len(order) + 1) - 0.44) / (len(order) + 0.12)
    ax.plot(-np.log(-np.log(emp)), order, "o", ms=3.5, color="#4292c6",
            label="관측")
    gl = np.linspace(-2, 8, 100)
    ax.plot(gl, g_loc + g_scale * gl, "k", lw=1.6, label="Gumbel")
    for period in RETURN_PERIODS:
        ax.axhline(levels[period][0], color="#d94801", ls=":", lw=1)
        ax.text(-1.8, levels[period][0], f"{period}회", fontsize=7.5,
                va="bottom", color="#d94801")
    ax.set_xlabel("Gumbel 축  −ln(−ln F)")
    ax.set_ylabel("장력 [N]")
    ax.set_title("적합 진단 (직선이면 Gumbel)")
    ax.legend(fontsize=8)

    ax = axes[2]
    ns = list(conv)
    means = [conv[n][0] for n in ns]
    sds = [conv[n][1] for n in ns]
    ax.errorbar(ns, means, yerr=sds, fmt="o-", color="#4292c6", capsize=3)
    ax.axhline(rl100, color="k", ls="--", lw=1.2,
               label=f"전체 {len(peaks)}회 기준 {rl100:.0f} N")
    ax.set_xscale("log")
    ax.set_xlabel("사용한 비행 수")
    ax.set_ylabel("100회 재현수준 [N]")
    ax.set_title("몇 회를 돌려야 하는가")
    ax.legend(fontsize=8)

    for a in axes:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path("extremes.png"), dpi=260)
    print("  saved out/extremes.png")


if __name__ == "__main__":
    main()
