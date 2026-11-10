"""Digital twin core: turn a flight log into calibrated parameters.

CL and CD are still guesses to within 40%, and the uncertainty study showed
they dominate almost every prediction. Nothing downstream improves until a
flight pins them down, so this is the first thing the twin has to do.

Method: pick the quasi-static stretches of a log -- the kite is always moving,
but much of the time it is moving slowly enough that the force balance holds --
then invert that balance for CL and CD at each sample and aggregate. Fitting
the full dynamics instead would use every sample but needs accelerations, which
a ground station does not measure.

Two corrections the naive algebra misses and this does not:
  * the tether's own weight and drag act on the kite, and at 50 m they are not
    small the way they were in the 3 m indoor rig;
  * a ground angle sensor reads the cable tangent, while the balance is written
    about the chord, so sag has to be removed first.

Validated against synthetic logs where the true parameters are known.
"""

from pathlib import Path
import numpy as np

import wind as wind_module
from model import equilibrium, line_force, sag_angle
from params import CD_CYLINDER, G, RHO_AIR, SILK, Kite

# One row per ground-station sample. The ground node must log its own channels
# alongside what it receives: without tension, length and angle there is
# nothing to invert.
LOG_COLUMNS = (
    "t_s",             # ground clock
    "line_length_m",   # encoder, unstretched length paid out
    "tension_n",       # BTS7960 current sense
    "elevation_deg",   # ground angle sensor: cable tangent, not chord
    "wind_ms",         # anemometer, at its own mounting height
    "payload_tilt_deg",
    "pv_pmax_w",
)

# Random, independent per sample. These average away over a long log.
ERR_TENSION = 0.08
ERR_ELEVATION = np.radians(2.0)
ERR_WIND = 0.05
ERR_LENGTH = 0.02

# Systematic, drawn once per campaign and shared by every sample in it: a
# calibration gain, a sensor mounting offset, a measured area. These do not
# average away at all, and past a short log they are the whole error budget.
BIAS_TENSION = 0.05        # current-sense gain
BIAS_WIND = 0.04           # anemometer calibration
BIAS_ELEVATION = np.radians(1.5)   # angle sensor mounting
BIAS_AREA = 0.03
BIAS_MASS = 0.01

# A line below this is not holding the kite up in any useful sense.
MIN_TENSION = 0.25           # N
# model.sag_angle's own limit on the small-sag form.
MAX_SAG = np.radians(20.0)

STEADY_TENSION_RATE = 0.8    # N/s, above this the sample is not quasi-static
STEADY_ELEVATION_RATE = 1.5  # deg/s



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def synthesize(kite, tether, speeds, seconds_each=30.0, rate=20.0,
               seed=0, sag=True, systematic=None, bias_scale=None):
    """A synthetic ground-station log with known truth behind it.

    Lets the whole identification pipeline be validated before any hardware
    flies, which is the only way to tell a working estimator from one that
    happens to return plausible numbers.

    With `systematic`, one set of calibration biases is drawn for the whole log,
    which is how real instruments behave and what the bootstrap cannot see.
    """
    rng = np.random.default_rng(seed)
    rows = []
    t = 0.0
    step = 1.0 / rate

    active = set(systematic) if systematic else set()
    scale = bias_scale or {}
    gain_t = (1 + rng.normal(0, BIAS_TENSION * scale.get("tension", 1.0))
              if "tension" in active else 1.0)
    gain_w = (1 + rng.normal(0, BIAS_WIND * scale.get("wind", 1.0))
              if "wind" in active else 1.0)
    offset_e = (rng.normal(0, BIAS_ELEVATION * scale.get("elevation", 1.0))
                if "elevation" in active else 0.0)

    for speed in speeds:
        field = wind_module.uniform(speed)
        state, tension, elev_deg = equilibrium(kite, tether, field, 50.0)
        if not np.isfinite(elev_deg) or elev_deg <= 5.0:
            continue
        elev = np.radians(elev_deg)
        length = 50.0

        measured_elev = elev - (sag_angle(tension, length, elev, tether)
                                if sag else 0.0)
        n = int(seconds_each * rate)
        for _ in range(n):
            rows.append((
                t,
                length * (1 + rng.normal(0, ERR_LENGTH)),
                tension * gain_t * (1 + rng.normal(0, ERR_TENSION)),
                np.degrees(measured_elev + offset_e
                           + rng.normal(0, ERR_ELEVATION)),
                speed * gain_w * (1 + rng.normal(0, ERR_WIND)),
                24.0 + rng.normal(0, 3.0),
                0.0,
            ))
            t += step

    return np.array(rows)


def _slope(values, times, smooth_n, lag):
    """Slope of a noisy signal, measured after smoothing and over a long base.

    Differentiating the raw log measures the sensor noise, not the motion: at
    20 Hz an 8% tension error turns into several N/s of apparent rate, which
    would reject every sample as 'moving'. Smoothing first and then taking the
    difference across a two-second baseline leaves the noise far below the
    rates that actually matter.
    """
    kernel = np.ones(smooth_n) / smooth_n
    smoothed = np.convolve(values, kernel, mode="same")

    out = np.full(len(values), np.inf)
    span = times[2 * lag:] - times[:-2 * lag]
    out[lag:-lag] = np.abs((smoothed[2 * lag:] - smoothed[:-2 * lag]) / span)
    return out


def quasi_static(log, rate=20.0):
    """Rows where the force balance the inversion assumes actually holds.

    Steadiness is necessary but not sufficient. A slack line is perfectly
    steady -- it just is not carrying the kite, and every quantity read off
    the balance there is meaningless rather than noisy. Worse, the sag
    correction goes as 1/T, so those rows do not merely add scatter, they
    pull the estimate: at 0.05 N the inversion returns CD = -0.05, a drag
    coefficient no object can have.

    Synthetic logs never showed this because they are generated at steady
    flying conditions. A real log with a lull in it would have.
    """
    times = log[:, 0]
    smooth_n = max(int(2 * rate), 1)
    lag = max(int(rate), 1)
    if len(log) < 4 * lag + smooth_n:
        return np.zeros(len(log), dtype=bool)

    d_tension = _slope(log[:, 2], times, smooth_n, lag)
    d_elevation = _slope(log[:, 3], times, smooth_n, lag)
    steady = (d_tension < STEADY_TENSION_RATE) & (d_elevation < STEADY_ELEVATION_RATE)

    # The line has to be carrying the kite, and the small-sag form has to be
    # inside its stated range (model.sag_angle: past ~20 deg it is out of
    # range, and 1/T takes it there as soon as the line slackens).
    tension = log[:, 2]
    carrying = tension > MIN_TENSION
    with np.errstate(divide="ignore", invalid="ignore"):
        sag = np.where(tension > 0,
                       0.5 * SILK.mass_per_m * log[:, 1] * G
                       * np.cos(np.radians(log[:, 3])) / np.maximum(tension, 1e-12),
                       np.inf)
    return steady & carrying & (sag <= MAX_SAG)


def invert_row(length, tension, measured_elev_deg, wind_speed, kite, tether):
    """Recover (CL, CD) from one quasi-static sample."""
    measured = np.radians(measured_elev_deg)

    # Undo sag to get the chord angle the force balance is written about.
    chord = measured + sag_angle(tension, length, measured, tether)
    rhat = np.array([np.cos(chord), np.sin(chord)])
    r = length * rhat

    wind_vec = np.array([wind_speed, 0.0])
    aero = (tension * rhat
            - line_force(np.zeros(2), wind_vec, r, tether, length)
            + np.array([0.0, kite.mass * G]))

    q = 0.5 * RHO_AIR * wind_speed**2 * kite.area
    if q <= 0:
        return np.nan, np.nan
    return aero[1] / q, aero[0] / q


def invert_rows(length, tension, measured_elev_deg, wind_speed, kite, tether):
    """Vectorised `invert_row`, same algebra over whole arrays.

    `identify` ran this per sample through two-element numpy vectors, which
    costs far more in interpreter overhead than in arithmetic. The scalar
    version stays as the readable reference and test_twin checks the two agree.
    """
    measured = np.radians(np.asarray(measured_elev_deg, dtype=float))
    length = np.asarray(length, dtype=float)
    tension = np.asarray(tension, dtype=float)
    speed = np.asarray(wind_speed, dtype=float)

    weight = tether.mass_per_m * length * G
    chord = measured + 0.5 * weight * np.cos(measured) / tension
    cos_c, sin_c = np.cos(chord), np.sin(chord)

    # Line drag: only the wind component normal to the cable counts, and half
    # the distributed load is carried at the kite.
    drag = (0.25 * RHO_AIR * (speed * sin_c) ** 2
            * tether.diameter * length * CD_CYLINDER)

    aero_x = tension * cos_c - drag * sin_c
    aero_z = tension * sin_c + drag * cos_c + 0.5 * weight + kite.mass * G

    q = 0.5 * RHO_AIR * speed**2 * kite.area
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(q > 0, aero_z / q, np.nan), np.where(q > 0, aero_x / q,
                                                             np.nan)


def identify(log, kite, tether, bootstrap=2000, seed=0):
    """Fit CL and CD from a log, with a bootstrap confidence interval."""
    usable = quasi_static(log)
    if usable.sum() < 50:
        raise ValueError("not enough quasi-static samples to identify anything")

    rows = log[usable]
    cls, cds = invert_rows(rows[:, 1], rows[:, 2], rows[:, 3], rows[:, 4],
                           kite, tether)
    good = np.isfinite(cls) & np.isfinite(cds)
    cls, cds = cls[good], cds[good]
    rng = np.random.default_rng(seed)
    n = len(cls)
    draws = rng.integers(0, n, size=(bootstrap, n))
    cl_boot = np.median(cls[draws], axis=1)
    cd_boot = np.median(cds[draws], axis=1)

    return {
        "cl": float(np.median(cls)),
        "cd": float(np.median(cds)),
        "cl_ci": tuple(np.percentile(cl_boot, [2.5, 97.5])),
        "cd_ci": tuple(np.percentile(cd_boot, [2.5, 97.5])),
        "samples": n,
        "usable_fraction": float(usable.mean()),
    }


def campaign_spread(truth, speeds, seconds, biases, rng, trials=25,
                    bias_scale=None):
    """95% spread of fitted CL across repeated campaigns.

    Each campaign draws its own calibration biases, so this captures what the
    bootstrap cannot: the part of the error that every sample in a log shares.
    """
    from dataclasses import replace

    fits = []
    for trial in range(trials):
        assumed = truth
        if "area" in biases:
            sigma = BIAS_AREA * (bias_scale or {}).get("area", 1.0)
            assumed = replace(truth,
                              area=truth.area * (1 + rng.normal(0, sigma)))
        log = synthesize(truth, SILK, speeds, seconds_each=seconds,
                         seed=int(rng.integers(0, 10**6)), systematic=biases,
                         bias_scale=bias_scale)
        fits.append(identify(log, assumed, SILK, bootstrap=1)["cl"])
    return (np.percentile(fits, 97.5) - np.percentile(fits, 2.5)) / truth.cl


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    truth = Kite()
    print("=" * 78)
    print("1. 합성 로그로 식별기 검증 (참값을 아는 상태)")
    print("=" * 78)
    print(f"  참값 CL={truth.cl:.3f}, CD={truth.cd:.3f}")
    print()

    log = synthesize(truth, SILK, [5.0, 6.0, 7.0, 8.0], seed=1)
    fit = identify(log, truth, SILK)
    print(f"  표본 {fit['samples']} (준정적 비율 {fit['usable_fraction']:.0%})")
    print(f"  CL {fit['cl']:.3f}  95% CI [{fit['cl_ci'][0]:.3f}, "
          f"{fit['cl_ci'][1]:.3f}]  오차 {100*(fit['cl']/truth.cl-1):+.1f}%")
    print(f"  CD {fit['cd']:.3f}  95% CI [{fit['cd_ci'][0]:.3f}, "
          f"{fit['cd_ci'][1]:.3f}]  오차 {100*(fit['cd']/truth.cd-1):+.1f}%")
    print()

    print("=" * 78)
    print("2. 보정을 빼면 얼마나 틀어지는가")
    print("=" * 78)

    def naive_identify(log, kite):
        """The algebra without line forces or sag removal."""
        usable = quasi_static(log)
        rows = log[usable]
        cls, cds = [], []
        for _, length, tension, elev_deg, speed, _, _ in rows:
            elev = np.radians(elev_deg)
            q = 0.5 * RHO_AIR * speed**2 * kite.area
            cds.append(tension * np.cos(elev) / q)
            cls.append((tension * np.sin(elev) + kite.mass * G) / q)
        return float(np.median(cls)), float(np.median(cds))

    n_cl, n_cd = naive_identify(log, truth)
    print(f"  {'방식':<26}{'CL':>10}{'오차':>10}{'CD':>10}{'오차':>10}")
    print(f"  {'보정 없음':<26}{n_cl:>10.3f}{100*(n_cl/truth.cl-1):>9.1f}%"
          f"{n_cd:>10.3f}{100*(n_cd/truth.cd-1):>9.1f}%")
    print(f"  {'처짐+줄하중 보정':<26}{fit['cl']:>10.3f}"
          f"{100*(fit['cl']/truth.cl-1):>9.1f}%{fit['cd']:>10.3f}"
          f"{100*(fit['cd']/truth.cd-1):>9.1f}%")
    print()

    print("=" * 78)
    print("3. 실험 설계: 표본오차 vs 계통오차")
    print("=" * 78)
    print("  부트스트랩은 표본오차만 봅니다. 캠페인마다 한 번 뽑히는 계통오차")
    print("  (풍속계 게인, 각도 오프셋, 장력 게인, 면적·질량)는 안 보입니다.")
    print("  그래서 캠페인 자체를 여러 번 돌려서 실제 흩어짐을 봅니다.")
    print()
    print(f"  {'로그 조건':<26}{'부트스트랩 폭':>16}{'실제 흩어짐':>16}")
    designs = [
        ("단일 6 m/s, 30 s", [6.0], 30.0),
        ("단일 6 m/s, 300 s", [6.0], 300.0),
        ("4개 풍속, 각 30 s", [5.0, 6.0, 7.0, 8.0], 30.0),
        ("4개 풍속, 각 120 s", [5.0, 6.0, 7.0, 8.0], 120.0),
    ]
    rng = np.random.default_rng(99)
    for label, speeds, seconds in designs:
        sample = synthesize(truth, SILK, speeds, seconds_each=seconds, seed=2)
        f = identify(sample, truth, SILK, bootstrap=500)
        boot_w = (f["cl_ci"][1] - f["cl_ci"][0]) / truth.cl

        spread = campaign_spread(truth, speeds, seconds,
                                 ("tension", "wind", "elevation", "area"), rng)
        print(f"  {label:<26}{100*boot_w:>15.1f}%{100*spread:>15.1f}%")
    print()
    print(f"  => 로그를 아무리 늘려도 계통오차 아래로는 안 내려갑니다.")
    print(f"     부트스트랩만 보고 '0.6% 정확도'라고 하면 틀립니다.")
    print(f"     (오른쪽 열은 캠페인 25회 몬테카를로라 수 %p는 그 자체의 잡음입니다.")
    print(f"      읽을 것은 값의 차이가 아니라 '추세가 없다'는 사실입니다.)")
    print()

    print("=" * 78)
    print("4. 어느 교정을 먼저 잡아야 하는가 (계통오차 기여도)")
    print("=" * 78)
    speeds, seconds = [5.0, 6.0, 7.0, 8.0], 60.0
    ranked = []
    for name, label in (("wind", "풍속계 게인 4%"),
                        ("tension", "장력 게인 5%"),
                        ("elevation", "각도 오프셋 1.5°"),
                        ("area", "면적 3%")):
        s = campaign_spread(truth, speeds, seconds, (name,), rng)
        ranked.append((label, s))
    total = campaign_spread(truth, speeds, seconds,
                            ("tension", "wind", "elevation", "area"), rng)

    print(f"  {'오차원':<24}{'CL 95% 폭':>14}")
    for label, s in sorted(ranked, key=lambda x: -x[1]):
        print(f"  {label:<24}{100*s:>13.1f}%")
    print(f"  {'(전부 합쳐서)':<24}{100*total:>13.1f}%")
    print()
    worst = max(ranked, key=lambda x: x[1])
    print(f"  => {worst[0]}이 단독으로 {100*worst[1]:.0f}%를 만듭니다.")
    print(f"     CL은 동압(풍속 제곱)으로 나누는 값이라 풍속 오차가 2배로 들어갑니다.")
    print()

    print("=" * 78)
    print("5. 계통오차를 어떻게 없앨 것인가")
    print("=" * 78)
    print("  비행 데이터만으로 센서 교정까지 같이 추정해봤는데, 두 방법 다")
    print("  실패했습니다. 기록해둡니다:")
    print("   * 두 힘 평형식을 최소제곱으로 공동 적합 -> 풍속이 잡음 있는")
    print("     회귀변수라 기울기가 축소됩니다(errors-in-variables). 계통오차가")
    print("     전혀 없는 로그에서도 CL이 20% 낮게 나왔습니다.")
    print("   * '추정 CL이 풍속에 안 흔들리게 하는 게인' 탐색 -> mg가 주는")
    print("     신호는 실재하지만 장력 8%·풍속 5% 잡음에 묻혀, 무편향 로그에서")
    print("     게인 0.86을 집어냈습니다.")
    print("  => 지금 잡음 수준에서는 비행 데이터로 센서를 교정할 수 없습니다.")
    print()
    print("  대신 벤치에서 직접 교정하면 간단히 끝납니다:")
    print("   * 장력 게인  : 아는 무게를 매달고 전류센싱 읽기 (EA 측정과 같은 추)")
    print("   * 각도 오프셋: 수평계/다림추로 기준각 맞추기")
    print("   * 면적      : 자로 재기")
    bench = campaign_spread(truth, speeds, seconds,
                            ("tension", "elevation", "wind", "area"), rng,
                            trials=15,
                            bias_scale={"tension": 0.2, "elevation": 0.2,
                                        "area": 0.3})
    print()
    print(f"  {'교정 전 (전부)':<22}{100*total:>8.1f}%")
    print(f"  {'벤치 교정 후':<22}{100*bench:>8.1f}%")
    print()
    print("  => 풍속 게인은 원리적으로 보정 불가능합니다. 측정량으로 쓰면")
    print("     CL과 g_w^2이 항상 CL/g_w^2 형태로만 나타나서, 데이터를 아무리")
    print("     모아도 분리되지 않습니다.")
    print("     다만 실용적으로는 대부분 문제가 안 됩니다 — 같은 풍속계로 재서")
    print("     같은 CL로 예측하면 g_w가 정확히 상쇄됩니다.")
    print("     문제가 되는 건 'CL 값 자체를 보고할 때'뿐입니다.")
    print()

    print("=" * 78)
    print("지상국 로그에 반드시 들어가야 하는 채널")
    print("=" * 78)
    for name in LOG_COLUMNS:
        print(f"  - {name}")
    print()
    print("  현재 펌웨어는 탑재물에서 받은 것만 기록하고 지상국 자신의")
    print("  장력·길이·각도·풍속을 안 남깁니다. 그대로 두면 식별이 불가능합니다.")
    print()

    usable = quasi_static(log)
    rows = log[usable]
    pairs = np.array([invert_row(r[1], r[2], r[3], r[4], truth, SILK)
                      for r in rows])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, col, name, true_val in ((axes[0], 0, "CL", truth.cl),
                                    (axes[1], 1, "CD", truth.cd)):
        ax.hist(pairs[:, col], bins=60, color="#4C72B0", alpha=0.85)
        ax.axvline(true_val, color="k", ls="--", lw=1.5, label="참값")
        ax.axvline(np.median(pairs[:, col]), color="r", lw=1.5, label="추정")
        ax.set_xlabel(name)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("표본 수")
    fig.suptitle("합성 비행 로그에서 식별한 공력계수")
    fig.tight_layout()
    fig.savefig(out_path("twin.png"), dpi=260)
    print("  saved out/twin.png")
