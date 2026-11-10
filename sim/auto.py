"""Self-updating calibration, and knowing when to stop.

Each flight log yields a CL estimate. Pooling them reduces the random part,
but not the systematic part: the anemometer's gain is the same gain on every
flight, so averaging ten logs reproduces the same bias ten times. The pooled
estimate therefore converges to CL_true plus a fixed offset, and once the
random part falls below that offset there is nothing left for more flights to
buy.

So the loop here is not "keep improving". It is:

  * gate   -- refuse logs that cannot support an estimate at all
  * pool   -- combine accepted logs, tracking only the random error honestly
  * stop   -- declare the floor once movement drops below it, and say what
              would actually lower it (bench calibration, not more flying)
  * watch  -- after convergence, a shifted estimate means the kite changed,
              not that the estimate improved

The last two are the same statistic read in opposite directions, which is why
they belong in one place.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from dataclasses import replace

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

from params import SILK, Kite
from twin import identify, quasi_static, synthesize

MIN_STEADY_FRACTION = 0.25
MIN_SAMPLES = 200
PLAUSIBLE_CL = (0.2, 1.6)
PLAUSIBLE_CD = (0.05, 1.0)

# 95% campaign-to-campaign spread from twin.py, as a 1-sigma fraction.
SYSTEMATIC_FLOOR = 0.232 / 4.0
CONVERGE_AFTER = 3        # consecutive quiet updates before declaring the floor

# Change detection. The first version armed only after convergence and fired on
# a single 2.5-sigma window; validation gave a 40% false-alarm rate, and a large
# change suppressed its own detection by preventing convergence in the first
# place. Both are fixed here: the baseline is taken from a fixed number of early
# flights regardless of convergence, and a fire needs two consecutive windows.
BASELINE_FLIGHTS = 8
CHANGE_WINDOW = 3
CHANGE_SIGMA = 3.0
CHANGE_PERSISTENCE = 2



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def gate(log, kite, tether):
    """Reject logs that cannot support an estimate, with a reason."""
    steady = quasi_static(log)
    if steady.sum() < MIN_SAMPLES:
        return None, f"준정적 표본 {int(steady.sum())}개 (최소 {MIN_SAMPLES})"
    if steady.mean() < MIN_STEADY_FRACTION:
        return None, f"준정적 비율 {steady.mean():.0%} (최소 {MIN_STEADY_FRACTION:.0%})"

    try:
        fit = identify(log, kite, tether, bootstrap=400)
    except ValueError as exc:
        return None, str(exc)

    if not PLAUSIBLE_CL[0] <= fit["cl"] <= PLAUSIBLE_CL[1]:
        return None, f"CL {fit['cl']:.2f}가 물리적 범위 밖"
    if not PLAUSIBLE_CD[0] <= fit["cd"] <= PLAUSIBLE_CD[1]:
        return None, f"CD {fit['cd']:.2f}가 물리적 범위 밖"
    return fit, "accepted"


class Calibration:
    """Running estimate with a floor detector and a change detector."""

    def __init__(self, floor=SYSTEMATIC_FLOOR):
        self.floor = floor
        self.values = []
        self.history = []
        self.quiet = 0
        self.converged = False
        self.baseline = float("nan")
        self.baseline_scatter = 0.0
        self.exceeded = 0
        self.flagged = False

    @property
    def estimate(self):
        return float(np.mean(self.values)) if self.values else float("nan")

    @property
    def random_sigma(self):
        """Standard error of the pooled mean -- the part flying more reduces."""
        if len(self.values) < 2:
            return float("inf")
        return float(np.std(self.values, ddof=1) / np.sqrt(len(self.values)))

    def add(self, value):
        previous = self.estimate
        self.values.append(value)
        now = self.estimate

        note = "updated"
        if len(self.values) == BASELINE_FLIGHTS:
            self.baseline = now
            # Floored at the known systematic spread. An unusually quiet set of
            # baseline flights would otherwise set a scatter near zero, and the
            # detector would either never arm or fire on everything after.
            measured = float(np.std(self.values, ddof=1))
            self.baseline_scatter = max(measured, self.floor * abs(now))
        elif len(self.values) > BASELINE_FLIGHTS and self.baseline_scatter > 0:
            recent = self.values[-CHANGE_WINDOW:]
            err = self.baseline_scatter / np.sqrt(CHANGE_WINDOW)
            if abs(np.mean(recent) - self.baseline) > CHANGE_SIGMA * err:
                self.exceeded += 1
                if self.exceeded >= CHANGE_PERSISTENCE and not self.flagged:
                    self.flagged = True
                    note = "CHANGE: 지속적 이동 감지"
            else:
                self.exceeded = 0

        if note == "updated" and len(self.values) >= 2:
            moved = abs(now - previous) / abs(now)
            if moved < 0.25 * self.floor:
                self.quiet += 1
                if self.quiet >= CONVERGE_AFTER and not self.converged:
                    self.converged = True
                    note = "CONVERGED: 계통오차 바닥 도달"
            else:
                self.quiet = 0

        self.history.append((now, self.random_sigma, note))
        return note


def flight_log(kite, tether, seed, speeds=(5.0, 6.0, 7.0, 8.0), seconds=40.0):
    return synthesize(kite, tether, list(speeds), seconds_each=seconds,
                      seed=seed, systematic=("tension", "elevation", "wind",
                                             "area"))


if __name__ == "__main__":
    truth = Kite()
    assumed = truth          # what the estimator believes about area and mass

    print("=" * 78)
    print("1. 게이트: 못 쓰는 로그 걸러내기")
    print("=" * 78)
    cases = []

    good = flight_log(truth, SILK, seed=1)
    cases.append(("정상 비행", good))

    gusty = flight_log(truth, SILK, seed=2)
    gusty[:, 2] += np.linspace(0, 600, len(gusty))   # tension never settles
    cases.append(("과도상태만 (바람 급변)", gusty))

    short = flight_log(truth, SILK, seed=3, speeds=(6.0,), seconds=5.0)
    cases.append(("너무 짧은 로그", short))

    broken = flight_log(truth, SILK, seed=4)
    broken[:, 2] *= 0.25                              # tension sensor fell off
    cases.append(("장력센서 고장", broken))

    for label, log in cases:
        fit, reason = gate(log, assumed, SILK)
        verdict = f"CL {fit['cl']:.3f}" if fit else f"거부 — {reason}"
        print(f"  {label:<24}{verdict}")
    print()
    print("  주의: 장력센서 고장은 게이트를 통과합니다. 게인이 틀린 센서는")
    print("  물리적으로 모순된 값을 만들지 않고 그냥 다른 CL을 만들 뿐이라,")
    print("  로그만 보고는 '센서가 변한 것'과 '연이 변한 것'을 구분할 수 없습니다.")
    print("  (twin.py에서 mg 신호가 잡음에 묻힌다고 확인한 것과 같은 한계입니다.)")
    print("  => 이건 아래 변화 감지가 잡아주고, 그다음은 사람이 물리적으로")
    print("     확인해야 합니다. 계산으로 해결되는 문제가 아닙니다.")
    print()

    print("=" * 78)
    print("2. 누적 보정: 언제 멈춰야 하는가")
    print("=" * 78)
    cal = Calibration()
    print(f"  {'비행':>5}{'이번 추정':>11}{'누적 추정':>11}{'무작위 SE':>12}  상태")
    for n in range(1, 13):
        fit, reason = gate(flight_log(truth, SILK, seed=100 + n), assumed, SILK)
        if fit is None:
            print(f"  {n:>5}{'—':>11}{'—':>11}{'—':>12}  거부: {reason}")
            continue
        note = cal.add(fit["cl"])
        se = cal.random_sigma
        se_text = "—" if not np.isfinite(se) else f"{100*se/cal.estimate:.1f}%"
        print(f"  {n:>5}{fit['cl']:>11.3f}{cal.estimate:>11.3f}"
              f"{se_text:>12}  {note}")
    print()
    print(f"  수렴 추정값 {cal.estimate:.3f} "
          f"(참값 {truth.cl:.3f}, 편차 {100*(cal.estimate/truth.cl-1):+.1f}%)")
    print(f"  남은 무작위 오차 {100*cal.random_sigma/cal.estimate:.1f}%, "
          f"계통오차 바닥 {100*SYSTEMATIC_FLOOR:.1f}%")
    print()
    print("  => 비행을 더 해도 편차는 안 줄어듭니다. 매 비행이 같은 풍속계,")
    print("     같은 면적값을 쓰므로 같은 편향이 반복될 뿐입니다.")
    print("     바닥을 낮추려면 날지 말고 벤치에서 교정해야 합니다.")
    print()

    print("=" * 78)
    print("3. 수렴 뒤: 변화 감지")
    print("=" * 78)
    print("  연을 수리해서 실제 CL이 바뀐 상황을 넣어봅니다.")
    repaired = replace(truth, cl=truth.cl * 0.85)   # patched canopy, worse lift
    for n in range(13, 17):
        fit, _ = gate(flight_log(repaired, SILK, seed=200 + n), assumed, SILK)
        note = cal.add(fit["cl"])
        print(f"  {n:>5}{fit['cl']:>11.3f}{cal.estimate:>11.3f}"
              f"{'':>12}  {note}")
    print()
    print("  => 수렴 선언 뒤의 이동은 '추정이 좋아진 것'이 아니라")
    print("     '연이 달라진 것'으로 해석해야 맞습니다. 같은 통계를 반대")
    print("     방향으로 읽는 것이고, 그래서 한 곳에 둡니다.")
    print()

    print("=" * 78)
    print("4. 감지기 자체의 검증 (오경보율과 지연)")
    print("=" * 78)

    def run_trial(seed_base, change_at=None, shift=1.0, flights=20):
        c = Calibration()
        kite = truth
        fired = None
        for n in range(flights):
            if change_at is not None and n == change_at:
                kite = replace(truth, cl=truth.cl * shift)
            fit, _ = gate(flight_log(kite, SILK, seed=seed_base + n), assumed,
                          SILK)
            if fit is None:
                continue
            note = c.add(fit["cl"])
            if "CHANGE" in note and fired is None:
                fired = n
        return c, fired

    false_alarms = 0
    trials = 20
    for k in range(trials):
        _, fired = run_trial(3000 + 100 * k)
        if fired is not None:
            false_alarms += 1
    print(f"  변화가 없는 비행 {trials}회 시퀀스 중 오경보 {false_alarms}건 "
          f"({100*false_alarms/trials:.0f}%)")

    print(f"  {'실제 변화량':>12}{'감지율':>10}{'평균 지연':>12}")
    for shift, label in ((0.95, "-5%"), (0.85, "-15%"), (0.70, "-30%")):
        delays, hits = [], 0
        for k in range(trials):
            _, fired = run_trial(5000 + 100 * k, change_at=10, shift=shift)
            if fired is not None and fired >= 10:
                hits += 1
                delays.append(fired - 10)
        delay = f"{np.mean(delays):.1f}회" if delays else "—"
        print(f"  {label:>12}{100*hits/trials:>9.0f}%{delay:>12}")
    print()
    print("  => 작은 변화(-5%)는 비행별 산포에 묻혀 못 잡습니다. 이건 한계이지")
    print("     버그가 아닙니다 — 산포가 그만큼 크기 때문이고, 낮추려면 역시")
    print("     센서 교정이 먼저입니다.")
    print()

    values = [h[0] for h in cal.history]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(range(1, len(values) + 1), values, "o-", lw=1.5, label="누적 추정")
    ax.axhline(truth.cl, color="k", ls="--", lw=1, label="원래 참값")
    ax.axhline(repaired.cl, color="g", ls=":", lw=1, label="수리 후 참값")
    converged_at = next((i for i, h in enumerate(cal.history, 1)
                         if "CONVERGED" in h[2]), None)
    if converged_at:
        ax.axvline(converged_at, color="r", lw=1, label="수렴 선언")
    ax.set_xlabel("비행 횟수")
    ax.set_ylabel("CL 누적 추정")
    ax.set_title("자동 보정: 바닥에서 멈추고, 변화는 잡아낸다")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path("auto.png"), dpi=260)
    print("  saved out/auto.png")
