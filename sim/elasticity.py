"""Tether stiffness as a dial.

A spring spliced into a stiff line sets the axial stiffness directly, and in
series the spring dominates, so EA = k * L. That makes k the single knob for
the whole rigid-versus-elastic question -- the damping study in
compare_tether.py showed material damping is swamped by the kite's own
aerodynamic damping, so stiffness is the only thing left to vary.

Run bare to sweep k and draw the curves. Pass a number to size one spring:

    python elasticity.py 60
"""

import sys

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from dataclasses import replace

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import wind
from model import effective_mass, equilibrium, simulate
from params import LINE_LENGTH, SILK, WIND_BASE, Kite

SPOOL_RATE = 0.047
TURBULENCE_BAND = 0.1   # Hz; gust energy lives below this




def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def ascii_log_ticks(axis):
    """Log tick labels without mathtext, so no U+2212 is needed.

    matplotlib's LogFormatterSciNotation writes 10^{-1} as mathtext using a
    Unicode minus, and the Korean UI font has no glyph for it -- the figure
    then reads "10¤1". axes.unicode_minus does not cover this path because it
    only touches the plain scalar formatter. Writing the labels directly is
    the one way that does not depend on which font gets picked.
    """
    from matplotlib.ticker import FuncFormatter, NullFormatter

    def fmt(v, _pos):
        if v <= 0:
            return ""
        e = int(round(np.log10(v)))
        if not np.isclose(v, 10.0 ** e):
            return ""
        if -2 <= e <= 3:
            return f"{10.0 ** e:g}"
        return f"1e{e}"        # ASCII hyphen, never U+2212

    axis.set_major_formatter(FuncFormatter(fmt))
    axis.set_minor_formatter(NullFormatter())

def tether_for(k, length=LINE_LENGTH, base=SILK):
    """A spring of stiffness k in a stiff line of this length."""
    return replace(base, name=f"k={k:.0f} N/m", EA=k * length,
                   damping_ratio=0.002, mass_per_m=0.0006, diameter=0.0008)


def gust_field(base=WIND_BASE, peak=10.0, t0=2.0, dur=1.5):
    return wind.gust(wind.uniform(base), peak / base, t0, dur)


def chatter_cmd(freq, length=LINE_LENGTH, t0=2.0, rate=SPOOL_RATE):
    amp = rate / (2 * np.pi * freq)
    def cmd(t):
        return length if t < t0 else length + amp * np.sin(2 * np.pi * freq * (t - t0))
    return cmd


def evaluate(k, kite, length=LINE_LENGTH):
    tether = tether_for(k, length)
    state, T0, elev = equilibrium(kite, tether, WIND_BASE, length)
    if not np.isfinite(elev) or elev <= 0:
        return None

    fn = tether.natural_freq(length, effective_mass(kite))
    gust = simulate(kite, tether, gust_field(), lambda t: length, state,
                    20.0, n_out=600)
    chat = simulate(kite, tether, wind.uniform(WIND_BASE), chatter_cmd(fn, length),
                    state, 20.0, n_out=800)
    late = chat["t"] >= 10.0

    return {
        "k": k,
        "ea": tether.EA,
        "stretch": np.linalg.norm(state[:2]) - length,
        "freq": fn,
        "tension": T0,
        "peak": gust["tension"].max(),
        "ripple": np.ptp(chat["tension"][late]) / 2,
        "ripple_pct": 100 * np.ptp(chat["tension"][late]) / 2 / T0,
        "break_margin": tether.break_load / gust["tension"].max(),
    }


def one(k, kite, length=LINE_LENGTH):
    r = evaluate(k, kite, length)
    print("=" * 74)
    print(f"용수철 상수 k = {k:.1f} N/m, 라인 {length:.0f} m")
    print("=" * 74)
    if r is None:
        print("  이 조건에서는 연이 뜨지 않습니다.")
        return
    print(f"  등가 EA          {r['ea']:>10.0f} N   (= k x L)")
    print(f"  정상 장력        {r['tension']:>10.2f} N")
    print(f"  정상 늘어남      {r['stretch']*100:>10.1f} cm")
    print(f"  고유진동수       {r['freq']:>10.2f} Hz")
    print(f"  돌풍 최대장력    {r['peak']:>10.1f} N  (파단여유 {r['break_margin']:.0f}배)")
    print(f"  채터 장력맥동    {r['ripple_pct']:>10.0f} %")
    print()
    if r["freq"] < 5 * TURBULENCE_BAND:
        print(f"  주의: 고유진동수가 난류 대역({TURBULENCE_BAND:.1f} Hz)에 너무 가깝습니다.")
        print(f"        k를 {((5*TURBULENCE_BAND*2*np.pi)**2)*kite.mass:.0f} N/m 이상으로 올리세요.")
    else:
        print(f"  고유진동수가 난류 대역({TURBULENCE_BAND:.1f} Hz)의 "
              f"{r['freq']/TURBULENCE_BAND:.0f}배라 깨끗하게 분리됩니다.")
    print()


def sweep(kite, length=LINE_LENGTH):
    ks = np.unique(np.round(np.logspace(np.log10(5), np.log10(5000), 22)))
    rows = [r for r in (evaluate(k, kite, length) for k in ks) if r]

    print("=" * 74)
    print(f"용수철 상수 스윕 (라인 {length:.0f} m, 바람 {WIND_BASE:.0f} m/s, "
          f"돌풍 10 m/s)")
    print("=" * 74)
    print(f"  {'k [N/m]':>9}{'EA [N]':>10}{'늘어남':>10}{'고유진동':>11}"
          f"{'돌풍최대':>10}{'채터맥동':>10}")
    for r in rows:
        print(f"  {r['k']:>9.0f}{r['ea']:>10.0f}{r['stretch']*100:>9.1f}cm"
              f"{r['freq']:>9.2f}Hz{r['peak']:>9.1f}N{r['ripple_pct']:>9.0f}%")
    print()

    soft, stiff = rows[0], rows[-1]
    print("=" * 74)
    print("해석")
    print("=" * 74)
    print(f"  * 늘어남은 1/k로 줄고 고유진동수는 sqrt(k)로 오릅니다. 둘 다 교과서")
    print(f"    그대로이고, 모델이 제대로 돌고 있다는 확인이기도 합니다.")
    print(f"  * 돌풍 최대장력은 {soft['peak']:.0f} N에서 {stiff['peak']:.0f} N으로 "
          f"오르다 포화합니다.")
    print(f"    뻣뻣할수록 충격을 그대로 전달하고, 포화값이 강체 한계입니다.")
    print(f"  * 채터 맥동은 {soft['ripple_pct']:.0f}%에서 "
          f"{stiff['ripple_pct']:.0f}%까지 단조 증가합니다.")
    print()
    print(f"  => 하중과 채터만 보면 무조건 무를수록 좋습니다. 반대 방향으로")
    print(f"     미는 건 늘어남 하나뿐인데, 그건 장력으로 보정되는 값입니다.")
    floor = ((5 * TURBULENCE_BAND * 2 * np.pi) ** 2) * kite.mass
    print(f"     진짜 하한은 고유진동수입니다 — 난류 대역과 겹치지 않으려면")
    print(f"     k > {floor:.0f} N/m. 그 위에서는 무른 쪽이 유리합니다.")
    print()

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    ks = [r["k"] for r in rows]
    series = [
        (axes[0][0], [r["stretch"] * 100 for r in rows], "정상 늘어남 [cm]", True),
        (axes[0][1], [r["freq"] for r in rows], "고유진동수 [Hz]", True),
        (axes[1][0], [r["peak"] for r in rows], "돌풍 최대장력 [N]", False),
        (axes[1][1], [r["ripple_pct"] for r in rows], "채터 장력맥동 [%]", False),
    ]
    for ax, ys, label, logy in series:
        ax.plot(ks, ys, "o-", lw=1.6, ms=3.5, color="#4C72B0")
        ax.set_xscale("log")
        ascii_log_ticks(ax.xaxis)
        if logy:
            ax.set_yscale("log")
            ascii_log_ticks(ax.yaxis)
        ax.set_xlabel("용수철 상수 k [N/m]")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3, which="both")
        ax.axvline(floor, color="#B4322C", ls="--", lw=1)

    axes[0][0].text(floor * 1.25, max(series[0][1]) * 0.6,
                    f"난류 분리 하한\nk = {floor:.0f} N/m",
                    color="#B4322C", fontsize=8)
    fig.suptitle("줄 강성 하나만 바꿨을 때 — 빨간 선 왼쪽은 고유진동이 난류 대역과 겹침")
    fig.tight_layout()
    fig.savefig(out_path("elasticity.png"), dpi=260)
    print("  saved out/elasticity.png")


if __name__ == "__main__":
    kite = Kite()
    if len(sys.argv) > 1:
        one(float(sys.argv[1]), kite)
    else:
        sweep(kite)
