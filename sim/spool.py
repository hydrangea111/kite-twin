"""The spool radius is not a constant, and everything downstream assumed it was.

100 m of line cannot fit in one layer on a small drum, so it winds in layers and
the effective radius grows with every one. That breaks two things at once: the
payout rate (v = omega * r) and, worse, the encoder-counts-to-metres constant
that the whole altitude estimate rests on.

This sizes the effect and compares the fixes. Nothing here needs the kite model;
it is pure winding geometry.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

CORE_RADIUS = 0.015      # m, as currently assumed
LINE_DIAMETER = 0.0010   # m, the silk                            MEASURE
SPOOL_WIDTH = 0.050      # m, flange to flange                    MEASURE
TOTAL_LINE = 100.0       # m
OUTPUT_RPM = 30.0



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def wind_on(total=TOTAL_LINE, core=CORE_RADIUS, width=SPOOL_WIDTH,
            d=LINE_DIAMETER):
    """Length stored vs effective radius, layer by layer."""
    turns_per_layer = width / d
    lengths, radii = [0.0], [core]
    stored = 0.0
    layer = 0
    while stored < total:
        layer += 1
        r = core + (layer - 0.5) * d
        stored += turns_per_layer * 2 * np.pi * r
        lengths.append(min(stored, total))
        radii.append(core + layer * d)
    return np.array(lengths), np.array(radii)


def report():
    lengths, radii = wind_on()
    layers = len(radii) - 1
    r_full = radii[-1]
    omega = OUTPUT_RPM * 2 * np.pi / 60.0

    print("=" * 76)
    print("스풀 권선 기하")
    print("=" * 76)
    print(f"  코어 반지름 {CORE_RADIUS*1000:.0f} mm, 폭 {SPOOL_WIDTH*1000:.0f} mm, "
          f"실 지름 {LINE_DIAMETER*1000:.1f} mm, 총 {TOTAL_LINE:.0f} m")
    print(f"  -> {layers}겹으로 감기고, 반지름이 "
          f"{CORE_RADIUS*1000:.0f} -> {r_full*1000:.0f} mm "
          f"({r_full/CORE_RADIUS:.2f}배)")
    print()
    print(f"  권취속도: 빈 스풀 {omega*CORE_RADIUS*100:.1f} cm/s, "
          f"가득 찬 스풀 {omega*r_full*100:.1f} cm/s")
    print(f"  -> 지금까지 쓴 4.7 cm/s는 코어 기준이라 실제보다 보수적이었습니다.")
    print()

    print("=" * 76)
    print("엔코더 길이 환산 오차 (이쪽이 진짜 문제)")
    print("=" * 76)
    print("  고정 반지름을 가정하고 회전수로 길이를 환산할 때의 오차:")
    print(f"  {'가정 반지름':>14}{'최대 오차':>14}{'50 m 지점에서':>18}")

    # True length paid out vs what a fixed-radius conversion would report.
    revs = np.cumsum(np.diff(lengths) / (2 * np.pi * radii[1:]))
    revs = np.concatenate([[0.0], revs])
    for assumed in (CORE_RADIUS, radii[len(radii) // 2], r_full):
        estimated = revs * 2 * np.pi * assumed
        err = estimated - lengths
        at50 = np.interp(50.0, lengths, err)
        print(f"  {assumed*1000:>12.0f} mm{np.abs(err).max():>12.1f} m"
              f"{at50:>16.1f} m")
    print()
    print("  => 어떤 고정값을 써도 수십 m가 틀립니다. 저희가 10 cm 단위로")
    print("     다듬던 늘어남·처짐 보정을 완전히 덮어버리는 크기입니다.")
    print()

    print("=" * 76)
    print("해결책 비교")
    print("=" * 76)
    print("  1. 층수를 펌웨어에서 모델링  — 비용 0, 다만 실 지름/폭/초기")
    print("     감긴 양을 정확히 알아야 하고 실이 고르게 안 감기면 틀어집니다.")
    print("  2. 레벨와인드(실 안내 기구)로 1층 유지 — 기구가 복잡하고")
    print(f"     {TOTAL_LINE:.0f} m를 1층에 감으려면 폭이 "
          f"{TOTAL_LINE/(2*np.pi*CORE_RADIUS)*LINE_DIAMETER*100:.0f} cm 필요. 비현실적.")
    print("  3. 측정휠(엔코더 달린 아이들러)에 실을 통과시켜 길이를 직접 측정")
    print("     — 스풀 기하와 완전히 무관해집니다. 케이블 길이 측정의 표준 방식.")
    print("     이미 장력 도르래 자리를 검토했었으니 같은 지점에 달면 됩니다.")
    print()
    print("  => 3번을 권장합니다. 1번은 보정이 틀리면 조용히 틀리는데,")
    print("     3번은 구조적으로 안 틀립니다. 엔코더 하나 값이면 됩니다.")
    print()


if __name__ == "__main__":
    report()
    lengths, radii = wind_on()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].plot(lengths, radii * 1000, lw=2)
    axes[0].axhline(CORE_RADIUS * 1000, color="k", ls="--", lw=1,
                    label="가정했던 15 mm")
    axes[0].set_xlabel("감긴 실 길이 [m]")
    axes[0].set_ylabel("유효 반지름 [mm]")
    axes[0].set_title("반지름은 상수가 아님")
    axes[0].legend()

    revs = np.concatenate([[0.0], np.cumsum(np.diff(lengths)
                                            / (2 * np.pi * radii[1:]))])
    for assumed, name in ((CORE_RADIUS, "코어 15 mm"),
                          (radii[len(radii) // 2], "중간값"),
                          (radii[-1], "최대")):
        axes[1].plot(lengths, revs * 2 * np.pi * assumed - lengths,
                     label=f"{name} 가정")
    axes[1].axhline(0, color="k", lw=1)
    axes[1].set_xlabel("실제 풀린 길이 [m]")
    axes[1].set_ylabel("추정 오차 [m]")
    axes[1].set_title("고정 반지름 환산의 오차")
    axes[1].legend(fontsize=8)

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path("spool.png"), dpi=260)
    print("  saved out/spool.png")
