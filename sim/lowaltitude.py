"""Can the rig fly low at all, and what does small scale change?

Everything so far assumed one wind speed everywhere. That is fine at 30 m, but
a low-altitude test sits deep in the surface boundary layer, where wind follows
roughly a power law and a kite at 3 m sees noticeably less than the forecast.
Lift goes as the square of that, so the question is not precision -- it is
whether the thing leaves the ground.

Altitude and wind are coupled (the kite's altitude sets the wind it sees, which
sets its altitude), so each case is solved by fixed-point iteration.

Also reports what a short line does to the spool, sag and stretch errors, since
those were sized for 50-100 m.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

from model import equilibrium, sag_angle
from params import G, RHO_AIR, SILK, Kite
from spool import wind_on

REF_HEIGHT = 10.0     # m, standard met reference
ALPHA = 0.20          # power-law exponent, open flat ground
Z_MIN = 0.5           # m, below this the profile is meaningless



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def wind_at(height, reference_speed, alpha=ALPHA):
    return reference_speed * (max(height, Z_MIN) / REF_HEIGHT) ** alpha


def solve(kite, tether, reference_speed, length, alpha=ALPHA):
    """Self-consistent altitude: the kite feels the wind at its own height."""
    z = 0.5 * length
    for _ in range(200):
        v = wind_at(z, reference_speed, alpha)
        try:
            state, T, elev = equilibrium(kite, tether, v, length)
        except Exception:
            return None
        if not np.isfinite(elev) or elev <= 0:
            return None
        z_new = state[1]
        if abs(z_new - z) < 1e-6:
            z = z_new
            break
        z = 0.7 * z + 0.3 * z_new
    return {"altitude": z, "wind": wind_at(z, reference_speed, alpha),
            "tension": T, "elevation": elev, "length": length}


def min_wind_to_fly(kite, tether, length, target_elev=30.0):
    for ref in np.arange(2.0, 15.01, 0.05):
        r = solve(kite, tether, ref, length)
        if r and r["elevation"] >= target_elev:
            return ref, r
    return None, None


if __name__ == "__main__":
    kite = Kite()

    print("=" * 78)
    print(f"바람 경계층 (지수 {ALPHA}, 기준높이 {REF_HEIGHT:.0f} m)")
    print("=" * 78)
    print(f"  {'고도':>6}{'풍속 비율':>12}{'기준 6 m/s일 때':>18}")
    for z in (2, 3, 5, 10, 20, 30):
        ratio = wind_at(z, 1.0)
        print(f"  {z:>5.0f}m{ratio:>11.2f}{6*ratio:>16.2f} m/s")
    print()

    print("=" * 78)
    print("라인 길이별 비행 가능 최소 풍속 (고도각 30° 기준, 10 m 높이 기준풍속)")
    print("=" * 78)
    print(f"  {'라인':>6}{'필요 기준풍속':>16}{'도달 고도':>12}{'연이 느끼는 바람':>18}"
          f"{'장력':>9}")
    for length in (10.0, 20.0, 30.0, 50.0, 80.0):
        ref, r = min_wind_to_fly(kite, SILK, length)
        if ref is None:
            print(f"  {length:>5.0f}m{'불가':>16}")
            continue
        print(f"  {length:>5.0f}m{ref:>14.2f} m/s{r['altitude']:>11.1f}m"
              f"{r['wind']:>16.2f} m/s{r['tension']:>8.1f}N")
    print()

    uniform_min = np.sqrt(2 * kite.mass * G
                          / (RHO_AIR * kite.area
                             * (kite.cl - kite.cd * np.tan(np.radians(30.0)))))
    ref10, _ = min_wind_to_fly(kite, SILK, 10.0)
    ref50, _ = min_wind_to_fly(kite, SILK, 50.0)
    print("=" * 78)
    print("해석")
    print("=" * 78)
    print(f"  * 균일풍 가정이면 {uniform_min:.2f} m/s면 충분합니다. 경계층을 넣으면")
    print(f"    라인 10 m에서 {ref10:.2f} m/s, 50 m에서 {ref50:.2f} m/s가 필요합니다.")
    print(f"    -> 저고도일수록 '예보 풍속'이 더 높아야 합니다. "
          f"{100*(ref10/ref50-1):+.0f}% 차이.")
    print(f"  * 즉 저고도 테스트의 위험은 측정 정밀도가 아니라 '안 뜨는 것'입니다.")
    print(f"    풍속계가 1순위 측정 항목이라는 결론이 여기서도 다시 나옵니다.")
    print()

    survey_wind = 7.0
    print("=" * 78)
    print(f"짧은 라인이 기존 오차들을 얼마나 줄이는가 "
          f"(기준풍속 {survey_wind:.0f} m/s)")
    print("=" * 78)
    print(f"  {'라인':>6}{'스풀 층수':>11}{'반지름 변화':>14}"
          f"{'처짐 오차':>12}{'늘어남':>10}")
    for length in (10.0, 20.0, 50.0, 100.0):
        lengths, radii = wind_on(total=length)
        layers = len(radii) - 1
        r = solve(kite, SILK, survey_wind, length)
        if r is None:
            continue
        elev = np.radians(r["elevation"])
        d = sag_angle(r["tension"], length, elev, SILK)
        sag_err = length * np.cos(elev) * d
        stretch = length * r["tension"] / SILK.EA
        print(f"  {length:>5.0f}m{layers:>10d}겹"
              f"{radii[0]*1000:>7.0f}->{radii[-1]*1000:.0f} mm"
              f"{sag_err:>11.2f} m{stretch*100:>9.1f} cm")
    print()
    print("  => 저고도 소형 테스트에서는 스풀 층수도 처짐도 작아집니다.")
    print("     이 단계에서는 기압계도, 측정휠도 필요 없습니다.")
    print("     라인을 길게 쓰기 시작할 때 다시 꺼내시면 됩니다.")
    print()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    z = np.linspace(0.5, 40, 300)
    for ref in (4, 6, 8):
        axes[0].plot([wind_at(h, ref) for h in z], z, label=f"기준 {ref} m/s")
    axes[0].set_xlabel("풍속 [m/s]")
    axes[0].set_ylabel("고도 [m]")
    axes[0].set_title("지표 경계층 풍속 분포")
    axes[0].legend(fontsize=8)

    lens = np.arange(8.0, 81.0, 4.0)
    need = [min_wind_to_fly(kite, SILK, L)[0] for L in lens]
    axes[1].plot(lens, need, lw=2)
    axes[1].set_xlabel("라인 길이 [m]")
    axes[1].set_ylabel("필요 기준풍속 [m/s]")
    axes[1].set_title("짧은 라인일수록 더 센 바람이 필요")

    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path("lowaltitude.png"), dpi=260)
    print("  saved out/lowaltitude.png")
