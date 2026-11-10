"""Designing the indoor controlled-wind test.

Indoors removes the boundary layer and the gusts, which makes most of the
outdoor analysis irrelevant. What it adds is a hard requirement on the fan, and
one large opportunity: with wind speed under control you can sweep it and fit
CL and CD directly. Those two are the second-biggest source of prediction error
and everything downstream -- the digital twin, trim control, power modelling --
has been waiting on them.

The identification is algebraic. At equilibrium with wind v, tension T and
elevation theta:

    T*cos(theta) = q*A*CD              ->  CD = T*cos(theta) / (q*A)
    T*sin(theta) = q*A*CL - m*g        ->  CL = (T*sin(theta) + m*g) / (q*A)

with q = rho*v^2/2. So a run needs only wind, tension, angle, area and mass.
This sizes how accurately that recovers CL and CD given realistic instruments.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

from model import equilibrium
from params import G, RHO_AIR, SILK, Kite

LINE_INDOOR = 3.0        # m, what fits in a hall
FAN_EFFICIENCY = 0.40    # electrical to air power, generous for a cheap fan

# Measurement errors, 1 sigma. Angle is a photo with a protractor, which
# indoors is easier and better than any sensor on the ground station.
ERR_WIND = 0.05          # relative, decent handheld anemometer
ERR_TENSION = 0.08       # relative, BTS7960 current sense
ERR_ANGLE = np.radians(2.0)
ERR_AREA = 0.03
ERR_MASS = 0.01



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def fan_requirement(kite, speed):
    area = kite.area * 1.5          # cover the kite with margin
    air_power = 0.5 * RHO_AIR * area * speed**3
    return area, air_power, air_power / FAN_EFFICIENCY


def identify(wind, tension, elevation, area, mass):
    q = 0.5 * RHO_AIR * wind**2
    cd = tension * np.cos(elevation) / (q * area)
    cl = (tension * np.sin(elevation) + mass * G) / (q * area)
    return cl, cd


MIN_USABLE_ELEVATION = 10.0   # deg; below this the kite is not really flying


def usable_speeds(kite, speeds, length=LINE_INDOOR):
    """Drop wind speeds where the kite does not actually fly.

    `equilibrium` still returns a root below the horizon, which is not a flying
    kite but does produce plausible-looking numbers. Averaging one of those
    into a sweep silently biases the fitted coefficients, so they are filtered
    here rather than trusted.
    """
    good = []
    for v in speeds:
        try:
            _, _, elev = equilibrium(kite, SILK, v, length)
        except Exception:
            continue
        if np.isfinite(elev) and elev >= MIN_USABLE_ELEVATION:
            good.append(v)
    return good


def campaign(kite, speeds, rng, trials=4000):
    """Monte Carlo the whole measurement campaign; return CL/CD estimates."""
    speeds = usable_speeds(kite, speeds)
    if not speeds:
        raise ValueError("no wind speed in this set actually flies the kite")

    truth = []
    for v in speeds:
        state, T, elev = equilibrium(kite, SILK, v, LINE_INDOOR)
        truth.append((v, T, np.radians(elev)))

    out = np.empty((trials, 2))
    for i in range(trials):
        # Area and mass are measured once, so their error is shared by every
        # run in the campaign and cannot average out.
        area = kite.area * (1 + rng.normal(0, ERR_AREA))
        mass = kite.mass * (1 + rng.normal(0, ERR_MASS))

        cls, cds = [], []
        for v, T, elev in truth:
            cl, cd = identify(
                v * (1 + rng.normal(0, ERR_WIND)),
                T * (1 + rng.normal(0, ERR_TENSION)),
                elev + rng.normal(0, ERR_ANGLE),
                area, mass,
            )
            cls.append(cl)
            cds.append(cd)
        out[i] = (np.mean(cls), np.mean(cds))
    return out


if __name__ == "__main__":
    kite = Kite()
    rng = np.random.default_rng(3)

    print("=" * 78)
    print("1. 송풍기 요구사양")
    print("=" * 78)
    print(f"  {'풍속':>7}{'필요 송풍면적':>14}{'공기 출력':>12}{'전기 입력(효율 40%)':>20}")
    for v in (4.0, 5.0, 6.0, 8.0):
        area, air, elec = fan_requirement(kite, v)
        print(f"  {v:>6.1f}m/s{area:>12.2f} m^2{air:>10.0f} W{elec:>16.0f} W")
    print()
    print("  => 가정용 선풍기(보통 50~100 W, 2~3 m/s)로는 불가능합니다.")
    print("     산업용 송풍기나 서큘레이터 여러 대를 배열해야 합니다.")
    print("     실내는 공기가 되돌아오므로, 방 단면이 송풍 단면보다")
    print("     훨씬 커야 순환류가 실험을 망치지 않습니다.")
    print()

    print("=" * 78)
    print(f"2. 필요 공간 (라인 {LINE_INDOOR:.0f} m 기준)")
    print("=" * 78)
    print(f"  {'풍속':>7}{'고도각':>9}{'연 높이':>10}{'풍하 거리':>11}{'장력':>9}")
    for v in (4.0, 5.0, 6.0, 8.0):
        try:
            state, T, elev = equilibrium(kite, SILK, v, LINE_INDOOR)
        except Exception:
            continue
        if not np.isfinite(elev) or elev <= 0:
            print(f"  {v:>6.1f}m/s{'안 뜸':>9}")
            continue
        print(f"  {v:>6.1f}m/s{elev:>8.1f}°{state[1]:>9.2f} m"
              f"{state[0]:>10.2f} m{T:>8.1f} N")
    print()

    print("=" * 78)
    print("3. CL/CD 식별 정확도")
    print("=" * 78)
    requested = [4.0, 5.0, 6.0, 7.0, 8.0]
    usable = usable_speeds(kite, requested)
    dropped = [v for v in requested if v not in usable]
    single = campaign(kite, [6.0], rng)
    sweep = campaign(kite, usable, rng)

    if dropped:
        print(f"  사용 가능 풍속 {usable} m/s "
              f"(제외 {dropped} — 고도각 {MIN_USABLE_ELEVATION:.0f}° 미만)")
    print(f"  참값: CL={kite.cl:.3f}, CD={kite.cd:.3f}")
    print(f"  {'방식':<22}{'CL 추정':>20}{'CD 추정':>20}")
    for label, data in (("단일 풍속 1회", single),
                        (f"{len(usable)}점 풍속 스윕", sweep)):
        cl_lo, cl_hi = np.percentile(data[:, 0], [5, 95])
        cd_lo, cd_hi = np.percentile(data[:, 1], [5, 95])
        print(f"  {label:<22}{cl_lo:>8.3f} ~{cl_hi:>7.3f}"
              f"{cd_lo:>10.3f} ~{cd_hi:>7.3f}")
    cl_spread = np.ptp(np.percentile(sweep[:, 0], [5, 95])) / kite.cl
    cd_spread = np.ptp(np.percentile(sweep[:, 1], [5, 95])) / kite.cd
    print()
    print(f"  {len(usable)}점 스윕 기준 상대 폭: CL ±{100*cl_spread/2:.0f}%, "
          f"CD ±{100*cd_spread/2:.0f}%")
    print(f"  (현재 가정하던 불확실성은 ±40%)")
    print()

    print("=" * 78)
    print("해석")
    print("=" * 78)
    print("  * 실내 제어풍의 진짜 가치는 정밀도가 아니라 '반복 가능한 스윕'입니다.")
    print("    풍속을 바꿔가며 장력·고도각만 재면 CL/CD가 대수적으로 풀립니다.")
    print(f"    ±40%였던 불확실성이 ±{100*cl_spread/2:.0f}% 수준으로 줄어듭니다.")
    print("  * 각도는 실내에서 사진 한 장이면 됩니다. 지상국 각도센서를")
    print("    보류하신 판단이 이 단계에서는 완전히 맞습니다.")
    print("  * 남는 오차는 면적·질량처럼 캠페인 전체가 공유하는 항목이라")
    print("    스윕을 늘려도 안 줄어듭니다. 그 둘만 한 번 제대로 재두세요.")
    print("  * 걸림돌은 송풍기입니다. 선풍기로는 안 되고, 이게 안 되면")
    print("    실내 계획 자체가 성립하지 않으니 가장 먼저 확인하실 항목입니다.")
    print()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, col, name, truth_val in ((axes[0], 0, "CL", kite.cl),
                                     (axes[1], 1, "CD", kite.cd)):
        ax.hist(single[:, col], bins=60, alpha=0.6, label="단일 풍속")
        ax.hist(sweep[:, col], bins=60, alpha=0.8, label="5점 스윕")
        ax.axvline(truth_val, color="k", ls="--", lw=1.5, label="참값")
        ax.set_xlabel(name)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("표본 수")
    fig.suptitle("실내 풍속 스윕으로 좁혀지는 공력계수")
    fig.tight_layout()
    fig.savefig(out_path("indoor.png"), dpi=260)
    print("  saved out/indoor.png")
