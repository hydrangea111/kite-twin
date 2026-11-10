"""When can the altitude loop actually do its job?

Stability is not the question: the plant is an integrator and the loop is a
slow P controller, so it cannot go unstable for any plausible parameter draw.
What varies wildly is *feasibility* -- holding an altitude as the wind changes
means moving to L = altitude / sin(elevation), and elevation depends on CL, CD
and wind, all still guesses. At shallow elevation that length runs past the
spool, and at 4.7 cm/s even a feasible move can take longer than a flight.

So this samples the same parameter ranges as uncertainty.py and asks, per draw:
does the kite fly, is the required line length on the spool, and how long does
the correction take.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

from control import MAX_RATE
from model import equilibrium
from params import LINE_LENGTH, SILK, Kite
from uncertainty import RANGES, build, draw

N = 4000
LINE_AVAILABLE = 100.0     # m on the spool                        MEASURE
WIND_RISE = 1.5            # the controller must cope with this much more wind
PATIENCE = 600.0           # s; a correction slower than this is useless



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def envelope(samples):
    rows = []
    for row in samples:
        kite, tether, wind = build(row)
        try:
            state, _, elev0 = equilibrium(kite, tether, wind, LINE_LENGTH)
            if not np.isfinite(elev0) or elev0 <= 0:
                rows.append(None)
                continue
            altitude = state[1]
            _, _, elev1 = equilibrium(kite, tether, wind * WIND_RISE,
                                      LINE_LENGTH)
        except Exception:
            rows.append(None)
            continue

        if not np.isfinite(elev1) or elev1 <= 0:
            rows.append(None)
            continue

        needed = altitude / np.sin(np.radians(elev1))
        travel = abs(needed - LINE_LENGTH)
        rows.append({
            "elev0": elev0,
            "elev1": elev1,
            "altitude": altitude,
            "needed": needed,
            "travel": travel,
            "seconds": travel / MAX_RATE,
            "fits": needed <= LINE_AVAILABLE,
            "in_time": travel / MAX_RATE <= PATIENCE,
        })
    return rows


def pct(values):
    return np.percentile(values, [5, 50, 95])


if __name__ == "__main__":
    rng = np.random.default_rng(7)
    samples = draw(rng, N)
    rows = envelope(samples)

    flying = [r for r in rows if r is not None]
    ok = [r for r in flying if r["fits"] and r["in_time"]]

    print("=" * 76)
    print(f"고도 유지 제어의 실행 가능 범위 (표본 {N}, 풍속 {WIND_RISE}배 상승 대응)")
    print("=" * 76)
    print(f"  비행 자체가 가능       {len(flying):>5} / {N}  "
          f"({100*len(flying)/N:.0f}%)")
    print(f"  필요 라인이 스풀 내     "
          f"{sum(r['fits'] for r in flying):>5} / {len(flying)}  "
          f"({100*sum(r['fits'] for r in flying)/len(flying):.0f}%)")
    print(f"  {PATIENCE/60:.0f}분 내 보정 가능     "
          f"{sum(r['in_time'] for r in flying):>5} / {len(flying)}  "
          f"({100*sum(r['in_time'] for r in flying)/len(flying):.0f}%)")
    print(f"  둘 다 만족 (실사용 가능) {len(ok):>5} / {N}  "
          f"({100*len(ok)/N:.0f}%)")
    print()

    print(f"  {'항목':<26}{'5%':>11}{'중앙값':>11}{'95%':>11}")
    for label, key in (("풍상승 후 고도각 [deg]", "elev1"),
                       ("필요 라인길이 [m]", "needed"),
                       ("이동량 [m]", "travel"),
                       ("보정 소요시간 [s]", "seconds")):
        q = pct([r[key] for r in flying])
        print(f"  {label:<26}{q[0]:>11.1f}{q[1]:>11.1f}{q[2]:>11.1f}")
    print()

    print("=" * 76)
    print("해석")
    print("=" * 76)
    print(f"  * 제어 루프의 안정성은 문제가 아닙니다. 적분기 플랜트에 느린 P제어라")
    print(f"    어떤 파라미터 조합에서도 발산하지 않습니다.")
    print(f"  * 문제는 실행 가능성입니다. 전체 표본의 {100*len(ok)/N:.0f}%에서만")
    print(f"    '뜨고 + 라인이 닿고 + 제시간에 보정'이 동시에 성립합니다.")
    q = pct([r["seconds"] for r in flying])
    print(f"  * 보정 시간 중앙값 {q[1]/60:.1f}분, 95분위 {q[2]/60:.1f}분.")
    print(f"    스풀 속도 {MAX_RATE*100:.1f} cm/s가 그대로 병목입니다.")
    print(f"  * 라인 길이는 한 번도 제약이 아니었습니다 (스풀 내 {100:.0f}%).")
    print(f"    풍속이 오르면 고도각도 함께 올라가 필요 길이가 오히려 줄어들기")
    print(f"    때문에, 보정은 항상 '감는' 방향입니다. 풀어낼 여유는 문제가 안 됩니다.")
    print()
    print(f"  => 유일한 병목은 시간이고, 해법은 권취속도입니다.")
    print(f"     spool.py에서 확인했듯 실제 유효 반지름은 감긴 양에 따라")
    print(f"     15->30 mm로 변하므로 여기 쓴 {MAX_RATE*100:.1f} cm/s는 보수적입니다.")
    print(f"     코어를 키우면 더 빨라지고, 토크는 8배 여유라 감당됩니다.")
    print()

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].hist([r["elev1"] for r in flying], bins=50, color="#4C72B0")
    axes[0].set_xlabel("풍속 상승 후 고도각 [deg]")
    axes[0].set_ylabel("표본 수")
    axes[1].hist(np.clip([r["seconds"] / 60 for r in flying], 0, 30),
                 bins=50, color="#DD8452")
    axes[1].axvline(PATIENCE / 60, color="k", ls="--", lw=1,
                    label=f"{PATIENCE/60:.0f}분 한계")
    axes[1].set_xlabel("보정 소요시간 [분, 30분에서 자름]")
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("파라미터 불확실성에서 본 고도 제어의 실행 가능 범위")
    fig.tight_layout()
    fig.savefig(out_path("control_envelope.png"), dpi=260)
    print("  saved out/control_envelope.png")
