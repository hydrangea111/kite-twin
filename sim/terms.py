"""How big is every term we kept, and every term we threw away?

The report repeats that this is "the lightest model that answers the
question". That is a claim, and until now it was supported by argument rather
than by numbers. This measures it.

Two halves:

  * the terms that are in the model, sized against each other over a real
    flight, so it is clear which ones carry the answer and which are
    decoration;
  * the terms that are *not* in the model, estimated analytically and ranked.
    A simplification is only defensible if the term it drops is smaller than
    the uncertainty already present -- and the parameter study puts that
    uncertainty at tens of percent, which is the bar every entry below has
    to clear.

Anything that fails that bar is not a simplification, it is an error, and it
is listed as one.

    python terms.py
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import wind
from model import aero_force, equilibrium, line_force, simulate, tension, effective_mass
from params import (CD_CYLINDER, G, LINE_LENGTH, RHO_AIR, SILK,
                    SUSPENSION_LENGTH, WIND_BASE, Kite, Payload)

# Parameter uncertainty the report already carries (7장): C_L and C_D are
# known to about +/-40 %, and the campaign spread on C_L is 31 %. A dropped
# term below this is invisible; above it, dropping it is a choice that has to
# be defended.
UNCERTAINTY_FLOOR = 0.31



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def kept_terms(kite, tether, field, length, state, v=None):
    """Magnitude of each force the model actually computes, in newtons."""
    r = state[:2]
    v = np.zeros(2) if v is None else v
    radius = float(np.linalg.norm(r))
    rhat = r / radius

    w = field(0.0, r) - v
    speed = float(np.linalg.norm(w))
    q = 0.5 * RHO_AIR * speed**2 * kite.area

    T = tension(radius, float(v @ rhat), length, tether, kite.mass)
    lf = line_force(v, field(0.0, 0.5 * r), r, tether, length)
    weight_line = 0.5 * tether.mass_per_m * length * G
    drag_line = float(np.linalg.norm(lf - np.array([0.0, -weight_line])))

    k = tether.stiffness(length)
    c = 2 * tether.damping_ratio * np.sqrt(k * kite.mass)

    return {
        "연 양력 q·A·C_L": q * kite.cl,
        "연 항력 q·A·C_D": q * kite.cd,
        "중력 m·g": kite.mass * G,
        "줄 장력 T": float(T),
        "줄 자중 (연 몫)": weight_line,
        "줄 항력 (연 몫)": drag_line,
        "줄 감쇠 c·ṙ": float(abs(c * (v @ rhat))),
    }


def dropped_terms(kite, tether, length, speed, elev_deg):
    """Terms the model does not contain, estimated analytically.

    Each is expressed as a fraction of the dominant aerodynamic force, so it
    can be compared directly against the parameter uncertainty.
    """
    q = 0.5 * RHO_AIR * speed**2 * kite.area
    aero = q * float(np.hypot(kite.cl, kite.cd))
    payload = Payload()
    out = {}

    # 1. Tether inertia. The line has mass and it accelerates with the kite;
    #    the model gives it weight and drag but no inertia. For a line fixed
    #    at one end the effective added mass is one third of its total.
    m_line = tether.mass_per_m * length
    out["줄 관성 (유효질량 m_줄/3)"] = (
        (m_line / 3.0) / kite.mass,
        "고유진동수를 낮춘다. 동역학에만 영향, 정상상태에는 무영향",
    )

    # 2. Added mass of air. A flat plate accelerating normal to itself drags
    #    along roughly the air in a sphere of diameter equal to its span.
    span = float(np.sqrt(kite.area))
    m_added = RHO_AIR * np.pi / 6.0 * span**3
    out["공기 부가질량"] = (
        m_added / kite.mass,
        "급가속 구간에서만. 돌풍 상승률이 클수록 커진다",
    )

    # 3. Kite rotational inertia -- the bridle assumption removes the pitch
    #    degree of freedom entirely. Size it as the aerodynamic moment that
    #    a small offset between centre of pressure and bridle point would
    #    produce, for an offset of 5 % of chord.
    out["연 피치 자유도 (압력중심 5 % 어긋남)"] = (
        0.05 * span * aero / (aero * span) * 1.0,
        "브라이들 가정 자체. 수치로는 확인 불가, 비행 필요",
    )

    # 4. Reynolds dependence of C_D. For a bluff plate the drag coefficient is
    #    nearly Re-independent above ~1e4; here Re is well above that.
    nu = 1.5e-5
    re = speed * span / nu
    out["레이놀즈 의존성"] = (
        0.02 if re > 1e4 else 0.2,
        f"Re = {re:.0e}. 평판 둔각물체는 1e4 위에서 거의 무관",
    )

    # 5. Canopy deformation. Nylon at this loading stretches by T/(E t width);
    #    take the resulting camber change as proportional to that strain.
    strain_fabric = (q * kite.cl) / (2.0e9 * 1e-4 * span)   # E~2 GPa, t~0.1 mm
    out["캐노피 변형"] = (
        min(strain_fabric * 10.0, 1.0),
        "신축성 없는 원단 선택으로 억제됨 (2.2절)",
    )

    # 6. Payload swing back-reaction on the kite. The two-body model has it;
    #    the single-body model used for most studies does not.
    out["탑재물 진자 반력 (단일체 모델)"] = (
        payload.mass * G * np.sin(np.radians(24.0)) / aero,
        "2체 모델(3.7절)에는 있음. 단일체 결과에만 해당",
    )

    # 7. Tether bending stiffness. A silk line of this diameter has negligible
    #    EI; the ratio of bending to tension stiffness over the span.
    ei = 2.0e9 * np.pi * tether.diameter**4 / 64.0
    t_typ = q * float(np.hypot(kite.cl, kite.cd))
    out["줄 굽힘강성"] = (
        ei / (t_typ * length**2),
        "장력이 지배. 줄을 케이블로 다루는 근거",
    )

    # 8. Apparent-wind correction from kite motion in steady flight: zero by
    #    definition at equilibrium, but finite during transients.
    out["정상상태 겉보기바람 보정"] = (0.0, "평형에서는 정확히 0")

    return out


if __name__ == "__main__":
    kite, tether = Kite(), SILK
    field = wind.shear(WIND_BASE)
    state, T0, elev = equilibrium(kite, tether, field, LINE_LENGTH)

    print("=" * 78)
    print(f"1. 모델이 계산하는 항의 크기 (정상비행, 고도각 {elev:.1f}°)")
    print("=" * 78)
    kept = kept_terms(kite, tether, field, LINE_LENGTH, state)
    biggest = max(kept.values())
    print(f"  {'항':<26}{'크기 [N]':>12}{'최대항 대비':>14}")
    for name, value in sorted(kept.items(), key=lambda kv: -kv[1]):
        print(f"  {name:<26}{value:>11.3f}N{100*value/biggest:>13.1f}%")
    print()

    # 돌풍 중에는 비중이 어떻게 바뀌는가
    gust = wind.gust(wind.shear(WIND_BASE), 10.0 / 6.0, 2.0, 1.5)
    res = simulate(kite, tether, gust, lambda t: LINE_LENGTH, state, 12.0)
    i = int(np.argmax(res["tension"]))
    peak_state = np.concatenate([res["r"][i], res["v"][i]])
    kept_gust = kept_terms(kite, tether, gust, LINE_LENGTH,
                           peak_state, v=res["v"][i])

    print("=" * 78)
    print("2. 돌풍 최대 장력 순간에는 어떻게 달라지는가")
    print("=" * 78)
    print(f"  {'항':<26}{'정상 [N]':>11}{'돌풍 [N]':>11}{'배율':>9}")
    for name in sorted(kept, key=lambda n: -kept_gust[n]):
        a, b = kept[name], kept_gust[name]
        ratio = f"{b/a:.2f}x" if a > 1e-9 else "—"
        print(f"  {name:<26}{a:>10.3f}N{b:>10.3f}N{ratio:>9}")
    print()
    print("  감쇠항이 정상상태에서 정확히 0인 것은 ṙ = 0 이기 때문이고,")
    print("  돌풍 중에만 값을 갖는다. 그 크기가 다른 항에 비해 얼마나 되는지가")
    print("  5장에서 '재질 감쇠는 영향이 없다'고 한 결론의 수치적 근거이다.")
    print()

    # ── 생략한 항 ────────────────────────────────────────────────────
    print("=" * 78)
    print("3. 모델에 없는 항 — 크기 순")
    print("=" * 78)
    speed = float(np.linalg.norm(field(0.0, state[:2])))
    dropped = dropped_terms(kite, tether, LINE_LENGTH, speed, elev)
    print(f"  파라미터 불확실성 바닥: {UNCERTAINTY_FLOOR:.0%} "
          f"(이보다 작으면 넣어도 보이지 않는다)")
    print()
    print(f"  {'생략한 항':<34}{'상대크기':>10}{'판정':>10}  근거")
    ranked = sorted(dropped.items(), key=lambda kv: -kv[1][0])
    for name, (size, note) in ranked:
        verdict = "무시 가능" if size < UNCERTAINTY_FLOOR else "검토 필요"
        print(f"  {name:<34}{size:>9.1%}{verdict:>10}  {note}")
    print()

    over = [n for n, (sz, _) in ranked if sz >= UNCERTAINTY_FLOOR]
    if over:
        print("  불확실성 바닥을 넘는 항:")
        for n in over:
            print(f"    - {n}")
        print("  이들은 '단순화'가 아니라 '한계'로 기록해야 한다(17장).")
    else:
        print("  모든 생략 항이 현재 파라미터 불확실성 아래에 있다.")
    print()

    # ── 줄 관성이 고유진동수를 얼마나 낮추는가 ─────────────────────────
    print("=" * 78)
    print("4. 가장 큰 생략 항의 실제 영향 — 줄 관성")
    print("=" * 78)
    m_line = tether.mass_per_m * LINE_LENGTH
    # 기체 질량이 아니라 공기 부가질량을 더한 유효 질량을 쓴다. 다른 다섯 곳은
    # 부가질량을 넣을 때 함께 고쳤는데 이 두 줄만 남아 1.84 Hz 를 찍고 있었다.
    m_eff = effective_mass(kite)
    f0 = tether.natural_freq(LINE_LENGTH, m_eff)
    f1 = tether.natural_freq(LINE_LENGTH, m_eff + m_line / 3.0)
    print(f"  줄 질량 {1000*m_line:.0f} g, 유효 질량 {1000*m_line/3:.0f} g "
          f"(연+부가질량 {1000*m_eff:.0f} g 의 {100*m_line/3/m_eff:.1f} %)")
    print(f"  고유진동수 {f0:.3f} Hz -> {f1:.3f} Hz  ({100*(f1/f0-1):+.1f} %)")
    print()
    print("  제어 대역폭을 고유진동수의 1/5 이하로 잡는다는 설계 규칙에서")
    print(f"  {f0/5:.2f} Hz 와 {f1/5:.2f} Hz 의 차이는 실질적이지 않다.")
    print("  줄을 분절해 관성을 넣는 비용(상태변수 수십 배)에 비해 이득이 없다.")
    print()

    # ── 그림 ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))

    ax = axes[0]
    names = [n for n, _ in sorted(kept.items(), key=lambda kv: -kv[1])]
    xs = np.arange(len(names))
    ax.bar(xs - 0.2, [kept[n] for n in names], 0.4, label="정상비행")
    ax.bar(xs + 0.2, [kept_gust[n] for n in names], 0.4, label="돌풍 최대")
    ax.set_xticks(xs)
    ax.set_xticklabels([n.split()[0] + "\n" + " ".join(n.split()[1:])
                        for n in names], fontsize=7.5)
    ax.set_ylabel("크기 [N]")
    ax.set_title("모델이 계산하는 항")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    ax = axes[1]
    dn = [n for n, _ in ranked]
    FLOOR = 1.2e-4          # 로그 축의 왼쪽 끝. 이보다 작은 항은 여기 붙인다
    dv = [max(dropped[n][0], FLOOR) for n in dn]
    colors = ["#d62728" if v >= UNCERTAINTY_FLOOR else "#4292c6" for v in dv]
    ax.barh(range(len(dn)), dv, color=colors, left=1e-4)
    for i, n in enumerate(dn):
        if dropped[n][0] < FLOOR:
            ax.text(FLOOR * 1.3, i, "< 0.01 %", fontsize=7,
                    va="center", color="#555")
    ax.axvline(UNCERTAINTY_FLOOR, color="#111", ls="--", lw=1.4,
               label=f"파라미터 불확실성 {UNCERTAINTY_FLOOR:.0%}")
    ax.set_yticks(range(len(dn)))
    ax.set_yticklabels([n[:26] for n in dn], fontsize=7.5)
    ax.invert_yaxis()
    # 로그 축에 눈금 라벨을 직접 쓴다. 기본 포매터는 10^{-4} 를 mathtext 로
    # 그리는데, 그 글꼴에 U+2212(마이너스)가 없어 '10¤4' 처럼 깨진 기호가
    # 찍혔다. 퍼센트로 쓰면 읽기도 쉽고 깨질 글자도 없다.
    ax.set_xscale("log")
    ax.set_xlim(1e-4, 2.0)
    ticks = [1e-4, 1e-3, 1e-2, 1e-1, 1.0]
    ax.set_xticks(ticks)
    ax.set_xticklabels(["0.01 %", "0.1 %", "1 %", "10 %", "100 %"], fontsize=8)
    ax.set_xlabel("지배항 대비 상대 크기")
    ax.set_title("모델에 없는 항")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="x")

    fig.tight_layout()
    fig.savefig(out_path("terms.png"), dpi=260)
    print("  saved out/terms.png")
