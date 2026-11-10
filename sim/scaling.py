"""Dimensional analysis, and a similarity test that the other checks cannot do.

Every validation so far compares the model to something: a closed-form
equilibrium, an energy budget, a hand-derived cable reaction. Those catch a
wrong formula. They do not catch a formula that is right in one place and
inconsistently scaled in another, because each check is run at one size.

A similarity test does. Write the governing equations in dimensionless form
and the system is described by a handful of Pi groups. Two physically
different rigs -- different span, different wind, different mass -- that share
those groups must produce *identical* dimensionless results. If any term in
the force balance carries the wrong power of length, mass or velocity, the two
runs separate, and the size of the separation points at which term is wrong.

This is also the theoretical frame the project has been missing. The report so
far quotes results at one set of numbers; the Pi groups say which of those
numbers actually matter and which are coordinates.

    python scaling.py
"""

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
from params import G, LINE_LENGTH, RHO_AIR, SILK, WIND_BASE, Kite

# --------------------------------------------------------------------------
# the Pi groups
# --------------------------------------------------------------------------
#
# Variables: m, g, rho, A, v, L, EA, w (tether mass per length), d, C_L, C_D.
# Three dimensions (M, L, T) over eleven variables leaves eight groups, three
# of which are already dimensionless (C_L, C_D, and the damping ratio).
#
#   Pi_W = rho A v^2 / (m g)    aerodynamic force over weight. The one that
#                               decides whether the kite flies at all.
#   Pi_E = EA / (rho A v^2)     tether stiffness over dynamic pressure. Decides
#                               stretch, bounce frequency and snap behaviour.
#   Pi_m = w L / m              tether weight over kite weight.
#   Pi_d = d L / A              tether drag area over wing area.
#   Fr   = v / sqrt(g L)        Froude number.
#   Pi_A = sqrt(A) / L          wing size over tether length.
#
# Pi_A was the second omission, and the test suite found it the same way the
# first one was found. Adding the canopy's added mass to the dynamics broke
# the reduced-frequency algebra under a *Froude-mismatched* transform while
# leaving the matched case untouched. The reason is that
#
#   m_added / m = (8 / 3 pi^1.5) * Pi_W * Pi_A / Fr^2
#
# so added mass introduces no new group -- but it does depend on Pi_A, which
# the transform happened to hold (A ~ lam^2, L ~ lam) without it ever being
# named. A group that is conserved by accident is not part of the analysis;
# it is a coincidence waiting to stop being true.
#
# The Froude number was missing from the first version of this analysis, and
# the similarity test below is what found it. Holding only the first four
# groups, four of the five dimensionless outputs collapsed to 1e-14 while the
# reduced frequency moved by 36%. The reason is algebraic:
#
#   f L / v = (1 / 2 pi) sqrt(Pi_E * Pi_W) / Fr
#
# so a frequency cannot be scale-invariant unless Fr is held too. Gravity is
# fixed in any ground experiment, which makes Fr a constraint rather than a
# free parameter: matching it forces
#
#   mu = sqrt(lam),   i.e.  v ~ sqrt(L)
#
# A kite test cannot be scaled at whatever wind happens to be available. The
# size of the rig dictates the wind speed that is dynamically similar to it --
# which is a statement about the indoor programme, not only about this script
# (see the note printed at the end).



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def pi_groups(kite, tether, length, speed):
    q_area = RHO_AIR * kite.area * speed**2
    return {
        "Pi_W": q_area / (kite.mass * G),
        "Pi_E": tether.EA / q_area,
        "Pi_m": tether.mass_per_m * length / kite.mass,
        "Pi_d": tether.diameter * length / kite.area,
        "Fr": speed / np.sqrt(G * length),
        "Pi_A": np.sqrt(kite.area) / length,
        "C_L": kite.cl,
        "C_D": kite.cd,
        "zeta": tether.damping_ratio,
    }


def froude_mu(lam):
    """The only speed ratio that keeps Fr = v/sqrt(gL) fixed."""
    return float(np.sqrt(lam))


def rescale(kite, tether, length, speed, lam, mu=None):
    """A similarity transform: lengths by `lam`, speeds by `mu`.

    `mu` defaults to sqrt(lam), the Froude-matched value. Passing something
    else deliberately breaks Fr, which is how the incompleteness of the
    four-group set is demonstrated below.

    With g fixed, holding the Pi groups constant forces
        A ~ lam^2,  m ~ lam^2 mu^2,  EA ~ lam^2 mu^2,  w ~ lam mu^2,  d ~ lam.
    Nothing here is a free choice; it falls out of the groups.
    """
    if mu is None:
        mu = froude_mu(lam)
    f = lam * lam * mu * mu
    k2 = replace(
        kite,
        area=kite.area * lam**2,
        mass_canopy=kite.mass_canopy * f,
        mass_payload=kite.mass_payload * f,
    )
    t2 = replace(
        tether,
        EA=tether.EA * f,
        mass_per_m=tether.mass_per_m * lam * mu**2,
        diameter=tether.diameter * lam,
    )
    return k2, t2, length * lam, speed * mu


def dimensionless_state(kite, tether, length, speed):
    """Steady flight, reported in groups that must be scale-invariant."""
    state, tension, elev = equilibrium(kite, tether, speed, length)
    radius = float(np.hypot(state[0], state[1]))
    return {
        "고도각 θ [deg]": float(elev),
        "장력비 T/mg": float(tension / (kite.mass * G)),
        "늘어남 (r−L)/L": float((radius - length) / length),
        "고도비 z/L": float(state[1] / length),
        "환산진동수 f·L/v": float(
            tether.natural_freq(length, effective_mass(kite)) * length / speed),
    }


def gust_response(kite, tether, length, speed, ratio=10.0 / 6.0):
    """Peak tension under a gust, as a fraction of the steady value."""
    field = wind.gust(wind.uniform(speed), ratio, 2.0, 1.5 * length / speed
                      * (WIND_BASE / LINE_LENGTH))
    state, steady, elev = equilibrium(kite, tether, speed, length)
    if not np.isfinite(elev) or elev < 5.0:
        return float("nan")
    # Time is scaled too: the gust lasts the same number of L/v units.
    t_end = 10.0 * length / speed
    res = simulate(kite, tether, field, lambda t: length, state, t_end,
                   n_out=2000, max_step=0.05 * length / speed)
    return float(res["tension"].max() / steady)


def collapse_table(cases):
    rows = []
    for label, (k, t, L, v) in cases.items():
        d = dimensionless_state(k, t, L, v)
        d["케이스"] = label
        rows.append(d)
    return rows


if __name__ == "__main__":
    kite, tether = Kite(), SILK
    base = (kite, tether, LINE_LENGTH, WIND_BASE)

    print("=" * 78)
    print("1. 기준 형상의 무차원 수")
    print("=" * 78)
    pis = pi_groups(*base)
    for name, value in pis.items():
        print(f"  {name:<8}{value:>12.4f}")
    print()
    print("  Pi_W 는 공기력 대 중력이다. 이 값이 작으면 어떤 조작으로도 날지 않는다.")
    print("  Pi_E 는 강성 대 동압이다. 늘어남·고유진동수·스냅이 전부 여기서 나온다.")
    print("  Pi_m, Pi_d 는 줄의 무게와 항력이 연에 비해 얼마나 되는지를 말한다.")
    print()

    # ── 상사 시험 ────────────────────────────────────────────────────
    print("=" * 78)
    print("2. 상사 시험 — 물리적으로 다른 세 장비가 같은 무차원 결과를 내는가")
    print("=" * 78)
    cases = {"기준 (50 m, 6.0 m/s)": base}
    for lam in (0.2, 2.0):
        k2, t2, L2, v2 = rescale(*base, lam)
        cases[f"{'축소' if lam < 1 else '확대'} ({L2:.0f} m, {v2:.1f} m/s)"] =             (k2, t2, L2, v2)

    print(f"  {'케이스':<22}{'면적':>8}{'질량':>8}{'EA':>9}{'길이':>7}{'풍속':>7}")
    for label, (k, t, L, v) in cases.items():
        print(f"  {label:<22}{k.area:>7.3f}㎡{k.mass:>7.3f}kg{t.EA:>8.0f}N"
              f"{L:>6.0f}m{v:>6.1f}")
    print()

    # 무차원 수가 정말 같은지부터 확인한다
    ref = pi_groups(*base)
    worst_pi = 0.0
    for label, args in cases.items():
        p = pi_groups(*args)
        worst_pi = max(worst_pi, max(abs(p[k] / ref[k] - 1) for k in ref))
    print(f"  무차원 수 일치: 최대 상대오차 {worst_pi:.2e}")
    print()

    rows = collapse_table(cases)
    keys = [k for k in rows[0] if k != "케이스"]
    header = "  " + f"{'케이스':<22}" + "".join(f"{k:>18}" for k in keys)
    print(header)
    for r in rows:
        print("  " + f"{r['케이스']:<22}" + "".join(f"{r[k]:>18.6f}" for k in keys))
    print()

    worst = {}
    for k in keys:
        vals = [r[k] for r in rows]
        scale = max(abs(v) for v in vals) or 1.0
        worst[k] = (max(vals) - min(vals)) / scale
    print("  무차원 출력의 상대 흩어짐:")
    for k, v in worst.items():
        verdict = "일치" if v < 1e-6 else ("근사" if v < 1e-3 else "불일치")
        print(f"    {k:<20}{v:>12.2e}   {verdict}")
    print()
    if max(worst.values()) < 1e-6:
        print("  -> 세 장비가 같은 무차원 상태를 낸다. 힘 평형의 모든 항이")
        print("     길이·질량·속도에 대해 일관되게 스케일한다는 뜻이다.")
        print("     한 가지 크기에서만 돌리는 검사로는 확인할 수 없는 성질이다.")
    else:
        print("  -> 흩어진다. 어떤 항의 차원이 맞지 않는다는 뜻이다.")
    print()

    # ── 변이: 일부러 차원을 틀리게 만들면 드러나는가 ───────────────────
    print("=" * 78)
    print("3. 변이 시험 — 상사를 깨면 실제로 드러나는가")
    print("=" * 78)
    print("  (a) 프루드 수를 맞추지 않은 경우 — 이 분석의 첫 판이 그랬다.")
    broken = {"기준": base}
    for lam, mu in ((0.2, 0.7), (2.0, 1.5)):
        broken[f"λ={lam}, μ={mu}"] = rescale(*base, lam, mu)
    brows = collapse_table(broken)
    spread = {}
    for k in keys:
        vals = [r[k] for r in brows]
        scale = max(abs(v) for v in vals) or 1.0
        spread[k] = (max(vals) - min(vals)) / scale
    for k, v in spread.items():
        print(f"    {k:<20}{v:>12.2e}")
    detected = max(spread.values()) > 1e-4
    print(f"  -> {'검출됨' if detected else '검출 실패'}: 환산진동수만 어긋난다.")
    print("     이것이 프루드 수가 빠졌다는 신호였다. 다른 출력은 Fr 을 포함하지")
    print("     않으므로 네 군만으로도 모이고, 진동수만 모이지 않는다.")
    print()

    print("  (b) 줄 지름만 상사에서 벗어나게(λ 대신 λ^1.5) 둔 경우.")
    broken2 = {"기준": base}
    for lam in (0.2, 2.0):
        k2, t2, L2, v2 = rescale(*base, lam)
        broken2[f"λ={lam}"] = (k2, replace(t2, diameter=tether.diameter * lam**1.5),
                               L2, v2)
    b2rows = collapse_table(broken2)
    spread2 = {}
    for k in keys:
        vals = [r[k] for r in b2rows]
        scale = max(abs(v) for v in vals) or 1.0
        spread2[k] = (max(vals) - min(vals)) / scale
    for k, v in spread2.items():
        print(f"    {k:<20}{v:>12.2e}")
    d2 = max(spread2.values()) > 1e-4
    print(f"  -> {'검출됨' if d2 else '검출 실패'}. 상사 시험이 차원 오류에 반응한다"
          f"{'' if d2 else '고 보기 어렵다'}.")
    print()

    # ── Pi_W 하나로 체공이 결정되는가 ──────────────────────────────────
    print("=" * 78)
    print("4. Pi_W 가 체공을 결정하는가 (면적·질량·풍속을 따로 바꿔도 같은가)")
    print("=" * 78)
    pts = []
    for area in (0.35, 0.5, 0.67, 0.9):
        for speed in (4.0, 5.0, 6.0, 8.0, 10.0):
            for mass_scale in (0.7, 1.0, 1.4):
                k = replace(kite, area=area,
                            mass_canopy=kite.mass_canopy * mass_scale,
                            mass_payload=kite.mass_payload * mass_scale)
                p = pi_groups(k, tether, LINE_LENGTH, speed)
                try:
                    _, _, elev = equilibrium(k, tether, speed, LINE_LENGTH)
                except Exception:                                  # noqa: BLE001
                    continue
                pts.append((p["Pi_W"], float(elev), area, speed))
    pts.sort()
    pw = np.array([p[0] for p in pts])
    el = np.array([p[1] for p in pts])

    flying = el > 5.0
    if flying.any() and (~flying).any():
        print(f"  체공한 표본의 Pi_W 최소: {pw[flying].min():.3f}")
        print(f"  체공 못 한 표본의 Pi_W 최대: {pw[~flying].max():.3f}")
        overlap = pw[flying].min() <= pw[~flying].max()
        print(f"  구간 겹침: {'있음' if overlap else '없음'}")
        print()
        if not overlap:
            print("  -> Pi_W 하나가 체공 여부를 가른다. 면적·질량·풍속을 어떻게")
            print("     조합하든 이 수 하나로 설명된다는 뜻이고, 설계 변수가")
            print("     세 개가 아니라 사실상 하나라는 말이다.")
        else:
            print("  -> 겹친다. Pi_W 만으로는 설명되지 않는 부분이 있다.")
    print()

    # ── 그림 ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.4))

    ax = axes[0]
    areas = sorted({p[2] for p in pts})
    for a in areas:
        sel = [(p[0], p[1]) for p in pts if p[2] == a]
        sel.sort()
        ax.plot([x for x, _ in sel], [y for _, y in sel], "o-", ms=3.5,
                lw=1, label=f"{a} ㎡")
    ax.axhline(5.0, color="#d62728", ls="--", lw=1.2, label="체공 한계 5°")
    ax.set_xlabel("Π_W = ρAv² / mg")
    ax.set_ylabel("고도각 [deg]")
    ax.set_title("면적·질량·풍속이 달라도 Π_W 하나로 모인다")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    labels = [r["케이스"] for r in rows]
    x = np.arange(len(keys))
    width = 0.26
    for i, r in enumerate(rows):
        vals = [r[k] for k in keys]
        ax.bar(x + (i - 1) * width, vals, width, label=r["케이스"])
    ax.set_xticks(x)
    ax.set_xticklabels([k.split()[0] for k in keys], fontsize=8.5)
    ax.set_title("상사 시험: 세 장비의 무차원 출력")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    print("=" * 78)
    print("5. 실내 시험에 대한 함의")
    print("=" * 78)
    print("  프루드 정합이 요구하는 것은 v ~ √L 이다. 실내에서 라인을 짧게 쓰면")
    print("  바람도 그만큼 줄여야 야외 비행과 역학적으로 같은 상태가 된다.")
    print()
    print(f"  {'실내 라인':>10}{'상사 풍속':>12}{'(야외 50 m, 6 m/s 기준)':>26}")
    for L in (2.0, 3.0, 5.0, 10.0, 20.0):
        lam = L / LINE_LENGTH
        print(f"  {L:>9.0f}m{WIND_BASE * froude_mu(lam):>11.2f} m/s")
    print()
    print("  라인 3 m 짜리 실내 시험에서 6 m/s 를 불면 야외와 같은 상태가 아니다.")
    print("  같은 Π_W 를 만들 수는 있어도 Fr 이 4배 어긋나므로, 정상상태 고도각과")
    print("  장력비는 맞지만 진동·과도 거동은 맞지 않는다. 실내에서 공력계수를")
    print("  뽑는 것(17.2절)은 정상상태 값만 쓰므로 유효하고, 거기서 제어 응답이나")
    print("  스냅 하중을 재서 야외로 옮기는 것은 유효하지 않다.")
    print()

    fig.tight_layout()
    fig.savefig(out_path("scaling.png"), dpi=260)
    print("  saved out/scaling.png")
