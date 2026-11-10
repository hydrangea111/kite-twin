"""Does tether elasticity change how the rig behaves?

Runs the same three disturbances against silk, Dyneema and a near-rigid
reference, then reports the numbers that constrain the winch controller:
longitudinal mode frequency, peak tension under gust, and the gap between
commanded line length (what the encoder sees) and true distance to the kite.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

import wind
from model import effective_mass, equilibrium, min_area_for_elevation, simulate
from params import LINE_LENGTH, TETHERS, WIND_BASE, Kite

SPOOL_RATE = 0.047  # m/s, 30 rpm on a 15 mm spool radius



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def gust_wind(base=WIND_BASE, peak=10.0, t0=2.0, dur=1.5):
    return wind.gust(wind.uniform(base), peak / base, t0, dur)


def lull_wind(base=WIND_BASE, low=1.5, t0=2.0, ramp=0.5):
    return wind.lull(wind.uniform(base), low / base, t0, ramp)


def steady_wind(base=WIND_BASE):
    return wind.uniform(base)


def fixed_length(length=LINE_LENGTH):
    return lambda t: length


def reel_in(length=LINE_LENGTH, t0=2.0, rate=SPOOL_RATE, total=1.5):
    def cmd(t):
        if t < t0:
            return length
        return length - min(rate * (t - t0), total)
    return cmd


def chatter(freq, length=LINE_LENGTH, t0=2.0, rate=SPOOL_RATE):
    """Worst case a bang-bang controller could do: drive the length command at
    the tether's own bounce frequency, amplitude capped by the spool's top speed."""
    amp = rate / (2 * np.pi * freq)

    def cmd(t):
        if t < t0:
            return length
        return length + amp * np.sin(2 * np.pi * freq * (t - t0))
    return cmd


def scenarios_for(tether, kite):
    fn = tether.natural_freq(LINE_LENGTH, effective_mass(kite))
    return {
        "gust": dict(wind=gust_wind(), length=fixed_length(), t_end=20.0,
                     title="돌풍 응답 (6 -> 10 m/s, 1.5 s)"),
        "reel": dict(wind=steady_wind(), length=reel_in(), t_end=50.0,
                     title="라인 감기 (1.5 m @ 4.7 cm/s)"),
        "lull": dict(wind=lull_wind(), length=fixed_length(), t_end=15.0,
                     title="바람 급감 (6 -> 1.5 m/s)"),
        "chatter": dict(wind=steady_wind(), length=chatter(fn), t_end=20.0,
                        title="제어기 채터링 (각 줄의 고유진동수로 최대속도 가진)"),
    }


SCENARIO_NAMES = ("gust", "reel", "lull", "chatter")


def flight_check(kite):
    print("=" * 74)
    print("비행 가능성 (sizing check)")
    print("=" * 74)
    print(f"  총 질량      : {kite.mass*1000:.0f} g "
          f"(캐노피 {kite.mass_canopy*1000:.0f} + 탑재물 {kite.mass_payload*1000:.0f})")
    print(f"  현재 면적    : {kite.area:.2f} m^2")
    print(f"  최대 고도각  : {np.degrees(np.arctan2(kite.cl, kite.cd)):.0f}° "
          f"(L/D = {kite.cl/kite.cd:.1f}, 면적 무한대 기준 상한)")
    for angle in (30, 45):
        need = min_area_for_elevation(kite, WIND_BASE, angle)
        verdict = "불가 (L/D 한계)" if np.isinf(need) else f"{need:.2f} m^2 필요"
        flag = "" if np.isinf(need) else ("  <-- 부족" if need > kite.area else "  OK")
        print(f"  고도각 {angle}° @ {WIND_BASE} m/s : {verdict}{flag}")
    print()


def run(kite):
    results = {name: {} for name in SCENARIO_NAMES}
    rows = []

    for tether in TETHERS:
        state0, T0, elev0 = equilibrium(kite, tether, WIND_BASE, LINE_LENGTH)
        stretch0 = np.linalg.norm(state0[:2]) - LINE_LENGTH
        fn = tether.natural_freq(LINE_LENGTH, effective_mass(kite))
        scen = scenarios_for(tether, kite)

        for name in SCENARIO_NAMES:
            cfg = scen[name]
            results[name][tether.name] = simulate(
                kite, tether, cfg["wind"], cfg["length"], state0, cfg["t_end"]
            )

        gust = results["gust"][tether.name]
        reel = results["reel"][tether.name]
        lull = results["lull"][tether.name]
        chat = results["chatter"][tether.name]

        peak_T = gust["tension"].max()
        reel_err = np.abs(reel["radius"] - reel["commanded"]).max()
        dt = lull["t"][1] - lull["t"][0]
        slack_s = float((lull["tension"] <= 1e-9).sum() * dt)

        # Steady-state ripple once the chatter transient has settled.
        late = chat["t"] >= 10.0
        ripple = (chat["tension"][late].max() - chat["tension"][late].min()) / 2

        rows.append({
            "name": tether.name,
            "elev0": elev0,
            "T0": T0,
            "stretch0": stretch0,
            "fn": fn,
            "peak_T": peak_T,
            "peak_ratio": peak_T / T0,
            "margin": tether.break_load / peak_T,
            "reel_err": reel_err,
            "slack_s": slack_s,
            "ripple": ripple,
            "ripple_ratio": ripple / T0,
        })

    return results, rows


def damping_budget(kite=None):
    """Why the spring and the silk behave the same despite 15x the damping.

    Compares the tether's internal damping against the aerodynamic damping the
    kite already has. Moving through the air changes the apparent wind, which
    changes the force -- that is a damper, and it turns out to be the only one
    that matters here.
    """
    from params import RHO_AIR, SILK, SPRUNG

    kite = kite or Kite()
    mass, L0 = kite.mass, LINE_LENGTH
    k = SILK.EA / L0
    print("=" * 74)
    print("감쇠 예산 — 줄의 감쇠는 왜 거의 영향이 없는가")
    print("=" * 74)
    print(f"  {'출처':<28}{'감쇠계수 [N·s/m]':>18}")
    for tether in (SILK, SPRUNG):
        c = 2 * tether.damping_ratio * np.sqrt(k * mass)
        print(f"  {tether.name + ' 내부감쇠':<28}{c:>18.3f}")

    # The kite's own drag: moving at v changes apparent wind by v, and the
    # aerodynamic force with it.
    coeff = np.hypot(kite.cl, kite.cd)
    c_aero = RHO_AIR * WIND_BASE * kite.area * coeff
    print(f"  {'연의 공력 감쇠':<28}{c_aero:>18.3f}")
    print()
    print(f"  => 공력 감쇠가 명주실 내부감쇠의 "
          f"{c_aero / (2 * SILK.damping_ratio * np.sqrt(k * mass)):.0f}배, "
          f"용수철의 {c_aero / (2 * SPRUNG.damping_ratio * np.sqrt(k * mass)):.0f}배입니다.")
    print(f"     줄의 재질 감쇠는 반올림 오차 수준이라, 15배를 바꿔도 결과가")
    print(f"     움직이지 않습니다. 강성만이 변수입니다.")
    print()


def report(rows):
    print("=" * 74)
    print("정상 비행 상태 + 줄 특성")
    print("=" * 74)
    print(f"  {'줄':<22}{'고도각':>8}{'장력':>9}{'늘어남':>10}{'종방향 고유진동':>16}")
    for r in rows:
        print(f"  {r['name']:<22}{r['elev0']:>7.1f}°{r['T0']:>8.1f}N"
              f"{r['stretch0']*100:>9.1f}cm{r['fn']:>14.2f}Hz")
    print()

    print("=" * 74)
    print("돌풍 응답 (6 -> 10 m/s)")
    print("=" * 74)
    print(f"  {'줄':<22}{'최대장력':>10}{'정상대비':>10}{'파단여유':>10}")
    for r in rows:
        print(f"  {r['name']:<22}{r['peak_T']:>9.1f}N{r['peak_ratio']:>9.1f}x"
              f"{r['margin']:>9.1f}x")
    print()

    print("=" * 74)
    print("제어 영향")
    print("=" * 74)
    print(f"  {'줄':<22}{'엔코더 길이오차':>18}{'슬랙 지속':>12}{'채터 장력맥동':>16}")
    for r in rows:
        print(f"  {r['name']:<22}{r['reel_err']*100:>16.1f}cm"
              f"{r['slack_s']:>11.1f}s"
              f"{r['ripple']:>13.2f}N ({r['ripple_ratio']*100:.0f}%)")
    print()

    damping_budget()

    silk, rigid = rows[0], rows[-1]
    print("=" * 74)
    print("해석")
    print("=" * 74)
    print(f"  1. 길이 추정 편향: 감기 중 엔코더 길이와 실제 거리가 최대 "
          f"{silk['reel_err']*100:.1f} cm 차이")
    print(f"     (다이니마 {rows[1]['reel_err']*100:.1f} cm, 강체 "
          f"{rigid['reel_err']*100:.1f} cm). 돌풍 때는 장력이 뛰면서 이 값이 "
          f"수십 cm까지")
    print(f"     벌어집니다 -> 라인 길이만으로 고도를 추정하면 이만큼이 그대로 오차.")
    print()
    print(f"  2. 돌풍 충격: 명주실 최대장력이 강체 대비 "
          f"{silk['peak_T']/rigid['peak_T']:.2f}배")
    print(f"     -> 탄성이 오히려 유리합니다. 줄이 완충기 역할을 해서 연 구조체와")
    print(f"        매듭에 걸리는 피크 하중을 낮춰줍니다.")
    print()
    print(f"  3. 제어 채터링: 스풀 최고속도({SPOOL_RATE*100:.1f} cm/s)로 각 줄의")
    print(f"     고유진동수를 가진했을 때 장력 맥동이 정상장력 대비")
    print(f"     명주실 {silk['ripple_ratio']*100:.0f}% / "
          f"다이니마 {rows[1]['ripple_ratio']*100:.0f}% / "
          f"강체 {rigid['ripple_ratio']*100:.0f}%.")
    print(f"     공진 자체는 세 경우 다 일어나지만, 뻣뻣할수록 같은 길이 지령 떨림이")
    print(f"     훨씬 큰 장력 변동으로 증폭됩니다. 명주실의 유연성이 제어기 떨림을")
    print(f"     흡수하는 쪽으로 작동합니다.")
    print()
    print(f"  => 결론: 명주실의 실질적 페널티는 길이 추정 편향 하나뿐이고,")
    print(f"     돌풍 충격과 제어 채터링에서는 오히려 유리합니다. 길이 편향은")
    print(f"     IMU/GPS로 고도를 직접 재서 보완하는 게 맞는 방향입니다.")
    print()


# Colour belongs to the tether, not to the drawing order. When the order was
# changed below to stop the stiffest trace painting over the others, every
# colour shifted and the report's caption ("the rigid one is red") silently
# became wrong -- for the second time. Pinning them here lets the captions
# outlive any future reordering.
TETHER_COLOUR = {
    "명주실 (silk)": "#1f77b4",
    "강체줄 + 용수철": "#ff7f0e",
    "다이니마 (UHMWPE)": "#2ca02c",
    "강체 근사 (rigid ref)": "#d62728",
}


def plot(results, titles):
    for name in SCENARIO_NAMES:
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

        # The stiffest tether oscillates hardest, so drawn last it paints over
        # the three the reader is supposed to compare it against -- the whole
        # point of the figure. Draw the violent ones first and thin, so the
        # quiet traces stay visible on top of them.
        order = sorted(results[name].items(),
                       key=lambda kv: -float(np.ptp(kv[1]["tension"])))
        for i, (tether_name, res) in enumerate(order):
            width = dict(lw=0.8, alpha=0.6) if i == 0 else dict(lw=1.3)
            style = dict(label=tether_name,
                         color=TETHER_COLOUR.get(tether_name), **width)
            axes[0].plot(res["t"], res["tension"], **style)
            axes[1].plot(res["t"], res["radius"] - res["commanded"], **style)
            axes[2].plot(res["t"], res["elevation"], **style)

        axes[0].set_ylabel("장력 [N]")
        axes[1].set_ylabel("실제거리 - 지령길이 [m]")
        axes[2].set_ylabel("고도각 [deg]")
        axes[2].set_xlabel("시간 [s]")
        axes[0].set_title(titles[name])
        for ax in axes:
            ax.grid(alpha=0.3)
        # Outside the axes: the legend sat on top of the first seconds of data.
        axes[0].legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8.5)

        fig.tight_layout()
        path = out_path(f"{name}.png")
        fig.savefig(path, dpi=260)
        plt.close(fig)
        print(f"  saved {path}")


if __name__ == "__main__":
    kite = Kite()
    flight_check(kite)
    results, rows = run(kite)
    report(rows)
    titles = {n: cfg["title"] for n, cfg in scenarios_for(TETHERS[0], kite).items()}
    plot(results, titles)
