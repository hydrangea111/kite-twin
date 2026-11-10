"""What can the winch controller actually do?

Two hard constraints decide the answer, and both are checked here before any
loop is designed.

1. The JGY-370 drives through a self-locking worm gear, so the line never
   back-drives the spool. Tension is relieved only by actively paying out, and
   never faster than the spool turns -- about ten times too slow for a gust.

2. Steady-state tension is set by the kite's aerodynamic balance, not by how
   much line is out. `authority()` shows a fourfold change in line length moves
   tension by a few percent while a 50% wind change moves it threefold. A
   tension setpoint loop driven by the winch therefore has almost no DC gain
   and cannot work; an earlier PI design here failed for exactly that reason.

So the winch controls *distance*, and tension is an output to watch, not a
variable to command. Commanding tension needs the kite's angle of attack, which
on the quad-line rig means differential line lengths -- that is what the four
motors are really for, and it needs CL(alpha) from flight data before it can be
designed.

Modelling choices: the winch is a rate- and bandwidth-limited velocity source
(torque never binds, see `capability()`), and the controller is discrete with a
zero-order hold like the firmware it will become.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp

import wind
from model import aero_force, equilibrium, line_force, sag_angle, tension
from params import G, LINE_LENGTH, SILK, WIND_BASE, Kite

SPOOL_RADIUS = 0.015
GEAR_RATIO = 250.0
WORM_EFFICIENCY = 0.4
MOTOR_KT = 0.0153             # N*m/A                              MEASURE
STALL_CURRENT = 2.5           # A at 12 V                          MEASURE
MAX_RATE = 0.047              # m/s, 30 rpm on a 15 mm spool
INNER_TAU = 0.2               # s, closed velocity-loop time constant
CONTROL_HZ = 20.0

TENSION_LIMIT = 40.0          # N, above this just pay out at full speed


class AltitudeLoop:
    """Hold payload altitude by trimming line length.

    Altitude is radius * sin(elevation), and elevation is set by the wind, so
    holding altitude through a wind change means paying out or reeling in. That
    needs elevation to be measured -- without it the loop is open on the term
    that actually moves.
    """

    def __init__(self, target_alt, kp=0.05, max_rate=MAX_RATE,
                 elevation_noise=0.0, ea=None, line_weight_per_m=None,
                 rng=None):
        self.target = target_alt
        self.kp = kp
        self.max_rate = max_rate
        self.elevation_noise = elevation_noise
        self.ea = ea                              # measured tether stiffness
        self.line_weight_per_m = line_weight_per_m  # for the sag correction
        self.rng = rng or np.random.default_rng(0)
        self.protecting = False

    def update(self, length, measured_elevation, measured_tension):
        """`measured_elevation` is what a ground angle sensor reads: the cable
        tangent at the anchor, which sag makes shallower than the chord."""
        if self.elevation_noise:
            measured_elevation += self.rng.normal(0.0, self.elevation_noise)

        self.protecting = measured_tension > TENSION_LIMIT
        if self.protecting:
            return self.max_rate          # dump line, the only protection we have

        elevation = measured_elevation
        if self.line_weight_per_m and measured_tension > 1e-6:
            weight = self.line_weight_per_m * length * G
            elevation += 0.5 * weight * np.cos(elevation) / measured_tension

        # The encoder knows unstretched length; the kite sits at length+stretch.
        span = length
        if self.ea:
            span += length * measured_tension / self.ea

        sin_el = max(np.sin(elevation), 0.1)
        needed = (self.target - span * sin_el) / sin_el
        return float(np.clip(self.kp * needed, -self.max_rate, self.max_rate))



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def plant(t, y, kite, tether, wind_fn, commanded):
    r, v = y[0:2], y[2:4]
    L0, rate = y[4], y[5]

    radius = np.linalg.norm(r)
    rhat = r / radius
    T = tension(radius, v @ rhat, L0, tether, kite.mass)

    force = (
        aero_force(v, wind_fn(t, r), kite, r)
        + line_force(v, wind_fn(t, 0.5 * r), r, tether, L0)
        + np.array([0.0, -kite.mass * G])
        - T * rhat
    )
    return np.concatenate([v, force / kite.mass,
                           [rate, (commanded - rate) / INNER_TAU]])


def run(kite, tether, wind_fn, ctrl, t_end):
    base, _, _ = equilibrium(kite, tether, WIND_BASE, LINE_LENGTH)
    y = np.concatenate([base, [LINE_LENGTH, 0.0]])
    dt = 1.0 / CONTROL_HZ

    log = {k: [] for k in ("t", "tension", "length", "rate", "altitude",
                           "elevation", "protecting")}
    t = 0.0
    while t < t_end - 1e-9:
        r, v = y[0:2], y[2:4]
        radius = np.linalg.norm(r)
        rhat = r / radius
        T = tension(radius, v @ rhat, y[4], tether, kite.mass)
        elev = np.arctan2(r[1], r[0])
        # A ground-mounted sensor sees the cable tangent, not the chord.
        sensed = elev - sag_angle(T, y[4], elev, tether)

        cmd = ctrl.update(y[4], sensed, T) if ctrl else 0.0

        log["t"].append(t)
        log["tension"].append(T)
        log["length"].append(y[4])
        log["rate"].append(y[5])
        log["altitude"].append(r[1])
        log["elevation"].append(np.degrees(elev))
        log["protecting"].append(ctrl.protecting if ctrl else False)

        sol = solve_ivp(plant, (t, t + dt), y,
                        args=(kite, tether, wind_fn, cmd),
                        method="RK45", rtol=1e-6, atol=1e-8)
        if not sol.success:
            raise RuntimeError(sol.message)
        y = sol.y[:, -1]
        t += dt

    return {k: np.asarray(v) for k, v in log.items()}


def steady(speed):
    return wind.uniform(speed)


def ramp(low=6.0, high=9.0, t0=20.0, dur=120.0):
    return wind.ramp(wind.uniform(low), high / low, t0, dur)


def capability():
    stall_torque = MOTOR_KT * STALL_CURRENT * GEAR_RATIO * WORM_EFFICIENCY
    max_pull = stall_torque / SPOOL_RADIUS
    print("=" * 74)
    print("1. 윈치 능력 한계")
    print("=" * 74)
    print(f"  최대 견인력 {max_pull:.0f} N / 최대 권취속도 {MAX_RATE*100:.1f} cm/s")
    print(f"  실제 장력 5.5~33 N -> 토크 여유 {max_pull/33:.0f}배. "
          f"토크는 제약이 아닙니다.")
    extra = (33.0 - 5.5) / (SILK.EA / LINE_LENGTH)
    print(f"  돌풍 1 s에 늘어남 {extra*100:.0f} cm -> 상쇄하려면 "
          f"{extra*100:.0f} cm/s 필요, 가능한 건 {MAX_RATE*100:.1f} cm/s")
    print(f"  => 속도가 {extra/MAX_RATE:.0f}배 부족. 돌풍은 제어로 못 막습니다.")
    print()


def authority(kite):
    print("=" * 74)
    print("2. 윈치는 장력에 권한이 있는가 (핵심 확인)")
    print("=" * 74)
    print(f"  {'풍속':>6}{'라인길이':>10}{'장력':>10}{'고도각':>10}")
    for wind in (6.0, 9.0):
        for L0 in (20.0, 50.0, 80.0):
            _, T, el = equilibrium(kite, SILK, wind, L0)
            print(f"  {wind:>5.1f}{L0:>9.0f}m{T:>9.2f}N{el:>9.1f}°")
    _, t_short, _ = equilibrium(kite, SILK, 6.0, 20.0)
    _, t_long, _ = equilibrium(kite, SILK, 6.0, 80.0)
    _, t_wind, _ = equilibrium(kite, SILK, 9.0, 50.0)
    _, t_base, _ = equilibrium(kite, SILK, 6.0, 50.0)
    print()
    print(f"  라인 4배 변화 -> 장력 {100*(t_long/t_short-1):+.1f}%")
    print(f"  풍속 1.5배 변화 -> 장력 {100*(t_wind/t_base-1):+.0f}%")
    print("  => 장력은 공력 평형이 정합니다. 윈치로는 거의 못 바꿉니다.")
    print("     장력 지령 추종 루프는 성립하지 않습니다.")
    print()


if __name__ == "__main__":
    kite = Kite()
    capability()
    authority(kite)

    base, T0, elev0 = equilibrium(kite, SILK, WIND_BASE, LINE_LENGTH)
    target_alt = base[1]

    print("=" * 74)
    print("3. 윈치가 할 수 있는 것: 고도 유지 (6 -> 9 m/s)")
    print("=" * 74)
    print(f"  목표 고도 {target_alt:.1f} m "
          f"(풍속 6 m/s, 라인 50 m, 고도각 {elev0:.1f}°)")

    horizon = 450.0
    cases = [
        ("제어 없음", None),
        ("보정 없음", AltitudeLoop(target_alt)),
        ("늘어남 보정만", AltitudeLoop(target_alt, ea=SILK.EA)),
        ("늘어남+처짐 보정", AltitudeLoop(target_alt, ea=SILK.EA,
                                   line_weight_per_m=SILK.mass_per_m)),
        ("전체보정+각도잡음 1°", AltitudeLoop(
            target_alt, ea=SILK.EA, line_weight_per_m=SILK.mass_per_m,
            elevation_noise=np.radians(1.0))),
        ("전체보정+EA 40% 오차", AltitudeLoop(
            target_alt, ea=0.6 * SILK.EA,
            line_weight_per_m=SILK.mass_per_m)),
    ]
    results = {}
    for label, ctrl in cases:
        results[label] = run(kite, SILK, ramp(), ctrl, horizon)

    free = results["제어 없음"]
    held = results["늘어남+처짐 보정"]

    for label, res in results.items():
        tail = res["t"] >= horizon - 30.0
        alt = res["altitude"][tail].mean()
        print(f"  {label:<22}최종고도 {alt:>6.2f} m "
              f"(오차 {alt-target_alt:>+6.2f} m), "
              f"라인 {res['length'][-1]:.1f} m")
    print()

    low = sag_angle(T0, LINE_LENGTH, np.radians(elev0), SILK)
    print(f"  처짐각: 약풍(장력 {T0:.1f} N, 라인 {LINE_LENGTH:.0f} m)에서 "
          f"{np.degrees(low):.1f}°")
    print(f"          -> 보정 안 하면 고도 추정이 "
          f"{LINE_LENGTH*np.cos(np.radians(elev0))*low:.2f} m 틀어집니다.")
    print()
    print(f"  고도각이 {elev0:.0f}° -> {held['elevation'][-1]:.0f}°로 올라가므로,")
    print(f"  같은 고도를 유지하려면 라인을 "
          f"{LINE_LENGTH - held['length'][-1]:.1f} m 감아야 합니다.")
    print(f"  최대속도 {MAX_RATE*100:.1f} cm/s로는 "
          f"{(LINE_LENGTH-held['length'][-1])/MAX_RATE/60:.1f}분 걸립니다.")
    print()

    print("=" * 74)
    print("설계 결론")
    print("=" * 74)
    print("  * 제어 대상은 장력이 아니라 '라인 길이 -> 고도'입니다.")
    print("    내부 속도루프(엔코더) + 외부 고도루프(P, Kp=0.05, 시정수 20 s).")
    print("  * 고도루프에는 고도각 측정이 필요합니다. 라인 길이만으로는")
    print("    풍속이 바뀔 때 고도가 얼마나 변했는지 알 방법이 없습니다.")
    print("    -> 보류하신 지상국 2축 각도센서가 여기서 필수가 됩니다.")
    print("    각도 잡음 1°는 거의 영향이 없으니 정밀한 센서는 불필요합니다.")
    print("  * 고도 추정에는 보정 두 개가 필요하고, 둘 다 이미 측정 중인")
    print("    장력만으로 계산됩니다 (추가 센서 없음):")
    print("      - 늘어남: 엔코더는 늘어나기 전 길이만 압니다 (EA 실측 필요)")
    print("      - 처짐: 각도센서는 접선각을 읽지만 고도엔 현각이 필요합니다")
    print("    처짐 오차는 장력에 반비례하므로 약풍에서 최악입니다.")
    print("  * 장력은 제어 변수가 아니라 보호 변수입니다.")
    print(f"    {TENSION_LIMIT:.0f} N 초과 시 최대속도로 풀어주는 동작만 넣으세요.")
    print("    (돌풍 자체는 못 막습니다. 완충은 명주실 탄성 몫입니다.)")
    print("  * 장력/발전량을 능동 제어하려면 받음각을 바꿔야 하고,")
    print("    그게 쿼드라인 4개 모터의 진짜 용도입니다. 다만 CL(alpha)를")
    print("    모르면 설계가 불가능하니, 첫 비행 데이터가 선행 조건입니다.")
    print()

    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for label in ("제어 없음", "보정 없음", "늘어남+처짐 보정"):
        axes[0].plot(results[label]["t"], results[label]["altitude"], label=label)
    axes[0].axhline(target_alt, color="k", ls="--", lw=1, label="목표")
    axes[0].set_ylabel("탑재물 고도 [m]")
    axes[0].set_title("윈치가 제어할 수 있는 것은 고도이지 장력이 아님")

    axes[1].plot(free["t"], free["tension"], label="제어 없음")
    axes[1].plot(held["t"], held["tension"], label="고도 제어")
    axes[1].set_ylabel("장력 [N]")

    axes[2].plot(free["t"], free["length"], label="제어 없음")
    axes[2].plot(held["t"], held["length"], label="고도 제어")
    axes[2].set_ylabel("라인 길이 [m]")
    axes[2].set_xlabel("시간 [s]")

    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path("control.png"), dpi=260)
    print("  saved out/control.png")
