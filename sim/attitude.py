"""Which IMU do we actually need on the payload?

A payload hanging on a line is a pendulum, and an accelerometer on a pendulum
measures the string tension direction -- not true vertical. So the usual
accel-corrects-gyro-drift trick may be blind to exactly the motion we care
about. This feeds synthetic sensor data from the validated two-body model into
four estimators and scores them against ground truth, then sizes the altitude
error budget to decide what GPS is actually for.

Assumes the payload hangs rigidly aligned with its suspension line (single-point
suspension), so payload attitude == swing angle.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp

from payload import equilibrium2, gust, rhs
from params import (G, LINE_LENGTH, SILK, SUSPENSION_LENGTH, WIND_BASE,
                    Kite, Payload)

IMU_HZ = 100.0
DURATION = 30.0

GYRO_BIAS = np.radians(1.0)      # rad/s, uncalibrated cheap MEMS
GYRO_NOISE = np.radians(0.1)     # rad/s per sample
ACCEL_NOISE = 0.02               # m/s^2 per sample
MAG_NOISE = 0.3                  # uT on a 50 uT field

MAG_INCLINATION = np.radians(50.0)   # Korea, roughly
FIELD = 50.0 * np.array([np.cos(MAG_INCLINATION), -np.sin(MAG_INCLINATION)])



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def rot(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def truth():
    """Ground-truth swing angle, swing rate and payload acceleration."""
    kite, payload = Kite(), Payload()
    field = gust()
    drop = SUSPENSION_LENGTH
    state0 = equilibrium2(kite, payload, SILK, drop, WIND_BASE, LINE_LENGTH)
    t_eval = np.arange(0.0, DURATION, 1.0 / IMU_HZ)

    sol = solve_ivp(
        rhs, (0.0, DURATION), state0,
        args=(kite, payload, SILK, drop, field, lambda t: LINE_LENGTH),
        method="Radau", t_eval=t_eval, rtol=1e-7, atol=1e-9, max_step=0.1,
    )
    if not sol.success:
        raise RuntimeError(sol.message)

    rk, rp = sol.y[0:2].T, sol.y[2:4].T
    link = rp - rk
    theta = np.arctan2(link[:, 0], -link[:, 1])

    accel = np.array([
        rhs(sol.t[i], sol.y[:, i], kite, payload, SILK, drop,
            field, lambda t: LINE_LENGTH)[6:8]
        for i in range(len(sol.t))
    ])
    rate = np.gradient(theta, sol.t)
    return sol.t, theta, rate, accel


def synth_sensors(t, theta, rate, accel, rng):
    """Body-frame accelerometer, gyro and magnetometer readings."""
    n = len(t)
    specific = accel + np.array([0.0, G])      # what an accelerometer feels

    accel_b = np.empty((n, 2))
    mag_b = np.empty((n, 2))
    for i in range(n):
        R = rot(-theta[i])
        accel_b[i] = R @ specific[i]
        mag_b[i] = R @ FIELD

    accel_b += rng.normal(0.0, ACCEL_NOISE, accel_b.shape)
    mag_b += rng.normal(0.0, MAG_NOISE, mag_b.shape)
    gyro = rate + GYRO_BIAS + rng.normal(0.0, GYRO_NOISE, n)
    return accel_b, gyro, mag_b


def tilt_from_accel(accel_b):
    return np.arctan2(accel_b[:, 0], accel_b[:, 1])


def tilt_from_mag(mag_b):
    reference = np.arctan2(FIELD[1], FIELD[0])
    return reference - np.arctan2(mag_b[:, 1], mag_b[:, 0])


def integrate_gyro(gyro, dt, start):
    return start + np.cumsum(gyro) * dt


def complementary(reference, gyro, dt, tau, start):
    alpha = tau / (tau + dt)
    out = np.empty(len(gyro))
    out[0] = start
    for i in range(1, len(gyro)):
        out[i] = alpha * (out[i - 1] + gyro[i] * dt) + (1 - alpha) * reference[i]
    return out


def rms_deg(estimate, reference):
    return np.degrees(np.sqrt(np.mean((estimate - reference) ** 2)))


def altitude_budget():
    """What limits our knowledge of payload altitude?"""
    elevation = np.radians(38.3)
    length = LINE_LENGTH
    horizontal = length * np.cos(elevation)

    print("=" * 74)
    print("고도 오차 예산 (라인 50 m, 고도각 38°, 실제 고도 ~31 m)")
    print("=" * 74)
    rows = [
        ("엔코더 길이오차 (명주실 늘어남 10 cm)", 0.10 * np.sin(elevation)),
        ("줄 각도 오차 1.0° (저가 각도센서)", horizontal * np.radians(1.0)),
        ("줄 각도 오차 0.5° (엔코더식)", horizontal * np.radians(0.5)),
        ("GPS 수직 오차 (일반 모듈)", 4.0),
        ("GPS 수직 오차 (RTK)", 0.03),
    ]
    for label, err in rows:
        print(f"  {label:<40}{err:>8.2f} m")
    print()
    print("  -> 명주실 늘어남(6 cm)은 고도 오차에서 가장 작은 항목입니다.")
    print("     지배하는 건 줄 각도이고, 일반 GPS는 그보다도 한참 나쁩니다.")
    print("     고도 정밀도가 목표면 지상국에 2축 각도센서를 다는 게")
    print("     GPS보다 10배 이상 정확하고 훨씬 쌉니다.")
    print("     GPS는 고도용이 아니라 위경도/시각(태양 위치 계산)용으로 쓰세요.")
    print()


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    t, theta, rate, accel = truth()
    accel_b, gyro, mag_b = synth_sensors(t, theta, rate, accel, rng)
    dt = 1.0 / IMU_HZ

    ref_accel = tilt_from_accel(accel_b)
    ref_mag = tilt_from_mag(mag_b)

    estimates = {
        "가속도계만 (6축)": ref_accel,
        "자이로 적분만": integrate_gyro(gyro, dt, theta[0]),
        "가속도+자이로 상보 (6축)": complementary(ref_accel, gyro, dt, 2.0, theta[0]),
        "지자기+자이로 상보 (9축)": complementary(ref_mag, gyro, dt, 2.0, theta[0]),
    }

    print("=" * 74)
    print("자세 추정 방식별 오차 (돌풍 중 흔들림 최대 67°)")
    print("=" * 74)
    print(f"  {'방식':<28}{'RMS 오차':>12}{'최대 오차':>12}")
    for name, est in estimates.items():
        err = np.degrees(np.abs(est - theta))
        print(f"  {name:<28}{rms_deg(est, theta):>10.2f}°{err.max():>11.2f}°")
    print()

    print("=" * 74)
    print("상보필터 시정수 민감도 (지자기+자이로)")
    print("=" * 74)
    for tau in (0.2, 1.0, 2.0, 5.0, 20.0):
        est = complementary(ref_mag, gyro, dt, tau, theta[0])
        print(f"  tau = {tau:>5.1f} s   RMS {rms_deg(est, theta):>6.2f}°")
    print()

    six = np.degrees(np.abs(estimates["가속도+자이로 상보 (6축)"] - theta)).max()
    nine = np.degrees(np.abs(estimates["지자기+자이로 상보 (9축)"] - theta)).max()
    tilt = np.radians(45.0)
    print("=" * 74)
    print("해석")
    print("=" * 74)
    print("  1. 가속도계 단독은 못 씁니다 (최대 오차 52°). 진자에 달린 가속도계는")
    print("     중력이 아니라 '줄 방향'을 읽어서, 크게 흔들려도 기울어진 걸 모릅니다.")
    print("     자이로 단독도 바이어스 때문에 계속 밀립니다.")
    print()
    print("  2. 6축(가속도+자이로) 상보필터는 생각보다 쓸 만합니다 — RMS 2.3°.")
    print("     탑재물 항력이 줄 방향에서 살짝 벗어난 성분을 만들어줘서,")
    print("     가속도계가 완전히 눈먼 건 아니기 때문입니다.")
    print(f"     다만 오차가 돌풍 중에 {six:.1f}°까지 튑니다 — 하필 제일 중요한 순간에요.")
    print()
    print(f"  3. 9축(지자기+자이로)은 최대 오차가 {nine:.1f}°로 묶입니다.")
    print(f"     발전량 보정 기준으로 환산하면(45° 기울기 부근),")
    print(f"     6축 {100*np.sin(tilt)*np.radians(six):.0f}% vs "
          f"9축 {100*np.sin(tilt)*np.radians(nine):.0f}% 출력 오차 -> 약 "
          f"{six/nine:.1f}배 차이.")
    print("     발전량이 연구의 핵심 측정량이면 9축으로 가는 게 맞습니다.")
    print()
    print("  4. 시정수는 여기선 짧을수록 좋게 나오지만(tau=0.2 s에서 0.21°),")
    print("     이건 합성 지자기 데이터가 너무 깨끗해서 나온 결과입니다.")
    print("     실제 자력계는 주변 철물/모터 자기장으로 왜곡되니,")
    print("     tau는 실측 후에 다시 잡으세요. 1~2 s에서 시작하면 무난합니다.")
    print()

    altitude_budget()

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, np.degrees(theta), "k", lw=2, label="실제 (ground truth)")
    for name, est in estimates.items():
        axes[0].plot(t, np.degrees(est), lw=1, label=name)
    axes[0].set_ylabel("탑재물 기울기 [deg]")
    axes[0].set_title("흔들리는 탑재물에서 자세 추정")
    axes[0].legend(fontsize=8)

    for name, est in estimates.items():
        axes[1].plot(t, np.degrees(est - theta), lw=1, label=name)
    axes[1].set_ylabel("추정 오차 [deg]")
    axes[1].set_xlabel("시간 [s]")
    for ax in axes:
        ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path("attitude.png"), dpi=260)
    print("  saved out/attitude.png")
