"""Designing the power-generation measurement.

Three things decide the hardware here, and all three come out of numbers rather
than preference:

* The payload swings. Incident angle moves with it, and irradiance on the panel
  follows the cosine, so a power reading taken without a simultaneous attitude
  reading cannot be interpreted. This sizes how much of the variation is swing.

* A fixed resistive load only sits at the maximum power point under one
  condition, so its readings drift with irradiance for reasons that have
  nothing to do with the experiment. Sweeping the IV curve measures Pmax
  directly, plus Voc and Isc, which separate temperature from irradiance.

* The sweep has to finish inside a swing, or it smears several incident angles
  into one curve. That sets the sampling rate, and the sampling rate picks the
  sensor.

Note this is an outdoor measurement. The indoor phase characterises aerodynamics
(see indoor.py); there is no sun in a hall.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

from params import G, LINE_LENGTH, SILK, SUSPENSION_LENGTH, WIND_BASE, Kite, Payload
from payload import dominant_freq, gust, simulate

PANEL_AREA = 0.05          # m^2, flexible perovskite module          MEASURE
EFFICIENCY = 0.12          # module, not cell                         MEASURE
IRRADIANCE = 800.0         # W/m^2, clear day, panel not normal to sun
VOC_NOMINAL = 8.0          # V at full irradiance                     MEASURE
N_CELLS = 10
IDEALITY = 1.5             # perovskite runs higher than silicon
THERMAL_V = 0.02585        # kT/q at 300 K

SWEEP_FRACTION = 0.10      # of a swing period; beyond this the curve smears
SWEEP_POINTS = 50
MIN_SWEEP_POINTS = 30      # enough to place the knee and the maximum

# Per sample the INA226 costs two 140 us conversions plus two I2C register
# reads. A 16-bit read is about 50 bit-times, so the bus speed matters as much
# as the ADC: at 400 kHz the transfer is the same order as the conversion.
INA226_CONVERSION = 2 * 140e-6
I2C_BITS_PER_READ = 50
I2C_SPEEDS = {"400 kHz (기본)": 400e3, "1 MHz (고속)": 1e6}



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def per_point(bus_hz):
    return INA226_CONVERSION + 2 * I2C_BITS_PER_READ / bus_hz
SUN_ELEVATIONS = (30.0, 50.0, 70.0, 90.0)


def iv_curve(irradiance, voltage):
    """Single-diode model with series and shunt resistance neglected.

    Good enough to size a shunt, a capacitor and a sample rate; not good enough
    to report a fill factor from. The real curve will be softer.
    """
    isc = PANEL_AREA * irradiance * EFFICIENCY / (0.8 * VOC_NOMINAL)
    scale = N_CELLS * IDEALITY * THERMAL_V
    voc = VOC_NOMINAL + scale * np.log(max(irradiance, 1.0) / 1000.0)
    i0 = isc / (np.exp(voc / scale) - 1.0)
    return np.maximum(isc - i0 * (np.exp(voltage / scale) - 1.0), 0.0), isc, voc


def max_power(irradiance):
    v = np.linspace(0.0, VOC_NOMINAL * 1.2, 2000)
    i, isc, voc = iv_curve(irradiance, v)
    p = v * i
    k = int(np.argmax(p))
    return {"pmax": p[k], "vmp": v[k], "imp": i[k], "isc": isc, "voc": voc,
            "fill": p[k] / (isc * voc)}


def incidence_from_swing(swing_deg, sun_elevation_deg=50.0):
    """Angle between the panel normal and the sun.

    The panel hangs with the suspension line, so its normal tilts by the swing
    angle. Taken in the flight plane, which is the worst case -- out-of-plane
    sun would couple less.
    """
    panel_tilt = np.radians(swing_deg)
    sun_from_vertical = np.radians(90.0 - sun_elevation_deg)
    return np.degrees(np.abs(panel_tilt - sun_from_vertical))


if __name__ == "__main__":
    print("=" * 78)
    print("1. 전력 규모 (셔트/ADC 레인지 결정용)")
    print("=" * 78)
    print(f"  {'일사량':>10}{'Pmax':>9}{'Vmp':>8}{'Imp':>8}{'Isc':>8}{'Voc':>8}")
    for g in (200.0, 500.0, 800.0, 1000.0):
        m = max_power(g)
        print(f"  {g:>8.0f}W/m²{m['pmax']:>8.2f}W{m['vmp']:>7.2f}V"
              f"{m['imp']:>7.3f}A{m['isc']:>7.3f}A{m['voc']:>7.2f}V")
    full = max_power(1000.0)
    print()
    print(f"  => 전류 레인지 0~{full['isc']*1.3:.1f} A, "
          f"전압 레인지 0~{full['voc']*1.3:.1f} V로 잡으면 충분합니다.")
    print()

    print("=" * 78)
    print("2. 흔들림이 발전량을 얼마나 흔드는가")
    print("=" * 78)
    kite, payload = Kite(), Payload()
    res = simulate(kite, payload, SILK, SUSPENSION_LENGTH, gust(), LINE_LENGTH,
                   t_end=30.0, n_out=3000)

    print(f"  흔들림 {res['swing'].min():.0f}° ~ {res['swing'].max():.0f}° "
          f"(정지 {res['swing'][0]:.0f}°, 돌풍 중 최대 {res['swing'].max():.0f}°)")
    print(f"  영향은 태양 고도에 크게 좌우되므로 고정하지 않고 훑습니다.")
    print()
    print(f"  {'태양고도':>10}{'입사각 범위':>18}{'출력 범위 [W]':>18}"
          f"{'변동폭':>9}{'변동계수':>10}")

    pmax = max_power(IRRADIANCE)["pmax"]
    worst = None
    curves = {}
    for sun in SUN_ELEVATIONS:
        incidence = incidence_from_swing(res["swing"], sun)
        power = pmax * np.cos(np.radians(incidence))
        curves[sun] = power
        cv = np.std(power) / np.mean(power)
        ratio = power.max() / max(power.min(), 1e-9)
        print(f"  {sun:>9.0f}°{incidence.min():>9.0f}~{incidence.max():.0f}°"
              f"{power.min():>12.2f}~{power.max():.2f}{ratio:>8.1f}배"
              f"{100*cv:>9.0f}%")
        if worst is None or cv > worst[1]:
            worst = (sun, cv, ratio)
    print()
    print(f"  최악 조건은 태양고도 {worst[0]:.0f}°에서 변동계수 "
          f"{100*worst[1]:.0f}%, 출력 {worst[2]:.1f}배 변동입니다.")
    print(f"  전부 자세 때문이고 일사량과는 무관합니다. 자세를 같은 타임스탬프로")
    print(f"  기록하지 않으면 일사량 변화와 구분할 방법이 없습니다.")
    print(f"  보정식: P_normalized = P_measured / cos(입사각)")
    print()

    print("=" * 78)
    print("3. IV 스윕 속도 제약")
    print("=" * 78)
    window = (res["t"] >= 5.0) & (res["t"] < 15.0)
    f_swing = dominant_freq(res["t"][window], res["swing"][window])
    period = 1.0 / f_swing
    budget = SWEEP_FRACTION * period

    print(f"  진자 주파수 {f_swing:.2f} Hz -> 주기 {period*1000:.0f} ms")
    print(f"  상한: 주기의 {SWEEP_FRACTION:.0%} = {budget*1000:.0f} ms "
          f"(넘으면 여러 입사각이 한 곡선에 섞임)")
    print()
    print(f"  센서 후보:")
    print(f"    ADS1115 (16bit, 860 SPS)   점당 ~2300 us -> "
          f"{MIN_SWEEP_POINTS}점에 {MIN_SWEEP_POINTS*2300/1000:.0f} ms, 상한 초과")
    print(f"    ESP32-S3 내장 ADC (12bit)  빠르지만 비선형이 심해 비권장")
    print(f"    INA226  (16bit, 변환 140 us x2) -> I2C 속도에 따라 달라짐:")
    for label, hz in I2C_SPEEDS.items():
        t = per_point(hz)
        print(f"      {label:<14} 점당 {t*1e6:>4.0f} us "
              f"(변환 {INA226_CONVERSION*1e6:.0f} + 전송 "
              f"{2*I2C_BITS_PER_READ/hz*1e6:.0f}) -> "
              f"{MIN_SWEEP_POINTS}점에 {MIN_SWEEP_POINTS*t*1000:.1f} ms")
    print(f"  => INA226. 단 점당 시간은 변환시간만이 아니라 I2C 전송이 절반입니다.")
    print()
    print(f"  => 상한 {budget*1000:.0f} ms, 하한은 시간이 아니라 점수로 정해집니다.")
    print(f"     상한에 붙일 이유는 없습니다. 길게 잡을수록 커패시터만 커집니다.")
    print()

    print("=" * 78)
    print("4. 부하 스윕 방식")
    print("=" * 78)
    print("  ESP32-S3에는 DAC가 없습니다 (원조 ESP32에는 있지만 S3에서 빠졌습니다).")
    print("  MOSFET을 선형영역에서 쓰려면 외부 DAC나 PWM+RC가 추가로 필요합니다.")
    print()
    cap_v = full["voc"]
    cap_i = full["imp"]
    print(f"  더 간단한 방법: 커패시터 충전 스윕.")
    print(f"  방전된 커패시터를 패널에 물리면 전압이 0에서 Voc까지 스스로 올라가며")
    print(f"  IV 곡선을 그대로 그립니다. 부품은 커패시터 1개와 방전용 MOSFET 1개.")
    print()
    print(f"  용량을 고르면 스윕 시간 t = C*Voc/Imp 가 따라 나옵니다:")
    # Judged on what actually matters: enough points to resolve the knee, and
    # short enough to stay inside one swing. A fixed minimum sweep time would
    # be the wrong test -- a smaller capacitor simply takes fewer samples, and
    # thirty across the curve is already plenty.
    print(f"  {'용량':>9}{'스윕 시간':>12}{'400kHz':>10}{'1MHz':>9}{'판정':>14}")
    chosen = None
    for farads in (220e-6, 470e-6, 1000e-6, 2200e-6, 4700e-6):
        t = farads * cap_v / cap_i
        slow = int(t / per_point(400e3))
        fast = int(t / per_point(1e6))
        if t > budget:
            verdict = "흔들림 중 변함"
        elif slow >= MIN_SWEEP_POINTS:
            verdict = "적합"
        elif fast >= MIN_SWEEP_POINTS:
            verdict = "1MHz 필요"
        else:
            verdict = "점수 부족"
        if verdict == "적합" and chosen is None:
            chosen = (farads, t, slow)
        print(f"  {farads*1e6:>7.0f}uF{t*1000:>10.1f}ms{slow:>9d}점"
              f"{fast:>8d}점{verdict:>14}")
    print()
    sweep_time = chosen[1]
    print(f"  => 가장 작은 적합 용량인 {chosen[0]*1e6:.0f} uF 채택 "
          f"(비행체라 작을수록 좋습니다).")
    print(f"     스윕 {sweep_time*1000:.1f} ms, 400 kHz에서 {chosen[2]}점, "
          f"주기의 {100*sweep_time/period:.1f}%")
    print(f"     I2C를 1 MHz로 올려도 한 단계 아래 용량(1000 uF)은 "
          f"{int(1000e-6*cap_v/cap_i/per_point(1e6))}점에 그쳐 여전히 부족합니다.")
    print(f"     1 MHz의 이득은 용량 절감이 아니라 같은 용량에서 점수가 늘어나는 것입니다.")
    print(f"     (상한 {budget*1000:.0f} ms에 맞췄다면 "
          f"{cap_i*budget/cap_v*1e6:.0f} uF가 필요했습니다 — 비행체에 못 싣는")
    print(f"      크기입니다. 스윕 시간을 어디서 고르냐가 부품을 좌우합니다.)")
    print()
    print("=" * 78)
    print("설계 요약")
    print("=" * 78)
    print("  * 측정: 커패시터 충전식 IV 스윕, INA226으로 V/I 동시 샘플링")
    print(f"  * 스윕 1회 {sweep_time*1000:.1f} ms, 커패시터 {chosen[0]*1e6:.0f} uF,")
    print(f"    흔들림 1주기({period*1000:.0f} ms)당 1회면 충분")
    print("  * 매 스윕마다 자세(EBIMU)를 같은 타임스탬프로 기록")
    print("  * 보고 값은 Pmax(부하 무관) + Voc(온도 지표) + Isc(일사량 지표)")
    print("  * 발전량 분석은 cos(입사각)으로 정규화한 뒤에만 의미가 있습니다")
    print()

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    v = np.linspace(0, VOC_NOMINAL * 1.15, 400)
    for g in (200.0, 500.0, 800.0, 1000.0):
        i, _, _ = iv_curve(g, v)
        axes[0].plot(v, i, label=f"{g:.0f} W/m²")
        axes[1].plot(v, v * i, label=f"{g:.0f} W/m²")
    axes[0].set_xlabel("전압 [V]")
    axes[0].set_ylabel("전류 [A]")
    axes[0].set_title("IV 곡선")
    axes[1].set_xlabel("전압 [V]")
    axes[1].set_ylabel("전력 [W]")
    axes[1].set_title("전력 곡선 (최대점이 일사량에 따라 이동)")

    for sun, curve in curves.items():
        axes[2].plot(res["t"], curve, lw=1, label=f"태양고도 {sun:.0f}°")
    axes[2].set_xlabel("시간 [s]")
    axes[2].set_ylabel("출력 [W]")
    axes[2].set_title("일사량이 일정해도 흔들림만으로 생기는 변동")

    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path("power.png"), dpi=260)
    print("  saved out/power.png")
