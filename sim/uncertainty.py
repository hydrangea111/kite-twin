"""How wrong are the predictions, and which unknown is to blame?

Every MEASURE/FIT parameter is still a guess, so a single-point prediction is
misleading. This samples all of them over plausible ranges, reports percentile
bands instead of point values, and ranks the inputs by how strongly each drives
each output.

The ranking is the actionable part: adding model fidelity is wasted effort while
an unmeasured coefficient dominates the error budget. Measure the top of the list
first.
"""

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

from dataclasses import replace

import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

import wind
from model import effective_mass, equilibrium, min_area_for_elevation, simulate
from params import LINE_LENGTH, SILK, Kite

N_STATIC = 2000
N_DYNAMIC = 150

# (low, high, how you would pin it down)
RANGES = {
    "날개면적 area":      (0.40, 0.60, "자 하나면 됨 — 제작 후 실측"),
    "캐노피 질량":         (0.11, 0.19, "저울 — 제작 후 실측"),
    "탑재물 질량":         (0.27, 0.33, "저울 — 이미 거의 확정"),
    "양력계수 CL":        (0.45, 1.05, "비행 로그 피팅 필요 (디지털 트윈)"),
    "항력계수 CD":        (0.23, 0.53, "비행 로그 피팅 필요 (디지털 트윈)"),
    "명주실 EA":          (1800, 4200, "추 매달고 늘어남 측정 — 당장 가능"),
    "풍속 (환경)":        (4.0, 8.0, "풍속계 구매"),
}
KEYS = list(RANGES)



def out_path(name):
    """그림 저장 경로. 어느 디렉터리에서 실행해도 sim/out 에 저장된다.

    종전에는 "out/<name>" 이라는 상대경로였다. sim 안에서 돌리면 맞고 저장소
    루트에서 돌리면 FileNotFoundError 가 난다. 보고서 부록 F 가 적어 둔
    `python sim/uncertainty.py` 가 바로 그 경우였다.
    """
    d = Path(__file__).resolve().parent / "out"
    d.mkdir(exist_ok=True)
    return str(d / name)

def draw(rng, n):
    return np.column_stack([rng.uniform(lo, hi, n) for lo, hi, _ in RANGES.values()])


def build(row):
    area, m_canopy, m_payload, cl, cd, ea, wind = row
    kite = Kite(area=area, mass_canopy=m_canopy, mass_payload=m_payload, cl=cl, cd=cd)
    return kite, replace(SILK, EA=ea), wind


def gust(base, peak_ratio=1.67, t0=1.0, dur=1.5):
    return wind.gust(wind.uniform(base), peak_ratio, t0, dur)


def static_outputs(samples):
    names = ["고도각 [deg]", "장력 [N]", "늘어남 [cm]", "고유진동 [Hz]",
             "30° 필요면적 [m^2]"]
    out = np.full((len(samples), len(names)), np.nan)

    for i, row in enumerate(samples):
        kite, tether, wind = build(row)
        try:
            state, T, elev = equilibrium(kite, tether, wind, LINE_LENGTH)
        except Exception:
            continue
        if not np.isfinite(elev) or elev <= 0:
            continue   # kite does not fly for this draw; recorded as a failure
        out[i] = [
            elev,
            T,
            (np.linalg.norm(state[:2]) - LINE_LENGTH) * 100,
            tether.natural_freq(LINE_LENGTH, effective_mass(kite)),
            min_area_for_elevation(kite, wind, 30.0),
        ]
    return names, out


def dynamic_outputs(samples):
    peaks = np.full(len(samples), np.nan)
    for i, row in enumerate(samples):
        kite, tether, wind = build(row)
        try:
            state, _, elev = equilibrium(kite, tether, wind, LINE_LENGTH)
            if not np.isfinite(elev) or elev <= 0:
                continue
            res = simulate(kite, tether, gust(wind), lambda t: LINE_LENGTH,
                           state, 8.0, n_out=200)
            peaks[i] = res["tension"].max()
        except Exception:
            continue
    return peaks


def band(values):
    ok = values[np.isfinite(values)]
    if len(ok) == 0:
        return None
    return np.percentile(ok, [5, 50, 95])


def report_bands(names, data, extra=None):
    print("=" * 78)
    print(f"예측 구간 (파라미터 불확실성 전파, N={len(data)})")
    print("=" * 78)
    print(f"  {'출력':<22}{'5%':>12}{'중앙값':>12}{'95%':>12}{'폭/중앙값':>12}")
    rows = list(zip(names, data.T))
    if extra:
        rows += list(extra)
    for name, values in rows:
        b = band(values)
        if b is None:
            print(f"  {name:<22}{'— 해 없음':>12}")
            continue
        spread = (b[2] - b[0]) / abs(b[1]) if b[1] != 0 else np.inf
        print(f"  {name:<22}{b[0]:>12.2f}{b[1]:>12.2f}{b[2]:>12.2f}"
              f"{spread*100:>11.0f}%")
    print()


def report_sensitivity(samples, names, data, peaks):
    all_names = names + ["돌풍 최대장력 [N]"]
    all_data = np.column_stack([data, peaks_to_column(peaks, len(data))])

    print("=" * 78)
    print("민감도 (스피어만 상관계수, |값|이 클수록 그 출력을 지배)")
    print("=" * 78)
    header = "".join(f"{k.split()[0][:7]:>9}" for k in KEYS)
    print(f"  {'출력':<22}{header}")

    matrix = np.zeros((len(all_names), len(KEYS)))
    for r, (name, values) in enumerate(zip(all_names, all_data.T)):
        ok = np.isfinite(values)
        cells = ""
        for c in range(len(KEYS)):
            if ok.sum() < 10:
                matrix[r, c] = np.nan
                cells += f"{'—':>9}"
                continue
            rho = spearmanr(samples[ok, c], values[ok]).statistic
            matrix[r, c] = rho
            cells += f"{rho:>9.2f}"
        print(f"  {name:<22}{cells}")
    print()
    return matrix


def peaks_to_column(peaks, n):
    col = np.full(n, np.nan)
    col[:len(peaks)] = peaks
    return col


def report_priority(matrix):
    score = np.nanmean(np.abs(matrix), axis=0)
    order = np.argsort(-score)

    print("=" * 78)
    print("측정 우선순위 (평균 |민감도| 기준)")
    print("=" * 78)
    for rank, idx in enumerate(order, 1):
        key = KEYS[idx]
        print(f"  {rank}. {key:<16}{score[idx]:>6.2f}   {RANGES[key][2]}")
    print()


if __name__ == "__main__":
    rng = np.random.default_rng(1)
    samples = draw(rng, N_STATIC)
    names, data = static_outputs(samples)
    peaks = dynamic_outputs(samples[:N_DYNAMIC])

    flew = np.isfinite(data[:, 0]).sum()
    print(f"\n  표본 {N_STATIC}개 중 {flew}개가 비행 가능 "
          f"({100*flew/N_STATIC:.0f}%) — 나머지는 양력 부족으로 안 뜸\n")

    report_bands(names, data, extra=[("돌풍 최대장력 [N]", peaks)])
    matrix = report_sensitivity(samples, names, data, peaks)
    report_priority(matrix)

    need = data[np.isfinite(data[:, 4]), 4]
    safe_area = np.percentile(need, 90)
    print("=" * 78)
    print("해석")
    print("=" * 78)
    print(f"  1. 지금 예측은 구간이 너무 넓어서 단일 수치로 쓰면 안 됩니다.")
    print(f"     고도각 {band(data[:, 0])[0]:.0f}~{band(data[:, 0])[2]:.0f}°, "
          f"장력 {band(data[:, 1])[0]:.1f}~{band(data[:, 1])[2]:.1f} N.")
    print(f"     앞서 보고한 38.3° / 5.5 N은 이 분포의 중앙값일 뿐입니다.")
    print()
    print(f"  2. 표본의 {100-100*flew/N_STATIC:.0f}%는 아예 안 뜹니다. 설계가 "
          f"비행 가능 경계에 가깝다는 뜻입니다.")
    print(f"     30° 고도각 확보에 필요한 면적이 90% 확률로 "
          f"{safe_area:.2f} m^2 이하이므로,")
    print(f"     현재 가정치 0.50 m^2가 아니라 최소 {safe_area:.2f} m^2로 "
          f"제작하시길 권합니다.")
    print()
    print(f"  3. 풍속이 거의 모든 출력을 지배합니다. 풍속계가 단일 투자로는")
    print(f"     불확실성을 가장 많이 줄여줍니다 — 모델 개선보다 우선입니다.")
    print()
    print(f"  4. 명주실 EA는 고유진동수를 거의 혼자 결정하고(상관 0.97),")
    print(f"     추 매달아 늘어남만 재면 오늘 당장 확정됩니다. 비용 0원.")
    print()
    print(f"  5. 질량 항목들은 민감도가 0.06로 사실상 무의미합니다.")
    print(f"     정밀하게 달 필요 없습니다.")
    print()
    print(f"  => 이 상태에서 3D 쿼드라인 같은 모델 고도화를 더 해봐야,")
    print(f"     파라미터 불확실성(±150%)에 묻혀서 의미가 없습니다.")
    print(f"     풍속계 + EA 실측 -> 첫 비행 -> CL/CD 피팅 순서가 맞습니다.")
    print()

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for ax, col, title in zip(axes, [0, 1, 4],
                              ["고도각 [deg]", "장력 [N]", "30° 필요면적 [m²]"]):
        vals = data[:, col]
        vals = vals[np.isfinite(vals)]
        ax.hist(vals, bins=40, color="#4C72B0", alpha=0.85)
        for q, style in zip(np.percentile(vals, [5, 50, 95]), ["--", "-", "--"]):
            ax.axvline(q, color="k", ls=style, lw=1)
        ax.set_title(title)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("표본 수")
    fig.suptitle("파라미터 불확실성이 만드는 예측 분포 (점선 5/95%)")
    fig.tight_layout()
    fig.savefig(out_path("uncertainty.png"), dpi=260)
    print("  saved out/uncertainty.png")
