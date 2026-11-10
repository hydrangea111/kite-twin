"""Run every analysis script in order and report what happened.

Each script is a standalone study that prints its own findings and writes a
figure. Running them by hand is fine until you change something in model.py,
at which point you want to know that all sixteen still run and still agree
with the numbers quoted in the report.

Scripts run in a subprocess rather than by import, because several of them
configure matplotlib and seed RNGs at module scope; sharing one interpreter
would let one study's state leak into the next.

    python run_all.py              # everything
    python run_all.py --quick      # skip the slow sweeps
    python run_all.py elasticity   # just these
"""

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
OUT = HERE / "out"

# (module, what it answers, slow?)
STUDIES = [
    ("convergence",       "적분기 허용오차 수렴성", False),
    ("scaling",           "차원해석과 상사 시험", False),
    ("compare_tether",    "줄 종류별 거동 비교", False),
    ("elasticity",        "축강성 단독 스윕", True),
    ("control",           "고도 제어 응답", False),
    ("control_envelope",  "제어 실행가능 영역", True),
    ("uncertainty",       "파라미터 불확실성 전파", True),
    ("lowaltitude",       "지표 경계층과 저고도", False),
    ("payload",           "탑재물 진자와 장력 오염", False),
    ("attitude",          "자세 추정 방식 비교", False),
    ("loads",             "난류 하중 스펙트럼", True),
    ("spool",             "스풀 권선 기하", False),
    ("power",             "발전량 계측 설계", False),
    ("predict",           "예측 한계와 역산", False),
    ("twin",              "공력계수 식별", True),
    ("auto",              "자동 보정과 변화 감지", True),
    ("indoor",            "실내 공력 특성화", False),
    ("extremes",          "설계하중 극값 분포", True),
]

# Numbers the report quotes. If a code change moves one of these, the report is
# now wrong and should be regenerated -- that is the whole point of checking.
QUOTED = {
    "compare_tether": [
        # 부가질량 반영 후 값. 1.84 Hz 는 기체 질량만 쓰던 때의 수치다.
        (r"명주실.*?(\d+\.\d+)\s*Hz", 1.52, 0.02, "명주실 고유진동수"),
    ],
    "payload": [
        (r"최대\s*(\d+)%", 3.0, 1.0, "장력 맥동"),
    ],
    "spool": [
        (r"15\s*->\s*(\d+)\s*mm", 30.0, 1.0, "최대 유효 반지름"),
    ],
}


def check_quoted(name, stdout):
    """Compare a few headline numbers against what the report says."""
    issues = []
    for pattern, expect, tol, label in QUOTED.get(name, []):
        m = re.search(pattern, stdout, re.S)
        if not m:
            issues.append(f"{label}: 찾지 못함")
            continue
        got = float(m.group(1))
        if abs(got - expect) > tol:
            issues.append(f"{label}: {got} (보고서 {expect})")
    return issues


def run(name, desc, stream=False):
    """Run one study. `stream` echoes its progress lines as they arrive.

    Capturing output is what lets this check quoted numbers afterwards, but a
    study that runs for twenty minutes then has no visible progress at all --
    there is no way to tell a slow run from a wedged one. The slow ones echo
    their indented progress lines through while still being captured.
    """
    started = time.perf_counter()
    if not stream:
        proc = subprocess.run(
            [sys.executable, "-X", "utf8", f"{name}.py"],
            cwd=HERE, capture_output=True, text=True, encoding="utf-8", errors="replace",
        )
        out, err, code = proc.stdout, proc.stderr, proc.returncode
    else:
        print(flush=True)
        lines = []
        proc = subprocess.Popen(
            [sys.executable, "-X", "utf8", "-u", f"{name}.py"],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", bufsize=1,
        )
        for line in proc.stdout:
            lines.append(line)
            if line.startswith("    ") and "/" in line:
                print("     " + line.rstrip(), flush=True)
        proc.wait()
        out, err, code = "".join(lines), "", proc.returncode
    elapsed = time.perf_counter() - started
    # Keep what the study printed. A 40-minute run whose output is discarded
    # because it succeeded has to be run again to read its own numbers, which
    # is what happened the first time this was needed.
    try:
        (OUT / f"{name}.txt").write_text(out, encoding="utf-8")
    except OSError:
        pass

    ok = code == 0
    issues = check_quoted(name, out) if ok else []
    return {
        "name": name, "desc": desc, "ok": ok, "elapsed": elapsed,
        "issues": issues, "stdout": out, "stderr": err,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("only", nargs="*", help="run only these modules")
    ap.add_argument("--quick", action="store_true", help="skip the slow sweeps")
    ap.add_argument("--verbose", action="store_true", help="print each script's output")
    args = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    before = {f.name: f.stat().st_mtime for f in OUT.glob("*.png")}

    todo = STUDIES
    if args.only:
        todo = [s for s in STUDIES if s[0] in args.only]
        missing = set(args.only) - {s[0] for s in STUDIES}
        if missing:
            sys.exit(f"알 수 없는 모듈: {', '.join(sorted(missing))}")
    if args.quick:
        todo = [s for s in todo if not s[2]]

    print(f"{len(todo)}개 스터디 실행\n")
    results = []
    for i, (name, desc, slow) in enumerate(todo, 1):
        mark = " (느림)" if slow else ""
        print(f"  [{i:>2}/{len(todo)}] {name:<18}{desc}{mark} ... ", end="", flush=True)
        r = run(name, desc, stream=(name == "extremes"))
        results.append(r)
        if not r["ok"]:
            print(f"실패 ({r['elapsed']:.1f}s)")
        elif r["issues"]:
            print(f"값 불일치 ({r['elapsed']:.1f}s)")
        else:
            print(f"{r['elapsed']:.1f}s")
        if args.verbose and r["stdout"]:
            print("\n" + r["stdout"])

    after = {f.name: f.stat().st_mtime for f in OUT.glob("*.png")}
    fresh = sorted(n for n, t in after.items() if before.get(n) != t)

    print()
    print("=" * 72)
    failed = [r for r in results if not r["ok"]]
    drifted = [r for r in results if r["ok"] and r["issues"]]
    total = sum(r["elapsed"] for r in results)
    print(f"{len(results) - len(failed)}/{len(results)} 성공, "
          f"그림 {len(fresh)}개 갱신, 총 {total:.0f}초")
    print("=" * 72)

    if fresh:
        print("\n갱신된 그림:")
        for n in fresh:
            print(f"  out/{n}")

    for r in drifted:
        print(f"\n[값 불일치] {r['name']}")
        for issue in r["issues"]:
            print(f"  {issue}")
        print("  -> 보고서의 해당 수치를 다시 확인하세요.")

    for r in failed:
        print(f"\n[실패] {r['name']} — {r['desc']}")
        tail = (r["stderr"] or r["stdout"]).strip().splitlines()[-12:]
        for line in tail:
            print(f"  {line}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
