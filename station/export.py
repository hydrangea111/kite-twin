"""Bundle a recorded flight into one self-contained HTML file.

The result opens by double-clicking it. No Python, no bridge, no network, no
server -- which is the point: the people who need to look at a flight are
rarely sitting at the machine that recorded it, and a monitor that only works
next to the rig is not much use for showing anyone anything.

    python station/export.py                    # the newest recording
    python station/export.py logs/....csv       # a particular one
    python station/export.py --open             # and open it

What goes in the file:

  * the frames, already derived. The corrections, the force-balance
    inversion and the model prediction all run here, through the same
    Deriver the live bridge uses, so the bundle shows exactly what the live
    screen showed. Re-deriving in the browser would mean a second
    implementation of the physics, and two implementations drift.
  * three.js, the scene, the page and its styles, inlined.
  * the sensor calibration the rig was using, read from the firmware.

Module imports do not work from file://, so the ES modules are flattened into
one classic module script. three.js ends in a single `export { ... }`, which
becomes a plain object literal; the project's own `export` keywords are
dropped, leaving ordinary top-level declarations in one shared scope.
"""

import argparse
import json
import re
import sys
import time
import webbrowser
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))

import bridge                                              # noqa: E402
from params import Payload, SILK, Kite                     # noqa: E402

MAX_FRAMES = 12000          # 20 Hz -> ten minutes; beyond that the file bloats


def flatten_three(src):
    """three.module.js -> the same code with its exports as a THREE object."""
    m = re.search(r"\nexport\s*\{([^}]*)\}\s*;?\s*$", src, re.S)
    if not m:
        raise SystemExit("three.module.js 의 export 블록을 찾지 못했습니다")

    pairs = []
    for item in m.group(1).split(","):
        item = item.strip()
        if not item:
            continue
        if " as " in item:
            local, exported = (x.strip() for x in item.split(" as "))
        else:
            local = exported = item
        pairs.append((exported, local))

    body = src[:m.start()]
    obj = ", ".join(f"{e}: {l}" for e, l in pairs)
    return f"{body}\nconst THREE = {{ {obj} }};\n"


def strip_module_syntax(src):
    """Drop import lines and `export` keywords from one of our own modules."""
    src = re.sub(r"^\s*import\s+[^;]+;\s*$", "", src, flags=re.M)
    src = re.sub(r"^export\s+(const|let|var|class|function)\b", r"\1", src,
                 flags=re.M)
    return src


def derive_frames(csv_path, kite, tether, limit=MAX_FRAMES):
    """Run the recording through the real Deriver."""
    derive = bridge.Deriver(kite, Payload(), tether)
    frames = []
    t0 = None
    with csv_path.open(encoding="utf-8", newline="") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith(("#", "t_ms")):
                continue
            row = bridge.parse_row(line.split(","))
            if not row:
                continue
            if t0 is None:
                t0 = row["t_ms"]
            frame = derive(row, (row["t_ms"] - t0) / 1000.0)
            frame.pop("_row", None)
            frames.append(frame)
            if len(frames) >= limit:
                print(f"[export] {limit}프레임에서 잘랐습니다")
                break
    return frames


def build(csv_path, out_path, kite, tether):
    frames = derive_frames(csv_path, kite, tether)
    if not frames:
        raise SystemExit(f"{csv_path.name}: 쓸 수 있는 행이 없습니다")

    meta = {
        "mode": f"오프라인 기록 {csv_path.name}",
        "live": False,
        "source": "replay",
        "current": "replay",
        "status": "running",
        "error": None,
        "sources": [],
        "sim_cfg": None,
        "recording": None,
        "port": None,
        "kite": {"area": kite.area, "mass": kite.mass,
                 "cl": kite.cl, "cd": kite.cd},
        "tether": {"name": tether.name, "EA": tether.EA,
                   "break_load": tether.break_load},
    }

    payload = {
        "meta": meta,
        "sensors": bridge.read_sensor_config(),
        "frames": frames,
        "recorded": csv_path.name,
        "exported": time.strftime("%Y-%m-%d %H:%M"),
    }

    html = (HERE / "monitor.html").read_text(encoding="utf-8")
    css = (HERE / "style.css").read_text(encoding="utf-8")
    three = flatten_three((HERE / "vendor" / "three.module.js")
                          .read_text(encoding="utf-8"))
    scene = strip_module_syntax((HERE / "scene.js").read_text(encoding="utf-8"))
    app = strip_module_syntax((HERE / "app.js").read_text(encoding="utf-8"))

    html = html.replace('<link rel="stylesheet" href="style.css">',
                        "<style>\n" + css + "\n</style>")

    # </script> inside the data would close the tag early; JSON escapes it fine.
    data = json.dumps(payload, ensure_ascii=False).replace("</", "<\\/")

    # A classic script, not type="module". Chrome blocks module *imports* from
    # file://, and although an inline module with no imports is allowed, this
    # bundle needs nothing from module semantics -- no top-level await, no
    # import.meta -- so the right tag is the one with nothing left to block.
    # Wrapped in an IIFE so the three.js internals stay out of the global
    # scope the way they did inside the module.
    bundle = (
        "<script>window.__KITE_BUNDLE__ = " + data + ";</script>\n"
        "<script>\n(function () {\n"
        "/* three.js r160 (MIT) */\n" + three + "\n"
        "/* scene.js */\n" + scene + "\n"
        "/* app.js */\n" + app + "\n"
        "})();\n</script>"
    )
    html = html.replace('<script type="module" src="app.js"></script>', bundle)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding="utf-8")
    return len(frames), out_path.stat().st_size


def main():
    ap = argparse.ArgumentParser(description="기록을 단일 HTML 로 내보내기")
    ap.add_argument("csv", nargs="?", help="기록 파일 (기본: 가장 최근 것)")
    ap.add_argument("-o", "--out", help="출력 경로")
    ap.add_argument("--area", type=float, help="실측 날개 면적 [m²]")
    ap.add_argument("--mass", type=float, help="실측 전체 질량 [kg]")
    ap.add_argument("--ea", type=float, help="실측 계류줄 EA [N]")
    ap.add_argument("--open", action="store_true", help="만든 뒤 열기")
    args = ap.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
    else:
        logs = sorted(bridge.LOG_DIR.glob("*.csv"),
                      key=lambda f: f.stat().st_mtime, reverse=True)
        if not logs:
            raise SystemExit(f"{bridge.LOG_DIR} 에 기록이 없습니다")
        csv_path = logs[0]
    if not csv_path.is_file():
        raise SystemExit(f"파일을 찾을 수 없습니다: {csv_path}")

    from dataclasses import replace
    kite = Kite()
    if args.area:
        kite = replace(kite, area=args.area)
    if args.mass:
        kite = replace(kite, mass_canopy=args.mass - kite.mass_payload)
    tether = replace(SILK, EA=args.ea) if args.ea else SILK

    out = Path(args.out) if args.out else (HERE / "export" / (csv_path.stem + ".html"))
    n, size = build(csv_path, out, kite, tether)
    print(f"[export] {csv_path.name} -> {out}")
    print(f"[export] {n:,}프레임 · {size / 1024 / 1024:.1f} MB · 서버 없이 열립니다")
    if args.open:
        webbrowser.open(out.resolve().as_uri())


if __name__ == "__main__":
    main()
