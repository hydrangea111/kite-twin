"""Ground-station bridge: telemetry in, browser out.

The ground ESP32 prints one CSV line per received packet on USB serial. This
reads that, applies the corrections the simulation study established, and
serves the result to a browser over Server-Sent Events.

Three sources, same output schema, so the UI cannot tell them apart:

    python bridge.py --serial COM5      live rig
    python bridge.py --replay log.csv   a recorded flight, at its own pace
    python bridge.py --sim              the validated two-body model

`--sim` exists so the UI can be built and judged before the hardware runs, and
so a demo does not depend on wind. It is not a cartoon: it pre-integrates the
same model the report is based on and streams the result at wall-clock rate.

Stdlib plus pyserial only. SSE rather than WebSocket because telemetry is
one-way, and the browser reconnects on its own when the bridge restarts.
"""

import argparse
import csv
import io
import json
import math
import os
import queue
import sys
import threading
from collections import deque
from dataclasses import replace
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Frozen (PyInstaller) and source runs put the UI files and the sim package in
# different places, and a frozen app's own directory is a temp folder that is
# wiped on exit -- so recordings and the remembered port go to the user's
# AppData instead. DATA is the only writable one of the three.
if getattr(sys, "frozen", False):
    _BUNDLE = Path(sys._MEIPASS)
    HERE = _BUNDLE / "station"
    SIM = _BUNDLE / "sim"
    DATA = Path(os.environ.get("LOCALAPPDATA") or Path.home()) / "KiteStation"
else:
    HERE = Path(__file__).parent
    SIM = HERE.parent / "sim"
    DATA = HERE
sys.path.insert(0, str(SIM))

from model import sag_angle                                    # noqa: E402
from params import G, RHO_AIR, SILK, Kite, Payload             # noqa: E402

# Column order emitted by firmware/ground/src/main.cpp. Kept here rather than
# parsed from the header line so a silently reordered firmware is caught.
COLUMNS = (
    "t_ms", "line_length_m", "tension_n", "elevation_deg", "wind_ms",
    "rx_ms", "seq", "uptime_ms", "roll", "pitch", "yaw", "fix", "sats",
    "lat", "lon", "alt_gps", "gps_time", "imu_age_ms", "imu_fields", "dropped",
    "pv_sweep_ms", "pv_pmax", "pv_vmp", "pv_imp", "pv_voc", "pv_isc",
    "pv_points", "pv_valid",
)
FLOATS = {"line_length_m", "tension_n", "elevation_deg", "wind_ms", "roll",
          "pitch", "yaw", "lat", "lon", "alt_gps", "pv_pmax", "pv_vmp",
          "pv_imp", "pv_voc", "pv_isc"}

# Per-column wire format, mirroring the firmware's printf. test_bridge.py
# checks these against main.cpp, so a firmware edit that is not reflected
# here fails rather than silently producing lines the rig would never emit.
COLUMN_FORMATS = {
    "t_ms": "%d", "line_length_m": "%.3f", "tension_n": "%.3f",
    "elevation_deg": "%.2f", "wind_ms": "%.2f", "rx_ms": "%d", "seq": "%d",
    "uptime_ms": "%d", "roll": "%.2f", "pitch": "%.2f", "yaw": "%.2f",
    "fix": "%d", "sats": "%d", "lat": "%.7f", "lon": "%.7f", "alt_gps": "%.1f",
    "gps_time": "%d", "imu_age_ms": "%d", "imu_fields": "%d", "dropped": "%d",
    "pv_sweep_ms": "%d", "pv_pmax": "%.4f", "pv_vmp": "%.3f", "pv_imp": "%.4f",
    "pv_voc": "%.3f", "pv_isc": "%.4f", "pv_points": "%d", "pv_valid": "%d",
}


# model.sag_angle's own limit: the small-sag form stops describing the cable
# past roughly this much, and 1/T sends it there as soon as the line slackens.
MAX_SAG = math.radians(20.0)

# Below this the line is not carrying the kite in any useful sense, and every
# quantity derived from the force balance is meaningless rather than noisy.
SLACK_TENSION = 0.25           # N

STEADY_TENSION_RATE = 0.8      # N/s   ─ sim/twin.py
STEADY_ELEVATION_RATE = 1.5    # deg/s


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    return ordered[mid] if n % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


STATE_FILE = DATA / ".station-state.json"
LOG_DIR = DATA / "logs"
FAKE_RIG_URL = "socket://127.0.0.1:5577"


def load_state():
    """Settings remembered from the last run.

    Kept so that restarting the bridge -- which happens constantly while the
    rig is being built -- does not throw away the simulator settings someone
    just dialled in, or the port they just found.
    """
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_state(**patch):
    state = load_state()
    state.update(patch)
    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=1),
                              encoding="utf-8")
    except OSError as exc:
        print(f"[bridge] 설정 저장 실패: {exc}")


def list_ports():
    """Serial ports the machine can see right now, plus anything remembered.

    A port that is not present is still offered, greyed out by the UI: the
    usual case is that the ESP32 is simply not plugged in yet, and hiding the
    port someone used five minutes ago is not helpful.
    """
    found = []
    try:
        from serial.tools import list_ports as lp
        for p in lp.comports():
            found.append({
                "port": p.device,
                "label": (p.description or p.device),
                "present": True,
                "kind": "serial",
            })
    except Exception as exc:                                   # noqa: BLE001
        print(f"[bridge] 포트 목록 실패: {exc}")

    seen = {p["port"] for p in found}
    # The fake rig's address is always offered, running or not. It is the one
    # way to exercise the whole chain with no hardware, and someone who has
    # not got the rig up yet is exactly the person who needs to find it.
    if FAKE_RIG_URL not in seen:
        found.append({"port": FAKE_RIG_URL, "label": "모의 리그 (fake_rig.py)",
                      "present": False, "kind": "emulated"})
        seen.add(FAKE_RIG_URL)
    for remembered in load_state().get("ports", []):
        if remembered not in seen:
            found.append({"port": remembered, "label": remembered,
                          "present": False,
                          "kind": "emulated" if "://" in remembered else "serial"})
    return found


CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".woff2": "font/woff2",
}


class Recorder:
    """Writes every row to CSV, in the firmware's own format.

    Two reasons. A flight that was not recorded cannot be re-examined, and
    the identification work all happens after landing. And a recording plays
    back through `--replay` with no rig, no wind and no network, which is what
    makes the whole thing usable away from the field.

    The file is the firmware's wire format rather than the derived frames:
    derived values can be recomputed from a row whenever the corrections
    change, and often will be. The reverse is not true.
    """

    def __init__(self, directory, tag):
        self.path = None
        self.rows = 0
        self._fh = None
        try:
            directory.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            safe = "".join(c if c.isalnum() else "-" for c in tag)[:24]
            self.path = directory / f"{stamp}-{safe}.csv"
            self._fh = self.path.open("w", encoding="utf-8", newline="")
            self._fh.write(",".join(COLUMNS) + "\n")
        except OSError as exc:
            print(f"[bridge] 기록 실패: {exc}")
            self.path = None

    def write(self, row):
        if not self._fh:
            return
        try:
            self._fh.write(format_row(row) + "\n")
            self.rows += 1
            # Flushed every second rather than per row: a power cut mid-flight
            # should cost a second of data, not the session, and 20 Hz of
            # fsync would be silly.
            if self.rows % 20 == 0:
                self._fh.flush()
        except (OSError, KeyError, TypeError) as exc:
            print(f"[bridge] 기록 중단: {exc}")
            self.close()

    def close(self):
        if self._fh:
            try:
                self._fh.flush()
                self._fh.close()
            except OSError:
                pass
            self._fh = None


class Hub:
    """Fans one telemetry stream out to however many browsers are attached.

    Frames carry the token of the source that produced them. A source that is
    being torn down can still be blocked inside a one-second serial read, and
    it will publish one more frame on the way out -- after the new source has
    already cleared `latest`. Without the token that stale frame arrives at
    the browser labelled as the new source, which is how a toggle ends up
    showing a reading that belongs to the other one.
    """

    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()
        self.latest = None
        self.token = None

    def subscribe(self):
        q = queue.Queue(maxsize=240)
        with self._lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def publish(self, frame, token=None):
        if token is not None and token is not self.token:
            return
        self.latest = frame
        payload = json.dumps(frame, ensure_ascii=False)
        with self._lock:
            targets = list(self._clients)
        for q in targets:
            try:
                q.put_nowait(payload)
            except queue.Full:
                # A browser that cannot keep up loses samples rather than
                # stalling the rig's reader thread.
                pass


# --------------------------------------------------------------------------
# derived quantities
# --------------------------------------------------------------------------

class Deriver:
    """Raw channels -> what the report says they actually mean.

    Three corrections, each established in the simulation study:
      * the encoder reads unstretched length, so the kite is further out;
      * a ground angle sensor reads the cable tangent, not the chord;
      * CL and CD follow from the force balance once both are undone.

    Everything here is also reported uncorrected, because seeing the size of
    each correction live is the point -- on the bench they are millimetres, at
    50 m they are not.
    """

    def __init__(self, kite, payload, tether, suspension_m=0.5):
        self.kite = kite
        self.payload = payload
        self.tether = tether
        self.suspension = suspension_m
        self._seen = 0
        self._first_wall = None
        self._last_seq = None
        # Rolling state for the quasi-static gate and the CL/CD median.
        self._recent = deque(maxlen=40)      # (wall, tension, elevation)
        self._cls = deque(maxlen=200)
        self._cds = deque(maxlen=200)
        self._steady = False
        self._pred_cache = {}

    def __call__(self, row, wall):
        length = max(row.get("line_length_m") or 0.0, 1e-6)
        tension = max(row.get("tension_n") or 0.0, 0.0)
        measured = row.get("elevation_deg") or 0.0
        wind = max(row.get("wind_ms") or 0.0, 0.0)

        # Stretch: r = L0 (1 + T/EA).
        strain = tension / self.tether.EA
        span = length * (1.0 + strain)

        # Sag: the tangent the sensor reads is shallower than the chord.
        #
        # delta ~ 1/T, so as the line goes slack the correction runs away:
        # at 0.01 N the small-sag form returns 1151 deg. model.sag_angle says
        # in as many words that a result past ~20 deg is out of range, and
        # applying it anyway produced elevations of 98 deg on screen -- past
        # what this kite's L/D can reach at any wind. Clamp it and say the
        # correction is not trustworthy, rather than reporting a number the
        # geometry forbids.
        meas_rad = math.radians(measured)
        raw_sag = sag_angle(tension, length, meas_rad, self.tether) if tension > 1e-9 else 0.0
        # bool(), not the bare comparison: sag_angle goes through numpy, so
        # `raw_sag <= MAX_SAG` is a numpy.bool_, and json refuses it. That
        # would take down the live stream too, not just the export.
        sag_valid = bool(raw_sag <= MAX_SAG)
        sag = min(raw_sag, MAX_SAG)
        chord = meas_rad + sag

        x, z = span * math.cos(chord), span * math.sin(chord)
        x_raw = length * math.cos(meas_rad)
        z_raw = length * math.sin(meas_rad)

        self._recent.append((wall, tension, measured))
        self._steady = self._quasi_static()
        cl, cd, wind_est = self._coefficients(length, tension, chord, wind)
        # Only a quasi-static sample may enter the estimate. An instantaneous
        # inversion during a transient is not a noisy CL, it is a wrong one:
        # the balance it assumes does not hold while the kite is accelerating.
        if (self._steady and sag_valid and tension > SLACK_TENSION
                and cl is not None and 0.0 < cl < 4.0 and 0.0 < cd < 4.0):
            self._cls.append(cl)
            self._cds.append(cd)
        cl_est = _median(self._cls)
        cd_est = _median(self._cds)
        d_chord_deg = math.degrees(chord)

        # The payload hangs on its own line; its tilt is the swing angle, and
        # that is what sets the panel's incident angle.
        tilt = row.get("pitch")
        tilt = 0.0 if tilt is None else tilt
        pv = self._power(row, tilt)

        pred = self.predict(wind, length)
        div = None
        if pred:
            div = {
                "elevation": round(d_chord_deg - pred["elevation"], 2),
                "tension": round(tension - pred["tension"], 3),
                "z": round(z - pred["z"], 3),
            }

        self._seen += 1
        if self._first_wall is None:
            self._first_wall = wall

        return {
            "t": round(wall - self._first_wall, 3),
            "seq": row.get("seq"),
            "_row": row,
            "raw": {
                "length": round(length, 3),
                "tension": round(tension, 3),
                "elevation": round(measured, 2),
                "wind": round(wind, 2),
                "roll": round(row.get("roll") or 0.0, 2),
                "pitch": round(tilt, 2),
                "yaw": round(row.get("yaw") or 0.0, 2),
            },
            "derived": {
                "span": round(span, 3),
                "stretch_cm": round((span - length) * 100.0, 2),
                "sag_deg": round(math.degrees(sag), 3),
                "sag_valid": sag_valid,
                "slack": bool(tension <= SLACK_TENSION),
                "chord_deg": round(math.degrees(chord), 2),
                "x": round(x, 3), "z": round(z, 3),
                "x_raw": round(x_raw, 3), "z_raw": round(z_raw, 3),
                "alt_err_cm": round((z - z_raw) * 100.0, 1),
                "cl": None if cl_est is None else round(cl_est, 3),
                "cd": None if cd_est is None else round(cd_est, 3),
                "ld": (None if not cl_est or not cd_est
                       else round(cl_est / cd_est, 2)),
                "cl_now": None if cl is None else round(cl, 3),
                "steady": self._steady,
                "n_steady": len(self._cls),
                "wind_est": None if wind_est is None else round(wind_est, 2),
                "payload_x": round(x + self.suspension * math.sin(math.radians(tilt)), 3),
                "payload_z": round(z - self.suspension * math.cos(math.radians(tilt)), 3),
            },
            "pv": pv,
            "predicted": pred,
            "divergence": div,
            "link": self._link(row),
            "limits": {
                "break_load": self.tether.break_load,
                "design_load": 50.0,
                "tension_frac": round(tension / self.tether.break_load, 4),
            },
        }

    def predict(self, wind, length):
        """Where the model says the kite should be, for this wind and length.

        This is the twin's whole point: the rig and the model are given the
        same inputs, and the gap between them is the finding. A twin that only
        replays the measurement cannot be wrong, and therefore cannot teach
        anything.

        Cached on rounded inputs rather than throttled on time -- the
        equilibrium depends only on wind and length, so two identical
        conditions must give the same answer no matter when they arrive.
        """
        if wind <= 0.3:
            return None
        key = (round(wind, 2), round(length, 1))
        hit = self._pred_cache.get(key)
        if hit is not None:
            return hit

        try:
            from model import equilibrium
            state, tension, elev = equilibrium(self.kite, self.tether, wind, length)
        except Exception:                                      # noqa: BLE001
            return None
        # Two different non-answers, and both have to be caught. The solver
        # returns NaN when no equilibrium exists at all, and NaN fails every
        # comparison -- `elev < 2.0` is False for it, so a bare angle check
        # let a frame of NaNs through to the browser. Below the horizon it
        # returns a real root, but the outward-lift bridle assumption does
        # not describe a kite there either.
        out = None if (not math.isfinite(elev) or elev < 2.0) else {
            "elevation": round(float(elev), 2),
            "tension": round(float(tension), 3),
            "x": round(float(state[0]), 3),
            "z": round(float(state[1]), 3),
            "span": round(float((state[0] ** 2 + state[1] ** 2) ** 0.5), 3),
        }
        if len(self._pred_cache) > 4000:
            self._pred_cache.clear()
        self._pred_cache[key] = out
        return out

    def _quasi_static(self):
        """True when the force balance the inversion assumes actually holds.

        Thresholds are sim/twin.py's. Rates are taken across the whole window
        rather than between adjacent samples, because differentiating a noisy
        signal sample-to-sample measures the sensor noise and nothing else --
        that mistake rejected every sample the first time it was written.
        """
        if len(self._recent) < self._recent.maxlen:
            return False
        t0, T0, e0 = self._recent[0]
        t1, T1, e1 = self._recent[-1]
        dt = t1 - t0
        if dt < 0.5:
            return False
        return (abs(T1 - T0) / dt < STEADY_TENSION_RATE
                and abs(e1 - e0) / dt < STEADY_ELEVATION_RATE)

    def _coefficients(self, length, tension, chord, wind):
        """Force-balance inversion, same algebra as sim/twin.py invert_rows."""
        if tension <= 1e-6:
            return None, None, None

        cos_c, sin_c = math.cos(chord), math.sin(chord)
        weight = self.tether.mass_per_m * length * G

        cl = cd = None
        if wind > 0.5:
            drag = (0.25 * RHO_AIR * (wind * sin_c) ** 2
                    * self.tether.diameter * length * 1.1)
            aero_x = tension * cos_c - drag * sin_c
            aero_z = tension * sin_c + drag * cos_c + 0.5 * weight + self.kite.mass * G
            q = 0.5 * RHO_AIR * wind * wind * self.kite.area
            if q > 0:
                cl, cd = aero_z / q, aero_x / q

        # Inverse direction: what wind would the nominal coefficients need to
        # produce this tension? Disagreement with the anemometer is the single
        # most useful live check there is, because the kite sits in a different
        # part of the boundary layer than the mast does.
        #
        # Solved from the full balance, not T = q*sqrt(CL^2+CD^2). Dropping the
        # mg term looks harmless but is not: lift has to carry the weight before
        # any of it reaches the line, so ignoring it reads low at every speed
        # and worst in light wind, exactly where the check matters.
        #
        #   T^2 = (q*CD)^2 + (q*CL - mg)^2,   q = 0.5*rho*A*v^2
        #   -> q^2 (CL^2+CD^2) - 2 q CL mg + (mg^2 - T^2) = 0
        wind_est = None
        kl, kd = self.kite.cl, self.kite.cd
        mg = self.kite.mass * G
        a = kl * kl + kd * kd
        disc = (kl * mg) ** 2 - a * (mg * mg - tension * tension)
        if a > 0 and disc >= 0:
            q = (kl * mg + math.sqrt(disc)) / a
            denom = 0.5 * RHO_AIR * self.kite.area
            if q > 0 and denom > 0:
                wind_est = math.sqrt(q / denom)
        return cl, cd, wind_est

    def _power(self, row, tilt):
        pmax = row.get("pv_pmax")
        if not row.get("pv_valid") or pmax is None:
            return {"valid": False}
        incidence = abs(tilt)
        cosi = math.cos(math.radians(min(incidence, 85.0)))
        return {
            "valid": True,
            "pmax": round(pmax, 4),
            "vmp": round(row.get("pv_vmp") or 0.0, 3),
            "imp": round(row.get("pv_imp") or 0.0, 4),
            "voc": round(row.get("pv_voc") or 0.0, 3),
            "isc": round(row.get("pv_isc") or 0.0, 4),
            "points": row.get("pv_points"),
            "incidence": round(incidence, 1),
            # Attitude and power ride in the same packet precisely so this
            # division is legitimate; normalising against a stale tilt would
            # be worse than not normalising at all.
            "normalised": round(pmax / cosi, 4) if cosi > 0.05 else None,
            "fill": (round(pmax / (row["pv_voc"] * row["pv_isc"]), 3)
                     if row.get("pv_voc") and row.get("pv_isc") else None),
        }

    def _link(self, row):
        seq = row.get("seq")
        gap = 0
        if seq is not None and self._last_seq is not None and seq > self._last_seq + 1:
            gap = seq - self._last_seq - 1
        if seq is not None:
            self._last_seq = seq
        return {
            "received": self._seen,
            "dropped": row.get("dropped") or 0,
            "gap": gap,
            "imu_age_ms": row.get("imu_age_ms") or 0,
            "imu_fields": row.get("imu_fields") or 0,
            "fix": row.get("fix") or 0,
            "sats": row.get("sats") or 0,
        }


def parse_row(fields):
    """CSV fields -> typed dict. Returns None for comments and short lines."""
    if len(fields) < len(COLUMNS):
        return None
    row = {}
    for name, raw in zip(COLUMNS, fields):
        raw = raw.strip()
        try:
            row[name] = float(raw) if name in FLOATS else int(float(raw))
        except ValueError:
            return None
    return row


# --------------------------------------------------------------------------
# sources
# --------------------------------------------------------------------------

def source_serial(port, baud, publish, derive, stop):
    import serial

    if not port:
        raise RuntimeError("포트가 지정되지 않았습니다")

    # serial_for_url, not Serial: it takes a real port name ("COM5",
    # "/dev/ttyUSB0") exactly the same way, and additionally accepts
    # "socket://host:port", which is what lets the loopback test drive this
    # function through the real pyserial stack instead of a stand-in.
    # Reconnect rather than die. A USB cable gets knocked out, the ESP32 gets
    # reflashed, the ground station is power-cycled -- none of that should
    # mean the operator has to go back to a terminal and restart the bridge
    # while the kite is still in the air.
    while not stop.is_set():
        try:
            _read_serial(serial, port, baud, publish, derive, stop)
        except Exception as exc:                         # noqa: BLE001
            if stop.is_set():
                return
            print(f"[bridge] {port} 재접속 대기 ({type(exc).__name__}: {exc})")
        if stop.is_set():
            return
        # Long enough not to spin on a port that is simply absent, short
        # enough that replugging the cable comes back on its own.
        stop.wait(2.0)


def _read_serial(serial, port, baud, publish, derive, stop):
    # The close() inside `with` can itself raise when the far end has already
    # gone, so the guard has to wrap the whole block -- not just the reads.
    # Without this a dropped link prints a raw traceback instead of a line
    # saying the link dropped.
    ser = serial.serial_for_url(port, baudrate=baud, timeout=1.0)
    print(f"[bridge] {port} @ {baud} 열림")
    try:
        while not stop.is_set():
            try:
                line = ser.readline().decode("utf-8", "replace").strip()
            except Exception as exc:                     # noqa: BLE001
                if not stop.is_set():
                    print(f"[bridge] 시리얼 끊김: {exc}")
                break
            if not line or line.startswith("#") or line.startswith("t_ms"):
                if line.startswith("#"):
                    print(f"[rig] {line}")
                continue
            row = parse_row(line.split(","))
            if row:
                publish(derive(row, time.time()))
    finally:
        try:
            ser.close()
        except Exception:                                # noqa: BLE001
            pass


def source_replay(path, publish, derive, stop, speed=1.0):
    if not path:
        raise RuntimeError("재생할 기록이 지정되지 않았습니다")
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for fields in csv.reader(fh):
            if not fields or fields[0].startswith(("#", "t_ms")):
                continue
            row = parse_row(fields)
            if row:
                rows.append(row)
    if not rows:
        print(f"[bridge] {path}: 재생할 행이 없습니다")
        return
    print(f"[bridge] {path} 재생: {len(rows)}행")

    t0 = rows[0]["t_ms"]
    started = time.time()
    while not stop.is_set():
        for row in rows:
            if stop.is_set():
                return
            target = started + (row["t_ms"] - t0) / 1000.0 / speed
            delay = target - time.time()
            if delay > 0:
                time.sleep(min(delay, 1.0))
            publish(derive(row, time.time()))
        started = time.time()
        print("[bridge] 재생 반복")


# Simulator settings the UI can change. Names match the report's symbols so a
# reader can tie a slider to the equation it feeds.
SIM_DEFAULTS = {
    "wind": 6.0,            # m/s at the 10 m reference height
    "turbulence": 0.12,     # sigma_u / U; 0 disables
    "shear": 1,             # power-law boundary layer on/off
    "gust_ratio": 1.0,      # peak / base; 1 disables
    "gust_at": 20.0,        # s
    "gust_dur": 1.5,        # s
    "line_length": 50.0,    # m
    "ea": 3000.0,           # N
    "damping": 0.03,
    "area": 0.67,           # m^2
    "mass_canopy": 0.15,    # kg
    "mass_payload": 0.30,   # kg
    "suspension": 0.50,     # m
    "cl": 0.75,
    "cd": 0.38,
    "seed": 7,
}

_SIM_CACHE = {}


def simulated_rows(rate=20.0, window=120.0, cfg=None):
    """Pre-integrate the real two-body model and yield ground-station rows.

    The integration is cached per (rate, window): switching sources from the
    UI would otherwise pay ~30 s of Radau every time someone toggles back.

    Separated from the streaming so the fake rig can emit the same physics
    over a serial link in the firmware's own wire format. If the fake rig
    invented its own numbers, testing against it would prove nothing about
    the path the real rig takes.

    Integrating inside the streaming loop would couple the solver's step size
    to the browser's frame rate; integrating ahead keeps the physics honest and
    the output steady. The window loops, which is visible in the time axis and
    is fine for an instrument check.
    """
    import numpy as np
    from scipy.integrate import solve_ivp

    import wind as wind_module
    from payload import equilibrium2, rhs

    cfg = {**SIM_DEFAULTS, **(cfg or {})}
    WIND_BASE = float(cfg["wind"])
    LINE_LENGTH = float(cfg["line_length"])
    SUSPENSION_LENGTH = float(cfg["suspension"])

    # The default 0.50 m^2 kite is marginal by this project's own analysis --
    # 14% of the uncertainty samples never fly, and a turbulent lull at
    # 6 m/s is enough to drop it out of the sky. The report recommends
    # 0.67 m^2, so the demo flies the recommended kite rather than one that
    # spends part of the window on the ground.
    key = (round(rate, 3), round(window, 3),
           tuple(sorted((k, round(float(v), 6)) for k, v in cfg.items())))
    cached = _SIM_CACHE.get(key)

    kite = replace(Kite(), area=float(cfg["area"]),
                   mass_canopy=float(cfg["mass_canopy"]),
                   cl=float(cfg["cl"]), cd=float(cfg["cd"]))
    payload = replace(Payload(), mass=float(cfg["mass_payload"]))
    tether = replace(SILK, EA=float(cfg["ea"]),
                     damping_ratio=float(cfg["damping"]))
    if cached is None:
        print("[bridge] 시뮬 모드: 2체 모델 적분 중 ...")

        # turbulence() wraps a base field multiplicatively rather than being added
        # to one, so the shear profile goes inside it. Composing them with add()
        # would stack two full wind fields and double the speed.
        base = (wind_module.shear(WIND_BASE) if cfg["shear"]
                else wind_module.uniform(WIND_BASE))
        if float(cfg["turbulence"]) > 0:
            base = wind_module.turbulence(
                base, intensity=float(cfg["turbulence"]), speed=WIND_BASE,
                duration=window + 5.0, seed=int(cfg["seed"]),
            )
        if float(cfg["gust_ratio"]) > 1.0:
            base = wind_module.gust(base, float(cfg["gust_ratio"]),
                                    float(cfg["gust_at"]),
                                    float(cfg["gust_dur"]))
        field = base
        # Trim against the field the run will actually use. Seeding from a uniform
        # WIND_BASE instead leaves a real startup transient, during which the
        # force-balance inversion is not valid and the UI would report a CD that is
        # simply wrong.
        state0 = equilibrium2(kite, payload, tether, SUSPENSION_LENGTH,
                              field, LINE_LENGTH)
        n = int(window * rate)
        t_eval = np.linspace(0.0, window, n)
        sol = solve_ivp(
            rhs, (0.0, window), state0,
            args=(kite, payload, tether, SUSPENSION_LENGTH, field,
                  lambda t: LINE_LENGTH),
            method="Radau", t_eval=t_eval, rtol=1e-6, atol=1e-8, max_step=0.1,
        )
        if not sol.success:
            print(f"[bridge] 적분 실패: {sol.message}")
            return

        from model import tension as tension_of

        rk, rp, vk = sol.y[0:2].T, sol.y[2:4].T, sol.y[4:6].T
        radius = np.linalg.norm(rk, axis=1)
        rhat = rk / radius[:, None]
        rate_r = np.einsum("ij,ij->i", vk, rhat)
        T = np.array([tension_of(radius[i], rate_r[i], LINE_LENGTH, tether,
                                 kite.mass_canopy + payload.mass)
                      for i in range(n)])
        _ = vk
        chord = np.degrees(np.arctan2(rk[:, 1], rk[:, 0]))
        # Below the horizon the outward-lift bridle assumption stops describing a
        # real kite, so those samples are not telemetry, they are an artefact.
        # Say so rather than streaming them as if they were a flight.
        if chord.min() < 5.0:
            bad = 100.0 * (chord < 5.0).mean()
            print(f"[bridge] 경고: 구간의 {bad:.0f}%에서 고도각이 5° 아래입니다 "
                  f"(최저 {chord.min():.1f}°). 풍속/면적을 올리세요.")
        link = rp - rk
        swing = np.degrees(np.arctan2(link[:, 0], -link[:, 1]))
        speed = np.array([np.linalg.norm(field(t, rk[i])) for i, t in enumerate(sol.t)])

        # The sensors do not see the state, they see the state plus their own
        # error. Feeding clean values would make the UI look better than the rig
        # ever will, and would hide exactly the noise the corrections fight.
        rng = np.random.default_rng(11)
        # Same 20 deg limit the Deriver applies, and for the same reason: the
        # small-sag form is 1/T, so as the line slackens it runs away. Without
        # the cap here the synthetic angle sensor reported tangents like
        # -236 deg, which no sensor on a real rig could produce -- the fake
        # rig was manufacturing data the hardware cannot emit.
        sag = np.minimum(
            np.array([sag_angle(T[i], LINE_LENGTH, np.radians(chord[i]), tether)
                      for i in range(n)]),
            MAX_SAG)
        elev_meas = chord - np.degrees(sag) + rng.normal(0, 0.4, n)
        T_meas = T * (1 + rng.normal(0, 0.03, n))
        wind_meas = speed * (1 + rng.normal(0, 0.04, n))
        _SIM_CACHE[key] = (sol.t, T_meas, elev_meas, wind_meas,
                           swing, rk, n)
    sol_t, T_meas, elev_meas, wind_meas, swing, rk, n = _SIM_CACHE[key]

    # Separate stream from the one used to build the arrays, so a cache hit and
    # a cache miss produce the same per-sample noise instead of diverging
    # depending on whether this window had been integrated before.
    rng_live = np.random.default_rng(23)

    print(f"[bridge] 준비 완료 — {window:.0f} s, {n}샘플, {rate:.0f} Hz 재생")

    seq = 0
    sweep_every = int(rate * 2)
    while True:
        for i in range(n):
            tilt = swing[i]
            row = {
                "t_ms": int(sol_t[i] * 1000), "line_length_m": LINE_LENGTH,
                "tension_n": float(T_meas[i]),
                "elevation_deg": float(elev_meas[i]),
                "wind_ms": float(wind_meas[i]),
                "rx_ms": int(time.time() * 1000) % 10**9, "seq": seq,
                "uptime_ms": int(sol_t[i] * 1000),
                "roll": float(rng_live.normal(0, 1.2)), "pitch": float(tilt),
                "yaw": float(rng_live.normal(0, 2.5)),
                "fix": 1, "sats": 9, "lat": 37.5665, "lon": 126.9780,
                "alt_gps": float(rk[i, 1]) + rng_live.normal(0, 4.0),
                "gps_time": 0, "imu_age_ms": int(rng_live.integers(3, 14)),
                "imu_fields": 3, "dropped": 0,
            }
            if seq % sweep_every == 0:
                irr = max(0.0, math.cos(math.radians(min(abs(tilt), 85.0))))
                isc = 0.42 * (0.85 + 0.3 * irr)
                voc = 4.1 + rng_live.normal(0, 0.02)
                row.update({
                    "pv_sweep_ms": row["uptime_ms"], "pv_pmax": 0.78 * voc * isc,
                    "pv_vmp": 0.82 * voc, "pv_imp": 0.95 * isc,
                    "pv_voc": voc, "pv_isc": isc, "pv_points": 37, "pv_valid": 1,
                })
            else:
                row.update({"pv_sweep_ms": 0, "pv_pmax": 0.0, "pv_vmp": 0.0,
                            "pv_imp": 0.0, "pv_voc": 0.0, "pv_isc": 0.0,
                            "pv_points": 0, "pv_valid": 0})
            yield row
            seq += 1


def source_sim(publish, derive, stop, rate=20.0, window=120.0, cfg=None):
    period = 1.0 / rate
    next_t = time.time()
    for row in simulated_rows(rate=rate, window=window, cfg=cfg):
        if stop.is_set():
            return
        publish(derive(row, time.time()))
        next_t += period
        delay = next_t - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.time()


def format_row(row):
    """Render a row the way firmware/ground/src/main.cpp prints it.

    Formats come from COLUMN_FORMATS, which the test suite checks against the
    firmware's actual printf. Keeping the widths right matters: a float
    emitted with more digits than the ESP32 produces would let the parser pass
    on data the real rig never sends.
    """
    return ",".join(COLUMN_FORMATS[c] % row[c] for c in COLUMNS)


# The calibration constants the rig actually applies. Parsed out of the
# firmware rather than restated here: a copy in two places is a copy that
# drifts, and the whole reason the monitor shows these is so the person
# reading a coefficient knows what produced it.
SENSOR_CONSTANTS = (
    ("TENSION_PER_COUNT", "장력 이득", "N/count",
     "로드셀이 아니라 BTS7960 전류센싱. 기지 하중으로 교정 필요"),
    ("ELEVATION_PER_COUNT", "각도 이득", "deg/count", "지상국 각도센서"),
    ("ELEVATION_OFFSET", "각도 오프셋", "deg", "수평계·다림추 기준으로 교정"),
    ("WIND_PER_HZ", "풍속 이득", "(m/s)/Hz",
     "식별 정확도를 가장 크게 좌우하는 값 — C_L 흩어짐의 26 %"),
    ("ESPNOW_CHANNEL", "ESP-NOW 채널", "", "두 노드가 같아야 한다"),
)


def read_sensor_config():
    """Calibration constants as the firmware defines them."""
    src_path = (HERE.parent / "firmware" / "ground" / "src" / "main.cpp")
    out = {"source": "firmware/ground/src/main.cpp", "values": [], "error": None}
    try:
        src = src_path.read_text(encoding="utf-8")
    except OSError as exc:
        out["error"] = f"펌웨어 소스를 읽지 못했습니다: {exc}"
        return out

    import re
    for name, label, unit, note in SENSOR_CONSTANTS:
        m = re.search(
            r"static\s+const\s+\w+\s+" + name + r"\s*=\s*([-\d.]+)", src)
        out["values"].append({
            "name": name, "label": label, "unit": unit, "note": note,
            "value": float(m.group(1)) if m else None,
        })
    return out


class SourceManager:
    """Holds the sources the bridge was started with, and switches between them.

    One runs at a time. Switching stops the current reader, builds a fresh
    Deriver and clears the last frame: the quasi-static window and the CL/CD
    medians belong to one source, and carrying them across a switch would mix
    a real flight's samples into a simulated one.
    """

    def __init__(self, hub, specs, default, sim_cfg=None, record=True):
        self.hub = hub
        self.specs = specs            # name -> dict(label, live, kind, make, kite, tether)
        self.sim_cfg = dict(sim_cfg or SIM_DEFAULTS)
        self.record = record
        self._recorder = None
        self.current = None
        self.status = "idle"
        self.error = None
        self._stop = None
        self._thread = None
        self._lock = threading.Lock()
        self.switch(default)

    def meta(self):
        spec = self.specs[self.current]
        return {
            "mode": spec["label"],
            "live": spec["live"],
            "source": spec["kind"],
            "current": self.current,
            "status": self.status,
            "error": self.error,
            "sources": [{"name": n, "label": v["label"], "kind": v["kind"],
                         "live": v["live"], "port": v.get("port")}
                        for n, v in self.specs.items()],
            "port": (self.specs.get("serial") or {}).get("port"),
            "sim_cfg": self.sim_cfg,
            "recording": (None if not self._recorder or not self._recorder.path
                          else {"file": self._recorder.path.name,
                                "rows": self._recorder.rows}),
            "kite": {"area": spec["kite"].area, "mass": spec["kite"].mass,
                     "cl": spec["kite"].cl, "cd": spec["kite"].cd},
            "tether": {"name": spec["tether"].name, "EA": spec["tether"].EA,
                       "break_load": spec["tether"].break_load},
        }

    def set_port(self, port):
        """Point the serial source at a different port."""
        if "serial" not in self.specs:
            raise KeyError("시리얼 소스가 없습니다")
        self.specs["serial"]["port"] = port
        self.specs["serial"]["label"] = (
            ("모의 리그 " + port) if "://" in port else f"실측 {port}")
        emulated = "://" in port
        self.specs["serial"]["live"] = not emulated
        self.specs["serial"]["kind"] = "emulated" if emulated else "hardware"
        remembered = [p for p in load_state().get("ports", []) if p != port]
        save_state(ports=([port] + remembered)[:8], last_port=port)
        return self.switch("serial")

    def set_replay(self, name):
        """Play back one of the recordings in station/logs."""
        target = (LOG_DIR / name).resolve()
        # Same guard as the static files: a name is not a path, and this one
        # arrives over HTTP.
        try:
            target.relative_to(LOG_DIR.resolve())
        except ValueError:
            raise KeyError("기록 폴더 밖의 파일입니다")
        if not target.is_file():
            raise KeyError(f"기록을 찾을 수 없습니다: {name}")
        if "replay" not in self.specs:
            raise KeyError("재생 소스가 없습니다")
        self.specs["replay"]["path"] = str(target)
        self.specs["replay"]["label"] = f"재생 {name}"
        return self.switch("replay")

    def set_sim_cfg(self, patch):
        """Apply simulator settings. Restarts the model source if it is live.

        Unknown keys are rejected rather than ignored: a typo that silently
        does nothing is the kind of thing someone spends an afternoon on.
        """
        unknown = set(patch) - set(SIM_DEFAULTS)
        if unknown:
            raise KeyError(f"알 수 없는 설정: {', '.join(sorted(unknown))}")
        cfg = {**self.sim_cfg}
        for k, v in patch.items():
            cfg[k] = float(v)
        self.sim_cfg = cfg
        save_state(sim_cfg=cfg)
        # The kite area is a simulator setting too, so the deriver that
        # inverts those frames has to follow it or every CL it reports is
        # scaled by the mismatch.
        if "sim" in self.specs:
            self.specs["sim"]["kite"] = replace(
                self.specs["sim"]["kite"], area=cfg["area"], cl=cfg["cl"],
                cd=cfg["cd"], mass_canopy=cfg["mass_canopy"],
                mass_payload=cfg["mass_payload"])
            self.specs["sim"]["tether"] = replace(
                self.specs["sim"]["tether"], EA=cfg["ea"],
                damping_ratio=cfg["damping"])
        if self.current == "sim":
            return self.switch("sim")
        return self.meta()

    def switch(self, name):
        if name not in self.specs:
            raise KeyError(name)
        with self._lock:
            if self._stop:
                self._stop.set()
            # The old reader may be blocked in a 1 s serial read, so it is not
            # joined -- it sees the flag and exits on its own. It publishes to
            # a hub that no longer has it as current, so nothing it emits in
            # the meantime reaches the browser.
            self.current = name
            self.status = "starting"
            self.error = None
            self.hub.latest = None
            spec = self.specs[name]
            derive = Deriver(spec["kite"], Payload(), spec["tether"])
            stop = threading.Event()
            self._stop = stop

            # Identity, not a counter: two switches in the same millisecond
            # still get distinct tokens, and `is` cannot collide.
            token = object()
            self.hub.token = token
            hub = self.hub

            if self._recorder:
                self._recorder.close()
            # Replaying a recording and recording it again just fills the list
            # with near-duplicates of a file that already exists.
            self._recorder = (Recorder(LOG_DIR, name)
                              if self.record and name != "replay" else None)
            recorder = self._recorder

            def publish(frame):
                # Bookkeeping must never be able to kill a source. pop() with
                # no default raised KeyError for any frame that did not carry
                # a raw row, and that exception propagated out of the reader
                # thread -- a recording detail taking the telemetry with it.
                raw = frame.pop("_row", None)
                if recorder is not None and raw is not None:
                    recorder.write(raw)
                hub.publish(frame, token)
                if self.status == "starting" and self.current == name:
                    self.status = "running"

            # The manager is passed in rather than captured: switch() runs from
            # __init__, before the `sources = SourceManager(...)` assignment has
            # completed, so a lambda closing over that name raises NameError and
            # the source never starts.
            gen = spec["make"](derive, stop, publish, self)

            def run():
                try:
                    gen()
                except Exception as exc:                       # noqa: BLE001
                    # Only the source that is still current may report status:
                    # a thread torn down by a toggle must not overwrite the
                    # state of the source that replaced it.
                    if not stop.is_set() and self.current == name:
                        self.status = "error"
                        self.error = f"{type(exc).__name__}: {exc}"
                        print(f"[bridge] {name} 실패: {self.error}")
                else:
                    if not stop.is_set() and self.current == name:
                        self.status = "ended"

            self._thread = threading.Thread(target=run, daemon=True)
            self._thread.start()
            print(f"[bridge] 소스 전환 -> {spec['label']}")
            return self.meta()

    def tick(self):
        if self.status == "starting" and self.hub.latest:
            self.status = "running"


# --------------------------------------------------------------------------
# http
# --------------------------------------------------------------------------

def make_handler(hub, sources):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _static(self, path):
            """Serve the UI's own files, and nothing else.

            Resolved against HERE and checked with is_relative_to, so "..",
            absolute paths and symlinks cannot reach outside the station
            directory. Localhost-only is not a reason to skip this.
            """
            rel = path.lstrip("/")
            if not rel:
                rel = "monitor.html"
            try:
                target = (HERE / rel).resolve()
                target.relative_to(HERE.resolve())
            except (ValueError, OSError):
                self._send(403, b"forbidden", "text/plain")
                return
            if target.suffix not in CONTENT_TYPES or not target.is_file():
                self._send(404, b"not found", "text/plain")
                return
            self._send(200, target.read_bytes(), CONTENT_TYPES[target.suffix])

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._static("monitor.html")
            elif path == "/api/meta":
                sources.tick()
                self._send(200, json.dumps(sources.meta(), ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            elif path == "/api/ports":
                cur = (sources.specs.get("serial") or {}).get("port")
                ports = list_ports()
                # The port in use is always offered, even when it came from a
                # command-line flag and was never saved. Leaving it out of its
                # own list is how a working connection looks unavailable.
                if cur and not any(p["port"] == cur for p in ports):
                    ports.insert(0, {
                        "port": cur, "label": cur, "present": True,
                        "kind": "emulated" if "://" in cur else "serial"})
                self._send(200,
                           json.dumps({"ports": ports, "current": cur},
                                      ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            elif path == "/api/logs":
                items = []
                try:
                    for f in sorted(LOG_DIR.glob("*.csv"), reverse=True)[:40]:
                        st = f.stat()
                        items.append({"name": f.name, "kb": round(st.st_size / 1024, 1),
                                      "when": time.strftime(
                                          "%m-%d %H:%M", time.localtime(st.st_mtime))})
                except OSError:
                    pass
                self._send(200, json.dumps({"logs": items}, ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            elif path == "/api/sensors":
                self._send(200,
                           json.dumps(read_sensor_config(), ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            elif path == "/api/sim/params":
                self._send(200,
                           json.dumps({"defaults": SIM_DEFAULTS,
                                       "current": sources.sim_cfg},
                                      ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            elif path == "/api/state":
                body = json.dumps(hub.latest or {}, ensure_ascii=False).encode()
                self._send(200, body, "application/json; charset=utf-8")
            elif path == "/stream":
                self._stream()
            else:
                self._static(path)

        def do_POST(self):
            route = self.path.split("?")[0]
            if route not in ("/api/source", "/api/sim/params", "/api/port",
                             "/api/replay"):
                self._send(404, b"not found", "text/plain")
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if route == "/api/sim/params":
                    meta = sources.set_sim_cfg(body)
                elif route == "/api/port":
                    meta = sources.set_port(body["port"])
                elif route == "/api/replay":
                    meta = sources.set_replay(body["name"])
                else:
                    meta = sources.switch(body["name"])
            except KeyError as exc:
                # Report what was actually wrong. A fixed "unknown source" on
                # a settings call sends the reader hunting in the wrong place.
                self._send(400,
                           json.dumps({"error": str(exc.args[0] if exc.args else exc)},
                                      ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")
            except Exception as exc:                            # noqa: BLE001
                self._send(500, json.dumps({"error": str(exc)}).encode(),
                           "application/json; charset=utf-8")
            else:
                self._send(200, json.dumps(meta, ensure_ascii=False).encode(),
                           "application/json; charset=utf-8")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q = hub.subscribe()
            try:
                self.wfile.write(b": connected\n\n")
                self.wfile.flush()
                while True:
                    try:
                        payload = q.get(timeout=10.0)
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")   # survives proxies
                        self.wfile.flush()
                        continue
                    self.wfile.write(b"data: " + payload.encode("utf-8") + b"\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.unsubscribe(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(
        description="지상국 텔레메트리 브리지",
        epilog="소스를 여러 개 주면 화면에서 버튼으로 전환할 수 있다. "
               "예: --serial COM5 --sim")
    # Not mutually exclusive any more: the point of the switch is to have the
    # real rig and the model side by side and flip between them mid-flight.
    ap.add_argument("--serial", metavar="PORT", help="지상국 ESP32 포트 (예: COM5)")
    ap.add_argument("--replay", metavar="CSV", help="기록된 로그 재생")
    ap.add_argument("--sim", action="store_true", help="검증된 모델로 생성")
    ap.add_argument("--default", choices=("serial", "replay", "sim"),
                    help="시작할 소스 (기본: 지정한 것 중 첫 번째)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="재생 배속")
    ap.add_argument("--port", type=int, default=8787, help="웹 포트")
    ap.add_argument("--window", type=float, default=120.0, help="시뮬 구간 [s]")
    # The kite you actually built, not the one in params.py. Area scales every
    # coefficient this reports: fly a 0.67 m^2 kite and derive with 0.50 and
    # CL comes out 34% high, with nothing on screen to say so. L/D survives,
    # because the area cancels in the ratio.
    ap.add_argument("--area", type=float, help="실측 날개 면적 [m²]")
    ap.add_argument("--mass", type=float, help="실측 전체 질량 [kg]")
    ap.add_argument("--ea", type=float, help="실측 계류줄 축강성 EA [N]")
    ap.add_argument("--no-record", action="store_true",
                    help="CSV 기록을 남기지 않음")
    args = ap.parse_args()

    if not (args.serial or args.replay or args.sim):
        ap.error("--serial, --replay, --sim 중 적어도 하나가 필요합니다")

    remembered = load_state()

    # Measured rig: params.py unless told otherwise.
    measured = Kite()
    if args.area:
        measured = replace(measured, area=args.area)
    if args.mass:
        measured = replace(measured, mass_canopy=args.mass - measured.mass_payload)
    tether = replace(SILK, EA=args.ea) if args.ea else SILK
    # The model flies the kite the report recommends (7.3절), which is not
    # necessarily the one that was built.
    modelled = replace(measured, area=args.area or 0.67)

    hub = Hub()
    specs = {}

    # The serial source is always offered, even with no port yet: "nothing is
    # plugged in" is a state the operator should be able to fix from the
    # screen, not a reason to restart the bridge with a different flag.
    serial_port = args.serial or remembered.get("last_port")
    emulated = bool(serial_port) and "://" in serial_port
    specs["serial"] = {
        "label": ("모의 리그 " + serial_port) if emulated
                 else (f"실측 {serial_port}" if serial_port else "실측 (포트 없음)"),
        "live": bool(serial_port) and not emulated,
        "kind": "emulated" if emulated else "hardware",
        "port": serial_port,
        "kite": measured, "tether": tether,
        "make": lambda d, st, pub, mgr: (lambda: source_serial(
            specs["serial"]["port"], args.baud, pub, d, st)),
    }
    # Replay is always offered too: recordings pile up on their own, and
    # picking one from the screen beats restarting with a path.
    specs["replay"] = {
        "label": (f"재생 {Path(args.replay).name}" if args.replay
                  else "재생 (기록 선택)"),
        "live": False, "kind": "replay",
        "path": args.replay,
        "kite": measured, "tether": tether,
        "make": lambda d, st, pub, mgr: (lambda: source_replay(
            specs["replay"]["path"], pub, d, st, args.speed)),
    }
    if args.sim:
        specs["sim"] = {
            "label": "시뮬레이션",
            "live": False, "kind": "model",
            "kite": modelled, "tether": tether,
            "make": lambda d, st, pub, mgr: (lambda: source_sim(
                pub, d, st, window=args.window, cfg=mgr.sim_cfg)),
        }

    if args.default in specs:
        default = args.default
    elif args.serial:
        default = "serial"
    elif args.replay:
        default = "replay"
    elif args.sim:
        default = "sim"
    else:
        default = "serial"
    sources = SourceManager(hub, specs, default,
                            sim_cfg={**SIM_DEFAULTS,
                                     **(remembered.get("sim_cfg") or {})},
                            record=not args.no_record)

    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 make_handler(hub, sources))
    server.daemon_threads = True
    print(f"[bridge] 소스 {'/'.join(specs)}  ->  http://127.0.0.1:{args.port}")
    if not args.no_record:
        print(f"[bridge] 기록 -> {LOG_DIR}")
    print("[bridge] Ctrl+C 로 종료")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[bridge] 종료")
    finally:
        if sources._stop:
            sources._stop.set()
        if sources._recorder:
            sources._recorder.close()
        server.shutdown()


if __name__ == "__main__":
    main()
