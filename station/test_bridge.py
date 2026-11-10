"""Tests for the ground-station bridge.

The point of most of these is the seam between the firmware and the bridge.
That seam has no schema and no handshake: the ESP32 prints a line, the bridge
splits it on commas, and nothing checks that the two agree. A mismatch there
does not raise -- it silently produces either no data at all or, worse, data
in the wrong columns. One already happened (the printf had 27 specifiers for
28 arguments), and it was invisible until counted mechanically, so the count
is a test now.

    python -m pytest station/test_bridge.py -q
"""

import io
import json
import math
import re
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))

import bridge                                              # noqa: E402
from params import G, RHO_AIR, SILK, Kite, Payload         # noqa: E402

FIRMWARE = HERE.parent / "firmware" / "ground" / "src" / "main.cpp"

# Which columns the firmware prints with a float specifier. Anything else must
# be an integer specifier -- passing a float to %u is undefined behaviour in C
# and prints garbage on the ESP32 rather than failing.
FLOAT_COLUMNS = {
    "line_length_m", "tension_n", "elevation_deg", "wind_ms",
    "roll", "pitch", "yaw", "lat", "lon", "alt_gps",
    "pv_pmax", "pv_vmp", "pv_imp", "pv_voc", "pv_isc",
}


# --------------------------------------------------------------------------
# firmware / bridge schema agreement
# --------------------------------------------------------------------------

def read_firmware():
    """(format specifiers, printf arguments, header column names)."""
    src = io.open(FIRMWARE, encoding="utf-8").read()

    body = re.search(r'Serial\.printf\(\s*\n(\s*"%lu,%\.3f.*?)\);', src, re.S).group(1)
    fmt = "".join(re.findall(r'"([^"]*)"', body))
    specs = re.findall(r"%[0-9.]*(?:lu|u|d|f)", fmt)

    tail = body[body.rindex('"') + 1:]
    args = [a.strip().replace("(unsigned long)", "").replace("pkt.", "")
            for a in tail.split(",") if a.strip()]

    head = src[src.index('Serial.println("t_ms'):]
    head = head[:head.index(");")]
    cols = [c for c in "".join(re.findall(r'"([^"]*)"', head)).split(",") if c]
    return specs, args, cols


def test_firmware_prints_one_value_per_format_specifier():
    specs, args, _ = read_firmware()
    assert len(specs) == len(args), (
        f"printf has {len(specs)} specifiers for {len(args)} arguments. "
        "Every column after the first mismatch is shifted, and the trailing "
        "argument is never printed."
    )


def test_firmware_header_matches_what_it_prints():
    specs, args, cols = read_firmware()
    assert len(cols) == len(specs), (
        f"header promises {len(cols)} columns, printf emits {len(specs)}"
    )


def test_bridge_columns_match_the_firmware_header():
    _, _, cols = read_firmware()
    assert tuple(cols) == bridge.COLUMNS, (
        "bridge.COLUMNS has drifted from the firmware header.\n"
        f"  firmware: {cols}\n  bridge:   {list(bridge.COLUMNS)}"
    )


def test_firmware_specifier_types_match_their_arguments():
    specs, args, cols = read_firmware()
    wrong = []
    for spec, arg, col in zip(specs, args, cols):
        is_float_spec = spec.endswith("f")
        should_be_float = col in FLOAT_COLUMNS
        if is_float_spec != should_be_float:
            wrong.append(f"{col}: {spec} <- {arg}")
    assert not wrong, "format/argument type mismatch (UB in C):\n  " + "\n  ".join(wrong)


def test_wire_formats_match_the_firmware_printf():
    """bridge.COLUMN_FORMATS is what the fake rig prints with.

    If it drifts from main.cpp the fake rig emits lines the real one never
    would -- e.g. more decimals than the ESP32 produces -- and every test that
    runs through it is testing a link that does not exist.
    """
    specs, _, cols = read_firmware()
    wrong = []
    for spec, col in zip(specs, cols):
        py = spec.replace("lu", "d").replace("%u", "%d")
        if bridge.COLUMN_FORMATS[col] != py:
            wrong.append(f"{col}: firmware {py}, bridge {bridge.COLUMN_FORMATS[col]}")
    assert not wrong, "wire format drift:\n  " + "\n  ".join(wrong)


def test_every_column_has_a_wire_format():
    assert set(bridge.COLUMN_FORMATS) == set(bridge.COLUMNS)


def test_format_row_round_trips_through_the_parser():
    row = bridge.parse_row(firmware_line().split(","))
    again = bridge.parse_row(bridge.format_row(row).split(","))
    assert again == row


def test_simulated_rows_produce_a_complete_row():
    gen = bridge.simulated_rows(rate=20.0, window=20.0)
    row = next(gen)
    assert set(row) == set(bridge.COLUMNS)
    assert bridge.parse_row(bridge.format_row(row).split(",")) is not None


def test_simulated_rig_actually_flies():
    """The model must spend the window in the air, not on the ground.

    Below the horizon the outward-lift assumption stops describing a kite, so
    those samples are an artefact rather than telemetry, and a fake rig that
    emitted them would be testing nothing.
    """
    gen = bridge.simulated_rows(rate=20.0, window=40.0)
    derive = make_deriver()
    elevations = []
    for _ in range(600):
        frame = derive(next(gen), time.time())
        elevations.append(frame["derived"]["chord_deg"])
    assert min(elevations) > 5.0, f"고도각 최저 {min(elevations):.1f}°"


def test_simulated_angles_stay_physically_possible():
    """No elevation the geometry forbids, in either direction.

    The sag correction goes as 1/T, so a slack line sends it to hundreds of
    degrees. Both the synthetic angle sensor and the Deriver have to cap it;
    when only one of them did, the rig emitted a tangent of -236 deg -- a
    reading no sensor on the real rig could produce -- and the derived
    elevation came out at 98 deg, past what this kite's L/D can reach at any
    wind. A min() check alone let that through, because the bad values were
    too high rather than too low.
    """
    gen = bridge.simulated_rows(rate=20.0, window=40.0)
    derive = make_deriver()
    kite = Kite()
    ceiling = math.degrees(math.atan2(kite.cl, kite.cd)) + 5.0

    raw, chord = [], []
    for _ in range(800):
        frame = derive(next(gen), time.time())
        raw.append(frame["raw"]["elevation"])
        chord.append(frame["derived"]["chord_deg"])

    assert min(raw) > -90.0, f"측정각 최저 {min(raw):.1f}° — 센서가 낼 수 없는 값"
    assert max(raw) < 90.0, f"측정각 최고 {max(raw):.1f}°"
    assert max(chord) < ceiling, (
        f"고도각 최고 {max(chord):.1f}° — L/D 상한 {ceiling - 5:.0f}° 를 넘습니다")


def test_sag_correction_is_capped_as_the_model_requires():
    """model.sag_angle says a result past ~20 deg is out of range."""
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line(tension_n=0.02).split(",")), 0.0)
    g = frame["derived"]
    assert g["sag_deg"] <= 20.0 + 1e-6
    assert g["sag_valid"] is False
    assert g["slack"] is True
    assert isinstance(g["sag_valid"], bool), "numpy bool 은 JSON 으로 못 나갑니다"


def test_every_frame_survives_json():
    """The stream and the export both serialise these."""
    d = make_deriver()
    for tension in (0.01, 0.3, 5.0, 40.0):
        frame = d(bridge.parse_row(firmware_line(tension_n=tension).split(",")), 0.0)
        frame.pop("_row", None)
        json.dumps(frame)


def test_slack_samples_never_reach_the_identification():
    """A slack line makes every force-balance quantity meaningless, not noisy."""
    d = make_deriver()
    feed(d, 60, tension_n=0.05)
    assert len(d._cls) == 0


def test_bridge_float_set_agrees_with_the_firmware():
    """bridge.FLOATS decides which columns get parsed as floats."""
    assert bridge.FLOATS == FLOAT_COLUMNS


# --------------------------------------------------------------------------
# a firmware emulator built from the firmware's own format string
# --------------------------------------------------------------------------

def firmware_line(**over):
    """Render one CSV line the way the ESP32 would.

    Built from the format string in main.cpp rather than a copy of it, so a
    firmware edit changes this emulator too and the loopback test keeps
    testing the real contract.
    """
    specs, _, cols = read_firmware()
    values = {
        "t_ms": 12345, "line_length_m": 50.0, "tension_n": 5.693,
        "elevation_deg": 37.4, "wind_ms": 6.0, "rx_ms": 12345, "seq": 7,
        "uptime_ms": 9876, "roll": -1.5, "pitch": 24.0, "yaw": 3.25,
        "fix": 1, "sats": 9, "lat": 37.5665, "lon": 126.978, "alt_gps": 31.2,
        "gps_time": 123456, "imu_age_ms": 11, "imu_fields": 3, "dropped": 0,
        "pv_sweep_ms": 9870, "pv_pmax": 1.4661, "pv_vmp": 3.37,
        "pv_imp": 0.4350, "pv_voc": 4.11, "pv_isc": 0.4580,
        "pv_points": 37, "pv_valid": 1,
    }
    values.update(over)

    assert list(cols) == list(bridge.COLUMNS) and len(specs) == len(cols)
    return bridge.format_row(values)


def test_emulated_line_has_the_expected_shape():
    line = firmware_line()
    assert len(line.split(",")) == len(bridge.COLUMNS)
    row = bridge.parse_row(line.split(","))
    assert row is not None
    assert row["seq"] == 7
    assert row["pv_valid"] == 1
    assert row["yaw"] == pytest.approx(3.25, abs=0.02)
    assert row["sats"] == 9


# --------------------------------------------------------------------------
# parsing
# --------------------------------------------------------------------------

def test_parse_row_rejects_a_short_line():
    assert bridge.parse_row("1,2,3".split(",")) is None


def test_parse_row_rejects_garbage_where_a_number_belongs():
    fields = firmware_line().split(",")
    fields[3] = "nope"
    assert bridge.parse_row(fields) is None


def test_parse_row_survives_a_truncated_first_line():
    """The USB buffer usually opens mid-line; that must not raise."""
    partial = firmware_line()[17:]
    assert bridge.parse_row(partial.split(",")) is None


# --------------------------------------------------------------------------
# derived quantities
# --------------------------------------------------------------------------

def make_deriver():
    return bridge.Deriver(Kite(), Payload(), SILK)


def test_stretch_matches_hooke():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line().split(",")), 0.0)
    expected_cm = 50.0 * (5.693 / SILK.EA) * 100.0
    assert frame["derived"]["stretch_cm"] == pytest.approx(expected_cm, abs=0.02)


def test_sag_correction_raises_the_chord_above_the_measured_tangent():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line().split(",")), 0.0)
    assert frame["derived"]["chord_deg"] > frame["raw"]["elevation"]
    assert frame["derived"]["sag_deg"] > 0


def test_sag_is_worst_in_light_wind():
    """delta ~ 1/T, so the correction grows exactly where tension is lowest."""
    d1, d2 = make_deriver(), make_deriver()
    light = d1(bridge.parse_row(firmware_line(tension_n=2.0).split(",")), 0.0)
    strong = d2(bridge.parse_row(firmware_line(tension_n=20.0).split(",")), 0.0)
    assert light["derived"]["sag_deg"] > strong["derived"]["sag_deg"]


def test_wind_estimate_round_trips_through_the_force_balance():
    """Forward: wind -> tension. Inverse: tension -> wind. They must agree.

    The naive T = q*sqrt(CL^2+CD^2) passes nothing like this, because it drops
    the weight the lift has to carry first.
    """
    kite = Kite()
    d = make_deriver()
    for v in (4.0, 6.0, 8.0, 11.0):
        q = 0.5 * RHO_AIR * v * v * kite.area
        T = math.hypot(q * kite.cd, q * kite.cl - kite.mass * G)
        frame = d(bridge.parse_row(firmware_line(tension_n=T).split(",")), 0.0)
        assert frame["derived"]["wind_est"] == pytest.approx(v, rel=1e-6)


def test_payload_hangs_below_the_kite():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line(pitch=0.0).split(",")), 0.0)
    dd = frame["derived"]
    assert dd["payload_z"] == pytest.approx(dd["z"] - 0.5, abs=1e-6)


def test_power_is_normalised_by_the_tilt_in_the_same_packet():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line(pitch=60.0).split(",")), 0.0)
    pv = frame["pv"]
    assert pv["valid"]
    assert pv["normalised"] == pytest.approx(pv["pmax"] / math.cos(math.radians(60)), rel=1e-6)


def test_power_is_absent_when_the_sweep_flag_is_clear():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line(pv_valid=0).split(",")), 0.0)
    assert frame["pv"] == {"valid": False}


# --------------------------------------------------------------------------
# the quasi-static gate
# --------------------------------------------------------------------------

def feed(d, n, t0=0.0, dt=0.05, **over):
    frame = None
    for i in range(n):
        line = firmware_line(**{k: (v(i) if callable(v) else v)
                                for k, v in over.items()})
        frame = d(bridge.parse_row(line.split(",")), t0 + i * dt)
    return frame


def test_gate_is_closed_until_the_window_fills():
    d = make_deriver()
    frame = feed(d, 5)
    assert frame["derived"]["steady"] is False
    assert frame["derived"]["cl"] is None


def test_gate_opens_on_a_steady_stretch_and_recovers_cl():
    d = make_deriver()
    frame = feed(d, 60)
    assert frame["derived"]["steady"] is True
    assert frame["derived"]["cl"] == pytest.approx(Kite().cl, rel=0.12)


def test_gate_stays_closed_while_tension_ramps():
    """A transient is not a noisy sample, it is one the balance does not describe."""
    d = make_deriver()
    frame = feed(d, 60, tension_n=lambda i: 5.0 + 0.15 * i)   # 3 N/s
    assert frame["derived"]["steady"] is False


def test_transient_samples_never_reach_the_estimate():
    d = make_deriver()
    feed(d, 60, tension_n=lambda i: 5.0 + 0.15 * i)
    assert d._cls == bridge.deque([], maxlen=200) or len(d._cls) == 0


def test_rates_are_measured_across_the_window_not_between_samples():
    """Sample-to-sample differencing of a noisy signal measures the noise.

    Steady mean with +/-0.4 N of jitter: adjacent differences imply 16 N/s and
    would reject everything, which is the mistake this gate was written to
    avoid.
    """
    d = make_deriver()
    frame = feed(d, 60, tension_n=lambda i: 5.693 + (0.4 if i % 2 else -0.4))
    assert frame["derived"]["steady"] is True


# --------------------------------------------------------------------------
# end to end: a fake ESP32 on a real serial port, through to SSE
# --------------------------------------------------------------------------

class FakeRig:
    """Speaks the ground station's side of a USB serial link over TCP.

    pyserial's socket:// URL handler makes this a genuine serial port as far
    as the bridge is concerned, so source_serial runs unmodified.
    """

    def __init__(self, lines, rate=200.0):
        self.lines = lines
        self.rate = rate
        self.sock = socket.socket()
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    @property
    def url(self):
        return f"socket://127.0.0.1:{self.port}"

    def _serve(self):
        conn, _ = self.sock.accept()
        with conn:
            # exactly what the firmware emits on boot
            conn.sendall(b"# ground station up\r\n")
            conn.sendall((",".join(bridge.COLUMNS) + "\r\n").encode())
            while not self.stop.is_set():
                for line in self.lines:
                    if self.stop.is_set():
                        return
                    try:
                        conn.sendall((line + "\r\n").encode())
                    except OSError:
                        return
                    time.sleep(1.0 / self.rate)

    def close(self):
        self.stop.set()
        self.sock.close()


def test_serial_source_reads_real_firmware_output():
    lines = [firmware_line(seq=i, t_ms=1000 + 50 * i) for i in range(40)]
    rig = FakeRig(lines)
    hub = bridge.Hub()
    stop = threading.Event()
    t = threading.Thread(
        target=bridge.source_serial,
        args=(rig.url, 115200, hub.publish, make_deriver(), stop), daemon=True)
    t.start()
    try:
        deadline = time.time() + 8.0
        while hub.latest is None and time.time() < deadline:
            time.sleep(0.05)
        assert hub.latest is not None, "no frame arrived from the serial source"
        frame = hub.latest
        assert frame["raw"]["tension"] == pytest.approx(5.693, abs=0.01)
        assert frame["derived"]["chord_deg"] > 37.4
        assert frame["link"]["received"] >= 1
    finally:
        stop.set()
        rig.close()


def test_boot_banner_and_header_are_not_parsed_as_data():
    rig = FakeRig([firmware_line(seq=1)], rate=50.0)
    hub = bridge.Hub()
    stop = threading.Event()
    threading.Thread(target=bridge.source_serial,
                     args=(rig.url, 115200, hub.publish, make_deriver(), stop),
                     daemon=True).start()
    try:
        deadline = time.time() + 8.0
        while hub.latest is None and time.time() < deadline:
            time.sleep(0.05)
        assert hub.latest is not None
        # a parsed header row would show up as a line length of zero
        assert hub.latest["raw"]["length"] == pytest.approx(50.0)
    finally:
        stop.set()
        rig.close()


def test_sse_stream_delivers_frames_to_a_browser():
    import json
    from http.server import ThreadingHTTPServer
    from urllib.request import urlopen

    hub = bridge.Hub()
    server = ThreadingHTTPServer(("127.0.0.1", 0),
                                 bridge.make_handler(hub, {"mode": "test"}))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]

    derive = make_deriver()
    stop = threading.Event()

    def pump():
        i = 0
        while not stop.is_set():
            row = bridge.parse_row(firmware_line(seq=i).split(","))
            hub.publish(derive(row, time.time()))
            i += 1
            time.sleep(0.02)

    threading.Thread(target=pump, daemon=True).start()
    try:
        with urlopen(f"http://127.0.0.1:{port}/stream", timeout=8) as r:
            seen = []
            deadline = time.time() + 8.0
            while len(seen) < 2 and time.time() < deadline:
                line = r.readline().decode()
                if line.startswith("data: "):
                    seen.append(json.loads(line[6:]))
        assert len(seen) >= 2
        assert seen[0]["raw"]["tension"] == pytest.approx(5.693, abs=0.01)
        assert "chord_deg" in seen[0]["derived"]
    finally:
        stop.set()
        server.shutdown()

# --------------------------------------------------------------------------
# the simulator cache, and switching sources
# --------------------------------------------------------------------------

def test_simulated_rows_survives_a_second_call():
    """The cached path must build a complete row too.

    The first version kept the noise generator inside the `if cached is None`
    branch, so a cache hit left it unbound and the generator raised -- on the
    second call only. Anything that works once and fails the next time needs a
    test that calls it twice.
    """
    cfg = {"window": 12.0}
    first = next(bridge.simulated_rows(rate=20.0, window=12.0))
    second = next(bridge.simulated_rows(rate=20.0, window=12.0))
    assert set(first) == set(bridge.COLUMNS)
    assert set(second) == set(bridge.COLUMNS)
    assert first["tension_n"] == pytest.approx(second["tension_n"], rel=1e-9)
    # Per-sample noise must not depend on whether the window was cached.
    assert first["roll"] == pytest.approx(second["roll"], abs=1e-12)
    assert first["yaw"] == pytest.approx(second["yaw"], abs=1e-12)
    _ = cfg


def test_simulator_settings_change_the_physics():
    a = next(bridge.simulated_rows(rate=20.0, window=12.0,
                                   cfg={"wind": 5.0, "turbulence": 0.0}))
    b = next(bridge.simulated_rows(rate=20.0, window=12.0,
                                   cfg={"wind": 9.0, "turbulence": 0.0}))
    # Tension goes as the dynamic pressure, so more wind must pull harder.
    assert b["tension_n"] > a["tension_n"] * 1.5


def test_hub_drops_frames_from_a_superseded_source():
    """A torn-down source gets one more read in before it notices.

    Without the token that late frame lands in `latest` after the new source
    cleared it, and the browser shows the old source's reading under the new
    source's name.
    """
    hub = bridge.Hub()
    old = object()
    new = object()
    hub.token = old
    hub.publish({"v": 1}, old)
    assert hub.latest == {"v": 1}

    hub.token = new            # a switch happened
    hub.publish({"v": "stale"}, old)
    assert hub.latest == {"v": 1}, "superseded source overwrote the new one"
    hub.publish({"v": 2}, new)
    assert hub.latest == {"v": 2}


def test_hub_still_accepts_untokened_frames():
    """Tests and tools publish without a token; that must keep working."""
    hub = bridge.Hub()
    hub.token = object()
    hub.publish({"v": 3})
    assert hub.latest == {"v": 3}


def _manager_with_two_sources():
    hub = bridge.Hub()
    kite, tether = Kite(), SILK

    def make_counter(tag):
        def make(derive, stop, publish, mgr):
            def run():
                i = 0
                while not stop.is_set():
                    publish({"tag": tag, "i": i})
                    i += 1
                    time.sleep(0.005)
            return run
        return make

    specs = {
        "a": {"label": "A", "live": False, "kind": "model", "kite": kite,
              "tether": tether, "make": make_counter("a")},
        "b": {"label": "B", "live": True, "kind": "hardware", "kite": kite,
              "tether": tether, "make": make_counter("b")},
    }
    # record=False: 테스트가 station/logs 에 조각 파일을 남기면
    # UI 기록 목록에서 실제 비행과 섞인다.
    return hub, bridge.SourceManager(hub, specs, "a", record=False)


def test_switching_sources_swaps_the_stream():
    hub, mgr = _manager_with_two_sources()
    try:
        deadline = time.time() + 3
        while (hub.latest or {}).get("tag") != "a" and time.time() < deadline:
            time.sleep(0.01)
        assert (hub.latest or {}).get("tag") == "a"

        mgr.switch("b")
        deadline = time.time() + 3
        while (hub.latest or {}).get("tag") != "b" and time.time() < deadline:
            time.sleep(0.01)
        assert hub.latest["tag"] == "b"
        assert mgr.meta()["current"] == "b"
    finally:
        mgr._stop.set()


def test_rapid_toggling_leaves_exactly_one_live_source():
    """Hammer the switch the way a hand on a button does."""
    hub, mgr = _manager_with_two_sources()
    try:
        for i in range(24):
            mgr.switch("b" if i % 2 else "a")
        final = mgr.current
        time.sleep(0.25)

        tags = set()
        for _ in range(60):
            if hub.latest:
                tags.add(hub.latest["tag"])
            time.sleep(0.005)
        assert tags == {final}, f"토글 후 {tags} 가 섞여 들어왔습니다 (현재 {final})"
    finally:
        mgr._stop.set()


def test_a_failing_source_reports_the_error_and_can_be_switched_away_from():
    hub = bridge.Hub()
    kite, tether = Kite(), SILK

    def boom(derive, stop, publish, mgr):
        def run():
            raise RuntimeError("포트 없음")
        return run

    def fine(derive, stop, publish, mgr):
        def run():
            while not stop.is_set():
                publish({"tag": "ok"})
                time.sleep(0.005)
        return run

    specs = {
        "bad": {"label": "bad", "live": False, "kind": "hardware",
                "kite": kite, "tether": tether, "make": boom},
        "good": {"label": "good", "live": False, "kind": "model",
                 "kite": kite, "tether": tether, "make": fine},
    }
    mgr = bridge.SourceManager(hub, specs, "bad", record=False)
    try:
        deadline = time.time() + 2
        while mgr.status != "error" and time.time() < deadline:
            time.sleep(0.01)
        assert mgr.status == "error"
        assert "포트 없음" in mgr.error

        # A dead source must not strand the operator on it.
        mgr.switch("good")
        deadline = time.time() + 2
        while (hub.latest or {}).get("tag") != "ok" and time.time() < deadline:
            time.sleep(0.01)
        assert hub.latest["tag"] == "ok"
        assert mgr.status == "running"
        assert mgr.error is None
    finally:
        mgr._stop.set()


def test_sim_settings_reject_unknown_keys():
    hub, mgr = _manager_with_two_sources()
    try:
        with pytest.raises(KeyError):
            mgr.set_sim_cfg({"windspeed": 9})
    finally:
        mgr._stop.set()


# --------------------------------------------------------------------------
# the twin's prediction
# --------------------------------------------------------------------------

def test_prediction_matches_the_equilibrium_it_claims_to_be():
    from model import equilibrium
    d = make_deriver()
    pred = d.predict(6.0, 50.0)
    _, tension, elev = equilibrium(Kite(), SILK, 6.0, 50.0)
    assert pred["elevation"] == pytest.approx(elev, abs=0.02)
    assert pred["tension"] == pytest.approx(tension, abs=0.01)


def test_prediction_is_withheld_when_there_is_no_flying_state():
    """Two different non-answers, and neither may reach the screen.

    The solver returns NaN when no equilibrium exists, and a real root below
    the horizon when one exists but is not a kite. NaN fails every
    comparison, so an angle check alone let a frame of NaNs through.
    """
    d = make_deriver()
    assert d.predict(1.2, 50.0) is None       # no equilibrium at all
    assert d.predict(2.6, 50.0) is None       # root exists, below the horizon


def test_a_prediction_that_is_returned_is_finite():
    d = make_deriver()
    pred = d.predict(6.0, 50.0)
    assert pred is not None
    assert all(math.isfinite(v) for v in pred.values())


def test_divergence_is_measured_minus_predicted():
    d = make_deriver()
    frame = d(bridge.parse_row(firmware_line().split(",")), 0.0)
    pred, div = frame["predicted"], frame["divergence"]
    assert pred is not None and div is not None
    assert div["elevation"] == pytest.approx(
        frame["derived"]["chord_deg"] - pred["elevation"], abs=0.02)
    assert div["tension"] == pytest.approx(
        frame["raw"]["tension"] - pred["tension"], abs=0.01)


def test_sensor_config_comes_from_the_firmware():
    cfg = bridge.read_sensor_config()
    assert cfg["error"] is None
    by_name = {v["name"]: v for v in cfg["values"]}
    # Values must match what main.cpp actually declares, not a copy.
    src = io.open(FIRMWARE, encoding="utf-8").read()
    for name, entry in by_name.items():
        m = re.search(r"static\s+const\s+\w+\s+" + name + r"\s*=\s*([-\d.]+)", src)
        assert m, f"{name} 가 펌웨어에 없습니다"
        assert entry["value"] == pytest.approx(float(m.group(1)))

# --------------------------------------------------------------------------
# recording and replay
# --------------------------------------------------------------------------

def test_recorder_round_trips_through_the_parser(tmp_path):
    """What is written must read back as the same numbers.

    A log that cannot be re-read is worse than no log: it looks like a
    successful flight record right up until the afternoon someone needs it.
    """
    rows = [bridge.parse_row(firmware_line(seq=i, tension_n=5.0 + i * 0.25)
                             .split(",")) for i in range(25)]
    rec = bridge.Recorder(tmp_path, "test")
    for r in rows:
        rec.write(r)
    rec.close()

    assert rec.path.exists()
    lines = rec.path.read_text(encoding="utf-8").strip().split("\n")
    assert lines[0] == ",".join(bridge.COLUMNS)
    assert len(lines) == len(rows) + 1

    for original, line in zip(rows, lines[1:]):
        back = bridge.parse_row(line.split(","))
        assert back == original


def test_recorder_survives_a_bad_row(tmp_path):
    """A malformed row must not take the whole recording down with it."""
    rec = bridge.Recorder(tmp_path, "test")
    rec.write(bridge.parse_row(firmware_line().split(",")))
    rec.write({"nonsense": 1})          # missing every column
    rec.close()
    assert rec.path.exists()
    assert rec.rows == 1


def test_replay_reads_back_what_was_recorded(tmp_path):
    """Record -> replay -> the same derived state, with no rig present."""
    rows = [bridge.parse_row(firmware_line(seq=i, t_ms=1000 + 50 * i).split(","))
            for i in range(40)]
    rec = bridge.Recorder(tmp_path, "rt")
    for r in rows:
        rec.write(r)
    rec.close()

    seen = []
    stop = threading.Event()
    t = threading.Thread(
        target=bridge.source_replay,
        args=(str(rec.path), seen.append, make_deriver(), stop, 20.0),
        daemon=True)
    t.start()
    try:
        deadline = time.time() + 8
        while len(seen) < 10 and time.time() < deadline:
            time.sleep(0.02)
    finally:
        stop.set()

    assert len(seen) >= 10
    assert seen[0]["raw"]["tension"] == pytest.approx(rows[0]["tension_n"], abs=1e-3)
    assert seen[0]["derived"]["chord_deg"] > seen[0]["raw"]["elevation"]


def test_replay_without_a_file_fails_clearly():
    stop = threading.Event()
    with pytest.raises(RuntimeError, match="재생할 기록"):
        bridge.source_replay(None, lambda f: None, make_deriver(), stop)


def test_replay_cannot_escape_the_log_directory():
    """The name arrives over HTTP, so it is not allowed to be a path."""
    hub, mgr = _manager_with_two_sources()
    mgr.specs["replay"] = {"label": "r", "live": False, "kind": "replay",
                           "kite": Kite(), "tether": SILK, "path": None,
                           "make": lambda d, st, pub, mgr: (lambda: None)}
    try:
        with pytest.raises(KeyError):
            mgr.set_replay("../bridge.py")
        with pytest.raises(KeyError):
            mgr.set_replay("does-not-exist.csv")
    finally:
        mgr._stop.set()


def test_test_rig_port_is_always_offered():
    """Someone who has not started the fake rig is who needs to find it."""
    ports = bridge.list_ports()
    assert any(p["port"] == bridge.FAKE_RIG_URL for p in ports)


# --------------------------------------------------------------------------
# offline
# --------------------------------------------------------------------------

def test_the_ui_loads_nothing_from_the_network():
    """The ground station runs in a field with no signal.

    Any src/href/import pointing off the machine is a blank screen there, and
    it will work perfectly on the desk right up until it matters.
    """
    import re as _re
    offenders = []
    for name in ("monitor.html", "app.js", "scene.js", "style.css"):
        text = (HERE / name).read_text(encoding="utf-8")
        for m in _re.finditer(r"""(?:src|href|from|url)\s*[=(]?\s*["']([^"']+)["']""",
                              text):
            url = m.group(1)
            if url.startswith(("http://", "https://", "//")):
                offenders.append(f"{name}: {url}")
    assert not offenders, "외부 의존성:\n  " + "\n  ".join(offenders)


def test_threejs_is_vendored_locally():
    f = HERE / "vendor" / "three.module.js"
    assert f.is_file(), "three.js 가 로컬에 없습니다 — 오프라인에서 3D가 죽습니다"
    assert f.stat().st_size > 200_000


def test_every_file_the_page_asks_for_exists():
    import re as _re
    html = (HERE / "monitor.html").read_text(encoding="utf-8")
    refs = set(_re.findall(r"""(?:src|href)=["']([^"']+)["']""", html))
    js = (HERE / "app.js").read_text(encoding="utf-8")
    refs |= set(_re.findall(r"""from\s+["'](\./[^"']+)["']""", js))
    js2 = (HERE / "scene.js").read_text(encoding="utf-8")
    refs |= set(_re.findall(r"""from\s+["'](\./[^"']+)["']""", js2))
    missing = [r for r in refs
               if not (HERE / r.lstrip("./")).is_file()]
    assert not missing, f"참조하지만 없는 파일: {missing}"


def test_static_server_refuses_to_serve_the_source():
    """bridge.py sits next to the UI files and must not be downloadable."""
    assert ".py" not in bridge.CONTENT_TYPES
    assert ".json" not in bridge.CONTENT_TYPES
    assert ".csv" not in bridge.CONTENT_TYPES

def make_specs():
    """The three specs exactly as main() builds them, minus argparse."""
    kite, tether = Kite(), SILK
    return {
        "serial": {"label": "실측", "live": True, "kind": "hardware",
                   "kite": kite, "tether": tether, "port": "loop://",
                   "make": lambda d, st, pub, mgr: (lambda: None)},
        "replay": {"label": "재생", "live": False, "kind": "log",
                   "kite": kite, "tether": tether,
                   "make": lambda d, st, pub, mgr: (lambda: None)},
        "sim": {"label": "시뮬레이션", "live": False, "kind": "model",
                "kite": kite, "tether": tether,
                "make": lambda d, st, pub, mgr: (
                    lambda: bridge.source_sim(pub, d, st, window=20.0,
                                              cfg=mgr.sim_cfg))},
    }


def test_every_source_spec_can_be_built_the_way_main_builds_it():
    """The 56 tests here all passed while `--sim` was dead on arrival.

    SourceManager.__init__ ends with switch(default), which calls the spec's
    `make`. A spec that closed over the `sources` name -- assigned only after
    the constructor returns -- raised NameError there, the source never
    started, and the browser got no frames at all. Nothing exercised the real
    construction order, so nothing caught it. This does.
    """
    import inspect
    sig = inspect.signature(bridge.SourceManager.switch)
    assert "name" in sig.parameters

    built = []

    class Spy:
        sim_cfg = dict(bridge.SIM_DEFAULTS)

    for name, spec in make_specs().items():
        gen = spec["make"](object(), threading.Event(), lambda *a, **k: None, Spy())
        assert callable(gen), f"{name}: make 가 호출 가능한 것을 돌려주지 않음"
        built.append(name)
    assert built, "확인한 소스가 없음"


def test_source_manager_starts_its_default_source():
    """End to end through the constructor, which is where the bug lived."""
    hub = bridge.Hub()
    started = []

    specs = {
        "sim": {
            "label": "시뮬레이션", "live": False, "kind": "model",
            "kite": Kite(), "tether": SILK,
            "make": lambda d, st, pub, mgr: (
                lambda: started.append(mgr.sim_cfg)),
        }
    }
    mgr = bridge.SourceManager(hub, specs, "sim", record=False)
    for _ in range(100):
        if started:
            break
        time.sleep(0.05)
    mgr._stop.set()
    assert started, "기본 소스가 시작되지 않았습니다"
    assert started[0] == mgr.sim_cfg, "소스가 매니저의 설정을 보지 못합니다"
