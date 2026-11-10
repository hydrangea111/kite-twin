"""A stand-in ground station, for testing the chain without hardware.

Speaks exactly what `firmware/ground/src/main.cpp` prints -- boot banner, CSV
header, then one row per packet -- over a TCP socket that pyserial treats as a
serial port. The bridge cannot tell it apart from a real ESP32, so this
exercises the serial path, not a bypass of it.

    python station/fake_rig.py                       # listens on 127.0.0.1:5577
    python station/bridge.py --serial socket://127.0.0.1:5577

The numbers are the two-body model's, not invented ones: a rig that emitted
plausible-looking noise would let the bridge pass a test that proves nothing
about the physics it reports.

Options worth having when checking the UI's failure behaviour:

    --drop 0.05     lose 5% of lines, as a weak radio link would
    --garbage 0.02  emit a corrupted line now and then
    --stall 20      go silent after 20 s, to see the "신호 끊김" state
"""

import argparse
import random
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))

import bridge                                              # noqa: E402


def serve(conn, args):
    rng = random.Random(args.seed)
    conn.sendall(b"# ground station up\r\n")
    conn.sendall((",".join(bridge.COLUMNS) + "\r\n").encode())

    period = 1.0 / args.rate
    next_t = time.time()
    started = time.time()
    sent = skipped = 0

    for row in bridge.simulated_rows(rate=args.rate, window=args.window):
        if args.stall and time.time() - started > args.stall:
            print(f"[rig] {args.stall:.0f} s 경과 — 송신 중단 (연결은 유지)")
            while True:
                time.sleep(1.0)

        line = bridge.format_row(row)

        if args.garbage and rng.random() < args.garbage:
            # A real link truncates mid-line far more often than it flips a
            # byte, so that is what this injects.
            line = line[:rng.randint(5, len(line) - 5)]

        if args.drop and rng.random() < args.drop:
            skipped += 1
        else:
            try:
                conn.sendall((line + "\r\n").encode())
            except OSError:
                print(f"[rig] 연결 끊김 — {sent}행 전송")
                return
            sent += 1

        if sent and sent % (int(args.rate) * 10) == 0:
            print(f"[rig] {sent}행 전송, {skipped}행 유실")

        next_t += period
        delay = next_t - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.time()


def main():
    ap = argparse.ArgumentParser(description="가짜 지상국 (하드웨어 없이 전 구간 시험)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--rate", type=float, default=20.0, help="행/초")
    ap.add_argument("--window", type=float, default=120.0, help="모델 구간 [s]")
    ap.add_argument("--drop", type=float, default=0.0, help="행 유실 확률")
    ap.add_argument("--garbage", type=float, default=0.0, help="깨진 행 확률")
    ap.add_argument("--stall", type=float, default=0.0, help="N초 뒤 송신 중단")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    # Integrate before anyone connects. simulated_rows caches per settings, so
    # this makes the first connection instant instead of making the browser
    # wait 30 s for Radau -- and a reconnect costs nothing at all.
    print("[rig] 모델 준비 중 ...")
    next(bridge.simulated_rows(rate=args.rate, window=args.window))
    print("[rig] 준비 완료")

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    url = f"socket://{args.host}:{args.port}"
    print(f"[rig] 대기 중 — python station/bridge.py --serial {url}")

    try:
        while True:
            conn, addr = srv.accept()
            print(f"[rig] 접속: {addr[0]}:{addr[1]}")
            with conn:
                serve(conn, args)
    except KeyboardInterrupt:
        print("\n[rig] 종료")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
