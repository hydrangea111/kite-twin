"""방패연 지상국 모니터 — 더블클릭으로 실행하는 포장.

bridge.py 는 플래그를 받아 도는 개발용 도구다. 받아서 바로 쓰는 쪽은 플래그를
모르므로, 이 파일이 그 사이를 메운다.

  * 포트를 직접 고른다. 8787 이 막혀 있으면 다음 빈 포트로 옮긴다 — 다른
    프로그램이 쓰고 있다고 멈추면 받은 사람은 고칠 방법이 없다.
  * 하드웨어 없이 뜬다 (`--sim`). 연도 기기도 없는 사람이 열어도 화면이
    채워지고, 포트가 꽂히면 화면에서 골라 실측으로 넘어갈 수 있다.
  * 브라우저를 열어 준다. 주소를 직접 입력하라고 하면 거기서 끊긴다.

플래그를 주면 그대로 bridge 에 넘긴다. 즉 개발 중에 쓰던 명령은 그대로 통한다.
"""
import socket
import sys
import threading
import time
import webbrowser
from pathlib import Path


def _utf8_console():
    """한글 Windows 콘솔은 cp949 라서 '—' 하나에 print 가 터진다.

    터지는 자리가 소스 스레드 안이면 그 스레드가 통째로 죽고, 화면에는 영원히
    "데이터 수집 중" 만 남는다 — 받은 사람 입장에서는 그냥 고장 난 프로그램이다.
    콘솔 출력 코드페이지를 UTF-8 로 올리고, 그래도 못 쓰는 글자는 버린다.
    """
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:                                       # noqa: BLE001
            pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:                                       # noqa: BLE001
            pass                                                # 콘솔이 없을 때


_utf8_console()

if getattr(sys, "frozen", False):
    sys.path.insert(0, str(Path(sys._MEIPASS) / "station"))
else:
    sys.path.insert(0, str(Path(__file__).parent))

import bridge


def free_port(start=8787, tries=40):
    """비어 있는 첫 포트. 전부 막혀 있으면 OS 가 고르게 둔다."""
    for port in range(start, start + tries):
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", port))
                return port
            except OSError:
                continue
    return 0


def open_when_up(port, timeout=20.0):
    """서버가 받기 시작한 뒤에 브라우저를 연다.

    바로 열면 아직 듣고 있지 않아 '연결할 수 없음' 이 뜨고, 받은 사람은 그게
    준비 중이라는 뜻인지 고장인지 알 수 없다.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket() as s:
            s.settimeout(0.3)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                webbrowser.open(f"http://127.0.0.1:{port}/")
                return
        time.sleep(0.2)


def main():
    args = sys.argv[1:]
    # 플래그를 하나도 안 줬을 때만 기본값을 채운다.
    if not args:
        port = free_port()
        args = ["--sim", "--default", "sim", "--port", str(port)]
        print("=" * 60)
        print("  방패연 지상국 모니터")
        print("=" * 60)
        print()
        print(f"  브라우저가 열립니다 →  http://127.0.0.1:{port}/")
        print()
        print("  지금은 모델이 만든 데이터로 돌고 있습니다.")
        print("  실제 기기를 꽂았으면 화면 위 [포트] 에서 고르세요.")
        print()
        print(f"  기록과 설정:  {bridge.DATA}")
        print()
        print("  이 창을 닫으면 종료됩니다.")
        print("=" * 60)
        print()
        threading.Thread(target=open_when_up, args=(port,), daemon=True).start()

    sys.argv = [sys.argv[0]] + args
    bridge.main()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        # 번들에서는 트레이스백만 떠 있어도 창이 바로 닫혀 아무것도 못 읽는다.
        import traceback
        traceback.print_exc()
        print()
        input("오류가 났습니다. 위 내용을 알려주세요. Enter 로 닫기...")
