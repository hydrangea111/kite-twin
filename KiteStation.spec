# -*- mode: python ; coding: utf-8 -*-
"""지상국 모니터를 단일 exe 로 묶는다.  빌드:  pyinstaller KiteStation.spec

station/ 의 UI 파일과 sim/ 의 모델을 함께 싣는다. bridge.py 는 frozen 일 때
sys._MEIPASS 아래에서 둘을 찾고, 쓰기가 필요한 것(기록·설정)만 AppData 로
보낸다 — 번들 폴더는 종료할 때 지워지기 때문이다.

matplotlib 은 빼 두었다. sim 의 연구 스크립트만 쓰고 브리지는 쓰지 않는데,
넣으면 45 MB 쯤 커진다.
"""

a = Analysis(
    ['station/station_app.py'],
    pathex=['station', 'sim'],
    binaries=[],
    datas=[
        ('station/monitor.html', 'station'),
        ('station/app.js', 'station'),
        ('station/scene.js', 'station'),
        ('station/style.css', 'station'),
        ('station/vendor/three.module.js', 'station/vendor'),
    ],
    # bridge 는 serial·numpy·scipy·payload 를 함수 안에서 늦게 import 한다.
    # 정적 분석으로는 안 보이므로 여기 적어 둔다.
    hiddenimports=['bridge', 'model', 'params', 'wind', 'payload', 'serial'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib', 'tkinter', 'PyQt5', 'PyQt6', 'PySide2', 'PySide6',
        'IPython', 'pytest', 'pandas', 'PIL', 'notebook', 'sympy',
    ],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='KiteStation',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    # 콘솔을 남긴다. 주소와 기록 위치를 적어 주고, 창을 닫는 것이 곧 종료다.
    # 서버가 도는 프로그램이라 "떠 있다"는 표시가 있는 편이 낫다.
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    # 16 px 에서도 읽히게 그린 다중 크기 아이콘 (station/kite.ico 참고).
    icon='station/kite.ico',
)
