# -*- coding: utf-8 -*-
"""소개 사이트의 코드 열람 데이터(docs/data/code.json)를 만든다.

저장소의 소스 파일을 한 JSON 으로 묶어 두면 소개 페이지가 서버 없이 코드를 보여 줄 수 있다.
소스를 고친 뒤에는 이 스크립트를 다시 돌려 JSON 을 갱신한다.

    python tools/build_site.py
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "data" / "code.json"

EXT_LANG = {
    ".py": "python", ".js": "javascript", ".html": "xml", ".css": "css",
    ".cpp": "cpp", ".h": "cpp", ".c": "c", ".ini": "ini", ".md": "markdown",
    ".txt": "plaintext", ".spec": "python",
}

# 한 줄 설명. 위 목록에 없는 파일은 설명 없이 표시한다.
DESC = {
    # sim — 핵심 모듈
    "sim/params.py": "연·줄·탑재물·윈치의 물리 상수. 실측 전 추정값에는 MEASURE 표시",
    "sim/model.py": "2차원 점질량 연과 신축 계류줄의 운동방정식(Radau 적분), 처짐각, 평형 계산",
    "sim/wind.py": "바람장: 풍속 연직분포(멱법칙), 난류(Ornstein–Uhlenbeck), 돌풍을 합성하는 호출 객체",
    "sim/twin.py": "디지털 트윈의 핵심: 비행 기록에서 양력·항력계수를 역산해 모형을 교정",
    "sim/predict.py": "예측: 현재 상태에서 장력·고도를 앞질러 계산하고, 장력과 고도각으로 풍속을 역산",
    "sim/auto.py": "비행 기록을 누적해 계수를 갱신하고, 더 날려도 개선되지 않는 시점을 판정",
    "sim/terms.py": "모형에 남긴 항과 버린 항의 크기를 비교해 단순화의 타당성을 점검",
    "sim/run_all.py": "분석 스크립트 전체를 하위 프로세스로 실행하고 결과를 점검",
    # sim — 분석
    "sim/elasticity.py": "줄 강성을 변수로 하여 정상 장력과 돌풍 피크 장력을 비교(핵심 가설 검증)",
    "sim/compare_tether.py": "줄의 종류(명주실, 다이니마, 강체)에 따른 거동 비교",
    "sim/extremes.py": "구조물이 견뎌야 할 설계 하중: 극값 통계와 블록 길이 영향",
    "sim/uncertainty.py": "입력 가정의 불확실성을 전파하고, 어느 입력이 오차를 지배하는지 판별",
    "sim/convergence.py": "적분기 허용오차 수렴성과 계산 비용",
    "sim/scaling.py": "차원해석과 상사 시험(크기가 달라도 같은 무차원 결과가 나오는지)",
    "sim/control.py": "윈치 제어기가 실제로 할 수 있는 일의 범위",
    "sim/control_envelope.py": "고도 제어가 성립하는 풍속·조건의 영역",
    "sim/loads.py": "실제 비행에서 발생하는 난류 장력의 분포와 스펙트럼",
    "sim/lowaltitude.py": "지표 경계층에서의 저고도 비행 가능성과 소형화의 영향",
    "sim/payload.py": "매달린 탑재물의 흔들림이 장력 신호를 오염시키는 정도",
    "sim/attitude.py": "탑재물에 필요한 자세 센서의 종류 비교",
    "sim/spool.py": "스풀 반지름 변화가 줄 길이 환산에 주는 오차",
    "sim/power.py": "발전량 측정의 설계: 입사각 보정과 출력 변동",
    "sim/indoor.py": "실내 송풍 시험의 설계",
    # sim — 시험
    "sim/test_model.py": "모형 검증: 코드와 독립적으로 알려진 값(평형, 에너지 보존 등)과 대조",
    "sim/test_wind.py": "바람장 시험",
    "sim/test_twin.py": "식별 절차 시험: 참값을 아는 합성 기록으로 검증",
    "sim/test_scaling.py": "차원해석 시험",
    "sim/test_extremes.py": "극값 통계 시험",
    "sim/test_control.py": "윈치 제어기와 그 전제의 시험",
    "sim/test_auto.py": "자동 교정 루프 시험",
    # station
    "station/bridge.py": "지상국 중계: 시리얼·재생·시뮬레이터 입력을 받아 파생값을 계산하고 브라우저로 송출(SSE)",
    "station/monitor.html": "지상국 모니터 화면의 구조",
    "station/app.js": "텔레메트리 수신과 계기 표시",
    "station/scene.js": "3차원 디지털 트윈 뷰(three.js)",
    "station/style.css": "계기판 스타일",
    "station/station_app.py": "더블클릭으로 실행하는 포장: 포트 선택, 빈 포트 탐색, 브라우저 열기",
    "station/export.py": "기록된 비행을 오프라인 단일 HTML 로 내보내기",
    "station/fake_rig.py": "하드웨어 없이 시험하기 위한 가짜 지상국(TCP로 실제와 같은 바이트 송신)",
    "station/test_bridge.py": "중계 시험: 펌웨어 출력 형식과 열 수의 일치까지 검사",
    "station/README.md": "지상국 모니터 사용 설명과 설계상 주의점",
    # firmware
    "firmware/common/telemetry.h": "두 노드가 공유하는 패킷 정의(ESP-NOW)",
    "firmware/common/ebimu_parse.h": "EBIMU ASCII 프레임 파서(아두이노 헤더에 의존하지 않음)",
    "firmware/common/iv_sweep.h": "커패시터 충전 IV 스윕을 전송할 수치로 축약",
    "firmware/payload/src/main.cpp": "공중 노드: IMU·GPS·발전 측정을 ESP-NOW 로 송신",
    "firmware/payload/src/ina226.h": "INA226 전력 센서 읽기",
    "firmware/ground/src/main.cpp": "지상 노드: 수신한 텔레메트리와 지상 측정값을 CSV 로 USB 시리얼에 출력",
    "firmware/test/test_ebimu_parse.c": "EBIMU 파서의 호스트 측 시험",
    "firmware/test/test_iv_sweep.c": "IV 스윕 축약의 호스트 측 시험",
    "firmware/test/imu_cam/src/main.cpp": "ESP32-CAM 과 EBIMU 의 SD 카드 기록 시험",
    # 기타
    "twin-panel.html": "디지털 트윈 패널(단일 HTML)",
    "README.md": "저장소 안내",
    "requirements.txt": "Python 의존 패키지",
}

INCLUDE_DIRS = ["sim", "station", "firmware"]
INCLUDE_TOP = ["twin-panel.html", "README.md", "requirements.txt"]
SKIP_PARTS = {"__pycache__", ".pytest_cache", ".pio", "logs", "export", "vendor"}


def collect():
    files = []
    for d in INCLUDE_DIRS:
        for p in sorted((ROOT / d).rglob("*")):
            if not p.is_file():
                continue
            rel = p.relative_to(ROOT)
            if SKIP_PARTS & set(rel.parts):
                continue
            if rel.parts[:2] == ("sim", "out") and p.suffix != ".txt":
                continue
            if p.suffix not in EXT_LANG:
                continue
            files.append(rel)
    for t in INCLUDE_TOP:
        if (ROOT / t).exists():
            files.append(Path(t))
    return files


def main():
    data = {}
    for rel in collect():
        key = rel.as_posix()
        text = (ROOT / rel).read_text(encoding="utf-8", errors="replace").replace("\r\n", "\n")
        data[key] = {
            "lang": EXT_LANG[rel.suffix],
            "lines": text.count("\n") + 1,
            "desc": DESC.get(key, "계산 결과 출력" if key.startswith("sim/out/") else ""),
            "text": text,
        }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    total = sum(v["lines"] for v in data.values())
    print("파일 %d개, %d줄, %.0f KB -> %s" % (len(data), total, OUT.stat().st_size / 1024, OUT))
    missing = [k for k, v in data.items() if not v["desc"]]
    if missing:
        print("설명 없음:", ", ".join(missing))


if __name__ == "__main__":
    sys.exit(main())
