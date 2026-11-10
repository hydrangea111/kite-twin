# kite-twin — 방패연 계류줄 탄성 시뮬레이션과 지상국 디지털 트윈

> **가설: 계류줄에 탄성체를 활용하면 더 적은 힘으로도 연을 날릴 수 있다.**
> 방패연을 지상 윈치와 계류줄로 운용하는 에너지 자립형 공중 태양광 방식을 위해,
> 연–줄–탑재물 계를 시뮬레이션하여 이 가설을 판정한 연구의 코드입니다.

소개 페이지: https://hydrangea111.github.io/kite-twin/

## 핵심 결과 (모두 시뮬레이션 결과이며 실측 전 값입니다)

| 항목 | 결과 |
|---|---|
| 줄 강성 k = 5~5000 N/m | 정상 장력 5.59 N으로 일정 |
| 1.5초 돌풍(6→10 m/s) 피크 장력 | 38.6 N → 15.0 N (61 % 감소) |
| 돌풍 10초 | 감소 효과 약 12 % |
| 난류 비행 150회 최대 장력 | 감소 효과 5 % |
| 100회 비행에 한 번 초과하는 하중 | 54.8 N (95 % 구간 51.3~58.2) |
| 단위시험 | 152건 통과 (sim 94 + station 58) |

## 구성

| 폴더 | 내용 |
|---|---|
| `sim/` | 2차원 점질량 연 + 스프링·감쇠기 줄 모형(`model.py`), 바람(`wind.py`), 분석 스크립트와 테스트. `run_all.py`로 전체 실행 |
| `station/` | 지상국 모니터: 시리얼/재생/시뮬레이션 입력을 브라우저에서 보고 모형 예측과 비교 (`bridge.py`, `monitor.html`) |
| `firmware/` | 지상국·탑재물 ESP32 펌웨어(PlatformIO)와 파서 테스트 |
| `docs/` | 소개 페이지(GitHub Pages) |
| `twin-panel.html` | 디지털 트윈 패널 |

## 실행

```bash
pip install -r requirements.txt

# 시뮬레이션 전체 / 일부
python sim/run_all.py --quick
python sim/run_all.py elasticity

# 테스트
cd sim && python -m pytest -q
cd ../station && python -m pytest -q

# 지상국 모니터 (시뮬레이션 입력)
python station/bridge.py --sim --area 0.67      # http://127.0.0.1:8787/
```

지상국 실행 파일(KiteStation.exe)은 [kite-station 릴리스](https://github.com/hydrangea111/kite-station/releases/latest)에서 받을 수 있습니다.

## 한계

양력·항력계수와 면적·질량, 돌풍의 크기는 가정값이며 실측으로 교체해야 합니다. 모형은 2차원이므로
좌우 운동과 자세 운동은 다루지 않습니다.
