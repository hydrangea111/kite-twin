// ESP32-CAM + EBIMU — SD 카드에 남기는 수신 시험.
//
// 블루투스를 걷어냈다. 이 작업 내내 링크가 한 번 끊기면 다시 붙지 않아
// 번번이 막혔고, 그걸 되살리려고 하드웨어를 몇 번이나 다시 꽂게 만들었다.
// 결과를 카드에 남기면 무선이 필요 없다. 전파를 안 켜니 전류도 안 뛴다.
//
// 대신 펌웨어 교체는 MB 보드로만 된다. OTA 가 블루투스를 타고 있었다.
// 그 교환을 받아들인 것이다. 카드에 다 남으므로 펌웨어를 고칠 일 자체가
// 줄어든다.
//
// ── 배선 (선 세 개) ────────────────────────────────────────────────
//
//   EBIMU VCC  ←── 3.3V 출력핀
//   EBIMU GND  ──→ GND          (서플라이 - 와 같은 GND 여야 한다)
//   EBIMU TX   ──→ GPIO13
//
// EBIMU 의 RX 는 꽂지 않는다. 받기만 하면 되고, 더 중요한 이유가 있다.
// SD 카드를 1비트로 붙이면 GPIO2·14·15 를 쓴다. GPIO15 에 EBIMU 의 RX 가
// 붙어 있으면 카드 신호가 EBIMU 로 들어간다. EBIMU 는 ASCII 명령을 받는
// 장치라서 쓰레기 명령을 먹을 수 있다. 그 선은 빼야 한다.
//
// GPIO13 은 1비트 모드에서 카드가 쓰지 않는다. 그래서 카드와 수신이 같이
// 돌아간다. 4비트로 붙이면 GPIO13 을 카드가 가져가 버린다.
//
// ── 카드에 남는 것 ─────────────────────────────────────────────────
//
//   /kite_imu.log 에 이어서 적는다. 전원을 넣을 때마다 구분선이 들어간다.
//   리셋 원인, 부팅 결정핀 상태, 선 진단, 보율 탐색, 초당 수신량이 남는다.
//   한 줄마다 파일을 닫으므로 전원이 갑자기 끊겨도 적힌 데까지는 남는다.
//
// ── 명령 (USB 로 한 글자. MB 에 꽂았을 때만) ───────────────────────
//   r 수신   v 핀 식별   p 선 진단   t 자체검증   b 보율
//   l 로그 보기   e 로그 지우기   s 카드 상태   U 펌웨어 교체   ? 도움말

#include <Arduino.h>
#include <Preferences.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Update.h>
#include <driver/uart.h>
#include <string.h>

static const int PIN_IMU_RX = 13;    // EBIMU TX -> 여기로 들어온다

// EBIMU V5 의 공장 기본값은 115200 이다. 틀렸을 때를 대비해 흔한 값을
// 전부 넣고, 자주 맞는 것부터 시도한다.
static const uint32_t BAUDS[] = {115200, 921600, 460800, 230400,
                                 57600, 38400, 19200, 9600};
static const size_t N_BAUD = sizeof(BAUDS) / sizeof(BAUDS[0]);
static size_t baud_idx = 0;

static HardwareSerial ImuSerial(1);
static Preferences prefs;

#include "ebimu_parse.h"

// ── SD 카드 ──────────────────────────────────────────────────────────
static bool sd_ok = false;
static const char *LOG_PATH = "/kite_imu.log";

static void sdLog(const char *s) {
  if (!sd_ok) return;
  // 한 줄마다 열고 닫는다. 느리지만 초당 한 줄이라 문제가 없고, 전원이
  // 갑자기 끊겨도 적힌 데까지는 남는다. 연을 띄우다 전원이 끊기는 것은
  // 드문 일이 아니다.
  File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  f.print(s);
  f.close();
}

static void say(const char *s) {
  Serial.print(s);
  sdLog(s);
}

static bool sdStart() {
  if (sd_ok) return true;
  sd_ok = SD_MMC.begin("/sdcard", true);   // true = 1비트. GPIO13 을 비워 둔다
  return sd_ok;
}

static void sdStop() {
  if (!sd_ok) return;
  SD_MMC.end();
  sd_ok = false;
}

// ── 핀 식별 ──────────────────────────────────────────────────────────
//
// 이 보드는 실크스크린이 없어서 어느 핀이 GPIO13 인지 눈으로 알 수 없다.
// 그래서 핀마다 다른 전압을 걸어 둔다. 멀티미터는 1 kHz 구형파를 평균해서
// 읽으므로 듀티가 그대로 전압으로 보인다.
//
// 전에는 GPIO13·15 만 걸고 나머지는 0 V 로 두었다. 그런데 GPIO0 도 0 V 로
// 읽히고 GND 도 0 V 라서, GND 를 찾다가 GPIO0 을 고르면 부팅 때 다운로드
// 모드로 들어가 앱이 아예 돌지 않는다. 그 혼선을 없애려고 헤더로 나온
// GPIO 를 전부 다른 전압으로 건다.
//
//   GPIO14   0.41 V      GPIO15   0.83 V      GPIO2    1.24 V
//   GPIO12   1.65 V      GPIO0    2.06 V      GPIO13   2.48 V
//
// 건드리지 않는 핀은 셋뿐이다. GPIO1·3 은 USB 직렬이라 명령 경로가 막히고,
// GPIO16 은 PSRAM 칩선택이라 건드리면 죽을 수 있다. 이 셋과 GND 를 가리는
// 것은 도통 확인으로 한다. GND 끼리는 서로 삑 울린다.
//
// GPIO0 을 거는 것은 부팅이 끝난 뒤라 무해하다. 전원을 끊으면 보드의 풀업이
// 되살아나 다음 부팅도 멀쩡하다.
struct MarkPin { int pin; uint32_t duty; const char *label; };
static const MarkPin MARKS[] = {
  {14,  128, "0.41 V"},
  {15,  256, "0.83 V"},
  { 2,  384, "1.24 V"},
  {12,  512, "1.65 V"},
  { 0,  640, "2.06 V"},
  {13,  767, "2.48 V"},
};
static const size_t N_MARK = sizeof(MARKS) / sizeof(MARKS[0]);
static const uint32_t LEDC_HZ = 1000;
static const uint8_t LEDC_BITS = 10;

enum Mode { MODE_MARK, MODE_RUN };
static Mode mode = MODE_RUN;

static void pilotOn();   // 전원 표시등. 아래 LED 절에 있다

static void enterMark() {
  ImuSerial.end();
  sdStop();              // 카드가 GPIO2·14·15 를 쥐고 있으면 걸 수 없다

  pilotOn();             // 표시등을 되살린다. 끄고 들어가면 죽은 줄 안다

  for (size_t i = 0; i < N_MARK; i++) {
    ledcSetup((uint8_t)i, LEDC_HZ, LEDC_BITS);
    ledcAttachPin(MARKS[i].pin, (uint8_t)i);
    ledcWrite((uint8_t)i, MARKS[i].duty);
  }
  mode = MODE_MARK;

  Serial.print("\n[핀 식별] 검은 탐침을 GND 에 대고 16핀을 훑어라.\n");
  for (size_t i = 0; i < N_MARK; i++) {
    char b[80];
    snprintf(b, sizeof(b), "  %-8s -> GPIO%d%s\n", MARKS[i].label,
             MARKS[i].pin, MARKS[i].pin == PIN_IMU_RX ? "   <- EBIMU TX" : "");
    Serial.print(b);
  }
  Serial.print("  3.3 V -> 전원 출력 (EBIMU VCC),  5 V -> 전원 입력\n");
  Serial.print("  0 V 이고 서로 도통 -> GND.  GPIO1·3·16 은 건드리지 않았다.\n");
  Serial.print("r 을 보내면 수신 시험으로 돌아간다. 카드는 그때 다시 연다.\n");
}

static void leaveMark() {
  if (mode != MODE_MARK) return;
  for (size_t i = 0; i < N_MARK; i++) {
    ledcDetachPin(MARKS[i].pin);
    pinMode(MARKS[i].pin, INPUT);
  }
}

// ── 선 상태 진단 ─────────────────────────────────────────────────────
//
// 바이트가 하나도 안 들어올 때, 선이 안 붙은 것인지 붙었는데 상대가 조용한
// 것인지 가려야 한다. 보율을 아무리 훑어도 이것은 알 수 없다.
//
// 내부 풀업·풀다운은 수십 kΩ 이고 UART 출력은 그보다 훨씬 세다. 그래서
// 상대가 선을 쥐고 있으면 풀 저항이 져서 값이 안 따라오고, 아무것도 안
// 붙어 있으면 그대로 따라온다.
// 카드를 열기 전에 한 번 재서 적어 둔다. GPIO13 은 SD 의 D3 선이고,
// 드라이버가 카드를 SD 모드로 잡아 두려고 그 핀을 HIGH 로 당긴다. 카드를
// 연 뒤에 재면 무엇이 쥐고 있는지 가릴 수 없다. 실제로 그렇게 재서
// "선은 붙었고 상대가 조용하다"는 잘못된 판정을 냈다.
static char probe_early[400];

static void probe13() {
  pinMode(PIN_IMU_RX, INPUT_PULLDOWN);
  delay(20);
  int with_pd = digitalRead(PIN_IMU_RX);

  pinMode(PIN_IMU_RX, INPUT_PULLUP);
  delay(20);
  int with_pu = digitalRead(PIN_IMU_RX);

  pinMode(PIN_IMU_RX, INPUT);
  delay(5);
  int last = digitalRead(PIN_IMU_RX);
  uint32_t edges = 0, highs = 0, n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    int v = digitalRead(PIN_IMU_RX);
    if (v != last) { edges++; last = v; }
    if (v) highs++;
    n++;
  }

  char b[240];
  snprintf(b, sizeof(b),
           "[선 진단] 풀다운 %s · 풀업 %s · 300ms 전환 %lu회 · HIGH %lu%%\n",
           with_pd ? "HIGH" : "LOW", with_pu ? "HIGH" : "LOW",
           (unsigned long)edges, (unsigned long)(n ? highs * 100 / n : 0));
  say(b);
  strncpy(probe_early, b, sizeof(probe_early) - 1);

  // 판정은 전환 횟수로만 한다. 풀 시험은 카드가 꽂혀 있으면 쓸 수 없다.
  // GPIO13 은 SD 의 D3 선이고, 카드 안에 그 선을 끌어올리는 저항이 있어서
  // 내부 풀다운이 져 버린다. 그래서 카드가 있으면 무엇이 붙어 있든 HIGH 로
  // 보인다. 실제로 그 값을 "선은 붙었고 상대가 조용하다"고 읽어 틀렸다.
  if (edges > 20) {
    say("  -> 선에 신호가 흐른다. 데이터는 오고 있다.\n");
  } else if (with_pu == LOW) {
    say("  -> 누가 LOW 로 쥐고 있다. GND 와 붙었거나 상대가 출력 LOW 다.\n");
  } else {
    say("  -> 전환이 없다. 상대가 조용하거나 선이 안 붙었다.\n");
    say("     카드가 꽂혀 있으면 이 핀은 카드의 D3 저항 때문에 늘 HIGH 로\n");
    say("     보인다. 붙었는지 여부는 이 측정으로 가릴 수 없다.\n");
  }
}

// ── 수신 경로 자체 검증 ──────────────────────────────────────────────
//
// EBIMU 가 없어도 수신 경로가 맞는지 확인할 수 있어야 한다. 아니면 0 B/s 일
// 때 선이 문제인지 이 코드가 문제인지 영원히 가릴 수 없다.
//
// 전에는 UART2 의 송신을 GPIO13 패드로 내보내고 같은 패드를 UART1 로 읽는
// 식으로 짰는데, 한 패드에 두 주변장치를 묶는 설정이 틀려서 아무것도 못
// 읽었다. 그 결과를 "핀 번호가 틀렸다"고 읽었던 것이 잘못이었다.
//
// 그래서 UART 내부 루프백을 쓴다. 패드를 거치지 않으므로 핀 설정과 분리해서
// UART·파서·집계만 검증한다. 통과하면 0 B/s 의 원인은 바깥이다.
static void selfTest() {
  const char *frame = "*12.34,-56.78,90.12\r\n";

  say("\n[자체검증] UART 내부 루프백. 패드를 거치지 않는다.\n");

  ImuSerial.end();
  ImuSerial.begin(115200, SERIAL_8N1, PIN_IMU_RX, -1);
  if (uart_set_loop_back(UART_NUM_1, true) != ESP_OK) {
    say("  루프백을 켤 수 없다. 검증 불가.\n");
    return;
  }

  uint32_t sent = 0, got = 0, frames = 0, bad = 0;
  size_t l = 0;
  char buf[160];

  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    if (sent < 20) { ImuSerial.print(frame); sent++; }
    while (ImuSerial.available()) {
      char c = (char)ImuSerial.read();
      got++;
      if (c == '*') { l = 0; continue; }
      if (c == '\r') continue;
      if (c == '\n') {
        buf[l] = '\0';
        if (l > 0) {
          EbimuFrame f;
          if (ebimu_parse(buf, &f)) frames++; else bad++;
        }
        l = 0;
        continue;
      }
      if (l < sizeof(buf) - 1) buf[l++] = c;
    }
  }

  uart_set_loop_back(UART_NUM_1, false);

  char b[220];
  snprintf(b, sizeof(b),
           "  보낸 %lu개 · 받은 바이트 %lu · 파싱 %lu · 깨짐 %lu\n",
           (unsigned long)sent, (unsigned long)got,
           (unsigned long)frames, (unsigned long)bad);
  say(b);
  if (frames >= sent / 2 && bad == 0) {
    say("  -> 통과. UART 와 파서는 맞다. 0 B/s 면 바깥에서 안 오는 것이다.\n");
  } else if (got == 0) {
    say("  -> 실패. 내부에서도 못 읽는다. UART 설정이 틀렸다.\n");
  } else {
    say("  -> 부분 실패. 바이트는 오는데 프레임이 안 맞는다. 파서를 보라.\n");
  }
}

// ── 상태 LED ─────────────────────────────────────────────────────────
//
// 리포로 돌릴 때는 USB 도 블루투스도 없다. 카드를 빼서 읽기 전까지는 보드가
// 무엇을 하고 있는지 알 방법이 없었다. 그래서 깜빡임으로 말하게 한다.
//
//   2초마다 짧게 1번   돌고 있음. GPIO13 에 아무것도 안 들어옴
//   2초마다 짧게 2번   바이트는 들어오는데 프레임이 안 맞음 (보율 의심)
//   2초마다 짧게 3번   프레임 정상. 받고 있다
//   2초마다 짧게 4번   SD 카드가 없다
//
// 빨간 LED 는 액티브 LOW 다. 흰 플래시 LED(GPIO4)는 전류를 크게 먹어서
// 상태 표시로 쓰지 않는다. 그쪽은 L 명령으로 잠깐만 켜 본다.
static const int PIN_LED_RED = 33;
static const int PIN_LED_FLASH = 4;

// 전원이 들어오면 켜고 그대로 둔다. 깜빡임으로 상태를 말하게 했었는데,
// 깜빡임을 세어서 해석해야 하는 것 자체가 번거롭다. 켜져 있으면 살아 있는
// 것이고 꺼지면 죽은 것이다. 그 둘만 보면 된다.
//
// 빨간 LED 는 액티브 LOW 라 LOW 가 켜짐이다. 전류는 몇 mA 수준이라
// 리포로 돌려도 부담이 없다.
// MB 보드의 빨간 LED 는 MB 자신의 전원 표시등이고 모듈 것이 아니다. MB 를
// 빼면 아무것도 안 보인다. 모듈에 있는 것은 흰 플래시 LED(GPIO4)뿐이므로
// 그것을 전원 표시로 쓴다.
//
// 다만 그 LED 는 전조등용이라 꽉 켜면 100 mA 를 넘게 먹는다. 리포로 돌릴 때
// 그만큼을 표시등에 쓸 수는 없다. 그래서 듀티를 4 % 로 낮춘다. 평균 전류는
// 5 mA 쯤이고 눈에는 켜진 것으로 보인다.
//
// 핀 식별 모드가 LEDC 채널 0~5 를 쓰므로 표시등은 6번 채널을 쓴다.
static const int LEDC_CH_PILOT = 6;
static const uint32_t PILOT_DUTY = 40;     // 1023 의 약 4 %

static void pilotOn() {
  ledcSetup(LEDC_CH_PILOT, LEDC_HZ, LEDC_BITS);
  ledcAttachPin(PIN_LED_FLASH, LEDC_CH_PILOT);
  ledcWrite(LEDC_CH_PILOT, PILOT_DUTY);
}

static void ledInit() {
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);          // 보드에 달려 있으면 켜진다
  pilotOn();
}

// LED 가 실제로 전류를 얼마나 먹는지는 보드마다 다르다. 직렬 저항이 배치마다
// 달라서 데이터시트로는 알 수 없다. 그래서 듀티를 단계로 바꿔 주고, 서플라이
// 전류계로 차이를 읽게 한다. 추정값을 적어 두는 것보다 이쪽이 낫다.
//
// 빨간 LED 도 같이 끄고 켜서 그 몫을 따로 읽을 수 있게 한다.
static int pilot_step = 1;               // 0 꺼짐 · 1 4% · 2 25% · 3 100%

static void pilotStep() {
  static const uint32_t DUTY[] = {0, 40, 256, 1023};
  static const char *NAME[] = {"꺼짐", "4%", "25%", "100%"};
  pilot_step = (pilot_step + 1) % 4;

  ledcWrite(LEDC_CH_PILOT, DUTY[pilot_step]);
  // 꺼짐 단계에서는 빨간 LED 도 끈다. 둘을 분리해서 재려는 것이다.
  digitalWrite(PIN_LED_RED, pilot_step == 0 ? HIGH : LOW);

  char b[200];
  snprintf(b, sizeof(b),
           "[LED] 흰 LED 듀티 %s · 빨간 LED %s. 서플라이 전류를 읽어라.\n",
           NAME[pilot_step], pilot_step == 0 ? "꺼짐" : "켜짐");
  say(b);
}


// 켜진 상태를 유지한다. 다른 코드가 실수로 이 핀을 건드려도 되돌린다.
static void updateLed() {
  digitalWrite(PIN_LED_RED, LOW);
}

// 플래시 LED 가 이 보드에 실제로 달려 있는지 확인하는 용도. 밝고 전류를
// 많이 먹으므로 짧게만 켠다. 리포로 돌릴 때는 쓰지 않는 쪽이 낫다.
static void flashTest() {
  say("[LED] 흰 플래시 LED 를 0.2초씩 3번 켠다. 전류가 크므로 짧게만.\n");
  pinMode(PIN_LED_FLASH, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_FLASH, HIGH);
    delay(200);
    digitalWrite(PIN_LED_FLASH, LOW);
    delay(300);
  }
  pilotOn();                 // 시험이 끝나면 표시등으로 되돌린다
  say("[LED] 끝. 안 켜졌으면 이 보드엔 플래시 LED 가 없는 것이다.\n");
}

// ── 수신 집계 ────────────────────────────────────────────────────────
static char line[160];
static size_t len = 0;

static uint32_t t_window = 0;
static uint32_t n_bytes = 0, n_frames = 0, n_bad = 0;
static uint8_t last_count = 0;
static bool saw_star = false;

static bool baud_locked = false;
static uint32_t sweep_t = 0;
static uint32_t sweep_bytes = 0, sweep_frames = 0;
static const uint32_t SWEEP_QUIET_MS = 6000;
static const uint32_t SWEEP_NOISE_MS = 12000;

static void startImu() {
  // TX 를 -1 로 두어 GPIO15 를 비운다. 받기만 하면 되고, 비워 두면 카드가
  // 그 핀을 쓸 수 있다.
  ImuSerial.end();
  pinMode(PIN_IMU_RX, INPUT);
  ImuSerial.begin(BAUDS[baud_idx], SERIAL_8N1, PIN_IMU_RX, -1);
  len = 0;
  sweep_t = millis();
  sweep_bytes = sweep_frames = 0;
}

static void nextBaud(const char *why) {
  baud_idx = (baud_idx + 1) % N_BAUD;
  char b[140];
  snprintf(b, sizeof(b), "보율 %lu 로 갈아탐 (%s)\n",
           (unsigned long)BAUDS[baud_idx], why);
  say(b);
  startImu();
}

// 초당 보고를 카드에 다 적으면 몇 분 돌려도 수백 KB 가 된다. 실제로 705 KB
// 까지 찼고, 115200 으로는 다 내보낼 수 없어서 어느 회차에 무슨 일이
// 있었는지 못 찾았다.
//
// 그래서 아무 일도 없을 때는 30초에 한 줄만 카드에 적는다. USB 로는 매초
// 그대로 내보낸다 — 사람이 보고 있을 때는 촘촘한 쪽이 낫다. 바이트가 한 번
// 이라도 들어오면 그 줄은 무조건 적는다. 그것이 찾고 있는 사건이다.
static const uint32_t SD_IDLE_EVERY = 30;
static uint32_t idle_lines = 0;

static void report() {
  char buf[260];
  snprintf(buf, sizeof(buf),
           "[%6lus] %5lu B/s  프레임 %3lu/s  깨짐 %2lu  필드 %u개  보율 %lu%s",
           (unsigned long)(millis() / 1000), (unsigned long)n_bytes,
           (unsigned long)n_frames, (unsigned long)n_bad, last_count,
           (unsigned long)BAUDS[baud_idx], baud_locked ? " (확정)" : "");

  bool quiet = (n_bytes == 0 && n_frames == 0 && n_bad == 0);
  bool to_card = !quiet || (idle_lines % SD_IDLE_EVERY == 0);
  if (quiet) idle_lines++;
  else idle_lines = 0;

  Serial.print(buf);
  if (to_card) sdLog(buf);

  const char *hint = NULL;
  if (n_bytes == 0) {
    hint = "   <- 아무것도 안 들어옴. TX 가 GPIO13 인지, GND 가 공통인지 보라";
  } else if (n_frames == 0 && !saw_star) {
    hint = "   <- 바이트는 오는데 '*' 가 없다. 보율을 더 훑는다";
  } else if (n_bad > n_frames) {
    hint = "   <- 깨진 프레임이 더 많다. 접지나 전압을 의심하라";
  }
  if (hint) {
    Serial.print(hint);
    if (to_card) sdLog(hint);
  }
  Serial.print("\n");
  if (to_card) sdLog("\n");

  n_bytes = n_frames = n_bad = 0;
  saw_star = false;
}

static void enterRun() {
  leaveMark();
  pilotOn();
  sdStart();

  mode = MODE_RUN;
  baud_idx = 0;
  baud_locked = false;
  n_bytes = n_frames = n_bad = 0;
  saw_star = false;
  t_window = millis();

  startImu();

  char b[200];
  snprintf(b, sizeof(b),
           "[수신 시험] GPIO13 으로 받는다. 보율 %lu 부터 훑는다.\n",
           (unsigned long)BAUDS[baud_idx]);
  say(b);
}

// ── 진단 ─────────────────────────────────────────────────────────────
//
// 한 번 켜서 최대한 많이 알아내는 것이 목적이다. 통신이 없어도 카드만 보면
// 판단이 되어야 하고, 추측으로 메울 자리를 남기지 않아야 한다.
//
// RTC 메모리는 리셋을 넘겨도 남지만 전원이 끊기면 사라진다. 그 성질로
// "칩이 리셋한 것"과 "전원이 끊긴 것"을 가린다. NVS 회차는 전원이 끊겨도
// 남으므로, 둘을 비교하면 어느 쪽인지 나온다. 플래시 수명을 깎지 않으려고
// 초당 기록은 RTC 쪽에만 한다.
#define RTC_MAGIC 0x4B495445u          /* "KITE" */
RTC_NOINIT_ATTR static uint32_t rtc_magic;
RTC_NOINIT_ATTR static uint32_t rtc_uptime;    // 직전 회차가 살아 있던 초
RTC_NOINIT_ATTR static uint32_t rtc_resets;    // 전원 끊김 이후 리셋 횟수

static char diag_early[1200];          // 카드 열기 전에 담아 두는 측정

static void diagAppend(const char *s) {
  size_t n = strlen(diag_early);
  if (n < sizeof(diag_early) - 1) {
    strncpy(diag_early + n, s, sizeof(diag_early) - n - 1);
  }
}

// 헤더로 나온 핀을 전부 당겨 보고 무엇이 붙었는지 분류한다. 내부 풀은 수십
// kΩ 이라, 바깥에서 쥐고 있으면 값이 안 따라온다.
//
// GPIO1·3 은 USB 직렬이라 건드리면 명령 경로가 끊기므로 뺀다. 나머지는
// 입력으로 읽기만 하므로 무해하다.
static void scanPins() {
  // GPIO16 은 넣지 않는다. PSRAM 칩선택이라 입력으로 바꾸는 순간 PSRAM 이
  // 끊기고, 그 위에 올라가 있던 힙이 통째로 깨진다. 실제로 넣어서 돌렸다가
  // 다음 malloc 에서 LoadProhibited 로 죽었다. 스캔에서 빠진 핀은 GPIO1·3
  // (USB 직렬)과 GPIO16 셋이다.
  static const int PINS[] = {0, 2, 4, 12, 13, 14, 15};
  diagAppend("[핀 스캔] 풀다운/풀업으로 당겨 본 결과\n");
  for (size_t i = 0; i < sizeof(PINS) / sizeof(PINS[0]); i++) {
    int p = PINS[i];
    pinMode(p, INPUT_PULLDOWN);
    delay(4);
    int pd = digitalRead(p);
    pinMode(p, INPUT_PULLUP);
    delay(4);
    int pu = digitalRead(p);
    pinMode(p, INPUT);

    // 전환도 같이 센다. 풀 시험으로는 "붙었는지"만 알고 "신호가 흐르는지"는
    // 모른다. EBIMU 가 송신 중인데 어느 핀에 들어오는지 몰라서 사람이 선을
    // 따라가며 짚어야 했다. 이 한 줄이 그 수고를 없앤다.
    delay(2);
    int last = digitalRead(p);
    uint32_t edges = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 200) {
      int v = digitalRead(p);
      if (v != last) { edges++; last = v; }
    }

    // 전환만 보고 판정하면 안 된다. 떠 있는 핀도 전원 잡음을 받아 200ms 에
    // 수십 번 흔들린다. 실제로 GPIO12 가 42회로 나왔고 그걸 "신호가 들어온다"
    // 고 읽어 틀린 판정을 냈다. UART 는 대기 중 HIGH 로 선을 세게 쥐므로,
    // 풀다운으로 당겨도 HIGH 여야 한다. 쥐고 있으면서 전환도 많아야 신호다.
    bool driven_high = (pd == 1 && pu == 1);
    const char *verdict;
    if (driven_high && edges > 20) verdict = "신호가 흐른다 <<< 여기로 들어온다";
    else if (pd == 0 && pu == 1)   verdict = "떠 있음 (아무것도 안 붙음). 전환은 잡음";
    else if (pd == 1 && pu == 1) verdict = "HIGH 로 쥐고 있음";
    else if (pd == 0 && pu == 0) verdict = "LOW 로 쥐고 있음";
    else                         verdict = "이상 (설명 안 되는 조합)";

    char b[160];
    snprintf(b, sizeof(b),
             "  GPIO%-2d  풀다운 %s · 풀업 %s · 200ms 전환 %4lu회  -> %s\n",
             p, pd ? "H" : "L", pu ? "H" : "L", (unsigned long)edges, verdict);
    diagAppend(b);
  }
  diagAppend("  GPIO1·3 은 USB 직렬이라 건드리지 않았다.\n");
}

// GPIO13 에 들어오는 가장 짧은 펄스를 재서 보율을 역산한다. 보율 목록에
// 없는 값이면 8개를 다 훑어도 못 찾으므로, 폭에서 직접 구하는 쪽이 낫다.
// UART 한 비트의 시간이 최단 펄스이므로 보율은 그 역수다.
static void measureBaud() {
  pinMode(PIN_IMU_RX, INPUT);
  delay(2);

  uint32_t t_edge = micros();
  int last = digitalRead(PIN_IMU_RX);
  uint32_t min_run = 0xFFFFFFFFu, edges = 0, highs = 0, samples = 0;
  uint32_t t0 = micros();
  while (micros() - t0 < 2000000u) {          // 2초
    int v = digitalRead(PIN_IMU_RX);
    samples++;
    if (v) highs++;
    if (v != last) {
      uint32_t now = micros();
      uint32_t run = now - t_edge;
      if (run < min_run) min_run = run;
      t_edge = now;
      last = v;
      edges++;
    }
  }

  char b[260];
  if (edges == 0) {
    snprintf(b, sizeof(b),
             "[보율 측정] 2초간 전환 0회. HIGH 비율 %lu%%. 선에 신호가 없다.\n",
             (unsigned long)(samples ? highs * 100 / samples : 0));
    diagAppend(b);
    return;
  }
  uint32_t baud = min_run ? 1000000u / min_run : 0;
  snprintf(b, sizeof(b),
           "[보율 측정] 전환 %lu회 · 최단 펄스 %lu us -> 보율 약 %lu\n"
           "  HIGH 비율 %lu%% · 표본 %lu개 (표본 간격이 최단 펄스보다 "
           "크면 과소 추정이다)\n",
           (unsigned long)edges, (unsigned long)min_run, (unsigned long)baud,
           (unsigned long)(samples ? highs * 100 / samples : 0),
           (unsigned long)samples);
  diagAppend(b);
}

// 보드와 빌드를 특정한다. 보드를 바꿨는지 모르고 추론해서 오늘 한 번
// 틀렸다. MAC 을 적어 두면 그런 일이 없다.
static void sayIdentity() {
  uint64_t mac = ESP.getEfuseMac();
  uint8_t m[6];
  for (int i = 0; i < 6; i++) m[i] = (uint8_t)(mac >> (8 * i));

  char b[300];
  snprintf(b, sizeof(b),
           "보드: MAC %02x:%02x:%02x:%02x:%02x:%02x · %s rev%d · %d MHz\n"
           "플래시 %lu MB · 여유 힙 %lu B · 최소 여유 힙 %lu B\n"
           "빌드: " __DATE__ " " __TIME__ "\n",
           m[0], m[1], m[2], m[3], m[4], m[5],
           ESP.getChipModel(), ESP.getChipRevision(), (int)getCpuFrequencyMhz(),
           (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)),
           (unsigned long)ESP.getFreeHeap(),
           (unsigned long)ESP.getMinFreeHeap());
  say(b);
}

static void sayCardInfo() {
  if (!sd_ok) { say("카드: 없음\n"); return; }
  const char *type;
  switch (SD_MMC.cardType()) {
    case CARD_MMC:  type = "MMC"; break;
    case CARD_SD:   type = "SD"; break;
    case CARD_SDHC: type = "SDHC"; break;
    default:        type = "알 수 없음"; break;
  }
  uint64_t logsz = 0;
  File f = SD_MMC.open(LOG_PATH, FILE_READ);
  if (f) { logsz = f.size(); f.close(); }

  char b[220];
  snprintf(b, sizeof(b),
           "카드: %s · %llu MB · 사용 %llu MB · 로그 %llu B\n",
           type, SD_MMC.cardSize() / (1024ULL * 1024ULL),
           SD_MMC.usedBytes() / (1024ULL * 1024ULL),
           (unsigned long long)logsz);
  say(b);
}

// ── 리셋 원인 ────────────────────────────────────────────────────────
//
// 보드가 왜 재시작했는지를 안 찍어서, 전류계로 짐작해야 했다. 이것을 남기면
// 통신이 하나도 없어도 카드만 보고 판단할 수 있다.
// 부팅 직후에 읽어 둔 값. 핀 스캔이 풀업·풀다운을 걸었다 떼면 핀이 떠서
// 잔류 전하를 읽는다. 실제로 GPIO12 를 그렇게 읽어 "부팅 막는다"고 찍었고,
// 같은 핀을 스캔은 "떠 있음"으로 분류했다. 외부 요인이 아니라 순서 문제였다.
static int boot_g0 = -1, boot_g12 = -1;

static void readStrapPins() {
  pinMode(0, INPUT);
  pinMode(12, INPUT);
  delay(5);
  boot_g0 = digitalRead(0);
  boot_g12 = digitalRead(12);
}

static void sayBootCause() {
  const char *why;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   why = "POWERON (정상)"; break;
    case ESP_RST_BROWNOUT:  why = "BROWNOUT (전압 부족)"; break;
    case ESP_RST_TASK_WDT:  why = "TASK_WDT (와치독)"; break;
    case ESP_RST_INT_WDT:   why = "INT_WDT (와치독)"; break;
    case ESP_RST_WDT:       why = "WDT (와치독)"; break;
    case ESP_RST_PANIC:     why = "PANIC (예외)"; break;
    case ESP_RST_SW:        why = "SW (ESP.restart)"; break;
    case ESP_RST_EXT:       why = "EXT (리셋핀)"; break;
    case ESP_RST_DEEPSLEEP: why = "DEEPSLEEP"; break;
    default:                why = "기타"; break;
  }

  // 부팅 직후에 읽어 둔 값을 쓴다. 지금 읽으면 핀 스캔이 남긴 잔류 전하가
  // 섞인다. 확정적인 분류는 아래 핀 스캔의 풀 시험 쪽이다.
  int g0 = boot_g0, g12 = boot_g12;

  char b[220];
  snprintf(b, sizeof(b), "리셋 원인: %s · GPIO0=%s · GPIO12=%s\n", why,
           g0 ? "HIGH(정상)" : "LOW(다운로드 모드가 된다)",
           g12 ? "HIGH(부팅 막는다)" : "LOW(정상)");
  say(b);
}

// ── 로그 보기·지우기 ─────────────────────────────────────────────────
static void dumpLog() {
  if (!sd_ok) { Serial.print("카드 없음\n"); return; }
  File f = SD_MMC.open(LOG_PATH, FILE_READ);
  if (!f) { Serial.print("로그 없음\n"); return; }
  char b[80];
  snprintf(b, sizeof(b), "\n---- %s · %lu 바이트 ----\n", LOG_PATH,
           (unsigned long)f.size());
  Serial.print(b);
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.print("---- 끝 ----\n");
}

// 로그에서 요점만 뽑는다.
//
// 초당 보고가 전부 카드에 들어가므로 몇 분만 돌려도 수백 KB 가 된다. 그걸
// 115200 으로 다 내보내면 앞부분만 읽히고 끝난다. 실제로 그래서 어느 회차에
// 무슨 일이 있었는지 못 찾았다.
//
// 그래서 "아무 일도 없었다"는 줄만 버린다. 0 B/s 로 끝난 초당 보고가
// 그것이고, 나머지는 다 남긴다. 바이트가 들어온 보고는 당연히 남는다.
static void dumpSummary() {
  if (!sd_ok) { Serial.print("카드 없음\n"); return; }
  File f = SD_MMC.open(LOG_PATH, FILE_READ);
  if (!f) { Serial.print("로그 없음\n"); return; }

  char b[300];
  snprintf(b, sizeof(b), "\n---- 요약 · 원본 %lu 바이트 ----\n",
           (unsigned long)f.size());
  Serial.print(b);

  char line[300];
  size_t len = 0;
  uint32_t kept = 0, dropped = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\n' || len >= sizeof(line) - 1) {
      line[len] = '\0';
      // 평범한 초당 보고는 버린다. 대괄호로 시작하고 0 B/s 인 줄이다.
      bool routine = (line[0] == '[' && strstr(line, "0 B/s") != NULL &&
                      strstr(line, "프레임   0/s") != NULL);
      if (routine) {
        dropped++;
      } else if (len > 0) {
        kept++;
        Serial.print(line);
        Serial.print("\n");
      }
      len = 0;
      continue;
    }
    if (c != '\r') line[len++] = c;
  }
  f.close();
  snprintf(b, sizeof(b), "---- 끝 · 남긴 줄 %lu · 버린 줄 %lu ----\n",
           (unsigned long)kept, (unsigned long)dropped);
  Serial.print(b);
}

static void eraseLog() {
  if (!sd_ok) { Serial.print("카드 없음\n"); return; }
  Serial.print(SD_MMC.remove(LOG_PATH) ? "로그 지웠다\n" : "못 지웠다\n");
}

// ── 펌웨어 교체 (USB) ────────────────────────────────────────────────
//
// 규약: "U<바이트수>\n" -> 0x06 하나 -> 본문. 덩이마다 0x06 을 돌려준다.
// 실패하면 쓰던 슬롯을 버리고 지금 펌웨어가 그대로 다시 뜬다.
static void otaReceive(Stream &src) {
  char num[16];
  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    if (!src.available()) continue;
    char c = (char)src.read();
    if (c == '\n' || c == '\r') break;
    if (c < '0' || c > '9') continue;
    if (n < sizeof(num) - 1) num[n++] = c;
  }
  num[n] = '\0';
  uint32_t total = (uint32_t)atol(num);

  char b[200];
  if (total < 100000UL || total > 1900000UL) {
    snprintf(b, sizeof(b), "[OTA] 크기가 이상하다 (%lu). 중단한다.\n",
             (unsigned long)total);
    Serial.print(b);
    return;
  }
  snprintf(b, sizeof(b), "[OTA] %lu 바이트 받는다.\n", (unsigned long)total);
  Serial.print(b);

  if (!Update.begin(total)) {
    snprintf(b, sizeof(b), "[OTA] 시작 실패: %s\n", Update.errorString());
    Serial.print(b);
    return;
  }

  const uint8_t ACK = 0x06;
  const uint32_t ACK_EVERY = 512;
  uint32_t since_ack = 0;
  src.write(ACK);
  src.flush();

  uint8_t chunk[1024];
  uint32_t done = 0, mark = 0, last_rx = millis();
  while (done < total) {
    int avail = src.available();
    if (avail <= 0) {
      if (millis() - last_rx > 10000) {
        Update.abort();
        Serial.print("[OTA] 10초간 끊김. 지금 펌웨어를 유지한다.\n");
        return;
      }
      continue;
    }
    size_t want = (size_t)avail;
    if (want > sizeof(chunk)) want = sizeof(chunk);
    if (want > total - done) want = total - done;
    size_t got = src.readBytes((char *)chunk, want);
    if (got == 0) continue;
    if (Update.write(chunk, got) != got) {
      snprintf(b, sizeof(b), "[OTA] 쓰기 실패: %s\n", Update.errorString());
      Serial.print(b);
      Update.abort();
      return;
    }
    done += got;
    last_rx = millis();

    since_ack += got;
    if (since_ack >= ACK_EVERY || done == total) {
      since_ack = 0;
      src.write(ACK);
      src.flush();
    }
    if (done - mark >= total / 10) {
      mark = done;
      snprintf(b, sizeof(b), "[OTA] %lu%%\n",
               (unsigned long)(done * 100 / total));
      Serial.print(b);
    }
  }

  if (!Update.end(true)) {
    snprintf(b, sizeof(b), "[OTA] 마무리 실패: %s\n", Update.errorString());
    Serial.print(b);
    return;
  }
  Serial.print("[OTA] 완료. 재부팅한다.\n");
  delay(300);
  ESP.restart();
}

// ── 명령 ─────────────────────────────────────────────────────────────
static int readCmdSerial() {
  if (Serial.available()) return Serial.read();
  return -1;
}

static void help() {
  Serial.print("\n명령: r 수신  v 핀식별  p 선진단  t 자체검증  b 보율"
               "  l 로그  T 요약  e 로그지우기  s 카드  L 플래시LED  d LED듀티  U 펌웨어"
               "  ? 도움말\n");
}

static void handle(int ch) {
  switch (ch) {
    case 'r': enterRun(); break;
    case 'v': enterMark(); break;
    case 'p': if (mode == MODE_RUN) probe13(); break;
    case 't': selfTest(); if (mode == MODE_RUN) startImu(); break;
    case 'b':
      if (mode == MODE_RUN) { baud_locked = false; nextBaud("수동"); }
      break;
    case 'L': flashTest(); break;
    case 'd': pilotStep(); break;
    case 'l': dumpLog(); break;
    case 'T': dumpSummary(); break;
    case 'e': eraseLog(); break;
    case 's': {
      char b[60];
      snprintf(b, sizeof(b), "SD 카드 %s\n", sd_ok ? "있음" : "없음");
      Serial.print(b);
      break;
    }
    case 'U': otaReceive(Serial); break;
    case '?': help(); break;
    default: break;                   // 개행이나 잡음은 무시한다
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // 회차 기록을 가장 먼저 한다. 늦게 적으면 안 된다.
  //
  // 처음에는 진단이 끝난 뒤에 적었는데, 보율 측정만 2초를 쓰기 때문에
  // 기록 시점이 2.5초쯤으로 밀렸다. 그러면 2초에 전원을 끊은 경우 아무것도
  // 안 적힌 상태가 되어, 다음 부팅이 "직전 회차는 정상"으로 읽는다. 짧게
  // 껐다 켜는 창이 2.5~5초 사이로 좁아져 쓸 수 없었다.
  uint32_t n = 0;
  bool short_prev = false;
  if (prefs.begin("kite", false)) {
    n = prefs.getUInt("boot", 0);
    prefs.putUInt("boot", n + 1);
    short_prev = (prefs.getUChar("ran", 1) == 0);
    prefs.putUChar("ran", 0);          // 5초를 넘기면 loop 에서 1 로 적는다
    prefs.end();
  }

  // 카드를 열기 전에 다 잰다. 카드가 GPIO13(D3)을 HIGH 로 당기므로 순서를
  // 바꾸면 무엇이 쥐고 있는지 가릴 수 없다. 핀 스캔과 보율 측정도 같은
  // 이유로 여기서 한다. 결과는 담아 두고 카드가 열린 뒤에 적는다.
  readStrapPins();          // 다른 어떤 핀도 건드리기 전에
  ledInit();

  diag_early[0] = '\0';
  scanPins();
  measureBaud();
  probe13();

  sdStart();

  // RTC 메모리는 리셋을 넘겨도 남고 전원이 끊기면 사라진다. 그 차이로
  // 전원 끊김과 칩 리셋을 가린다.
  uint32_t prev_up = 0;
  bool had_rtc = (rtc_magic == RTC_MAGIC);
  if (had_rtc) {
    prev_up = rtc_uptime;
    rtc_resets++;
  } else {
    rtc_magic = RTC_MAGIC;
    rtc_resets = 1;
  }
  rtc_uptime = 0;


  char b[220];
  snprintf(b, sizeof(b),
           "\n======== %lu번째 전원 ========\n"
           "ESP32-CAM · EBIMU 수신 시험 · 블루투스 없음\n",
           (unsigned long)(n + 1));
  say(b);
  say(sd_ok ? "SD 카드 있음. /kite_imu.log 에 이어 적는다.\n"
            : "SD 카드 없음. USB 로만 내보낸다.\n");
  sayBootCause();
  sayIdentity();
  sayCardInfo();

  // 직전 회차가 얼마나 살았는지. 이것이 리셋 반복을 가리는 핵심이다.
  char u[260];
  if (had_rtc) {
    snprintf(u, sizeof(u),
             "직전 회차: %lu초 살았다 · 전원 끊김 이후 리셋 %lu회\n"
             "  -> RTC 가 남아 있으므로 전원이 끊긴 것이 아니라 칩이 "
             "다시 시작했다.\n",
             (unsigned long)prev_up, (unsigned long)rtc_resets);
  } else {
    snprintf(u, sizeof(u),
             "직전 회차: 기록 없음 (RTC 비어 있음)\n"
             "  -> 전원이 완전히 끊겼다가 들어온 것이다.\n");
  }
  say(u);

  say("LED: 흰 플래시 LED 를 4% 듀티로 약하게 켜 둔다. 꺼지면 죽은 것이다.\n");

  if (diag_early[0]) say(diag_early);
  if (probe_early[0]) say(probe_early);

  // 부팅마다 수신 경로를 검증한다. 0 B/s 가 나왔을 때 선 문제인지 코드
  // 문제인지 매번 기록으로 갈리게 하려는 것이다.
  selfTest();

  help();

  if (short_prev) {
    say("직전 회차가 5초를 못 넘겼다. 핀 식별 모드로 뜬다.\n");
    say("  약 2.48 V 가 GPIO13 이다. EBIMU 의 TX 를 거기 꽂아라.\n");
    say("  수신 시험으로 가려면 5초 넘게 켜 두고 전원을 다시 넣어라.\n");
    enterMark();
  } else {
    enterRun();
  }
}

void loop() {
  int ch = readCmdSerial();
  if (ch >= 0) handle(ch);

  updateLed();

  // 5초를 넘기면 "정상 가동"을 플래시에 적는다. 회차당 한 번만 쓴다.
  //
  // 이 일을 초당 보고 블록 안에 넣었다가 함정을 만들었다. 그 블록은 수신
  // 모드에서만 도는데, 식별 모드에 한 번 들어가면 표시를 못 적으니 다음
  // 부팅도 또 식별 모드가 되어 영원히 못 나온다. 그래서 모드와 무관하게
  // 여기서 적는다.
  static bool marked_ran = false;
  if (!marked_ran && millis() > 5000) {
    marked_ran = true;
    if (prefs.begin("kite", false)) {
      prefs.putUChar("ran", 1);
      prefs.end();
    }
  }

  if (mode == MODE_MARK) {
    // 걸어 둔 전압이 그대로 있다. 할 일이 없다.
    delay(10);
    return;
  }

  while (ImuSerial.available()) {
    char c = (char)ImuSerial.read();
    n_bytes++;
    sweep_bytes++;

    if (c == '*') { saw_star = true; len = 0; continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      if (len > 0) {
        EbimuFrame f;
        if (ebimu_parse(line, &f)) {
          n_frames++;
          sweep_frames++;
          last_count = f.count;
          // 초당 한 줄만 원본을 보여 준다. 전부 찍으면 읽을 수가 없고
          // 카드도 금방 찬다.
          static uint32_t last_echo = 0;
          if (millis() - last_echo > 1000) {
            last_echo = millis();
            char b[220];
            snprintf(b, sizeof(b), "       원본: *%s\n", line);
            say(b);
          }
        } else {
          n_bad++;
        }
      }
      len = 0;
      continue;
    }
    if (len < sizeof(line) - 1) line[len++] = c;
    else { len = 0; n_bad++; }
  }

  // 보율 탐색. 프레임이 한 번이라도 잡히면 그 보율로 확정한다.
  if (!baud_locked) {
    if (sweep_frames > 0) {
      baud_locked = true;
      char b[120];
      snprintf(b, sizeof(b), "보율 %lu 로 확정. 프레임이 잡힌다.\n",
               (unsigned long)BAUDS[baud_idx]);
      say(b);
    } else if (sweep_bytes == 0 && millis() - sweep_t > SWEEP_QUIET_MS) {
      nextBaud("아무것도 안 들어옴");
    } else if (sweep_bytes > 0 && millis() - sweep_t > SWEEP_NOISE_MS) {
      nextBaud("바이트는 오는데 프레임이 없음");
    }
  }

  if (millis() - t_window >= 1000) {
    t_window += 1000;
    rtc_uptime = millis() / 1000;   // 리셋을 넘겨 남는다. 플래시는 안 쓴다
    report();
  }
}
