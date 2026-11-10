/* 지상국 모니터 — 텔레메트리 수신과 계기 표시.
 *
 * 브리지가 보내는 프레임은 소스(시리얼/재생/시뮬)와 무관하게 같은 스키마다.
 * 이 파일은 그 스키마만 알면 되고, 어디서 왔는지는 알 필요가 없다.
 */

import { TwinScene, KITE_SCALE } from "./scene.js";

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) =>
  (v === null || v === undefined || Number.isNaN(v)) ? "—" : Number(v).toFixed(d);

/* ───────────────────────── 화면 설정 기억 ─────────────────────────
 * 서버는 소스와 시뮬 설정을 기억한다. 분할 여부나 시점처럼 보는 사람 쪽
 * 취향은 브라우저에 둔다 — 잠깐 다녀왔다고 매번 다시 맞추게 할 이유가 없다.
 * 사생활도 상태도 아닌 값이라 실패해도 조용히 넘어간다. */
const PREFS = "kite-station-prefs";

function loadPrefs() {
  try { return JSON.parse(localStorage.getItem(PREFS) || "{}"); }
  catch { return {}; }
}

function savePrefs(patch) {
  try {
    localStorage.setItem(PREFS, JSON.stringify({ ...loadPrefs(), ...patch }));
  } catch { /* 사생활 모드 등에서는 저장이 막힌다 */ }
}

/* 전환 직후에는 이전 소스의 숫자가 패널에 남는다. 연결 표시가 '끊김'이라
 * 해도, 고도와 장력이 멀쩡히 적혀 있으면 그게 지금 값으로 읽힌다. */
function blankInstruments() {
  const dash = ["sAlt", "sElev", "sTen", "sWind", "sRate", "hAlt", "hElev",
                "hSpan", "hTen", "hSwing", "hWind", "vRoll", "vPitch", "vYaw",
                "vTen", "vTenMax", "cStretch", "cSag", "cChord", "cAlt",
                "iCl", "iCd", "iLd", "iWind", "lRx", "lImu", "lGps",
                "pAlt", "pElev", "pTen", "dElev", "dTen", "dZ"];
  for (const id of dash) {
    const n = $(id);
    if (n) { n.textContent = "—"; n.className = n.className.replace(/delta.*/, ""); }
  }
  $("barTen").style.width = "0%";
  $("divbar").textContent = "";
  $("pvBody").innerHTML = '<div class="empty">IV 스윕 대기 중</div>';
  $("pvNote").textContent = "";
  $("gateText").textContent = "—";
  $("gate").className = "gate";
}

/* 단독 실행 묶음.
 *
 * export.py 가 만든 한 파일짜리 HTML 안에서는 서버가 없다. 브리지도,
 * /stream 도, /api 도 없이 file:// 로 열린다. 그 경우 프레임은 이미 파생까지
 * 끝난 채로 파일 안에 들어 있고, 여기서는 시간 순서대로 재생만 한다.
 *
 * 서버가 필요한 조작(포트, 기록, 소스 전환, 시뮬 설정)은 숨긴다 — 눌러도
 * 아무 일도 안 일어나는 버튼을 남겨두면 고장으로 읽힌다. */
const BUNDLE = window.__KITE_BUNDLE__ || null;

const HIST_SECONDS = 60;
const hist = [];
const rateWin = [];
let last = null, lastAt = 0, meta = null;

/* ───────────────────────── 3D ───────────────────────── */

let twin = null, twinPred = null;
try {
  twin = new TwinScene($("viewport"));
} catch (err) {
  // WebGL이 없는 환경에서도 계기는 살아 있어야 한다
  $("viewport").innerHTML =
    '<div class="empty" style="padding-top:80px">3D를 표시할 수 없습니다 (WebGL 미지원)<br>' +
    '<span style="font-size:11px">계기와 시계열은 정상 동작합니다</span></div>';
  console.warn("3D disabled:", err);
}

document.querySelectorAll(".viewctl button[data-view]").forEach(b => {
  b.addEventListener("click", () => {
    document.querySelectorAll(".viewctl button[data-view]").forEach(
      x => x.classList.remove("on"));
    b.classList.add("on");
    twin?.preset(b.dataset.view);
    twinPred?.preset(b.dataset.view);
    savePrefs({ view: b.dataset.view });
  });
});

/* 분할: 왼쪽은 센서가 말하는 상태, 오른쪽은 같은 조건에서 모델이 말하는 상태.
 * 장면을 둘로 두는 이유는 둘이 실제로 다른 상태이기 때문이다 — 한 장면에
 * 겹쳐 그리면 어느 쪽이 측정이고 어느 쪽이 모델인지 구분되지 않는다. */
let splitOn = false;

/* 좌우를 묶을지 따로 볼지.
 *
 * 묶음  — 한쪽을 돌리면 다른 쪽도 같이 돈다. 같은 각도에서 봐야 실측과 예측의
 *         차이가 자세 차이인지 시점 차이인지 헷갈리지 않는다. 기본값.
 * 따로  — 한쪽은 측면, 한쪽은 평면처럼 서로 다른 각도로 둘 수 있다.
 *
 * 구현은 매 프레임 '마지막으로 만진 쪽'의 카메라를 반대쪽에 복사하는 쪽을
 * 골랐다. 양쪽에 변경 콜백을 걸어 서로 밀면 재귀를 막을 장치가 필요하고,
 * 그 장치가 한 번 어긋나면 카메라가 떨린다.
 */
let linked = loadPrefs().linked !== false;
let activePane = "real";

function reflectLink() {
  $("link").classList.toggle("on", linked);
  $("link").textContent = linked ? "연동" : "개별";
  $("link").title = linked ? "좌우 시점을 함께 움직입니다" : "좌우를 따로 조정합니다";
  if (twinPred) twinPred.autoFrame = !linked;
}

$("link").addEventListener("click", () => {
  linked = !linked;
  savePrefs({ linked });
  reflectLink();
  // 묶는 순간 바로 맞춰준다. 다음 상호작용까지 기다리면 눌렀는데 아무 일도
  // 일어나지 않은 것처럼 보인다.
  if (linked && twin && twinPred) {
    twinPred.applyCameraState(twin.cameraState());
    activePane = "real";
  }
});

$("split").addEventListener("click", () => {
  splitOn = !splitOn;
  savePrefs({ split: splitOn });
  $("split").classList.toggle("on", splitOn);
  $("panes").classList.toggle("split", splitOn);
  $("viewportPred").hidden = !splitOn;
  $("link").hidden = !splitOn;
  if (splitOn && !twinPred) {
    try {
      twinPred = new TwinScene($("viewportPred"));
      twinPred.autoFrame = !linked;
      twinPred.preset("wide");
      twinPred.onInteract(() => { activePane = "pred"; });
      twin?.onInteract(() => { activePane = "real"; });
      if (linked && twin) twinPred.applyCameraState(twin.cameraState());
    } catch (err) { console.warn("예측 뷰를 열 수 없습니다:", err); }
  }
  reflectLink();
});

/* 예측값을 실측 프레임과 같은 모양으로 감싼다. 장면 쪽은 이것이 측정인지
 * 모델인지 알 필요가 없고, 알게 하면 코드가 두 벌로 갈라진다. */
function predictedFrame(f) {
  const p = f.predicted;
  if (!p) return null;
  const el = (p.elevation * Math.PI) / 180;
  return {
    t: f.t, seq: f.seq,
    raw: { ...f.raw, tension: p.tension, elevation: p.elevation },
    derived: {
      span: p.span, stretch_cm: 0, sag_deg: 0, chord_deg: p.elevation,
      x: p.x, z: p.z,
      // 보정 전 위치를 따로 두지 않는다 — 예측에는 센서가 없고 보정할 것도 없다
      x_raw: p.x, z_raw: p.z, alt_err_cm: 0,
      cl: null, cd: null, ld: null, wind_est: null,
      steady: false, n_steady: 0,
      payload_x: p.x + 0.5 * Math.sin(el),
      payload_z: p.z - 0.5 * Math.cos(el),
    },
    pv: { valid: false },
    link: f.link,
    limits: f.limits,
  };
}
$("spin").addEventListener("click", () => {
  const on = twin?.toggleSpin();
  $("spin").classList.toggle("on", !!on);
  // 묶여 있으면 복사가 돌기 때문에 따로 켤 필요가 없고, 따로 보는 중이면
  // 양쪽 다 돌아야 자연스럽다.
  if (!linked && twinPred) twinPred.toggleSpin();
  activePane = "real";
});

/* ───────────────────────── 자세계 ───────────────────────── */

const SVGNS = "http://www.w3.org/2000/svg";

function drawAttitude(f) {
  const svg = $("att");
  svg.textContent = "";
  const add = (tag, attrs, text, parent) => {
    const n = document.createElementNS(SVGNS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    if (text !== undefined) n.textContent = text;
    (parent || svg).appendChild(n);
    return n;
  };

  const cx = 84, cy = 84, R = 66;
  const roll = f ? (f.raw.roll || 0) : 0;
  const pitch = f ? (f.raw.pitch || 0) : 0;

  const defs = add("defs");
  const clip = add("clipPath", { id: "ball" }, undefined, defs);
  add("circle", { cx, cy, r: R }, undefined, clip);

  const g = add("g", { "clip-path": "url(#ball)" });
  // pitch 는 진자 흔들림 각이므로, 수평선이 얼마나 기울었는지가 곧 흔들림이다
  const inner = add("g", {
    transform: `rotate(${-roll} ${cx} ${cy}) translate(0 ${pitch * 1.9})`,
  }, undefined, g);
  add("rect", { x: cx - 190, y: cy - 260, width: 380, height: 260, fill: "#1b3444" },
      undefined, inner);
  add("rect", { x: cx - 190, y: cy, width: 380, height: 260, fill: "#2a2519" },
      undefined, inner);
  add("line", { x1: cx - 190, y1: cy, x2: cx + 190, y2: cy,
                stroke: "#e3ecf2", "stroke-width": 1.5 }, undefined, inner);
  for (const p of [-40, -30, -20, -10, 10, 20, 30, 40]) {
    const y = cy + p * 1.9, w = (p % 20 === 0) ? 23 : 13;
    add("line", { x1: cx - w, y1: y, x2: cx + w, y2: y,
                  stroke: "#9fb2bf", "stroke-width": 1, opacity: 0.8 }, undefined, inner);
    if (p % 20 === 0)
      add("text", { x: cx - w - 5, y: y + 3, fill: "#9fb2bf", "font-size": 8,
                    "text-anchor": "end" }, String(Math.abs(p)), inner);
  }

  add("circle", { cx, cy, r: R, fill: "none", stroke: "#22323d", "stroke-width": 2 });

  // 고정 기준 마크
  add("path", { d: `M ${cx - 27} ${cy} l 17 0 l 5 6 l 5 -6 l 17 0`,
                fill: "none", stroke: "#e8b341", "stroke-width": 2.2,
                "stroke-linejoin": "round" });
  add("circle", { cx, cy, r: 2.2, fill: "#e8b341" });

  // roll 눈금과 지침
  for (const a of [-60, -45, -30, -15, 0, 15, 30, 45, 60]) {
    const r = (a * Math.PI) / 180;
    const r0 = R + 1, r1 = R + (a % 45 === 0 ? 7 : 4);
    add("line", {
      x1: cx + r0 * Math.sin(r), y1: cy - r0 * Math.cos(r),
      x2: cx + r1 * Math.sin(r), y2: cy - r1 * Math.cos(r),
      stroke: a === 0 ? "#e8b341" : "#44586a", "stroke-width": a % 45 === 0 ? 1.6 : 1,
    });
  }
  const rr = (roll * Math.PI) / 180;
  add("path", {
    d: `M ${cx + (R - 3) * Math.sin(rr)} ${cy - (R - 3) * Math.cos(rr)} ` +
       `l ${-4.5 * Math.cos(rr) - 0} ${-4.5 * Math.sin(rr)} ` +
       `l ${9 * Math.cos(rr)} ${9 * Math.sin(rr)} z`,
    fill: "#e8b341",
  });

  // yaw 나침반
  const bx = 232, by = 84, br = 50;
  const yaw = f ? (((f.raw.yaw || 0) % 360) + 360) % 360 : 0;
  add("circle", { cx: bx, cy: by, r: br, fill: "#0f171d", stroke: "#22323d" });
  const ring = add("g", { transform: `translate(${bx},${by})` });
  for (let a = 0; a < 360; a += 15) {
    const rel = (((a - yaw + 540) % 360) - 180) * Math.PI / 180;
    const major = a % 90 === 0;
    const r0 = br - 2, r1 = br - (major ? 10 : 5);
    add("line", {
      x1: r0 * Math.sin(rel), y1: -r0 * Math.cos(rel),
      x2: r1 * Math.sin(rel), y2: -r1 * Math.cos(rel),
      stroke: major ? "#9fb2bf" : "#3a4c5a", "stroke-width": major ? 1.5 : 1,
    }, undefined, ring);
    if (major)
      add("text", {
        x: (br - 19) * Math.sin(rel), y: -(br - 19) * Math.cos(rel) + 3.5,
        fill: "#9fb2bf", "font-size": 9, "text-anchor": "middle",
      }, ["N", "E", "S", "W"][a / 90], ring);
  }
  add("path", { d: `M ${bx} ${by - br + 1} l -4 -7 l 8 0 z`, fill: "#e8b341" });
  add("text", { x: bx, y: by + 5, fill: "#e3ecf2", "font-size": 16,
                "text-anchor": "middle", "font-family": "Consolas,monospace" },
      f ? `${yaw.toFixed(0)}°` : "—");
  add("text", { x: bx, y: by + 21, fill: "#6b7f8d", "font-size": 8.5,
                "text-anchor": "middle" }, "YAW");
}

/* ───────────────────────── 시계열 ───────────────────────── */

const SERIES = [
  { key: "tension", label: "장력", unit: "N", color: "#52b9e8", floor: 0 },
  { key: "elev", label: "고도각", unit: "°", color: "#4fc99a", floor: 0 },
  { key: "wind", label: "풍속", unit: "m/s", color: "#e8b341", floor: 0 },
];

function drawStrip() {
  const cv = $("strip");
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const W = cv.clientWidth, H = 168;
  if (!W) return;
  if (cv.width !== Math.round(W * dpr)) {
    cv.width = Math.round(W * dpr);
    cv.height = Math.round(H * dpr);
  }
  const g = cv.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);

  if (hist.length < 2) {
    g.fillStyle = "#6b7f8d";
    g.font = '12px "Malgun Gothic",sans-serif';
    g.textAlign = "center";
    g.fillText("데이터 수집 중", W / 2, H / 2);
    return;
  }

  const padL = 52, padR = 58, rowH = H / SERIES.length;
  const t1 = hist[hist.length - 1].t, t0 = t1 - HIST_SECONDS;

  SERIES.forEach((s, i) => {
    const top = i * rowH + 13, h = rowH - 24;
    const vals = hist.map(p => p[s.key]).filter(v => v != null && !Number.isNaN(v));
    if (!vals.length) return;
    let lo = Math.min(...vals), hi = Math.max(...vals);
    if (hi - lo < 1e-6) hi = lo + 1;
    const pad = (hi - lo) * 0.16;
    hi += pad;
    // 바닥을 패딩 뒤에 자른다: 풍속 축이 음수로 내려가는 건 눈금 선택이
    // 아니라 존재할 수 없는 값이다
    lo = s.floor != null ? Math.max(s.floor, Math.min(lo - pad, Math.min(...vals))) : lo - pad;

    g.strokeStyle = "#1a2832";
    g.lineWidth = 1;
    for (const frac of [0, 0.5, 1]) {
      const y = top + h * frac;
      g.beginPath(); g.moveTo(padL, y); g.lineTo(W - padR, y); g.stroke();
    }

    g.fillStyle = "#6b7f8d";
    g.font = '10px Consolas,monospace';
    g.textAlign = "right";
    g.fillText(hi.toFixed(1), padL - 6, top + 4);
    g.fillText(lo.toFixed(1), padL - 6, top + h + 3);

    g.textAlign = "left";
    g.fillStyle = "#9fb2bf";
    g.font = '11px "Malgun Gothic",sans-serif';
    g.fillText(`${s.label} [${s.unit}]`, padL + 4, top - 3);

    const X = t => padL + ((t - t0) / HIST_SECONDS) * (W - padL - padR);
    const Y = v => top + h - ((v - lo) / (hi - lo)) * h;

    // 면적 채움으로 세 줄을 서로 구분되게 한다
    g.beginPath();
    let started = false;
    for (const p of hist) {
      const v = p[s.key];
      if (v == null || Number.isNaN(v)) continue;
      const x = X(p.t), y = Y(v);
      if (!started) { g.moveTo(x, y); started = true; } else g.lineTo(x, y);
    }
    const grad = g.createLinearGradient(0, top, 0, top + h);
    grad.addColorStop(0, s.color + "33");
    grad.addColorStop(1, s.color + "05");
    g.save();
    g.lineTo(X(hist[hist.length - 1].t), top + h);
    g.lineTo(X(hist[0].t), top + h);
    g.closePath();
    g.fillStyle = grad;
    g.fill();
    g.restore();

    g.beginPath();
    started = false;
    for (const p of hist) {
      const v = p[s.key];
      if (v == null || Number.isNaN(v)) continue;
      const x = X(p.t), y = Y(v);
      if (!started) { g.moveTo(x, y); started = true; } else g.lineTo(x, y);
    }
    g.strokeStyle = s.color;
    g.lineWidth = 1.5;
    g.stroke();

    const lp = hist[hist.length - 1];
    if (lp[s.key] != null) {
      const x = X(lp.t), y = Y(lp[s.key]);
      g.fillStyle = s.color;
      g.beginPath(); g.arc(x, y, 2.8, 0, Math.PI * 2); g.fill();
      g.font = '12px Consolas,monospace';
      g.textAlign = "left";
      g.fillText(lp[s.key].toFixed(2), W - padR + 7, y + 4);
    }
  });
}

/* ───────────────────────── 계기 ───────────────────────── */

function paint(f) {
  const d = f.derived, r = f.raw;
  const flying = d.chord_deg > 2 && d.z > 0;

  $("sAlt").textContent = flying ? `${fmt(d.z, 1)} m` : "—";
  $("sElev").textContent = flying ? `${fmt(d.chord_deg, 1)}°` : "—";
  $("sTen").textContent = `${fmt(r.tension, 2)} N`;
  $("sWind").textContent = `${fmt(r.wind, 2)} m/s`;

  $("hAlt").textContent = !flying ? "비행 중 아님"
    : d.slack ? "줄 늘어짐" : `${fmt(d.z, 1)} m`;
  $("hElev").textContent = `${fmt(d.chord_deg, 1)}°`;
  $("hSpan").textContent = `${fmt(d.span, 2)} m`;
  $("hTen").textContent = `${fmt(r.tension, 2)} N`;
  $("hSwing").textContent = `${fmt(r.pitch, 1)}°`;
  $("hWind").textContent = `${fmt(r.wind, 2)} m/s`;

  $("vRoll").innerHTML = `${fmt(r.roll, 1)}<u>°</u>`;
  $("vPitch").innerHTML = `${fmt(r.pitch, 1)}<u>°</u>`;
  $("vYaw").innerHTML = `${fmt(r.yaw, 1)}<u>°</u>`;
  $("attNote").textContent = Math.abs(r.pitch) > 45 ? "크게 흔들림" : "";

  // 하중은 설계하중 대비로 본다. 파단하중 대비로 보면 11배 여유라 언제나
  // 안전해 보이는데, 먼저 부러지는 건 줄이 아니라 대나무 이음부와 매듭이다.
  const design = f.limits?.design_load || 50;
  const frac = r.tension / design;
  const bar = $("barTen");
  bar.style.width = `${Math.min(100, frac * 100).toFixed(1)}%`;
  bar.className = `fill ${frac > 1 ? "bad" : frac > 0.7 ? "warn" : ""}`;
  $("tTen").className = `tile ${frac > 1 ? "alert" : frac > 0.7 ? "warn" : ""}`;
  $("vTen").innerHTML = `${fmt(r.tension, 2)}<u>N</u>`;
  const peak = hist.length ? Math.max(...hist.map(p => p.tension)) : r.tension;
  $("vTenMax").innerHTML = `${fmt(peak, 2)}<u>N</u>`;
  $("loadNote").textContent =
    `설계하중 ${design} N · 파단 ${fmt(f.limits?.break_load, 0)} N`;

  $("cStretch").textContent = `${fmt(d.stretch_cm, 1)} cm`;
  // 처짐 보정은 장력에 반비례해서 줄이 늘어지면 발산한다. 브리지가 20°에서
  // 자르고 알려주므로, 잘린 값을 그냥 숫자로 보여주지 않는다.
  $("cSag").textContent = d.sag_valid === false
    ? `${fmt(d.sag_deg, 1)}° (범위 밖)` : `${fmt(d.sag_deg, 2)}°`;
  $("cSag").className = d.sag_valid === false ? "delta pos" : "";
  $("cChord").textContent = `${fmt(r.elevation, 1)}° → ${fmt(d.chord_deg, 1)}°`;
  $("cAlt").textContent = `${d.alt_err_cm >= 0 ? "+" : ""}${fmt(d.alt_err_cm, 0)} cm`;

  $("iCl").textContent = d.cl == null ? "—" : fmt(d.cl, 3);
  $("iCd").textContent = d.cd == null ? "—" : fmt(d.cd, 3);
  $("iLd").textContent = d.ld == null ? "—" : fmt(d.ld, 2);
  $("iWind").textContent = d.wind_est == null ? "—"
    : `${fmt(d.wind_est, 2)}  (${fmt(r.wind, 2)})`;
  $("gate").className = `gate ${d.steady ? "open" : ""}`;
  $("gateText").textContent = d.steady
    ? `준정적 · ${d.n_steady}표본` : `과도 · ${d.n_steady}표본`;

  const pv = f.pv;
  if (pv && pv.valid) {
    $("pvNote").textContent = `${pv.points}점`;
    $("pvBody").innerHTML = `
      <div class="tiles">
        <div class="tile"><div class="k">P_max</div>
          <div class="v">${fmt(pv.pmax, 3)}<u>W</u></div></div>
        <div class="tile"><div class="k">입사각 보정</div>
          <div class="v">${pv.normalised == null ? "—" : fmt(pv.normalised, 3)}<u>W</u></div></div>
      </div>
      <table class="kv" style="margin-top:9px">
        <tr><td>V<sub>oc</sub> / I<sub>sc</sub></td>
            <td>${fmt(pv.voc, 2)} V / ${fmt(pv.isc, 3)} A</td></tr>
        <tr><td>V<sub>mp</sub> / I<sub>mp</sub></td>
            <td>${fmt(pv.vmp, 2)} V / ${fmt(pv.imp, 3)} A</td></tr>
        <tr><td>충전율 FF</td><td>${pv.fill == null ? "—" : fmt(pv.fill, 3)}</td></tr>
        <tr><td>입사각<span class="sub">= 탑재물 기울기</span></td>
            <td>${fmt(pv.incidence, 1)}°</td></tr>
      </table>`;
  }

  const p = f.predicted, dv = f.divergence;
  if (p) {
    $("pAlt").textContent = `${fmt(p.z, 1)} m`;
    $("pElev").textContent = `${fmt(p.elevation, 1)}°`;
    $("pTen").textContent = `${fmt(p.tension, 2)} N`;
  }
  const delta = (el, v, unit, digits) => {
    const n = $(el);
    if (v == null) { n.textContent = "—"; n.className = ""; return; }
    n.textContent = `${v >= 0 ? "+" : ""}${fmt(v, digits)} ${unit}`;
    n.className = `delta ${v >= 0 ? "pos" : "neg"}`;
  };
  delta("dElev", dv && dv.elevation, "°", 2);
  delta("dTen", dv && dv.tension, "N", 2);
  delta("dZ", dv && dv.z, "m", 2);
  if (dv) {
    $("divbar").textContent =
      `괴리  ${dv.elevation >= 0 ? "+" : ""}${fmt(dv.elevation, 1)}°  ` +
      `${dv.tension >= 0 ? "+" : ""}${fmt(dv.tension, 2)} N  ` +
      `${dv.z >= 0 ? "+" : ""}${fmt(dv.z, 1)} m`;
  }

  const L = f.link;
  $("lRx").textContent = `${L.received} / ${L.dropped}`;
  $("lImu").textContent = `${L.imu_age_ms} ms · ${L.imu_fields}축`;
  $("lGps").textContent = L.fix ? `fix · ${L.sats}기` : "no fix";
}

/* ───────────────────────── 스트림 ───────────────────────── */

function playBundle() {
  const frames = BUNDLE.frames;
  if (!frames.length) {
    $("connText").textContent = "기록이 비어 있습니다";
    return;
  }
  $("dot").className = "dot on";
  $("connText").textContent = "기록 재생";

  let i = 0;
  let t0 = performance.now();
  const step = () => {
    const elapsed = (performance.now() - t0) / 1000;
    // 기록된 시각을 따라간다. 한 틱에 여러 프레임이 걸리면 건너뛰고 마지막
    // 것만 그린다 — 20 Hz 기록을 60 Hz 화면에서 늘려 재생하지 않는다.
    let drew = false;
    while (i < frames.length && frames[i].t <= elapsed) {
      last = frames[i];
      lastAt = performance.now();
      hist.push({ t: last.t, tension: last.raw.tension,
                  elev: last.derived.chord_deg, wind: last.raw.wind,
                  pitch: last.raw.pitch });
      while (hist.length && last.t - hist[0].t > HIST_SECONDS) hist.shift();
      i += 1;
      drew = true;
    }
    if (drew) paint(last);
    if (i >= frames.length) {       // 끝나면 처음부터
      i = 0;
      t0 = performance.now();
      hist.length = 0;
    }
    setTimeout(step, 40);
  };
  step();
}

function connect() {
  const es = new EventSource("/stream");
  es.onopen = () => {
    $("dot").className = "dot on";
    $("connText").textContent = "연결됨";
  };
  es.onerror = () => {
    $("dot").className = "dot";
    $("connText").textContent = "재연결 중…";
  };
  es.onmessage = ev => {
    let f;
    try { f = JSON.parse(ev.data); } catch { return; }
    last = f;
    lastAt = performance.now();

    rateWin.push(lastAt);
    while (rateWin.length && lastAt - rateWin[0] > 2000) rateWin.shift();
    $("sRate").textContent = `${(rateWin.length / 2).toFixed(0)} Hz`;

    hist.push({
      t: f.t, tension: f.raw.tension, elev: f.derived.chord_deg,
      wind: f.raw.wind, pitch: f.raw.pitch,
    });
    while (hist.length && f.t - hist[0].t > HIST_SECONDS) hist.shift();

    paint(f);
  };
}



/* ───────────────────────── 기록 재생 ───────────────────────── */

async function openLogs() {
  $("logModal").hidden = false;
  await refreshLogs();
}

async function refreshLogs() {
  const host = $("logList");
  host.innerHTML = '<div class="simnote">찾는 중…</div>';
  let d;
  try {
    d = await (await fetch("/api/logs")).json();
  } catch {
    host.innerHTML = '<div class="simnote">기록 목록을 가져오지 못했습니다.</div>';
    return;
  }
  if (!d.logs.length) {
    host.innerHTML =
      '<div class="simnote">아직 기록이 없습니다. 연결된 상태로 잠시 두면 ' +
      "자동으로 쌓입니다.</div>";
    return;
  }
  host.innerHTML = "";
  for (const g of d.logs) {
    const row = document.createElement("div");
    row.className = "portrow";
    row.innerHTML =
      '<span class="pip"></span><span class="nm">' + g.when + "</span>" +
      '<span class="ds">' + g.name + " · " + g.kb + " KB</span>";
    const b = document.createElement("button");
    b.textContent = "재생";
    b.addEventListener("click", () => playLog(g.name));
    row.appendChild(b);
    host.appendChild(row);
  }
}

async function playLog(name) {
  if (switching) return;
  switching = true;
  hist.length = 0;
  last = null;
  blankInstruments();
  try {
    const r = await fetch("/api/replay", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    const m = await r.json();
    if (m.error) throw new Error(m.error);
    applyMeta(m);
    $("logModal").hidden = true;
  } catch (e) {
    console.warn(e);
  } finally {
    switching = false;
  }
}

$("logBtn").addEventListener("click", openLogs);
$("logRefresh").addEventListener("click", refreshLogs);
$("logClose").addEventListener("click", () => { $("logModal").hidden = true; });
$("logModal").addEventListener("click", e => {
  if (e.target === $("logModal")) $("logModal").hidden = true;
});

/* ───────────────────────── 포트 선택 ───────────────────────── */

async function openPorts() {
  $("portModal").hidden = false;
  await refreshPorts();
}

async function refreshPorts() {
  const host = $("portList");
  host.innerHTML = '<div class="simnote">찾는 중…</div>';
  let d;
  try {
    d = await (await fetch("/api/ports")).json();
  } catch (e) {
    host.innerHTML = '<div class="simnote">포트 목록을 가져오지 못했습니다.</div>';
    return;
  }
  if (!d.ports.length) {
    host.innerHTML =
      '<div class="simnote">보이는 포트가 없습니다. 지상국 ESP32를 USB로 연결하거나, ' +
      '아래에 <code>socket://127.0.0.1:5577</code> 을 입력해 모의 리그에 붙으세요.</div>';
    return;
  }
  host.innerHTML = "";
  for (const p of d.ports) {
    const row = document.createElement("div");
    row.className = "portrow" + (p.present ? "" : " absent") +
                    (p.port === d.current ? " on" : "");
    row.innerHTML =
      '<span class="pip"></span><span class="nm">' + p.port + "</span>" +
      '<span class="ds">' + (p.present ? p.label : "지금은 보이지 않음") + "</span>";
    const b = document.createElement("button");
    b.textContent = p.port === d.current ? "다시 연결" : "연결";
    b.addEventListener("click", () => connectPort(p.port));
    row.appendChild(b);
    host.appendChild(row);
  }
  if (d.current) $("portManual").value = d.current;
}

async function connectPort(port) {
  if (!port || switching) return;
  switching = true;
  $("portNote").textContent = port + " 에 연결 중…";
  hist.length = 0;
  last = null;
  blankInstruments();
  try {
    const r = await fetch("/api/port", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port }),
    });
    const m = await r.json();
    if (m.error) throw new Error(m.error);
    applyMeta(m);
    $("portModal").hidden = true;
  } catch (e) {
    $("portNote").textContent = "연결 실패: " + e.message;
  } finally {
    switching = false;
  }
}

$("portBtn").addEventListener("click", openPorts);
$("portRefresh").addEventListener("click", refreshPorts);
$("portClose").addEventListener("click", () => { $("portModal").hidden = true; });
$("portConnect").addEventListener("click",
  () => connectPort($("portManual").value.trim()));
$("portManual").addEventListener("keydown", e => {
  if (e.key === "Enter") connectPort($("portManual").value.trim());
});
$("portModal").addEventListener("click", e => {
  if (e.target === $("portModal")) $("portModal").hidden = true;
});
document.addEventListener("keydown", e => {
  if (e.key !== "Escape") return;
  $("portModal").hidden = true;
  $("logModal").hidden = true;
});

/* 연결이 안 된 상태를 3D 위에 띄우고, 거기서 바로 고치게 한다. 배지만 바뀌면
 * 왜 화면이 비었는지 알 수 없고, 터미널로 돌아가 브리지를 다시 띄우게 된다. */
function showDisconnected(m) {
  let el = $("connBanner");
  const needed = m.source !== "model" && (m.status === "error" || !m.port);
  if (!needed) { if (el) el.remove(); return; }
  if (!el) {
    el = document.createElement("div");
    el.className = "banner";
    el.id = "connBanner";
    $("viewport").appendChild(el);
  }
  el.innerHTML =
    '<div class="inner"><h4>' + (m.port ? "연결 끊김" : "포트 미지정") + "</h4>" +
    "<p>" + (m.error ? m.error : "연결할 포트를 고르세요.") + "</p>" +
    '<button class="apply narrow" id="bannerPick">포트 선택</button></div>';
  el.querySelector("#bannerPick").addEventListener("click", openPorts);
}

/* ───────────────────────── 소스 전환 ───────────────────────── */

const SOURCE_LABEL = { hardware: "실측", emulated: "모의 리그",
                       replay: "재생", model: "시뮬레이터" };

function renderSources(m) {
  const seg = $("sourceSeg");
  // 소스 목록이 없는 응답(에러 등)으로 컨트롤을 지워버리면 전환할 방법이
  // 사라진다. 목록이 올 때만 다시 그린다.
  if (!m.sources || m.sources.length < 2) return;
  seg.innerHTML = "";
  for (const src of m.sources) {
    const b = document.createElement("button");
    b.textContent = SOURCE_LABEL[src.kind] || src.name;
    b.className = src.name === m.current ? "on" : "";
    b.addEventListener("click", async () => {
      if (src.name === m.current || switching) return;
      // 전환 중에는 잠근다. 연타하면 서버에 소스 스레드가 겹쳐 쌓이고,
      // 어느 쪽이 화면에 그려지는지도 순서에 달리게 된다.
      switching = true;
      seg.querySelectorAll("button").forEach(x => { x.disabled = true; });
      hist.length = 0;
      last = null;
      blankInstruments();
      try {
        const r = await fetch("/api/source", {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ name: src.name }),
        });
        const m2 = await r.json();
        if (m2.error) throw new Error(m2.error);
        applyMeta(m2);
      } catch (e) {
        console.warn(e);
        // 실패하면 버튼을 다시 살린다. 안 그러면 되돌아갈 방법이 없다.
        seg.querySelectorAll("button").forEach(x => { x.disabled = false; });
      } finally {
        switching = false;
      }
    });
    seg.appendChild(b);
  }
}

/* ───────────────────────── 시뮬레이터 설정 ───────────────────────── */

const SIM_FIELDS = [
  { k: "wind", label: "기준 풍속", unit: "m/s", min: 2, max: 14, step: 0.1 },
  { k: "turbulence", label: "난류 강도", unit: "σ/U", min: 0, max: 0.3, step: 0.01 },
  { k: "gust_ratio", label: "돌풍 배율", unit: "배", min: 1, max: 2.5, step: 0.05 },
  { k: "gust_dur", label: "돌풍 지속", unit: "s", min: 0.3, max: 6, step: 0.1 },
  { k: "line_length", label: "라인 길이", unit: "m", min: 5, max: 120, step: 1 },
  { k: "ea", label: "축강성 EA", unit: "N", min: 200, max: 30000, step: 100 },
  { k: "area", label: "날개 면적", unit: "m²", min: 0.2, max: 1.6, step: 0.01 },
  { k: "cl", label: "양력계수 C_L", unit: "", min: 0.2, max: 1.6, step: 0.01 },
  { k: "cd", label: "항력계수 C_D", unit: "", min: 0.1, max: 1.2, step: 0.01 },
  { k: "mass_payload", label: "탑재물 질량", unit: "kg", min: 0, max: 1.5, step: 0.01 },
  { k: "suspension", label: "목줄 길이", unit: "m", min: 0.1, max: 3, step: 0.05 },
  { k: "shear", label: "지표 경계층 적용", unit: "", min: 0, max: 1, step: 1, bool: true },
];

let simCfg = null, simDefaults = null, switching = false;

function renderSimFields(cfg) {
  const host = $("simFields");
  host.innerHTML = "";
  for (const f of SIM_FIELDS) {
    const wrap = document.createElement("div");
    wrap.className = "field" + (f.bool ? " check" : "");
    if (f.bool) {
      wrap.innerHTML =
        '<input type="checkbox" id="f_' + f.k + '"' +
        (Number(cfg[f.k]) ? " checked" : "") + ">" +
        '<label for="f_' + f.k + '">' + f.label + "</label>";
    } else {
      wrap.innerHTML =
        "<label>" + f.label + ' <span class="u">' + f.unit + "</span></label>" +
        '<output id="o_' + f.k + '">' + cfg[f.k] + "</output>" +
        '<input type="range" id="f_' + f.k + '" min="' + f.min + '" max="' + f.max +
        '" step="' + f.step + '" value="' + cfg[f.k] + '">';
    }
    host.appendChild(wrap);
    if (!f.bool) {
      const input = wrap.querySelector("input");
      input.addEventListener("input", () => {
        $("o_" + f.k).textContent = input.value;
      });
    }
  }
}

async function applySim() {
  if (switching) return;
  switching = true;
  const patch = {};
  for (const f of SIM_FIELDS) {
    const el = $("f_" + f.k);
    patch[f.k] = f.bool ? (el.checked ? 1 : 0) : Number(el.value);
  }
  $("simApply").disabled = true;
  // 재적분은 수십 초가 걸릴 수 있다. 끝났는지는 /api/meta 의 status 가 말한다.
  $("simStatus").textContent = "재적분 중… (처음 보는 조합이면 30초 정도)";
  hist.length = 0;
  last = null;
  blankInstruments();
  try {
    const r = await fetch("/api/sim/params", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(patch),
    });
    const m = await r.json();
    if (m.error) $("simStatus").textContent = "거부됨: " + m.error;
    else applyMeta(m);
  } catch (e) {
    $("simStatus").textContent = "실패: " + e;
  } finally {
    switching = false;
    $("simApply").disabled = false;
  }
}

$("simApply").addEventListener("click", applySim);
$("simReset").addEventListener("click", () => {
  if (simDefaults) renderSimFields(simDefaults);
});

/* ───────────────────────── 센서 교정값 ───────────────────────── */

function renderSensors(d) {
  if (!d) return;
  $("sensorSrc").textContent = d.source.split("/").pop();
  $("sensorTable").innerHTML = d.error
    ? '<tr><td colspan="2">' + d.error + "</td></tr>"
    : d.values.map(v =>
        "<tr><td>" + v.label + '<span class="sub">' + v.note + "</span></td>" +
        "<td>" + (v.value === null ? "—" : v.value) + " " + v.unit +
        "</td></tr>").join("");
}

async function loadSensors() {
  try {
    renderSensors(await (await fetch("/api/sensors")).json());
  } catch (e) { console.warn(e); }
}

/* ───────────────────────── meta ───────────────────────── */

function applyMeta(m) {
  meta = m;
  const b = $("mode");
  b.textContent = m.mode;
  b.className = "badge" + (m.live ? " live" : "") +
    (m.source === "model" || m.source === "emulated" ? " sim" : "");
  // 실측이 아닌 동안에는 3D 위에도 적는다. 배지 하나는 발표 중에 지나치기 쉽다.
  if (m.source === "model" || m.source === "emulated") {
    $("inferred").innerHTML =
      "<b>모의 데이터</b> — 실제 비행 아님 · 방위각은 탑재물 yaw에서 추정";
  } else {
    $("inferred").textContent = "방위각은 탑재물 yaw에서 추정 — 측정값 아님";
  }
  $("twinNote").textContent =
    m.tether.name + " · EA " + m.tether.EA + " N · 면적 " + m.kite.area + " m²" +
    " · 연·탑재물 " + KITE_SCALE + "× 과장";
  $("lKite").textContent = m.kite.area + " m² · " + fmt(m.kite.mass, 2) + " kg";
  renderSources(m);

  // 시뮬레이터에는 센서도 없고 비교할 실측도 없다. 그 카드들을 띄워두면
  // 모델이 모델을 예측하는 화면이 되고, 괴리는 정의상 0이다.
  const isSim = m.source === "model";
  $("cardSim").hidden = !isSim;
  $("cardSensor").hidden = isSim;
  $("cardDiv").hidden = isSim;
  $("split").hidden = isSim;
  if (isSim && splitOn) $("split").click();
  if (m.sim_cfg) {
    simCfg = m.sim_cfg;
    if (isSim) renderSimFields(simCfg);
  }
  $("simStatus").textContent =
    m.status === "starting" ? "준비 중…"
    : m.status === "error" ? "오류: " + m.error : "";
  if (BUNDLE) {
    renderSensors(BUNDLE.sensors);
    $("cardSim").hidden = true;
    $("cardSensor").hidden = false;
  } else if (!isSim) {
    loadSensors();
  }
  if (!BUNDLE) showDisconnected(m);

  const rec = m.recording;
  $("recState").hidden = !rec;
  if (rec) $("recText").textContent = rec.rows.toLocaleString() + "행";

  // 분할은 소스가 바뀌어도 유지한다 — 비교하던 사람이 매번 다시 켜야 하면
  // 토글을 쓰지 않게 된다.
  const prefs = loadPrefs();
  if (!isSim && prefs.split && !splitOn) $("split").click();
  if (prefs.view && !restoredView) {
    restoredView = true;
    const b = document.querySelector('.viewctl button[data-view="' + prefs.view + '"]');
    if (b) b.click();
  }
}

let restoredView = false;

if (!BUNDLE) {
  fetch("/api/sim/params").then(r => r.json())
    .then(d => { simDefaults = d.defaults; }).catch(() => {});
}

// 링크가 끊긴 것은 프레임이 '안 오는' 일이라 스트림만 봐서는 모른다.
// 상태는 주기적으로 물어봐야 알 수 있다.
if (!BUNDLE) setInterval(async () => {
  if (switching) return;
  try {
    const m = await (await fetch("/api/meta")).json();
    if (meta && m.status !== meta.status) applyMeta(m);
    else if (meta) {
      meta.status = m.status;
      showDisconnected(m);
      $("recState").hidden = !m.recording;
      if (m.recording)
        $("recText").textContent = m.recording.rows.toLocaleString() + "행";
    }
  } catch { /* 브리지가 내려갔다. EventSource 가 알아서 재접속한다. */ }
}, 3000);

if (BUNDLE) {
  for (const id of ["portBtn", "logBtn", "sourceSeg", "recState"]) {
    const n = $(id);
    if (n) n.hidden = true;
  }
  applyMeta(BUNDLE.meta);
} else {
  fetch("/api/meta").then(r => r.json()).then(m => {
    applyMeta(m);
  }).catch(() => {});
}

// 패킷마다가 아니라 프레임 클럭으로 그린다. 20 Hz 텔레메트리에서 비싼 건
// 수신이 아니라 렌더이고, 화면 주사율보다 자주 그리는 건 낭비다.
function loop() {
  requestAnimationFrame(loop);
  if (last && performance.now() - lastAt > 2500) {
    $("dot").className = "dot stale";
    $("connText").textContent = "신호 끊김";
  }
  twin?.apply(last);
  if (splitOn && twinPred && last) twinPred.apply(predictedFrame(last));
  if (splitOn && twinPred && linked) {
    // 마지막으로 만진 쪽이 끌고 간다.
    if (activePane === "pred") twin?.applyCameraState(twinPred.cameraState());
    else twinPred.applyCameraState(twin.cameraState());
  }
  twin?.render();
  if (splitOn && twinPred && last) twinPred.render();
  drawAttitude(last);
  drawStrip();
}

drawAttitude(null);
loop();
if (BUNDLE) playBundle(); else connect();
