/* 3D 디지털 트윈 뷰.
 *
 * 무엇이 측정값이고 무엇이 가정인지가 이 화면의 핵심이다.
 *
 *   측정  — 라인 길이(엔코더), 고도각(지상 각도센서), 장력, 풍속,
 *           탑재물 roll/pitch/yaw (9축 AHRS)
 *   보정  — 늘어남 r = L0(1+T/EA), 처짐 δ = wL·cosθ/2T  (둘 다 장력만으로 계산)
 *   가정  — 방위각. 지상국에는 방위 센서가 없다. 연이 어느 쪽에 있는지는
 *           측정되지 않으며, 여기서는 탑재물 yaw를 비행면 방위로 쓴다.
 *           탑재물이 바람에 정렬해 매달린다는 가정이고, 화면에 그렇게 적는다.
 *
 * 2축 각도센서를 달면 방위각이 가정에서 측정으로 바뀐다. 그 전까지
 * 가로 방향 위치는 참고용이다.
 */

import * as THREE from "./vendor/three.module.js";

const C = {
  sky0: 0x0a1016, sky1: 0x16222c,
  ground: 0x16212a, grid: 0x24323d, gridMajor: 0x2f4452,
  kite: 0x52b9e8, kiteEdge: 0x8ad4f5, spar: 0xb9a37a,
  tether: 0x7d93a3, tetherGhost: 0x33454f,
  payload: 0xe8b341, mast: 0x44586a, wind: 0x3d6274,
  ok: 0x4fc99a, warn: 0xe8b341, bad: 0xe8655f,
};

/* ───────────────────────── 궤도 카메라 ─────────────────────────
 * OrbitControls 애드온 대신 직접 둔다. 필요한 건 궤도·줌·팬 세 가지뿐이고,
 * 애드온 하나 때문에 import 경로를 하나 더 관리할 이유가 없다.
 */
class Orbit {
  constructor(camera, dom, target) {
    this.cam = camera;
    this.dom = dom;
    this.target = target.clone();
    this.theta = -0.62;     // 방위
    this.phi = 1.32;        // 천정각. pi/2 가 수평이고, 작을수록 위에서 내려다본다.
    this.radius = 80;
    this.minPhi = 0.08;
    this.maxPhi = Math.PI / 2 - 0.035;   // 지면 아래로 내려가지 않는다
    this.dragging = null;
    this.auto = false;

    dom.addEventListener("pointerdown", e => {
      this.dragging = e.button === 2 || e.shiftKey ? "pan" : "orbit";
      this.px = e.clientX; this.py = e.clientY;
      dom.setPointerCapture(e.pointerId);
      this.auto = false;
    });
    dom.addEventListener("pointermove", e => {
      if (!this.dragging) return;
      const dx = e.clientX - this.px, dy = e.clientY - this.py;
      this.px = e.clientX; this.py = e.clientY;
      if (this.dragging === "orbit") {
        this.theta -= dx * 0.005;
        this.phi = Math.min(this.maxPhi, Math.max(this.minPhi, this.phi - dy * 0.005));
      } else {
        const s = this.radius * 0.0016;
        const right = new THREE.Vector3(Math.cos(this.theta), 0, -Math.sin(this.theta));
        this.target.addScaledVector(right, -dx * s);
        this.target.y = Math.max(0, this.target.y + dy * s);
      }
    });
    const end = e => { this.dragging = null; try { dom.releasePointerCapture(e.pointerId); } catch {} };
    dom.addEventListener("pointerup", end);
    dom.addEventListener("pointercancel", end);
    dom.addEventListener("contextmenu", e => e.preventDefault());
    dom.addEventListener("wheel", e => {
      e.preventDefault();
      this.radius = Math.min(260, Math.max(10, this.radius * (1 + Math.sign(e.deltaY) * 0.1)));
      this.auto = false;
    }, { passive: false });
  }

  set(theta, phi, radius, target) {
    this.theta = theta; this.phi = phi; this.radius = radius;
    if (target) this.target.copy(target);
  }

  update(dt) {
    if (this.auto) this.theta += dt * 0.085;
    const sp = Math.sin(this.phi), cp = Math.cos(this.phi);
    this.cam.position.set(
      this.target.x + this.radius * sp * Math.sin(this.theta),
      this.target.y + this.radius * cp,
      this.target.z + this.radius * sp * Math.cos(this.theta),
    );
    this.cam.lookAt(this.target);
  }
}

/* ───────────────────────── 방패연 ─────────────────────────
 * 사각 캐노피 + 가운데 방구멍 + 대나무 살. 방구멍은 장식이 아니라
 * 이 연의 공력 특성을 정하는 요소라 형상에 반드시 들어가야 한다.
 */
export const KITE_SCALE = 3.2;   // 형상 과장 배율 (HUD에 표기)

/* 실제 0.67 m^2 연은 약 0.8 x 0.84 m 라, 50 m 거리에서 실물 크기로 그리면
 * 점으로 보인다. 자세를 읽으려면 과장이 필요하고, 과장했다는 사실은
 * 화면에 적는다. */
function buildKite(width = 2.6, height = 3.2) {
  const g = new THREE.Group();
  const hw = width / 2, hh = height / 2;
  const vent = height / 3 / 2;          // 방구멍 지름 = 세로의 1/3

  const shape = new THREE.Shape();
  shape.moveTo(-hw, -hh);
  shape.lineTo(hw, -hh);
  shape.lineTo(hw, hh);
  shape.lineTo(-hw, hh);
  shape.closePath();
  const hole = new THREE.Path();
  hole.absarc(0, 0, vent, 0, Math.PI * 2, true);
  shape.holes.push(hole);

  const canopy = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    new THREE.MeshStandardMaterial({
      color: C.kite, side: THREE.DoubleSide,
      roughness: 0.78, metalness: 0.0,
      emissive: C.kite, emissiveIntensity: 0.1,
    }),
  );
  g.add(canopy);

  const edge = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.ShapeGeometry(shape)),
    new THREE.LineBasicMaterial({ color: C.kiteEdge, transparent: true, opacity: 0.85 }),
  );
  g.add(edge);

  // 대나무 살: 세로 중살, 가로 허릿살, 대각선 둘
  const sparMat = new THREE.MeshStandardMaterial({ color: C.spar, roughness: 0.9 });
  const spar = (len, rot, pos) => {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, len, 6), sparMat);
    m.rotation.z = rot;
    m.position.copy(pos);
    return m;
  };
  g.add(spar(height, 0, new THREE.Vector3(0, 0, 0.02)));
  g.add(spar(width, Math.PI / 2, new THREE.Vector3(0, hh * 0.42, 0.02)));
  const diag = Math.hypot(width, height);
  const a = Math.atan2(height, width);
  g.add(spar(diag, a + Math.PI / 2, new THREE.Vector3(0, 0, 0.02)));
  g.add(spar(diag, -a + Math.PI / 2, new THREE.Vector3(0, 0, 0.02)));

  return g;
}

function buildPayload() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(1.5, 0.22, 1.05),
    new THREE.MeshStandardMaterial({
      color: C.payload, roughness: 0.45, metalness: 0.25,
      emissive: C.payload, emissiveIntensity: 0.14,
    }),
  );
  g.add(body);
  g.add(new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(1.5, 0.22, 1.05)),
    new THREE.LineBasicMaterial({ color: 0xffd98a, transparent: true, opacity: 0.6 }),
  ));
  // 패널 법선: 입사각이 발전량을 좌우하므로 어느 쪽을 보는지 보여준다
  const n = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0.09, 0), 1.1, C.payload, 0.3, 0.16);
  n.line.material.transparent = true; n.line.material.opacity = 0.5;
  g.add(n);
  return g;
}

/* ───────────────────────── 장면 ───────────────────────── */

export class TwinScene {
  constructor(host) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    host.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(C.sky0);
    this.scene.fog = new THREE.Fog(C.sky0, 150, 330);

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.3, 900);
    this.orbit = new Orbit(this.camera, this.renderer.domElement, new THREE.Vector3(14, 14, 0));

    this._lights();
    this._ground();
    this._station();
    this._flight();

    // When two panes are linked, one drives and the other follows. A follower
    // that keeps re-framing itself fights the pane driving it, once per frame.
    this.autoFrame = true;

    this.clock = new THREE.Clock();
    this._resize();
    this._ro = new ResizeObserver(() => this._resize());
    this._ro.observe(host);
  }

  _lights() {
    this.scene.add(new THREE.HemisphereLight(0x486a84, 0x0e1419, 1.5));
    const sun = new THREE.DirectionalLight(0xfff0d8, 1.9);
    sun.position.set(42, 58, 34);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    const d = 60;
    Object.assign(sun.shadow.camera, { left: -d, right: d, top: d, bottom: -d, near: 1, far: 190 });
    this.scene.add(sun);
    this.sun = sun;
  }

  _ground() {
    const ground = new THREE.Mesh(
      new THREE.CircleGeometry(170, 64).rotateX(-Math.PI / 2),
      new THREE.MeshStandardMaterial({ color: C.ground, roughness: 1 }),
    );
    ground.receiveShadow = true;
    ground.position.y = -0.01;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(200, 40, C.gridMajor, C.grid);
    grid.material.transparent = true;
    grid.material.opacity = 0.42;
    this.scene.add(grid);

    // 10 m 간격 거리 링. 고도·거리를 눈대중할 기준이 없으면 3D는 읽히지 않는다.
    for (let r = 10; r <= 90; r += 20) {
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(r - 0.07, r + 0.07, 96).rotateX(-Math.PI / 2),
        new THREE.MeshBasicMaterial({ color: C.gridMajor, transparent: true, opacity: 0.5,
                                      side: THREE.DoubleSide }),
      );
      ring.position.y = 0.02;
      this.scene.add(ring);
    }
  }

  _station() {
    const g = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: C.mast, roughness: 0.6, metalness: 0.3 });

    const base = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.35, 1.6), mat);
    base.position.y = 0.18; base.castShadow = true;
    g.add(base);
    const drum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.42, 0.42, 1.0, 20).rotateZ(Math.PI / 2), mat);
    drum.position.y = 0.72; drum.castShadow = true;
    g.add(drum);

    // 풍속계 마스트 — 설치 높이는 기록해야 하는 값이라 눈에 보이게 둔다
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 10, 8), mat);
    mast.position.set(-5, 5, 3);
    g.add(mast);
    const cup = new THREE.Mesh(
      new THREE.SphereGeometry(0.22, 12, 10),
      new THREE.MeshStandardMaterial({ color: C.ok, emissive: C.ok, emissiveIntensity: 0.3 }));
    cup.position.set(-5, 10.1, 3);
    g.add(cup);
    this.anemo = cup;

    this.scene.add(g);
  }

  _flight() {
    this.kite = buildKite();
    this.kite.castShadow = true;
    this.kite.traverse(o => { if (o.isMesh) o.castShadow = true; });
    this.scene.add(this.kite);

    this.payload = buildPayload();
    this.payload.traverse(o => { if (o.isMesh) o.castShadow = true; });
    this.scene.add(this.payload);

    // 계류줄: 처짐을 반영한 곡선. 보정 전 직선은 흐리게 같이 그린다.
    this.tetherGeo = new THREE.BufferGeometry();
    this.tetherGeo.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(3 * 33), 3));
    this.tether = new THREE.Line(this.tetherGeo,
      new THREE.LineBasicMaterial({ color: C.tether }));
    this.scene.add(this.tether);

    this.ghostGeo = new THREE.BufferGeometry();
    this.ghostGeo.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(3 * 2), 3));
    this.ghost = new THREE.Line(this.ghostGeo,
      new THREE.LineDashedMaterial({ color: C.tetherGhost, dashSize: 1.4, gapSize: 1.0 }));
    this.scene.add(this.ghost);

    this.drop = new THREE.Line(
      new THREE.BufferGeometry().setAttribute("position",
        new THREE.BufferAttribute(new Float32Array(6), 3)),
      new THREE.LineBasicMaterial({ color: C.payload, transparent: true, opacity: 0.65 }));
    this.scene.add(this.drop);

    // 방위 불확실성: 지면에 눕힌 부채꼴. 수직 반투명 판으로 그렸더니 회색 벽이
    // 되어 연을 가렸다 — 가정을 알리자고 측정값을 못 보게 만들 수는 없다.
    // 바닥에 누우면 '연은 이 방향 어딘가'를 말하면서 시야를 비운다.
    const WEDGE = THREE.MathUtils.degToRad(25);
    this.wedge = new THREE.Mesh(
      new THREE.CircleGeometry(1, 40, Math.PI / 2 - WEDGE, 2 * WEDGE)
        .rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: 0x52b9e8, transparent: true,
                                    opacity: 0.09, side: THREE.DoubleSide,
                                    depthWrite: false }),
    );
    this.wedge.position.y = 0.05;
    this.scene.add(this.wedge);

    // 연 바로 아래 지면까지의 수선 — 고도를 눈으로 읽는 기준
    this.plumb = new THREE.Line(
      new THREE.BufferGeometry().setAttribute("position",
        new THREE.BufferAttribute(new Float32Array(6), 3)),
      new THREE.LineDashedMaterial({ color: 0x3f5666, dashSize: 1.1, gapSize: 0.9 }));
    this.scene.add(this.plumb);

    this.windArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0), new THREE.Vector3(-22, 12, 0), 10, C.wind, 2.4, 1.3);
    this.scene.add(this.windArrow);
  }

  _resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  /* 프리셋은 비행면을 기준으로 잡는다.
   *
   * 카메라 방위를 월드 고정값으로 두면 '측면'이 비행면과 나란히 서서 연을
   * 정면에서 보게 된다 — 줄이 수직선 하나로 뭉개지고 고도각이 안 읽힌다.
   * 비행면 방위(az)를 기준으로 삼아야 '측면'이 실제로 옆모습이 된다.
   *
   * az 는 누를 때 한 번만 읽는다. 매 프레임 따라가면 yaw 잡음만큼 화면이 돈다.
   */
  preset(name) {
    const t = this.frameTarget ?? new THREE.Vector3(0, 21, 0);
    const R = this.framedFor ? this.framedFor * 1.5 : 80;
    const az = this.az || 0;
    if (name === "side")  this.orbit.set(az, Math.PI / 2 - 0.06, R, t);
    if (name === "top")   this.orbit.set(az - 0.62, 0.16, R * 1.15, t);
    if (name === "chase") this.orbit.set(az - Math.PI / 2, 1.2, R * 0.75, t);
    if (name === "wide")  this.orbit.set(az - 0.62, 1.32, R, t);
    this.orbit.auto = false;
  }

  toggleSpin() { this.orbit.auto = !this.orbit.auto; return this.orbit.auto; }

  /** 카메라 상태를 꺼내고 넣는다 — 분할 뷰를 묶는 데 쓴다. */
  cameraState() {
    const o = this.orbit;
    return { theta: o.theta, phi: o.phi, radius: o.radius,
             tx: o.target.x, ty: o.target.y, tz: o.target.z, auto: o.auto };
  }

  applyCameraState(c) {
    if (!c) return;
    const o = this.orbit;
    o.theta = c.theta;
    o.phi = c.phi;
    o.radius = c.radius;
    o.target.set(c.tx, c.ty, c.tz);
    o.auto = c.auto;
  }

  /** 이 장면을 사용자가 직접 건드렸는지 알려준다(묶기 방향 결정용). */
  onInteract(fn) {
    this.renderer.domElement.addEventListener("pointerdown", fn);
    this.renderer.domElement.addEventListener("wheel", fn, { passive: true });
  }

  /** 한 프레임의 텔레메트리를 장면에 반영한다. */
  apply(f) {
    if (!f) return;
    const d = f.derived, raw = f.raw;
    const flying = d.chord_deg > 2 && d.z > 0;

    // 방위각: 측정값이 아니라 탑재물 yaw에서 가져온 가정값
    const az = THREE.MathUtils.degToRad(raw.yaw || 0);
    this.az = az;
    const ca = Math.cos(az), sa = Math.sin(az);
    const toWorld = (x, y) => new THREE.Vector3(x * ca, y, -x * sa);

    const elev = THREE.MathUtils.degToRad(d.chord_deg);
    const horiz = d.span * Math.cos(elev);
    const kitePos = toWorld(horiz, Math.max(d.z, 0));

    this.kite.visible = this.payload.visible = flying;
    this.tether.visible = this.ghost.visible = this.drop.visible = flying;
    this.wedge.visible = this.plumb.visible = flying;
    if (!flying) return;

    // 연: 양력이 겉보기바람에 수직이고 줄 바깥쪽이라는 브라이들 가정 →
    // 캐노피 법선이 대략 줄 방향을 향한다. 측정된 자세가 아니라 모델이다.
    this.kite.position.copy(kitePos);
    this.kite.lookAt(0, 0, 0);
    this.kite.rotateY(Math.PI);
    this.kite.rotateZ(THREE.MathUtils.degToRad(raw.roll || 0) * 0.5);

    // 계류줄: 현을 따라가되 처짐만큼 아래로 휘는 2차 곡선
    const sag = Math.tan(THREE.MathUtils.degToRad(d.sag_deg)) * d.span / 2;
    const pos = this.tetherGeo.attributes.position;
    const n = pos.count;
    for (let i = 0; i < n; i++) {
      const u = i / (n - 1);
      const p = kitePos.clone().multiplyScalar(u);
      p.y -= 4 * sag * u * (1 - u);        // 양 끝 0, 가운데 최대
      pos.setXYZ(i, p.x, Math.max(p.y, 0), p.z);
    }
    pos.needsUpdate = true;
    this.tetherGeo.computeBoundingSphere();

    // 보정 전 위치: 엔코더 길이 + 측정 각도 그대로
    const ghostPos = toWorld(d.x_raw, Math.max(d.z_raw, 0));
    const gp = this.ghostGeo.attributes.position;
    gp.setXYZ(0, 0, 0, 0);
    gp.setXYZ(1, ghostPos.x, ghostPos.y, ghostPos.z);
    gp.needsUpdate = true;
    this.ghost.computeLineDistances();

    // 탑재물: 흔들림 각도만큼 기운 채 목줄에 매달린다.
    // 목줄 0.5 m는 라인 50 m 옆에서 1%라 실측 비율로 그리면 연에 붙어버리고,
    // 흔들림 — 이 화면에서 제일 보고 싶은 것 — 이 안 보인다. 연 형상과 같은
    // 배율로 늘려 그리고, 그 사실을 화면에 적는다. 각도는 손대지 않는다.
    const pRaw = toWorld(d.payload_x, d.payload_z);
    const pPos = pRaw.clone().sub(kitePos).multiplyScalar(KITE_SCALE).add(kitePos);
    pPos.y = Math.max(pPos.y, 0);
    this.payload.position.copy(pPos);
    this.payload.rotation.set(
      THREE.MathUtils.degToRad(raw.roll || 0),
      az,
      THREE.MathUtils.degToRad(raw.pitch || 0),
    );
    const dp = this.drop.geometry.attributes.position;
    dp.setXYZ(0, kitePos.x, kitePos.y, kitePos.z);
    dp.setXYZ(1, pPos.x, pPos.y, pPos.z);
    dp.needsUpdate = true;

    this.wedge.scale.setScalar(Math.max(horiz, 1));
    this.wedge.rotation.set(-Math.PI / 2, 0, az - Math.PI / 2);

    const pl = this.plumb.geometry.attributes.position;
    pl.setXYZ(0, kitePos.x, 0, kitePos.z);
    pl.setXYZ(1, kitePos.x, kitePos.y, kitePos.z);
    pl.needsUpdate = true;
    this.plumb.computeLineDistances();

    const ws = Math.max(raw.wind, 0.5);
    this.windArrow.position.set(-26 * ca, 6, 26 * sa);
    this.windArrow.setDirection(new THREE.Vector3(ca, 0, -sa));
    this.windArrow.setLength(3 + ws * 1.5, 2.4, 1.3);

    const frac = raw.tension / (f.limits?.design_load || 50);
    const col = frac > 1 ? C.bad : frac > 0.7 ? C.warn : C.tether;
    this.tether.material.color.setHex(col);

    this._frame(d.span);
  }

  /* 카메라는 고정한다.
   *
   * 처음에는 시선을 연에 붙여 따라가게 했는데, 그러면 연이 화면 가운데
   * 못 박힌 채 세상이 움직인다 — 보고 싶은 건 연의 움직임인데 정확히 그것만
   * 안 보인다. 그래서 시선을 윈치 위 한 점에 묶고, 연이 화면 안에서
   * 돌아다니게 둔다.
   *
   * 다시 잡는 건 라인 길이가 크게 바뀌었을 때뿐이다. 매 프레임 다시 잡으면
   * 장력에 따라 경사거리가 cm 단위로 흔들려 화면이 미세하게 계속 떤다.
   */
  _frame(span) {
    if (this.framedFor && Math.abs(span - this.framedFor) / this.framedFor < 0.25) return;
    this.framedFor = span;
    // 방위각과 무관한 점이라야 yaw가 흔들려도 화면이 따라 흔들리지 않는다.
    // 높이는 비행 범위(고도각 20~60°)의 한가운데에 둔다 — 윈치에 맞추면
    // 화면 절반이 땅이고 연은 위쪽 구석에 걸린다.
    this.frameTarget = new THREE.Vector3(span * 0.22, span * 0.46, 0);
    // frameTarget is still recorded when auto-framing is off, because the
    // view presets are written against it.
    if (!this.autoFrame) return;
    this.orbit.target.copy(this.frameTarget);
    this.orbit.radius = span * 1.5;
    if (!this.aimed) {
      this.aimed = true;
      this.orbit.theta = (this.az || 0) - 0.62;
    }
  }

  render() {
    this.orbit.update(Math.min(this.clock.getDelta(), 0.1));
    this.renderer.render(this.scene, this.camera);
  }
}
