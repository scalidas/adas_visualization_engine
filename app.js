/**
 * ADAS Visualization Engine — app.js
 *
 * Real-time Autonomous Vehicle Perception Visualization Engine
 * Protocol: WebSocket (ws://) default ws://localhost:9002
 *
 * Coordinate Frame:
 *   +X = Forward (longitudinal depth in front of vehicle)
 *   +Y = Right   (lateral offset: positive is right of centerline, negative is left)
 *   +Z = Up      (vertical height above ground)
 *
 * Three.js World Frame Mapping:
 *   Three.js -Z = Vehicle +X (forward into screen)
 *   Three.js +X = Vehicle +Y (screen-right)
 *   Three.js +Y = Vehicle +Z (screen-up)
 *   => toThree(jx, jy, jz) = new THREE.Vector3(jy, jz, -jx)
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ─────────────────────────────────────────────
// Coordinate Mapping Helper
// ─────────────────────────────────────────────

/**
 * Convert vehicle frame (+X=fwd, +Y=right, +Z=up) to Three.js world frame.
 */
function toThree(jx, jy, jz = 0) {
  return new THREE.Vector3(jy, jz, -jx);
}

// ─────────────────────────────────────────────
// Color Palette
// ─────────────────────────────────────────────

const COLORS = {
  vehicle:        0x3B82F6,  // blue
  pedestrian:     0xF59E0B,  // amber
  bottle:         0xF43F5E,  // rose/coral
  obstacle:       0xEF4444,  // red
  cyclist:        0x10B981,  // green
  stopSign:       0xEF4444,  // red
  laneLine:       0xFFFFFF,  // white
  laneCenterline: 0x60A5FA,  // light blue for ego centerline
  ground:              0x040D1A,
  grid:                0x0D2440,
  ego:                 0x60A5FA,  // ego vehicle light blue
  trajectory:          0x00F5D4,  // neon cyan / emerald planned path
  trajectoryEmergency: 0xEF4444,  // emergency stop red
  trajectoryBead:      0x22D3EE,  // waypoint bead cyan
};

// ─────────────────────────────────────────────
// Scene Globals
// ─────────────────────────────────────────────

let renderer, scene, camera, controls;
let sceneObjects = [];         // dynamic meshes/lines cleared per render
let egoVehicleMesh = null;     // persistent ego mesh
let frontLeftWheel = null;     // steerable wheel
let frontRightWheel = null;    // steerable wheel
let currentCam = 'follow';
let animFrameId = null;

const LANE_SAMPLE_RANGE = [0.2, 5.0]; // sample polynomial from 0.2m to 5.0m ahead
const LANE_SAMPLE_STEP = 0.1;         // discrete step 0.1m

// Current world model reference for snapshots/inspection
let latestWorldModel = null;
let lastRenderTime = 0;

// ─────────────────────────────────────────────
// ADASStreamClient — WebSocket Client
// ─────────────────────────────────────────────

class ADASStreamClient {
  constructor(url = 'ws://localhost:9002', options = {}) {
    this.url = this._normalizeUrl(url);
    this.onData = options.onData || null;
    this.onStatus = options.onStatus || null;
    this.onError = options.onError || null;

    this.ws = null;
    this.reconnectTimer = null;
    this.countdownTimer = null;
    this.baseDelay = 2000;    // 2s initial backoff
    this.maxDelay = 10000;    // 10s maximum cap
    this.currentDelay = this.baseDelay;
    this.retryCount = 0;
    this.isManualDisconnect = false;
    this.isPaused = false;

    // Metrics
    this.totalFrames = 0;
    this.lastFrameSeq = null;
    this.lastTimestampUs = null;
    this.prevTimestampUs = null;
    this.fps = 0;
    this.frameCount = 0;
    this.lastFpsCalcTime = performance.now();
  }

  _normalizeUrl(rawUrl) {
    let u = (rawUrl || '').trim();
    if (!u) return 'ws://localhost:9002';
    if (!u.startsWith('ws://') && !u.startsWith('wss://')) {
      if (u.startsWith('http://')) u = 'ws://' + u.slice(7);
      else if (u.startsWith('https://')) u = 'wss://' + u.slice(8);
      else u = 'ws://' + u;
    }
    return u;
  }

  connect(customUrl) {
    if (customUrl) {
      this.url = this._normalizeUrl(customUrl);
    }
    this.isManualDisconnect = false;
    clearTimeout(this.reconnectTimer);
    clearInterval(this.countdownTimer);

    if (this.ws) {
      try {
        this.ws.onopen = null;
        this.ws.onclose = null;
        this.ws.onerror = null;
        this.ws.onmessage = null;
        this.ws.close();
      } catch (e) {}
      this.ws = null;
    }

    this._notifyStatus('connecting', { url: this.url });

    try {
      this.ws = new WebSocket(this.url);
    } catch (err) {
      this._handleError(err);
      this._scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      console.log('[ADAS] Connected to WebSocket at', this.url);
      this.retryCount = 0;
      this.currentDelay = this.baseDelay;
      clearTimeout(this.reconnectTimer);
      clearInterval(this.countdownTimer);
      this._notifyStatus('connected', { url: this.url });
    };

    this.ws.onmessage = (event) => {
      this.totalFrames++;
      this.frameCount++;

      const now = performance.now();
      if (now - this.lastFpsCalcTime >= 500) {
        this.fps = (this.frameCount / ((now - this.lastFpsCalcTime) / 1000));
        this.frameCount = 0;
        this.lastFpsCalcTime = now;
      }

      try {
        const worldModel = JSON.parse(event.data);

        if (worldModel.timestamp_us) {
          this.prevTimestampUs = this.lastTimestampUs;
          this.lastTimestampUs = worldModel.timestamp_us;
        }
        if (worldModel.frame_seq != null) {
          this.lastFrameSeq = worldModel.frame_seq;
        }

        if (!this.isPaused && this.onData) {
          this.onData(worldModel, {
            fps: this.fps,
            totalFrames: this.totalFrames,
            prevTs: this.prevTimestampUs,
            currTs: this.lastTimestampUs,
          });
        }
      } catch (err) {
        console.error('[ADAS] Error parsing WorldModel JSON:', err);
        if (this.onError) this.onError(err);
      }
    };

    this.ws.onclose = () => {
      if (this.isManualDisconnect) {
        this._notifyStatus('disconnected', { url: this.url, manual: true });
        return;
      }
      this._scheduleReconnect();
    };

    this.ws.onerror = (err) => {
      console.warn('[ADAS] WebSocket connection error:', err);
      if (this.onError) this.onError(err);
      // onclose will trigger following error to handle reconnect
    };
  }

  _scheduleReconnect() {
    this.retryCount++;
    const delay = this.currentDelay;
    // Exponential backoff
    this.currentDelay = Math.min(this.currentDelay * 1.5, this.maxDelay);

    let remainingSeconds = Math.round(delay / 1000);
    this._notifyStatus('reconnecting', {
      url: this.url,
      retryCount: this.retryCount,
      delaySeconds: remainingSeconds,
    });

    clearInterval(this.countdownTimer);
    this.countdownTimer = setInterval(() => {
      remainingSeconds--;
      if (remainingSeconds > 0) {
        this._notifyStatus('reconnecting', {
          url: this.url,
          retryCount: this.retryCount,
          delaySeconds: remainingSeconds,
        });
      } else {
        clearInterval(this.countdownTimer);
      }
    }, 1000);

    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => {
      clearInterval(this.countdownTimer);
      this.connect();
    }, delay);
  }

  disconnect() {
    this.isManualDisconnect = true;
    clearTimeout(this.reconnectTimer);
    clearInterval(this.countdownTimer);
    if (this.ws) {
      try {
        this.ws.close();
      } catch (e) {}
      this.ws = null;
    }
    this._notifyStatus('disconnected', { url: this.url, manual: true });
  }

  setPaused(paused) {
    this.isPaused = paused;
  }

  _notifyStatus(status, details = {}) {
    if (this.onStatus) this.onStatus(status, details);
  }

  _handleError(err) {
    if (this.onError) this.onError(err);
  }
}

// Stream client instance
let streamClient = null;

// ─────────────────────────────────────────────
// Three.js Initialization & Scene Setup
// ─────────────────────────────────────────────

function initThree() {
  const canvas = document.getElementById('three-canvas');
  const container = document.getElementById('viewport-container');

  renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: false,
    powerPreference: 'high-performance',
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x040D1A);
  scene.fog = new THREE.FogExp2(0x040D1A, 0.04);

  camera = new THREE.PerspectiveCamera(55, 1, 0.01, 100);
  setCameraPreset('follow');

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.rotateSpeed = 0.6;
  controls.panSpeed = 0.8;
  controls.zoomSpeed = 1.0;
  controls.minDistance = 0.2;
  controls.maxDistance = 25;
  controls.target.set(0, 0, 0);

  // Lighting
  const ambLight = new THREE.AmbientLight(0x0A1E35, 4.0);
  scene.add(ambLight);

  const dirLight = new THREE.DirectionalLight(0xA0C4FF, 2.2);
  dirLight.position.set(2, 6, -3);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 1024;
  dirLight.shadow.mapSize.height = 1024;
  scene.add(dirLight);

  const fillLight = new THREE.DirectionalLight(0x3B82F6, 0.6);
  fillLight.position.set(-3, 2, 2);
  scene.add(fillLight);

  // Ground plane & coordinate grid
  buildGround();

  // Create persistent ego car at origin
  egoVehicleMesh = buildEgoVehicle();
  scene.add(egoVehicleMesh);

  // Resize handler
  onResize();
  window.addEventListener('resize', onResize);

  // Start animation loop
  renderLoop();
}

function onResize() {
  const container = document.getElementById('viewport-container');
  if (!container || !renderer || !camera) return;
  const w = container.clientWidth;
  const h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function renderLoop() {
  animFrameId = requestAnimationFrame(renderLoop);
  controls.update();
  renderer.render(scene, camera);
}

function buildGround() {
  // Ground plane
  const geo = new THREE.PlaneGeometry(60, 60);
  const mat = new THREE.MeshStandardMaterial({
    color: COLORS.ground,
    roughness: 0.95,
    metalness: 0.0,
  });
  const ground = new THREE.Mesh(geo, mat);
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  // Grid helper
  const grid = new THREE.GridHelper(60, 120, COLORS.grid, COLORS.grid);
  grid.material.opacity = 0.35;
  grid.material.transparent = true;
  scene.add(grid);

  // Forward longitudinal distance markers (tick lines every 1m along -Z)
  const markerGroup = new THREE.Group();
  for (let m = 1; m <= 10; m++) {
    const pts = [
      new THREE.Vector3(-0.4, 0.001, -m),
      new THREE.Vector3( 0.4, 0.001, -m),
    ];
    const markerGeo = new THREE.BufferGeometry().setFromPoints(pts);
    const markerMat = new THREE.LineBasicMaterial({
      color: 0x1A3A60,
      transparent: true,
      opacity: 0.4,
    });
    markerGroup.add(new THREE.Line(markerGeo, markerMat));
  }
  scene.add(markerGroup);
}

// ─────────────────────────────────────────────
// Camera Presets
// ─────────────────────────────────────────────

function setCameraPreset(preset) {
  currentCam = preset;
  if (!camera) return;

  switch (preset) {
    case 'follow':
      // Tesla-style 3rd-person follow: elevated behind car looking forward along -Z
      camera.position.set(0, 1.3, 2.4);
      if (controls) controls.target.set(0, 0.1, -1.2);
      break;
    case 'top':
      // Bird's Eye View (BEV) top-down
      camera.position.set(0, 6.5, -0.01);
      if (controls) controls.target.set(0, 0, -1.5);
      break;
    case 'side':
      // Side profile view
      camera.position.set(3.2, 1.2, -0.8);
      if (controls) controls.target.set(0, 0.2, -0.8);
      break;
  }
  camera.updateProjectionMatrix();
}

// ─────────────────────────────────────────────
// Stylized Ego Vehicle
// ─────────────────────────────────────────────

function buildEgoVehicle() {
  const group = new THREE.Group();

  // Main chassis body
  const bodyGeo = new THREE.BoxGeometry(0.20, 0.10, 0.32);
  const bodyMat = new THREE.MeshStandardMaterial({
    color: COLORS.ego,
    roughness: 0.3,
    metalness: 0.7,
    emissive: new THREE.Color(COLORS.ego),
    emissiveIntensity: 0.18,
  });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.position.y = 0.07;
  body.castShadow = true;
  group.add(body);

  // Cabin
  const cabinGeo = new THREE.BoxGeometry(0.16, 0.07, 0.18);
  const cabinMat = new THREE.MeshStandardMaterial({
    color: 0x0A1E35,
    roughness: 0.1,
    metalness: 0.85,
    transparent: true,
    opacity: 0.88,
  });
  const cabin = new THREE.Mesh(cabinGeo, cabinMat);
  cabin.position.set(0, 0.145, 0.02);
  group.add(cabin);

  // Wheels (4 cylinders)
  const wheelGeo = new THREE.CylinderGeometry(0.035, 0.035, 0.025, 16);
  const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1A1A2E, roughness: 0.9, metalness: 0.2 });

  // Rear wheels (fixed)
  const rwL = new THREE.Mesh(wheelGeo, wheelMat);
  rwL.rotation.z = Math.PI / 2;
  rwL.position.set(-0.115, 0.035, 0.10);
  rwL.castShadow = true;
  group.add(rwL);

  const rwR = new THREE.Mesh(wheelGeo, wheelMat);
  rwR.rotation.z = Math.PI / 2;
  rwR.position.set(0.115, 0.035, 0.10);
  rwR.castShadow = true;
  group.add(rwR);

  // Front wheels (steerable pivot containers)
  frontLeftWheel = new THREE.Group();
  frontLeftWheel.position.set(-0.115, 0.035, -0.10);
  const fwLMesh = new THREE.Mesh(wheelGeo, wheelMat);
  fwLMesh.rotation.z = Math.PI / 2;
  fwLMesh.castShadow = true;
  frontLeftWheel.add(fwLMesh);
  group.add(frontLeftWheel);

  frontRightWheel = new THREE.Group();
  frontRightWheel.position.set(0.115, 0.035, -0.10);
  const fwRMesh = new THREE.Mesh(wheelGeo, wheelMat);
  fwRMesh.rotation.z = Math.PI / 2;
  fwRMesh.castShadow = true;
  frontRightWheel.add(fwRMesh);
  group.add(frontRightWheel);

  // Headlights
  const lightGeo = new THREE.BoxGeometry(0.04, 0.02, 0.01);
  const lightMat = new THREE.MeshStandardMaterial({
    color: 0xFFFFEE,
    emissive: new THREE.Color(0xFFFFCC),
    emissiveIntensity: 3.0,
    roughness: 0.1,
  });
  for (const xOff of [-0.07, 0.07]) {
    const hl = new THREE.Mesh(lightGeo, lightMat);
    hl.position.set(xOff, 0.07, -0.162);
    group.add(hl);
  }

  // Tail lights
  const tlMat = new THREE.MeshStandardMaterial({
    color: 0xFF2200,
    emissive: new THREE.Color(0xFF1100),
    emissiveIntensity: 2.0,
  });
  for (const xOff of [-0.07, 0.07]) {
    const tl = new THREE.Mesh(lightGeo, tlMat);
    tl.position.set(xOff, 0.07, 0.162);
    group.add(tl);
  }

  // Headlight illumination forward
  const headlightPoint = new THREE.PointLight(0xAABBFF, 0.7, 2.0);
  headlightPoint.position.set(0, 0.1, -0.28);
  group.add(headlightPoint);

  // Vehicle glow outline
  const outlineGeo = new THREE.BoxGeometry(0.205, 0.105, 0.325);
  const outlineMat = new THREE.MeshBasicMaterial({
    color: COLORS.ego,
    wireframe: true,
    transparent: true,
    opacity: 0.14,
  });
  const outline = new THREE.Mesh(outlineGeo, outlineMat);
  outline.position.y = 0.07;
  group.add(outline);

  return group;
}

// ─────────────────────────────────────────────
// Lane Polynomial Modeling & Geometry
// ─────────────────────────────────────────────

/**
 * Sample 2nd-degree polynomial (with optional 3rd degree):
 *   y(x) = c0 + c1*x + c2*x^2 + c3*x^3
 *
 * x = forward longitudinal distance along path in meters (x >= 0)
 * y = lateral position in meters (+ right, - left)
 *
 * Samples x from xMin to xMax in step increments (default 0.2m to 5.0m in 0.1m)
 */
function samplePolynomial(boundary, xMin = LANE_SAMPLE_RANGE[0], xMax = LANE_SAMPLE_RANGE[1], step = LANE_SAMPLE_STEP) {
  if (!boundary || boundary.valid === false) return null;

  const c0 = Number(boundary.c0) || 0;
  const c1 = Number(boundary.c1) || 0;
  const c2 = Number(boundary.c2) || 0;
  const c3 = Number(boundary.c3) || 0;

  const pts = [];
  const count = Math.max(2, Math.round((xMax - xMin) / step));
  for (let i = 0; i <= count; i++) {
    const jx = xMin + (i / count) * (xMax - xMin);
    const jy = c0 + c1 * jx + c2 * jx * jx + c3 * jx * jx * jx;
    pts.push(toThree(jx, jy, 0));
  }
  return pts;
}

function buildLaneLine(points, color, dashed = false) {
  if (!points || points.length < 2) return null;

  const geo = new THREE.BufferGeometry().setFromPoints(points);
  let mat;
  if (dashed) {
    mat = new THREE.LineDashedMaterial({
      color,
      dashSize: 0.14,
      gapSize: 0.10,
      transparent: true,
      opacity: 0.85,
    });
  } else {
    mat = new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity: 0.95,
    });
  }

  const line = new THREE.Line(geo, mat);
  if (dashed) line.computeLineDistances();
  return line;
}

function buildLaneFill(leftPts, rightPts, isEgo = false) {
  if (!leftPts || !rightPts) return null;
  const nPts = Math.min(leftPts.length, rightPts.length);
  const vertices = [];
  const indices = [];

  for (let i = 0; i < nPts; i++) {
    vertices.push(leftPts[i].x,  leftPts[i].y,  leftPts[i].z);
    vertices.push(rightPts[i].x, rightPts[i].y, rightPts[i].z);
  }
  for (let i = 0; i < nPts - 1; i++) {
    const a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
    indices.push(a, b, c,  b, d, c);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();

  const mat = new THREE.MeshBasicMaterial({
    color: isEgo ? 0x1E3A66 : 0x0F2540,
    transparent: true,
    opacity: isEgo ? 0.22 : 0.08,
    side: THREE.DoubleSide,
    depthWrite: false,
  });

  return new THREE.Mesh(geo, mat);
}

function buildLaneGeometry(lane, isEgo = false) {
  const group = new THREE.Group();
  const xMin = LANE_SAMPLE_RANGE[0];
  const xMax = LANE_SAMPLE_RANGE[1];

  const leftPts   = samplePolynomial(lane.left_boundary,  xMin, xMax);
  const rightPts  = samplePolynomial(lane.right_boundary, xMin, xMax);
  const centerPts = samplePolynomial(lane.centerline,    xMin, xMax);

  // Surface corridor fill
  const fill = buildLaneFill(leftPts, rightPts, isEgo);
  if (fill) {
    fill.position.y = 0.001;
    group.add(fill);
  }

  // Left boundary line
  if (leftPts) {
    const leftLine = buildLaneLine(leftPts, COLORS.laneLine, false);
    if (leftLine) {
      leftLine.position.y = 0.003;
      group.add(leftLine);
    }
  }

  // Right boundary line
  if (rightPts) {
    const rightLine = buildLaneLine(rightPts, COLORS.laneLine, false);
    if (rightLine) {
      rightLine.position.y = 0.003;
      group.add(rightLine);
    }
  }

  // Centerline (dashed)
  if (centerPts) {
    const centerLine = buildLaneLine(centerPts, isEgo ? COLORS.laneCenterline : 0x7FA8CC, true);
    if (centerLine) {
      centerLine.position.y = 0.002;
      group.add(centerLine);
    }
  }

  return group;
}

// ─────────────────────────────────────────────
// Obstacle 3D Bounding Boxes
// ─────────────────────────────────────────────

function getObjectColor(label = '') {
  const l = String(label).toLowerCase();
  if (['vehicle', 'car', 'truck', 'bus', 'van'].some(k => l.includes(k))) return COLORS.vehicle;
  if (['pedestrian', 'person', 'walker'].some(k => l.includes(k))) return COLORS.pedestrian;
  if (['bottle', 'cup', 'can'].some(k => l.includes(k))) return COLORS.bottle;
  if (['cyclist', 'bicycle', 'bike'].some(k => l.includes(k))) return COLORS.cyclist;
  if (['stop_sign', 'sign'].some(k => l.includes(k))) return COLORS.stopSign;
  return COLORS.obstacle;
}

function buildObstacleBBox(obj) {
  const group = new THREE.Group();
  const color = getObjectColor(obj.label);

  // Dimensions: width = lateral (Three X), height = vertical (Three Y), length = longitudinal (Three Z)
  const threeW = Math.max(obj.width  || 0.08, 0.05);
  const threeH = Math.max(obj.height || 0.08, 0.05);
  const threeD = Math.max(obj.length || 0.08, 0.05);

  const geo = new THREE.BoxGeometry(threeW, threeH, threeD);

  // Translucent volumetric fill
  const fillMat = new THREE.MeshStandardMaterial({
    color,
    transparent: true,
    opacity: 0.16,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const fillMesh = new THREE.Mesh(geo, fillMat);
  group.add(fillMesh);

  // Edges wireframe
  const edges = new THREE.EdgesGeometry(geo);
  const lineMat = new THREE.LineBasicMaterial({
    color,
    transparent: true,
    opacity: 0.9,
  });
  const wireframe = new THREE.LineSegments(edges, lineMat);
  group.add(wireframe);

  // Corner brackets (Tesla-style)
  addCornerBrackets(group, threeW, threeH, threeD, color);

  // Position: map vehicle frame (+X=fwd, +Y=right, +Z=up)
  const posX = obj.position?.x ?? 0;
  const posY = obj.position?.y ?? 0;
  const posZ = obj.position?.z ?? 0;
  const pos = toThree(posX, posY, posZ);

  // Center box vertically around position.z base
  group.position.set(pos.x, pos.y + threeH / 2, pos.z);

  // Label text sprite
  const confPct = ((obj.confidence ?? 0) * 100).toFixed(0);
  const labelText = `${(obj.label || 'OBJ').toUpperCase()} #${obj.id} ${confPct}%`;
  const sprite = makeTextSprite(labelText, color);
  sprite.position.set(0, threeH / 2 + 0.07, 0);
  group.add(sprite);

  // Velocity vector arrow
  if (obj.velocity) {
    const vx = obj.velocity.x || 0;
    const vy = obj.velocity.y || 0;
    const vz = obj.velocity.z || 0;
    const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
    if (speed > 0.05) {
      const dir = toThree(vx, vy, vz).normalize();
      const arrowLen = Math.min(speed * 0.3, 0.6);
      const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), arrowLen, color, 0.04, 0.03);
      group.add(arrow);
    }
  }

  return group;
}

function addCornerBrackets(group, w, h, d, color) {
  const bLen = Math.min(w, h, d) * 0.28;
  const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 1.0 });

  const hw = w / 2, hh = h / 2, hd = d / 2;
  const corners = [
    [-hw, -hh, -hd], [ hw, -hh, -hd],
    [-hw,  hh, -hd], [ hw,  hh, -hd],
    [-hw, -hh,  hd], [ hw, -hh,  hd],
    [-hw,  hh,  hd], [ hw,  hh,  hd],
  ];
  const axes = [
    [bLen, 0, 0], [0, bLen, 0], [0, 0, bLen],
  ];

  for (const [cx, cy, cz] of corners) {
    for (const [ax, ay, az] of axes) {
      const sx = Math.sign(cx) || 1;
      const sy = Math.sign(cy) || 1;
      const sz = Math.sign(cz) || 1;

      const start = new THREE.Vector3(cx, cy, cz);
      const end   = new THREE.Vector3(cx - sx * ax, cy - sy * ay, cz - sz * az);

      const geo = new THREE.BufferGeometry().setFromPoints([start, end]);
      const line = new THREE.Line(geo, mat);
      group.add(line);
    }
  }
}

function makeTextSprite(text, color = 0xFFFFFF) {
  const canvas = document.createElement('canvas');
  canvas.width  = 300;
  canvas.height = 70;
  const ctx = canvas.getContext('2d');

  const hexColor = '#' + color.toString(16).padStart(6, '0');
  ctx.fillStyle = 'rgba(4, 13, 26, 0.88)';
  roundRect(ctx, 4, 8, 292, 54, 10);
  ctx.fill();
  ctx.strokeStyle = hexColor;
  ctx.lineWidth = 2;
  roundRect(ctx, 4, 8, 292, 54, 10);
  ctx.stroke();

  ctx.fillStyle = hexColor;
  ctx.font = 'bold 22px Inter, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 150, 35);

  const texture = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(0.36, 0.084, 1);
  return sprite;
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.lineTo(x + w - r, y);
  ctx.quadraticCurveTo(x + w, y, x + w, y + r);
  ctx.lineTo(x + w, y + h - r);
  ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  ctx.lineTo(x + r, y + h);
  ctx.quadraticCurveTo(x, y + h, x, y + h - r);
  ctx.lineTo(x, y + r);
  ctx.quadraticCurveTo(x, y, x + r, y);
  ctx.closePath();
}

// ─────────────────────────────────────────────
// Traffic Signs
// ─────────────────────────────────────────────

function buildTrafficSign(sign) {
  const group = new THREE.Group();

  const posX = sign.position?.x ?? 0;
  const posY = sign.position?.y ?? 0;
  const posZ = sign.position?.z ?? 0;
  const pos = toThree(posX, posY, posZ);

  const signHeight = Math.max(pos.y, 0.35);

  if (sign.type === 'STOP_SIGN' || (sign.type && sign.type.includes('STOP'))) {
    // Octagonal stop sign plate
    const octGeo = new THREE.CylinderGeometry(0.06, 0.06, 0.006, 8);
    const octMat = new THREE.MeshStandardMaterial({
      color: 0xCC0000,
      emissive: new THREE.Color(0x990000),
      emissiveIntensity: 0.5,
      roughness: 0.4,
    });
    const oct = new THREE.Mesh(octGeo, octMat);
    oct.rotation.y = Math.PI / 8;
    oct.rotation.x = Math.PI / 2;
    group.add(oct);

    // Support post down to ground
    const postGeo = new THREE.CylinderGeometry(0.006, 0.006, signHeight, 8);
    const postMat = new THREE.MeshStandardMaterial({ color: 0x888888, roughness: 0.7 });
    const post = new THREE.Mesh(postGeo, postMat);
    post.position.y = -signHeight / 2;
    group.add(post);

    // "STOP" sprite
    const label = makeTextSprite('STOP', 0xFF4444);
    label.position.set(0, 0.09, 0);
    group.add(label);
  } else {
    // Generic diamond/rectangle sign
    const signGeo = new THREE.BoxGeometry(0.08, 0.08, 0.006);
    const signMat = new THREE.MeshStandardMaterial({ color: 0xF59E0B, roughness: 0.5 });
    const signMesh = new THREE.Mesh(signGeo, signMat);
    group.add(signMesh);

    const postGeo = new THREE.CylinderGeometry(0.006, 0.006, signHeight, 8);
    const postMat = new THREE.MeshStandardMaterial({ color: 0x888888, roughness: 0.7 });
    const post = new THREE.Mesh(postGeo, postMat);
    post.position.y = -signHeight / 2;
    group.add(post);

    const label = makeTextSprite((sign.type || 'SIGN').replace('_', ' '), 0xF59E0B);
    label.position.set(0, 0.09, 0);
    group.add(label);
  }

  group.position.set(pos.x, signHeight, pos.z);
  return group;
}

// ─────────────────────────────────────────────
// Planned Trajectory Geometry
// ─────────────────────────────────────────────

function buildTrajectoryGeometry(trajectory) {
  if (!trajectory || !Array.isArray(trajectory.points) || trajectory.points.length === 0) {
    return null;
  }

  const group = new THREE.Group();
  const points = trajectory.points;
  const isEmergency = Boolean(trajectory.is_emergency_stop);
  const ribbonColor = isEmergency ? 0xEF4444 : 0x06B6D4;
  const emissiveColor = isEmergency ? 0xEF4444 : 0x00F5D4;

  // Convert points to Three.js coordinates
  // Elevation Y = 0.015m sits nicely above ground plane and lane fill
  const threePts = points.map(p => toThree(p.x, p.y, 0.015));

  // If we have >= 2 points, construct the 3D ribbon path
  if (threePts.length >= 2) {
    const ribbonHalfWidth = 0.09; // 18cm total width ribbon
    const vertices = [];
    const indices = [];

    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      const tPt = threePts[i];
      const yaw = Number(p.yaw) || 0;

      // In vehicle frame, +X is fwd, +Y is right.
      // Unit normal to the right in vehicle frame: (-sin(yaw), cos(yaw))
      // In Three.js: X = Y_veh, Z = -X_veh
      let nx = Math.cos(yaw);
      let nz = Math.sin(yaw);

      // Smooth blend with segment tangent
      if (i < points.length - 1) {
        const dx = threePts[i + 1].x - tPt.x;
        const dz = threePts[i + 1].z - tPt.z;
        const len = Math.sqrt(dx * dx + dz * dz);
        if (len > 0.001) {
          const segNx = -dz / len;
          const segNz = dx / len;
          nx = (nx + segNx) * 0.5;
          nz = (nz + segNz) * 0.5;
          const nLen = Math.sqrt(nx * nx + nz * nz);
          if (nLen > 0.001) { nx /= nLen; nz /= nLen; }
        }
      }

      // Left and right vertices
      vertices.push(tPt.x - nx * ribbonHalfWidth, 0.012, tPt.z - nz * ribbonHalfWidth);
      vertices.push(tPt.x + nx * ribbonHalfWidth, 0.012, tPt.z + nz * ribbonHalfWidth);

      if (i < points.length - 1) {
        const a = i * 2;
        const b = i * 2 + 1;
        const c = i * 2 + 2;
        const d = i * 2 + 3;
        indices.push(a, b, c, b, d, c);
      }
    }

    const ribbonGeo = new THREE.BufferGeometry();
    ribbonGeo.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    ribbonGeo.setIndex(indices);
    ribbonGeo.computeVertexNormals();

    const ribbonMat = new THREE.MeshStandardMaterial({
      color: ribbonColor,
      emissive: emissiveColor,
      emissiveIntensity: isEmergency ? 0.85 : 0.45,
      roughness: 0.3,
      metalness: 0.1,
      transparent: true,
      opacity: isEmergency ? 0.75 : 0.50,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    group.add(new THREE.Mesh(ribbonGeo, ribbonMat));

    // Centerline path along waypoints
    const lineGeo = new THREE.BufferGeometry().setFromPoints(
      threePts.map(pt => new THREE.Vector3(pt.x, 0.016, pt.z))
    );
    const lineMat = new THREE.LineBasicMaterial({
      color: isEmergency ? 0xFCA5A5 : 0xFFFFFF,
      linewidth: 2,
      transparent: true,
      opacity: 0.95,
    });
    group.add(new THREE.Line(lineGeo, lineMat));
  }

  // Waypoint discs / beads & directional markers
  const beadGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.004, 16);
  const beadMat = new THREE.MeshStandardMaterial({
    color: isEmergency ? 0xEF4444 : 0x00F5D4,
    emissive: isEmergency ? 0xFF2222 : 0x00E5C0,
    emissiveIntensity: 0.6,
    roughness: 0.2,
  });

  for (let i = 0; i < points.length; i++) {
    const pt = points[i];
    const tPt = threePts[i];

    // Waypoint disc on road
    const bead = new THREE.Mesh(beadGeo, beadMat);
    bead.position.set(tPt.x, 0.018, tPt.z);
    group.add(bead);

    // Direction arrow / heading indicator for each waypoint
    const yaw = Number(pt.yaw) || 0;
    const dirX = Math.sin(yaw);
    const dirZ = -Math.cos(yaw);
    const dir = new THREE.Vector3(dirX, 0, dirZ).normalize();

    const arrow = new THREE.ArrowHelper(
      dir,
      new THREE.Vector3(tPt.x, 0.022, tPt.z),
      0.08,
      isEmergency ? 0xFCA5A5 : 0x22D3EE,
      0.03,
      0.025
    );
    group.add(arrow);
  }

  // Final destination beacon & target velocity text tag
  if (points.length > 0) {
    const lastPt = points[points.length - 1];
    const lastTPt = threePts[threePts.length - 1];

    // End beacon ring
    const ringGeo = new THREE.RingGeometry(0.04, 0.065, 24);
    const ringMat = new THREE.MeshBasicMaterial({
      color: isEmergency ? 0xEF4444 : 0x00F5D4,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.85,
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(lastTPt.x, 0.020, lastTPt.z);
    group.add(ring);

    // Floating billboard label
    const tagText = isEmergency
      ? 'EMERGENCY STOP'
      : `v_tgt: ${(lastPt.target_velocity ?? 0).toFixed(1)} m/s`;
    const labelSprite = makeTextSprite(tagText, isEmergency ? 0xEF4444 : 0x00F5D4);
    labelSprite.position.set(lastTPt.x, 0.14, lastTPt.z);
    labelSprite.scale.set(0.38, 0.09, 1);
    group.add(labelSprite);
  }

  return group;
}

// ─────────────────────────────────────────────
// 2D Camera Viewfinder Overlay (640×480 Camera Space)
// ─────────────────────────────────────────────

function render2DCameraView(obstacles = [], signs = [], trajectory = null) {
  const canvas = document.getElementById('cam2d-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;  // 640
  const h = canvas.height; // 480

  // Clear dark background
  ctx.fillStyle = '#030A14';
  ctx.fillRect(0, 0, w, h);

  // Camera grid / viewfinder guides
  ctx.strokeStyle = 'rgba(26, 58, 96, 0.45)';
  ctx.lineWidth = 1;

  // Grid lines
  ctx.beginPath();
  ctx.moveTo(w / 2, 0);
  ctx.lineTo(w / 2, h);
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  // Artificial Horizon pitch lines
  ctx.strokeStyle = 'rgba(96, 165, 250, 0.25)';
  ctx.beginPath();
  ctx.moveTo(w * 0.25, h * 0.52);
  ctx.lineTo(w * 0.75, h * 0.52);
  ctx.stroke();

  // Center optical crosshair reticle
  ctx.strokeStyle = 'rgba(96, 165, 250, 0.6)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  // Center cross
  ctx.moveTo(w / 2 - 12, h / 2); ctx.lineTo(w / 2 + 12, h / 2);
  ctx.moveTo(w / 2, h / 2 - 12); ctx.lineTo(w / 2, h / 2 + 12);
  ctx.stroke();

  // Draw 2D bounding boxes
  let boxCount = 0;
  for (const obs of obstacles) {
    if (!obs.bounding_box) continue;
    const bb = obs.bounding_box;
    const bx = Number(bb.x) || 0;
    const by = Number(bb.y) || 0;
    const bw = Number(bb.width) || 0;
    const bh = Number(bb.height) || 0;

    if (bw <= 0 || bh <= 0) continue;
    boxCount++;

    const colorHex = '#' + getObjectColor(obs.label).toString(16).padStart(6, '0');

    // Translucent box fill
    ctx.fillStyle = colorHex + '22';
    ctx.fillRect(bx, by, bw, bh);

    // Box border
    ctx.strokeStyle = colorHex;
    ctx.lineWidth = 2;
    ctx.strokeRect(bx, by, bw, bh);

    // Corner accents
    const cLen = Math.min(bw, bh) * 0.22;
    ctx.strokeStyle = '#FFFFFF';
    ctx.lineWidth = 2.5;

    // Top-Left
    ctx.beginPath();
    ctx.moveTo(bx, by + cLen); ctx.lineTo(bx, by); ctx.lineTo(bx + cLen, by);
    ctx.stroke();

    // Top-Right
    ctx.beginPath();
    ctx.moveTo(bx + bw - cLen, by); ctx.lineTo(bx + bw, by); ctx.lineTo(bx + bw, by + cLen);
    ctx.stroke();

    // Bottom-Left
    ctx.beginPath();
    ctx.moveTo(bx, by + bh - cLen); ctx.lineTo(bx, by + bh); ctx.lineTo(bx + cLen, by + bh);
    ctx.stroke();

    // Bottom-Right
    ctx.beginPath();
    ctx.moveTo(bx + bw - cLen, by + bh); ctx.lineTo(bx + bw, by + bh); ctx.lineTo(bx + bw, by + bh - cLen);
    ctx.stroke();

    // Label banner
    const confPct = ((obs.confidence ?? 0) * 100).toFixed(0);
    const tagText = `${obs.label} #${obs.id} (${confPct}%)`;

    ctx.font = 'bold 13px JetBrains Mono, monospace';
    const textWidth = ctx.measureText(tagText).width;
    const pillHeight = 20;
    const pillY = Math.max(by - pillHeight - 2, 4);

    ctx.fillStyle = 'rgba(4, 13, 26, 0.88)';
    ctx.fillRect(bx, pillY, textWidth + 12, pillHeight);
    ctx.strokeStyle = colorHex;
    ctx.lineWidth = 1;
    ctx.strokeRect(bx, pillY, textWidth + 12, pillHeight);

    ctx.fillStyle = colorHex;
    ctx.fillText(tagText, bx + 6, pillY + 14);
  }

  // Draw Trajectory Projection on 2D Camera Viewfinder
  if (trajectory && Array.isArray(trajectory.points) && trajectory.points.length >= 2) {
    const isEmergency = Boolean(trajectory.is_emergency_stop);
    const trajColor = isEmergency ? '#EF4444' : '#00F5D4';

    ctx.strokeStyle = trajColor;
    ctx.lineWidth = 2.5;
    ctx.beginPath();

    let started = false;
    for (const p of trajectory.points) {
      const px = Number(p.x) || 0;
      const py = Number(p.y) || 0;
      if (px < 0.05) continue; // skip points at or behind camera

      // Ground plane perspective projection into 640x480 camera view
      const camX = 320 + (py * 300) / (px + 0.2);
      const camY = 240 + 190 / (px + 0.3);

      if (camX < 0 || camX > w || camY < 0 || camY > h) continue;

      if (!started) {
        ctx.moveTo(camX, camY);
        started = true;
      } else {
        ctx.lineTo(camX, camY);
      }
    }
    if (started) {
      ctx.stroke();
    }

    // Waypoint dots
    ctx.fillStyle = trajColor;
    for (const p of trajectory.points) {
      const px = Number(p.x) || 0;
      const py = Number(p.y) || 0;
      if (px < 0.05) continue;
      const camX = 320 + (py * 300) / (px + 0.2);
      const camY = 240 + 190 / (px + 0.3);
      if (camX >= 0 && camX <= w && camY >= 0 && camY <= h) {
        ctx.beginPath();
        ctx.arc(camX, camY, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  // Update footer count
  const countEl = document.getElementById('cam2d-box-count');
  if (countEl) {
    const trajCount = trajectory && Array.isArray(trajectory.points) ? trajectory.points.length : 0;
    countEl.textContent = `${boxCount} BBoxes | ${trajCount} Traj Pts`;
  }
}

// ─────────────────────────────────────────────
// Scene Rendering from WorldModel
// ─────────────────────────────────────────────

function clearDynamicScene() {
  for (const obj of sceneObjects) {
    scene.remove(obj);
    obj.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        const mats = Array.isArray(child.material) ? child.material : [child.material];
        for (const m of mats) {
          if (m.map) m.map.dispose();
          m.dispose();
        }
      }
    });
  }
  sceneObjects = [];
}

function buildSceneFromWorldModel(wm) {
  latestWorldModel = wm;

  // Clear previous dynamic meshes cleanly
  clearDynamicScene();

  // Dynamic steering angle on ego wheels
  const steer = wm.ego_state?.steering_angle || 0;
  if (frontLeftWheel)  frontLeftWheel.rotation.y = -steer;
  if (frontRightWheel) frontRightWheel.rotation.y = -steer;

  // ── Lanes ──
  let laneCount = 0;

  if (wm.current_lane) {
    const laneGeo = buildLaneGeometry(wm.current_lane, true);
    scene.add(laneGeo);
    sceneObjects.push(laneGeo);
    laneCount++;
  }

  if (wm.left_neighbor_lane) {
    const laneGeo = buildLaneGeometry(wm.left_neighbor_lane, false);
    scene.add(laneGeo);
    sceneObjects.push(laneGeo);
    laneCount++;
  }

  if (wm.right_neighbor_lane) {
    const laneGeo = buildLaneGeometry(wm.right_neighbor_lane, false);
    scene.add(laneGeo);
    sceneObjects.push(laneGeo);
    laneCount++;
  }

  // ── Obstacles ──
  const obstacles = wm.obstacles || [];
  for (const obs of obstacles) {
    const bbox = buildObstacleBBox(obs);
    scene.add(bbox);
    sceneObjects.push(bbox);
  }

  // ── Traffic Signs ──
  const signs = wm.traffic_signs || [];
  for (const sign of signs) {
    const signMesh = buildTrafficSign(sign);
    scene.add(signMesh);
    sceneObjects.push(signMesh);
  }

  // ── Planned Trajectory ──
  if (wm.trajectory) {
    const trajMesh = buildTrajectoryGeometry(wm.trajectory);
    if (trajMesh) {
      scene.add(trajMesh);
      sceneObjects.push(trajMesh);
    }
  }

  // ── Update 2D Camera Viewfinder ──
  render2DCameraView(obstacles, signs, wm.trajectory);

  // ── Update HUDs and Side Panels ──
  updateHUD(wm, laneCount);
  updateSidePanels(wm);
  updateDashboard(wm);

  // Hide placeholder if visible
  const ph = document.getElementById('viewport-placeholder');
  if (ph && ph.style.display !== 'none') {
    ph.style.display = 'none';
  }
}

// ─────────────────────────────────────────────
// HUD Updates
// ─────────────────────────────────────────────

function updateHUD(wm, laneCount) {
  const ego = wm.ego_state || {};

  // Header badges
  const frameSeqEl = document.getElementById('frame-seq-display');
  if (frameSeqEl) frameSeqEl.textContent = wm.frame_seq ?? '—';

  const tsEl = document.getElementById('ts-display');
  if (tsEl) {
    tsEl.textContent = wm.timestamp_us
      ? (wm.timestamp_us / 1e6).toFixed(3) + 's'
      : '—';
  }

  // Viewport top-left ego dynamics
  const hudVel = document.getElementById('hud-velocity');
  if (hudVel) hudVel.textContent = (ego.velocity ?? 0).toFixed(2);

  const hudYaw = document.getElementById('hud-yaw');
  if (hudYaw) hudYaw.textContent = (ego.yaw ?? 0).toFixed(3);

  const hudSteer = document.getElementById('hud-steer');
  if (hudSteer) hudSteer.textContent = (ego.steering_angle ?? 0).toFixed(3);

  // Viewport top-right perception track counts
  const hudObs = document.getElementById('hud-objects');
  if (hudObs) hudObs.textContent = (wm.obstacles || []).length;

  const hudSigns = document.getElementById('hud-signs');
  if (hudSigns) hudSigns.textContent = (wm.traffic_signs || []).length;

  const hudLanes = document.getElementById('hud-lanes');
  if (hudLanes) hudLanes.textContent = laneCount;

  // Viewport top-right trajectory waypoint count
  const hudTraj = document.getElementById('hud-traj');
  if (hudTraj) {
    if (wm.trajectory && Array.isArray(wm.trajectory.points) && wm.trajectory.points.length > 0) {
      const count = wm.trajectory.points.length;
      if (wm.trajectory.is_emergency_stop) {
        hudTraj.textContent = `E-STOP (${count})`;
        hudTraj.style.color = '#EF4444';
      } else {
        hudTraj.textContent = `${count} pts`;
        hudTraj.style.color = '#00F5D4';
      }
    } else {
      hudTraj.textContent = '—';
      hudTraj.style.color = '';
    }
  }
}

// ─────────────────────────────────────────────
// Left Panel Stream Dashboard Updates
// ─────────────────────────────────────────────

function updateDashboard(wm) {
  const ego = wm.ego_state || {};
  const vel = ego.velocity || 0;
  const steer = ego.steering_angle || 0;
  const yaw = ego.yaw || 0;

  // Speed
  const dashVel = document.getElementById('dash-vel');
  if (dashVel) dashVel.textContent = vel.toFixed(2);

  const dashVelKmh = document.getElementById('dash-vel-kmh');
  if (dashVelKmh) dashVelKmh.textContent = (vel * 3.6).toFixed(1) + ' km/h';

  // Steering
  const dashSteer = document.getElementById('dash-steer');
  if (dashSteer) dashSteer.textContent = steer.toFixed(3);

  const dashSteerDeg = document.getElementById('dash-steer-deg');
  if (dashSteerDeg) dashSteerDeg.textContent = (steer * (180 / Math.PI)).toFixed(1) + '°';

  // Yaw
  const dashYaw = document.getElementById('dash-yaw');
  if (dashYaw) dashYaw.textContent = yaw.toFixed(3);

  const dashYawDeg = document.getElementById('dash-yaw-deg');
  if (dashYawDeg) dashYawDeg.textContent = (yaw * (180 / Math.PI)).toFixed(1) + '°';

  // Lane polynomial equations display: y = c0 + c1*x + c2*x^2
  if (wm.current_lane) {
    const lane = wm.current_lane;

    const formatPoly = (b) => {
      if (!b) return '—';
      const c0 = (b.c0 ?? 0).toFixed(3);
      const c1 = (b.c1 ?? 0) >= 0 ? `+ ${(b.c1 ?? 0).toFixed(3)}x` : `- ${Math.abs(b.c1).toFixed(3)}x`;
      const c2 = (b.c2 ?? 0) >= 0 ? `+ ${(b.c2 ?? 0).toFixed(4)}x²` : `- ${Math.abs(b.c2).toFixed(4)}x²`;
      return `y = ${c0} ${c1} ${c2}`;
    };

    const leftEq = document.getElementById('poly-left-eq');
    if (leftEq) leftEq.textContent = formatPoly(lane.left_boundary);

    const centerEq = document.getElementById('poly-center-eq');
    if (centerEq) centerEq.textContent = formatPoly(lane.centerline);

    const rightEq = document.getElementById('poly-right-eq');
    if (rightEq) rightEq.textContent = formatPoly(lane.right_boundary);

    const laneWidthEl = document.getElementById('poly-lane-width');
    if (laneWidthEl) laneWidthEl.textContent = (lane.lane_width?.toFixed(2) ?? '0.50') + 'm';
  }

  // Planned Trajectory Card
  const trajCard = document.getElementById('dash-traj-card');
  const trajPill = document.getElementById('traj-status-pill');
  const trajTargetVel = document.getElementById('traj-target-vel');
  const trajTargetVelKmh = document.getElementById('traj-target-vel-kmh');
  const trajLookahead = document.getElementById('traj-lookahead');
  const trajPointsCount = document.getElementById('traj-points-count');
  const trajEndYaw = document.getElementById('traj-end-yaw');
  const trajEndYawDeg = document.getElementById('traj-end-yaw-deg');
  const trajEstopStatus = document.getElementById('traj-estop-status');
  const trajModeSub = document.getElementById('traj-mode-sub');

  if (wm.trajectory && Array.isArray(wm.trajectory.points) && wm.trajectory.points.length > 0) {
    const pts = wm.trajectory.points;
    const isEmergency = Boolean(wm.trajectory.is_emergency_stop);
    const lastPt = pts[pts.length - 1];
    const lookaheadDist = Math.sqrt((lastPt.x || 0) ** 2 + (lastPt.y || 0) ** 2);
    const targetVel = lastPt.target_velocity ?? 0;
    const endYaw = lastPt.yaw ?? 0;

    if (trajPill) {
      trajPill.textContent = isEmergency ? 'E-STOP' : 'Active';
      trajPill.className = `stream-pill ${isEmergency ? 'emergency' : 'connected'}`;
      if (isEmergency) {
        trajPill.style.background = 'rgba(239, 68, 68, 0.2)';
        trajPill.style.borderColor = 'rgba(239, 68, 68, 0.5)';
        trajPill.style.color = '#EF4444';
      } else {
        trajPill.style.background = 'rgba(0, 245, 212, 0.15)';
        trajPill.style.borderColor = 'rgba(0, 245, 212, 0.4)';
        trajPill.style.color = '#00F5D4';
      }
    }

    if (trajTargetVel) trajTargetVel.textContent = targetVel.toFixed(2);
    if (trajTargetVelKmh) trajTargetVelKmh.textContent = (targetVel * 3.6).toFixed(1) + ' km/h';
    if (trajLookahead) trajLookahead.textContent = lookaheadDist.toFixed(2);
    if (trajPointsCount) trajPointsCount.textContent = `${pts.length} waypoints`;
    if (trajEndYaw) trajEndYaw.textContent = endYaw.toFixed(3);
    if (trajEndYawDeg) trajEndYawDeg.textContent = (endYaw * (180 / Math.PI)).toFixed(1) + '°';

    if (trajEstopStatus) {
      trajEstopStatus.textContent = isEmergency ? 'EMERGENCY' : 'NORMAL';
      trajEstopStatus.style.color = isEmergency ? '#EF4444' : '#00F5D4';
    }
    if (trajModeSub) {
      trajModeSub.textContent = isEmergency ? 'E-STOP TRIGGERED' : 'Path Tracking Active';
      trajModeSub.style.color = isEmergency ? '#F87171' : 'var(--text-muted)';
    }

    if (trajCard) {
      if (isEmergency) trajCard.classList.add('emergency');
      else trajCard.classList.remove('emergency');
    }
  } else {
    if (trajPill) {
      trajPill.textContent = 'No Plan';
      trajPill.className = 'stream-pill';
      trajPill.style.background = '';
      trajPill.style.borderColor = '';
      trajPill.style.color = '';
    }
    if (trajTargetVel) trajTargetVel.textContent = '0.00';
    if (trajTargetVelKmh) trajTargetVelKmh.textContent = '0.0 km/h';
    if (trajLookahead) trajLookahead.textContent = '0.00';
    if (trajPointsCount) trajPointsCount.textContent = '0 waypoints';
    if (trajEndYaw) trajEndYaw.textContent = '0.000';
    if (trajEndYawDeg) trajEndYawDeg.textContent = '0.0°';
    if (trajEstopStatus) {
      trajEstopStatus.textContent = 'STANDBY';
      trajEstopStatus.style.color = '';
    }
    if (trajModeSub) {
      trajModeSub.textContent = 'No Active Path';
      trajModeSub.style.color = '';
    }
    if (trajCard) trajCard.classList.remove('emergency');
  }

  // Total frames in card
  const totalFramesEl = document.getElementById('card-total-frames');
  if (totalFramesEl && streamClient) {
    totalFramesEl.textContent = streamClient.totalFrames.toLocaleString();
  }
}

// ─────────────────────────────────────────────
// Side Panel (Objects, Signs, Lanes, Trajectory) Updates
// ─────────────────────────────────────────────

function confClass(c) {
  if (c >= 0.8) return 'high';
  if (c >= 0.5) return 'medium';
  return 'low';
}

function updateSidePanels(wm) {
  updateObjectList(wm.obstacles || []);
  updateSignList(wm.traffic_signs || []);
  updateLaneInfo(wm);
  updateTrajectoryList(wm.trajectory);
}

function updateTrajectoryList(trajectory) {
  const el = document.getElementById('trajectory-list');
  const countBadge = document.getElementById('count-traj-points');

  const pts = trajectory && Array.isArray(trajectory.points) ? trajectory.points : [];
  if (countBadge) countBadge.textContent = `${pts.length} pts`;

  if (!el) return;
  if (!pts.length) {
    el.innerHTML = '<div class="empty-state">No trajectory planned.</div>';
    return;
  }

  const isEmergency = Boolean(trajectory.is_emergency_stop);
  const lastPt = pts[pts.length - 1];
  const lookaheadDist = Math.sqrt((lastPt.x || 0) ** 2 + (lastPt.y || 0) ** 2);
  const targetVel = lastPt.target_velocity ?? 0;

  const pointsRowsHtml = pts.map((p, idx) => {
    const px = (Number(p.x) || 0).toFixed(2);
    const py = (Number(p.y) || 0).toFixed(2);
    const pyaw = ((Number(p.yaw) || 0) * (180 / Math.PI)).toFixed(1);
    const pvel = (Number(p.target_velocity) || 0).toFixed(2);
    return `
      <div class="traj-point-row">
        <span class="traj-point-idx">#${idx}</span>
        <span class="traj-point-coords">(${px}, ${py})</span>
        <span class="traj-point-yaw font-mono" style="color:var(--text-muted);font-size:9px;">${pyaw}°</span>
        <span class="traj-point-vel">${pvel} m/s</span>
      </div>
    `;
  }).join('');

  el.innerHTML = `
    <div class="traj-card ${isEmergency ? 'emergency' : ''}">
      <div class="traj-card-header">
        <span class="traj-card-title">${isEmergency ? '⚠ EMERGENCY STOP' : 'PLANNED PATH'}</span>
        <span class="traj-badge ${isEmergency ? 'emergency' : ''}">${pts.length} WPTS</span>
      </div>
      <div class="traj-badges">
        <span class="traj-badge">Lookahead: ${lookaheadDist.toFixed(2)}m</span>
        <span class="traj-badge">Target: ${targetVel.toFixed(2)}m/s</span>
      </div>
      <div class="traj-points-list">
        ${pointsRowsHtml}
      </div>
    </div>
  `;
}

function updateObjectList(obstacles) {
  const el = document.getElementById('object-list');
  const countBadge = document.getElementById('count-obstacles');
  if (countBadge) countBadge.textContent = obstacles.length;

  if (!el) return;
  if (!obstacles.length) {
    el.innerHTML = '<div class="empty-state">No obstacles detected.</div>';
    return;
  }

  el.innerHTML = obstacles.map((obj) => {
    const rawLabel = (obj.label || 'obstacle').toLowerCase();
    const type = rawLabel.includes('vehicle') ? 'vehicle'
      : rawLabel.includes('pedestrian') ? 'pedestrian'
      : rawLabel.includes('bottle') ? 'bottle'
      : 'obstacle';

    const conf = obj.confidence ?? 0;
    const confPct = (conf * 100).toFixed(0);

    const dist = obj.position
      ? Math.sqrt((obj.position.x || 0) ** 2 + (obj.position.y || 0) ** 2).toFixed(2)
      : '—';

    const speed = obj.velocity
      ? Math.sqrt((obj.velocity.x || 0) ** 2 + (obj.velocity.y || 0) ** 2 + (obj.velocity.z || 0) ** 2).toFixed(2)
      : '—';

    const bbox = obj.bounding_box;
    const bboxStr = bbox
      ? `[${Math.round(bbox.x)}, ${Math.round(bbox.y)}, ${Math.round(bbox.width)}×${Math.round(bbox.height)}px]`
      : 'None';

    return `
    <div class="object-card type-${type}" id="obj-card-${obj.id}">
      <div class="object-card-header">
        <div class="object-label">
          <div class="label-dot ${type}"></div>
          <span class="label-text">${obj.label}</span>
          <span class="label-id">#${obj.id}</span>
        </div>
      </div>
      <div class="confidence-bar-wrap">
        <div class="confidence-bar">
          <div class="confidence-fill ${confClass(conf)}" style="width:${confPct}%"></div>
        </div>
        <span class="confidence-pct">${confPct}%</span>
      </div>
      <div class="object-metrics">
        <div class="metric"><span class="metric-label">Dist</span><span class="metric-value">${dist}m</span></div>
        <div class="metric"><span class="metric-label">Speed</span><span class="metric-value">${speed}m/s</span></div>
        <div class="metric"><span class="metric-label">L×W×H</span><span class="metric-value">${obj.length?.toFixed(2)}×${obj.width?.toFixed(2)}×${obj.height?.toFixed(2)}m</span></div>
        <div class="metric"><span class="metric-label">X,Y,Z</span><span class="metric-value">${obj.position?.x?.toFixed(2)}, ${obj.position?.y?.toFixed(2)}, ${obj.position?.z?.toFixed(2)}</span></div>
        <div class="metric bbox-metric"><span class="metric-label">2D BBox</span><span class="metric-value">${bboxStr}</span></div>
      </div>
    </div>`;
  }).join('');
}

function updateSignList(signs) {
  const el = document.getElementById('sign-list');
  const countBadge = document.getElementById('count-signs');
  if (countBadge) countBadge.textContent = signs.length;

  if (!el) return;
  if (!signs.length) {
    el.innerHTML = '<div class="empty-state">No signs detected.</div>';
    return;
  }

  el.innerHTML = signs.map((sign, i) => {
    const dist = sign.distance_along_lane != null
      ? sign.distance_along_lane.toFixed(2) + 'm'
      : (sign.position ? Math.sqrt((sign.position.x || 0) ** 2 + (sign.position.y || 0) ** 2).toFixed(2) + 'm' : '—');

    const conf = sign.confidence ?? 0;
    const confPct = (conf * 100).toFixed(0);
    const signName = (sign.type || 'STOP_SIGN').replace(/_/g, ' ');

    return `
    <div class="sign-card" id="sign-card-${i}">
      <div class="sign-header">
        <div class="sign-type">
          <div class="sign-icon">✕</div>
          ${signName}
        </div>
      </div>
      <div class="confidence-bar-wrap" style="margin-bottom:6px">
        <div class="confidence-bar"><div class="confidence-fill ${confClass(conf)}" style="width:${confPct}%"></div></div>
        <span class="confidence-pct">${confPct}%</span>
      </div>
      <div class="object-metrics">
        <div class="metric"><span class="metric-label">Dist</span><span class="metric-value">${dist}</span></div>
        <div class="metric"><span class="metric-label">X,Y,Z</span><span class="metric-value">${sign.position?.x?.toFixed(2)}, ${sign.position?.y?.toFixed(2)}, ${sign.position?.z?.toFixed(2)}</span></div>
      </div>
    </div>`;
  }).join('');
}

function updateLaneInfo(wm) {
  const el = document.getElementById('lane-info');
  if (!el) return;

  const renderLane = (lane, name) => {
    if (!lane) return '';
    return `
    <div class="lane-card">
      <div class="lane-card-title">${name} (ID: ${lane.lane_id})</div>
      <div class="lane-badges">
        <span class="lane-badge info">Width: ${lane.lane_width?.toFixed(2)}m</span>
        ${lane.can_change_left  ? '<span class="lane-badge yes">← Change Left</span>' : '<span class="lane-badge no">← No Change</span>'}
        ${lane.can_change_right ? '<span class="lane-badge yes">→ Change Right</span>' : '<span class="lane-badge no">→ No Change</span>'}
      </div>
    </div>`;
  };

  const html = [
    renderLane(wm.current_lane,        'Current Ego Lane'),
    renderLane(wm.left_neighbor_lane,  'Left Adjacent Lane'),
    renderLane(wm.right_neighbor_lane, 'Right Adjacent Lane'),
  ].filter(Boolean).join('');

  el.innerHTML = html || '<div class="empty-state">No lane data.</div>';
}

// ─────────────────────────────────────────────
// Preset Samples Library
// ─────────────────────────────────────────────

const SAMPLES = {
  adas_backend: JSON.stringify({
    "timestamp_us": 123456789,
    "frame_seq": 42,
    "ego_state": {
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0,
      "velocity": 0.0,
      "steering_angle": 0.0
    },
    "current_lane": {
      "lane_id": 1,
      "lane_width": 0.50,
      "can_change_left": true,
      "can_change_right": true,
      "left_boundary": { "c0": -0.25, "c1": 0.0, "c2": 0.0 },
      "right_boundary": { "c0": 0.25, "c1": 0.0, "c2": 0.0 },
      "centerline": { "c0": 0.0, "c1": 0.0, "c2": 0.0 }
    },
    "left_neighbor_lane": null,
    "right_neighbor_lane": null,
    "obstacles": [
      {
        "id": 1,
        "label": "bottle",
        "confidence": 0.88,
        "bounding_box": { "x": 120.0, "y": 140.0, "width": 80.0, "height": 160.0 },
        "position": { "x": 1.50, "y": -0.15, "z": 0.0 },
        "velocity": { "x": 0.0, "y": 0.0, "z": 0.0 },
        "length": 0.10,
        "width": 0.10,
        "height": 0.25
      }
    ],
    "traffic_signs": [],
    "trajectory": {
      "timestamp_us": 123456795,
      "is_emergency_stop": false,
      "points": [
        { "x": 0.00, "y": 0.00, "yaw": 0.00, "target_velocity": 1.00 },
        { "x": 0.50, "y": 0.01, "yaw": 0.02, "target_velocity": 1.00 },
        { "x": 1.00, "y": 0.03, "yaw": 0.04, "target_velocity": 1.00 },
        { "x": 1.50, "y": 0.06, "yaw": 0.05, "target_velocity": 1.00 },
        { "x": 2.00, "y": 0.10, "yaw": 0.07, "target_velocity": 1.00 }
      ]
    }
  }, null, 2),

  highway: JSON.stringify({
    "timestamp_us": 1726189200123456,
    "frame_seq": 1042,
    "ego_state": { "x": 3.452, "y": 1.108, "yaw": 0.045, "velocity": 0.85, "steering_angle": -0.012 },
    "current_lane": {
      "lane_id": 0, "lane_width": 0.50, "can_change_left": false, "can_change_right": true,
      "left_boundary":  { "c0": -0.25, "c1": 0.005, "c2": -0.001 },
      "right_boundary": { "c0":  0.25, "c1": 0.005, "c2": -0.001 },
      "centerline":     { "c0":  0.00, "c1": 0.005, "c2": -0.001 }
    },
    "left_neighbor_lane": null,
    "right_neighbor_lane": {
      "lane_id": 1, "lane_width": 0.50, "can_change_left": true, "can_change_right": false,
      "left_boundary":  { "c0": 0.25, "c1": 0.005, "c2": -0.001 },
      "right_boundary": { "c0": 0.75, "c1": 0.005, "c2": -0.001 },
      "centerline":     { "c0": 0.50, "c1": 0.005, "c2": -0.001 }
    },
    "obstacles": [
      {
        "id": 101, "label": "vehicle", "confidence": 0.94,
        "bounding_box": { "x": 300.0, "y": 200.0, "width": 110.0, "height": 90.0 },
        "position": { "x": 1.42, "y": 0.02, "z": 0.06 },
        "velocity": { "x": 0.80, "y": 0.0, "z": 0.0 },
        "length": 0.32, "width": 0.18, "height": 0.14
      }
    ],
    "traffic_signs": [],
    "trajectory": {
      "timestamp_us": 1726189200123500,
      "is_emergency_stop": false,
      "points": [
        { "x": 0.00, "y": 0.00, "yaw": 0.045, "target_velocity": 0.85 },
        { "x": 0.75, "y": 0.01, "yaw": 0.045, "target_velocity": 0.85 },
        { "x": 1.50, "y": 0.02, "yaw": 0.045, "target_velocity": 0.85 },
        { "x": 2.25, "y": 0.03, "yaw": 0.045, "target_velocity": 0.85 },
        { "x": 3.00, "y": 0.04, "yaw": 0.045, "target_velocity": 0.85 }
      ]
    }
  }, null, 2),

  stop_sign: JSON.stringify({
    "timestamp_us": 1726189200200000,
    "frame_seq": 1100,
    "ego_state": { "x": 5.1, "y": 0.3, "yaw": 0.0, "velocity": 0.35, "steering_angle": 0.01 },
    "current_lane": {
      "lane_id": 0, "lane_width": 0.50, "can_change_left": false, "can_change_right": false,
      "left_boundary":  { "c0": -0.25, "c1": 0.0, "c2": 0.0 },
      "right_boundary": { "c0":  0.25, "c1": 0.0, "c2": 0.0 },
      "centerline":     { "c0":  0.00, "c1": 0.0, "c2": 0.0 }
    },
    "left_neighbor_lane": null,
    "right_neighbor_lane": null,
    "obstacles": [
      {
        "id": 42, "label": "vehicle", "confidence": 0.89,
        "bounding_box": { "x": 310.0, "y": 210.0, "width": 95.0, "height": 85.0 },
        "position": { "x": 2.20, "y": -0.01, "z": 0.07 },
        "velocity": { "x": 0.0, "y": 0.0, "z": 0.0 },
        "length": 0.30, "width": 0.17, "height": 0.13
      }
    ],
    "traffic_signs": [
      {
        "type": "STOP_SIGN", "confidence": 0.97,
        "position": { "x": 2.50, "y": 0.38, "z": 0.24 }
      }
    ],
    "trajectory": {
      "timestamp_us": 1726189200200050,
      "is_emergency_stop": true,
      "points": [
        { "x": 0.00, "y": 0.00, "yaw": 0.00, "target_velocity": 0.25 },
        { "x": 0.50, "y": 0.00, "yaw": 0.00, "target_velocity": 0.10 },
        { "x": 1.00, "y": 0.00, "yaw": 0.00, "target_velocity": 0.00 }
      ]
    }
  }, null, 2),

  multi_object: JSON.stringify({
    "timestamp_us": 1726189200500000,
    "frame_seq": 1255,
    "ego_state": { "x": 12.5, "y": 2.1, "yaw": -0.08, "velocity": 1.10, "steering_angle": -0.05 },
    "current_lane": {
      "lane_id": 0, "lane_width": 0.50, "can_change_left": true, "can_change_right": true,
      "left_boundary":  { "c0": -0.25, "c1": -0.008, "c2": 0.001 },
      "right_boundary": { "c0":  0.25, "c1": -0.008, "c2": 0.001 },
      "centerline":     { "c0":  0.00, "c1": -0.008, "c2": 0.001 }
    },
    "left_neighbor_lane": {
      "lane_id": -1, "lane_width": 0.50, "can_change_left": false, "can_change_right": true,
      "left_boundary":  { "c0": -0.75, "c1": -0.008, "c2": 0.001 },
      "right_boundary": { "c0": -0.25, "c1": -0.008, "c2": 0.001 },
      "centerline":     { "c0": -0.50, "c1": -0.008, "c2": 0.001 }
    },
    "right_neighbor_lane": {
      "lane_id": 1, "lane_width": 0.50, "can_change_left": true, "can_change_right": false,
      "left_boundary":  { "c0": 0.25, "c1": -0.008, "c2": 0.001 },
      "right_boundary": { "c0": 0.75, "c1": -0.008, "c2": 0.001 },
      "centerline":     { "c0": 0.50, "c1": -0.008, "c2": 0.001 }
    },
    "obstacles": [
      {
        "id": 10, "label": "vehicle", "confidence": 0.97,
        "bounding_box": { "x": 280.0, "y": 230.0, "width": 120.0, "height": 95.0 },
        "position": { "x": 0.90, "y": -0.01, "z": 0.07 },
        "velocity": { "x": 1.05, "y": 0.0, "z": 0.0 },
        "length": 0.32, "width": 0.18, "height": 0.14
      },
      {
        "id": 11, "label": "vehicle", "confidence": 0.88,
        "bounding_box": { "x": 420.0, "y": 210.0, "width": 80.0, "height": 65.0 },
        "position": { "x": 1.80, "y": 0.52, "z": 0.07 },
        "velocity": { "x": 0.60, "y": -0.02, "z": 0.0 },
        "length": 0.30, "width": 0.17, "height": 0.13
      },
      {
        "id": 22, "label": "pedestrian", "confidence": 0.75,
        "bounding_box": { "x": 160.0, "y": 190.0, "width": 45.0, "height": 110.0 },
        "position": { "x": 1.20, "y": -0.50, "z": 0.09 },
        "velocity": { "x": 0.10, "y": 0.05, "z": 0.0 },
        "length": 0.12, "width": 0.12, "height": 0.20
      },
      {
        "id": 33, "label": "bottle", "confidence": 0.62,
        "bounding_box": { "x": 305.0, "y": 270.0, "width": 30.0, "height": 60.0 },
        "position": { "x": 3.00, "y": 0.00, "z": 0.04 },
        "velocity": { "x": 0.00, "y": 0.00, "z": 0.0 },
        "length": 0.10, "width": 0.10, "height": 0.08
      }
    ],
    "traffic_signs": [
      {
        "type": "STOP_SIGN", "confidence": 0.91,
        "position": { "x": 3.80, "y": 0.35, "z": 0.22 }
      }
    ]
  }, null, 2),

  curve: JSON.stringify({
    "timestamp_us": 1726189200800000,
    "frame_seq": 1400,
    "ego_state": { "x": 20.0, "y": 5.5, "yaw": 0.30, "velocity": 0.70, "steering_angle": 0.18 },
    "current_lane": {
      "lane_id": 0, "lane_width": 0.50, "can_change_left": false, "can_change_right": false,
      "left_boundary":  { "c0": -0.25, "c1":  0.04, "c2": -0.025 },
      "right_boundary": { "c0":  0.25, "c1":  0.04, "c2": -0.025 },
      "centerline":     { "c0":  0.00, "c1":  0.04, "c2": -0.025 }
    },
    "left_neighbor_lane": null,
    "right_neighbor_lane": null,
    "obstacles": [
      {
        "id": 55, "label": "vehicle", "confidence": 0.83,
        "bounding_box": { "x": 340.0, "y": 210.0, "width": 105.0, "height": 90.0 },
        "position": { "x": 1.50, "y": 0.08, "z": 0.07 },
        "velocity": { "x": 0.65, "y": 0.02, "z": 0.0 },
        "length": 0.30, "width": 0.17, "height": 0.13
      }
    ],
    "traffic_signs": []
  }, null, 2),
};

// ─────────────────────────────────────────────
// Manual JSON Parsing & Validation
// ─────────────────────────────────────────────

function setJsonStatus(msg, type = 'idle') {
  const el = document.getElementById('json-status');
  if (!el) return;
  el.textContent = msg;
  el.className = `json-status status-${type}`;
}

function parseAndRenderManualJson() {
  const raw = document.getElementById('json-input')?.value.trim();
  if (!raw) {
    setJsonStatus('No JSON input provided', 'warning');
    return;
  }

  let wm;
  try {
    wm = JSON.parse(raw);
  } catch (e) {
    setJsonStatus('JSON parse error: ' + e.message, 'error');
    return;
  }

  if (!wm.ego_state) {
    setJsonStatus('Missing ego_state field', 'error');
    return;
  }

  try {
    buildSceneFromWorldModel(wm);
    setJsonStatus('Rendered scene OK', 'ok');
  } catch (e) {
    setJsonStatus('Render error: ' + e.message, 'error');
    console.error('Render error:', e);
  }
}

// ─────────────────────────────────────────────
// Stream Status UI Synchronization
// ─────────────────────────────────────────────

function updateConnectionStatusUI(status, details = {}) {
  const badge = document.getElementById('ws-status-badge');
  const text = document.getElementById('ws-status-text');
  const toggleBtn = document.getElementById('btn-ws-toggle');
  const toggleLabel = document.getElementById('ws-toggle-label');
  const dashPill = document.getElementById('dashboard-status-pill');
  const retryState = document.getElementById('card-retry-state');
  const hudTxt = document.getElementById('hud-stream-txt');
  const hudIndicator = document.getElementById('hud-stream-indicator');
  const placeholderDesc = document.getElementById('placeholder-status-desc');
  const pauseBtn = document.getElementById('btn-stream-pause');

  const url = details.url || streamClient?.url || 'ws://localhost:9002';
  const cardEndpoint = document.getElementById('card-endpoint-val');
  if (cardEndpoint) cardEndpoint.textContent = url;

  // Clear previous status classes
  if (badge) {
    badge.classList.remove('status-connected', 'status-connecting', 'status-reconnecting', 'status-disconnected');
  }

  switch (status) {
    case 'connected':
      if (badge) badge.classList.add('status-connected');
      if (text) text.textContent = 'CONNECTED';
      if (toggleBtn) {
        toggleBtn.classList.add('disconnect');
        toggleLabel.textContent = 'Disconnect';
      }
      if (dashPill) {
        dashPill.textContent = 'Live Online';
        dashPill.style.background = 'rgba(16, 185, 129, 0.15)';
        dashPill.style.color = '#10B981';
        dashPill.style.borderColor = 'rgba(16, 185, 129, 0.3)';
      }
      if (retryState) retryState.textContent = 'Connected (Healthy)';
      if (hudTxt) hudTxt.textContent = 'LIVE STREAM';
      if (hudIndicator) {
        hudIndicator.style.borderColor = 'rgba(16, 185, 129, 0.5)';
        const dot = hudIndicator.querySelector('.hud-dot');
        if (dot) dot.style.background = '#10B981';
      }
      if (pauseBtn) pauseBtn.classList.remove('hidden');
      break;

    case 'connecting':
      if (badge) badge.classList.add('status-connecting');
      if (text) text.textContent = 'CONNECTING…';
      if (toggleBtn) {
        toggleBtn.classList.remove('disconnect');
        toggleLabel.textContent = 'Connecting';
      }
      if (dashPill) {
        dashPill.textContent = 'Connecting';
        dashPill.style.background = 'rgba(59, 130, 246, 0.15)';
        dashPill.style.color = '#60A5FA';
        dashPill.style.borderColor = 'rgba(59, 130, 246, 0.3)';
      }
      if (retryState) retryState.textContent = 'Connecting…';
      if (hudTxt) hudTxt.textContent = 'CONNECTING';
      break;

    case 'reconnecting':
      if (badge) badge.classList.add('status-reconnecting');
      const delay = details.delaySeconds ?? 2;
      const count = details.retryCount ?? 1;
      if (text) text.textContent = `RETRY IN ${delay}s (#${count})`;
      if (toggleBtn) {
        toggleBtn.classList.remove('disconnect');
        toggleLabel.textContent = 'Connect';
      }
      if (dashPill) {
        dashPill.textContent = 'Reconnecting';
        dashPill.style.background = 'rgba(245, 158, 11, 0.15)';
        dashPill.style.color = '#F59E0B';
        dashPill.style.borderColor = 'rgba(245, 158, 11, 0.3)';
      }
      if (retryState) retryState.textContent = `Auto-retry in ${delay}s (Attempt #${count})`;
      if (hudTxt) hudTxt.textContent = `RETRYING (#${count})`;
      if (hudIndicator) {
        const dot = hudIndicator.querySelector('.hud-dot');
        if (dot) dot.style.background = '#F59E0B';
      }
      if (placeholderDesc) {
        placeholderDesc.innerHTML = `Backend offline at <strong>${url}</strong>.<br>Auto-retrying in ${delay}s… (Attempt #${count})`;
      }
      break;

    case 'disconnected':
    default:
      if (badge) badge.classList.add('status-disconnected');
      if (text) text.textContent = 'DISCONNECTED';
      if (toggleBtn) {
        toggleBtn.classList.remove('disconnect');
        toggleLabel.textContent = 'Connect';
      }
      if (dashPill) {
        dashPill.textContent = 'Offline';
        dashPill.style.background = 'rgba(239, 68, 68, 0.12)';
        dashPill.style.color = '#F87171';
        dashPill.style.borderColor = 'rgba(239, 68, 68, 0.25)';
      }
      if (retryState) retryState.textContent = details.manual ? 'Manual disconnect' : 'Disconnected';
      if (hudTxt) hudTxt.textContent = 'STANDBY';
      if (hudIndicator) {
        const dot = hudIndicator.querySelector('.hud-dot');
        if (dot) dot.style.background = '#EF4444';
      }
      break;
  }
}

// ─────────────────────────────────────────────
// Event Listeners & UI Wiring
// ─────────────────────────────────────────────

function setupEventListeners() {
  const urlInput = document.getElementById('ws-url-input');
  const toggleBtn = document.getElementById('btn-ws-toggle');
  const pauseBtn = document.getElementById('btn-stream-pause');
  const pauseLabel = document.getElementById('stream-pause-label');

  // Load persisted URL from localStorage
  const savedUrl = localStorage.getItem('adas_ws_url');
  if (savedUrl && urlInput) {
    urlInput.value = savedUrl;
  }

  // Connect / Disconnect button
  if (toggleBtn) {
    toggleBtn.addEventListener('click', () => {
      const url = urlInput.value.trim() || 'ws://localhost:9002';
      localStorage.setItem('adas_ws_url', url);

      if (streamClient && streamClient.ws && streamClient.ws.readyState === WebSocket.OPEN) {
        streamClient.disconnect();
      } else {
        streamClient.connect(url);
      }
    });
  }

  // Enter key in URL input to connect
  if (urlInput) {
    urlInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const url = urlInput.value.trim() || 'ws://localhost:9002';
        localStorage.setItem('adas_ws_url', url);
        streamClient.connect(url);
      }
    });
  }

  // Pause / Resume Stream Rendering button
  if (pauseBtn) {
    pauseBtn.addEventListener('click', () => {
      if (!streamClient) return;
      const willBePaused = !streamClient.isPaused;
      streamClient.setPaused(willBePaused);

      if (willBePaused) {
        pauseBtn.classList.add('paused');
        pauseLabel.textContent = 'Paused';
        const pauseIcon = document.getElementById('pause-icon');
        if (pauseIcon) pauseIcon.textContent = '▶';
        const hudTxt = document.getElementById('hud-stream-txt');
        if (hudTxt) hudTxt.textContent = 'PAUSED';
      } else {
        pauseBtn.classList.remove('paused');
        pauseLabel.textContent = 'Live';
        const pauseIcon = document.getElementById('pause-icon');
        if (pauseIcon) pauseIcon.textContent = '⏸';
        const hudTxt = document.getElementById('hud-stream-txt');
        if (hudTxt) hudTxt.textContent = 'LIVE STREAM';
      }
    });
  }

  // Tabs: Live Stream vs Manual JSON
  const tabLiveBtn = document.getElementById('tab-live-btn');
  const tabManualBtn = document.getElementById('tab-manual-btn');
  const contentLive = document.getElementById('tab-content-live');
  const contentManual = document.getElementById('tab-content-manual');

  if (tabLiveBtn && tabManualBtn) {
    tabLiveBtn.addEventListener('click', () => {
      tabLiveBtn.classList.add('active');
      tabManualBtn.classList.remove('active');
      contentLive.classList.remove('hidden');
      contentManual.classList.add('hidden');
    });

    tabManualBtn.addEventListener('click', () => {
      tabManualBtn.classList.add('active');
      tabLiveBtn.classList.remove('active');
      contentManual.classList.remove('hidden');
      contentLive.classList.add('hidden');
    });
  }

  // Quick Action: Snapshot current live frame to Manual tab
  const btnSnapshot = document.getElementById('btn-snapshot-json');
  if (btnSnapshot) {
    btnSnapshot.addEventListener('click', () => {
      if (!latestWorldModel) {
        alert('No WorldModel frame received yet to snapshot.');
        return;
      }
      const jsonStr = JSON.stringify(latestWorldModel, null, 2);
      const jsonInput = document.getElementById('json-input');
      if (jsonInput) jsonInput.value = jsonStr;

      // Switch to manual tab
      tabManualBtn.click();
      setJsonStatus('Live frame snapshot loaded — ready to inspect or edit', 'ok');
    });
  }

  // Quick Action: Copy current frame JSON to clipboard
  const btnCopy = document.getElementById('btn-copy-live-json');
  if (btnCopy) {
    btnCopy.addEventListener('click', async () => {
      if (!latestWorldModel) {
        alert('No WorldModel frame received yet to copy.');
        return;
      }
      try {
        await navigator.clipboard.writeText(JSON.stringify(latestWorldModel, null, 2));
        const originalText = btnCopy.textContent;
        btnCopy.textContent = '✓ Copied to Clipboard!';
        setTimeout(() => { btnCopy.innerHTML = originalText; }, 1800);
      } catch (err) {
        console.warn('Clipboard write failed:', err);
      }
    });
  }

  // Quick Action & HUD tool: Toggle 2D Camera Viewfinder
  const camOverlay = document.getElementById('cam2d-overlay');
  const btnCamToggle1 = document.getElementById('btn-toggle-cam-panel');
  const btnCamToggle2 = document.getElementById('btn-hud-cam2d');
  const btnCamClose = document.getElementById('btn-cam2d-close');

  const toggleCamView = () => {
    if (!camOverlay) return;
    camOverlay.classList.toggle('hidden');
    const isVisible = !camOverlay.classList.contains('hidden');
    if (btnCamToggle2) btnCamToggle2.classList.toggle('active', isVisible);
  };

  if (btnCamToggle1) btnCamToggle1.addEventListener('click', toggleCamView);
  if (btnCamToggle2) btnCamToggle2.addEventListener('click', toggleCamView);
  if (btnCamClose) btnCamClose.addEventListener('click', toggleCamView);

  // Manual Render button
  const btnRender = document.getElementById('btn-render');
  if (btnRender) btnRender.addEventListener('click', parseAndRenderManualJson);

  // Manual Clear button
  const btnClear = document.getElementById('btn-clear');
  if (btnClear) {
    btnClear.addEventListener('click', () => {
      const jsonInput = document.getElementById('json-input');
      if (jsonInput) jsonInput.value = '';
      setJsonStatus('Ready', 'idle');
    });
  }

  // Ctrl+Enter to render in textarea
  const jsonInput = document.getElementById('json-input');
  if (jsonInput) {
    jsonInput.addEventListener('keydown', (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') parseAndRenderManualJson();
    });
  }

  // Sample Preset Buttons
  document.querySelectorAll('.btn-sample').forEach((btn) => {
    btn.addEventListener('click', () => {
      const key = btn.dataset.sample;
      const json = SAMPLES[key];
      if (json) {
        const input = document.getElementById('json-input');
        if (input) input.value = json;

        // Auto render the sample
        let wm;
        try {
          wm = JSON.parse(json);
          buildSceneFromWorldModel(wm);
          setJsonStatus(`Preset '${key}' rendered`, 'ok');
        } catch (e) {
          setJsonStatus('Error rendering preset: ' + e.message, 'error');
        }
      }
    });
  });

  // Placeholder sample button
  const btnPhSample = document.getElementById('btn-placeholder-sample');
  if (btnPhSample) {
    btnPhSample.addEventListener('click', () => {
      const json = SAMPLES.adas_backend;
      const input = document.getElementById('json-input');
      if (input) input.value = json;
      try {
        buildSceneFromWorldModel(JSON.parse(json));
      } catch (e) {}
    });
  }

  // Camera Presets
  document.querySelectorAll('.btn-cam').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.btn-cam').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      setCameraPreset(btn.dataset.cam);
    });
  });
}

// ─────────────────────────────────────────────
// Stream Client Initialization
// ─────────────────────────────────────────────

function initStreamClient() {
  const urlInput = document.getElementById('ws-url-input');
  const initialUrl = urlInput ? (urlInput.value.trim() || 'ws://localhost:9002') : 'ws://localhost:9002';

  const rateDisplay = document.getElementById('rate-display');
  const latencyDisplay = document.getElementById('latency-display');

  streamClient = new ADASStreamClient(initialUrl, {
    onData: (worldModel, stats) => {
      // Build / update 3D scene smoothly
      buildSceneFromWorldModel(worldModel);

      // Packet / Frame Rate in Hz
      if (rateDisplay && stats.fps != null) {
        rateDisplay.textContent = stats.fps.toFixed(1) + ' Hz';
      }

      // Latency / Frame interval calculation
      if (latencyDisplay && stats.currTs && stats.prevTs) {
        const deltaUs = stats.currTs - stats.prevTs;
        if (deltaUs > 0) {
          const deltaMs = deltaUs / 1000;
          latencyDisplay.textContent = deltaMs < 1000 ? `${deltaMs.toFixed(1)}ms` : `${(deltaMs / 1000).toFixed(2)}s`;
        }
      }
    },
    onStatus: (status, details) => {
      updateConnectionStatusUI(status, details);
    },
    onError: (err) => {
      console.warn('[ADAS Stream Error]', err);
    },
  });

  // Attempt connection on startup as requested
  streamClient.connect();
}

// ─────────────────────────────────────────────
// Application Boot
// ─────────────────────────────────────────────

window.addEventListener('DOMContentLoaded', () => {
  initThree();
  setupEventListeners();

  // Populate JSON textarea with default ADAS backend sample
  const jsonInput = document.getElementById('json-input');
  if (jsonInput) {
    jsonInput.value = SAMPLES.adas_backend;
  }

  // Start WebSocket client connection
  initStreamClient();
});
