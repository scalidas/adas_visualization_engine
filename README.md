# ADAS Visualization Engine

A Tesla-style real-time 3D and 2D visualization web application for autonomous vehicle **ADAS Perception backends**.  
Streams live telemetry, lane models, and tracked 3D/2D obstacles over **WebSocket** (`ws://localhost:9002`) with automatic reconnection and interactive controls.

---

## Features

| Feature | Details |
|---|---|
| **Real-time WebSocket Client** | Connects to `ws://localhost:9002` (configurable host & port); auto-reconnect with exponential backoff |
| **Stream Telemetry HUD** | Live packet rate (Hz / FPS), frame sequence `#`, frame delta $\Delta t$, and timestamp badge |
| **Stream Controls** | One-click Connect/Disconnect, Pause/Resume stream rendering (freeze-frame inspection), Snapshot to JSON editor |
| **3D World View** | Three.js WebGL renderer with dynamic lighting, tone mapping, fog, and shadow mapping |
| **Ego Vehicle** | Stylized car model with dynamic steering angle animation responding to live telemetry |
| **Lane Geometry** | Samples 2nd-degree polynomials $y(x) = c_0 + c_1 x + c_2 x^2$ from $x=0.2\text{m}$ to $5.0\text{m}$ with blue corridor fill |
| **3D Tracked Obstacles** | Color-coded 3D wireframe boxes with corner brackets, velocity vectors, and confidence tags |
| **2D Camera Viewfinder (PiP)** | Picture-in-picture 640×480 forward camera HUD rendering 2D bounding boxes in normalized camera space |
| **Traffic Signs** | Octagonal 3D stop-sign geometry with ground posts and distance tracking |
| **Camera Presets** | Follow (Tesla-style 3rd person), Bird's-Eye-View (BEV top-down), and Side profile |
| **Dual Mode Interface** | **Live Stream** tab (telemetry gauges, connection health, formulas) & **Manual JSON** tab (editor & offline samples) |

---

## Backend Connection Details

- **Protocol**: WebSocket (`ws://` — no TLS)
- **Default Endpoint**: `ws://localhost:9002` (configurable in top bar, e.g. `ws://127.0.0.1:9002` or remote IP)
- **Persistence**: Custom WebSocket URL is automatically saved in `localStorage` across page refreshes
- **Auto-reconnect**: Exponential backoff starting at 2.0s up to 10.0s max, with real-time UI countdown
- **Status Indicator**:
  - 🟢 **CONNECTED** — Active WebSocket feed, real-time rendering
  - 🔵 **CONNECTING** — Establishing socket handshake
  - 🟡 **RECONNECTING** — Retrying with exponential backoff & countdown timer
  - 🔴 **DISCONNECTED** — Offline or manually disconnected

---

## Vehicle & Sensor Coordinate Frame

The perception engine publishes positions in the local vehicle coordinate system:
- **All distances**: Meters ($m$)
- **All velocities**: Meters per second ($m/s$)
- **+X axis**: **Forward** (longitudinal distance in front of vehicle)
- **+Y axis**: **Right** (lateral offset; positive is right of centerline, negative is left)
- **+Z axis**: **Up** (vertical height from ground level)
- **2D Bounding Boxes**: Normalized to standard 640×480 pixel camera space with `{ "x", "y", "width", "height" }`

### Three.js World Coordinate Mapping

```
Vehicle Frame         Three.js World Frame
+X (Forward)    →     -Z (forward into screen)
+Y (Right)      →     +X (screen-right)
+Z (Up)         →     +Y (screen-up)
```

Implementation formula:
```javascript
toThree(jx, jy, jz) = new THREE.Vector3(jy, jz, -jx)
```

---

## Lane Polynomial Calculation

Lane boundaries are represented as 2nd-degree polynomials:
$$y(x) = c_0 + c_1 \cdot x + c_2 \cdot x^2$$

Where:
- $x$ is the longitudinal distance forward along the path in meters ($x \ge 0$).
- $y$ is the lateral position in meters (+ right, - left).
- Sampled at discrete steps from $x = 0.2\text{m}$ to $x = 5.0\text{m}$ in $0.1\text{m}$ increments.

---

## WorldModel JSON Schema

Incoming WebSocket messages must be JSON strings matching:

```json
{
  "timestamp_us": 1729135000123456,
  "frame_seq": 1042,
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
    "left_boundary": { "c0": -0.25, "c1": 0.01, "c2": -0.002 },
    "right_boundary": { "c0": 0.25, "c1": -0.01, "c2": 0.002 },
    "centerline": { "c0": 0.0, "c1": 0.0, "c2": 0.0 }
  },
  "left_neighbor_lane": null,
  "right_neighbor_lane": null,
  "obstacles": [
    {
      "id": 0,
      "label": "bottle",
      "confidence": 0.85,
      "bounding_box": {
        "x": 240.0,
        "y": 180.0,
        "width": 80.0,
        "height": 160.0
      },
      "position": {
        "x": 1.45,
        "y": -0.12,
        "z": 0.05
      },
      "velocity": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "length": 0.10,
      "width": 0.10,
      "height": 0.26
    }
  ],
  "traffic_signs": [
    {
      "type": "STOP_SIGN",
      "position": {
        "x": 3.5,
        "y": 0.8,
        "z": 0.5
      },
      "confidence": 0.92
    }
  ]
}
```

---

## Running Locally

### Start Local Web Server:

Using Python simple HTTP server:
```bash
python -m http.server 8080
```

Then open in browser:
**`http://localhost:8080`**

### Running with ADAS Backend:

1. Launch your C++ ADAS Perception backend WebSocket publisher on port `9002`.
2. Open `http://localhost:8080`.
3. The visualization webapp automatically connects to `ws://localhost:9002` on startup.
4. If your backend is hosted on a different machine or port, update the address in the top bar (e.g. `ws://192.168.1.50:9002`) and click **Connect**.

---

## AI Assistance & Citation

This project's visualization web application (`index.html`, `style.css`, `app.js`, sample JSON datasets, and WebSocket streaming integration) was developed with the assistance of agentic AI coding tools within the **Antigravity IDE**.

### Tool & Environment

- **IDE / Environment:** [Antigravity](https://antigravity.dev) — Agentic AI coding assistant powered by Google DeepMind
- **Interaction Mode:** Agentic pair-programming (autonomous code generation, UI design, WebSocket client implementation, and interactive browser verification)

### Prompts & Development History

#### Phase 1 — Initial 3D Visualization Engine
- **Date:** September 2026
- **Model:** Claude Sonnet 4.6
- **Objective:** Create an offline 3D visualization tool for ADAS perception data using Three.js, rendering ego vehicle, polynomial cubic boundaries, bounding boxes with corner brackets, and traffic signs from structured `WorldModel` JSON.

#### Phase 2 — WebSocket Live Telemetry & ADAS Perception Backend Integration
- **Date:** October 2026
- **Model:** Gemini 3.8 Flash (High)
- **Prompt Submitted:**
  > *"Connect this visualization web application to our autonomous vehicle ADAS Perception backend via WebSocket to display real-time live telemetry, lane models, and tracked obstacles.*  
  > *Backend Connection Details: Protocol: WebSocket (no TLS / ws://), Default Address: ws://localhost:9002 (provide a UI input/config field allowing host/port configuration). Connection Lifecycle: auto-connect on startup, auto-reconnect with exponential backoff (2-3s retry), clear connection status indicator (Connected/Disconnected/Reconnecting) with latency or packet/frame rate counter in UI.*  
  > *Vehicle & Sensor Coordinate Frame: All distances in meters (m), velocities in m/s, +X forward, +Y right, +Z up. 2D Bounding Boxes normalized to 640x480 camera space.*  
  > *Lane boundaries represented as 2nd-degree polynomials: y(x) = c0 + c1*x + c2*x^2, sampled discretely from x = 0.2m to 5.0m in 0.1m increments."*
- **Deliverables Implemented:**
  - Robust `ADASStreamClient` with exponential backoff and connection state machine.
  - Vehicle coordinate frame transformation (`toThree: Three.js (-Z, +X, +Y)`).
  - 2nd-degree polynomial sampling ($x = 0.2\text{m} \to 5.0\text{m}$, step $0.1\text{m}$) and math HUD display.
  - Picture-in-picture 2D camera viewfinder overlay ($640 \times 480$ normalized camera space) with crosshairs and 2D bounding boxes.
  - Live stream telemetry dashboard with velocity, steering angle, and yaw indicators.

