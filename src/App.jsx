import { useState, useEffect, useRef, useCallback } from "react";

// ─── PHYSICS CONSTANTS ────────────────────────────────────────────────────────
const G = 6.674e-11;
const M_EARTH = 5.972e24;
const R_EARTH = 6.371e6;
const ATM_SCALE_HEIGHT = 8500;
const RHO0 = 1.225;
const AU = 1.496e11;

// ─── RK4 INTEGRATOR ──────────────────────────────────────────────────────────
function rk4Step(state, dt, forces) {
  const k1 = forces(state);
  const k2 = forces(state.map((s, i) => s + k1[i] * (dt / 2)));
  const k3 = forces(state.map((s, i) => s + k2[i] * (dt / 2)));
  const k4 = forces(state.map((s, i) => s + k3[i] * dt));
  return state.map((s, i) => s + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

function computeForces(state, params) {
  const [x, y, vx, vy] = state;
  const { mass, dragCoeff, crossSection, enableDrag, enableGravity } = params;

  const r = Math.sqrt(x * x + y * y);
  const ax_grav = enableGravity ? -(G * M_EARTH * x) / (r * r * r) : 0;
  const ay_grav = enableGravity ? -(G * M_EARTH * y) / (r * r * r) : 0;

  let ax_drag = 0, ay_drag = 0;
  if (enableDrag) {
    const altitude = r - R_EARTH;
    const rho = altitude > 0 ? RHO0 * Math.exp(-altitude / ATM_SCALE_HEIGHT) : RHO0;
    const v2 = vx * vx + vy * vy;
    const v = Math.sqrt(v2);
    const F_drag = 0.5 * rho * dragCoeff * crossSection * v2;
    ax_drag = v > 0 ? -(F_drag / mass) * (vx / v) : 0;
    ay_drag = v > 0 ? -(F_drag / mass) * (vy / v) : 0;
  }

  return [vx, vy, ax_grav + ax_drag, ay_grav + ay_drag];
}

// ─── TRAJECTORY SIMULATION ───────────────────────────────────────────────────
function simulateTrajectory(params) {
  const {
    altitude,      // m above surface
    speed,         // m/s
    angle,         // degrees from horizontal
    mass,
    dragCoeff,
    crossSection,
    enableDrag,
    enableGravity,
    maxSteps = 8000,
    dt = 20,
  } = params;

  const r0 = R_EARTH + altitude;
  const angleRad = (angle * Math.PI) / 180;

  // Start at top of Earth (north pole for clarity), tangential launch
  const launchAngle = Math.PI / 2; // launch from "top"
  const x0 = r0 * Math.cos(launchAngle);
  const y0 = r0 * Math.sin(launchAngle);

  // Velocity: angle is from local horizontal (tangential)
  const tangent = [-Math.sin(launchAngle), Math.cos(launchAngle)];
  const radial = [Math.cos(launchAngle), Math.sin(launchAngle)];
  const vx0 = speed * (Math.cos(angleRad) * tangent[0] + Math.sin(angleRad) * radial[0]);
  const vy0 = speed * (Math.cos(angleRad) * tangent[1] + Math.sin(angleRad) * radial[1]);

  let state = [x0, y0, vx0, vy0];
  const path = [{ x: x0, y: y0, t: 0, alt: altitude, v: speed }];

  const forcesFn = (s) => computeForces(s, { mass, dragCoeff, crossSection, enableDrag, enableGravity });

  for (let i = 0; i < maxSteps; i++) {
    state = rk4Step(state, dt, forcesFn);
    const [x, y, vx, vy] = state;
    const r = Math.sqrt(x * x + y * y);
    const alt = r - R_EARTH;
    const v = Math.sqrt(vx * vx + vy * vy);
    const t = (i + 1) * dt;

    path.push({ x, y, t, alt, v });

    // Stop if hit the ground or escaped (3x Earth radius)
    if (alt < 0 || r > 3 * R_EARTH) break;
  }

  return path;
}

// ─── ORBIT TYPE CLASSIFIER ──────────────────────────────────────────────────
function classifyOrbit(speed, altitude) {
  const r = R_EARTH + altitude;
  const v_circ = Math.sqrt(G * M_EARTH / r);
  const v_esc = v_circ * Math.sqrt(2);
  const ratio = speed / v_circ;

  if (speed < 1000) return { label: "Suborbital", color: "#f97316" };
  if (ratio < 0.9) return { label: "Decaying Orbit", color: "#ef4444" };
  if (ratio < 1.05) return { label: "Circular Orbit", color: "#22c55e" };
  if (ratio < 1.41) return { label: "Elliptical Orbit", color: "#3b82f6" };
  if (ratio < 1.6) return { label: "Escape Trajectory", color: "#a855f7" };
  return { label: "Hyperbolic Escape", color: "#ec4899" };
}

// ─── CANVAS RENDERER ─────────────────────────────────────────────────────────
function TrajectoryCanvas({ path, width, height }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!canvasRef.current || !path || path.length === 0) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;

    ctx.clearRect(0, 0, W, H);

    // Background
    const bg = ctx.createRadialGradient(W / 2, H / 2, 0, W / 2, H / 2, Math.max(W, H) / 2);
    bg.addColorStop(0, "#0a0e1a");
    bg.addColorStop(1, "#020408");
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, W, H);

    // Stars
    const rng = (n) => ((Math.sin(n * 127.1) * 43758.5453) % 1 + 1) % 1;
    for (let i = 0; i < 200; i++) {
      const sx = rng(i) * W;
      const sy = rng(i + 1000) * H;
      const sr = rng(i + 2000) * 1.5;
      ctx.beginPath();
      ctx.arc(sx, sy, sr, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(255,255,255,${0.2 + rng(i + 3000) * 0.6})`;
      ctx.fill();
    }

    // Scale: find bounding box
    const scale = 3 * R_EARTH;
    const toCanvas = (worldX, worldY) => {
      const cx = W / 2 + (worldX / scale) * (W * 0.45);
      const cy = H / 2 - (worldY / scale) * (H * 0.45);
      return [cx, cy];
    };

    // Atmosphere glow
    const earthR_px = (R_EARTH / scale) * W * 0.45;
    const atmR_px = ((R_EARTH + 100000) / scale) * W * 0.45;
    const atmGrad = ctx.createRadialGradient(W / 2, H / 2, earthR_px, W / 2, H / 2, atmR_px);
    atmGrad.addColorStop(0, "rgba(59,130,246,0.35)");
    atmGrad.addColorStop(1, "rgba(59,130,246,0)");
    ctx.beginPath();
    ctx.arc(W / 2, H / 2, atmR_px, 0, Math.PI * 2);
    ctx.fillStyle = atmGrad;
    ctx.fill();

    // Earth
    const earthGrad = ctx.createRadialGradient(W / 2 - earthR_px * 0.3, H / 2 - earthR_px * 0.3, earthR_px * 0.1, W / 2, H / 2, earthR_px);
    earthGrad.addColorStop(0, "#2a6496");
    earthGrad.addColorStop(0.4, "#1a4a6e");
    earthGrad.addColorStop(1, "#0d2b40");
    ctx.beginPath();
    ctx.arc(W / 2, H / 2, earthR_px, 0, Math.PI * 2);
    ctx.fillStyle = earthGrad;
    ctx.fill();

    // Earth continents hint
    ctx.beginPath();
    ctx.arc(W / 2, H / 2, earthR_px, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(34,197,94,0.15)";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Orbit reference circle (circular orbit for current alt)
    if (path.length > 0) {
      const altStart = path[0].alt;
      const orbR = R_EARTH + altStart;
      const orbR_px = (orbR / scale) * W * 0.45;
      ctx.beginPath();
      ctx.arc(W / 2, H / 2, orbR_px, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(255,255,255,0.07)";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 8]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Trajectory
    if (path.length > 1) {
      const maxV = Math.max(...path.map((p) => p.v));
      ctx.lineWidth = 2.5;
      for (let i = 1; i < path.length; i++) {
        const [x0, y0] = toCanvas(path[i - 1].x, path[i - 1].y);
        const [x1, y1] = toCanvas(path[i].x, path[i].y);
        const t = i / path.length;
        const speed_t = path[i].v / maxV;
        const r = Math.round(255 * speed_t);
        const b = Math.round(255 * (1 - speed_t));
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.strokeStyle = `rgba(${r},120,${b},${0.5 + t * 0.5})`;
        ctx.stroke();
      }

      // Launch point
      const [lx, ly] = toCanvas(path[0].x, path[0].y);
      ctx.beginPath();
      ctx.arc(lx, ly, 5, 0, Math.PI * 2);
      ctx.fillStyle = "#22c55e";
      ctx.fill();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // End point
      const last = path[path.length - 1];
      const [ex, ey] = toCanvas(last.x, last.y);
      ctx.beginPath();
      ctx.arc(ex, ey, 5, 0, Math.PI * 2);
      ctx.fillStyle = last.alt < 100 ? "#ef4444" : "#f97316";
      ctx.fill();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Labels
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.font = "11px 'JetBrains Mono', monospace";
    ctx.fillText("● Launch", 12, H - 28);
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.fillText("● Impact / End", 12, H - 14);

  }, [path, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ width: "100%", height: "100%", borderRadius: "4px" }}
    />
  );
}

// ─── ALTITUDE GRAPH ──────────────────────────────────────────────────────────
function AltitudeGraph({ path, width, height }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!canvasRef.current || !path || path.length < 2) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;
    const PAD = { top: 20, right: 20, bottom: 36, left: 60 };

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, W, H);

    const maxAlt = Math.max(...path.map((p) => p.alt), 1);
    const maxT = path[path.length - 1].t;

    const toX = (t) => PAD.left + ((t / maxT) * (W - PAD.left - PAD.right));
    const toY = (alt) => PAD.top + ((1 - alt / maxAlt) * (H - PAD.top - PAD.bottom));

    // Grid
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = PAD.top + (i / 4) * (H - PAD.top - PAD.bottom);
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(W - PAD.right, y);
      ctx.stroke();
      const label = ((maxAlt * (1 - i / 4)) / 1000).toFixed(0);
      ctx.fillStyle = "rgba(255,255,255,0.35)";
      ctx.font = "10px monospace";
      ctx.textAlign = "right";
      ctx.fillText(`${label}km`, PAD.left - 6, y + 4);
    }

    // Axis labels
    ctx.fillStyle = "rgba(255,255,255,0.45)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    for (let i = 0; i <= 4; i++) {
      const t = (maxT * i) / 4;
      const x = toX(t);
      ctx.fillText(`${(t / 60).toFixed(1)}m`, x, H - PAD.bottom + 14);
    }

    ctx.fillStyle = "rgba(255,255,255,0.4)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Time (minutes)", W / 2, H - 4);

    ctx.save();
    ctx.translate(14, H / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("Altitude (km)", 0, 0);
    ctx.restore();

    // Fill under curve
    const grad = ctx.createLinearGradient(0, PAD.top, 0, H - PAD.bottom);
    grad.addColorStop(0, "rgba(59,130,246,0.4)");
    grad.addColorStop(1, "rgba(59,130,246,0.02)");

    ctx.beginPath();
    ctx.moveTo(toX(path[0].t), toY(0));
    path.forEach((p) => ctx.lineTo(toX(p.t), toY(Math.max(0, p.alt))));
    ctx.lineTo(toX(path[path.length - 1].t), toY(0));
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    // Line
    ctx.beginPath();
    path.forEach((p, i) => {
      const x = toX(p.t);
      const y = toY(Math.max(0, p.alt));
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = "#3b82f6";
    ctx.lineWidth = 2;
    ctx.stroke();

  }, [path, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ width: "100%", height: "100%", borderRadius: "4px" }}
    />
  );
}

// ─── VELOCITY GRAPH ──────────────────────────────────────────────────────────
function VelocityGraph({ path, width, height }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!canvasRef.current || !path || path.length < 2) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;
    const PAD = { top: 20, right: 20, bottom: 36, left: 70 };

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, W, H);

    const maxV = Math.max(...path.map((p) => p.v));
    const maxT = path[path.length - 1].t;

    const toX = (t) => PAD.left + ((t / maxT) * (W - PAD.left - PAD.right));
    const toY = (v) => PAD.top + ((1 - v / maxV) * (H - PAD.top - PAD.bottom));

    // Grid
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = PAD.top + (i / 4) * (H - PAD.top - PAD.bottom);
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(W - PAD.right, y);
      ctx.stroke();
      const label = ((maxV * (1 - i / 4)) / 1000).toFixed(1);
      ctx.fillStyle = "rgba(255,255,255,0.35)";
      ctx.font = "10px monospace";
      ctx.textAlign = "right";
      ctx.fillText(`${label}km/s`, PAD.left - 6, y + 4);
    }

    ctx.fillStyle = "rgba(255,255,255,0.45)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    for (let i = 0; i <= 4; i++) {
      const t = (maxT * i) / 4;
      const x = toX(t);
      ctx.fillText(`${(t / 60).toFixed(1)}m`, x, H - PAD.bottom + 14);
    }

    ctx.fillStyle = "rgba(255,255,255,0.4)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.fillText("Time (minutes)", W / 2, H - 4);

    ctx.save();
    ctx.translate(14, H / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("Speed (km/s)", 0, 0);
    ctx.restore();

    // Fill
    const grad = ctx.createLinearGradient(0, PAD.top, 0, H - PAD.bottom);
    grad.addColorStop(0, "rgba(168,85,247,0.4)");
    grad.addColorStop(1, "rgba(168,85,247,0.02)");

    ctx.beginPath();
    ctx.moveTo(toX(path[0].t), toY(0));
    path.forEach((p) => ctx.lineTo(toX(p.t), toY(p.v)));
    ctx.lineTo(toX(path[path.length - 1].t), toY(0));
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();

    // Line
    ctx.beginPath();
    path.forEach((p, i) => {
      i === 0 ? ctx.moveTo(toX(p.t), toY(p.v)) : ctx.lineTo(toX(p.t), toY(p.v));
    });
    ctx.strokeStyle = "#a855f7";
    ctx.lineWidth = 2;
    ctx.stroke();

  }, [path, width, height]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      style={{ width: "100%", height: "100%", borderRadius: "4px" }}
    />
  );
}

// ─── PRESETS ─────────────────────────────────────────────────────────────────
const PRESETS = [
  { label: "LEO Circular", altitude: 400000, speed: 7670, angle: 0, mass: 5000, dragCoeff: 2.2, crossSection: 10, enableDrag: true, enableGravity: true },
  { label: "GTO Transfer", altitude: 200000, speed: 10200, angle: 0, mass: 5000, dragCoeff: 2.2, crossSection: 10, enableDrag: false, enableGravity: true },
  { label: "Escape Velocity", altitude: 400000, speed: 11200, angle: 0, mass: 5000, dragCoeff: 2.2, crossSection: 10, enableDrag: false, enableGravity: true },
  { label: "Ballistic Missile", altitude: 100000, speed: 6000, angle: 45, mass: 10000, dragCoeff: 0.5, crossSection: 2, enableDrag: true, enableGravity: true },
  { label: "Suborbital Hop", altitude: 50000, speed: 2000, angle: 30, mass: 2000, dragCoeff: 1.0, crossSection: 4, enableDrag: true, enableGravity: true },
];

// ─── SLIDER ──────────────────────────────────────────────────────────────────
function Slider({ label, value, min, max, step, onChange, format, unit }) {
  return (
    <div style={{ marginBottom: "14px" }}>
      <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "5px" }}>
        <span style={{ fontSize: "11px", color: "#8b949e", letterSpacing: "0.05em", textTransform: "uppercase" }}>{label}</span>
        <span style={{ fontSize: "12px", color: "#e2e8f0", fontFamily: "monospace" }}>
          {format ? format(value) : value} {unit}
        </span>
      </div>
      <div style={{ position: "relative" }}>
        <input
          type="range"
          min={min}
          max={max}
          step={step}
          value={value}
          onChange={(e) => onChange(Number(e.target.value))}
          style={{
            width: "100%",
            height: "4px",
            appearance: "none",
            background: `linear-gradient(to right, #3b82f6 0%, #3b82f6 ${((value - min) / (max - min)) * 100}%, #1f2937 ${((value - min) / (max - min)) * 100}%, #1f2937 100%)`,
            borderRadius: "2px",
            outline: "none",
            cursor: "pointer",
          }}
        />
      </div>
    </div>
  );
}

// ─── MAIN APP ────────────────────────────────────────────────────────────────
export default function App() {
  const [params, setParams] = useState(PRESETS[0]);
  const [path, setPath] = useState([]);
  const [isRunning, setIsRunning] = useState(false);
  const [activeTab, setActiveTab] = useState("trajectory");
  const [showReadme, setShowReadme] = useState(false);

  const set = (key) => (val) => setParams((p) => ({ ...p, [key]: val }));
  const toggle = (key) => () => setParams((p) => ({ ...p, [key]: !p[key] }));

  const runSim = useCallback(() => {
    setIsRunning(true);
    setTimeout(() => {
      const result = simulateTrajectory({ ...params, maxSteps: 10000, dt: 20 });
      setPath(result);
      setIsRunning(false);
    }, 50);
  }, [params]);

  // Run simulation once on mount to populate initial trajectory.
  // runSim is intentionally excluded — adding it would re-run on every
  // param change, but we want the user to trigger runs explicitly via
  // the Launch button (except for the very first render).
  // eslint-disable-next-line react-hooks/exhaustive-deps
  useEffect(() => { runSim(); }, []);

  const orbitType = classifyOrbit(params.speed, params.altitude);
  const last = path[path.length - 1];
  const duration = last ? (last.t / 60).toFixed(1) : "—";
  const maxAlt = path.length ? (Math.max(...path.map((p) => p.alt)) / 1000).toFixed(1) : "—";
  const minSpeed = path.length ? (Math.min(...path.map((p) => p.v)) / 1000).toFixed(2) : "—";
  const v_circ = Math.sqrt(G * M_EARTH / (R_EARTH + params.altitude));
  const v_esc = v_circ * Math.sqrt(2);

  const statStyle = {
    background: "#0d1117",
    border: "1px solid #21262d",
    borderRadius: "6px",
    padding: "10px 14px",
    flex: "1",
    minWidth: "80px",
  };

  return (
    <div style={{
      minHeight: "100vh",
      background: "#010409",
      color: "#e2e8f0",
      fontFamily: "'JetBrains Mono', 'Fira Code', 'Courier New', monospace",
      display: "flex",
      flexDirection: "column",
    }}>
      {/* Header */}
      <div style={{
        borderBottom: "1px solid #21262d",
        padding: "16px 24px",
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        background: "rgba(13,17,23,0.95)",
        backdropFilter: "blur(10px)",
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: "12px" }}>
          <div style={{
            width: "32px", height: "32px",
            background: "linear-gradient(135deg, #1d4ed8, #7c3aed)",
            borderRadius: "8px",
            display: "flex", alignItems: "center", justifyContent: "center",
            fontSize: "16px",
          }}>🛰</div>
          <div>
            <div style={{ fontSize: "14px", fontWeight: "700", letterSpacing: "0.02em", color: "#f0f6fc" }}>
              ORBITAL TRAJECTORY SIMULATOR
            </div>
            <div style={{ fontSize: "10px", color: "#6e7681", letterSpacing: "0.1em" }}>
              RK4 · NEWTONIAN GRAVITY · ATMOSPHERIC DRAG
            </div>
          </div>
        </div>
        <div style={{ display: "flex", gap: "8px", alignItems: "center" }}>
          <div style={{
            padding: "4px 12px",
            borderRadius: "20px",
            fontSize: "11px",
            fontWeight: "700",
            background: orbitType.color + "22",
            border: `1px solid ${orbitType.color}44`,
            color: orbitType.color,
            letterSpacing: "0.05em",
          }}>
            {orbitType.label.toUpperCase()}
          </div>
          <button
            onClick={() => setShowReadme(!showReadme)}
            style={{
              padding: "4px 12px",
              borderRadius: "6px",
              fontSize: "11px",
              background: showReadme ? "#21262d" : "transparent",
              border: "1px solid #30363d",
              color: "#8b949e",
              cursor: "pointer",
              letterSpacing: "0.05em",
            }}
          >
            {showReadme ? "◀ SIM" : "README"}
          </button>
        </div>
      </div>

      <div style={{ display: "flex", flex: 1, overflow: "hidden" }}>
        {/* Controls Panel */}
        <div style={{
          width: "280px",
          minWidth: "280px",
          borderRight: "1px solid #21262d",
          padding: "20px 16px",
          overflowY: "auto",
          background: "#0d1117",
        }}>

          {/* Presets */}
          <div style={{ marginBottom: "20px" }}>
            <div style={{ fontSize: "10px", color: "#6e7681", letterSpacing: "0.1em", marginBottom: "8px" }}>MISSION PRESETS</div>
            <div style={{ display: "flex", flexDirection: "column", gap: "4px" }}>
              {PRESETS.map((p) => (
                <button
                  key={p.label}
                  onClick={() => { setParams(p); }}
                  style={{
                    padding: "6px 10px",
                    borderRadius: "5px",
                    fontSize: "11px",
                    textAlign: "left",
                    background: params.label === p.label ? "#161b22" : "transparent",
                    border: `1px solid ${params.label === p.label ? "#30363d" : "transparent"}`,
                    color: params.label === p.label ? "#58a6ff" : "#8b949e",
                    cursor: "pointer",
                    letterSpacing: "0.02em",
                  }}
                >
                  {p.label}
                </button>
              ))}
            </div>
          </div>

          <div style={{ height: "1px", background: "#21262d", margin: "0 -16px 20px" }} />

          {/* Parameters */}
          <div style={{ fontSize: "10px", color: "#6e7681", letterSpacing: "0.1em", marginBottom: "14px" }}>PARAMETERS</div>

          <Slider label="Altitude" value={params.altitude} min={100000} max={2000000} step={10000}
            onChange={set("altitude")} format={(v) => (v / 1000).toFixed(0)} unit="km" />
          <Slider label="Speed" value={params.speed} min={500} max={15000} step={50}
            onChange={set("speed")} format={(v) => (v / 1000).toFixed(2)} unit="km/s" />
          <Slider label="Launch Angle" value={params.angle} min={-30} max={30} step={1}
            onChange={set("angle")} unit="°" />
          <Slider label="Mass" value={params.mass} min={100} max={50000} step={100}
            onChange={set("mass")} format={(v) => v.toLocaleString()} unit="kg" />

          <div style={{ height: "1px", background: "#21262d", margin: "4px -16px 20px" }} />
          <div style={{ fontSize: "10px", color: "#6e7681", letterSpacing: "0.1em", marginBottom: "14px" }}>DRAG CONFIG</div>

          <div style={{ display: "flex", justifyContent: "space-between", marginBottom: "14px" }}>
            {[
              { key: "enableGravity", label: "Gravity" },
              { key: "enableDrag", label: "Drag" },
            ].map(({ key, label }) => (
              <button
                key={key}
                onClick={toggle(key)}
                style={{
                  padding: "6px 14px",
                  borderRadius: "5px",
                  fontSize: "11px",
                  background: params[key] ? "#1d4ed822" : "transparent",
                  border: `1px solid ${params[key] ? "#3b82f6" : "#30363d"}`,
                  color: params[key] ? "#60a5fa" : "#6e7681",
                  cursor: "pointer",
                }}
              >
                {params[key] ? "✓" : "○"} {label}
              </button>
            ))}
          </div>

          {params.enableDrag && (
            <>
              <Slider label="Drag Coeff (Cd)" value={params.dragCoeff} min={0.1} max={5} step={0.1}
                onChange={set("dragCoeff")} />
              <Slider label="Cross Section" value={params.crossSection} min={1} max={100} step={1}
                onChange={set("crossSection")} unit="m²" />
            </>
          )}

          <div style={{ height: "1px", background: "#21262d", margin: "4px -16px 20px" }} />

          {/* Reference velocities */}
          <div style={{ fontSize: "10px", color: "#6e7681", letterSpacing: "0.1em", marginBottom: "10px" }}>REFERENCE VELOCITIES</div>
          {[
            { label: "Circular", v: v_circ, color: "#22c55e" },
            { label: "Escape", v: v_esc, color: "#a855f7" },
          ].map(({ label, v, color }) => (
            <div key={label} style={{ display: "flex", justifyContent: "space-between", marginBottom: "6px", fontSize: "11px" }}>
              <span style={{ color: "#6e7681" }}>{label} orbit</span>
              <span style={{ color }}>{(v / 1000).toFixed(2)} km/s</span>
            </div>
          ))}

          <div style={{ height: "1px", background: "#21262d", margin: "14px -16px 20px" }} />

          {/* Run */}
          <button
            onClick={runSim}
            disabled={isRunning}
            style={{
              width: "100%",
              padding: "10px",
              borderRadius: "6px",
              fontSize: "12px",
              fontWeight: "700",
              letterSpacing: "0.08em",
              background: isRunning ? "#1f2937" : "linear-gradient(135deg, #1d4ed8, #7c3aed)",
              border: "none",
              color: isRunning ? "#6e7681" : "#fff",
              cursor: isRunning ? "not-allowed" : "pointer",
              fontFamily: "inherit",
              transition: "all 0.2s",
            }}
          >
            {isRunning ? "COMPUTING…" : "▶  LAUNCH SIMULATION"}
          </button>
        </div>

        {/* Main Content */}
        <div style={{ flex: 1, display: "flex", flexDirection: "column", overflow: "hidden" }}>

          {showReadme ? (
            <div style={{
              flex: 1, overflowY: "auto", padding: "32px 40px",
              lineHeight: "1.75", fontSize: "13px", color: "#c9d1d9",
            }}>
              <div style={{ maxWidth: "720px", margin: "0 auto" }}>
                <h1 style={{ fontSize: "22px", color: "#f0f6fc", marginBottom: "8px", fontWeight: "700" }}>
                  Orbital Trajectory Simulator — Physics Reference
                </h1>
                <div style={{ color: "#6e7681", fontSize: "11px", letterSpacing: "0.08em", marginBottom: "32px" }}>
                  NUMERICAL INTEGRATION · CLASSICAL MECHANICS · ATMOSPHERIC MODEL
                </div>

                {[
                  {
                    title: "Equations of Motion",
                    body: `The simulation solves Newton's second law in 2D Cartesian coordinates:\n\n  ẍ = Fₓ/m,  ÿ = Fᵧ/m\n\nwhere the net force has two components:\n\n  F = F_gravity + F_drag`,
                  },
                  {
                    title: "Gravitational Force",
                    body: `Point-mass Newtonian gravity:\n\n  F_grav = −(G·M_⊕·m / r³) · r⃗\n\nwhere:\n  G = 6.674×10⁻¹¹ m³ kg⁻¹ s⁻²\n  M_⊕ = 5.972×10²⁴ kg (Earth mass)\n  r = |r⃗| = distance from Earth's centre`,
                  },
                  {
                    title: "Atmospheric Drag",
                    body: `Drag force using exponential atmosphere:\n\n  F_drag = ½ · ρ(h) · Cd · A · v²\n\nwhere atmospheric density decays with altitude:\n\n  ρ(h) = ρ₀ · exp(−h / H)\n\n  ρ₀ = 1.225 kg/m³ (sea-level density)\n  H = 8500 m (scale height)\n  Cd = drag coefficient (0.1–5.0)\n  A = cross-sectional area (m²)`,
                  },
                  {
                    title: "Numerical Integration — RK4",
                    body: `The 4th-order Runge-Kutta method is used. Given state vector [x, y, vx, vy]:\n\n  k₁ = f(yₙ)\n  k₂ = f(yₙ + dt·k₁/2)\n  k₃ = f(yₙ + dt·k₂/2)\n  k₄ = f(yₙ + dt·k₃)\n\n  yₙ₊₁ = yₙ + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)\n\nThis gives O(dt⁴) local error — far superior to Euler's O(dt) — crucial for orbital mechanics where energy conservation matters. Timestep dt = 20 s.`,
                  },
                  {
                    title: "Orbital Velocity Reference",
                    body: `For circular orbit at altitude h:\n\n  v_circ = √(G·M / (R_⊕ + h))\n\nFor escape from altitude h:\n\n  v_esc = v_circ · √2\n\nExample at h = 400 km:\n  v_circ ≈ 7.67 km/s\n  v_esc  ≈ 10.85 km/s`,
                  },
                  {
                    title: "Orbit Classification",
                    body: `Speed relative to circular velocity determines orbit type:\n\n  v < ~1 km/s      → Suborbital (ballistic)\n  v/v_circ < 0.9   → Decaying orbit (re-entry)\n  v/v_circ ≈ 1.0   → Circular orbit\n  1 < v/v_circ < √2 → Elliptical orbit\n  v/v_circ ≥ √2    → Escape / hyperbolic trajectory`,
                  },
                  {
                    title: "Limitations & Assumptions",
                    body: `• 2D simulation only (planar trajectory)\n• Spherical, non-rotating Earth\n• Point-mass gravitational model (no J₂ oblateness)\n• Simplified 1D exponential atmosphere\n• No thrust forces (coast/ballistic phase only)\n• No Moon/Sun third-body perturbations`,
                  },
                ].map(({ title, body }) => (
                  <div key={title} style={{ marginBottom: "28px" }}>
                    <h3 style={{ fontSize: "13px", color: "#58a6ff", marginBottom: "10px", letterSpacing: "0.05em" }}>
                      ▸ {title}
                    </h3>
                    <pre style={{
                      fontFamily: "inherit",
                      whiteSpace: "pre-wrap",
                      background: "#0d1117",
                      border: "1px solid #21262d",
                      borderRadius: "6px",
                      padding: "14px 18px",
                      fontSize: "12px",
                      color: "#c9d1d9",
                      lineHeight: "1.8",
                      margin: 0,
                    }}>{body}</pre>
                  </div>
                ))}
              </div>
            </div>
          ) : (
            <>
              {/* Stats Bar */}
              <div style={{
                padding: "12px 20px",
                borderBottom: "1px solid #21262d",
                display: "flex",
                gap: "10px",
                flexWrap: "wrap",
              }}>
                {[
                  { label: "DURATION", value: `${duration} min` },
                  { label: "MAX ALT", value: `${maxAlt} km` },
                  { label: "MIN SPEED", value: `${minSpeed} km/s` },
                  { label: "DATA PTS", value: path.length.toLocaleString() },
                  { label: "DT", value: "20 s (RK4)" },
                ].map(({ label, value }) => (
                  <div key={label} style={statStyle}>
                    <div style={{ fontSize: "9px", color: "#6e7681", letterSpacing: "0.1em", marginBottom: "3px" }}>{label}</div>
                    <div style={{ fontSize: "13px", color: "#f0f6fc" }}>{value}</div>
                  </div>
                ))}
              </div>

              {/* Tabs */}
              <div style={{
                display: "flex",
                borderBottom: "1px solid #21262d",
                padding: "0 20px",
                gap: "2px",
              }}>
                {["trajectory", "altitude", "velocity"].map((tab) => (
                  <button
                    key={tab}
                    onClick={() => setActiveTab(tab)}
                    style={{
                      padding: "10px 16px",
                      fontSize: "11px",
                      letterSpacing: "0.08em",
                      background: "transparent",
                      border: "none",
                      borderBottom: `2px solid ${activeTab === tab ? "#3b82f6" : "transparent"}`,
                      color: activeTab === tab ? "#60a5fa" : "#6e7681",
                      cursor: "pointer",
                      fontFamily: "inherit",
                      textTransform: "uppercase",
                    }}
                  >
                    {tab === "trajectory" ? "🛰 Trajectory" : tab === "altitude" ? "📈 Altitude" : "⚡ Velocity"}
                  </button>
                ))}
              </div>

              {/* Chart */}
              <div style={{ flex: 1, padding: "16px 20px", overflow: "hidden" }}>
                <div style={{ width: "100%", height: "100%" }}>
                  {activeTab === "trajectory" && (
                    <TrajectoryCanvas path={path} width={900} height={600} />
                  )}
                  {activeTab === "altitude" && (
                    <AltitudeGraph path={path} width={900} height={500} />
                  )}
                  {activeTab === "velocity" && (
                    <VelocityGraph path={path} width={900} height={500} />
                  )}
                </div>
              </div>
            </>
          )}
        </div>
      </div>

      {/* Footer */}
      <div style={{
        padding: "8px 24px",
        borderTop: "1px solid #21262d",
        fontSize: "10px",
        color: "#484f58",
        display: "flex",
        justifyContent: "space-between",
        letterSpacing: "0.05em",
      }}>
        <span>RK4 INTEGRATOR · dt=20s · EXPONENTIAL ATMOSPHERE · NEWTONIAN GRAVITY</span>
        <span>G={G} m³kg⁻¹s⁻² · M⊕={M_EARTH.toExponential(3)} kg · R⊕={R_EARTH.toExponential(3)} m</span>
      </div>

      <style>{`
        input[type=range]::-webkit-slider-thumb {
          -webkit-appearance: none;
          width: 14px;
          height: 14px;
          border-radius: 50%;
          background: #3b82f6;
          cursor: pointer;
          border: 2px solid #0d1117;
        }
        input[type=range]::-moz-range-thumb {
          width: 14px;
          height: 14px;
          border-radius: 50%;
          background: #3b82f6;
          cursor: pointer;
          border: 2px solid #0d1117;
        }
        ::-webkit-scrollbar { width: 6px; background: transparent; }
        ::-webkit-scrollbar-thumb { background: #21262d; border-radius: 3px; }
      `}</style>
    </div>
  );
}
