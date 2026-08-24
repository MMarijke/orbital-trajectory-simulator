# 🛰 Orbital Trajectory Simulator

A physics-based orbital and rocket trajectory simulator built with React. Uses **4th-order Runge-Kutta (RK4)** numerical integration to solve the equations of motion under Newtonian gravity and atmospheric drag.

![Orbital Trajectory Simulator](https://img.shields.io/badge/physics-RK4%20integration-blue)
![React](https://img.shields.io/badge/React-18-61DAFB?logo=react)
![Vite](https://img.shields.io/badge/Vite-5-646CFF?logo=vite)
![License](https://img.shields.io/badge/license-MIT-green)
![Deploy](https://github.com/YOUR_USERNAME/orbital-trajectory-simulator/actions/workflows/deploy.yml/badge.svg)

> **Live demo:** https://YOUR_USERNAME.github.io/orbital-trajectory-simulator/

---

## Features

- **RK4 numerical integration** — 4th-order Runge-Kutta with dt = 20 s timestep
- **Newtonian gravity** — point-mass gravitational model for Earth
- **Atmospheric drag** — exponential atmosphere model (scale height H = 8500 m)
- **5 mission presets** — LEO circular, GTO transfer, escape trajectory, ballistic, suborbital
- **Adjustable parameters** — altitude, speed, launch angle, mass, drag coefficient, cross-section
- **Live orbit classifier** — circular, elliptical, escape, suborbital, decaying
- **3 visualisation views** — 2D trajectory map, altitude vs time, velocity vs time
- **Built-in physics reference** — equations, derivations, and model assumptions

---

## Physics Model

### Equations of Motion

The simulator solves Newton's second law in 2D Cartesian coordinates:

```
ẍ = Fₓ / m
ÿ = Fᵧ / m
```

State vector: `[x, y, vx, vy]`

### Gravitational Force

```
F_grav = −(G · M_⊕ · m / r³) · r⃗

G     = 6.674 × 10⁻¹¹  m³ kg⁻¹ s⁻²
M_⊕   = 5.972 × 10²⁴  kg
R_⊕   = 6.371 × 10⁶   m
r     = √(x² + y²)     (distance from Earth centre)
```

### Atmospheric Drag

```
F_drag = ½ · ρ(h) · Cd · A · v²

ρ(h) = ρ₀ · exp(−h / H)

ρ₀  = 1.225 kg/m³   (sea-level density)
H   = 8500 m         (scale height)
Cd  = drag coefficient (configurable)
A   = cross-sectional area (configurable)
```

### RK4 Integrator

```
k₁ = f(yₙ)
k₂ = f(yₙ + dt · k₁/2)
k₃ = f(yₙ + dt · k₂/2)
k₄ = f(yₙ + dt · k₃)

yₙ₊₁ = yₙ + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)
```

RK4 is chosen over Euler integration for its **O(dt⁴) local truncation error**, which is essential for orbital mechanics where energy conservation over many orbital periods matters.

### Orbital Velocity Reference

```
v_circular = √(G · M_⊕ / (R_⊕ + h))
v_escape   = v_circular · √2
```

At h = 400 km (ISS altitude):
- v_circ ≈ **7.67 km/s**
- v_esc  ≈ **10.85 km/s**

---

## Getting Started

### Prerequisites

- Node.js ≥ 18
- npm ≥ 9

### Installation

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/orbital-trajectory-simulator.git
cd orbital-trajectory-simulator

# Install dependencies
npm install

# Start development server
npm run dev
```

Open [http://localhost:5173](http://localhost:5173) in your browser.

### Build for Production

```bash
npm run build
```

Output is written to `dist/`.

### Preview Production Build

```bash
npm run preview
```

---

## Deployment

This project deploys automatically to **GitHub Pages** on every push to `main`.

### Setup (one-time)

1. Push the repo to GitHub
2. Go to **Settings → Pages**
3. Set **Source** to `GitHub Actions`
4. Update `vite.config.js` — change `base` to match your repo name:
   ```js
   base: "/your-repo-name/",
   ```
5. Push to `main` — the workflow in `.github/workflows/deploy.yml` handles the rest

Your app will be live at: `https://YOUR_USERNAME.github.io/your-repo-name/`

---

## Project Structure

```
orbital-trajectory-simulator/
├── .github/
│   └── workflows/
│       └── deploy.yml        # GitHub Actions → GitHub Pages
├── public/
│   └── favicon.svg
├── src/
│   ├── App.jsx               # Main simulator component + all physics
│   ├── main.jsx              # React entry point
│   └── index.css             # Global styles + resets
├── index.html                # HTML shell with font imports
├── vite.config.js            # Vite + base path config
├── package.json
├── .gitignore
└── README.md
```

---

## Mission Presets

| Preset | Altitude | Speed | Notes |
|---|---|---|---|
| LEO Circular | 400 km | 7.67 km/s | ISS-like circular orbit |
| GTO Transfer | 200 km | 10.2 km/s | Geostationary transfer orbit |
| Escape Velocity | 400 km | 11.2 km/s | Minimum escape from Earth |
| Ballistic Missile | 100 km | 6.0 km/s | 45° launch angle |
| Suborbital Hop | 50 km | 2.0 km/s | Short-range trajectory |

---

## Known Limitations

- 2D planar simulation only (no orbital inclination)
- Spherical, non-rotating Earth (no Coriolis effect)
- No J₂ oblateness perturbation
- No thrust forces (coasting/ballistic only)
- No third-body effects (Moon, Sun)
- Simplified 1D exponential atmosphere (no wind, temperature variation)

---

## Possible Extensions

- [ ] 3D trajectory with inclination
- [ ] Thrust / burn phases (delta-v manoeuvres)
- [ ] Hohmann transfer calculator
- [ ] Multiple satellite tracking
- [ ] Earth rotation and launch azimuth
- [ ] Two-line element (TLE) import
- [ ] J₂ perturbation model

---

## License

MIT — see [LICENSE](LICENSE)

---

## Acknowledgements

Physics constants and atmospheric model from:
- US Standard Atmosphere 1976
- NASA Goddard Space Flight Center orbital mechanics references
- Bate, Mueller & White — *Fundamentals of Astrodynamics* (Dover, 1971)
